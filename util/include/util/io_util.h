#ifndef _IO_UTIL_H_
#define _IO_UTIL_H_

// Mabel Zhang
// 30 Dec 2016
//
// Refactored from active_visual_tactile feature_fpfh.h
//
// File I/O utility functions
//
// See other convenience functions in
//   #include <boost/filesystem/convenience.hpp>
//

#include <stdio.h>
#include <fstream>  // std::ifstream

// For create_dir_if_nonexist() and concat_paths()
#include <boost/filesystem.hpp>
// API https://www.boost.org/doc/libs/1_33_1/libs/filesystem/doc/convenience.htm
#include <boost/filesystem/convenience.hpp>  // basename(), extension()

// For current_time_string()
#include <ctime>


// Creates a directory if it does not exist.
// Ref: http://stackoverflow.com/questions/9235679/create-a-directory-if-it-doesnt-exist
// Parameters:
//   filename: A full path to a file (with '.' in the basename) or a directory
//     (without '.' in the basename). If file, if the parent directory of this
//     file doesn't exist, then create it.
void create_dir_if_nonexist (const std::string filepath)
{
  boost::filesystem::path filepath_bst (filepath);

  // If path is a file, remove the basename, create the directory before it.
  std::string base = boost::filesystem::basename (filepath_bst);
  // If path is a file, assume there is '.' in base name
  // Not using boost::filesystem::is_regular_file(), because file most likely
  //   does not exist yet! That is why this function is being called to create
  //   its parent directory!
  if (base.find ('.') != std::string::npos)
  {
    // Extract directory name from file path
    // Ref: http://stackoverflow.com/questions/3071665/getting-a-directory-name-from-a-filename
    filepath_bst.remove_filename ();
  }
 
  boost::filesystem::path sysdir (filepath_bst.string ().c_str ());
  if (boost::filesystem::create_directories (sysdir))
  {
    printf ("Directory Created: %s\n", filepath_bst.string ().c_str ());
  }
}

// Moved from active_visual_tactile parse_meta.h
// Copied from traingle_sampling sample_pcl.cpp
// Concatenates into "parent/child" path.
// Concatenate two parts of a path together, like python os.path.join(), but
//   with only two arguments.
// Parameters:
//   parent, child: Two paths to concatenate together
//   concat: Ret val. concat should be != child, else there may be errors.
void join_paths (const std::string parent, const std::string child,
  std::string & concat, bool canonicalize=true)
{
  // Ref: http://stackoverflow.com/questions/6297738/how-to-build-a-full-path-string-safely-from-separate-strings
  //   http://stackoverflow.com/questions/4179322/how-to-convert-boost-path-type-to-string
  boost::filesystem::path parent_bst (parent);
  boost::filesystem::path child_bst (child);
  boost::filesystem::path concat_bst = parent_bst / child_bst;

  // Ref canonicalization:
  //   https://stackoverflow.com/questions/1746136/how-do-i-normalize-a-pathname-using-boostfilesystem
  //   https://stackoverflow.com/questions/12643880/get-absolute-path-with-boostfilesystempath
  if (canonicalize)
  {
    try
    {
      boost::filesystem::path tmp;
      tmp = boost::filesystem::canonical (concat_bst);
      concat_bst = tmp;
    }
    catch (boost::filesystem::filesystem_error e)
    {
      // Potential causes:
      //   Path does not exist yet, it is constructed for a file to be created
      //   A string manipulation error
      //   A memory error that corrupted the string
      fprintf (stderr, "WARN in join_paths(): %s does not exist.\n",
        concat_bst.string ().c_str ());
    }
  }
  concat = concat_bst.string ();
}

// Overloaded function
// Parameters:
//   subpaths: Vector of many directory names to be concatenated together
//   concat: Ret val
void join_paths (const std::vector <std::string> & subpaths,
  std::string & concat)
{
  // Sanity checks before for-loop below
  if (subpaths.size () < 1)
  {
    concat = std::string ("");
    return;
  }
  else if (subpaths.size () < 2)
  {
    concat = std::string (subpaths [0]);
  }

  boost::filesystem::path concat_bst (subpaths [0]);
  for (int i = 0; i < subpaths.size () - 1; i ++)
  {
    // This indexes starting from [1]. Sanity checks above guards for this
    boost::filesystem::path child_bst (subpaths [i+1]);
    concat_bst = concat_bst / child_bst;
  }

  // Ref canonicalization:
  //   https://stackoverflow.com/questions/1746136/how-do-i-normalize-a-pathname-using-boostfilesystem
  //   https://stackoverflow.com/questions/12643880/get-absolute-path-with-boostfilesystempath
  try
  {
    concat_bst = boost::filesystem::canonical (concat_bst);
  }
  catch (boost::filesystem::filesystem_error e)
  {
    fprintf (stderr, "WARN in join_paths(): %s does not exist. Is this intentional (e.g. path name building for writing?)\n",
      concat_bst.string ().c_str ());
  }

  concat = concat_bst.string ();
}

// Moved from active_visual_tactile parse_meta.h
// Like python os.path.dirname ()
void dirname (const std::string & path, std::string & dir)
{
  // Find last slash before model file name
  // Ref: www.cplusplus.com/reference/string/string/find_last_of/
  std::size_t tmp_idx = path.find_last_of ("/");
  // Get substring from beginning to the char before last "/"
  dir = path.substr (0, tmp_idx);
}

// Like python os.path.basename ()
// Does not include extension of file
void basename (const std::string & path, std::string & base)
{
  boost::filesystem::path path_bst (path);
  base = boost::filesystem::basename (path_bst);
}

// Like python os.path.splitext ()
void splitext (const std::string & path, std::vector <std::string> & exts)
{
  exts.clear ();

  // API http://www.cplusplus.com/reference/string/string/find_last_of/
  std::size_t found = path.find_last_of (".");

  if (found != -1)
  {
    exts.push_back (path.substr (0, found));
    exts.push_back (path.substr (found));
  }
}

void replace_ext (const std::string & in, const std::string & new_ext,
  std::string & out)
{
  // API http://www.cplusplus.com/reference/string/string/find_last_of/
  std::size_t found = in.find_last_of (".");

  out = in.substr (0, found) + new_ext;
}

// TODO: Not tested yet
// Returns formatted date-time string in param.
void current_time_string (std::string & result)
{
  // Ref:
  // http://stackoverflow.com/questions/997512/string-representation-of-time-t
  // http://stackoverflow.com/questions/16357999/current-date-and-time-as-string
  std::time_t now = std::time (NULL);
  std::tm * timeinfo = std::localtime (&now);
  char buffer [80];

  // "%Y-%m-%d-%H-%M-%S"
  std::strftime (buffer, sizeof (buffer), "%Y-%m-%d-%H-%M-%S", timeinfo);

  result = std::string (buffer);
}


/*
class ReadFileLineByLine
{
  private:

    // Pointer, `.` can't use plain std::ifstream as member var. ios_base is
    //   not assignable, compiler errors.
    // Ref: http://www.cplusplus.com/forum/beginner/11541/
    std::ifstream * ifs_;

  public:

    // Ctor
    // Parameters:
    //   ifs: Caller instantiates and passes in e.g.
    //     std::ifstream ifs ("/path/to/file");
    ReadFileLineByLine (std::ifstream & ifs)
    {
      ifs_ = &ifs;
    }

    ~ReadFileLineByLine ()
    {
      ifs_ -> close ();
    }

    bool readline (std::string & ret_val)
    {
      return std::getline (*ifs_, ret_val);
    }

    void close ()
    {
      ifs_ -> close ();
    }

};
*/


#endif
