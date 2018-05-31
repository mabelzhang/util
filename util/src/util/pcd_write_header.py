#!/usr/bin/env python

# Mabel Zhang
# 11 Nov 2015
#
# Moved from triangle_sampling triangles_collect.py, and updated
#

# Python
import os
import argparse

import numpy as np

# My packages
from tactile_collect import tactile_config
from ansi_colors import ansi_colors


# Parameters:
#   pcd_name: Full path of output pcd file
def pcd_write_header (pcd_name, useNormals, nPts):

  pcd_file = open (pcd_name, 'wb')

  pcd_print_header_to_file (pcd_file, useNormals, nPts)

  pcd_file.close ()


# pcd_file: An opened file
def pcd_print_header_to_file (pcd_file, useNormals, nPts):

  #####
  # Write the header
  #   Ref: http://pointclouds.org/documentation/tutorials/pcd_file_format.php
  #####

  pcd_file.write ('# .PCD v0.7 - Point Cloud Data file format\n')
  pcd_file.write ('# Created by Mabel Zhang, data collected using triangles_collect.py\n')

  pcd_file.write ('VERSION 0.7\n')

  if useNormals:
    pcd_file.write ('FIELDS x y z normal_x normal_y normal_z\n')
    pcd_file.write ('SIZE 4 4 4 4 4 4\n')
    pcd_file.write ('TYPE F F F F F F\n')
    pcd_file.write ('COUNT 1 1 1 1 1 1\n')
  else:
    pcd_file.write ('FIELDS x y z\n')
    pcd_file.write ('SIZE 4 4 4\n')
    pcd_file.write ('TYPE F F F\n')
    pcd_file.write ('COUNT 1 1 1\n')

  pcd_file.write (format ('WIDTH %d\n' % nPts))
  pcd_file.write ('HEIGHT 1\n')
  pcd_file.write ('VIEWPOINT 0 0 0 1 0 0 0\n')
  pcd_file.write (format ('POINTS %d\n' % nPts))
  pcd_file.write ('DATA ascii\n')


def pcd_has_header (pcd_name):

  has_header = False

  # Open the PCD file
  with open (pcd_name, 'rb') as pcd_file:

    #####
    # Check if there is header
    #   Ref: http://pointclouds.org/documentation/tutorials/pcd_file_format.php
    #####
 
    for line in pcd_file:

      # Skip comment lines and empty lines, these must be before checking a
      #   line starts with VERSION, because that is the first valid line.
      #   Comment lines must skip to next loop BEFORE checking VERSION, else
      #   might see comment lines as invalid header!
 
      # Skip comment line
      if line.startswith ('#'):
        continue

      # Skip empty line
      if line.strip () == '':
        continue
 
      # According to pointclouds.org, header MUST be in order, which starts with
      #   VERSION. So after comment lines, first valid line must say VERSION,
      #   else can assume this file doesn't have a header.
      if line.startswith ('VERSION'):
        has_header = True
        break
      else:
        has_header = False
        break

  return has_header


# Parameters:
#   pcd_name: Full path of pcd file
# Return True if had header, False else.
# NOTE format in this function must correspond to format in pcd_write_header().
#   If format changes there, must check to change format here too. Else I can't
#   read files correctly!
def pcd_read_header (pcd_name):

  has_header = pcd_has_header (pcd_name)

  useNormals = True
  nPts = 0

  # Open the PCD file
  with open (pcd_name, 'rb') as pcd_file:

    for line in pcd_file:

      line = line.strip ()

      # Skip comment line
      if line.startswith ('#'):
        continue

      # Skip empty line
      if line.strip () == '':
        continue

      # Assumption: each line is a point
      if not has_header:

        # Other than comment line and empty line, first valid line MUST be
        #   VERSION, according to pointclouds.org. If don't see it, then invalid
        #   header, so assume no header.

        # If assumption is too loose, you can check each one can convert to
        #   float without ValueError
        # Ref: http://stackoverflow.com/questions/354038/how-do-i-check-if-a-string-is-a-number-float-in-python
        tokens = line.strip ().split (' ')
        #for val in tokens:
        #  try:
        #    float (val)
        #  except ValueError:
        #    # Don't count this as a point. Go on to next line in file
        #    continue

        # FIELDS x y z normal_x normal_y normal_z
        if len (tokens) == 7:
          useNormals = True
        # FIELDS x y z
        elif len (tokens) == 4:
          useNormals = False

        nPts += 1
        continue

      # If has header, look for the line starting with POINTS
      else:

        # POINTS %d
        if line.startswith ('POINTS'):
          tokens = line.split (' ')
          nPts = int (tokens [1])
          break

        # FIELDS
        if line.startswith ('FIELDS'):
          tokens = line.split (' ')

          # FIELDS x y z normal_x normal_y normal_z
          if len (tokens) == 7:
            useNormals = True
          # FIELDS x y z
          elif len (tokens) == 4:
            useNormals = False


    ''' Don't need this anymore. Python doesn't like mixing iteration and
      readline(), says will lose data.
    #####
    # Read header
    #   Ref: http://pointclouds.org/documentation/tutorials/pcd_file_format.php
    #####

    # If has header, above code should only have read the VERSION line.
    #   Read each subsequent line in order.
    if has_header:

      # FIELDS
      line = pcd_file.readline ().strip ().split (' ')
      # FIELDS x y z normal_x normal_y normal_z
      if len (line) == 7:
        useNormals = True
      # FIELDS x y z
      elif len (line) == 4:
        useNormals = False
     
      # SIZE
      line = pcd_file.readline ()
      # TYPE
      line = pcd_file.readline ()
      # COUNT
      line = pcd_file.readline ()
     
      # WIDTH %d. Expect WIDTH <nPts>, which pcd_write_header() writes
      line = pcd_file.readline ().strip ().split (' ')
      #width = int (line [1])  # Don't need
     
      # HEIGHT %d. Expect HEIGHT 1, which pcd_write_header() writes
      line = pcd_file.readline ().strip ().split (' ')
      #height = int (line [1])  # Don't need
     
      # VIEWPOINT
      line = pcd_file.readline ()
     
      # POINTS %d
      line = pcd_file.readline ().strip ().split (' ')
      nPts = int (line [1])
     
      # DATA %s
     
      # Can ignore all later lines
    '''


  return has_header, useNormals, nPts


# Parameters:
#   in_names: List of full paths (strings) to the CSV files to merge
#   out_name: String. Full path to save the combined file to. This should NOT
#     be the same as any name in in_names!! Else undefined behavior - you'd be
#     reading and writing to a file at the same time.
def combine_many_pcd_files (in_names, out_name):

  # Must read input files twice, once to count total number of points in input
  #   files - so can write header of output PCD file, again to transfer line
  #   by line from input to output file.

  print ('Merging these PCD files into one:')
  for in_name in in_names:
    print ('  %s' % in_name)


  # Sanity check. If file exists, ask user whether to overwrite
  if os.path.exists (out_name):
    valid_input = False
    while not valid_input:
      uinput = raw_input ('%sFile already exists at %s. Overwrite (Y/N)? %s' % (ansi_colors.WARNING, out_name, ansi_colors.ENDC))
      if uinput.lower () != 'y' and uinput.lower () != 'n':
        print ('Invalid input. Enter Y or N.')
        continue
      else:
        valid_input = True

    if uinput.lower () == 'n':
      print ('Will NOT overwrite. Returning...')
      return 0


  # Loop through each input file, count total number of points
  nPts_ttl = 0
  useNormals = True
  for in_name in in_names:

    # Assumption: useNormals is same across all files, so just overwrite each
    #   iteration's ret val
    _, useNormals, nPts = pcd_read_header (in_name)
    nPts_ttl += nPts


  # Write header to output file
  pcd_write_header (out_name, useNormals, nPts_ttl)

  # Don't open file before calling pcd_write_header(), that function will
  #   open file again, opening twice without closing is not safe.
  # Open in append mode, so don't lose the header we just wrote!
  out_file = open (out_name, 'a')
  print ('PCD files will be merged to %s, with %d points' % (out_name, nPts_ttl))

  # Loop through each input file, transfer line by line to output file
  for in_name in in_names:
    with open (in_name, 'rb') as in_file:

      for line in in_file:

        # Don't write comment lines, not sure if valid in data section of
        #   PCD file.
        if line.startswith ('#'):
          continue

        # Don't write any extra header lines! Header is already written above,
        #   do not write header lines in individual input files, they will
        #   appear in the middle of the output file, among the data!
        # Assumption: Lines starting with a letter are header lines. Point
        #   lines should always start with numbers.
        if line [0].isalpha ():
          continue

        out_file.write (line)

  out_file.close ()

  return nPts_ttl


# Returns NumPy array nPoints x 3, or nPoints x 6 if FIELDS contain normals.
# Parameters:
#   pcd_name: Full path to a pcd file
# Test using:
'''
import pcd_write_header
import numpy as np
a = pcd_write_header.pcd_read_pts_to_array ('/media/master/Data/courses/research/graspingRepo/train/triangle_sampling/pcd_gz_collected/sphere_3cm_gz.pcd')
reload (pcd_write_header)
'''
def pcd_read_pts_to_array (pcd_name):

  # Get number of points, so can initialize full NumPy array with size before
  #   reading points data, to save running time.
  _, useNormals, nPts = pcd_read_header (pcd_name)

  # Preallocate 2D array with known size, to save running time. Appending each
  #   row of .pcd file to a constantly changing size array wastes time
  if useNormals:
    pts = np.empty ((nPts, 6))
  else:
    pts = np.empty ((nPts, 3))

  row_i = 0

  with open (pcd_name, 'rb') as pcd_file:
    for line in pcd_file:

      # Don't write comment lines, not sure if valid in data section of
      #   PCD file.
      if line.startswith ('#'):
        continue

      # Don't write any extra header lines! Header is already written above,
      #   do not write header lines in individual input files, they will
      #   appear in the middle of the output file, among the data!
      # Assumption: Lines starting with a letter are header lines. Point
      #   lines should always start with numbers.
      if line [0].isalpha ():
        continue

      # Split the row of 6 floats in a string, delimited by space
      # Ref: http://stackoverflow.com/questions/11844986/convert-string-to-variables-like-format-but-in-reverse-in-python
      # Add an extra [] to make list into 2D array, for appending to 2D np arr
      row = np.array ([[float(num) for num in line.split ()]])

      #print (pts.shape)
      #print (row.shape)

      # Save row to matrix
      pts [row_i, :] = row

      row_i += 1

  return pts


# Copied from triangle_sampling triangles_collect.py
# Write a full PCD file, including the header and the body (lines of points).
# Parameters:
#   poses: n x 3
#   normals: n x 3
def record_pcd (outname, poses, normals=None):
    # Does PCL Python binding at least have a PointCloud type and a 
    #   writePCD() function? If so, then I can just keep adding pts to the 
    #   PointCloud type, then write to file at the end!
    #   `.` you cannot write a PCD file before knowing how many points
    #   you have in total! That information is required at top of file!
    # Alternative is, I can just write to a text file, then prepend the
    #   header after the run, using another node. It'd write the plain
    #   text to get number of points, then write to a new file with the
    #   header, and copy the text over.
    #   This is safer `.` if program ends or robot shut down whatever, I
    #   still have partial data!!!
    #   I also prefer this because I don't want to depend on PCL, as
    #   much as possible. It's a monster and it's horrible.
    #
    #   Actually I could even just output from this file, if it gets to
    #   the end! Then I can just keep code all in this node, don't have
    #   to write another, and don't have to always run another!
    #
    # Look at this file for reference of how to write a PCD file
    #   /home/master/graspingRepo/train/3DNet/cat10/train/pcd/apple/bd41ae80776809c09c25d440f3e4e51d.pcd
            
    nPts = np.shape (poses) [0]
    useNormals = (normals != None)

    nPts_stored = 0
    with open (outname, 'wb') as outfile:

      pcd_print_header_to_file (outfile, useNormals, nPts)

      # Print in yellow temporarily for debugging
      #print ('\n%s%d contacts%s' % (ansi_colors.WARNING, len(poses),
      #  ansi_colors.ENDC))
      #print ('Recording these x y z [nx ny nz] lines to .pcd file:')

      for i in range (0, len (poses)):

        # Copied from tactile_map est_center_axis_ransac.py

        # Keep 12 places after decimal. Robot has a lot of noise anyway, not worth
        #   keeping more. Not to mention Python also has floating point errors
        if not normals:
          line = format ('%.12g %.12g %.12g\n' \
          % (poses [i][0], poses [i][1], poses [i][2]))

        # If normals are specified, use them
        # Assumption: user pass in or does not pass in normals consistently. i.e.
        #   if normal is passed in for first point, it must be passed in for all
        #   subsequent points. Otherwise file is invalid! Currently we do not
        #   check this. It is easy to check though!!
        else:
            line = format ('%.12g %.12g %.12g %.12g %.12g %.12g\n' \
            % (poses [i][0], poses [i][1], poses [i][2],
                normals [i][0], normals [i][1], normals [i][2]))

        outfile.write (line)

        # Increment total number of points in the PCD file.
        #   One point per line. So best place to increment is when write a line.
        nPts_stored += 1
    
    print ('%s%d points total written to PCD file%s' % (ansi_colors.OKCYAN,            
        nPts_stored, ansi_colors.ENDC))
    
    # Return number of points written in this call
    return nPts_stored


# Didn't finish writing main(). Just used pcd_write_header in
#   triangles_collect.py. It works fine.
def main ():

  pcd_path_default = tactile_config.config_paths ('custom',
    'triangle_sampling/pcd_gz_collected/')


  #####
  # Parse command line args
  #   Ref: Tutorial https://docs.python.org/2/howto/argparse.html
  #        Full API https://docs.python.org/dev/library/argparse.html
  #####

  arg_parser = argparse.ArgumentParser ()

  # Variable number of args http://stackoverflow.com/questions/13219910/argparse-get-undefined-number-of-arguments
  arg_parser.add_argument ('in1', type=str,
    help='Full path to 1st pcd file to concat in order')
  arg_parser.add_argument ('in2s', type=str, nargs='+',
    help='Full path to 2nd [and more] pcd file to concat in order')
  arg_parser.add_argument ('out', type=str,
    help='Full path to output resulting concatenated pcd file')

  args = arg_parser.parse_args ()

  in_names = [args.in1]
  in_names.extend (args.in2s)

  out_name = args.out


  '''
  #arg_parser.add_argument ('pcd_path', type=str, default=pcd_path_default,
  #  help='Prefix path of PCD file')
  #arg_parser.add_argument ('pcd_infile_name', type=str,
  #  help='Base name of existing PCD file with only points stored and no header')

  #arg_parser.add_argument ('--useNormals', action='store_true', default=True,
  #  help='PCD file header information. True or False, depends on your input PCD file that has all the points already.')

  args = arg_parser.parse_args ()


  pcd_path = args.pcd_path
  pcd_tmp_name = args.pcd_infile_name
  useNormals = args.useNormals

  # TODO
  # Parse pcd_tmp_name to get this
  #timestring = 


  pcd_tmp_file = open (os.path.join (pcd_path, pcd_tmp_name), 'rb')

  nPts = 0
  # Read a line
  for line in pcd_tmp_file:
    nPts += 1

  #pcd_write_header (pcd_path, timestring, pcd_tmp_name, pcd_tmp_file,
  #  useNormals, nPts)
  '''


  # Simple way to let user to concatenate two files by running this file in
  #   command prompt $.
  combine_many_pcd_files (in_names, out_name)


if __name__ == '__main__':
  main ()

