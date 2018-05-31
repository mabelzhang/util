#!/usr/bin/env python

# Mabel Zhang
# 4 Feb 2016
#
# Combine multiple csv files into one, by concatenating. Header row must be
#   the same across all input csv files. In output file, header row is only
#   written once, at the very beginning (This is the part that's tedious
#   by manual "cat" commands, you'd have to go in and delete the repeated
#   header rows in the middle of the output file yourself. Hence the existence
#   of this script).
#

import os
import csv
import argparse

# My packages
from ansi_colors import ansi_colors


# This fn needs to open files. Make sure all files are closed before
#   calling the fn.
# Parameters:
#   in_names: List of full paths (strings) to the CSV files to merge
#   out_name: String. Full path to save the combined file to
#   has_header: Whetehr csv file has a header row
# Returns total lines (excluding header) in combined file.
def combine_many_csv_files (in_names, out_name, has_header=True):

  if len (in_names) < 1:
    print ('%scombine_many_csv_files: No input files specified. Returning...%s')
    return -1

  # Open input files, and initialize csv readers
  print ('Merging these CSV files into one:')
  in_files = []
  in_readers = []
  for in_name in in_names:
    print ('  %s' % in_name)

    in_file = open (in_name, 'rb')
    in_files.append (in_file)

    if has_header:
      in_readers.append (csv.DictReader (in_file))
    else:
      in_readers.append (csv.reader (in_file))


  # Already checked in_readers has more than length 0 above. Safe to access [0]
  fieldnames = in_readers [0].fieldnames

  # Sanity check. Make sure headers are the same in all input files!
  for in_reader in in_readers:
    if in_reader.fieldnames != fieldnames:
      print ('%scombine_many_csv_files: Headers of the input CSV files do not match. No automatic merge can be done. You must merge manually. Returning...%s' % (ansi_colors.FAIL, ansi_colors.ENDC))

      for in_file in in_files:
        in_file.close ()

      return -1


  # Create output file

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
      for in_file in in_files:
        in_file.close ()
      return -1


  out_file = open (out_name, 'wb')
  print ('CSV files will be merged to %s' % out_name)

  ttl_lines = 0

  if has_header:
    out_writer = csv.DictWriter (out_file, fieldnames=fieldnames)
    out_writer.writeheader ()
  else:
    out_writer = csv.writer ()

  # Read each input file. Write each row to output file
  for in_reader in in_readers:
    for row in in_reader:
      ttl_lines += 1
      out_writer.writerow (row)

  # Close all files when done
  for in_file in in_files:
    in_file.close ()
  out_file.close ()

  return ttl_lines



# This is a way to test the functions in this file
def main ():

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
    help='Full path to 2nd [and more] csv file to concat in order')
  arg_parser.add_argument ('out', type=str,
    help='Full path to output resulting concatenated pcd file')

  args = arg_parser.parse_args ()

  in_names = [args.in1]
  in_names.extend (args.in2s)
  out_name = args.out



  # Simple test

  # 49 + 196 + 487 triangles (exclude header line)
  #in_names = ['/media/master/Data/courses/research/graspingRepo/train/triangle_sampling/csv_tri/simple/cube_3cm_gz_robo.csv',
  #  '/media/master/Data/courses/research/graspingRepo/train/triangle_sampling/csv_tri/simple/cube_8cm_gz_robo.csv',
  #  '/media/master/Data/courses/research/graspingRepo/train/triangle_sampling/csv_tri/simple/sphere_3cm_gz_robo.csv']

  # 732 triangless
  #out_name = '/media/master/Data/courses/research/graspingRepo/train/triangle_sampling/csv_tri/simple/test.csv'

  combine_many_csv_files (in_names, out_name, True)


if __name__ == '__main__':
  main ()

