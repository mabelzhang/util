#!/usr/bin/env python

# Mabel Zhang
# 18 May 2017
#
# Utility functions for file I/O.
#

import os
import glob
import shutil

import time
from datetime import datetime


class TimestringFormat:

  fmt = '%Y-%m-%d-%H-%M-%S'


def current_timestamp_string ():

  # Output log file of predictions from tree search + Gazebo execution
  # Ref: http://stackoverflow.com/questions/13890935/timestamp-python
  timestamp = time.time ()
  timestring = datetime.fromtimestamp (timestamp).strftime (
    TimestringFormat.fmt)

  return timestring


# Compare strings returned by current_timestamp_string()
def compare_timestrings_float (str1, str2):

  # Ref: https://stackoverflow.com/questions/21378977/compare-two-timestamps-in-python
  #   https://docs.python.org/2/library/datetime.html#strftime-and-strptime-behavior
  t1 = datetime.strptime (str1, TimestringFormat.fmt)
  t2 = datetime.strptime (str2, TimestringFormat.fmt)

  diff = (t2 - t1).total_seconds ()

  return diff


# Slower way probably. Use compare_timestrings_float().
def compare_timestrings_int (str1, str2):

  return (datetime_to_timestamp (str2) - datetime_to_timestamp (str1))


# Returns number of seconds elapsed since epoch
# Note that no timezone are taken into consideration.
# utc tuple must be: (year, month, day, hour, minute, second)
# https://stackoverflow.com/questions/5067218/get-utc-timestamp-in-python-with-datetime
def utc_mktime (utc_tuple):

  if len (utc_tuple) == 6:
    utc_tuple += (0, 0, 0)

  return time.mktime (utc_tuple) - time.mktime ((1970, 1, 1, 0, 0, 0, 0, 0, 0))

# Converts a datetime object to UTC timestamp
# Returns number of seconds as integer
def datetime_to_timestamp (dt):

  return int (utc_mktime (dt.timetuple ()))



# Uses glob which is Unix only
#   https://stackoverflow.com/questions/3348753/search-for-a-file-using-a-wildcard
#   https://stackoverflow.com/questions/3207219/how-do-i-list-all-files-of-a-directory
def ls_unix (path, wildcard='*'):

  # glob returns full paths
  # List all files in given path
  return glob.glob (os.path.join (path, wildcard))


# Use listdir if you want it to work outside of Unix. I don't care for that.
#   https://stackoverflow.com/questions/3207219/how-do-i-list-all-files-of-a-directory
#def ls (path, ):

#  files = os.listdir (path)
#  for f in files:
    

# Parameters:
#   Absolute paths
def mv (old_name, new_name):
  shutil.move (old_name, new_name)


