#!/bin/bash

# Mabel Zhang
# 30 Nov 2018


import csv

import numpy as np


def csv_transpose (inname, outname):

  rows = []

  print ('Reading matrix from %s' % inname)
  with open (inname, 'rb') as f:
    reader = csv.reader (f)
    for row in reader:
      # Convert strings to floats
      rows.append ([float (s) for s in row])

  with open (outname, 'wb') as f:
    writer = csv.writer (f)
    writer.writerows (np.array (rows).T)
  print ('Written transpose of matrix to %s' % outname)


if __name__ == '__main__':
  csv_transpose ('/media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/data/contacts/part3.csv', '/media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/data/contacts/part3_transpose.csv')

