#!/usr/bin/env python

# Mabel Zhang
# 13 Sep 2018
#
# Simple filters for creating blobbed heat maps from images with sparse
#   non-zero pixels.
# Copied from blob_kernel.py.
#

import numpy as np
import scipy.ndimage

# Emphasizes extrema pixels. Whereas max filter zeros out negative values,
#   this increases the magnitude.
# Ref generic_filter function example
#   https://docs.scipy.org/doc/scipy/reference/tutorial/ndimage.html
def extremum_filter (buff):

  # Find the elt that has max of absolute value, i.e. max of magnitude
  #max_rc = np.unravel_index (np.argmax (np.abs (buff)), buff.shape)
  #return buff [max_rc]

  # Take absolute value, then find max element
  # Maybe linear indexing runs faster?
  max_i = np.argmax (np.abs (buff))
  # Access the max-magnitude element. Return it with its original sign, +/-
  # Ref access using linear index: https://stackoverflow.com/questions/15230179/how-to-get-the-linear-index-for-a-numpy-array-sub2ind
  return buff.ravel () [max_i]


# Parameters:
#   expand, gauss: in pixels
def blob_kernel (img, expand=5, gauss=3):

  # Ref API https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.generic_filter.html
  convolved = scipy.ndimage.generic_filter (img, extremum_filter, size=expand,
    mode='reflect')

  # Blur it
  # API: https://docs.scipy.org/doc/scipy-0.16.1/reference/generated/scipy.ndimage.filters.gaussian_filter.html
  convolved = scipy.ndimage.filters.gaussian_filter (convolved, gauss,
    mode='reflect')

  return convolved

