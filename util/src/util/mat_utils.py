#!/usr/bin/env python

# Mabel Zhang
# 2 Sep 2016
#
# Matrix utilities.
# Utility functions for numpy arrays, especially vectorization operations
#   that eliminate for-loops!
#

from copy import deepcopy

import numpy as np

from util.ansi_colors import ansi_colors


# Parameters:
#   mat, dups: n x m numpy 2D arrays. Return rows in mat that aren't duplicates
#     of dup. Alternatively, return a copy of mat, with rows in dups
#     removed. mat is not altered.
#   dup_thresh: Scalar threshold, a pair of numbers with difference below this
#     threshold will be considered duplicates.
# No for-loops
# To test this function:
#   mat = np.random.rand (6, 3)
#   dups = deepcopy (mat [[2,4], :])
#   print (extract_non_duplicates (mat, dups))
def extract_non_duplicates (mat, dups, dup_thresh=1e-6, get_copy=False):

  # Number of columns must be same
  assert mat.shape [1] == dups.shape [1]
  n_dims = dups.shape [1]

  n_dups = dups.shape [0]

  # Tile mat into 3rd dimension, so each mat can subtract 1 row in dups
  # This is the vectorization to avoid for-loop on dups.
  # Shape becomes (n_dups, mat.shape[0], n_dims)
  mat_3d = np.tile (mat, (n_dups, 1, 1))

  # Reshape dups into 3rd dimension
  # Shape becomes (n_dups, 1, n_dims)
  dups_3d = np.reshape (np.array ([dups]), (n_dups, 1, n_dims))

  diff = mat_3d - dups_3d
  # Result is n_dups x n_samples. Sample rows in mat that are duplicates
  #   of dups, will be marked with True.
  #   e.g. array([[False, False,  True, False, False, False],
  #               [False, False, False, False,  True, False]], dtype=bool)
  #     indicates samples [2] and [4] in mat are duplicates.
  dup_bools = np.all (diff < dup_thresh, axis=2)
  # Sum the booleans downwards, to get 1s where duplicates are. This is
  #   essentially summing across all slices in 3rd dimension, to find which
  #   slices in mat_3d got 0 when it subtracted dups_3d.
  # np.where == 0 on the sum, to get row indices whose bools summed to 0,
  #   e.g. everywhere that's not rows [2,4]. These are the non-dups.
  nondup_rows_i = np.where (np.sum (dup_bools, axis=0) == 0) [0]

  if get_copy:
    new_mat = deepcopy (mat)
    new_mat = new_mat [nondup_rows_i, :]
  else:
    new_mat = mat [nondup_rows_i, :]

  return new_mat



# =============== Below are copied from my new mat_util.py in continuumRepo ==

# Find if a row in samples is a duplicate of the given row.
# Parameters:
#   row: 1 x m
#   samples: nSamples x m
# Returns indices in samples that are duplicates of row
def find_dups_row (row, samples, thresh=1e-6):

  if row.ndim != 1:
    print ('%sERROR: mat_util.py find_dups(): row arg is not a 1D array. Cannot find duplicates! Check your find_dups() call.%s' % (
      ansi_colors.FAIL, ansi_colors.ENDC))
    return []

  # Check if row is a duplicate of an existing row in samples. Subtract,
  #   check if ~0 to get m x n booleans. Take rowwise AND.
  #   If all columns in a row are True, meaning a duplicate row.
  rowwise_eq = np.all (np.abs (samples - row.reshape (1, row.size)) < 1e-6, axis=1)
  dup_idxes = np.where (rowwise_eq) [0]

  return dup_idxes


# Find rows in incoming rows that already exist in existing rows.
# Order matters!! The return values are ordered based on the params being in
#   order.
# Returns two lists. Values are indices. nDups x 1. These are the rawest
#   returned from this file.
def find_dups_multirow_twolists (incoming, existing, thresh=1e-6):

  # Tile in 3rd dimension, so the matrix becomes nRows x 1 x nCols,
  #   depth x rows x cols
  incoming_3d = incoming.reshape (incoming.shape [0], 1, incoming.shape [1])

  # Result of subtraction is nIncomingChords x nExistingChords x nDims,
  #   depth x rows x cols.
  #   If a chord is a duplicate, all columns (which is axis=2) of a row
  #     would be 0s.
  # Result of np.all(, axis=2) is nIncomingChords x nExistingChords.
  #   In dup_bool [r, c], r indexes incoming rows, c indexes
  #     existing rows.
  #   If dup_bool [r, c] is true, that means incoming chord r is a duplicate
  #     of existing chord c.
  dup_bool = np.all (np.abs (existing - incoming_3d) < 1e-6, axis=2)
  dup_incoming_row, dup_existing_row = np.where (dup_bool)

  return dup_incoming_row, dup_existing_row


# Returns one list. Its indices index "incoming", its values index "existing".
#   nRows x 1. Value -1 indicates not a duplicate.
# Assumption: incoming != existing. If they are the same, then call
#   find_dups_self() instead! There are cases specific to incoming == existing 
#   that need additional care.
def find_dups_multirow_onelist (incoming, existing, thresh=1e-6):

  # It could be that an incoming row has multiple duplicates in existing rows,
  #   then you can have pairs e.g. (3, 1), (3, 2), (3, 3), note 3 is repeated.
  dup_incoming_row, dup_existing_row = find_dups_multirow_twolists (
    incoming, existing, thresh)

  # If a row finds multiple duplicates in incoming, just pick first one
  # np.unique() returns index of first occurence.
  uniq_dups, uniq_dup_idxes = np.unique (dup_incoming_row, return_index=True)
  '''
  print ('dup_incoming_row:')
  print (dup_incoming_row)
  print ('dup_existing_row:')
  print (dup_existing_row)
  print ('uniq_dups:')
  print (uniq_dups)
  print ('uniq_dup_idxes:')
  print (uniq_dup_idxes)
  '''

  if uniq_dups.size < dup_incoming_row.size:
    print ('%d extra pairings (for %d rows) are found. This means some incoming rows have multiple duplicates in existing rows. Will take the first occurrence in existing samples as the duplicate index.' % (
      dup_incoming_row.size - uniq_dup_idxes.size, uniq_dup_idxes.size))
    dup_incoming_row = dup_incoming_row [uniq_dup_idxes]
    dup_existing_row = dup_existing_row [uniq_dup_idxes]
  '''
  print ('dup_incoming_row:')
  print (dup_incoming_row)
  print ('dup_existing_row:')
  print (dup_existing_row)
  '''

  # Initialize to -1, to indicate no duplicates
  dup_idxes = np.zeros (incoming.shape [0],) - 1
  # Mark rows corresponding to duplicate incoming chords
  dup_idxes [dup_incoming_row] = dup_existing_row

  return dup_idxes


def find_dups_self (samples, thresh=1e-6):

  # nSamples x 1
  dup_idxes = find_dups_multirow_onelist (samples, samples, thresh)

  assert (dup_idxes.size == samples.shape [0])

  # Assumption: dup_idxes.size == samples.shape [0]
  idxes = range (0, dup_idxes.size)

  # An item is always duplicate of itself. Don't count it as a duplicate,
  #   i.e. set to -1 to indicate not a duplicate
  dup_idxes [dup_idxes == idxes] = -1

  return dup_idxes


