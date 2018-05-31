#!/usr/bin/env python

# Mabel Zhang
# 4 Sep 2015
#
# Refactored from triangle_sampling package triangles_svm.py
#
# Ref: http://scikit-learn.org/stable/auto_examples/plot_confusion_matrix.html
#


# Numpy
import numpy as np
from sklearn.metrics import confusion_matrix

# matplotlib
import matplotlib.pyplot as plt

# My packages
from util.ansi_colors import ansi_colors
from util.matplotlib_util import black_background, black_colorbar


# Because sklearn's train_test_split() is retarded and only splits the ratio
#   for overall data. It doesn't keep the ratio for each category. So when you
#   only have a few samples in a category, it can happen that you have 0
#   instances for testing!
# Used by triangle_sampling triangles_svm.py, task_compat_detection
#   regress_svm.py.
# Parameters:
#   samples: NumPy array, nSamples x nDims
#   lbls: Python list of n integers
def my_train_test_split (samples_orig, lbls_orig, test_size, random_state):

  rs = np.random.RandomState ()
  rs.seed (random_state)


  #####
  # Re-shuffle samples first. Generate a permuted list
  #####

  shuffle_idx = rs.permutation (len (lbls_orig))

  nDims = np.shape (samples_orig) [1]
  samples = np.zeros ((0, nDims))
  lbls = []

  for i in range (0, len (shuffle_idx)):
    # Grab row indexed by shuffle_idx[i].
    #   Make it 2D array, to preserve the row shape to pass to np.append().
    samples = np.append (samples,
      np.array ([samples_orig [shuffle_idx [i], :]]), axis=0)
    lbls.append (lbls_orig [shuffle_idx [i]])

  # Now that the samples are shuffled, can assign train/test split in order,
  #   without randomization needed (randomization already applied at
  #   permutation above).
  #   The ordered assignment is easier to check ratio-per-category! Hard to do
  #   that while randomly assigning a split.
  

  #####
  # Assign train/test split
  #####

  samples_tr = np.zeros ((0, nDims))
  samples_te = np.zeros ((0, nDims))
  lbls_tr = []
  lbls_te = []

  TRAIN = 0
  TEST = 1

  # Number of labels
  lbls_uniq = np.unique (lbls)
  # Get total number of samples per class, to calculate percentage later.
  #   Use a hashmap (dictionary), instead of list, because someone's labels
  #   might not start with 0, so you don't want to use the labels as list
  #   index. unique() also returns sorted list, so you won't know which elt to
  #   index! Just use the actual label as a key, easier.
  ttl_per_label = dict ()
  n_tests_per_label = dict ()
  n_trains_per_label = dict ()
  for i in range (0, len (lbls_uniq)):
    # Total number of samples per class
    ttl_per_label [lbls_uniq [i]] = np.size (np.where (lbls == lbls_uniq [i]))

    # Init this dictionary. Number of samples assigned to TEST SET so far
    n_tests_per_label [lbls_uniq [i]] = 0
    n_trains_per_label [lbls_uniq [i]] = 0


  train_size = 1.0 - test_size

  # Assign train or test, keeping ratio satisfied
  for i in range (0, len (lbls)):

    # Prioritize test
    #if float (n_tests_per_label [lbls [i]]) / ttl_per_label [lbls [i]] < \
    #  test_size:
    #  assign = TEST
    #else:
    #  assign = TRAIN

    # Prioritize train, `.` train can't have 0 samples!
    if float (n_trains_per_label [lbls [i]]) / ttl_per_label [lbls [i]] < \
      train_size:
      assign = TRAIN
    else:
      assign = TEST

    # Add ith row in samples to the corresponding split
    if assign == TRAIN:
      samples_tr = np.append (samples_tr, np.array ([samples [i, :]]), axis=0)
      lbls_tr.append (lbls [i])

      n_trains_per_label [lbls [i]] += 1

    else:
      samples_te = np.append (samples_te, np.array ([samples [i, :]]), axis=0)
      lbls_te.append (lbls [i])

      n_tests_per_label [lbls [i]] += 1

  return samples_tr, samples_te, lbls_tr, lbls_te


# Parameters:
#   ticks: String labels to display on x- and y-ticks. e.g. category names,
#     instead of integer class IDs that won't give you a clue what the class is.
#     If [], will just use default xticks (integers).
#   img_name: Full path to save image to. If '', will not save.
#   bg_color: Set 'black" for black background and white text, for black
#     presentation slides
def draw_confusion_matrix (true_lbls, predicted_lbls, ticks=[], img_name='',
  title_prefix='', draw_title=True, raw=False, bg_color='white'):
  
  # Compute confusion matrix
  # Make sure you keep order the same. In (true, predicted) order, the y-axis
  #   is true label, the x-axis is predicted. If you swap these, then the
  #   plot's x- and y-labels need to swap as well.
  # Ref: http://scikit-learn.org/stable/modules/generated/sklearn.metrics.confusion_matrix.html
  cm = confusion_matrix (true_lbls, predicted_lbls)
  print ('Confusion matrix:')
  print (cm)

  # Do percentages, instead of raw numbers that are returned by default
  if not raw:
    # Sum each row to get total number of objects in each category (y-axis
    #   label of confusion mat is true labels, so sum rows, not columns).
    #   Need floating point so cm divided by this array gives floats, not ints!
    nTtl = np.sum (cm, axis=1).astype (np.float32)

    # Reshape to column vector, so element-wise divide would divide each row of
    #   conf mat by total object sum in that row.
    nTtl = nTtl.reshape ((np.size (nTtl), 1))

    # Element-wise divide each row of confusion matrix by the total # objects
    cm = np.divide (cm, nTtl)
    #print (cm)

  # Show confusion matrix in a separate window
  # Using a figure makes colorbar same length as main plot!
  fig = plt.figure ()
  plt.matshow (cm, fignum=fig.number)
  plt.set_cmap ('jet')
  colorbar = plt.colorbar()
  if draw_title:
    # Ref move title up: http://stackoverflow.com/questions/12750355/python-matplotlib-figure-title-overlaps-axes-label-when-using-twiny
    plt.title(title_prefix + 'Confusion matrix', y=1.2)
  plt.ylabel('True label')
  plt.xlabel('Predicted label')


  # Display class names on the axes ticks
  if ticks:
    draw_classname_ticks (cm, ticks)

  # Set background color
  if bg_color == 'black':
    black_background ()
    black_colorbar (colorbar)


  if img_name:
    # Ref savefig() black background: https://stackoverflow.com/questions/4804005/matplotlib-figure-facecolor-background-color
    plt.savefig (img_name, bbox_inches='tight',
      facecolor=fig.get_facecolor (), edgecolor='none', transparent=True)
    print ('Plot saved to %s' % img_name)

  plt.show()


# Parameters:
#   unsorted_idx: n x n. Numpy 2D array of indices. Sort this to obtain the
#     0 to n ordering for the matrix.
#   distance_matrix: n x n. Distance matrix btw exhaustive pairwise distances
#     among n objects. Sorted left to right by close to far.
#     Not sorted by sample ID! This function sorts the unsorted_idx indices,
#     and indexes distance_matrix using the sorted indices, so that the
#     confusion matrix can be labeled as 0 to n for x and y axes.
#   ticks: Custom ticks, e.g. text ticks, if you don't want the default numeric
#     ticks. If not provided, numerical ticks will be used every
#     units_per_tick (set in function).
#   bg_color: Set 'black" for black background and white text, for black
#     presentation slides
# Useful for plotting knn distances.
def draw_confusion_matrix_per_sample (unsorted_idx, distance_matrix, ticks=[],
  img_name='', title_prefix='',
  draw_title=True, draw_xylbls=True, draw_xticks=True, draw_yticks=True,
  fontsize=10, bg_color='white', cmap_name='jet_r'):

  # Matrix is n x n
  nSamples = np.shape (unsorted_idx) [0]

  dists_sorted = np.zeros (np.shape (distance_matrix))
  
  for i in range (0, nSamples):

    # Sort the ith row of sample IDs. Save the indexes from sorting.
    # Ref: http://docs.scipy.org/doc/numpy/reference/generated/numpy.argsort.html
    sorted_idx_rowi = np.argsort (unsorted_idx [i, :])

    # Take ith row of distance matrix, index it using the sorted indices
    dists_sorted [i, :] = distance_matrix [i, :] [sorted_idx_rowi]


  print ('')
  print ('Max distance: %f. Min distance: %f. Mean distance: %f' % \
    (np.max (dists_sorted), np.min (dists_sorted), np.mean (dists_sorted)))

  print ('Confusion matrix:')
  print (dists_sorted)

  # Show confusion matrix in a separate window
  # Using a figure makes colorbar same length as main plot!
  fig = plt.figure ()
  plt.matshow (dists_sorted, fignum=fig.number)
  if draw_title:
    if ticks:
      plt.title(title_prefix + 'Confusion matrix', y=1.3)
    else:
      plt.title(title_prefix + 'Confusion matrix')
  # Use jet_r. Reverse colors, so that small dists are hot, far dists are cold.
  #   http://stackoverflow.com/questions/3279560/invert-colormap-in-matplotlib
  # Save orig cmap so can set it back for future plots
  #orig_cmap = plt.get_cmap ()
  plt.set_cmap (cmap_name)
  colorbar = plt.colorbar()

  if draw_xylbls:
    plt.ylabel('Objects')
    plt.xlabel('Objects')

  ax = plt.gca ()

  # Display class names on the axes ticks
  if draw_xticks or draw_yticks:
    # Custom ticks passed in
    if len (ticks) > 0:
      draw_classname_ticks (dists_sorted, ticks,
        draw_x=draw_xticks, draw_y=draw_yticks, fontsize=fontsize)

    # Automatically generate numerical ticks, every 20 units
    else:
      units_per_tick = 20.0

      if draw_xticks:
        # You can adjust these to make as frequent or as rare ticks as you want
        plt.xticks (np.arange (0, nSamples, units_per_tick))

      if draw_yticks:
        plt.yticks (np.arange (0, nSamples, units_per_tick))

      #print (nSamples)
      #print (np.arange (0, nSamples, units_per_tick))

  if not draw_xticks:
    # http://stackoverflow.com/questions/2176424/hiding-axis-text-in-matplotlib-plots
    #ax.get_xaxis ().set_visible (False)
    ax.get_xaxis ().set_ticks ([])

  if not draw_yticks:
    #ax.get_yaxis ().set_visible (False)
    ax.get_yaxis ().set_ticks ([])


  for tick in ax.xaxis.get_major_ticks ():
    # http://stackoverflow.com/questions/6390393/matplotlib-make-tick-labels-font-size-smaller
    tick.label.set_fontsize (10)
    # specify integer or one of preset strings, e.g.
    #tick.label.set_fontsize('x-small') 
    #tick.label.set_rotation('vertical')


  # Set background color
  if bg_color == 'black':
    black_background ()
    black_colorbar (colorbar)

  if img_name:
    # Ref savefig() black background: https://stackoverflow.com/questions/4804005/matplotlib-figure-facecolor-background-color
    plt.savefig (img_name, bbox_inches='tight',
      facecolor=fig.get_facecolor (), edgecolor='none', transparent=True)
    print ('Plot saved to %s' % img_name)

  plt.show()

  # This opens a window! I don't know how to do it without opening a window.
  #   So not doing it anymore.
  # Set colors back to whatever they were before
  #plt.set_cmap (orig_cmap)


def draw_classname_ticks (mat, ticks, draw_x=True, draw_y=True, fontsize=10):

  #print ('Plotting using these axis tick labels:')
  #print (ticks)

  # Ref set textual tick labels: http://stackoverflow.com/questions/5439708/python-matplotlib-creating-date-ticks-from-string
  ax = plt.gca ()

  if draw_x:
    ax.set_xticklabels (ticks, rotation='vertical')

    # Show ALL ticks (by default, it doesn't show all of them)
    # Ref: http://stackoverflow.com/questions/12608788/changing-the-tick-frequency-on-x-or-y-axis-in-matplotlib
    # ticks() font size
    #   http://stackoverflow.com/questions/13139630/how-can-i-change-the-font-size-of-ticks-of-axes-object-in-matplotlib
    plt.xticks (np.arange (0, np.shape (mat) [0], 1.0), fontsize=fontsize)

  if draw_y:
    ax.set_yticklabels (ticks)
    plt.yticks (np.arange (0, np.shape (mat) [0], 1.0), fontsize=fontsize)


def calc_accuracy (true_lbls, predicted_lbls, nClasses):

  # Find number of predictions that matched ground truth for test data.
  #   Can use np.sum() or np.count_nonzero() on a boolean array.
  # Ref: http://stackoverflow.com/questions/8364674/python-numpy-how-to-count-the-number-of-true-elements-in-a-bool-array
  nCorrect = np.sum (predicted_lbls == true_lbls)
  nSamples_te = len (true_lbls)

  acc = float (nCorrect) / float (nSamples_te)

  print ('Accuracy out of %d classes: %f%% (%d/%d)' % (nClasses, acc * 100.0,
    nCorrect, nSamples_te))

  return acc


# Parameters:
#   true_vals, predicted_vals: 1D NumPy arrays. Must be same size.
#   thresh: Threshold difference beyond which a predicted value would be
#     considered misprediction. E.g. true label is binary 0, prediction is 0.4,
#     if threshold is 0.5, then prediction is counted as correct. If threshold
#     is 0.2, then prediction is counted as incorrect.
def calc_regression_accuracy (true_vals, predicted_vals, thresh):

  # Sanity check
  if np.size (true_vals) != np.size (predicted_vals):
    print ('ERROR in calc_regression_accuracy(): Number of true labels and predicted labels do not match! They must be the same size. Check your inputs. Returning -1 for accuracy!' % (
      ansi_colors.FAIL, ansi_colors.ENDC))
    return -1

  nCorrect = np.sum (np.abs (true_vals - predicted_vals) < thresh)
  nSamples = len (true_vals)

  acc = float (nCorrect) / float (nSamples)

  nTrueVals = np.unique (true_vals).size
  print (np.unique (true_vals))

  print ('Accuracy out of %d unique regression labels: %f%% (%d/%d)' % (
    nTrueVals, acc * 100.0, nCorrect, nSamples))

  return acc

