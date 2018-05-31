#!/usr/bin/env python

# Mabel Zhang
# 6 Sep 2016
#
# Utility functions for matplotlib
#

import matplotlib
# For running on GPU cluster remotely, which does not have python-tk
import socket
if socket.gethostname () != 'snaefellsjokull':
  # Ref: https://stackoverflow.com/questions/4930524/how-can-i-set-the-backend-in-matplotlib-in-python
  matplotlib.use ('Agg')
import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap
from matplotlib.colors import LinearSegmentedColormap  # For custom colormap

import numpy as np

from util.ansi_colors import ansi_colors


# Call this fn at start of program, if you need truetype 42 fonts e.g. for
#   ICRA IROS submission PDF compliance!
def truetype ():

  # Copied from triangle_sampling triangles_svm.py
  # For ICRA PDF font compliance. No Type 3 font (rasterized) allowed
  #   Ref: http://phyletica.org/matplotlib-fonts/
  # You can do this in code, or edit matplotlibrc. But problem with matplotlibrc
  #   is that it's permanent. When you export EPS using TrueType (42), Mac OS X
  #   cannot convert to PDF. So you won't be able to view the file you
  #   outputted! Better to do it in code therefore.
  #   >>> import matplotlib
  #   >>> print matplotlib.matplotlib_fname()
  #   Ref: http://matplotlib.1069221.n5.nabble.com/Location-matplotlibrc-file-on-my-Mac-td24960.html
  matplotlib.rcParams['pdf.fonttype'] = 42
  matplotlib.rcParams['ps.fonttype'] = 42


# Figure and axis with black background, white text.
# Call this after you've plotted with plot(), matshow(), etc.
# Copied from active_touch/src/fig_recog_acc.py
def black_background (ax=None, bg_color='black', fg_color='white'):

  if ax is None:
    ax = plt.gca ()

  # Set background color
  # Copied from active_touch/src/fig_recog_acc.py
  ax.set_axis_bgcolor (bg_color)
  fig = plt.gcf ()
  fig.set_facecolor (bg_color)
  fig.set_edgecolor ('none')

  # Set text to white
  # Ref: https://stackoverflow.com/questions/4761623/changing-the-color-of-the-axis-ticks-and-labels-for-a-plot-in-matplotlib
  ax.spines ['bottom'].set_color (fg_color)
  ax.spines ['top'].set_color (fg_color)
  ax.spines ['left'].set_color (fg_color)
  ax.spines ['right'].set_color (fg_color)

  ax.xaxis.label.set_color (fg_color)
  ax.tick_params (axis='x', colors=fg_color)

  ax.yaxis.label.set_color (fg_color)
  ax.tick_params (axis='y', colors=fg_color)


# Colorbar with black background
# Parameters:
#   colorbar: object returned from plt.colorbar()
def black_colorbar (colorbar, bg_color='black', fg_color='white'):

  # Ref: https://stackoverflow.com/questions/9662995/matplotlib-change-title-and-colorbar-text-and-tick-colors?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
  # Label and label color
  #colorbar.set_label('colorbar label', color=fg_color)
  # Edge color 
  colorbar.outline.set_edgecolor (fg_color)
  # Tick color
  colorbar.ax.yaxis.set_tick_params (color=fg_color)
  # Tick label color
  plt.setp (plt.getp (colorbar.ax.axes, 'yticklabels'), color=fg_color)


# Legend with black background
# Copied from active_touch/src/fig_recog_acc.py
# Parameters:
#   legend: object returned from plt.legend ()
def black_legend (legend, bg_color='black', fg_color='white'):

  # Set legend background color
  # Ref: https://matplotlib.org/examples/api/legend_demo.html
  legend.get_frame ().set_facecolor (bg_color)

  # Ref: https://stackoverflow.com/questions/18909696/how-to-change-the-text-colour-of-font-in-legend
  legend.get_frame ().set_edgecolor (fg_color)
  for text in legend.get_texts ():
    text.set_color (fg_color)



# Return a 4-tuple color (r, g, b, a) in matplotlib colormap.
# Parameters:
#   item_idx: Index of current item to plot. This number is used to interpolate
#     the colormap and get you the color for this item, given the total number
#     of items you have.
#   n_items: Set higher numbers when have more things to plot!
#   colormap_name: string from
#     http://matplotlib.org/examples/color/colormaps_reference.html
#     jet is the common standard.
#     gist_ncar has quicker color change than jet, good when you have a lot of
#       items to plot.
#     nipy_spectral is also quick changing, but darker. I used this for IROS.
#       Very pretty rich colors in a variety of range. Good for a small number
#       of curves.
#     Dark2 and Set1 are fast changing dark pastels
def mpl_color (item_idx, n_items, rand_color=False,
  colormap_name='nipy_spectral'):

  if rand_color:
    item_idx = np.random.randint (n_items)

  colormap = get_cmap (colormap_name, n_items)

  return colormap (item_idx)


# Ref: https://matplotlib.org/examples/pylab_examples/custom_cmap.html
# https://stackoverflow.com/questions/16834861/create-own-colormap-using-matplotlib-and-plot-color-scale
def custom_colormap ():

  cm_name = 'BlueRed'

  cdict = {
    'red':   ((0.0, 0.0, 0.0),
             (0.5, 0.0, 0.1),
             (1.0, 1.0, 1.0)),

    'green': ((0.0, 0.0, 0.0),
              (1.0, 0.0, 0.0)),

    'blue':  ((0.0, 0.0, 1.0),
              (0.5, 0.1, 0.0),
              (1.0, 0.0, 0.0))
  }

  cm = LinearSegmentedColormap (cm_name, cdict)
  plt.register_cmap (cmap=cm)

  return cm_name


# Ref: https://matplotlib.org/examples/pylab_examples/custom_cmap.html
# Colors taken from Lepora BBS 2013 paper :D I love those neon colors!
# Use this in Python prompt to see all the colors in a gradient:
#   >> from util.matplotlib_util import plot_colormap; plot_colormap()
def custom_colormap_neon ():

  cm_name = 'my_neon'

  neon_list = [
    #(  1,   0,  66),  # very dark indigo  # Untested
    (  4,   3,  66),  # dark indigo  # Untested
    ( 58,  57, 184),  # light indigo
    (  1,   0, 237),  # royal blue
    #(  1,  45, 253),  # royal blue lighter  # Untested
    (  0, 109, 252),  # royal blue lightest
    #(  0, 175, 251),  # blue like my Terminal comment color  # Untested
    ( 24, 184, 243),  # dark neon blue
    ( 52, 233, 240),  # light neon blue
    #(  2, 240, 253),  # cyan  # Untested
    ( 49, 255, 207),  # greenish cyan
    (105, 255, 145),  # green
    (149, 255, 113),  # neon green  # This adds more green. Comment this out for smaller green
    (174, 253,  84),  # chartreuse green
    (240, 253,  59),  # neon yellow
    (241, 253,  12),  # lemon yellow
    (252, 206,   6),  # orange yellow
    (252, 149,  11),  # neon orange
    (252, 140,  20),  # dark orange
  ]

  # Divide by 255
  neon_list = (np.array (neon_list) / 255.0).tolist ()

  cm = LinearSegmentedColormap.from_list (cm_name, neon_list)
  plt.register_cmap (cmap=cm)

  # Register a reverse too
  cm_name_r = cm_name + '_r'
  cm_r = LinearSegmentedColormap.from_list (cm_name_r, neon_list[::-1])
  plt.register_cmap (cmap=cm_r)

  return cm_name


# Ref: https://matplotlib.org/examples/color/colormaps_reference.html
def plot_color_gradients (cm_name):

  gradient = np.linspace(0, 1, 256)
  gradient = np.vstack((gradient, gradient))

  ax = plt.gca ()
  ax.imshow(gradient, aspect='auto', cmap=plt.get_cmap(cm_name))
  pos = list(ax.get_position().bounds)
  x_text = pos[0] - 0.01
  y_text = pos[1] + pos[3]/2.
  fig = plt.gcf ()
  fig.text(x_text, y_text, cm_name, va='center', ha='right', fontsize=10)

  # Turn off *all* ticks & spines, not just the ones with colormaps.
  ax.set_axis_off()


def plot_colormap ():

  cm_name = custom_colormap_neon ()

  plot_color_gradients (cm_name)

  plt.show ()



# Copied from triangle_sampling stats_hist_num_bins.py
def plot_line (xdata, ydata, title, xlbl, ylbl, out_name, color, lbl,
  dots=True, grid=True,
  stdev=None, do_save=True):

  #####
  # Plot
  #####

  if dots:
    plt.plot (xdata, ydata, 'o', markersize=5,
      markeredgewidth=0, color=color)
  hdl, = plt.plot (xdata, ydata, '-', linewidth=2, color=color, label=lbl)

  # Plot error bars
  #   http://matplotlib.org/1.2.1/examples/pylab_examples/errorbar_demo.html
  #   Plot error bar without line, fmt='':
  #   http://stackoverflow.com/questions/18498742/how-do-you-make-an-errorbar-plot-in-matplotlib-using-linestyle-none-in-rcparams
  if stdev:
    plt.errorbar (xdata, ydata, fmt='', capthick=2, yerr=stdev, color=color)

  if grid:
    #plt.grid (True, color='gray')
    plt.grid (True, color=[0.8, 0.8, 0.8])
  else:
    plt.grid (False)

  if title:
    plt.title (title)
  plt.xlabel (xlbl)
  plt.ylabel (ylbl)


  # Save to file
  if do_save:
    plt.savefig (out_name, bbox_inches='tight')
    print ('Plot saved to %s' % out_name)

  #plt.show ()

  # If you want legend, pass in a label for each thing you plot. Then legend
  #   will automatically show with just plt.legend().
  # Or you can pass in handles.
  # Ref: http://matplotlib.org/users/legend_guide.html
  #plt.legend (handles=[nn_hdl, svm_hdl], loc=0)

  return hdl


# This function is for debugging. It is used for viewing, not for inputting
#   the saved image into another program. It doesn't bother to remove white
#   edges.
# Colorbar is plotted to help debugging.
def mpl_matshow (prob, prob_img_name='', cmap_name='jet', fig_num=0):

  if fig_num == 0:
    fig = plt.figure (frameon=False)
  else:
    fig = plt.figure (fig_num, frameon=False)

  ax = plt.Axes (fig, [0., 0., 1., 1.])
  ax.set_axis_off ()
  fig.add_axes (ax)

  # Ref plt.get_cmap(): https://matplotlib.org/examples/color/colormaps_reference.html
  cax = ax.matshow (prob, aspect='auto', cmap=plt.get_cmap (cmap_name))
  # https://matplotlib.org/examples/pylab_examples/colorbar_tick_labelling_demo.html
  fig.colorbar (cax)
  if prob_img_name != '' and prob_img_name != '':
    fig.savefig (prob_img_name)
    print ('%sSaved figure to %s%s' % (
      ansi_colors.OKCYAN, prob_img_name, ansi_colors.ENDC))


# Call matshow(), without showing any border on the image, useful for saving
#   to image file.
# If don't want to save to file, pass in prob_img_name=None.
def mpl_matshow_noborder (prob, prob_img_name='', cmap_name='jet', fig_num=0):

  # If nothing specified, set to default
  if cmap_name == '':
    cmap_name = 'jet'

  # Save probabilistic heat map, with no white margin
  #   Ref: http://stackoverflow.com/questions/8218608/scipy-savefig-without-frames-axes-only-content
  #     Adjusting dpi and set_size_inches is the trick. Use dpi with savefig(),
  #     not bbox_inches='tight' (this will add a border!)

  if fig_num == 0:
    fig = plt.figure (frameon=False)
  else:
    fig = plt.figure (fig_num, frameon=False)
  plt.clf ()

  height = prob.shape [0]
  width = prob.shape [1]
  dpi = 72.0
  fig.set_size_inches (width / dpi, height / dpi)

  ax = plt.Axes (fig, [0., 0., 1., 1.])
  ax.set_axis_off ()
  fig.add_axes (ax)

  # Ref plt.get_cmap(): https://matplotlib.org/examples/color/colormaps_reference.html
  cax = ax.matshow (prob, aspect='auto', cmap=plt.get_cmap (cmap_name))
  if prob_img_name != '' and prob_img_name != '':
    fig.savefig (prob_img_name, dpi=dpi)
    print ('%sSaved figure to %s%s' % (
      ansi_colors.OKCYAN, prob_img_name, ansi_colors.ENDC))

  # Draw colorbar after image has been saved, so colorbar doesn't get saved
  #   into clean border-free image!
  # To see this, caller needs to call plt.show ()
  fig.colorbar (cax)

  fig.canvas.draw ()

  return fig, ax, dpi


# Visualize points in 3D axes in matplotlib
# Parameters:
#   pts: n x 3
#   vecs: n x 3
def mpl_visualize_3D_arrows (pts, vecs, fig_num=0):

  # https://matplotlib.org/mpl_toolkits/mplot3d/tutorial.html
  if fig_num == 0:
    fig = plt.figure ()
  else:
    fig = plt.figure (fig_num)
  ax = fig.add_subplot(111, projection='3d')

  # https://matplotlib.org/mpl_toolkits/mplot3d/tutorial.html#quiver
  ax.quiver (pts[:, 0], pts[:, 1], pts[:, 2],
    vecs[:, 0], vecs[:, 1], vecs[:, 2], length=0.01)

  minx = pts[:, 0].min ()
  maxx = pts[:, 0].max ()
  miny = pts[:, 1].min ()
  maxy = pts[:, 1].max ()
  minz = pts[:, 2].min ()
  maxz = pts[:, 2].max ()

  # axis equal for 3D
  # Ref: https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to
  # Create cubic bounding box to simulate equal aspect ratio
  max_range = np.max([maxx-minx, maxy-miny, maxz-minz])
  Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(maxx+minx)
  Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(maxy+miny)
  Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(maxz+minz)
  # Comment or uncomment following two lines to test the fake bounding box:
  for xb, yb, zb in zip(Xb, Yb, Zb):
    ax.plot([xb], [yb], [zb], 'w')

  plt.show (block=False)

  return fig, ax


#   init_frame: NumPy array of an image
def mpl_animate_init (init_frame, fig_num=0):

  #fig, _ = plt.subplots (1)
  if fig_num == 0:
    fig = plt.figure (frameon=False)
  else:
    fig = plt.figure (fig_num, frameon=False)

  # So that image can be saved without axes and borders
  height = init_frame.shape [0]
  width = init_frame.shape [1]
  dpi = 72.0
  fig.set_size_inches (width/dpi, height/dpi)

  ax = plt.Axes (fig, [0., 0., 1., 1.])
  ax.set_axis_off ()
  fig.add_axes (ax)

  if init_frame is None:
    print ('%sERROR in mpl_animate_init(): Image is None! Nothing drawn.%s' % (
      ansi_colors.FAIL, ansi_colors.ENDC))
    return fig, None

  # Not RGB image, use matshow()
  if init_frame.ndim < 3:
    img_obj = ax.imshow (init_frame, aspect='auto')

  # RGB image, use imshow()
  else:  # init_frame.ndim >= 3, here assume it == 3.
    img_obj = ax.imshow (init_frame, aspect='auto')

  plt.draw ()
  plt.show (block=False)

  #return fig.number, img_obj
  return fig, img_obj, dpi


# fig_num: Number of an existing matplotlib image. (If none exists yet,
#   you should call mpl_animate_init() first with the initial frame
#def mpl_animate_update (fig_num, img_obj, new_frame):
def mpl_animate_update (fig, img_obj, new_frame):

  #fig = plt.figure (fig_num)

  img_obj.set_data (new_frame)
  # TEMPORARY testing if plot changes to black
  #img_obj.set_data (np.zeros ((640, 480, 3), dtype=np.int16))

  fig.canvas.draw ()
  plt.show (block=False)



# Plot and save image
# If want to save image, pass in absolute path to out_name (optional)
def mpl_imshow_savefig (img, out_name='', cmap_name=None, alpha=1.0, fig_num=0,
  colorbar=False):

  if fig_num == 0:
    fig = plt.figure (frameon=False)
  else:
    fig = plt.figure (fig_num, frameon=False)
  height = img.shape [0]
  width = img.shape [1]
  dpi = 72.0
  fig.set_size_inches (width/dpi, height/dpi)

  ax = plt.Axes (fig, [0., 0., 1., 1.])
  ax.set_axis_off ()
  fig.add_axes (ax)

  cax = ax.imshow (img, aspect='auto', cmap=cmap_name, alpha=alpha,
    interpolation='nearest')
    # TEMPORARY For 1-channel images
    #clim=(np.min(img), np.max(img)))
  if out_name != '':
    fig.savefig (out_name, dpi=dpi)
    print ('%sSaved figure to %s%s' % (ansi_colors.OKCYAN, out_name,
      ansi_colors.ENDC))

  # Draw colorbar after image has been saved, so colorbar doesn't get saved
  #   into clean border-free image!
  # To see this, caller needs to call plt.show ()
  # TEMPORARY for debugging. Remove this for grasp_collection_pipeline.py! Else
  #   tactile dots won't overlay correctly on the RGB image!
  if colorbar:
    fig.colorbar (cax, extend='both')

  return fig.number

