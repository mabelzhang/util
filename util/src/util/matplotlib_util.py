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
import matplotlib.patches as patches

import numpy as np

#from util.ansi_colors import ansi_colors


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
# To save the figure with black background:
# Ref savefig() black background: https://stackoverflow.com/questions/4804005/matplotlib-figure-facecolor-background-color
#   plt.savefig (img_name, bbox_inches='tight',
#     facecolor=fig.get_facecolor (), edgecolor='none', transparent=True)
def black_background (ax=None, title_hdl=None, bg_color='black', fg_color='white'):

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

  if title_hdl:
    plt.setp (title_hdl, color=fg_color)


def black_3d_background (ax=None):

  if ax is None:
    ax = plt.gca ()

  # Ref: https://stackoverflow.com/questions/11448972/changing-the-background-color-of-the-axes-planes-of-a-matplotlib-3d-plot
  #ax.w_xaxis.set_pane_color ((0, 0, 0, 0))
  #ax.w_yaxis.set_pane_color ((0, 0, 0, 0))
  #ax.w_zaxis.set_pane_color ((0, 0, 0, 0))

  # Get rid of colored axes planes
  # First remove fill
  ax.xaxis.pane.fill = False
  ax.yaxis.pane.fill = False
  ax.zaxis.pane.fill = False

  # Now set color to white (or whatever is "invisible")
  ax.xaxis.pane.set_edgecolor((1.0, 1.0, 1.0, 1))
  ax.yaxis.pane.set_edgecolor((1.0, 1.0, 1.0, 1))
  ax.zaxis.pane.set_edgecolor((1.0, 1.0, 1.0, 1))

  # Ref https://stackoverflow.com/questions/17925545/adjusting-gridlines-on-a-3d-matplotlib-figure
  ax.w_xaxis._axinfo.update({'grid' : {'color': (0.2, 0.2, 0.2, 1)}})
  ax.w_yaxis._axinfo.update({'grid' : {'color': (0.2, 0.2, 0.2, 1)}})
  ax.w_zaxis._axinfo.update({'grid' : {'color': (0.2, 0.2, 0.2, 1)}})


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
#     Can be a float. In that case it is interpreted as a fraction in [0, 1],
#     seems like.
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
# To use this colormap in plotting:
#   cm_name = custom_colormap_neon ()
#   mpl_color (item_idx, n_items, colormap_name=cm_name)
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


# Matplotlib official color, which my version doesn't have. So manually
#   define it.
# Ref: https://matplotlib.org/examples/color/colormaps_reference.html
#   https://matplotlib.org/examples/pylab_examples/custom_cmap.html
def tab10_colormap ():

  tab10 = np.array ([
    (31, 119, 180),
    (255, 127, 14),
    (44, 160, 44),
    (214, 39, 40),
    (148, 103, 189),
    (140, 86, 75),
    (227, 119, 194),
    (127, 127, 127),
    (23, 190, 207),
    (188, 189, 34)
  ], dtype=np.float32)
  tab10 /= 255.0

  return tab10

  #cm_name = 'tab10'
  #return cm_name


# Ref: https://matplotlib.org/examples/color/colormaps_reference.html
# Test with:
'''
from matplotlib_util import plot_color_gradients
plot_color_gradients('jet')
from matplotlib import pyplot as plt
plt.show()
'''
def plot_color_gradients (cm_name):

  gradient = np.linspace(0, 1, 256)
  gradient = np.vstack((gradient, gradient))

  ax = plt.gca ()
  ax.imshow(gradient, aspect='auto', cmap=plt.get_cmap(cm_name))
  pos = list(ax.get_position().bounds)
  x_text = pos[0] - 0.01
  y_text = pos[1] + pos[3]/2.

  #fig = plt.gcf ()
  #fig.text(x_text, y_text, cm_name, va='center', ha='right', fontsize=10)

  # Turn off *all* ticks & spines, not just the ones with colormaps.
  ax.set_axis_off()

  plt.savefig ('color_gradient.eps', bbox_inches='tight')
  print ('Color gradient image saved to color_gradient.eps')


def plot_colormap ():

  cm_name = custom_colormap_neon ()

  plot_color_gradients (cm_name)

  plt.show ()



# Copied from triangle_sampling stats_hist_num_bins.py
# Parameters:
#   style: 'line' and 'bar' are implemented currently
#   dots: Add large dots to the data points, in addition to the line graph.
def plot_line (xdata, ydata, title, xlbl, ylbl, out_name, color, lbl,
  style='line', dots=True, grid=True,
  stdev=None, do_save=True, do_show=False, return_title_hdl=False):

  #####
  # Plot
  #####

  if dots:
    plt.plot (xdata, ydata, 'o', markersize=5, markeredgewidth=0, color=color)
  if style == 'line':
    hdl, = plt.plot (xdata, ydata, '-', linewidth=2, color=color, label=lbl)
  elif style == 'bar':
    hdl = plt.bar (xdata, ydata, align='center', color=color, label=lbl)

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
    title_hdl = plt.title (title)
  plt.xlabel (xlbl)
  plt.ylabel (ylbl)


  # Save to file
  if do_save:
    plt.savefig (out_name, bbox_inches='tight')
    print ('Plot saved to %s' % out_name)

  if do_show:
    plt.show ()

  # If you want legend, pass in a label for each thing you plot. Then legend
  #   will automatically show with just plt.legend().
  # Or you can pass in handles.
  # Ref: http://matplotlib.org/users/legend_guide.html
  #plt.legend (handles=[nn_hdl, svm_hdl], loc=0)

  if title != '' and return_title_hdl:
    return hdl, title_hdl
  else:
    return hdl


# Parameters:
#   ax: Current axis can be obtained by plt.gca()
#   rot_degs: Rotation in degrees. 'vertical' for 90.
def mpl_diagonal_xticks (ax, xticks, xticklabels, rot_degs=45):

  ax.set_xticks (xticks)
  ax.set_xticklabels (xticklabels, rotation=rot_degs)



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



# Draw text and arc for an angle defined by two rays
# Parameters:
#   r1, r2: Two rays defining the angle. Each is a (x, y) vector
#   origin: Location of angle
#   lbl: Text label for angle
#   algn: Text label alignment
#   dist: Distance of arc and text label from angle origin, in axis units
#   scatter: Whether to plot a dot at the position of text
#   arc: Whether to draw an arc representing angle
#   axes: Matplotlib axes object. Only used if arc==True.
# Example usage:
'''
import matplotlib.pyplot as plt
from matplotlib_util import draw_angle
fig = plt.figure(figsize=(10,8))
axes = fig.add_subplot(111)
plt.plot ([0,3], [0,0])
plt.plot ([0,2], [0,1])
ax, ay, arc = draw_angle ([2,1],[3,0],[0,0],'angle','left',1,True,True,axes)
plt.show ()
'''
def draw_angle (r1, r2, origin, lbl, algn, dist, scatter=True, arc=False,
  axes=None):

  r1 /= np.linalg.norm (r1)
  r2 /= np.linalg.norm (r2)

  mid_ray = 0.5 * (r1 + r2)
  mid_ray /= np.linalg.norm (mid_ray)

  # Compute angle of each ray from the x-axis
  a1 = np.arctan2 (r1[1], r1[0])
  a2 = np.arctan2 (r2[1], r2[0])
  if a1 <= a2:
    astart = a1
    aend = a2
  else:
    astart = a2
    aend = a1
  astart = astart / np.pi * 180.0
  aend = aend / np.pi * 180.0
  print ('astart %g' % astart)
  print ('aend %g' % aend)

  ax, ay = origin + mid_ray * dist
  arc = patches.Arc (origin, 2*dist, 2*dist, 0, theta1=astart, theta2=aend)

  if scatter:
    plt.scatter (ax, ay)
  if arc:
    if axes != None:
      axes.add_patch (arc)
  if len (lbl) > 0:
    plt.text (ax, ay, lbl, horizontalalignment=algn)

  return (ax, ay, arc)

