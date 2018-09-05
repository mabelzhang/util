#!/usr/bin/env python

# Mabel Zhang
# 11 Jan 2018
#
# Refactored from interactive_image_util.py
#


import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

# Pillow
from PIL import Image

# For rgb2gray()
from skimage import color
from skimage import io
from skimage.io import imread, imsave

# For show_image()
from matplotlib import pyplot as plt


# Convert from sensor_msgs/Image to NumPy array, using cv_bridge
# Dependencies: CvBridge
# Parameters:
#   nan_to_num: "Replace nan with zero and inf with finite numbers. Returns an
#     array or scalar replacing Not a Number (NaN) with zero, (positive)
#     infinity with a very large number and negative infinity with a very small
#     (or negative) number."
def np_from_image (img, nan_to_num=False):

  # Ref: http://answers.ros.org/question/64318/how-do-i-convert-an-ros-image-into-a-numpy-array/
  #   http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
  bridge = CvBridge ()
  try:

    #print ('step: %d' % img.step)
    #print ('width: %d' % img.width)

    # Ref encoding strings e.g. rgb8, 32fc1, are defined here:
    #   http://docs.ros.org/jade/api/sensor_msgs/html/image__encodings_8h_source.html 
    #   sensor_msgs/Image encoding field http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
    # RGB
    #   step is full row length in bytes. width is number of columns in a row.
    #   RGB is 1 byte per color (8 bits, 0 to 255), 3 colors per pixel
    if img.step / img.width == 3:
      img_cv = bridge.imgmsg_to_cv2 (img, 'rgb8')
    # Depth. 4 bytes (float) per pixel
    elif img.step / img.width == 4:
      img_cv = bridge.imgmsg_to_cv2 (img, '32FC1')

    if nan_to_num:
      img_cv = np.nan_to_num (img_cv)
      # This only works in NumPy 1.13, copy parameter is new
      #np.nan_to_num (img_cv, copy=False)

    return np.asarray (img_cv)

  except CvBridgeError as e:
    print ('ERROR in get_img_np() converting sensor_msgs/Image to cv.Mat: %s' % e)
    return None


# Copied from my make_movie.py load_eps_to_numpy
# Useful for EPS images
# Dependencies: PIL
# Uses PIL to load image from file. Convert to NumPy array.
# Parameters:
#   img_name: Full path to image file. File can be any image format
#     supported by Pillow.
#   sequence: Pass in True if this is a part of a sequence that you will
#     put into a video. If just loading individual images, pass in False.
def np_from_eps (self, img_name, sequence=True):

  # Ref Pillow Image load EPS formats: http://pillow.readthedocs.org/en/latest/handbook/image-file-formats.html
  # Ref Pillow Image loading tutorial: http://pillow.readthedocs.org/en/latest/handbook/tutorial.html
  img = Image.open (img_name)

  # Ref convert Pillow Image into NumPy array: http://stackoverflow.com/questions/1109422/getting-list-of-pixel-values-from-pil
  img_data = np.asarray (img)
  #print (np.shape (img_data))

  if sequence:
    self.imgs.append (img_data)
   
    height, width, _ = np.shape (img_data)
    if height > self.max_h:
      self.max_h = height
    if width > self.max_w:
      self.max_w = width

  # Return the NumPy array of the image
  return img_data


# Read depth (e.g. PNG) image file to numpy array
def np_from_depth (img_path):

  # io.imread does not produce raw depth values.
  # cv2.imread does.
  return cv2.imread (img_path)



# Parameters:
#   im_np: Image represented by a NumPy matrix
def show_image (im_np, title=''):

  #plt.figure ()
  plt.imshow (im_np)#, cmap=colormap)
  plt.colorbar ()
  plt.title (title)
  plt.show (block=False)


def matshow_image (im_np, title=''):

  #plt.figure ()
  plt.matshow (im_np)#, cmap=colormap)
  plt.colorbar ()
  plt.title (title)
  plt.show (block=False)


# Copied from ./image_scale_depth.py
# Not tested in this file yet
def rgb2gray (in_name, out_name, save_img=False, colormap=plt.cm.binary):

  # Both lines work
  # Ref: https://stackoverflow.com/questions/12201577/how-can-i-convert-an-rgb-image-into-grayscale-in-python
  img_out = color.rgb2gray (io.imread (in_name))
  #img_out = io.imread (in_name, as_grey=True)

  if save_img:
    imshow_and_save (img_out, out_name, colormap)

  #plt.imshow (img_out, cmap=colormap)
  #plt.show ()


