// Mabel Zhang
// 13 Sep 2018
//
// Translated from ../src/util/filter.py.
//
// Simple filters for creating blobbed heat maps from images with sparse
//   non-zero pixels.
//

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>

// Custom
#include <util/ansi_colors.h>


// Translate to C++ if need this, might need to rewrite a whole filter fn,
//   since there doesn't seem to be an OpenCV equivalent of
//   scipy.ndimage.generic_filter(), which takes a function. OpenCV only takes
//   a kernel matrix, not a function.
//   dilate() source https://github.com/opencv/opencv/blob/master/modules/imgproc/src/morph.cpp#L1897
// Emphasizes extrema pixels. Whereas max filter zeros out negative values,
//   this increases the magnitude.
//def extremum_filter (buff):
  // Take absolute value, then find max element
  //max_i = np.argmax (np.abs (buff))
  // Ref access using linear index: https://stackoverflow.com/questions/15230179/how-to-get-the-linear-index-for-a-numpy-array-sub2ind
  //return buff.ravel () [max_i]

// Apply max and Gaussian filter to an image, to make any individual non-zero
//   pixels into blobs.
// Parameters:
//   expand: width of max filter, in pixels
//   gauss_size: kernel size of Gaussian filter, must be positive and odd.
//   gauss_sigma: pass in 0 to let OpenCV calculate sigma from size
// Ref filter tutorial https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/filter_2d/filter_2d.html
//   API https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html?highlight=filter2d#filter2d
void blob_filter (cv::Mat & img, cv::Mat & convolved, int expand=5,
  int gauss_size=3, float gauss_sigma=3)
{
  if (gauss_size % 2 != 1 || gauss_size <= 0)
  {
    fprintf (stderr, "%sERROR: blob_filter(): gauss_size is required by OpenCV to be positive and odd. Condition not met. Change your gauss_size. OpenCV will have runtime error.%s\n", FAIL, ENDC);
  }

  // Max filter.
  // NOTE: If input img contains negative values, and you want to take max 
  //   magnitude including negative values, this won't do! Need extremum
  //   filter.
  // Ref dilate() usage sample https://github.com/opencv/opencv/blob/master/samples/cpp/morphology2.cpp
  //   API https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html?highlight=filter2d#dilate
  cv::Mat kernel = cv::getStructuringElement (cv::MORPH_ELLIPSE,
    cv::Size (expand, expand));
  cv::Mat intermediate;
  cv::dilate (img, intermediate, kernel);

  // API https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html?highlight=filter2d#gaussianblur
  cv::GaussianBlur (intermediate, convolved,
    //cv::Size (2*gauss_size+1, 2*gauss_size+1), gauss_sigma, gauss_sigma);
    //cv::Size (gauss_size, gauss_size), 0, 0);
    // Pass in 0 to let OpenCV calculate sigma from size
    cv::Size (gauss_size, gauss_size), gauss_sigma, gauss_sigma);
}

