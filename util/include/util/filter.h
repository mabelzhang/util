// Mabel Zhang
// 13 Sep 2018
//
// Translated from ../src/util/filter.py.
//
// Simple filters for creating blobbed heat maps from images with sparse
//   non-zero pixels.
//


#include <opencv2/imgproc/imgproc.hpp>

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
//   gauss: kernel size of Gaussian filter, must be positive and odd.
// Ref filter tutorial https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/filter_2d/filter_2d.html
//   API https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html?highlight=filter2d#filter2d
void blob_filter (cv::Mat & img, cv::Mat & convolved, int expand=5, int gauss=3)
{
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
  // TODO: Figure out relationship between Gaussian size and sigma, what is best value
  cv::GaussianBlur (intermediate, convolved, cv::Size (2*gauss+1, 2*gauss+1),
    7, 7);
}

