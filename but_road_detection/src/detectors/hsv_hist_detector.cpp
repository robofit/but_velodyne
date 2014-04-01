/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Vita Beran (beranv@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 28/06/2013
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "but_road_detection/detectors/hsv_hist_detector.h"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace but_road_detection;
using namespace std;
using namespace cv;


HSVHistFeature::HSVHistFeature(int hbins, int sbins)
{
  hranges_[0] = 0;
  hranges_[1] = 180;
  sranges_[0] = 0;
  sranges_[1] = 256;
  ranges_[0] = hranges_;
  ranges_[1] = sranges_;
  channels_[0] = 0;
  channels_[1] = 1;
  init(hbins, sbins);
}


void HSVHistFeature::init(int hbins, int sbins)
{
  hbins_ = hbins;
  sbins_ = sbins;
  histSize_[0] = hbins_;
  histSize_[1] = sbins_;
  cout << "HSVHist Feature Extractor init: Hue bins [" << hbins << "] Saturation bins [" << sbins << "]" << endl;
}

int HSVHistFeature::length() const
{
  return hbins_ * sbins_;
}

bool HSVHistFeature::extract(cv::Mat& hsv, cv::Mat& feature_vector) const
{
  if (hsv.empty())
    return false;

  MatND h;
  calcHist(&hsv, 1, channels_, Mat(), h, 2, histSize_, const_cast<const float **>(ranges_), true, false);

  feature_vector = h.reshape(1, 1).clone();

  //Mat rgb, tmp;
  //cvtColor( hsv, rgb, CV_HSV2BGR );
  //resize( rgb, tmp, Size(), 3, 3, CV_INTER_NN );
  //imshow( "RGB", tmp );
  //waitKey(0);

  return true;
}


bool HSVHistFeature::extract(cv::Mat& hsv, cv::Rect roi, cv::Mat& feature_vector) const
{
  if (!ROIinMat(roi, hsv))
    return false;

  Mat sub_hsv = hsv(roi);
  bool ret = extract(sub_hsv, feature_vector);

  return ret;
}


bool HSVHistFeature::extract(cv::Mat& hsv, int wnd_size, int wnd_step, cv::Mat& feature_vector) const
{
  if (hsv.empty())
  {
    cerr << "HSVHistFeature::extract: input HSV image empty." << endl;
    return false;
  }

  int rows = cvFloor((hsv.rows - wnd_size) / wnd_step);
  int cols = cvFloor((hsv.cols - wnd_size) / wnd_step);

  if (rows < 0 || cols < 0)
    return false;

  // result values
  feature_vector = Mat(rows * cols, length(), CV_32FC1);
  MatND h;

  // compute histogram features from input HSV image
  for (int r = 0; r < rows; ++r)
  {
    for (int c = 0; c < cols; ++c)
    {
      Rect roi(c * wnd_step, r * wnd_step, wnd_size, wnd_size);

      Mat mrow, tmp;
      extract(hsv, roi, mrow);
      tmp = feature_vector.row(r * cols + c);
      mrow.copyTo(tmp);
    }
  }

  return true;
}


// pouzit Feature::extract v detectoru ...





bool AnnotMeta::loadCSV(const std::string& csv_filename, char separator)
{
  std::ifstream file(csv_filename.c_str(), ifstream::in);

  cout << "Annotations loading from " << csv_filename << " ... ";

  if (!file)
  {
    cerr << "No valid input file was given, please check the given filename." << endl;
    return false;
  }

  string line, term;

  while (getline(file, line))
  {

    iROI_ptr roi(new iROI);

    stringstream liness(line);
    getline(liness, roi->fname_, separator);

    getline(liness, term, separator);
    istringstream(term) >> roi->cid_;

    getline(liness, term, separator);
    istringstream(term) >> roi->roi_.x;

    getline(liness, term, separator);
    istringstream(term) >> roi->roi_.y;

    getline(liness, term, separator);
    istringstream(term) >> roi->roi_.width;

    getline(liness, term, separator);
    istringstream(term) >> roi->roi_.height;

    irois_[roi->fname_].push_back(roi);

    Arois_.push_back(roi);

    if (roi->cid_ > 0)
      Prois_.push_back(roi);
    else
      Nrois_.push_back(roi);
  }
  cout << "done P[" << Prois_.size() << "] N[" << Nrois_.size() << "]" << endl;

  return true;
}


bool AnnotMeta::exportROIs(const std::string& DataDir, const std::string& PositiveDir, const std::string& NegativeDir)
{
  cout << "Export ROIs ..." << endl;

  std::map<std::string, std::vector<iROI_ptr> >::iterator it = irois_.begin();
  int np = 0, nn = 0;
  char buff[1024];

  // for each image
  for (; it != irois_.end(); ++it)
  {
    // load image
    Mat img = imread(DataDir + (*it).first);

    if (img.empty())
    {
      cerr << "\tFailed open image: " << DataDir + (*it).first << endl;
      continue;
    }

    // for each roi in the image
    for (size_t i = 0; i < (*it).second.size(); ++i)
    {
      iROI_ptr roi = (*it).second[i];

      if (!ROIinMat(roi->roi_, img))
      {
        cerr << "\tROI out of image: " << roi->fname_ << " classID: " << roi->cid_
             << " (" << roi->roi_.x << "," << roi->roi_.y << "," << roi->roi_.width << "," << roi->roi_.height << ")" << endl;
        continue;
      }

      // prepare output path and filename
      if (roi->cid_ > 0)
        sprintf(buff, "%s/pos_%08d.png", PositiveDir.c_str(), np++);
      else
        sprintf(buff, "%s/neg_%08d.png", NegativeDir.c_str(), nn++);

      // store roi subimage
      imwrite(buff, img(roi->roi_));
    }
  }

  cout << "Export ROIs done - positives: " << np << " negatives: " << nn << endl;

  return true;
}


string AnnotMeta::getImgFilename(int i)
{
  if (i < 0 || i >= (int)irois_.size())
    return false;

  std::map<std::string, std::vector<iROI_ptr> >::iterator it = irois_.begin();
  for (int k = 0; k < i; ++k, ++it) {}

  return (*it).first;
}


void AnnotMeta::renderRois(cv::Mat& img, int i)
{
  if (i < 0 || i >= (int)irois_.size())
    return;

  std::map<std::string, std::vector<iROI_ptr> >::iterator it = irois_.begin();
  for (int k = 0; k < i; ++k, ++it) {}

  // for each roi in the image
  for (size_t i = 0; i < (*it).second.size(); ++i)
  {
    iROI_ptr roi = (*it).second[i];

    if (ROIinMat(roi->roi_, img))
    {
      if (roi->cid_ > 0)
        rectangle(img, roi->roi_, CV_RGB(255, 0, 0), 2);
      else
        rectangle(img, roi->roi_, CV_RGB(0, 255, 0), 2);
    }
  }

  return;
}


bool HSVHistTrainData::extract(int wnd_size, int wnd_step, const std::string& DataDir, AnnotMeta& annData)
{
  cout << "Features extraction ... " << endl;

  // extract features using annotated data
  vector<Mat> hsv_ftrs;
  vector<int> cids;   // class IDs
  int ftr_cnt = 0;

  string imgName = "";
  Mat hsv;
  for (size_t i = 0; i < annData.Arois_.size(); ++i)
  {
    iROI_ptr roi = annData.Arois_[i];

    if (imgName != roi->fname_)
    {
      try
      {
        imgName = roi->fname_;
        Mat img = imread(DataDir + imgName);
        if (img.empty())
          throw 1;
        cvtColor(img, hsv, CV_BGR2HSV);
      }
      catch (...)
      {
        cerr << "\tFailed open image: " << DataDir + imgName << endl;
        continue;
      }
    }

    if (!ROIinMat(roi->roi_, hsv))
      continue;

    // extract features
    Mat tmp;
    Mat hsv_roi = hsv(roi->roi_);
    featureExtractor_.extract(hsv_roi, wnd_size, wnd_step, tmp);

    if (!tmp.empty())
    {
      hsv_ftrs.push_back(tmp);
      cids.push_back(roi->cid_);
      ftr_cnt += tmp.rows;
    }

    //cout << "\tROI[" << i << "] features: " << tmp.rows << endl;
  }

  if (hsv_ftrs.empty())
    return false;

  // create one big data matrix
  features_ = Mat(ftr_cnt, featureExtractor_.length(), hsv_ftrs[0].type());
  labels_   = Mat(ftr_cnt, 1, CV_32FC1);
  int row = 0;
  for (size_t i = 0; i < hsv_ftrs.size(); ++i)
  {
    Mat c(hsv_ftrs[i].rows, 1, CV_32FC1, Scalar(cids[i]));

    Mat tmp;
    tmp = features_.rowRange(row, row + hsv_ftrs[i].rows);
    hsv_ftrs[i].copyTo(tmp);
    tmp = labels_.rowRange(row, row + hsv_ftrs[i].rows);
    c.copyTo(tmp);

    row += hsv_ftrs[i].rows;
  }

  cout << "Features extracted: " << features_.rows << endl;

  return true;
}


bool HSVHistTrainData::read(const std::string& filename)
{
  // store all data
  FileStorage fs;
  fs.open(filename, FileStorage::READ);

  if (!fs.isOpened())
  {
    cerr << "Failed to open: " << filename << endl;
    return false;
  }

  fs["labels"] >> labels_;
  fs["features"] >> features_;

  fs.release();

  cout << "HSVHistDetectorTrain::readFeatures: loaded " << features_.rows << " features from: " << filename << endl;

  return true;
}


bool HSVHistTrainData::write(const std::string& filename)
{
  // store all data
  FileStorage fs;
  fs.open(filename, FileStorage::WRITE);

  if (!fs.isOpened())
  {
    cerr << "Failed to open: " << filename << endl;
    return false;
  }

  fs << "labels" << labels_;
  fs << "features" << features_;

  fs.release();

  cout << "HSVHistDetectorTrain::writeFeatures: saved to: " << filename << endl;

  return true;
}




HSVHistDetector::HSVHistDetector(double hit, double miss, int hbins, int sbins, int wnd_size, int wnd_step)
{
  init(hit, miss, hbins, sbins, wnd_size, wnd_step);
  hsvftr_.init(hbins, sbins);
}

HSVHistDetector::~HSVHistDetector()
{
}


void HSVHistDetector::init(double hit, double miss, int hbins, int sbins, int wnd_size, int wnd_step)
{
  wnd_size_ = wnd_size;
  wnd_step_ = wnd_step;
  prob_hit_ = hit;
  prob_miss_ = miss;

  cout << "HSVHist Detector init: P hit [" << prob_hit_ << "], P miss [" << prob_miss_ << "], window size [" << wnd_size_ << "], window step [" << wnd_step_ << "]" << endl;

  hsvftr_.init(hbins, sbins);
}


void HSVHistDetector::setWnd(int wnd_size, int wnd_step)
{
  wnd_size_ = wnd_size;
  wnd_step_ = wnd_step;

  cout << "HSVHist Detector set Window: size [" << wnd_size_ << "], step [" << wnd_step_ << "]" << endl;
}


// extract more HSV image features using flowing window
//bool HSVHistDetector::detect( cv::Mat& hsv, int wnd_size, int wnd_step, cv::Mat& probability )
bool HSVHistDetector::detect(cv::Mat& hsv, cv::Mat& probability)
{
  if (hsv.empty())
    return false;

  Mat fvec;
  hsvftr_.extract(hsv, wnd_size_, wnd_step_, fvec);

  if (fvec.empty())
    return false;

  int rows = cvFloor((hsv.rows - wnd_size_) / wnd_step_);
  int cols = cvFloor((hsv.cols - wnd_size_) / wnd_step_);
  Mat res(rows, cols, CV_32FC1);

  for (int i = 0; i < fvec.rows; ++i)
  {
    float dist = svm_.predict(fvec.row(i), true);
    res.at<float>(i / cols, i % cols) = (dist > 0 ? prob_miss_ : prob_hit_);
    //res.at<float>(i/cols, i%cols) = dist;
  }

  probability = Mat(hsv.size(), res.type(), Scalar(0));
  int off = cvFloor(wnd_size_ / 2);
  Mat tmp;
  resize(res, tmp, Size(), wnd_step_, wnd_step_, CV_INTER_NN);
  Mat tmp2 = probability(Rect(off, off, tmp.cols, tmp.rows));
  tmp.copyTo(tmp2);

  //cout << res << endl;

  return true;
}


bool HSVHistDetector::predict(const cv::Mat& data, cv::Mat& result)
{
  if (data.empty())
    return false;

  result = Mat(data.rows, 1, CV_32FC1);

  for (int i = 0; i < data.rows; ++i)
  {
    float dist = svm_.predict(data.row(i), false);
    result.at<float>(i, 0) = dist;
  }

  return true;
}


bool HSVHistDetector::eval(const cv::Mat& data, const cv::Mat& labels, float * precision)
{
  Mat result;

  if (!predict(data, result) || result.rows != labels.rows)
    return false;

  int n = 0;
  for (int i = 0; i < data.rows; ++i)
  {
    n += (result.at<float>(i, 0) == labels.at<float>(i, 0) ? 1 : 0);
    //cout << "GT Class[" << i << "]: " << labels.at<float>(i,0) << " Predict: " << result.at<float>(i,0) << endl;
  }

  float res = n * (data.rows > 0 ? 1. / data.rows : 0);

  if (precision != 0)
    *precision = res;

  //cout << "Size: rows:" << results.rows << " cols:" << results.cols << endl;
  //cout << results(Rect(0,0,1,10)) << endl;

  return true;
}


bool HSVHistDetector::train(const cv::Mat& data, const cv::Mat& labels, int kernel_type)
{
  // Set up SVM's parameters
  CvSVMParams params;
  params.svm_type    = CvSVM::C_SVC;
  params.kernel_type = kernel_type;
  params.degree = 0;
  params.gamma = 1;
  params.C  = 1;
  params.nu = 0;
  params.p  = 0;
  params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
  //params.term_crit = cvTermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, FLT_EPSILON );

  cout << "SVM training ... ";
  cout.flush();

  //svm_.train( data, labels, Mat(), Mat(), params );
  svm_.train_auto(data, labels, Mat(), Mat(), params,
                  10,     // k-fold
                  CvSVM::get_default_grid(CvSVM::C),
                  CvSVM::get_default_grid(CvSVM::GAMMA),
                  CvSVM::get_default_grid(CvSVM::P),
                  CvSVM::get_default_grid(CvSVM::NU),
                  CvSVM::get_default_grid(CvSVM::COEF),
                  CvSVM::get_default_grid(CvSVM::DEGREE),
                  false);     // balanced


  cout << "done (#vec:" << svm_.get_support_vector_count()
       <<  ", #var:" << svm_.get_var_count() << ")" << endl;

  return true;
}


bool HSVHistDetector::read(const std::string& filename)
{
  cout << "HSVHist Detector loading from " << filename << endl;

  try
  {

    // store all data
    FileStorage fs;
    fs.open(filename, FileStorage::READ);

    if (!fs.isOpened())
    {
      cerr << "Failed to open: " << filename << endl;
      return false;
    }

    int hbins, sbins;
    fs["hbins"] >> hbins;
    fs["sbins"] >> sbins;

    int wnd_size, wnd_step;
    fs["wnd_size"] >> wnd_size;
    fs["wnd_step"] >> wnd_step;

    init(prob_hit_, prob_miss_, hbins, sbins, wnd_size, wnd_step);

    FileNode fn = fs["SVM_MODEL"];
    svm_.read(*fs, *fn);

    fs.release();
  }
  catch (...)
  {
    cout << "HSVHist Detector FAILED to open from " << filename << endl;
    return false;
  }

  cout << "SVM model (#vec:" << svm_.get_support_vector_count()
       << ", #var:" << svm_.get_var_count() << ")" << endl;

  return true;
}


bool HSVHistDetector::write(const std::string& filename)
{
  // store all data
  FileStorage fs;
  fs.open(filename, FileStorage::WRITE);

  if (!fs.isOpened())
  {
    cerr << "Failed to open: " << filename << endl;
    return false;
  }

  fs << "hbins" << hsvftr_.hbins();
  fs << "sbins" << hsvftr_.sbins();
  fs << "wnd_size" << wnd_size_;
  fs << "wnd_step" << wnd_step_;

  svm_.write(*fs, "SVM_MODEL");

  fs.release();

  cout << "HSVHist Detector saved to " << filename << endl;

  return true;
}





