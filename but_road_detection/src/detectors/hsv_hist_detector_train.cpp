/**
 * Developed by dcgm-robotics@FIT group
 * Author: Vita Beran (beranv@fit.vutbr.cz)
 * Date: 20.08.2013 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 *
 *------------------------------------------------------------------------------
 */


#include "but_road_detection/detectors/hsv_hist_detector.h"

#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace but_road_detection;
using namespace std;
using namespace cv;

/*
 * ./hsv_hist_detector_train 17 17 45 15 /home/beranv/data/ hsv_17_17_45_rbf.yaml annotations.csv "rbf train eval browse"
 */

int main(int argc, char** argv)
{
  int hbin     = 17;
  int sbin     = 17;
  int wnd_size = 45;
  int wnd_step = 15;
  int kernel_type = CvSVM::LINEAR;
  string datasetDir     = "/home/beranv/data/";
  string model_filename = "modelHsvHist_17_17_45_15_rbf.yaml";
  string annot_filename = "annotations.csv";
  string cmds = "rbf train eval";     // rbf, train, eval, export_rois, browse

  cout << "HSV Detector Training" << endl;

  if (argc >= 8)
  {
    hbin = atoi(argv[1]);
    sbin = atoi(argv[2]);
    wnd_size = atoi(argv[3]);
    wnd_step = atoi(argv[4]);
    datasetDir = string(argv[5]);
    model_filename = string(argv[6]);
    annot_filename = string(argv[7]);
    if (argc >= 9) cmds = string(argv[8]);
  }
  else
  {
    cerr << "params: hbins sbins wnd_size wnd_step model_name datasetDir [commands]" << endl;
    return -1;
  }

  if (cmds.find("rbf") != string::npos)
    kernel_type = CvSVM::RBF;


  cout << "Parameters:" << endl;
  cout << "\tHue Bins:        " << hbin << endl;
  cout << "\tSaturation Bins: " << sbin << endl;
  cout << "\tKernel Type:     " << (kernel_type == CvSVM::RBF ? "RBF" : "LINEAR") << endl;
  cout << "\tWindow Size:     " << wnd_size << endl;
  cout << "\tWindow Step:     " << wnd_step << endl;
  cout << "\tDataset Path:    " << datasetDir << endl;
  cout << "\tModel Filename:  " << model_filename << endl;
  cout << "\tAnnot. Filename: " << annot_filename << endl;
  cout << "\tConfig Commands: " << cmds << endl;


  //////////////////////////////////////////////////////////////
  AnnotMeta annData;
  annData.loadCSV(datasetDir + annot_filename, ';');

  // export ROIs if needed
  if (cmds.find("export_rois") != string::npos)
  {
    annData.exportROIs(
      datasetDir,
      datasetDir + "positives/",
      datasetDir + "negatives/");
  }


  //////////////////////////////////////////////////////////////
  HSVHistDetector hsvDetector(0.7, 0.3, hbin, sbin, wnd_size, wnd_step);

  //////////////////////////////////////////////////////////////
  HSVHistTrainData hsvHistTrainData(hsvDetector.featureExtractor());
  if (cmds.find("train") != string::npos ||
      cmds.find("eval")  != string::npos)
  {
    // get training data
    hsvHistTrainData.extract(wnd_size, wnd_step, datasetDir, annData);
  }


  //////////////////////////////////////////////////////////////
  // Model type
  if (cmds.find("train") != string::npos)
  {
    hsvDetector.train(hsvHistTrainData.features(), hsvHistTrainData.labels(), kernel_type);
    hsvDetector.write(datasetDir + model_filename);
  }
  else
  {
    hsvDetector.read(datasetDir + model_filename);
    hsvDetector.setWnd(wnd_size, wnd_step);          // overwrite params stored during training
  }

  if (hsvDetector.empty())
  {
    cerr << "Error: No Model available." << endl;
    return -1;
  }

  //////////////////////////////////////////////////////////////
  // eval detector on training data
  if (cmds.find("eval") != string::npos)
  {
    float prob = 0;
    hsvDetector.eval(hsvHistTrainData.features(), hsvHistTrainData.labels(), &prob);
    cout << "Evaluation result \t " << model_filename << "\t " << prob << endl;
  }


  //////////////////////////////////////////////////////////////
  // manual check
  if (cmds.find("browse") != string::npos && annData.size() > 0)
  {
    int no = 0;
    while (1)
    {
      Mat img, hsv;
      Mat prob;
      string fn;

      try
      {

        fn = annData.getImgFilename(no);
        if (fn.empty()) throw 1;

        cout << "Load image: " << fn << endl;
        img = imread(datasetDir + fn);
        if (img.empty()) throw 2;

        cvtColor(img, hsv, CV_BGR2HSV);
        hsvDetector.detect(hsv, prob);

        Mat mask;
        prob.convertTo(mask, CV_8UC1);
        img *= 0.7;
        add(img, Scalar(255, -30, -30), img, mask);
        annData.renderRois(img, no);

        imshow("Input image", img);
      }
      catch (int e)
      {
        switch (e)
        {
        case 1:
          cerr << "Empty image filename." << endl;
          break;
        case 2:
          cerr << "Image reading failed." << endl;
          break;
        }
      }

      char key = waitKey(0);
      if (key == 'q' || key == 27)
        break;
      switch (key)
      {
      case 'z' :
        no = 0;
        break;                 // go to first image
      case 'n' :
        no = MAX(0, annData.size() - 1);
        break;       // go to last image
      case 'x' :
        no = MAX(no - 10, 0);
        break;           // -10
      case 'b' :
        no = MIN(no + 10, annData.size());
        break;     // +10
      case 'c' :
        no = MAX(no - 1, 0);
        break;            // -1
      case 'v' :
        no = MIN(no + 1, annData.size());
        break;      // +1
      }
    }
  }

  return 0;
}


