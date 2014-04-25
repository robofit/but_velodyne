#ifndef __BUT_CALIBRATION_H__
#define __BUT_CALIBRATION_H__

#include <cstdlib>
#include <cstdio>
#include <iostream>

#include "opencv2/opencv.hpp"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/Similarity.h>

namespace but_calibration_camera_velodyne
{

typedef struct
{
  cv::Mat frame_rgb;
  cv::Mat frame_gray;
  cv::Mat P;
  ::pcl::PointCloud<Velodyne::Point> pc;
  std::string error;
  float x, y, z, rot_x, rot_y, rot_z;
} CalibrationInputs;

class Calibration6DoF
{
public:
  std::vector<float> DoF;
  float value; // NaN = wrong calibration

public:
  Calibration6DoF(float x, float y, float z, float x_r, float y_r, float z_r, float val)
  {
    set(x, y, z, x_r, y_r, z_r, val);
  }

  Calibration6DoF()
  {
    value = 0;
    DoF.resize(6, 0);
  }

  static Calibration6DoF wrong()
  {
    return Calibration6DoF(0, 0, 0, 0, 0, 0, NAN);
  }

  bool isGood()
  {
    return !isnan(value);
  }

  void set(float x, float y, float z, float x_r, float y_r, float z_r, float val)
  {
    value = val;

    DoF.clear();
    DoF.push_back(x);
    DoF.push_back(y);
    DoF.push_back(z);
    DoF.push_back(x_r);
    DoF.push_back(y_r);
    DoF.push_back(z_r);
  }

  bool operator <=(Calibration6DoF &other)
  {
    return this->value <= other.value;
  }

  void operator +=(Calibration6DoF &other)
  {
    this->value += other.value;
    ROS_ASSERT(this->DoF.size() == other.DoF.size());
    for (size_t i = 0; i < DoF.size(); i++)
    {
      this->DoF[i] += other.DoF[i];
    }
  }

  void operator /=(float div)
  {
    this->value /= div;
    for (size_t i = 0; i < DoF.size(); i++)
    {
      this->DoF[i] /= div;
    }
  }

  void print(void)
  {
    std::cout << "score: " << value << ";\t 6DoF: [" << DoF[0] << " " << DoF[1] << " " << DoF[2] << " " << DoF[3] << " "
        << DoF[4] << " " << DoF[5] << "]" << std::endl;
  }
};

class Calibration
{
public:
  static CalibrationInputs loadArgumets(int argc, char *argv[], bool DoF = false)
  {
    CalibrationInputs inputs;
    int expected_arguments = DoF ? 10 : 4;

    if (argc == expected_arguments)
    {
      inputs.frame_rgb = cv::imread(argv[1]);
      inputs.frame_gray = cv::imread(argv[1]);

      inputs.error = inputs.frame_rgb.data ? "" : "image not read";

      cv::FileStorage fs_P(argv[2], cv::FileStorage::READ);
      fs_P["P"] >> inputs.P;
      inputs.error = inputs.P.data ? "" : "projection matrix not read";
      fs_P.release();

      ::pcl::io::loadPCDFile(argv[3], inputs.pc);
      inputs.error = !inputs.pc.empty() ? "" : "velodyne points not read";

      if (DoF)
      {
        inputs.x = atof(argv[4]);
        inputs.y = atof(argv[5]);
        inputs.z = atof(argv[6]);
        inputs.rot_x = atof(argv[7]);
        inputs.rot_y = atof(argv[8]);
        inputs.rot_z = atof(argv[9]);
      }
    }
    else
    {
      inputs.error = "wrong number of arguments";
    }

    if (!inputs.error.empty())
    {
      perror(inputs.error.c_str());
      std::cerr << argv[0] << " <frame> <projection-matrix> <point-cloud>"
          << (DoF ? " <x> <y> <z> <rot-x> <rot-y> <rot-z>\n" : "\n");
      exit(1);
    }
    return inputs;
  }

  /**
   * Computes coarse calibration (only translation) from 2D - 3D correspondences.
   */
  static Calibration6DoF findTranslation(std::vector<cv::Point2f> image, std::vector<cv::Point3f> velodyne,
                                         cv::Mat projection, float radius2D, float radius3D)
  {
    std::vector<float> translation(3, 0);
    enum INDEX
    {
      X = 0, Y = 1, Z = 2
    };

    float focal_len = projection.at<float>(0, 0);

    // t_z:
    translation[INDEX::Z] = radius3D * focal_len / radius2D - velodyne.front().z;

    float principal_x = projection.at<float>(0, 2);
    float principal_y = projection.at<float>(1, 2);

    for (size_t i = 0; i < image.size(); i++)
    {
      // t_x:
      translation[INDEX::X] += (image[i].x - principal_x) * (velodyne[i].z + translation[INDEX::Z]) / focal_len
          - velodyne[i].x;
      // t_y:
      translation[INDEX::Y] += (image[i].y - principal_y) * (velodyne[i].z + translation[INDEX::Z]) / focal_len
          - velodyne[i].y;
    }
    translation[INDEX::X] /= image.size();
    translation[INDEX::Y] /= image.size();

    // no rotation and value of calibration
    return Calibration6DoF(translation[INDEX::X], translation[INDEX::Y], translation[INDEX::Z], 0, 0, 0, 0);
  }

  static void calibrationRefinement(Image::Image img, Velodyne::Velodyne scan, cv::Mat P, float x_rough, float y_rough,
                                    float z_rough, float max_translation, float max_rotation, unsigned steps,
                                    Calibration6DoF &best_calibration, Calibration6DoF &average)
  {
    scan.intensityByRangeDiff();
    scan = scan.threshold(0.05);

    img = Image::Image(img.computeIDTEdgeImage());

    float x_min = x_rough - max_translation;
    float y_min = y_rough - max_translation;
    float z_min = z_rough - max_translation;
    float x_rot_min = -max_rotation;
    float y_rot_min = -max_rotation;
    float z_rot_min = -max_rotation;

    float step_transl = max_translation * 2 / (steps - 1);
    float step_rot = max_rotation * 2 / (steps - 1);

    Velodyne::Velodyne transformed = scan.transform(x_rough, y_rough, z_rough, 0, 0, 0);
    float rough_val = Similarity::edgeSimilarity(img, transformed, P);

    best_calibration.set(x_rough, y_rough, z_rough, 0, 0, 0, rough_val);
    //cout << "rough:\t";
    //best_calibration.print();

    int counter = 0;

    float x = x_min;
    for (size_t xi = 0; xi < steps; xi++)
    {

      float y = y_min;
      for (size_t yi = 0; yi < steps; yi++)
      {

        float z = z_min;
        for (size_t zi = 0; zi < steps; zi++)
        {

          float x_r = x_rot_min;
          for (size_t x_ri = 0; x_ri < steps; x_ri++)
          {

            float y_r = y_rot_min;
            for (size_t y_ri = 0; y_ri < steps; y_ri++)
            {

              float z_r = z_rot_min;
              for (size_t z_ri = 0; z_ri < steps; z_ri++)
              {

                Velodyne::Velodyne transformed = scan.transform(x, y, z, x_r, y_r, z_r);
                float value = Similarity::edgeSimilarity(img, transformed, P);
                Calibration6DoF calibration(x, y, z, x_r, y_r, z_r, value);
                if (value > best_calibration.value)
                {
                  best_calibration.set(x, y, z, x_r, y_r, z_r, value);
                }

                if (value > rough_val)
                {
                  average += calibration;
                  counter++;
                }
                //cout << counter << ".\t";
                //calibration.print();

                z_r += step_rot;
              }
              y_r += step_rot;
            }
            x_r += step_rot;
          }
          z += step_transl;
        }
        y += step_transl;
      }
      x += step_transl;
    }
    average /= counter;
  }

};

};

#endif
