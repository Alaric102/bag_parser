#pragma once

#include <string>
#include <fstream>
#include <experimental/filesystem>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/emitter.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace data_logger{

namespace fs = std::experimental::filesystem;

struct CameraParameters{
    cv::Mat camera_mat, dist_mat, proj_mat;
};

class BaseLogger{
public:
BaseLogger();
protected:
fs::path output_path_;
size_t data_counter = 0;
};

class ImageLogger final : public BaseLogger {
public:
ImageLogger();
bool save_image(const cv::Mat& image);
bool save_cinfo(const CameraParameters& params);
private:
};

class CloudLogger final : public BaseLogger {
public:
CloudLogger();
private:
};

}