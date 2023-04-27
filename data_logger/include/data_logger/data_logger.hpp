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
private:
};

class ImageLogger final : BaseLogger {
public:
ImageLogger();
private:
};

class CloudLogger final : BaseLogger {
public:
CloudLogger();
private:
};

}