#include "data_logger/data_logger.hpp"

#include <string>
#include <string_view>
#include <experimental/filesystem>
#include <fstream>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/emitter.h>

namespace data_logger {

static inline YAML::Emitter& operator<<(YAML::Emitter& emitter, const cv::Mat& mat) {
    emitter << YAML::Flow;
    emitter << YAML::BeginSeq;
    for (auto it = mat.begin<double>(); it != mat.end<double>(); ++it ) {
        emitter << *it;
    }
    return emitter << YAML::EndSeq;;
}

static inline bool save_camera_parameters(const CameraParameters& params, const std::string& file_name)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "camera_matrix";
    out << params.camera_mat;
    out << YAML::Key << "projection_matrix";
    out << params.proj_mat;
    out << YAML::Key << "distortion_matrix";
    out << params.dist_mat;

    try {
        std::ofstream fout(file_name);
        fout << out.c_str();
        fout.close();
    } catch (const std::exception& e) {
        return false;
    }
    return true;
}

static inline fs::path make_save_path(const std::string& folder)
{
    std::error_code err;
    auto help_create = [&err](const fs::path& path){
        if (fs::exists(path, err)){
            return;
        }
        if (!fs::create_directories(path, err)){
            throw std::invalid_argument("Invalid path: " + path.string() + " . Line " + std::to_string(__LINE__));
        }
    };

    const fs::path root_path(folder);
    help_create(root_path);

    size_t folder_num = 0;
    for (const auto & entry : fs::directory_iterator(root_path)){
        ++folder_num;
    }
    const auto new_folder_name = "IMAGES_" + std::to_string(folder_num);
    const auto new_save_path = root_path / new_folder_name;
    help_create(new_save_path);
    return new_save_path;
}

BaseLogger::BaseLogger(const std::string& output_folder)
{
    output_path_ = make_save_path(output_folder);
}

ImageLogger::ImageLogger(const std::string& output_folder) : BaseLogger(output_folder)
{}

bool ImageLogger::save_image(const cv::Mat& image)
{
    const auto file_path = output_path_ / (std::to_string(data_counter_++) + ".png");
    return cv::imwrite(file_path.string(), image);
}

bool ImageLogger::save_cinfo(const CameraParameters& params)
{
    const auto params_path = output_path_ / "camera_info.yaml";
    return save_camera_parameters(params, params_path.string());
}

CloudLogger::CloudLogger(const std::string& output_folder) : BaseLogger(output_folder)
{}

bool CloudLogger::save_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    const auto file_path = output_path_ / (std::to_string(data_counter_++) + ".pcd");
    return pcl::io::savePCDFileASCII(file_path.string(), *cloud) == 0;
}

}