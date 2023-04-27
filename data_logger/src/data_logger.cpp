#include "data_logger/data_logger.hpp"

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
            throw std::invalid_argument("Invalid path. Line " + std::to_string(__LINE__));
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

BaseLogger::BaseLogger()
{

}

ImageLogger::ImageLogger() : BaseLogger()
{
    
}

CloudLogger::CloudLogger() : BaseLogger()
{
    
}

}