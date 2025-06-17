#include "camera_config.hpp"

using namespace std;
using namespace Eigen;
using namespace rapidjson;

namespace flir_icp_calib{

CameraParameters::CameraParameters(
    std::string SN, std::vector<float> Intrinsic, std::vector<float> Extrinsic, bool mm
)
{
    this->SN = SN;
    if (Intrinsic.size() == 4)
    {
        this->K = MatrixXf{3, 3};
        this->fx = Intrinsic[0];
        this->fy = Intrinsic[1];
        this->cx = Intrinsic[2];
        this->cy = Intrinsic[3];
        
        this->K <<  this->fx,   0,          this->cx,
                    0,          this->fy,   this->cy,
                    0,          0,          1;
    }
    else if (Intrinsic.size() == 9)
    {
        this->K = Map<MatrixXf>(Intrinsic.data(), 3, 3).transpose();
        this->fx = this->K(0, 0);
        this->fy = this->K(1, 1);
        this->cx = this->K(0, 2);
        this->cy = this->K(1, 2);
    }
    else
    {
        std::cout << "Intrinsic has invalid size, should be 4 or 9 but is " << Intrinsic.size() << "\n";
        assert(Intrinsic.size()==4 || Intrinsic.size()==9); 
    }
    this->Principle = VectorXf{2};
    this->Principle << this->cx, this->cy;
    this->M = Map<MatrixXf>(Extrinsic.data(), 4, 4).transpose();
    if (mm)
    {
        this->M(0, 3) /= 1000.f;
        this->M(1, 3) /= 1000.f;
        this->M(2, 3) /= 1000.f;
    }
    this->R = this->M.block<3, 3>(0, 0);
    this->t = this->M.col(3);

    this->M_inv = this->M.inverse();
    this->P = this->K * this->M.block<3, 4>(0, 0);
};

CameraParameters::~CameraParameters()
{
}

MultiCameras::MultiCameras()
{
    for (uint16_t i = 0; i < flirmulticamera::GLOBAL_CONST_NCAMS; i++)
    {
        for (uint16_t j = 0; j < flirmulticamera::GLOBAL_CONST_NCAMS; j++)
        {
            this->F[i][j] = MatrixXf{3, 3};
        }
    }
}

MultiCameras::~MultiCameras()
{
}

void MultiCameras::Init(void)
{
    // calculate Fundamental matrices
    for (uint16_t i = 0; i < flirmulticamera::GLOBAL_CONST_NCAMS; i++)
    {
        for (uint16_t j = 0; j < flirmulticamera::GLOBAL_CONST_NCAMS; j++)
        {
            if (i != j)
            {
                this->F[i][j] = MultiCameras::ComputeF(
                    this->Cam[i].K, this->Cam[j].K, this->Cam[i].M, this->Cam[j].M
                );
            }
            else
            {
                this->F[i][j] << 
                    1.0, 0.0, 1.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0;
            }
        }
    }
}

Matrix3f MultiCameras::ComputeF(MatrixXf K1, MatrixXf K2, MatrixXf M1, MatrixXf M2)
{
    MatrixXf M_relative;
    M_relative = M2 * M1.inverse();

    MatrixXf R = M_relative.block<3, 3>(0, 0);
    VectorXf t = M_relative.block<3, 1>(0, 3);

    VectorXf A = K1 * R.transpose() * t;

    MatrixXf C{3, 3};
    C << 0.0, -A(2), A(1),
        A(2), 0.0, -A(0),
        -A(1), A(0), 0.0;

    Matrix3f F = K2.inverse().transpose() * R * K1.transpose() * C;

    return F;
}

bool LoadCameras(
    Document& calib_doc,
    MultiCameras &CameraOut)
{
    spdlog::info("Calibration Timestamp: {}", calib_doc["timestamp"].GetString());
    // read camera params
    if (!calib_doc["CAMERAS"].IsArray()){
        return false;
    }
    auto cams = calib_doc["CAMERAS"].GetArray();
    std::array<std::string, flirmulticamera::GLOBAL_CONST_NCAMS> SNs;

    if (cams.Size() != flirmulticamera::GLOBAL_CONST_NCAMS)
    {
        std::string error_msg = "Number of cameras in calibration file ({}) != expected number of cameras ({})";
        spdlog::error(error_msg, cams.Size(), flirmulticamera::GLOBAL_CONST_NCAMS);
        throw std::runtime_error(error_msg);
        return false;
    }

    for (uint16_t cidx = 0; cidx < flirmulticamera::GLOBAL_CONST_NCAMS; cidx++)
    {
        SNs.at(cidx) = std::string(flirmulticamera::GLOBAL_CONST_CAMERA_SERIAL_NUMBERS.at(cidx));
        bool found = false; 
        for (uint16_t cdocidx = 0; cdocidx < flirmulticamera::GLOBAL_CONST_NCAMS; cdocidx++)
        {
            auto cam = cams[cdocidx].GetObject();
            if (SNs.at(cidx) == cam["SerialNumber"].GetString())
            {
                found = true;
                std::vector<float> Intrinsic;
                std::vector<float> Extrinsic;
                
                auto Intrinsic_temp = cam["Intrinsic"].GetArray();
                for (uint8_t i = 0; i < Intrinsic_temp.Size(); i++)
                {
                    Intrinsic.push_back(Intrinsic_temp[i].GetFloat());
                }
                
                auto Extrinsic_temp = cam["Extrinsic"].GetArray();
                for (uint8_t i = 0; i < Extrinsic_temp.Size(); i++)
                {
                    Extrinsic.push_back(Extrinsic_temp[i].GetFloat()); 
                }
                
                CameraOut.Cam.at(cidx) = CameraParameters(
                    SNs.at(cidx), 
                    Intrinsic, 
                    Extrinsic,
                    true // mm to m
                );
                spdlog::info("Loaded Calibration of {}", SNs.at(cidx));
                break;
            }
        }
        if (!found){
            spdlog::error("Serial number {} not found in Calibration", SNs.at(cidx));
        }
    }


    for(uint8_t i = 0; i < static_cast<uint8_t>(flirmulticamera::GLOBAL_CONST_NCAMS); i++)
    {
        if (SNs.at(i) == std::string(GLOBAL_CONST_TOP_CAM_SERIAL))
        {
            CameraOut.TopCamID = i;
            break;
        }
    }
    CameraOut.CameraSNs = SNs;
    CameraOut.Init();

    return true;
}

bool load_calibration(const std::string& calib_file, MultiCameras &CameraOut){
    spdlog::info("Loading Calibration File {}", calib_file);
    rapidjson::Document calib_doc;
    if (!cpp_utils::load_json_with_schema(
        calib_file, 
        std::string(CONFIG_DIR)+"/Calibration.schema.json", 
        65536, 
        calib_doc
    ))
    {
        std::string error_msg = "Could not load calibration file";
        spdlog::error(error_msg);
        throw std::runtime_error(error_msg);
        return false;
    }

    if (!LoadCameras(calib_doc, CameraOut))
    {
        std::string error_msg = "Could not load cameras from calibration file";
        spdlog::error(error_msg);
        throw std::runtime_error(error_msg);
        return false;
    }

    return true;
}

} // namespace flir_icp_calib