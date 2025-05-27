#include "camera_config.hpp"

using namespace std;
using namespace Eigen;
using namespace rapidjson;

namespace flir_icp_calib{

CameraParameters::CameraParameters(std::string SN, std::vector<float> Intrinsic, std::vector<float> Extrinsic)
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
    for (uint16_t i = 0; i < GLOBAL_CONST_NCAMS; i++)
    {
        for (uint16_t j = 0; j < GLOBAL_CONST_NCAMS; j++)
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
    for (uint16_t i = 0; i < GLOBAL_CONST_NCAMS; i++)
    {
        for (uint16_t j = 0; j < GLOBAL_CONST_NCAMS; j++)
        {
            if (i != j)
            {
                this->F[i][j] = MultiCameras::ComputeF(
                    this->Cam[i].K, this->Cam[j].K, this->Cam[i].M, this->Cam[j].M
                );
            }
            else
            {
                this->F[i][j] << 1.0, 0.0, 1.0,
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

    // read camera params
    if (!calib_doc["CAMERAS"].IsArray()){
        return false;
    }
    auto cams = calib_doc["CAMERAS"].GetArray();
    std::array<std::string, GLOBAL_CONST_NCAMS> SNs;

    if (cams.Size() != GLOBAL_CONST_NCAMS)
    {
        std::string error_msg = "Number of cameras in calibration file ({}) != expected number of cameras ({})";
        spdlog::error(error_msg, cams.Size(), GLOBAL_CONST_NCAMS);
        throw std::runtime_error(error_msg);
        return false;
    }

    std::array<std::string, GLOBAL_CONST_NCAMS> system_sns;
    for (uint16_t cidx = 0; cidx < GLOBAL_CONST_NCAMS; cidx++)
    {
        system_sns.at(cidx) = std::string(GLOBAL_CONST_CAMERA_SERIAL_NUMBERS.at(cidx));
    }


    for (uint16_t cidx = 0; cidx < GLOBAL_CONST_NCAMS; cidx++)
    {
        auto cam = cams[cidx].GetObject();
        SNs.at(cidx) = cam["SerialNumber"].GetString();

        // check serial number exists
        for (uint16_t i = 0; i < GLOBAL_CONST_NCAMS; i++)
        {
            if (SNs.at(cidx) == system_sns.at(i))
            {
                break;
            }
            if (i == GLOBAL_CONST_NCAMS - 1)
            {
                std::string error_msg = "Serial number {} not found in system serial numbers";
                spdlog::error(error_msg, SNs.at(cidx));
                throw std::runtime_error(error_msg);
                return false;
            }
        }

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
            Extrinsic
        );
    }
    for(uint8_t i = 0; i < static_cast<uint8_t>(GLOBAL_CONST_NCAMS); i++)
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