#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include "config.h"
#include "cpp_utils/jsontools.h"
#include "flirmulticamera/hardware_constants.h"
#include <cstdio>
#include <unistd.h>
#include <spdlog/spdlog.h>

namespace flir_icp_calib{


/**
 * Class containing camera matrices
 */
class CameraParameters
{
public:
    CameraParameters(std::string SN, std::vector<float> Intrinsic, std::vector<float> Extrinsic);
    CameraParameters(){};
    ~CameraParameters();
    // intrinsic coefficients
    float fx, fy, cx, cy;

    // intrinsic cx, cy
    Eigen::VectorXf Principle;

    // serial number
    std::string SN;

    // intrinsic
    Eigen::MatrixXf K;

    // extrinsic
    Eigen::MatrixXf M;
    // extrinsic_inv
    Eigen::MatrixXf M_inv;
    // extrinsic rot
    Eigen::MatrixXf R;
    // extrinsic trans
    Eigen::VectorXf t;


    // projection 3x4: 3d->pixel
    Eigen::MatrixXf P;
};

/**
 * Class containing camera objects 
*/
class MultiCameras
{
public:
    MultiCameras();
    ~MultiCameras();

    std::array<CameraParameters, flirmulticamera::GLOBAL_CONST_NCAMS> Cam;
    std::array<std::array<Eigen::MatrixXf, flirmulticamera::GLOBAL_CONST_NCAMS>, flirmulticamera::GLOBAL_CONST_NCAMS> F;

    uint8_t TopCamID;
    std::array<std::string, flirmulticamera::GLOBAL_CONST_NCAMS> CameraSNs;

    void Init(void);
private:
    static Eigen::Matrix3f ComputeF(Eigen::MatrixXf K1, Eigen::MatrixXf K2, Eigen::MatrixXf M1, Eigen::MatrixXf M2);
};

/**
 * Stores calibration document in Multicameras object
 *
 * @param [in] calib_doc contains calibration info
 * @param [out] CameraOut Final cameras object
 * \return true if operation successful, false otherwise.
 */
bool LoadCameras(
    Document& calib_doc,
    MultiCameras &CameraOut
);

/**
 * Loads calibration .json as rapidjson document and checks it against schema. 
 * Loads the cameras into a MultiCameras object.
 *
 * @param [in] calib_file Path to calibration file
 * @param [out] CameraOut Final cameras object
 * \return true if document could get loaded and checked, false otherwise.
 */
bool load_calibration(const std::string& calib_file, MultiCameras &CameraOut);

} // namespace flir_icp_calib