//
// Created by Siddhant Gangapurwala
//

#ifndef LFMC_CONTROLLER_HPP
#define LFMC_CONTROLLER_HPP

#include <memory>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include "networks_minimal/MultiLayerPerceptron.hpp"


class Controller {
public:

    Controller() = delete;

    Controller(const std::string &configurationPath,
               const std::string &networkParametersPath,
               const std::string &stateNormalizationOffsetPath,
               const std::string &stateNormalizationScalingPath);

    void reset();

    const Eigen::Matrix<double, 12, 1> &step(
            const Eigen::Matrix<double, 19, 1> &generalizedCoordinates,
            const Eigen::Matrix<double, 18, 1> &generalizedVelocities,
            const Eigen::Matrix<double, 3, 1> &velocityCommand);

    const Eigen::Matrix<double, 12, 1> &getDesiredJointPositions();

    const Eigen::Matrix<double, 19, 1> &getNominalGeneralizedCoordinates();

    static void quaternionToRotationMatrix(const Eigen::Matrix<double, 4, 1> &q, Eigen::Matrix<double, 3, 3> &r);

private:
    const Eigen::MatrixXd &loadParametersFromFile(const std::string &filePath);

private:
    Eigen::Matrix<double, 19, 1> nominalGeneralizedCoordinates_;

    Eigen::Matrix<double, 12, 1> desiredJointPositions_;

    Eigen::Matrix<double, 3, 1> baseLinearVelocity_, baseAngularVelocity_;
    Eigen::Matrix<double, 3, 3> rotationMatrix_;

    Eigen::Matrix<double, 48, 1> state_;
    Eigen::Matrix<double, 12, 1> action_;

    std::unique_ptr<MultiLayerPerceptron> policy_;

    Eigen::Matrix<double, 3, 1> velocityCommandScaling_;
    Eigen::Vector3d gravityVector_{0., 0., 1.};

    // State normalization
    Eigen::Matrix<double, 48, 1> stateOffset_, stateScaling_;

    // Containers
    Eigen::Matrix<double, 19, 1> prevGeneralizedCoordinates_;
    Eigen::Matrix<double, 18, 1> prevGeneralizedVelocities_;

    Eigen::MatrixXd fileParameters_;

    // YAML Node
    YAML::Node configurationAgent_;

    std::vector<unsigned int> policyLayers_;
    std::unordered_map<std::string, std::reference_wrapper<Activation>> policyActivationMap_;

    double actionScaling_;
    int stepCallbackFrequency_, controlFrequency_;

    int controlDecimation_, elapsedCallbackSteps_ = 0;
};

#endif // LFMC_CONTROLLER_HPP
