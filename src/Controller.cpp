//
// Created by Siddhant Gangapurwala
//

#include <iostream>
#include "lfmc/Controller.hpp"


Controller::Controller(const std::string &configurationPath,
                       const std::string &networkParametersPath,
                       const std::string &stateNormalizationOffsetPath,
                       const std::string &stateNormalizationScalingPath) {
    configurationAgent_ = YAML::LoadFile(configurationPath);

    /// Parse the policy network architecture
    policyLayers_.push_back(configurationAgent_["policy"]["input_dim"].as<unsigned int>());

    for (const auto &h: configurationAgent_["policy"]["hidden_layers"].as<std::vector<unsigned int>>()) {
        policyLayers_.push_back(h);
    }

    policyLayers_.push_back(configurationAgent_["policy"]["output_dim"].as<unsigned int>());

    /// Get the activation function
    policyActivationMap_ = {{"relu",       activation.relu},
                            {"tanh",       activation.tanh},
                            {"softsign",   activation.softsign},
                            {"sigmoid",    activation.sigmoid},
                            {"leaky_relu", activation.leakyReLu}};

    /// Create the Policy object
    policy_ = std::make_unique<MultiLayerPerceptron>(
            policyLayers_, policyActivationMap_.at(configurationAgent_["policy"]["activation"].as<std::string>()),
            networkParametersPath);

    /// Initialize nominal generalized coordinates
    nominalGeneralizedCoordinates_.setZero();
    auto coordinateIndex = 0;

    for (const auto &p: configurationAgent_["robot"]["nominal_pose"].as<std::vector<double>>()) {
        nominalGeneralizedCoordinates_.col(0)[coordinateIndex++] = p;
    }

    for (const auto &j: configurationAgent_["robot"]["nominal_joint_pos"].as<std::vector<double>>()) {
        nominalGeneralizedCoordinates_.col(0)[coordinateIndex++] = j;
    }

    /// Additional parameters from Config
    actionScaling_ = configurationAgent_["scaling"]["action"].as<double>();

    stepCallbackFrequency_ = configurationAgent_["frequency"]["callback"].as<int>();
    controlFrequency_ = configurationAgent_["frequency"]["control"].as<int>();

    controlDecimation_ = stepCallbackFrequency_ / controlFrequency_;

    /// State normalization
    stateOffset_ = loadParametersFromFile(stateNormalizationOffsetPath).transpose().col(0);
    stateScaling_ = loadParametersFromFile(stateNormalizationScalingPath).transpose().col(0);
    stateScaling_ = stateScaling_.cwiseSqrt();

    reset();
}

void Controller::reset() {
    baseLinearVelocity_.setZero();
    baseAngularVelocity_.setZero();

    rotationMatrix_.setIdentity();
    state_.setZero();
    action_.setZero();

    prevGeneralizedCoordinates_ = nominalGeneralizedCoordinates_;
    prevGeneralizedVelocities_.setZero();

    desiredJointPositions_ = nominalGeneralizedCoordinates_.block(7, 0, 12, 1);

    elapsedCallbackSteps_ = 0;
}

const Eigen::Matrix<double, 12, 1> &Controller::step(
        const Eigen::Matrix<double, 19, 1> &generalizedCoordinates,
        const Eigen::Matrix<double, 18, 1> &generalizedVelocities,
        const Eigen::Matrix<double, 3, 1> &velocityCommand) {

    if (elapsedCallbackSteps_++ % controlDecimation_ != 0) {
        if (elapsedCallbackSteps_ == controlDecimation_) {
            elapsedCallbackSteps_ = 0;
        }

        return desiredJointPositions_;
    }

    quaternionToRotationMatrix(generalizedCoordinates.block(3, 0, 4, 1), rotationMatrix_);

    state_.block(0, 0, 3, 1) = rotationMatrix_.transpose() * gravityVector_;
    state_.block(3, 0, 12, 1) = generalizedCoordinates.block(7, 0, 12, 1);
    state_.block(15, 0, 3, 1) = (rotationMatrix_.transpose() * generalizedVelocities.block(3, 0, 3, 1));
    state_.block(18, 0, 12, 1) = generalizedVelocities.block(6, 0, 12, 1);
    state_.block(30, 0, 3, 1) = (rotationMatrix_.transpose() * generalizedVelocities.block(0, 0, 3, 1));
    state_.block(33, 0, 3, 1) = velocityCommand;
    state_.block(36, 0, 12, 1) = desiredJointPositions_ - generalizedCoordinates.tail(12);

    // Normalize observations
    state_ = (state_ - stateOffset_).cwiseQuotient(stateScaling_).cwiseMin(10.).cwiseMax(-10.);

    action_ = policy_->forward(state_).cwiseMin(2.).cwiseMax(-2.);
    desiredJointPositions_ = (action_ * actionScaling_) + nominalGeneralizedCoordinates_.block(7, 0, 12, 1);

    prevGeneralizedCoordinates_ = generalizedCoordinates;
    prevGeneralizedVelocities_ = generalizedVelocities;

    return desiredJointPositions_;
}

const Eigen::Matrix<double, 12, 1> &Controller::getDesiredJointPositions() {
    return desiredJointPositions_;
}

void Controller::quaternionToRotationMatrix(const Eigen::Matrix<double, 4, 1> &q, Eigen::Matrix<double, 3, 3> &r) {
    r(0, 0) = 2. * (q(0) * q(0) + q(1) * q(1)) - 1.;
    r(0, 1) = 2. * (q(1) * q(2) - q(0) * q(3));
    r(0, 2) = 2. * (q(1) * q(3) + q(0) * q(2));

    r(1, 0) = 2. * (q(1) * q(2) + q(0) * q(3));
    r(1, 1) = 2. * (q(0) * q(0) + q(2) * q(2)) - 1.;
    r(1, 2) = 2. * (q(2) * q(3) - q(0) * q(1));

    r(2, 0) = 2. * (q(1) * q(3) - q(0) * q(2));
    r(2, 1) = 2. * (q(2) * q(3) + q(0) * q(1));
    r(2, 2) = 2. * (q(0) * q(0) + q(3) * q(3)) - 1.;
}

const Eigen::Matrix<double, 19, 1> &Controller::getNominalGeneralizedCoordinates() {
    return nominalGeneralizedCoordinates_;
}

const Eigen::MatrixXd &Controller::loadParametersFromFile(const std::string &filePath) {
    /// https://stackoverflow.com/a/22988866

    std::ifstream dataFile;
    dataFile.open(filePath);
    std::string line;
    std::vector<double> values;
    unsigned int rows = 0;

    while (std::getline(dataFile, line)) {
        std::stringstream lineStream(line);
        std::string cell;

        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }

        ++rows;
    }

    fileParameters_ = Eigen::Map<const Eigen::Matrix<typename Eigen::MatrixXd::Scalar,
            Eigen::MatrixXd::RowsAtCompileTime, Eigen::MatrixXd::ColsAtCompileTime,
            Eigen::RowMajor>>(values.data(), rows, static_cast<unsigned int>(values.size()) / rows);

    return fileParameters_;
}
