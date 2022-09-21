//
// Created by Siddhant Gangapurwala
//

#include <set>
#include <experimental/filesystem>

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include "lfmc/Controller.hpp"

#include "include/Actuation.hpp"


int main() {
    // Relevant Paths
    std::string currentPath(std::experimental::filesystem::current_path());
    std::string configurationDirectory(currentPath + "/../configuration/");
    std::string parametersDirectory(currentPath + "/../parameters/");
    std::string modelsDirectory(currentPath + "/../models/");

    YAML::Node simulationConfiguration = YAML::LoadFile(configurationDirectory + "simulation.yaml");
    std::string controllerParametersDirectory =
            parametersDirectory + "/policies/" + simulationConfiguration["policy"].as<std::string>();

    YAML::Node agentConfiguration = YAML::LoadFile(controllerParametersDirectory + "/agent.yaml");

    auto simFrequency = agentConfiguration["frequency"]["callback"].as<int>();
    auto actuationDecimation = simFrequency / 200;
    auto simTimeStep = 1. / simFrequency;

    // Simulation duration
    auto maxSimulationSteps = simulationConfiguration["simulation"]["duration"].as<int>() * simFrequency;

    // Command update duration
    auto commandUpdateDuration = simulationConfiguration["command"]["duration"].as<int>() * simFrequency;

    // Velocity command scaling
    Eigen::Vector3d velocityCommandScaling{simulationConfiguration["command"]["scaling"]["heading"].as<double>(),
                                           simulationConfiguration["command"]["scaling"]["lateral"].as<double>(),
                                           simulationConfiguration["command"]["scaling"]["yaw"].as<double>()};

    /// Class Objects
    // LFMC Controller
    Controller controller(controllerParametersDirectory + "/agent.yaml",
                          controllerParametersDirectory + "/policy.txt",
                          controllerParametersDirectory + "/state_mean.txt",
                          controllerParametersDirectory + "/state_var.txt");

    // Actuator Network
    Eigen::MatrixXd networkInputScaling(2, 1);
    networkInputScaling.col(0) << 1., 0.1;
    double networkOutputScaling = 100.;
    Actuation actuation(parametersDirectory + "actuation/coyote", networkInputScaling, networkOutputScaling, 12);

    // Initialize Containers
    Eigen::Matrix<double, 19, 1> generalizedCoordinates = controller.getNominalGeneralizedCoordinates();
    Eigen::Matrix<double, 18, 1> generalizedVelocities = Eigen::Matrix<double, 18, 1>::Zero();

    // RaiSim Objects
    raisim::World world;
    world.setTimeStep(simTimeStep);
    world.addGround();

    // Add Robot
    auto robot = world.addArticulatedSystem(modelsDirectory + "anymal_c/urdf/anymal_c.urdf");
    robot->setPdGains(Eigen::VectorXd::Zero(18), Eigen::VectorXd::Zero(18));

    // Launch RaiSim Server for Visualization
    raisim::RaisimServer server(&world);
    server.launchServer(8080);
    server.focusOn(robot);

    // Indices of links that should be the only ones to make contact with ground
    std::set<size_t> footIndices;
    footIndices.insert(robot->getBodyIdx("LF_SHANK"));
    footIndices.insert(robot->getBodyIdx("RF_SHANK"));
    footIndices.insert(robot->getBodyIdx("LH_SHANK"));
    footIndices.insert(robot->getBodyIdx("RH_SHANK"));

    /// Reset the robot state in sim and also the internal state of locomotion controller
    // Lambda function to handle resets
    auto reset = [](raisim::ArticulatedSystem *&r, Controller &c, Actuation &a) {
        r->setState(c.getNominalGeneralizedCoordinates(), Eigen::Matrix<double, 18, 1>::Zero());
        r->setGeneralizedForce(Eigen::Matrix<double, 18, 1>::Zero());

        c.reset();
        a.reset();
    };

    auto terminalState = [](raisim::World &w, raisim::ArticulatedSystem *&r, const std::set<size_t> &f) {
        if (r->getGeneralizedCoordinate()[0] > 6.) return true;

        for (auto &contact: r->getContacts()) {
            if (f.find(contact.getlocalBodyIndex()) == f.end()) {
                return true;
            }
        }

        return false;
    };

    Eigen::Matrix<double, 3, 1> velocityCommand;
    velocityCommand.setZero();

    Eigen::Matrix<double, 12, 1> jointPositionErrors, jointVelocities, jointTorques;
    Eigen::VectorXd generalizedForce;
    generalizedForce.setZero(18);

    reset(robot, controller, actuation);

    int elapsedSimSteps = 0;

    while (elapsedSimSteps < maxSimulationSteps) {
        auto stepStart = std::chrono::high_resolution_clock::now();

        if (elapsedSimSteps % commandUpdateDuration == 0) {
            velocityCommand = velocityCommand.setRandom().cwiseProduct(velocityCommandScaling);
        }

        generalizedCoordinates = robot->getGeneralizedCoordinate().e();
        generalizedVelocities = robot->getGeneralizedVelocity().e();

        // Call the Step function at the Callback Frequency defined in agent.yaml
        auto desiredJointPositions = controller.step(generalizedCoordinates, generalizedVelocities, velocityCommand);

        // Get the torques from the actuator network at 200 Hz
        if (elapsedSimSteps % actuationDecimation == 0) {
            for (auto j = 0; j < 12; ++j) {
                jointPositionErrors[j] = desiredJointPositions[j] - generalizedCoordinates[7 + j];
                jointVelocities[j] = generalizedVelocities[6 + j];
            }

            jointTorques = actuation.getActuationTorques(jointPositionErrors, jointVelocities);

            for (auto j = 0; j < 12; ++j) {
                generalizedForce[6 + j] = jointTorques[j];
            }
        }

        // Update the torques on the joints and integrate physics
        robot->setGeneralizedForce(generalizedForce);
        server.integrateWorldThreadSafe();

        if (terminalState(world, robot, footIndices)) reset(robot, controller, actuation);
        ++elapsedSimSteps;

        auto stepEnd = std::chrono::high_resolution_clock::now();
        auto delay = std::chrono::duration_cast<std::chrono::microseconds>(stepEnd - stepStart);

        std::this_thread::sleep_for(std::chrono::microseconds(
                std::max(static_cast<int>(simTimeStep * 1e6) - static_cast<int>(delay.count()), 0)));
    }

    server.killServer();
    return 0;
}
