//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  std::cout<<"init start"<<std::endl;

  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  std::cout<<"---------------urdfFile-----------"<<std::endl;
  std::cout<<urdfFile<<std::endl;
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);
//    for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
//        std::cout<<"contact name :"<<name<<std::endl;
//    }
  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();

  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
    std::cout<<"init finish"<<std::endl;

    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
    std::cout<<"init finish"<<std::endl;

    robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
    std::cout<<"init finish"<<std::endl;

    selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));
    std::cout<<"init finish"<<std::endl;

    // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{"R_JHipRz", "R_JHipRx", "R_JThigh", "R_JKnee", "R_JAnkle", "L_JHipRz",
                                       "L_JHipRx", "L_JThigh", "L_JKnee", "L_JAnkle", "R_JShoulderRx", "R_JUpperArm",
                                       "R_JForeArm","L_JShoulderRx", "L_JUpperArm", "L_JForeArm"};

  for (const auto& joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
    std::cout<<"init finish"<<std::endl;

    imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");
    std::cout<<"init finish"<<std::endl;

  // State estimation
  setupStateEstimate(taskFile, verbose);
    std::cout<<"init finish"<<std::endl;

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);

    wbc_->loadTasksSetting(taskFile, verbose);
    std::cout<<"init finish"<<std::endl;

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());
  std::cout<<"init finish"<<std::endl;

  return true;
}

void LeggedController::starting(const ros::Time& time) {
    std::cout<<"starting start"<<std::endl;
  
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);

//    std::cout<<"leggedInterface_->getCentroidalModelInfo().stateDim"<<leggedInterface_->getCentroidalModelInfo().stateDim<<std::endl;
//    std::cout<<"before StateEstimation currentObservation_: size"<<currentObservation_.state.size()<<std::endl;
//    std::cout<<"before StateEstimation currentObservation_"<<std::endl<<currentObservation_.state<<std::endl;

  updateStateEstimation(time, ros::Duration(0.002));
//    std::cout<<"after StateEstimation currentObservation_: size"<<currentObservation_.state.size()<<std::endl;
//    std::cout<<"after StateEstimation currentObservation_"<<std::endl<<currentObservation_.state<<std::endl;
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;
  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish

//  std::cout<<"1currentObservation_.input.size"<<currentObservation_.input.size()<<std::endl;
//  std::cout<<"1currentObservation_.input"<<currentObservation_.input<<std::endl;
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
  std::cout<<"starting finish"<<std::endl;
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
  std::cout<<"update start"<<std::endl;
  // State Estimate
  updateStateEstimation(time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state,
                                   optimizedState, optimizedInput, plannedMode);

  // Whole body control
  currentObservation_.input = optimizedInput;
//  std::cout<<"-------------- Whole body control start-----------------"<<std::endl;
//  std::cout<<"optimizedInput size "<<optimizedInput.size()<<std::endl;
//  std::cout<<optimizedInput<<std::endl;
//  std::cout<<"optimizedState size "<<optimizedState.size()<<std::endl;
//  std::cout<<optimizedState<<std::endl;

  wbcTimer_.startTimer();
  std::cout<<"-------------   wbc update  -------------"<<std::endl;
  vector_t x = wbc_->update(optimizedState, optimizedInput,
                            measuredRbdState_, plannedMode, period.toSec());
//  std::cout<<"optimizedState: "<<std::endl<<optimizedState<<std::endl; // 28 X 28
//  std::cout<<"optimizedInput: "<<std::endl<<optimizedInput<<std::endl; // 22 X 22
//  std::cout<<"x.size: "<<x.cols()<<"X"<<x.rows()<<std::endl<<x<<std::endl;
//  std::cout<<x<<std::endl;
//  std::cout<<"measuredRbdState_.size: "<<measuredRbdState_.cols()<<"X"<<measuredRbdState_.rows()<<std::endl<<x<<std::endl;
//  std::cout<<measuredRbdState_<<std::endl;
  wbcTimer_.endTimer();
//    std::cout<<"-------------- Whole body control end-----------------"<<std::endl;
  vector_t torque = x.tail(16);

  vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
//    std::cout<<"posDes.size: "<<posDes.size()<<std::endl;
//    std::cout<<posDes<<std::endl;
//    std::cout<<"velDes.size: "<<velDes.size()<<std::endl;
//    std::cout<<velDes<<std::endl;
//    std::cout<<"torque: "<<torque<<std::endl;
//    std::cout<<"---- end  ----"<<std::endl;
  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }
//    std::cout<<"hybridJointHandles_ "<<std::endl<<hybridJointHandles_.size()<<std::endl;

//    for (int i1 = 0; i1 < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++i1) {
//        posDes(i1)=0;
//        velDes(i1)=0;
//        torque(i1)=0;
//    }
  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
  }

  // Visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
    std::cout<<"update finish"<<std::endl;

}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
      std::cout<<"updateStateEstimation start"<<std::endl;

  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
//  std::cout<<"---- updateStateEstimation ----"<<std::endl;
//  std::cout<<"hybridJointHandles_.size() "<< hybridJointHandles_.size()<<std::endl;
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;
      std::cout<<"updateStateEstimation finish"<<std::endl;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
        std::cout<<"updateStateEstimation finish"<<std::endl;

   std::cout<<"contacts.size() "<<std::endl<< contacts.size()<<std::endl;
//    std::cout<<jointPos<<std::endl;
//    std::cout<<"jointVel "<<std::endl<< jointVel.size()<<std::endl;
//    std::cout<<jointVel<<std::endl;
//    std::cout<<"contactFlag "<<std::endl;
  for (size_t i = 0; i < contacts.size(); ++i) {
    contactFlag[i] = contactHandles_[i].isContact();
//      std::cout<<contactFlag[i] <<std::endl;
  }
      std::cout<<"updateStateEstimation finish"<<std::endl;

//    std::cout<<"Imu get Orientation"<<std::endl;
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
//    std::cout<<imuSensorHandle_.getOrientation()[i]<<std::endl;
  }
//    std::cout<<"Imu get AngularVelocity"<<std::endl;
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
//      std::cout<<imuSensorHandle_.getAngularVelocity()[i]<<std::endl;
  }
        std::cout<<"updateStateEstimation finish"<<std::endl;

//    std::cout<<"Imu get LinearAcceleration"<<std::endl;
    for (size_t i = 0; i < 3; ++i) {
        linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
//        std::cout<<imuSensorHandle_.getLinearAcceleration()[i]<<std::endl;
    }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }
      std::cout<<"updateStateEstimation finish"<<std::endl;

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
        std::cout<<"updateStateEstimation finish"<<std::endl;

//  std::cout<<"time: "<<time<<std::endl;
//  std::cout<<"period: "<<period<<std::endl;
//  std::cout<<"measuredRbdState_ size "<<measuredRbdState_.size()<<std::endl;
  std::cout<<measuredRbdState_<<std::endl;
        std::cout<<"updateStateEstimation finish"<<std::endl;

  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
//  std::cout<<"measuredRbdState_.size： "<<std::endl<<measuredRbdState_.size()<<std::endl;
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
//  std::cout<<"currentObservation_ size "<<currentObservation_.state.size()<<std::endl;

//  std::cout<<"currentObservation_.state"<<std::endl<<currentObservation_.state<<std::endl;
      std::cout<<"updateStateEstimation finish"<<std::endl;

}

LeggedController::~LeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
      std::cout<<"updateStateEstimation start"<<std::endl;

  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
//    std::cout<<"-------------setupLeggedInterface------------------------------"<<std::endl;

  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
      std::cout<<"updateStateEstimation finish"<<std::endl;

}

void LeggedController::setupMpc() {
      std::cout<<"updateStateEstimation start"<<std::endl;

  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
        std::cout<<"updateStateEstimation finish"<<std::endl;

}

void LeggedController::setupMrt() {
        std::cout<<"setupMrt start"<<std::endl;

  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();
    /**采用setThreadPriority设置一个新线程，一直循环执行advanceMpc**/
  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
          std::cout<<"setupMrt finish"<<std::endl;

}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
          std::cout<<"setupStateEstimate start"<<std::endl;

  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
            std::cout<<"setupStateEstimate finish"<<std::endl;

}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
            std::cout<<"setupStateEstimate start"<<std::endl;

  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
                                                                      std::cout<<"setupStateEstimate finish"<<std::endl;

}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
