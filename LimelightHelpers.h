#pragma once

///
/// https://github.com/LimelightVision/limelightlib-wpicpp
///

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#endif

// #include <curl/curl.h>
#include <fcntl.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <wpinet/PortForwarder.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include "networktables/DoubleArrayTopic.h"
#include "wpi/json.h"

/**
 * LimelightHelpers provides static methods and classes for interfacing with Limelight vision cameras in FRC.
 * This library supports all Limelight features including AprilTag tracking, Neural Networks, and standard color/retroreflective tracking.
 */
namespace LimelightHelpers {
  inline std::string sanitizeName(const std::string& name) {
    if (name == "") {
      return "limelight";
    }
    return name;
  }

  /**
     * Takes a 6-length array of pose data and converts it to a Pose3d object.
     * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
     * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
     * @return Pose3d object representing the pose, or empty Pose3d if invalid data
     */
  inline frc::Pose3d toPose3D(const std::vector<double>& inData) {
    if (inData.size() < 6) {
      return frc::Pose3d();
    }
    return frc::Pose3d(
        frc::Translation3d(
            units::length::meter_t(inData[0]), units::length::meter_t(inData[1]), units::length::meter_t(inData[2])),
        frc::Rotation3d(units::angle::radian_t(inData[3] * (std::numbers::pi / 180.0)),
                        units::angle::radian_t(inData[4] * (std::numbers::pi / 180.0)),
                        units::angle::radian_t(inData[5] * (std::numbers::pi / 180.0))));
  }

  /**
     * Takes a 6-length array of pose data and converts it to a Pose2d object.
     * Uses only x, y, and yaw components, ignoring z, roll, and pitch.
     * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
     * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
     * @return Pose2d object representing the pose, or empty Pose2d if invalid data
     */
  inline frc::Pose2d toPose2D(const std::vector<double>& inData) {
    if (inData.size() < 6) {
      return frc::Pose2d();
    }
    return frc::Pose2d(frc::Translation2d(units::length::meter_t(inData[0]), units::length::meter_t(inData[1])),
                       frc::Rotation2d(units::angle::radian_t(inData[5] * (std::numbers::pi / 180.0))));
  }

  /**
     * Converts a Pose3d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
     * Translation components are in meters, rotation components are in degrees.
     *
     * @param pose The Pose3d object to convert
     * @return A 6-element array containing [x, y, z, roll, pitch, yaw]
     */
  inline std::array<double, 6> pose3dToArray(const frc::Pose3d& pose) {
    std::array<double, 6> result;
    result[0] = pose.Translation().X().value();
    result[1] = pose.Translation().Y().value();
    result[2] = pose.Translation().Z().value();
    result[3] = pose.Rotation().X().value() * (180.0 / std::numbers::pi);
    result[4] = pose.Rotation().Y().value() * (180.0 / std::numbers::pi);
    result[5] = pose.Rotation().Z().value() * (180.0 / std::numbers::pi);
    return result;
  }

  /**
     * Converts a Pose2d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
     * Translation components are in meters, rotation components are in degrees.
     * Note: z, roll, and pitch will be 0 since Pose2d only contains x, y, and yaw.
     *
     * @param pose The Pose2d object to convert
     * @return A 6-element array containing [x, y, 0, 0, 0, yaw]
     */
  inline std::array<double, 6> pose2dToArray(const frc::Pose2d& pose) {
    std::array<double, 6> result;
    result[0] = pose.Translation().X().value();
    result[1] = pose.Translation().Y().value();
    result[2] = 0;
    result[3] = 0;
    result[4] = 0;
    result[5] = pose.Rotation().Degrees().value();
    return result;
  }

  inline std::shared_ptr<nt::NetworkTable> getLimelightNTTable(const std::string& tableName) {
    return nt::NetworkTableInstance::GetDefault().GetTable(sanitizeName(tableName));
  }

  inline void Flush() {
    nt::NetworkTableInstance::GetDefault().Flush();
  }

  inline nt::NetworkTableEntry getLimelightNTTableEntry(const std::string& tableName, const std::string& entryName) {
    return getLimelightNTTable(tableName)->GetEntry(entryName);
  }

  inline std::unordered_map<std::string, nt::DoubleArrayEntry> doubleArrayEntries;

  inline nt::DoubleArrayEntry& getLimelightDoubleArrayEntry(const std::string& tableName,
                                                            const std::string& entryName) {
    const std::string& key = tableName + "/" + entryName;
    auto it = doubleArrayEntries.find(key);
    if (it == doubleArrayEntries.end()) {
      std::shared_ptr<nt::NetworkTable> table = getLimelightNTTable(tableName);
      nt::DoubleArrayTopic daTopic = table->GetDoubleArrayTopic(entryName);
      nt::DoubleArrayEntry entry = daTopic.GetEntry(std::span<double>{});
      doubleArrayEntries.emplace(key, std::move(entry));
      return doubleArrayEntries[key];
    }
    return it->second;
  }

  inline double getLimelightNTDouble(const std::string& tableName, const std::string& entryName) {
    return getLimelightNTTableEntry(tableName, entryName).GetDouble(0.0);
  }

  inline std::vector<double> getLimelightNTDoubleArray(const std::string& tableName, const std::string& entryName) {
    return getLimelightNTTableEntry(tableName, entryName).GetDoubleArray(std::span<double>{});
  }

  inline std::string getLimelightNTString(const std::string& tableName, const std::string& entryName) {
    return getLimelightNTTableEntry(tableName, entryName).GetString("");
  }

  inline std::vector<std::string> getLimelightNTStringArray(const std::string& tableName,
                                                            const std::string& entryName) {
    return getLimelightNTTableEntry(tableName, entryName).GetStringArray(std::span<std::string>{});
  }

  inline void setLimelightNTDouble(const std::string& tableName, const std::string entryName, double val) {
    getLimelightNTTableEntry(tableName, entryName).SetDouble(val);
  }

  inline void setLimelightNTDoubleArray(const std::string& tableName,
                                        const std::string& entryName,
                                        const std::span<const double>& vals) {
    getLimelightNTTableEntry(tableName, entryName).SetDoubleArray(vals);
  }

  /**
     * Does the Limelight have a valid target?
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return True if a valid target is present, false otherwise
     */
  inline bool getTV(const std::string& limelightName = "") {
    return getLimelightNTDouble(limelightName, "tv") == 1.0;
  }

  /**
     * Gets the horizontal offset from the crosshair to the target in degrees.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Horizontal offset angle in degrees
     */
  inline double getTX(const std::string& limelightName = "") {
    return getLimelightNTDouble(limelightName, "tx");
  }

  /**
     * Gets the vertical offset from the crosshair to the target in degrees.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Vertical offset angle in degrees
     */
  inline double getTY(const std::string& limelightName = "") {
    return getLimelightNTDouble(limelightName, "ty");
  }

  /**
     * Gets the horizontal offset from the principal pixel/point to the target in degrees.  This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Horizontal offset angle in degrees
     */
  inline double getTXNC(const std::string& limelightName) {
    return getLimelightNTDouble(limelightName, "txnc");
  }

  /**
     * Gets the vertical offset from the principal pixel/point to the target in degrees. This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Vertical offset angle in degrees
     */
  inline double getTYNC(const std::string& limelightName) {
    return getLimelightNTDouble(limelightName, "tync");
  }

  /**
     * Gets the target area as a percentage of the image (0-100%).
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Target area percentage (0-100)
     */

  inline double getTA(const std::string& limelightName = "") {
    return getLimelightNTDouble(limelightName, "ta");
  }

  /**
     * T2D is an array that contains several targeting metrcis
     * @param limelightName Name of the Limelight camera
     * @return Array containing  [targetValid, targetCount, targetLatency, captureLatency, tx, ty, txnc, tync, ta, tid, targetClassIndexDetector,
     * targetClassIndexClassifier, targetLongSidePixels, targetShortSidePixels, targetHorizontalExtentPixels, targetVerticalExtentPixels, targetSkewDegrees]
     */
  inline std::vector<double> getT2DArray(const std::string& limelightName) {
    return getLimelightNTDoubleArray(limelightName, "t2d");
  }

  /**
     * Gets the number of targets currently detected.
     * @param limelightName Name of the Limelight camera
     * @return Number of detected targets
     */
  inline int getTargetCount(const std::string& limelightName) {
    std::vector<double> t2d = getT2DArray(limelightName);
    if (t2d.size() == 17) {
      return (int)t2d[1];
    }
    return 0;
  }

  /**
     * Gets the classifier class index from the currently running neural classifier pipeline
     * @param limelightName Name of the Limelight camera
     * @return Class index from classifier pipeline
     */
  inline int getClassifierClassIndex(const std::string& limelightName) {
    std::vector<double> t2d = getT2DArray(limelightName);
    if (t2d.size() == 17) {
      return (int)t2d[10];
    }
    return 0;
  }

  /**
     * Gets the detector class index from the primary result of the currently running neural detector pipeline.
     * @param limelightName Name of the Limelight camera
     * @return Class index from detector pipeline
     */
  inline int getDetectorClassIndex(const std::string& limelightName) {
    std::vector<double> t2d = getT2DArray(limelightName);
    if (t2d.size() == 17) {
      return (int)t2d[11];
    }
    return 0;
  }

  /**
     * Gets the current neural classifier result class name.
     * @param limelightName Name of the Limelight camera
     * @return Class name string from classifier pipeline
     */
  inline const std::string getClassifierClass(const std::string& limelightName) {
    return getLimelightNTString(limelightName, "tcclass");
  }

  /**
     * Gets the primary neural detector result class name.
     * @param limelightName Name of the Limelight camera
     * @return Class name string from detector pipeline
     */
  inline const std::string getDetectorClass(const std::string& limelightName) {
    return getLimelightNTString(limelightName, "tdclass");
  }

  /**
     * Gets the pipeline's processing latency contribution.
     * @param limelightName Name of the Limelight camera
     * @return Pipeline latency in milliseconds
     */

  inline double getLatency_Pipeline(const std::string& limelightName = "") {
    return getLimelightNTDouble(limelightName, "tl");
  }

  /**
     * Gets the capture latency.
     * @param limelightName Name of the Limelight camera
     * @return Capture latency in milliseconds
     */
  inline double getLatency_Capture(const std::string& limelightName = "") {
    return getLimelightNTDouble(limelightName, "cl");
  }
  /**
     * Gets the active pipeline index.
     * @param limelightName Name of the Limelight camera
     * @return Current pipeline index (0-9)
     */
  inline double getCurrentPipelineIndex(const std::string& limelightName) {
    return getLimelightNTDouble(limelightName, "getpipe");
  }

  /**
     * Gets the current pipeline type.
     * @param limelightName Name of the Limelight camera
     * @return Pipeline type string (e.g. "retro", "apriltag", etc)
     */
  inline const std::string getCurrentPipelineType(const std::string& limelightName) {
    return getLimelightNTString(limelightName, "getpipetype");
  }

  /**
     * Gets the full JSON results dump.
     * @param limelightName Name of the Limelight camera
     * @return JSON string containing all current results
     */
  inline std::string getJSONDump(const std::string& limelightName = "") {
    return getLimelightNTString(limelightName, "json");
  }

  inline std::vector<double> getBotpose(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "botpose");
  }

  inline std::vector<double> getBotpose_wpiRed(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
  }

  inline std::vector<double> getBotpose_wpiBlue(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
  }

  inline std::vector<double> getBotpose_TargetSpace(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
  }

  inline std::vector<double> getCameraPose_TargetSpace(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
  }

  inline std::vector<double> getCameraPose_RobotSpace(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "camerapose_robotspace");
  }

  inline std::vector<double> getTargetPose_CameraSpace(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
  }

  inline std::vector<double> getTargetPose_RobotSpace(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
  }

  inline std::vector<double> getTargetColor(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "tc");
  }

  inline double getFiducialID(const std::string& limelightName = "") {
    return getLimelightNTDouble(limelightName, "tid");
  }

  inline std::string getNeuralClassID(const std::string& limelightName = "") {
    return getLimelightNTString(limelightName, "tclass");
  }

  inline std::vector<std::string> getRawBarcodeData(const std::string& limelightName = "") {
    return getLimelightNTStringArray(limelightName, "rawbarcodes");
  }

  inline void setPipelineIndex(const std::string& limelightName, int index) {
    setLimelightNTDouble(limelightName, "pipeline", index);
  }

  inline void setPriorityTagID(const std::string& limelightName, int ID) {
    setLimelightNTDouble(limelightName, "priorityid", ID);
  }

  inline void setLEDMode_PipelineControl(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "ledMode", 0);
  }

  inline void setLEDMode_ForceOff(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "ledMode", 1);
  }

  inline void setLEDMode_ForceBlink(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "ledMode", 2);
  }

  inline void setLEDMode_ForceOn(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "ledMode", 3);
  }

  inline void setStreamMode_Standard(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "stream", 0);
  }

  inline void setStreamMode_PiPMain(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "stream", 1);
  }

  inline void setStreamMode_PiPSecondary(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "stream", 2);
  }

  /**
     * Sets the crop window for the camera. The crop window in the UI must be completely open.
     * @param limelightName Name of the Limelight camera
     * @param cropXMin Minimum X value (-1 to 1)
     * @param cropXMax Maximum X value (-1 to 1)
     * @param cropYMin Minimum Y value (-1 to 1)
     * @param cropYMax Maximum Y value (-1 to 1)
     */
  inline void setCropWindow(
      const std::string& limelightName, double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
    double cropWindow[4]{cropXMin, cropXMax, cropYMin, cropYMax};
    setLimelightNTDoubleArray(limelightName, "crop", cropWindow);
  }

  /**
     * Sets 3D offset point for easy 3D targeting.
     */
  inline void setFiducial3DOffset(const std::string& limelightName, double offsetX, double offsetY, double offsetZ) {
    std::array<double, 3> entries;
    entries[0] = offsetX;
    entries[1] = offsetY;
    entries[2] = offsetZ;
    setLimelightNTDoubleArray(limelightName, "fiducial_offset_set", entries);
  }

  inline void SetRobotOrientation_INTERNAL(const std::string& limelightName,
                                           double yaw,
                                           double yawRate,
                                           double pitch,
                                           double pitchRate,
                                           double roll,
                                           double rollRate,
                                           bool flush) {
    std::array<double, 6> entries;
    entries[0] = yaw;
    entries[1] = yawRate;
    entries[2] = pitch;
    entries[3] = pitchRate;
    entries[4] = roll;
    entries[5] = rollRate;
    setLimelightNTDoubleArray(limelightName, "robot_orientation_set", entries);
    if (flush) {
      Flush();
    }
  }

  /**
     * Sets the robot orientation for mt2.
     */
  inline void SetRobotOrientation(const std::string& limelightName,
                                  double yaw,
                                  double yawRate,
                                  double pitch,
                                  double pitchRate,
                                  double roll,
                                  double rollRate) {
    SetRobotOrientation_INTERNAL(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate, true);
  }

  inline void SetRobotOrientation_NoFlush(const std::string& limelightName,
                                          double yaw,
                                          double yawRate,
                                          double pitch,
                                          double pitchRate,
                                          double roll,
                                          double rollRate) {
    SetRobotOrientation_INTERNAL(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate, false);
  }

  /**
     * Configures the IMU mode for MegaTag2 Localization
     *
     * @param limelightName Name/identifier of the Limelight
     * @param mode IMU mode.
     */
  inline void SetIMUMode(const std::string& limelightName, int mode) {
    setLimelightNTDouble(limelightName, "imumode_set", mode);
  }

  /**
     * Sets the 3D point-of-interest offset for the current fiducial pipeline.
     * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d#point-of-interest-tracking
     *
     * @param limelightName Name/identifier of the Limelight
     * @param x X offset in meters
     * @param y Y offset in meters
     * @param z Z offset in meters
     */
  inline void SetFidcuial3DOffset(const std::string& limelightName, double x, double y, double z) {
    std::array<double, 3> entries;
    entries[0] = x;
    entries[1] = y;
    entries[2] = z;
    setLimelightNTDoubleArray(limelightName, "fiducial_offset_set", entries);
  }

  inline void SetFiducialIDFiltersOverride(const std::string& limelightName, const std::vector<int>& validIDs) {
    std::vector<double> validIDsDouble(validIDs.begin(), validIDs.end());
    setLimelightNTDoubleArray(limelightName, "fiducial_id_filters_set", validIDsDouble);
  }

  /**
     * Sets the camera pose in robotspace. The UI camera pose must be set to zeros
     */
  inline void setCameraPose_RobotSpace(
      const std::string& limelightName, double forward, double side, double up, double roll, double pitch, double yaw) {
    double entries[6] = {forward, side, up, roll, pitch, yaw};
    setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
  }

  inline void setPythonScriptData(const std::string& limelightName, const std::vector<double>& outgoingPythonData) {
    setLimelightNTDoubleArray(
        limelightName, "llrobot", std::span{outgoingPythonData.begin(), outgoingPythonData.size()});
  }

  inline std::vector<double> getPythonScriptData(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "llpython");
  }

  // Take async snapshot

  inline double extractBotPoseEntry(const std::vector<double>& inData, int position) {
    if (inData.size() < static_cast<size_t>(position + 1)) {
      return 0.0;
    }
    return inData[position];
  }

  class RawFiducial {
   public:
    int id{0};
    double txnc{0.0};
    double tync{0.0};
    double ta{0.0};
    double distToCamera{0.0};
    double distToRobot{0.0};
    double ambiguity{0.0};

    RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity)
        : id(id)
        , txnc(txnc)
        , tync(tync)
        , ta(ta)
        , distToCamera(distToCamera)
        , distToRobot(distToRobot)
        , ambiguity(ambiguity) {}
  };

  /**
     * Represents a 3D Pose Estimate.
     */
  class PoseEstimate {
   public:
    frc::Pose2d pose;
    units::time::second_t timestampSeconds{0.0};
    double latency{0.0};
    int tagCount{0};
    double tagSpan{0.0};
    double avgTagDist{0.0};
    double avgTagArea{0.0};
    std::vector<RawFiducial> rawFiducials;
    bool isMegaTag2;

    PoseEstimate() = default;

    PoseEstimate(const frc::Pose2d& pose,
                 units::time::second_t timestampSeconds,
                 double latency,
                 int tagCount,
                 double tagSpan,
                 double avgTagDist,
                 double avgTagArea,
                 const std::vector<RawFiducial>& rawFiducials,
                 bool isMegaTag2)
        : pose(pose)
        , timestampSeconds(timestampSeconds)
        , latency(latency)
        , tagCount(tagCount)
        , tagSpan(tagSpan)
        , avgTagDist(avgTagDist)
        , avgTagArea(avgTagArea)
        , rawFiducials(rawFiducials)
        , isMegaTag2(isMegaTag2) {}
  };

  inline bool validPoseEstimate(const PoseEstimate& pose) {
    return !pose.rawFiducials.empty();
  }

  /**
     * Encapsulates the state of an internal Limelight IMU.
     */
  class IMUData {
   public:
    double robotYaw{0.0};
    double Roll{0.0};
    double Pitch{0.0};
    double Yaw{0.0};
    double gyroX{0.0};
    double gyroY{0.0};
    double gyroZ{0.0};
    double accelX{0.0};
    double accelY{0.0};
    double accelZ{0.0};

    IMUData() = default;

    IMUData(double imuData[10]) {
      if (imuData != nullptr) {
        robotYaw = imuData[0];
        Roll = imuData[1];
        Pitch = imuData[2];
        Yaw = imuData[3];
        gyroX = imuData[4];
        gyroY = imuData[5];
        gyroZ = imuData[6];
        accelX = imuData[7];
        accelY = imuData[8];
        accelZ = imuData[9];
      }
    }
  };

  inline PoseEstimate getBotPoseEstimate(const std::string& limelightName,
                                         const std::string& entryName,
                                         bool isMegaTag2) {
    nt::DoubleArrayEntry& poseEntry = getLimelightDoubleArrayEntry(limelightName, entryName);
    auto tsValue = poseEntry.GetAtomic();

    std::vector<double> poseArray = tsValue.value;
    auto timestamp = tsValue.time;

    if (poseArray.size() == 0) {
      return PoseEstimate();
    }

    frc::Pose2d pose = toPose2D(poseArray);

    double latency = extractBotPoseEntry(poseArray, 6);
    int tagCount = static_cast<int>(extractBotPoseEntry(poseArray, 7));
    double tagSpan = extractBotPoseEntry(poseArray, 8);
    double tagDist = extractBotPoseEntry(poseArray, 9);
    double tagArea = extractBotPoseEntry(poseArray, 10);

    double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

    std::vector<RawFiducial> rawFiducials;
    int valsPerFiducial = 7;
    size_t expectedTotalVals = 11 + valsPerFiducial * tagCount;

    if (poseArray.size() == expectedTotalVals) {
      for (int i = 0; i < tagCount; i++) {
        int baseIndex = 11 + (i * valsPerFiducial);
        int id = static_cast<int>(extractBotPoseEntry(poseArray, baseIndex));
        double txnc = extractBotPoseEntry(poseArray, baseIndex + 1);
        double tync = extractBotPoseEntry(poseArray, baseIndex + 2);
        double ta = extractBotPoseEntry(poseArray, baseIndex + 3);
        double distToCamera = extractBotPoseEntry(poseArray, baseIndex + 4);
        double distToRobot = extractBotPoseEntry(poseArray, baseIndex + 5);
        double ambiguity = extractBotPoseEntry(poseArray, baseIndex + 6);
        rawFiducials.emplace_back(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
      }
    }

    return PoseEstimate(pose,
                        units::time::second_t(adjustedTimestamp),
                        latency,
                        tagCount,
                        tagSpan,
                        tagDist,
                        tagArea,
                        rawFiducials,
                        isMegaTag2);
  }

  inline PoseEstimate getBotPoseEstimate_wpiBlue(const std::string& limelightName = "") {
    return getBotPoseEstimate(limelightName, "botpose_wpiblue", false);
  }

  inline PoseEstimate getBotPoseEstimate_wpiRed(const std::string& limelightName = "") {
    return getBotPoseEstimate(limelightName, "botpose_wpired", false);
  }

  inline PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(const std::string& limelightName = "") {
    return getBotPoseEstimate(limelightName, "botpose_orb_wpiblue", true);
  }

  inline PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(const std::string& limelightName = "") {
    return getBotPoseEstimate(limelightName, "botpose_orb_wpired", true);
  }

  /**
     * Gets the current IMU data from NetworkTables.
     * IMU data is formatted as [robotYaw, Roll, Pitch, Yaw, gyroX, gyroY, gyroZ, accelX, accelY, accelZ].
     * Returns all zeros if data is invalid or unavailable.
     *
     * @param limelightName Name/identifier of the Limelight
     * @return IMUData object containing all current IMU data
     */
  inline IMUData getIMUData(const std::string& limelightName) {
    std::vector<double> imuData = getLimelightNTDoubleArray(limelightName, "imu");
    if (imuData.empty() || imuData.size() < 10) {
      return IMUData();  // Returns object with all zeros
    }
    return IMUData(imuData.data());
  }

  inline const double INVALID_TARGET = 0.0;
  class SingleTargetingResultClass {
   public:
    double m_TargetXPixels{INVALID_TARGET};
    double m_TargetYPixels{INVALID_TARGET};

    double m_TargetXNormalized{INVALID_TARGET};
    double m_TargetYNormalized{INVALID_TARGET};

    double m_TargetXNormalizedCrosshairAdjusted{INVALID_TARGET};
    double m_TargetYNormalizedCrosshairAdjusted{INVALID_TARGET};

    double m_TargetXDegreesCrosshairAdjusted{INVALID_TARGET};
    double m_TargetYDegreesCrosshairAdjusted{INVALID_TARGET};

    double m_TargetAreaPixels{INVALID_TARGET};
    double m_TargetAreaNormalized{INVALID_TARGET};
    double m_TargetAreaNormalizedPercentage{INVALID_TARGET};

    // not included in json//
    double m_timeStamp{-1.0};
    double m_latency{0};
    double m_pipelineIndex{-1.0};
    std::vector<std::vector<double>> m_TargetCorners;

    std::vector<double> m_CAMERATransform6DTARGETSPACE;
    std::vector<double> m_TargetTransform6DCAMERASPACE;
    std::vector<double> m_TargetTransform6DROBOTSPACE;
    std::vector<double> m_ROBOTTransform6DTARGETSPACE;
    std::vector<double> m_ROBOTTransform6DFIELDSPACE;
    std::vector<double> m_CAMERATransform6DROBOTSPACE;
  };

  class RetroreflectiveResultClass : public SingleTargetingResultClass {};

  class FiducialResultClass : public SingleTargetingResultClass {
   public:
    int m_fiducialID{0};
    std::string m_family{""};
  };

  class BarcodeResultClass : public SingleTargetingResultClass {
   public:
    std::string m_family{""};
    std::string m_data{""};

    // Should these actually use m_TargetXNormalizedCrosshairAdjusted and be part of SingleTargetingResultClass?
    double m_TargetXDegreesNoCrosshairAdjusted{INVALID_TARGET};
    double m_TargetYDegreesNoCrosshairAdjusted{INVALID_TARGET};
  };

  class DetectionResultClass : public SingleTargetingResultClass {
   public:
    int m_classID{-1};
    std::string m_className{""};
    double m_confidence{0};

    // Should these actually use m_TargetXNormalizedCrosshairAdjusted and be part of SingleTargetingResultClass?
    double m_TargetXDegreesNoCrosshairAdjusted{INVALID_TARGET};
    double m_TargetYDegreesNoCrosshairAdjusted{INVALID_TARGET};
  };

  class ClassificationResultClass : public SingleTargetingResultClass {
   public:
    int m_classID{-1};
    std::string m_className{""};
    double m_confidence{0};
  };

  class VisionResultsClass {
   public:
    std::vector<RetroreflectiveResultClass> RetroResults;
    std::vector<FiducialResultClass> FiducialResults;
    std::vector<DetectionResultClass> DetectionResults;
    std::vector<ClassificationResultClass> ClassificationResults;
    std::vector<BarcodeResultClass> BarcodeResults;
    double m_timeStamp{-1.0};
    double m_latencyPipeline{0};
    double m_latencyCapture{0};
    double m_latencyJSON{0};
    double m_pipelineIndex{-1.0};
    int valid{0};
    std::vector<double> botPose{6, 0.0};
    std::vector<double> botPose_wpired{6, 0.0};
    std::vector<double> botPose_wpiblue{6, 0.0};
    void Clear() {
      RetroResults.clear();
      FiducialResults.clear();
      DetectionResults.clear();
      ClassificationResults.clear();
      botPose.clear();
      botPose_wpired.clear();
      botPose_wpiblue.clear();
      m_timeStamp = -1.0;
      m_latencyPipeline = 0;

      m_pipelineIndex = -1.0;
    }
  };

  class LimelightResultsClass {
   public:
    VisionResultsClass targetingResults;
  };

  namespace internal {
    inline const std::string _key_timestamp{"ts"};
    inline const std::string _key_latency_pipeline{"tl"};
    inline const std::string _key_latency_capture{"cl"};

    inline const std::string _key_pipelineIndex{"pID"};
    inline const std::string _key_TargetXDegrees{"txdr"};
    inline const std::string _key_TargetYDegrees{"tydr"};
    inline const std::string _key_TargetXNormalized{"txnr"};
    inline const std::string _key_TargetYNormalized{"tynr"};
    inline const std::string _key_TargetXPixels{"txp"};
    inline const std::string _key_TargetYPixels{"typ"};

    inline const std::string _key_TargetXDegreesCrosshair{"tx"};
    inline const std::string _key_TargetXDegreesNoCrosshair{"tx_nocross"};
    inline const std::string _key_TargetYDegreesCrosshair{"ty"};
    inline const std::string _key_TargetYDegreesNoCrosshair{"ty_nocross"};
    inline const std::string _key_TargetXNormalizedCrosshair{"txn"};
    inline const std::string _key_TargetYNormalizedCrosshair{"tyn"};
    inline const std::string _key_TargetAreaNormalized{"ta"};
    inline const std::string _key_TargetAreaPixels{"tap"};
    inline const std::string _key_className{"class"};
    inline const std::string _key_classID{"classID"};
    inline const std::string _key_confidence{"conf"};
    inline const std::string _key_fiducialID{"fID"};
    inline const std::string _key_corners{"pts"};
    inline const std::string _key_transformCAMERAPOSE_TARGETSPACE{"t6c_ts"};
    inline const std::string _key_transformCAMERAPOSE_ROBOTSPACE{"t6c_rs"};

    inline const std::string _key_transformTARGETPOSE_CAMERASPACE{"t6t_cs"};
    inline const std::string _key_transformROBOTPOSE_TARGETSPACE{"t6r_ts"};
    inline const std::string _key_transformTARGETPOSE_ROBOTSPACE{"t6t_rs"};

    inline const std::string _key_botpose{"botpose"};
    inline const std::string _key_botpose_wpiblue{"botpose_wpiblue"};
    inline const std::string _key_botpose_wpired{"botpose_wpired"};

    inline const std::string _key_transformROBOTPOSE_FIELDSPACE{"t6r_fs"};
    inline const std::string _key_skew{"skew"};
    inline const std::string _key_ffamily{"fam"};
    inline const std::string _key_data{"data"};
    inline const std::string _key_colorRGB{"cRGB"};
    inline const std::string _key_colorHSV{"cHSV"};
  }  // namespace internal

#ifndef _WIN32
  inline void PhoneHome() {
    static int sockfd = -1;
    static struct sockaddr_in servaddr, cliaddr;

    if (sockfd == -1) {
      sockfd = socket(AF_INET, SOCK_DGRAM, 0);
      if (sockfd < 0) {
        std::cerr << "Socket creation failed" << std::endl;
        return;
      }

      std::memset(&servaddr, 0, sizeof(servaddr));
      servaddr.sin_family = AF_INET;
      servaddr.sin_addr.s_addr = inet_addr("255.255.255.255");
      servaddr.sin_port = htons(5809);

      // Set socket for broadcast
      int broadcast = 1;
      if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
        std::cerr << "Error in setting Broadcast option" << std::endl;
        close(sockfd);
        sockfd = -1;
        return;
      }

      // Set socket to non-blocking
      if (fcntl(sockfd, F_SETFL, O_NONBLOCK) < 0) {
        std::cerr << "Error setting socket to non-blocking" << std::endl;
        close(sockfd);
        sockfd = -1;
        return;
      }

      const char* msg = "LLPhoneHome";
      sendto(sockfd, msg, std::strlen(msg), 0, (const struct sockaddr*)&servaddr, sizeof(servaddr));
    }

    char receiveData[1024];
    socklen_t len = sizeof(cliaddr);

    ssize_t n = recvfrom(sockfd, reinterpret_cast<char*>(receiveData), 1024, 0, (struct sockaddr*)&cliaddr, &len);
    if (n > 0) {
      receiveData[n] = '\0';  // Null-terminate the received string
      std::string received(receiveData, n);
      std::cout << "Received response: " << received << std::endl;
    } else if (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
      std::cerr << "Error receiving data" << std::endl;
      close(sockfd);
      sockfd = -1;
    }
  }
#endif

  inline void SetupPortForwarding(const std::string& limelightName) {
    auto& portForwarder = wpi::PortForwarder::GetInstance();
    portForwarder.Add(5800, sanitizeName(limelightName), 5800);
    portForwarder.Add(5801, sanitizeName(limelightName), 5801);
    portForwarder.Add(5802, sanitizeName(limelightName), 5802);
    portForwarder.Add(5803, sanitizeName(limelightName), 5803);
    portForwarder.Add(5804, sanitizeName(limelightName), 5804);
    portForwarder.Add(5805, sanitizeName(limelightName), 5805);
    portForwarder.Add(5806, sanitizeName(limelightName), 5806);
    portForwarder.Add(5807, sanitizeName(limelightName), 5807);
    portForwarder.Add(5808, sanitizeName(limelightName), 5808);
    portForwarder.Add(5809, sanitizeName(limelightName), 5809);
  }

  template <typename T, typename KeyType>
  T SafeJSONAccess(const wpi::json& jsonData, const KeyType& key, const T& defaultValue) {
    try {
      return jsonData.at(key).template get<T>();
    } catch (wpi::json::exception& e) {
      return defaultValue;
    } catch (...) {
      return defaultValue;
    }
  }

  /**
     * Represents a Color/Retroreflective Target Result extracted from JSON Output
     */
  inline void from_json(const wpi::json& j, RetroreflectiveResultClass& t) {
    std::vector<double> defaultValueVector(6, 0.0);
    t.m_CAMERATransform6DTARGETSPACE =
        SafeJSONAccess<std::vector<double>>(j, internal::_key_transformCAMERAPOSE_TARGETSPACE, defaultValueVector);
    t.m_CAMERATransform6DROBOTSPACE =
        SafeJSONAccess<std::vector<double>>(j, internal::_key_transformCAMERAPOSE_ROBOTSPACE, defaultValueVector);

    t.m_TargetTransform6DCAMERASPACE =
        SafeJSONAccess<std::vector<double>>(j, internal::_key_transformTARGETPOSE_CAMERASPACE, defaultValueVector);
    t.m_TargetTransform6DROBOTSPACE =
        SafeJSONAccess<std::vector<double>>(j, internal::_key_transformTARGETPOSE_ROBOTSPACE, defaultValueVector);
    t.m_ROBOTTransform6DTARGETSPACE =
        SafeJSONAccess<std::vector<double>>(j, internal::_key_transformROBOTPOSE_TARGETSPACE, defaultValueVector);
    t.m_ROBOTTransform6DFIELDSPACE =
        SafeJSONAccess<std::vector<double>>(j, internal::_key_transformROBOTPOSE_FIELDSPACE, defaultValueVector);

    t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
    t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
    t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
    t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
    t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
    t.m_TargetCorners =
        SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
  }

  /**
     * Represents an AprilTag/Fiducial Target Result extracted from JSON Output
     */
  inline void from_json(const wpi::json& j, FiducialResultClass& t) {
    std::vector<double> defaultValueVector(6, 0.0);
    t.m_family = SafeJSONAccess<std::string>(j, internal::_key_ffamily, "");
    t.m_fiducialID = SafeJSONAccess<double>(j, internal::_key_fiducialID, 0.0);
    t.m_CAMERATransform6DTARGETSPACE =
        SafeJSONAccess<std::vector<double>>(j, internal::_key_transformCAMERAPOSE_TARGETSPACE, defaultValueVector);
    t.m_CAMERATransform6DROBOTSPACE =
        SafeJSONAccess<std::vector<double>>(j, internal::_key_transformCAMERAPOSE_ROBOTSPACE, defaultValueVector);
    t.m_TargetTransform6DCAMERASPACE =
        SafeJSONAccess<std::vector<double>>(j, internal::_key_transformTARGETPOSE_CAMERASPACE, defaultValueVector);
    t.m_TargetTransform6DROBOTSPACE =
        SafeJSONAccess<std::vector<double>>(j, internal::_key_transformTARGETPOSE_ROBOTSPACE, defaultValueVector);
    t.m_ROBOTTransform6DTARGETSPACE =
        SafeJSONAccess<std::vector<double>>(j, internal::_key_transformROBOTPOSE_TARGETSPACE, defaultValueVector);
    t.m_ROBOTTransform6DFIELDSPACE =
        SafeJSONAccess<std::vector<double>>(j, internal::_key_transformROBOTPOSE_FIELDSPACE, defaultValueVector);

    t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
    t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
    t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
    t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
    t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
    t.m_TargetCorners =
        SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
  }

  /**
     * Represents a Neural Detector Pipeline Result extracted from JSON Output
     */
  inline void from_json(const wpi::json& j, DetectionResultClass& t) {
    t.m_confidence = SafeJSONAccess<double>(j, internal::_key_confidence, 0.0);
    t.m_classID = SafeJSONAccess<double>(j, internal::_key_classID, 0.0);
    t.m_className = SafeJSONAccess<std::string>(j, internal::_key_className, "");
    t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
    t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
    t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
    t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
    t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
    t.m_TargetCorners =
        SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
  }

  /**
     * Represents a Barcode Target Result extracted from JSON Output
     */
  inline void from_json(const wpi::json& j, BarcodeResultClass& t) {
    /**
         * Barcode family type (e.g. "QR", "DataMatrix", etc.)
         */
    t.m_family = SafeJSONAccess<std::string>(j, internal::_key_ffamily, "");

    /**
         * Gets the decoded data content of the barcode
         */
    t.m_data = SafeJSONAccess<std::string>(j, internal::_key_data, "");
    t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
    t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);

    t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
    t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);

    t.m_TargetXDegreesNoCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesNoCrosshair, 0.0);
    t.m_TargetYDegreesNoCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesNoCrosshair, 0.0);
    t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
    t.m_TargetCorners =
        SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
  }

  /**
     * Represents a Neural Classifier Pipeline Result extracted from JSON Output
     */
  inline void from_json(const wpi::json& j, ClassificationResultClass& t) {
    t.m_confidence = SafeJSONAccess<double>(j, internal::_key_confidence, 0.0);
    t.m_classID = SafeJSONAccess<double>(j, internal::_key_classID, 0.0);
    t.m_className = SafeJSONAccess<std::string>(j, internal::_key_className, "");
    t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
    t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
    t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
    t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
    t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
    t.m_TargetCorners =
        SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
  }

  inline void from_json(const wpi::json& j, VisionResultsClass& t) {
    t.m_timeStamp = SafeJSONAccess<double>(j, internal::_key_timestamp, 0.0);
    t.m_latencyPipeline = SafeJSONAccess<double>(j, internal::_key_latency_pipeline, 0.0);
    t.m_latencyCapture = SafeJSONAccess<double>(j, internal::_key_latency_capture, 0.0);
    t.m_pipelineIndex = SafeJSONAccess<double>(j, internal::_key_pipelineIndex, 0.0);
    t.valid = SafeJSONAccess<double>(j, "v", 0.0);

    std::vector<double> defaultVector{};
    t.botPose = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose, defaultVector);
    t.botPose_wpired = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose_wpired, defaultVector);
    t.botPose_wpiblue = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose_wpiblue, defaultVector);

    t.RetroResults =
        SafeJSONAccess<std::vector<RetroreflectiveResultClass>>(j, "Retro", std::vector<RetroreflectiveResultClass>{});
    t.FiducialResults =
        SafeJSONAccess<std::vector<FiducialResultClass>>(j, "Fiducial", std::vector<FiducialResultClass>{});
    t.DetectionResults =
        SafeJSONAccess<std::vector<DetectionResultClass>>(j, "Detector", std::vector<DetectionResultClass>{});
    t.ClassificationResults =
        SafeJSONAccess<std::vector<ClassificationResultClass>>(j, "Detector", std::vector<ClassificationResultClass>{});
    t.BarcodeResults = SafeJSONAccess<std::vector<BarcodeResultClass>>(j, "Barcode", std::vector<BarcodeResultClass>{});
  }

  /**
     * Limelight Results object, parsed from a Limelight's JSON results output.
     */
  inline void from_json(const wpi::json& j, LimelightResultsClass& t) {
    t.targetingResults =
        SafeJSONAccess<LimelightHelpers::VisionResultsClass>(j, "Results", LimelightHelpers::VisionResultsClass{});
  }

  inline LimelightResultsClass getLatestResults(const std::string& limelightName = "", bool profile = false) {
    auto start = std::chrono::high_resolution_clock::now();
    std::string jsonString = getJSONDump(limelightName);
    wpi::json data;
    try {
      data = wpi::json::parse(jsonString);
    } catch (const std::exception& e) {
      return LimelightResultsClass();
    }

    auto end = std::chrono::high_resolution_clock::now();
    double nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    double millis = (nanos * 0.000001);
    try {
      LimelightResultsClass out = data.get<LimelightHelpers::LimelightResultsClass>();
      out.targetingResults.m_latencyJSON = millis;
      if (profile) {
        std::cout << "lljson: " << millis << std::endl;
      }
      return out;
    } catch (...) {
      return LimelightResultsClass();
    }
  }
}  // namespace LimelightHelpers
