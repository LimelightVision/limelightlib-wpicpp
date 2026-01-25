#pragma once

///
/// https://github.com/LimelightVision/limelightlib-wpicpp
///
// V1.14 - Requires 2026.0


// #include <curl/curl.h>
#include <fcntl.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <wpinet/PortForwarder.h>

#include <array>
#include <chrono>
#include <cstring>
#include <iostream>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include "networktables/DoubleArrayTopic.h"
#include "wpi/json.h"
#include <wpi/print.h>

/**
 * LimelightHelpers provides static methods and classes for interfacing with Limelight vision cameras in FRC.
 * This library supports all Limelight features including AprilTag tracking, Neural Networks, and standard color/retroreflective tracking.
 */
namespace LimelightHelpers {
  /**
   * Sanitizes the Limelight name, returning "limelight" if empty.
   * @param name The Limelight name to sanitize
   * @return Sanitized name, defaults to "limelight" if empty
   */
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

  /**
   * Gets the NetworkTable for a Limelight camera.
   * @param tableName Name of the Limelight camera
   * @return Shared pointer to the NetworkTable
   */
  inline std::shared_ptr<nt::NetworkTable> getLimelightNTTable(const std::string& tableName) {
    return nt::NetworkTableInstance::GetDefault().GetTable(sanitizeName(tableName));
  }

  /**
   * Flushes NetworkTables to ensure data is sent immediately.
   * Call after setting values that need to be sent to the Limelight immediately.
   */
  inline void Flush() {
    nt::NetworkTableInstance::GetDefault().Flush();
  }

  /**
   * Gets a NetworkTable entry for a Limelight camera.
   * @param tableName Name of the Limelight camera
   * @param entryName Name of the entry to retrieve
   * @return NetworkTableEntry for the specified entry
   */
  inline nt::NetworkTableEntry getLimelightNTTableEntry(const std::string& tableName, const std::string& entryName) {
    return getLimelightNTTable(tableName)->GetEntry(entryName);
  }

  /// @cond INTERNAL
  inline std::unordered_map<std::string, nt::DoubleArrayEntry> doubleArrayEntries;
  /// @endcond

  /**
   * Gets a cached DoubleArrayEntry for efficient repeated access.
   * @param tableName Name of the Limelight camera
   * @param entryName Name of the entry to retrieve
   * @return Reference to the DoubleArrayEntry
   */
  inline nt::DoubleArrayEntry& getLimelightDoubleArrayEntry(const std::string& tableName,
                                                            const std::string& entryName) {
    const std::string key = tableName + "/" + entryName;
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

  /**
   * Gets a double value from NetworkTables.
   * @param tableName Name of the Limelight camera
   * @param entryName Name of the entry to retrieve
   * @return The double value, or 0.0 if not found
   */
  inline double getLimelightNTDouble(const std::string& tableName, const std::string& entryName) {
    return getLimelightNTTableEntry(tableName, entryName).GetDouble(0.0);
  }

  /**
   * Gets a double array from NetworkTables.
   * @param tableName Name of the Limelight camera
   * @param entryName Name of the entry to retrieve
   * @return Vector of doubles, or empty vector if not found
   */
  inline std::vector<double> getLimelightNTDoubleArray(const std::string& tableName, const std::string& entryName) {
    return getLimelightNTTableEntry(tableName, entryName).GetDoubleArray(std::span<double>{});
  }

  /**
   * Gets a string value from NetworkTables.
   * @param tableName Name of the Limelight camera
   * @param entryName Name of the entry to retrieve
   * @return The string value, or empty string if not found
   */
  inline std::string getLimelightNTString(const std::string& tableName, const std::string& entryName) {
    return getLimelightNTTableEntry(tableName, entryName).GetString("");
  }

  /**
   * Gets a string array from NetworkTables.
   * @param tableName Name of the Limelight camera
   * @param entryName Name of the entry to retrieve
   * @return Vector of strings, or empty vector if not found
   */
  inline std::vector<std::string> getLimelightNTStringArray(const std::string& tableName,
                                                            const std::string& entryName) {
    return getLimelightNTTableEntry(tableName, entryName).GetStringArray(std::span<std::string>{});
  }

  /**
   * Sets a double value in NetworkTables.
   * @param tableName Name of the Limelight camera
   * @param entryName Name of the entry to set
   * @param val The value to set
   */
  inline void setLimelightNTDouble(const std::string& tableName, const std::string& entryName, double val) {
    getLimelightNTTableEntry(tableName, entryName).SetDouble(val);
  }

  /**
   * Sets a double array in NetworkTables.
   * @param tableName Name of the Limelight camera
   * @param entryName Name of the entry to set
   * @param vals The values to set
   */
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
  inline double getTXNC(const std::string& limelightName = "") {
    return getLimelightNTDouble(limelightName, "txnc");
  }

  /**
     * Gets the vertical offset from the principal pixel/point to the target in degrees. This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Vertical offset angle in degrees
     */
  inline double getTYNC(const std::string& limelightName = "") {
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
     * T2D is an array that contains several targeting metrics
     * @param limelightName Name of the Limelight camera
     * @return Array containing [0:targetValid, 1:targetCount, 2:targetLatency, 3:captureLatency, 4:tx, 5:ty, 6:txnc, 7:tync, 8:ta, 9:tid, 10:targetClassIndexDetector,
     * 11:targetClassIndexClassifier, 12:targetLongSidePixels, 13:targetShortSidePixels, 14:targetHorizontalExtentPixels, 15:targetVerticalExtentPixels, 16:targetSkewDegrees]
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
      return (int)t2d[11];
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
      return (int)t2d[10];
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

  /**
   * Gets the robot pose in field-space as a double array.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Array [x, y, z, roll, pitch, yaw] in meters and degrees
   */
  inline std::vector<double> getBotpose(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "botpose");
  }

  /**
   * Gets the robot pose in WPILib Red Alliance field-space as a double array.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Array [x, y, z, roll, pitch, yaw] in meters and degrees
   */
  inline std::vector<double> getBotpose_wpiRed(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
  }

  /**
   * Gets the robot pose in WPILib Blue Alliance field-space as a double array.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Array [x, y, z, roll, pitch, yaw] in meters and degrees
   */
  inline std::vector<double> getBotpose_wpiBlue(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
  }

  /**
   * Gets the robot pose relative to the target as a double array.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Array [x, y, z, roll, pitch, yaw] in meters and degrees
   */
  inline std::vector<double> getBotpose_TargetSpace(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
  }

  /**
   * Gets the camera pose relative to the target as a double array.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Array [x, y, z, roll, pitch, yaw] in meters and degrees
   */
  inline std::vector<double> getCameraPose_TargetSpace(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
  }

  /**
   * Gets the camera pose relative to the robot as a double array.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Array [x, y, z, roll, pitch, yaw] in meters and degrees
   */
  inline std::vector<double> getCameraPose_RobotSpace(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "camerapose_robotspace");
  }

  /**
   * Gets the target pose relative to the camera as a double array.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Array [x, y, z, roll, pitch, yaw] in meters and degrees
   */
  inline std::vector<double> getTargetPose_CameraSpace(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
  }

  /**
   * Gets the target pose relative to the robot as a double array.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Array [x, y, z, roll, pitch, yaw] in meters and degrees
   */
  inline std::vector<double> getTargetPose_RobotSpace(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
  }

  /**
   * Gets the target color as RGB values.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Array containing RGB color values
   */
  inline std::vector<double> getTargetColor(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "tc");
  }

  /**
   * Gets the ID of the primary AprilTag in view.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Fiducial/AprilTag ID, or 0 if none detected
   */
  inline double getFiducialID(const std::string& limelightName = "") {
    return getLimelightNTDouble(limelightName, "tid");
  }

  /////
  ///// Pose3d Getters
  /////

  /**
   * Gets the robot's 3D pose in field-space.
   * @param limelightName Name/identifier of the Limelight
   * @return Pose3d object representing the robot's position and orientation
   */
  inline frc::Pose3d getBotPose3d(const std::string& limelightName = "") {
    std::vector<double> poseArray = getLimelightNTDoubleArray(limelightName, "botpose");
    return toPose3D(poseArray);
  }

  /**
   * (Not Recommended) Gets the robot's 3D pose in the WPILib Red Alliance Coordinate System.
   * @param limelightName Name/identifier of the Limelight
   * @return Pose3d object representing the robot's position and orientation in Red Alliance field space
   */
  inline frc::Pose3d getBotPose3d_wpiRed(const std::string& limelightName = "") {
    std::vector<double> poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    return toPose3D(poseArray);
  }

  /**
   * (Recommended) Gets the robot's 3D pose in the WPILib Blue Alliance Coordinate System.
   * @param limelightName Name/identifier of the Limelight
   * @return Pose3d object representing the robot's position and orientation in Blue Alliance field space
   */
  inline frc::Pose3d getBotPose3d_wpiBlue(const std::string& limelightName = "") {
    std::vector<double> poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    return toPose3D(poseArray);
  }

  /**
   * Gets the robot's 3D pose with respect to the currently tracked target's coordinate system.
   * @param limelightName Name/identifier of the Limelight
   * @return Pose3d object representing the robot's position and orientation relative to the target
   */
  inline frc::Pose3d getBotPose3d_TargetSpace(const std::string& limelightName = "") {
    std::vector<double> poseArray = getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
    return toPose3D(poseArray);
  }

  /**
   * Gets the camera's 3D pose with respect to the currently tracked target's coordinate system.
   * @param limelightName Name/identifier of the Limelight
   * @return Pose3d object representing the camera's position and orientation relative to the target
   */
  inline frc::Pose3d getCameraPose3d_TargetSpace(const std::string& limelightName = "") {
    std::vector<double> poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
    return toPose3D(poseArray);
  }

  /**
   * Gets the target's 3D pose with respect to the camera's coordinate system.
   * @param limelightName Name/identifier of the Limelight
   * @return Pose3d object representing the target's position and orientation relative to the camera
   */
  inline frc::Pose3d getTargetPose3d_CameraSpace(const std::string& limelightName = "") {
    std::vector<double> poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
    return toPose3D(poseArray);
  }

  /**
   * Gets the target's 3D pose with respect to the robot's coordinate system.
   * @param limelightName Name/identifier of the Limelight
   * @return Pose3d object representing the target's position and orientation relative to the robot
   */
  inline frc::Pose3d getTargetPose3d_RobotSpace(const std::string& limelightName = "") {
    std::vector<double> poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    return toPose3D(poseArray);
  }

  /**
   * Gets the camera's 3D pose with respect to the robot's coordinate system.
   * @param limelightName Name/identifier of the Limelight
   * @return Pose3d object representing the camera's position and orientation relative to the robot
   */
  inline frc::Pose3d getCameraPose3d_RobotSpace(const std::string& limelightName = "") {
    std::vector<double> poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_robotspace");
    return toPose3D(poseArray);
  }

  /**
   * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
   * @param limelightName Name/identifier of the Limelight
   * @return Pose2d object representing the robot's position in Blue Alliance field space
   */
  inline frc::Pose2d getBotPose2d_wpiBlue(const std::string& limelightName = "") {
    std::vector<double> result = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    return toPose2D(result);
  }

  /**
   * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
   * @param limelightName Name/identifier of the Limelight
   * @return Pose2d object representing the robot's position in Red Alliance field space
   */
  inline frc::Pose2d getBotPose2d_wpiRed(const std::string& limelightName = "") {
    std::vector<double> result = getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    return toPose2D(result);
  }

  /**
   * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
   * @param limelightName Name/identifier of the Limelight
   * @return Pose2d object representing the robot's position
   */
  inline frc::Pose2d getBotPose2d(const std::string& limelightName = "") {
    std::vector<double> result = getLimelightNTDoubleArray(limelightName, "botpose");
    return toPose2D(result);
  }

  /**
   * Gets the neural network class name for the primary detected object.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Class name string from neural network pipeline
   */
  inline std::string getNeuralClassID(const std::string& limelightName = "") {
    return getLimelightNTString(limelightName, "tclass");
  }

  /**
   * Gets raw barcode data from the barcode scanner pipeline.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Vector of decoded barcode strings
   */
  inline std::vector<std::string> getRawBarcodeData(const std::string& limelightName = "") {
    return getLimelightNTStringArray(limelightName, "rawbarcodes");
  }

  /**
   * Gets the Limelight heartbeat value. Increments once per frame, allowing you to detect if the Limelight is
   * connected and alive.
   * @param limelightName Name of the Limelight camera
   * @return Heartbeat value that increments each frame
   */
  inline double getHeartbeat(const std::string& limelightName = "") {
    return getLimelightNTDouble(limelightName, "hb");
  }

  /**
   * Gets the corner coordinates of detected targets from NetworkTables.
   * Requires "send contours" to be enabled in the Limelight Output tab.
   *
   * @param limelightName Name/identifier of the Limelight
   * @return Array of doubles containing corner coordinates [x0, y0, x1, y1, ...]
   */
  inline std::vector<double> getCornerCoordinates(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "tcornxy");
  }

  /**
   * Sets the active pipeline index on the Limelight.
   * @param limelightName Name of the Limelight camera
   * @param index Pipeline index (0-9)
   */
  inline void setPipelineIndex(const std::string& limelightName, int index) {
    setLimelightNTDouble(limelightName, "pipeline", index);
  }

  /**
   * Sets the priority AprilTag ID for the Limelight to track.
   * @param limelightName Name of the Limelight camera
   * @param ID The AprilTag ID to prioritize
   */
  inline void setPriorityTagID(const std::string& limelightName, int ID) {
    setLimelightNTDouble(limelightName, "priorityid", ID);
  }

  /**
   * Sets LED mode to be controlled by the current pipeline.
   * @param limelightName Name of the Limelight camera ("" for default)
   */
  inline void setLEDMode_PipelineControl(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "ledMode", 0);
  }

  /**
   * Forces the LEDs off.
   * @param limelightName Name of the Limelight camera ("" for default)
   */
  inline void setLEDMode_ForceOff(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "ledMode", 1);
  }

  /**
   * Forces the LEDs to blink.
   * @param limelightName Name of the Limelight camera ("" for default)
   */
  inline void setLEDMode_ForceBlink(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "ledMode", 2);
  }

  /**
   * Forces the LEDs on.
   * @param limelightName Name of the Limelight camera ("" for default)
   */
  inline void setLEDMode_ForceOn(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "ledMode", 3);
  }

  /**
   * Sets the stream mode to standard (side-by-side).
   * @param limelightName Name of the Limelight camera ("" for default)
   */
  inline void setStreamMode_Standard(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "stream", 0);
  }

  /**
   * Sets the stream mode to Picture-in-Picture with the secondary camera in the corner.
   * @param limelightName Name of the Limelight camera ("" for default)
   */
  inline void setStreamMode_PiPMain(const std::string& limelightName = "") {
    setLimelightNTDouble(limelightName, "stream", 1);
  }

  /**
   * Sets the stream mode to Picture-in-Picture with the primary camera in the corner.
   * @param limelightName Name of the Limelight camera ("" for default)
   */
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

  /// @cond INTERNAL
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
  /// @endcond

  /**
   * Sets the robot orientation for MegaTag2 localization.
   * This should be called every frame with your robot's current orientation from a gyro.
   * @param limelightName Name of the Limelight camera
   * @param yaw Robot yaw in degrees (typically from gyro)
   * @param yawRate Robot yaw rate in degrees per second
   * @param pitch Robot pitch in degrees
   * @param pitchRate Robot pitch rate in degrees per second
   * @param roll Robot roll in degrees
   * @param rollRate Robot roll rate in degrees per second
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

  /**
   * Sets the robot orientation for MegaTag2 without flushing NetworkTables.
   * Use this for better performance when setting multiple values before flushing.
   * @param limelightName Name of the Limelight camera
   * @param yaw Robot yaw in degrees
   * @param yawRate Robot yaw rate in degrees per second
   * @param pitch Robot pitch in degrees
   * @param pitchRate Robot pitch rate in degrees per second
   * @param roll Robot roll in degrees
   * @param rollRate Robot roll rate in degrees per second
   */
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
   * Sets the keystone modification for the crop window.
   * @param limelightName Name of the Limelight camera
   * @param horizontal Horizontal keystone value (-0.95 to 0.95)
   * @param vertical Vertical keystone value (-0.95 to 0.95)
   */
  inline void setKeystone(const std::string& limelightName, double horizontal, double vertical) {
    std::array<double, 2> entries;
    entries[0] = horizontal;
    entries[1] = vertical;
    setLimelightNTDoubleArray(limelightName, "keystone_set", entries);
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
   * Configures the complementary filter alpha value for IMU Assist Modes (Modes 3 and 4)
   *
   * @param limelightName Name/identifier of the Limelight
   * @param alpha Defaults to .001. Higher values will cause the internal IMU to converge onto the assist source more
   * rapidly.
   */
  inline void SetIMUAssistAlpha(const std::string& limelightName, double alpha) {
    setLimelightNTDouble(limelightName, "imuassistalpha_set", alpha);
  }

  /**
   * Configures the throttle value. Set to 100-200 while disabled to reduce temp.
   *
   * @param limelightName Name/identifier of the Limelight
   * @param throttle Defaults to 0. Your Limelight will process one frame after skipping <throttle> frames.
   */
  inline void SetThrottle(const std::string& limelightName, int throttle) {
    setLimelightNTDouble(limelightName, "throttle_set", throttle);
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
  inline void SetFiducial3DOffset(const std::string& limelightName, double x, double y, double z) {
    std::array<double, 3> entries;
    entries[0] = x;
    entries[1] = y;
    entries[2] = z;
    setLimelightNTDoubleArray(limelightName, "fiducial_offset_set", entries);
  }

  /**
   * Overrides the valid AprilTag IDs for the current pipeline.
   * Only tags with IDs in this list will be used for localization.
   * @param limelightName Name of the Limelight camera
   * @param validIDs Vector of valid AprilTag IDs to track
   */
  inline void SetFiducialIDFiltersOverride(const std::string& limelightName, const std::vector<int>& validIDs) {
    std::vector<double> validIDsDouble(validIDs.begin(), validIDs.end());
    setLimelightNTDoubleArray(limelightName, "fiducial_id_filters_set", validIDsDouble);
  }

  /**
   * Sets the downscaling factor for AprilTag detection.
   * Increasing downscale can improve performance at the cost of potentially reduced detection range.
   *
   * @param limelightName Name/identifier of the Limelight
   * @param downscale Downscale factor. Valid values: 1.0 (no downscale), 1.5, 2.0, 3.0, 4.0. Set to 0 for pipeline
   * control.
   */
  inline void SetFiducialDownscalingOverride(const std::string& limelightName, float downscale) {
    int d = 0;  // pipeline
    if (downscale == 1.0f) {
      d = 1;
    }
    if (downscale == 1.5f) {
      d = 2;
    }
    if (downscale == 2.0f) {
      d = 3;
    }
    if (downscale == 3.0f) {
      d = 4;
    }
    if (downscale == 4.0f) {
      d = 5;
    }
    setLimelightNTDouble(limelightName, "fiducial_downscale_set", d);
  }

  /**
   * Sets the camera pose relative to the robot.
   * The UI camera pose must be set to zeros for this to take effect.
   * @param limelightName Name of the Limelight camera
   * @param forward Forward offset in meters (positive = forward)
   * @param side Side offset in meters (positive = left)
   * @param up Up offset in meters (positive = up)
   * @param roll Roll angle in degrees
   * @param pitch Pitch angle in degrees
   * @param yaw Yaw angle in degrees
   */
  inline void setCameraPose_RobotSpace(
      const std::string& limelightName, double forward, double side, double up, double roll, double pitch, double yaw) {
    double entries[6] = {forward, side, up, roll, pitch, yaw};
    setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
  }

  /**
   * Sends data to a Python SnapScript running on the Limelight.
   * @param limelightName Name of the Limelight camera
   * @param outgoingPythonData Vector of doubles to send to the Python script
   */
  inline void setPythonScriptData(const std::string& limelightName, const std::vector<double>& outgoingPythonData) {
    setLimelightNTDoubleArray(
        limelightName, "llrobot", std::span{outgoingPythonData.begin(), outgoingPythonData.size()});
  }

  /**
   * Gets data from a Python SnapScript running on the Limelight.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return Vector of doubles from the Python script
   */
  inline std::vector<double> getPythonScriptData(const std::string& limelightName = "") {
    return getLimelightNTDoubleArray(limelightName, "llpython");
  }

  /////
  ///// Snapshot and Rewind functions
  /////

  /**
   * Triggers a snapshot capture via NetworkTables by incrementing the snapshot counter.
   * Rate-limited to once per 10 frames on the Limelight.
   * @param limelightName Name of the Limelight camera
   */
  inline void triggerSnapshot(const std::string& limelightName = "") {
    double current = getLimelightNTDouble(limelightName, "snapshot");
    setLimelightNTDouble(limelightName, "snapshot", current + 1);
  }

  /**
   * Enables or pauses the rewind buffer recording.
   * @param limelightName Name of the Limelight camera
   * @param enabled True to enable recording, false to pause
   */
  inline void setRewindEnabled(const std::string& limelightName, bool enabled) {
    setLimelightNTDouble(limelightName, "rewind_enable_set", enabled ? 1 : 0);
  }

  /**
   * Triggers a rewind capture with the specified duration.
   * Maximum duration is 165 seconds. Rate-limited on the Limelight.
   * @param limelightName Name of the Limelight camera
   * @param durationSeconds Duration of rewind capture in seconds (max 165)
   */
  inline void triggerRewindCapture(const std::string& limelightName, double durationSeconds) {
    std::vector<double> currentArray = getLimelightNTDoubleArray(limelightName, "capture_rewind");
    double counter = (currentArray.empty()) ? 0 : currentArray[0];
    std::array<double, 2> entries;
    entries[0] = counter + 1;
    entries[1] = std::min(durationSeconds, 165.0);
    setLimelightNTDoubleArray(limelightName, "capture_rewind", entries);
  }

  /////

  /// @cond INTERNAL
  inline double extractEntry(const std::vector<double>& inData, int position) {
    if (inData.size() < static_cast<size_t>(position + 1)) {
      return 0.0;
    }
    return inData[position];
  }
  /// @endcond

  /**
   * Represents a Limelight Raw Fiducial result from Limelight's NetworkTables output.
   */
  class RawFiducial {
   public:
    int id{0};
    double txnc{0.0};
    double tync{0.0};
    double ta{0.0};
    double distToCamera{0.0};
    double distToRobot{0.0};
    double ambiguity{0.0};

    RawFiducial() = default;

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
   * Represents a Limelight Raw Target/Contour result from Limelight's NetworkTables output.
   */
  class RawTarget {
   public:
    double txnc{0.0};
    double tync{0.0};
    double ta{0.0};

    RawTarget() = default;

    RawTarget(double txnc, double tync, double ta)
        : txnc(txnc)
        , tync(tync)
        , ta(ta) {}
  };

  /**
   * Represents a Limelight Raw Neural Detector result from Limelight's NetworkTables output.
   */
  class RawDetection {
   public:
    int classId{0};
    double txnc{0.0};
    double tync{0.0};
    double ta{0.0};
    double corner0_X{0.0};
    double corner0_Y{0.0};
    double corner1_X{0.0};
    double corner1_Y{0.0};
    double corner2_X{0.0};
    double corner2_Y{0.0};
    double corner3_X{0.0};
    double corner3_Y{0.0};

    RawDetection() = default;

    RawDetection(int classId,
                 double txnc,
                 double tync,
                 double ta,
                 double corner0_X,
                 double corner0_Y,
                 double corner1_X,
                 double corner1_Y,
                 double corner2_X,
                 double corner2_Y,
                 double corner3_X,
                 double corner3_Y)
        : classId(classId)
        , txnc(txnc)
        , tync(tync)
        , ta(ta)
        , corner0_X(corner0_X)
        , corner0_Y(corner0_Y)
        , corner1_X(corner1_X)
        , corner1_Y(corner1_Y)
        , corner2_X(corner2_X)
        , corner2_Y(corner2_Y)
        , corner3_X(corner3_X)
        , corner3_Y(corner3_Y) {}
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
    bool isMegaTag2{false};

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

  /**
   * Checks if a PoseEstimate contains valid data.
   * A pose is considered valid if it contains at least one raw fiducial.
   * @param pose The PoseEstimate to validate
   * @return True if the pose estimate is valid, false otherwise
   */
  inline bool validPoseEstimate(const PoseEstimate& pose) {
    return !pose.rawFiducials.empty();
  }

  /**
   * Prints detailed information about a PoseEstimate to stdout.
   * @param pose The PoseEstimate to print
   */
  inline void PrintPoseEstimate(const PoseEstimate& pose) {
    wpi::print("Pose Estimate Information:\n");
    wpi::print("Timestamp (Seconds): {:.3f}\n", pose.timestampSeconds.value());
    wpi::print("Latency: {:.3f} ms\n", pose.latency);
    wpi::print("Tag Count: {}\n", pose.tagCount);
    wpi::print("Tag Span: {:.2f} meters\n", pose.tagSpan);
    wpi::print("Average Tag Distance: {:.2f} meters\n", pose.avgTagDist);
    wpi::print("Average Tag Area: {:.2f}% of image\n", pose.avgTagArea);
    wpi::print("Is MegaTag2: {}\n", pose.isMegaTag2 ? "true" : "false");
    wpi::print("\n");

    if (pose.rawFiducials.empty()) {
      wpi::print("No RawFiducials data available.\n");
      return;
    }

    wpi::print("Raw Fiducials Details:\n");
    for (size_t i = 0; i < pose.rawFiducials.size(); i++) {
      const auto& fiducial = pose.rawFiducials[i];
      wpi::print(" Fiducial #{}:\n", i + 1);
      wpi::print("  ID: {}\n", fiducial.id);
      wpi::print("  TXNC: {:.2f}\n", fiducial.txnc);
      wpi::print("  TYNC: {:.2f}\n", fiducial.tync);
      wpi::print("  TA: {:.2f}\n", fiducial.ta);
      wpi::print("  Distance to Camera: {:.2f} meters\n", fiducial.distToCamera);
      wpi::print("  Distance to Robot: {:.2f} meters\n", fiducial.distToRobot);
      wpi::print("  Ambiguity: {:.2f}\n", fiducial.ambiguity);
      wpi::print("\n");
    }
  }

  /**
     * Encapsulates the state of an internal Limelight IMU.
     */
  class IMUData {
   public:
    double robotYaw{0.0};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
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
        roll = imuData[1];
        pitch = imuData[2];
        yaw = imuData[3];
        gyroX = imuData[4];
        gyroY = imuData[5];
        gyroZ = imuData[6];
        accelX = imuData[7];
        accelY = imuData[8];
        accelZ = imuData[9];
      }
    }
  };

  /// @cond INTERNAL
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

    double latency = extractEntry(poseArray, 6);
    int tagCount = static_cast<int>(extractEntry(poseArray, 7));
    double tagSpan = extractEntry(poseArray, 8);
    double tagDist = extractEntry(poseArray, 9);
    double tagArea = extractEntry(poseArray, 10);

    double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

    std::vector<RawFiducial> rawFiducials;
    int valsPerFiducial = 7;
    size_t expectedTotalVals = 11 + valsPerFiducial * tagCount;

    if (poseArray.size() == expectedTotalVals) {
      for (int i = 0; i < tagCount; i++) {
        int baseIndex = 11 + (i * valsPerFiducial);
        int id = static_cast<int>(extractEntry(poseArray, baseIndex));
        double txnc = extractEntry(poseArray, baseIndex + 1);
        double tync = extractEntry(poseArray, baseIndex + 2);
        double ta = extractEntry(poseArray, baseIndex + 3);
        double distToCamera = extractEntry(poseArray, baseIndex + 4);
        double distToRobot = extractEntry(poseArray, baseIndex + 5);
        double ambiguity = extractEntry(poseArray, baseIndex + 6);
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
  /// @endcond

  /**
   * Gets the robot pose estimate in WPILib Blue Alliance field-space.
   * This is the recommended method for getting robot pose for vision-based localization.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return PoseEstimate containing pose, timestamp, and tag information
   */
  inline PoseEstimate getBotPoseEstimate_wpiBlue(const std::string& limelightName = "") {
    return getBotPoseEstimate(limelightName, "botpose_wpiblue", false);
  }

  /**
   * Gets the robot pose estimate in WPILib Red Alliance field-space.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return PoseEstimate containing pose, timestamp, and tag information
   */
  inline PoseEstimate getBotPoseEstimate_wpiRed(const std::string& limelightName = "") {
    return getBotPoseEstimate(limelightName, "botpose_wpired", false);
  }

  /**
   * Gets the robot pose estimate using MegaTag2 in WPILib Blue Alliance field-space.
   * MegaTag2 uses the robot's IMU orientation for improved accuracy.
   * Requires SetRobotOrientation() to be called each frame.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return PoseEstimate containing pose, timestamp, and tag information
   */
  inline PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(const std::string& limelightName = "") {
    return getBotPoseEstimate(limelightName, "botpose_orb_wpiblue", true);
  }

  /**
   * Gets the robot pose estimate using MegaTag2 in WPILib Red Alliance field-space.
   * MegaTag2 uses the robot's IMU orientation for improved accuracy.
   * Requires SetRobotOrientation() to be called each frame.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @return PoseEstimate containing pose, timestamp, and tag information
   */
  inline PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(const std::string& limelightName = "") {
    return getBotPoseEstimate(limelightName, "botpose_orb_wpired", true);
  }

  /**
   * Gets the latest raw fiducial/AprilTag detection results from NetworkTables.
   *
   * @param limelightName Name/identifier of the Limelight
   * @return Vector of RawFiducial objects containing detection details
   */
  inline std::vector<RawFiducial> getRawFiducials(const std::string& limelightName = "") {
    std::vector<double> rawFiducialArray = getLimelightNTDoubleArray(limelightName, "rawfiducials");
    int valsPerEntry = 7;
    if (rawFiducialArray.size() % valsPerEntry != 0) {
      return {};
    }

    int numFiducials = rawFiducialArray.size() / valsPerEntry;
    std::vector<RawFiducial> rawFiducials;

    for (int i = 0; i < numFiducials; ++i) {
      int baseIndex = i * valsPerEntry;
      int id = static_cast<int>(extractEntry(rawFiducialArray, baseIndex));
      double txnc = extractEntry(rawFiducialArray, baseIndex + 1);
      double tync = extractEntry(rawFiducialArray, baseIndex + 2);
      double ta = extractEntry(rawFiducialArray, baseIndex + 3);
      double distToCamera = extractEntry(rawFiducialArray, baseIndex + 4);
      double distToRobot = extractEntry(rawFiducialArray, baseIndex + 5);
      double ambiguity = extractEntry(rawFiducialArray, baseIndex + 6);

      rawFiducials.emplace_back(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
    }

    return rawFiducials;
  }

  /**
   * Gets the latest raw neural detector results from NetworkTables
   *
   * @param limelightName Name/identifier of the Limelight
   * @return Vector of RawDetection objects containing detection details
   */
  inline std::vector<RawDetection> getRawDetections(const std::string& limelightName = "") {
    std::vector<double> rawDetectionArray = getLimelightNTDoubleArray(limelightName, "rawdetections");
    int valsPerEntry = 12;
    if (rawDetectionArray.size() % valsPerEntry != 0) {
      return {};
    }

    int numDetections = rawDetectionArray.size() / valsPerEntry;
    std::vector<RawDetection> rawDetections;

    for (int i = 0; i < numDetections; ++i) {
      int baseIndex = i * valsPerEntry;
      int classId = static_cast<int>(extractEntry(rawDetectionArray, baseIndex));
      double txnc = extractEntry(rawDetectionArray, baseIndex + 1);
      double tync = extractEntry(rawDetectionArray, baseIndex + 2);
      double ta = extractEntry(rawDetectionArray, baseIndex + 3);
      double corner0_X = extractEntry(rawDetectionArray, baseIndex + 4);
      double corner0_Y = extractEntry(rawDetectionArray, baseIndex + 5);
      double corner1_X = extractEntry(rawDetectionArray, baseIndex + 6);
      double corner1_Y = extractEntry(rawDetectionArray, baseIndex + 7);
      double corner2_X = extractEntry(rawDetectionArray, baseIndex + 8);
      double corner2_Y = extractEntry(rawDetectionArray, baseIndex + 9);
      double corner3_X = extractEntry(rawDetectionArray, baseIndex + 10);
      double corner3_Y = extractEntry(rawDetectionArray, baseIndex + 11);

      rawDetections.emplace_back(
          classId, txnc, tync, ta, corner0_X, corner0_Y, corner1_X, corner1_Y, corner2_X, corner2_Y, corner3_X,
          corner3_Y);
    }

    return rawDetections;
  }

  /**
   * Gets the raw target contours from NetworkTables.
   * Returns ungrouped contours in normalized screen space (-1 to 1).
   *
   * @param limelightName Name/identifier of the Limelight
   * @return Vector of RawTarget objects containing up to 3 contours
   */
  inline std::vector<RawTarget> getRawTargets(const std::string& limelightName = "") {
    std::vector<double> rawTargetArray = getLimelightNTDoubleArray(limelightName, "rawtargets");
    int valsPerEntry = 3;
    if (rawTargetArray.size() % valsPerEntry != 0) {
      return {};
    }

    int numTargets = rawTargetArray.size() / valsPerEntry;
    std::vector<RawTarget> rawTargets;

    for (int i = 0; i < numTargets; ++i) {
      int baseIndex = i * valsPerEntry;
      double txnc = extractEntry(rawTargetArray, baseIndex);
      double tync = extractEntry(rawTargetArray, baseIndex + 1);
      double ta = extractEntry(rawTargetArray, baseIndex + 2);

      rawTargets.emplace_back(txnc, tync, ta);
    }

    return rawTargets;
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

  /// Constant representing an invalid target value
  inline const double INVALID_TARGET = 0.0;

  /**
   * Base class for all targeting results from JSON output.
   * Contains common targeting metrics shared by all pipeline types.
   */
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

  /**
   * Represents a retroreflective/color target result from JSON output.
   */
  class RetroreflectiveResultClass : public SingleTargetingResultClass {};

  /**
   * Represents an AprilTag/fiducial target result from JSON output.
   */
  class FiducialResultClass : public SingleTargetingResultClass {
   public:
    int m_fiducialID{0};
    std::string m_family{""};
  };

  /**
   * Represents a barcode target result from JSON output.
   */
  class BarcodeResultClass : public SingleTargetingResultClass {
   public:
    std::string m_family{""};   ///< Barcode family type (e.g., "QR", "DataMatrix")
    std::string m_data{""};     ///< Decoded barcode data

    double m_TargetXDegreesNoCrosshairAdjusted{INVALID_TARGET};  ///< Horizontal offset from principal point
    double m_TargetYDegreesNoCrosshairAdjusted{INVALID_TARGET};  ///< Vertical offset from principal point
  };

  /**
   * Represents a neural network detector result from JSON output.
   */
  class DetectionResultClass : public SingleTargetingResultClass {
   public:
    int m_classID{-1};          ///< Class ID from the neural network
    std::string m_className{""};  ///< Class name from the neural network
    double m_confidence{0};     ///< Detection confidence (0-1)

    double m_TargetXDegreesNoCrosshairAdjusted{INVALID_TARGET};  ///< Horizontal offset from principal point
    double m_TargetYDegreesNoCrosshairAdjusted{INVALID_TARGET};  ///< Vertical offset from principal point
  };

  /**
   * Represents a neural network classifier result from JSON output.
   */
  class ClassificationResultClass : public SingleTargetingResultClass {
   public:
    int m_classID{-1};          ///< Class ID from the neural network
    std::string m_className{""};  ///< Class name from the neural network
    double m_confidence{0};     ///< Classification confidence (0-1)
  };

  /**
   * Represents hardware statistics from the Limelight.
   */
  struct HardwareReport {
    std::string cameraId{""};
    double cpuUsage{0};
    double diskFree{0};
    double diskTotal{0};
    double ramUsage{0};
    double temperature{0};
  };

  /**
   * Represents IMU data from the JSON results.
   */
  struct IMUResults {
    std::vector<double> data;
    std::vector<double> quaternion{4, 0.0};
    double yaw{0};

    // Parsed from data array
    double robotYaw{0};
    double roll{0};
    double pitch{0};
    double rawYaw{0};
    double gyroZ{0};
    double gyroX{0};
    double gyroY{0};
    double accelZ{0};
    double accelX{0};
    double accelY{0};

    void parseDataArray() {
      if (data.size() >= 10) {
        robotYaw = data[0];
        roll = data[1];
        pitch = data[2];
        rawYaw = data[3];
        gyroZ = data[4];
        gyroX = data[5];
        gyroY = data[6];
        accelZ = data[7];
        accelX = data[8];
        accelY = data[9];
      }
    }
  };

  /**
   * Represents capture rewind buffer statistics.
   */
  struct RewindStats {
    double bufferUsage{0};
    int enabled{0};
    int flushing{0};
    int frameCount{0};
    int latencyPenalty{0};
    double storedSeconds{0};
  };

  /**
   * Contains all vision processing results from a single frame.
   */
  class VisionResultsClass {
   public:
    std::vector<RetroreflectiveResultClass> RetroResults;
    std::vector<FiducialResultClass> FiducialResults;
    std::vector<DetectionResultClass> DetectionResults;
    std::vector<ClassificationResultClass> ClassificationResults;
    std::vector<BarcodeResultClass> BarcodeResults;
    std::vector<double> pythonOutput;
    double m_timeStamp{-1.0};
    double m_latencyPipeline{0};
    double m_latencyCapture{0};
    double m_latencyJSON{0};
    double m_pipelineIndex{-1.0};
    std::string pipelineType{""};
    int valid{0};
    double tx{0};
    double ty{0};
    double tx_nocrosshair{0};
    double ty_nocrosshair{0};
    double ta{0};
    std::vector<double> botPose{6, 0.0};
    std::vector<double> botPose_wpired{6, 0.0};
    std::vector<double> botPose_wpiblue{6, 0.0};
    std::vector<double> botPose_orb{6, 0.0};
    std::vector<double> botPose_orb_wpiblue{6, 0.0};
    std::vector<double> botPose_orb_wpired{6, 0.0};
    double botpose_tagcount{0};
    double botpose_span{0};
    double botpose_avgdist{0};
    double botpose_avgarea{0};
    std::vector<double> camerapose_robotspace{6, 0.0};
    long long timestamp_nt{0};
    long long timestamp_sys{0};
    long long timestamp_us{0};
    HardwareReport hardware;
    IMUResults imuResults;
    RewindStats rewindStats;
    void Clear() {
      RetroResults.clear();
      FiducialResults.clear();
      DetectionResults.clear();
      ClassificationResults.clear();
      BarcodeResults.clear();
      pythonOutput.clear();
      botPose.assign(6, 0.0);
      botPose_wpired.assign(6, 0.0);
      botPose_wpiblue.assign(6, 0.0);
      botPose_orb.assign(6, 0.0);
      botPose_orb_wpiblue.assign(6, 0.0);
      botPose_orb_wpired.assign(6, 0.0);
      botpose_tagcount = 0;
      botpose_span = 0;
      botpose_avgdist = 0;
      botpose_avgarea = 0;
      camerapose_robotspace.assign(6, 0.0);
      m_timeStamp = -1.0;
      m_latencyPipeline = 0;
      m_latencyCapture = 0;
      m_latencyJSON = 0;
      m_pipelineIndex = -1.0;
      pipelineType = "";
      valid = 0;
      tx = 0;
      ty = 0;
      tx_nocrosshair = 0;
      ty_nocrosshair = 0;
      ta = 0;
      timestamp_nt = 0;
      timestamp_sys = 0;
      timestamp_us = 0;
      hardware = HardwareReport{};
      imuResults = IMUResults{};
      rewindStats = RewindStats{};
    }
  };

  /**
   * Top-level container for all Limelight results parsed from JSON.
   */
  class LimelightResultsClass {
   public:
    VisionResultsClass targetingResults;  ///< All targeting results from the current frame
  };

  /// @cond INTERNAL
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
    inline const std::string _key_botpose_orb{"botpose_orb"};
    inline const std::string _key_botpose_orb_wpiblue{"botpose_orb_wpiblue"};
    inline const std::string _key_botpose_orb_wpired{"botpose_orb_wpired"};
    inline const std::string _key_botpose_tagcount{"botpose_tagcount"};
    inline const std::string _key_botpose_span{"botpose_span"};
    inline const std::string _key_botpose_avgdist{"botpose_avgdist"};
    inline const std::string _key_botpose_avgarea{"botpose_avgarea"};
    inline const std::string _key_camerapose_robotspace{"t6c_rs"};

    inline const std::string _key_transformROBOTPOSE_FIELDSPACE{"t6r_fs"};
    inline const std::string _key_skew{"skew"};
    inline const std::string _key_ffamily{"fam"};
    inline const std::string _key_data{"data"};
    inline const std::string _key_colorRGB{"cRGB"};
    inline const std::string _key_colorHSV{"cHSV"};

    inline const std::string _key_pipelineType{"pTYPE"};
    inline const std::string _key_tx{"tx"};
    inline const std::string _key_ty{"ty"};
    inline const std::string _key_txnc{"txnc"};
    inline const std::string _key_tync{"tync"};
    inline const std::string _key_ta{"ta"};
    inline const std::string _key_timestamp_nt{"ts_nt"};
    inline const std::string _key_timestamp_sys{"ts_sys"};
    inline const std::string _key_timestamp_us{"ts_us"};
    inline const std::string _key_pythonOutput{"PythonOut"};
    inline const std::string _key_hw{"hw"};
    inline const std::string _key_imu{"imu"};
    inline const std::string _key_rewind{"rewind"};
  }  // namespace internal
  /// @endcond

  /**
   * Sets up port forwarding for a Limelight connected over Ethernet.
   * This allows access to the Limelight web interface through the roboRIO.
   * Call once during robot initialization.
   * @param limelightName Name/hostname of the Limelight camera
   */
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

  /**
   * Sets up port forwarding for a Limelight 3A/3G connected via USB.
   * This allows access to the Limelight web interface and video stream
   * when connected to the robot over USB.
   *
   * For usbIndex 0: ports 5800-5809 forward to 172.29.0.1
   * For usbIndex 1: ports 5810-5819 forward to 172.29.1.1
   * etc.
   *
   * Call this method once during robot initialization.
   * To access the interface of the camera with usbIndex0, you would go to roboRIO-(teamnum)-FRC.local:5801. Port 5811
   * for usb index 1
   *
   * @param usbIndex The USB index of the Limelight (0, 1, 2, etc.)
   */
  inline void SetupPortForwardingUSB(int usbIndex) {
    std::string ip = "172.29." + std::to_string(usbIndex) + ".1";
    int basePort = 5800 + (usbIndex * 10);

    auto& portForwarder = wpi::PortForwarder::GetInstance();
    for (int i = 0; i < 10; i++) {
      portForwarder.Add(basePort + i, ip, 5800 + i);
    }
  }

  /// @cond INTERNAL
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

  inline void from_json(const wpi::json& j, HardwareReport& t) {
    t.cameraId = SafeJSONAccess<std::string>(j, "cid", "");
    t.cpuUsage = SafeJSONAccess<double>(j, "cpu", 0.0);
    t.diskFree = SafeJSONAccess<double>(j, "dfree", 0.0);
    t.diskTotal = SafeJSONAccess<double>(j, "dtot", 0.0);
    t.ramUsage = SafeJSONAccess<double>(j, "ram", 0.0);
    t.temperature = SafeJSONAccess<double>(j, "temp", 0.0);
  }

  inline void from_json(const wpi::json& j, IMUResults& t) {
    t.data = SafeJSONAccess<std::vector<double>>(j, "data", std::vector<double>{});
    t.quaternion = SafeJSONAccess<std::vector<double>>(j, "quat", std::vector<double>{0, 0, 0, 0});
    t.yaw = SafeJSONAccess<double>(j, "yaw", 0.0);
    t.parseDataArray();
  }

  inline void from_json(const wpi::json& j, RewindStats& t) {
    t.bufferUsage = SafeJSONAccess<double>(j, "bufferUsage", 0.0);
    t.enabled = static_cast<int>(SafeJSONAccess<double>(j, "enabled", 0.0));
    t.flushing = static_cast<int>(SafeJSONAccess<double>(j, "flushing", 0.0));
    t.frameCount = static_cast<int>(SafeJSONAccess<double>(j, "frameCount", 0.0));
    t.latencyPenalty = static_cast<int>(SafeJSONAccess<double>(j, "latpen", 0.0));
    t.storedSeconds = SafeJSONAccess<double>(j, "storedSeconds", 0.0);
  }
  /// @endcond

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
    t.m_fiducialID = static_cast<int>(SafeJSONAccess<double>(j, internal::_key_fiducialID, 0.0));
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
    t.m_classID = static_cast<int>(SafeJSONAccess<double>(j, internal::_key_classID, 0.0));
    t.m_className = SafeJSONAccess<std::string>(j, internal::_key_className, "");
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
    t.m_classID = static_cast<int>(SafeJSONAccess<double>(j, internal::_key_classID, 0.0));
    t.m_className = SafeJSONAccess<std::string>(j, internal::_key_className, "");
    t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
    t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
    t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
    t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
    t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
    t.m_TargetCorners =
        SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
  }

  /// @cond INTERNAL
  inline void from_json(const wpi::json& j, VisionResultsClass& t) {
    t.m_timeStamp = SafeJSONAccess<double>(j, internal::_key_timestamp, 0.0);
    t.m_latencyPipeline = SafeJSONAccess<double>(j, internal::_key_latency_pipeline, 0.0);
    t.m_latencyCapture = SafeJSONAccess<double>(j, internal::_key_latency_capture, 0.0);
    t.m_pipelineIndex = SafeJSONAccess<double>(j, internal::_key_pipelineIndex, 0.0);
    t.pipelineType = SafeJSONAccess<std::string>(j, internal::_key_pipelineType, "");
    t.valid = static_cast<int>(SafeJSONAccess<double>(j, "v", 0.0));

    t.tx = SafeJSONAccess<double>(j, internal::_key_tx, 0.0);
    t.ty = SafeJSONAccess<double>(j, internal::_key_ty, 0.0);
    t.tx_nocrosshair = SafeJSONAccess<double>(j, internal::_key_txnc, 0.0);
    t.ty_nocrosshair = SafeJSONAccess<double>(j, internal::_key_tync, 0.0);
    t.ta = SafeJSONAccess<double>(j, internal::_key_ta, 0.0);

    t.timestamp_nt = SafeJSONAccess<long long>(j, internal::_key_timestamp_nt, 0LL);
    t.timestamp_sys = SafeJSONAccess<long long>(j, internal::_key_timestamp_sys, 0LL);
    t.timestamp_us = SafeJSONAccess<long long>(j, internal::_key_timestamp_us, 0LL);

    std::vector<double> defaultVector(6, 0.0);
    t.botPose = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose, defaultVector);
    t.botPose_wpired = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose_wpired, defaultVector);
    t.botPose_wpiblue = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose_wpiblue, defaultVector);
    t.botPose_orb = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose_orb, defaultVector);
    t.botPose_orb_wpiblue = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose_orb_wpiblue, defaultVector);
    t.botPose_orb_wpired = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose_orb_wpired, defaultVector);
    t.botpose_tagcount = SafeJSONAccess<double>(j, internal::_key_botpose_tagcount, 0.0);
    t.botpose_span = SafeJSONAccess<double>(j, internal::_key_botpose_span, 0.0);
    t.botpose_avgdist = SafeJSONAccess<double>(j, internal::_key_botpose_avgdist, 0.0);
    t.botpose_avgarea = SafeJSONAccess<double>(j, internal::_key_botpose_avgarea, 0.0);
    t.camerapose_robotspace = SafeJSONAccess<std::vector<double>>(j, internal::_key_camerapose_robotspace, defaultVector);

    t.pythonOutput = SafeJSONAccess<std::vector<double>>(j, internal::_key_pythonOutput, std::vector<double>{});
    t.hardware = SafeJSONAccess<HardwareReport>(j, internal::_key_hw, HardwareReport{});
    t.imuResults = SafeJSONAccess<IMUResults>(j, internal::_key_imu, IMUResults{});
    t.rewindStats = SafeJSONAccess<RewindStats>(j, internal::_key_rewind, RewindStats{});

    t.RetroResults =
        SafeJSONAccess<std::vector<RetroreflectiveResultClass>>(j, "Retro", std::vector<RetroreflectiveResultClass>{});
    t.FiducialResults =
        SafeJSONAccess<std::vector<FiducialResultClass>>(j, "Fiducial", std::vector<FiducialResultClass>{});
    t.DetectionResults =
        SafeJSONAccess<std::vector<DetectionResultClass>>(j, "Detector", std::vector<DetectionResultClass>{});
    t.ClassificationResults =
        SafeJSONAccess<std::vector<ClassificationResultClass>>(j, "Classifier", std::vector<ClassificationResultClass>{});
    t.BarcodeResults = SafeJSONAccess<std::vector<BarcodeResultClass>>(j, "Barcode", std::vector<BarcodeResultClass>{});
  }

  /**
     * Limelight Results object, parsed from a Limelight's JSON results output.
     */
  inline void from_json(const wpi::json& j, LimelightResultsClass& t) {
    try {
      if (j.is_null() || !j.is_object()) {
        t.targetingResults = VisionResultsClass{};
        return;
      }
      t.targetingResults = j.get<LimelightHelpers::VisionResultsClass>();
    } catch (...) {
      t.targetingResults = VisionResultsClass{};
    }
  }
  /// @endcond

  /**
   * Gets the latest JSON results and parses them into a LimelightResultsClass object.
   * This provides access to all targeting data including neural network results,
   * AprilTag detections, retroreflective targets, and barcodes.
   * @param limelightName Name of the Limelight camera ("" for default)
   * @param profile If true, prints JSON parsing time to stdout
   * @return LimelightResultsClass containing all parsed results
   */
  inline LimelightResultsClass getLatestResults(const std::string& limelightName = "", bool profile = false) {
    auto start = std::chrono::high_resolution_clock::now();
    std::string jsonString = getJSONDump(limelightName);

    if (jsonString.empty() || jsonString.find_first_not_of(" \t\n\r") == std::string::npos) {
      return LimelightResultsClass();
    }

    wpi::json data;
    try {
      data = wpi::json::parse(jsonString);
    } catch (const std::exception& e) {
      return LimelightResultsClass();
    } catch (...) {
      return LimelightResultsClass();
    }

    if (data.is_null() || !data.is_object()) {
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