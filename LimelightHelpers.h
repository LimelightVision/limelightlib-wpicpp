#ifndef LIMELIGHTHELPERS_H
#define LIMELIGHTHELPERS_H

///
//https://github.com/LimelightVision/limelightlib-wpicpp
///

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <wpinet/PortForwarder.h>
#include "wpi/json.h"
#include <string>
#include <unistd.h>
//#include <curl/curl.h>
#include <vector>
#include <chrono>
#include <iostream>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
    
namespace LimelightHelpers
{
    inline std::string sanitizeName(const std::string &name)
    {
        if (name == "")
        {
            return "limelight";
        }
        return name;
    }

    inline frc::Pose3d toPose3D(const std::vector<double>& inData)
    {
        if(inData.size() < 6)
        {
            return frc::Pose3d();
        }
        return frc::Pose3d(
            frc::Translation3d(units::length::meter_t(inData[0]), units::length::meter_t(inData[1]), units::length::meter_t(inData[2])),
            frc::Rotation3d(units::angle::radian_t(inData[3]*(M_PI/180.0)), units::angle::radian_t(inData[4]*(M_PI/180.0)),
                   units::angle::radian_t(inData[5]*(M_PI/180.0))));
    }

    inline frc::Pose2d toPose2D(const std::vector<double>& inData)
    {
        if(inData.size() < 6)
        {
            return frc::Pose2d();
        }
        return frc::Pose2d(
            frc::Translation2d(units::length::meter_t(inData[0]), units::length::meter_t(inData[1])), 
            frc::Rotation2d(units::angle::radian_t(inData[5]*(M_PI/180.0))));
    }

    inline std::shared_ptr<nt::NetworkTable> getLimelightNTTable(const std::string &tableName)
    {
        return nt::NetworkTableInstance::GetDefault().GetTable(sanitizeName(tableName));
    }

    inline nt::NetworkTableEntry getLimelightNTTableEntry(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTable(tableName)->GetEntry(entryName);
    }

    inline double getLimelightNTDouble(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTableEntry(tableName, entryName).GetDouble(0.0);
    }

    inline std::vector<double> getLimelightNTDoubleArray(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTableEntry(tableName, entryName).GetDoubleArray(std::span<double>{});
    }

    inline std::string getLimelightNTString(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTableEntry(tableName, entryName).GetString("");
    }

    inline void setLimelightNTDouble(const std::string &tableName, const std::string entryName, double val)
    {
        getLimelightNTTableEntry(tableName, entryName).SetDouble(val);
    }

    inline void setLimelightNTDoubleArray(const std::string &tableName, const std::string &entryName, const std::span<const double> &vals)
    {
        getLimelightNTTableEntry(tableName, entryName).SetDoubleArray(vals);
    }

    inline double getTX(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tx");
    }
    
    inline double getTV(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tv");
    }

    inline double getTY(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "ty");
    }

    inline double getTA(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "ta");
    }

    inline double getLatency_Pipeline(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tl");
    }

    inline double getLatency_Capture(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "cl");
    }

    inline std::string getJSONDump(const std::string &limelightName = "")
    {
        return getLimelightNTString(limelightName, "json");
    }

    inline std::vector<double> getBotpose(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    inline std::vector<double> getBotpose_wpiRed(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    inline std::vector<double> getBotpose_wpiBlue(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    inline std::vector<double> getBotpose_TargetSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
    }

    inline std::vector<double> getCameraPose_TargetSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
    }

    inline std::vector<double> getCameraPose_RobotSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "camerapose_robotspace");
    }

    inline std::vector<double> getTargetPose_CameraSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
    }

    inline std::vector<double> getTargetPose_RobotSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    }

    inline std::vector<double> getTargetColor(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "tc");
    }

    inline double getFiducialID(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tid");
    }

    inline double getNeuralClassID(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tclass");
    }

    /////
    /////

    inline void setPipelineIndex(const std::string &limelightName, int index)
    {
        setLimelightNTDouble(limelightName, "pipeline", index);
    }

    inline void setLEDMode_PipelineControl(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "ledMode", 0);
    }

    inline void setLEDMode_ForceOff(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    inline void setLEDMode_ForceBlink(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "ledMode", 2);
    }

    inline void setLEDMode_ForceOn(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }

    inline void setStreamMode_Standard(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "stream", 0);
    }

    inline void setStreamMode_PiPMain(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "stream", 1);
    }

    inline void setStreamMode_PiPSecondary(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "stream", 2);
    }

    /**
     * Sets the crop window. The crop window in the UI must be completely open for
     * dynamic cropping to work.
     */
    inline void setCropWindow(const std::string &limelightName, double cropXMin,
                              double cropXMax, double cropYMin, double cropYMax)
    {
        double cropWindow[4]{cropXMin, cropXMax, cropYMin, cropYMax};
        setLimelightNTDoubleArray(limelightName, "crop", cropWindow);
    }

    /////
    /////

    /**
     * Sets the camera pose in robotspace. The UI camera pose must be set to zeros
     */
    inline void setCameraPose_RobotSpace(const std::string &limelightName, double forward, double side, double up, double roll, double pitch, double yaw) {
        double entries[6] ={forward, side, up, roll, pitch, yaw};
        setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
    }

    inline void setPythonScriptData(const std::string &limelightName, const std::vector<double> &outgoingPythonData)
    {
        setLimelightNTDoubleArray(limelightName, "llrobot", std::span{outgoingPythonData.begin(), outgoingPythonData.size()});
    }

    inline std::vector<double> getPythonScriptData(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "llpython");
    }

    /////
    /////

    // Take async snapshot

    /////
    /////

    inline const double INVALID_TARGET = 0.0;
    class SingleTargetingResultClass
    {
    public:
        SingleTargetingResultClass(){};
        ~SingleTargetingResultClass(){};
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

    class RetroreflectiveResultClass : public SingleTargetingResultClass
    {
    public:
        RetroreflectiveResultClass() {}
        ~RetroreflectiveResultClass() {}
    };

    class FiducialResultClass : public SingleTargetingResultClass
    {
    public:
        FiducialResultClass() {}
        ~FiducialResultClass() {}
        int m_fiducialID{0};
        std::string m_family{""};
    };

    class DetectionResultClass : public SingleTargetingResultClass
    {
    public:
        DetectionResultClass() {}
        ~DetectionResultClass() {}

        int m_classID{-1};
        std::string m_className{""};
        double m_confidence{0};
    };

    class ClassificationResultClass : public SingleTargetingResultClass
    {
    public:
        ClassificationResultClass() {}
        ~ClassificationResultClass() {}

        int m_classID{-1};
        std::string m_className{""};
        double m_confidence{0};
    };

    class VisionResultsClass
    {
    public:
        VisionResultsClass() {}
        ~VisionResultsClass() {}
        std::vector<RetroreflectiveResultClass> RetroResults;
        std::vector<FiducialResultClass> FiducialResults;
        std::vector<DetectionResultClass> DetectionResults;
        std::vector<ClassificationResultClass> ClassificationResults;
        double m_timeStamp{-1.0};
        double m_latencyPipeline{0};
        double m_latencyCapture{0};
        double m_latencyJSON{0};
        double m_pipelineIndex{-1.0};
        int valid{0};
        std::vector<double> botPose;
        std::vector<double> botPose_wpired;
        std::vector<double> botPose_wpiblue;
        void Clear()
        {
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

    class LimelightResultsClass
    {
    public:
        LimelightResultsClass() {}
        ~LimelightResultsClass() {}
        VisionResultsClass targetingResults;
    };

    namespace internal
    {
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
        inline const std::string _key_TargetYDegreesCrosshair{"ty"};
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
        inline const std::string _key_colorRGB{"cRGB"};
        inline const std::string _key_colorHSV{"cHSV"};
    }

    inline void PhoneHome() 
    {
        static int sockfd = -1;
        static struct sockaddr_in servaddr, cliaddr;

        if (sockfd == -1) {
            sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd < 0) {
                std::cerr << "Socket creation failed" << std::endl;
                return;
            }

            memset(&servaddr, 0, sizeof(servaddr));
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

            const char *msg = "LLPhoneHome";
            sendto(sockfd, msg, strlen(msg), 0, (const struct sockaddr *) &servaddr, sizeof(servaddr));
        }

        char receiveData[1024];
        socklen_t len = sizeof(cliaddr);

        ssize_t n = recvfrom(sockfd, (char *)receiveData, 1024, 0, (struct sockaddr *) &cliaddr, &len);
        if (n > 0) {
            receiveData[n] = '\0'; // Null-terminate the received string
            std::string received(receiveData, n);
            std::cout << "Received response: " << received << std::endl;
        } else if (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
            std::cerr << "Error receiving data" << std::endl;
            close(sockfd);
            sockfd = -1;
        }
    }

    inline void SetupPortForwarding(const std::string& limelightName) 
    {
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
    T SafeJSONAccess(const wpi::json& jsonData, const KeyType& key, const T& defaultValue)
    {
        try
        {
           return jsonData.at(key).template get<T>();
        }
        catch (wpi::json::exception& e)
        {
            return defaultValue;
        }
        catch (...)
        {
            return defaultValue;
        }
    }
    inline void from_json(const wpi::json &j, RetroreflectiveResultClass &t)
    {
        std::vector<double> defaultValueVector(6, 0.0);
        t.m_CAMERATransform6DTARGETSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformCAMERAPOSE_TARGETSPACE, defaultValueVector);
        t.m_CAMERATransform6DROBOTSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformCAMERAPOSE_ROBOTSPACE, defaultValueVector);

        t.m_TargetTransform6DCAMERASPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformTARGETPOSE_CAMERASPACE, defaultValueVector);
        t.m_TargetTransform6DROBOTSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformTARGETPOSE_ROBOTSPACE, defaultValueVector);
        t.m_ROBOTTransform6DTARGETSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformROBOTPOSE_TARGETSPACE, defaultValueVector);
        t.m_ROBOTTransform6DFIELDSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformROBOTPOSE_FIELDSPACE, defaultValueVector);

        t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
        t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
        t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
        t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
        t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
        t.m_TargetCorners = SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
    }

    inline void from_json(const wpi::json &j,  FiducialResultClass &t)
    {
        std::vector<double> defaultValueVector(6, 0.0);
        t.m_family = SafeJSONAccess<std::string>(j, internal::_key_ffamily, "");
        t.m_fiducialID = SafeJSONAccess<double>(j, internal::_key_fiducialID, 0.0);
        t.m_CAMERATransform6DTARGETSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformCAMERAPOSE_TARGETSPACE, defaultValueVector);
        t.m_CAMERATransform6DROBOTSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformCAMERAPOSE_ROBOTSPACE, defaultValueVector);
        t.m_TargetTransform6DCAMERASPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformTARGETPOSE_CAMERASPACE, defaultValueVector);
        t.m_TargetTransform6DROBOTSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformTARGETPOSE_ROBOTSPACE, defaultValueVector);
        t.m_ROBOTTransform6DTARGETSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformROBOTPOSE_TARGETSPACE, defaultValueVector);
        t.m_ROBOTTransform6DFIELDSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformROBOTPOSE_FIELDSPACE, defaultValueVector);

        t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
        t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
        t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
        t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
        t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
        t.m_TargetCorners = SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
    }

    inline void from_json(const wpi::json &j,  DetectionResultClass &t)
    {
        t.m_confidence = SafeJSONAccess<double>(j, internal::_key_confidence, 0.0);
        t.m_classID = SafeJSONAccess<double>(j, internal::_key_classID, 0.0);
        t.m_className = SafeJSONAccess<std::string>(j, internal::_key_className, "");
        t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
        t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
        t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
        t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
        t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
        t.m_TargetCorners = SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
    }

    inline void from_json(const wpi::json &j,  ClassificationResultClass &t)
    {
        t.m_confidence = SafeJSONAccess<double>(j, internal::_key_confidence, 0.0);
        t.m_classID = SafeJSONAccess<double>(j, internal::_key_classID, 0.0);
        t.m_className = SafeJSONAccess<std::string>(j, internal::_key_className, "");
        t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
        t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
        t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
        t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
        t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
        t.m_TargetCorners = SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
    }

    inline void from_json(const wpi::json &j,  VisionResultsClass &t)
    {
        t.m_timeStamp = SafeJSONAccess<double>(j, internal::_key_timestamp, 0.0);
        t.m_latencyPipeline = SafeJSONAccess<double>(j, internal::_key_latency_pipeline, 0.0);
        t.m_latencyCapture = SafeJSONAccess<double>(j, internal::_key_latency_capture, 0.0);
        t.m_pipelineIndex = SafeJSONAccess<double>(j, internal::_key_pipelineIndex, 0.0);
        t.valid = SafeJSONAccess<double>(j, "v", 0.0);

        std::vector<double> defaultVector{};
        t.botPose = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose, defaultVector);
        t.botPose_wpired = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose_wpired, defaultVector);
        t.botPose_wpiblue = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose_wpiblue, defaultVector);

        t.RetroResults = SafeJSONAccess<std::vector< RetroreflectiveResultClass>>(j, "Retro", std::vector< RetroreflectiveResultClass>{});
        t.FiducialResults = SafeJSONAccess<std::vector< FiducialResultClass>>(j, "Fiducial", std::vector< FiducialResultClass>{});
        t.DetectionResults = SafeJSONAccess<std::vector< DetectionResultClass>>(j, "Detector", std::vector< DetectionResultClass>{});
        t.ClassificationResults = SafeJSONAccess<std::vector< ClassificationResultClass>>(j, "Detector", std::vector< ClassificationResultClass>{});
    }

    inline void from_json(const wpi::json &j,  LimelightResultsClass &t)
    {
        t.targetingResults = SafeJSONAccess<LimelightHelpers::VisionResultsClass>(j, "Results",  LimelightHelpers::VisionResultsClass{});
    }

    inline LimelightResultsClass getLatestResults(const std::string &limelightName = "", bool profile = false)
    {
        auto start = std::chrono::high_resolution_clock::now();
        std::string jsonString = getJSONDump(limelightName); 
        wpi::json data;
        try
        {
            data = wpi::json::parse(jsonString);
        }
        catch(const std::exception& e)
        {
           return LimelightResultsClass();
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        double nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        double millis = (nanos * 0.000001);
        try
        {
            LimelightResultsClass out = data.get<LimelightHelpers::LimelightResultsClass>();
            out.targetingResults.m_latencyJSON = millis;
            if (profile)
            {
                std::cout << "lljson: " << millis << std::endl;
            }
            return out;
        }
        catch(...) 
        {
            return LimelightResultsClass();
        }
    }
}
#endif // LIMELIGHTHELPERS_H