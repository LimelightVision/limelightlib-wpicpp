# Limelightlib (WPILIB CPP)

## Usage

This is a single-file, header-only library. All you need to do is copy the file into your CPP project's src/main/include folder, and add #include LimelightHelpers.h wherever you want to use the library. You don't need to create any objects for your Limelights - the library is designed to be used in a stateless/functional manner.

### Basic Usage
Every method in Limelightlib accepts a string parameter indicating the correct Limelight to use. If left blank or null, the name is assumed to be "limelight"
```
LimelightHelpers::setLEDMode_PipelineControl("");
LimelightHelpers::setLEDMode_ForceBlink("")
LimelightHelpers::setCropWindow("",-1,1,-1,1);
double tx = LimelightHelpers::getTX("");
```

### JSON Parsing
getLatestResults() parses the latest Limelight JSON dump into a LimelightResultsClass object.
Takes up to 2.5ms on RoboRIO 1.0. Parsing latency is logged in results.targetingresults.latency_jsonParse and may be added to latency_pipeline and latency_capture.

```
LimelightResultsClass llresults = LimelightHelpers::getLatestResults("");
```
Each LimelightResultsClass instance contains a Results object. Each Results object contains data such as botpose, an array for each target type, etc. With getLatestResults(), you now have easy access to 100% of your Limelight's output.

```
std::vector<double> botposeRed = llresults.results.botPose_wpired;
double pipelineLatency = llresults.results._TargetLatency;
FiducialResultClass = llresults.results.FiducialResults;
```

### Taking snapshots (wip in cpp lib)
```
LimelightHelpers::takeSnapshot("","snapshotname");
```

### Classes
```
LimelightTarget_Retro
LimelightTarget_Fiducial
LimelightTarget_Barcode
LimelightTarget_Classifier
LimelightTarget_Detector
Results
LimelightResults
(Pure Static) LimelightHelpers
```

### LimelightHelpers Methods (wip)
```
getLimelightNTTable()
getLimelightNTTableEntry()
getLimelightNTDouble()
setLimelightNTDouble()
setLimelightNTDoubleArray()
getLimelightNTDouleArray()
getLimelightNTString()
getLimelightURLString()

getTX()
getTY()
getTA()
getLatency_Pipeline()
getLatency_Capture()
getCurrentPipelineIndex()
getJsonDump()

getBotpose()
getBotpose_wpiRed()
getBotpose_wpiBlue()
getBotpose_TargetSpace
getCameraPsoe_TargetSpace()
getTargetPose_CameraSpace()
getTargetPose_RobotSpace()

getTargetColor()
getFiducialID()
getNeuralClassID()

setPipelineIndex()

setLEDMode_PipelineControl()
setLEDMode_ForceOff()
setLEDMode_ForceBlink()
setLEDMode_ForceOn()

setStreamMode_Standard()
setStreamMode_PiPMain()
setStreamMode_PiPSecondary()

setCropWindow()

setPythonScriptData()
getPythonScriptData()

takeSnapshot()
getLatestResults()
```