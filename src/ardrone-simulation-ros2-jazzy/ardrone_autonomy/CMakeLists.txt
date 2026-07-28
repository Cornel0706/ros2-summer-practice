cmake_minimum_required(VERSION 3.8)
project(ardrone_autonomy)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Matrix33.msg"
  "msg/Navdata.msg"
  "msg/NavdataAdcDataFrame.msg"
  "msg/NavdataAltitude.msg"
  "msg/NavdataDemo.msg"
  "msg/NavdataEulerAngles.msg"
  "msg/NavdataGames.msg"
  "msg/NavdataGyrosOffsets.msg"
  "msg/NavdataHdvideoStream.msg"
  "msg/NavdataKalmanPressure.msg"
  "msg/NavdataMagneto.msg"
  "msg/NavdataPhysMeasures.msg"
  "msg/NavdataPressureRaw.msg"
  "msg/NavdataPwm.msg"
  "msg/NavdataRawMeasures.msg"
  "msg/NavdataRcReferences.msg"
  "msg/NavdataReferences.msg"
  "msg/NavdataTime.msg"
  "msg/NavdataTrackersSend.msg"
  "msg/NavdataTrims.msg"
  "msg/NavdataVideoStream.msg"
  "msg/NavdataVision.msg"
  "msg/NavdataVisionDetect.msg"
  "msg/NavdataVisionOf.msg"
  "msg/NavdataVisionPerf.msg"
  "msg/NavdataVisionRaw.msg"
  "msg/NavdataWatchdog.msg"
  "msg/NavdataWifi.msg"
  "msg/NavdataWindSpeed.msg"
  "msg/NavdataZimmu3000.msg"
  "msg/Vector21.msg"
  "msg/Vector31.msg"
  "srv/CamSelect.srv"
  "srv/FlightAnim.srv"
  "srv/LedAnim.srv"
  "srv/RecordEnable.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
