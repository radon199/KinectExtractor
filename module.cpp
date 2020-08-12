#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "extractor.h"

#include <string>

namespace py = pybind11;

py::str CameraSpacePointRepr(const CameraSpacePoint& obj)
{
    return "<CameraSpacePoint: X=" + std::to_string(obj.X) + " Y=" + std::to_string(obj.Y) + " Z=" + std::to_string(obj.Z) + ">";
}

py::str CameraSpacePointStr(const CameraSpacePoint& obj)
{
    return "X=" + std::to_string(obj.X) + " Y=" + std::to_string(obj.Y) + " Z=" + std::to_string(obj.Z);
}

py::str Vector4Repr(const Vector4& obj)
{
    return "<Vector4: X=" + std::to_string(obj.x) + " Y=" + std::to_string(obj.y) + " Z=" + std::to_string(obj.z) + " W=" + std::to_string(obj.w) + ">";
}

py::str Vector4Str(const Vector4& obj)
{
    return "X=" + std::to_string(obj.z) + " Y=" + std::to_string(obj.y) + " Z=" + std::to_string(obj.z) + " W=" + std::to_string(obj.w);
}

PYBIND11_MODULE(KinectExtractor, m) {
    m.doc() = "Extracts data from the Kinect API.";

    py::class_<KinectExtractor>(m, "KinectExtractor")
        .def(py::init<>())
        .def("InitKinect", &KinectExtractor::InitKinect, "Intitalize the Kinect. Must be called before any calls to GetKinectData.")
        .def("CloseKinect", &KinectExtractor::CloseKinect, "Close the connection to the Kinect.")
        .def("GetKinectData", &KinectExtractor::GetKinectData, "Pull a new color, depth and body frame from the Kinect stream. Data might be uninitalized depending on how long after InitKinect this has been called.")
        .def("GetJointLocation", &KinectExtractor::GetJointLocation, "Get the location of a single joint in the body frame.")
        .def("GetJoints", &KinectExtractor::GetJoints, "Get all joints data.")
        .def("GetJointOrientations", &KinectExtractor::GetJointOrientations, "Get the orientation of all joints.")
        .def("GetColorData", &KinectExtractor::GetColorData, "Get the full resolution color data from the Kinect color camera.")
        .def("GetDownsampledColorData", &KinectExtractor::GetDownsampledColorData, "Get the color data downsampled to match the Kinect depth camera data.")
        .def("GetDepthLocations", &KinectExtractor::GetDepthLocations, "Get the depth camera data as worldspace locations in the Kinect's coordinate system.")
        .def_readwrite("extract_rgb", &KinectExtractor::extract_rgb)
        .def_readwrite("extract_depth", &KinectExtractor::extract_depth)
        .def_readwrite("extract_joint_locations", &KinectExtractor::extract_joint_locations)
        .def_readwrite("extract_joint_orientations", &KinectExtractor::extract_joint_orientations);

    py::class_<CameraSpacePoint>(m, "CameraSpacePoint")
        .def_readwrite("X", &CameraSpacePoint::X)
        .def_readwrite("Y", &CameraSpacePoint::Y)
        .def_readwrite("Z", &CameraSpacePoint::Z)
        .def("__str__", &CameraSpacePointStr)
        .def("__repr__", &CameraSpacePointRepr);

    py::class_<Vector4>(m, "Vector4")
        .def_readwrite("x", &Vector4::x)
        .def_readwrite("y", &Vector4::y)
        .def_readwrite("z", &Vector4::z)
        .def_readwrite("w", &Vector4::w)
        .def("__str__", &Vector4Str)
        .def("__repr__", &Vector4Repr);

    py::class_<Joint>(m, "Joint")
        .def_readwrite("JointType", &Joint::JointType)
        .def_readwrite("Position", &Joint::Position)
        .def_readwrite("TrackingState", &Joint::TrackingState);

    py::class_<JointOrientation>(m, "JointOrientation")
        .def_readwrite("JointType", &JointOrientation::JointType)
        .def_readwrite("Orientation", &JointOrientation::Orientation);

    py::enum_<TrackingState>(m, "TrackingState")
        .value("NotTracked", TrackingState_NotTracked)
        .value("Inferred", TrackingState_Inferred)
        .value("Tracked", TrackingState_Tracked)
        .export_values();

    py::enum_<JointType>(m, "JointType")
        .value("SpineBase", JointType_SpineBase)
        .value("SpineMid", JointType_SpineMid)
        .value("Neck", JointType_Neck)
        .value("Head", JointType_Head)
        .value("ShoulderLeft", JointType_ShoulderLeft)
        .value("ElbowLeft", JointType_ElbowLeft)
        .value("WristLeft", JointType_WristLeft)
        .value("HandLeft", JointType_HandLeft)
        .value("ShoulderRight", JointType_ShoulderRight)
        .value("ElbowRight", JointType_ElbowRight)
        .value("WristRight", JointType_WristRight)
        .value("HandRight", JointType_HandRight)
        .value("HipLeft", JointType_HipLeft)
        .value("KneeLeft", JointType_KneeLeft)
        .value("AnkleLeft", JointType_AnkleLeft)
        .value("FootLeft", JointType_FootLeft)
        .value("HipRight", JointType_HipRight)
        .value("KneeRight", JointType_KneeRight)
        .value("AnkleRight", JointType_AnkleRight)
        .value("FootRight", JointType_FootRight)
        .value("SpineShoulder", JointType_SpineShoulder)
        .value("HandTipLeft", JointType_HandTipLeft)
        .value("ThumbLeft", JointType_ThumbLeft)
        .value("HandTipRight", JointType_HandTipRight)
        .value("ThumbRight", JointType_ThumbRight)
        .value("Count", JointType_Count)
        .export_values();
}