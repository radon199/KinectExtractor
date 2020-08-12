#ifndef KINECT_EXTRACTOR_H
#define KINECT_EXTRACTOR_H

#include <Windows.h>
#include <Ole2.h>
#include <Kinect.h>

#include <vector>
#include <tuple>

// Hardware resolution of the Kinect V2
#define DEPTH_WIDTH  512
#define DEPTH_HEIGHT 424
#define COLOR_WIDTH  1920
#define COLOR_HEIGHT 1080

typedef std::tuple<float, float, float> PointTuple;
typedef std::tuple<float, float, float, float> Vector4Tuple;
// Stored as a raw tuple so it's cheaper to return to python
typedef std::vector<PointTuple> PointTupleVector;
typedef std::vector<Vector4Tuple> Vector4TupleVector;

class KinectExtractor
{
public:
	KinectExtractor();

	bool InitKinect();
	bool CloseKinect();
	void GetKinectData();

	const CameraSpacePoint& GetJointLocation(const JointType joint);

	std::vector<Joint> GetJoints() { return joints; }
	std::vector<JointOrientation> GetJointOrientations() { return orientations; }

	std::vector<byte>  GetColorData() { return rgbimage; }
	Vector4TupleVector GetDownsampledColorData();
	PointTupleVector   GetDepthLocations();

	bool test{ true };

	bool extract_rgb{ true };
	bool extract_depth{ true };
	bool extract_joint_locations{ true };
	bool extract_joint_orientations{ true };

private:

	void _GetDepthData(IMultiSourceFrame* frame);
	void _GetRgbData(IMultiSourceFrame* frame);
	void _GetBodyData(IMultiSourceFrame* frame);

	// Intermediate Buffers
	std::vector<byte> rgbimage;               // Stores RGB color image
	std::vector<ColorSpacePoint>  depth2rgb;  // Maps depth pixels to rgb pixels
	std::vector<CameraSpacePoint> depth2xyz;  // Maps depth pixels to 3d coordinates

	// Body tracking variables
	BOOLEAN tracked;                            // Whether we see a body
	std::vector<Joint> joints;	                // List of joints in the tracked body
	std::vector<JointOrientation> orientations; // List of joint orientations in the tracked body

	// Kinect Variables
	IKinectSensor* sensor;             // Kinect sensor
	IMultiSourceFrameReader* reader;   // Kinect data source
	ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates
};

#endif //KINECT_EXTRACTOR_H