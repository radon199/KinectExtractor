#include "extractor.h"

#include <cmath>
#include <cstdio>
#include <limits>

#include <iostream>
#include <string>

KinectExtractor::KinectExtractor()
{
	// Image buffers
	rgbimage.resize(COLOR_WIDTH * COLOR_HEIGHT * 4);
	depth2rgb.resize(DEPTH_WIDTH * DEPTH_HEIGHT);
	depth2xyz.resize(DEPTH_WIDTH * DEPTH_HEIGHT);

	// Joint buffers
	joints.resize(JointType_Count);
	orientations.resize(JointType_Count);
}

bool
KinectExtractor::InitKinect() {
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		return false;
	}
	if (sensor) {
		sensor->get_CoordinateMapper(&mapper);

		sensor->Open();
		sensor->OpenMultiSourceFrameReader(
			FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Body,
			&reader);
		return reader;
	}
	else {
		return false;
	}
}

bool
KinectExtractor::CloseKinect()
{
	if (sensor) {
		sensor->Close();
		sensor = nullptr;
		reader = nullptr;
		mapper = nullptr;
	}
	else
	{
		return true;
	}
}

void
KinectExtractor::_GetDepthData(IMultiSourceFrame* frame) {
	IDepthFrame* depthframe;
	IDepthFrameReference* frameref = NULL;
	frame->get_DepthFrameReference(&frameref);
	frameref->AcquireFrame(&depthframe);
	if (frameref) {
		frameref->Release();
	}

	if (!depthframe) {
		return;
	}

	// Get data from frame
	unsigned int sz;
	unsigned short* buf;
	depthframe->AccessUnderlyingBuffer(&sz, &buf);

	// Write vertex coordinates
	mapper->MapDepthFrameToCameraSpace(sz, buf, DEPTH_WIDTH * DEPTH_HEIGHT, depth2xyz.data());

	// Fill in depth2rgb map
	mapper->MapDepthFrameToColorSpace(sz, buf, DEPTH_WIDTH * DEPTH_HEIGHT, depth2rgb.data());

	if (depthframe) {
		depthframe->Release();
	}
}

void
KinectExtractor::_GetRgbData(IMultiSourceFrame* frame) {
	IColorFrame* colorframe;
	IColorFrameReference* frameref = NULL;
	frame->get_ColorFrameReference(&frameref);
	frameref->AcquireFrame(&colorframe);
	if (frameref) {
		frameref->Release();
	}

	if (!colorframe) {
		return;
	}

	// Get data from frame
	colorframe->CopyConvertedFrameDataToArray(COLOR_WIDTH * COLOR_HEIGHT * 4, rgbimage.data(), ColorImageFormat_Rgba);

	if (colorframe) {
		colorframe->Release();
	}
}

void
KinectExtractor::_GetBodyData(IMultiSourceFrame* frame) {
	IBodyFrame* bodyframe;
	IBodyFrameReference* frameref = NULL;
	frame->get_BodyFrameReference(&frameref);
	frameref->AcquireFrame(&bodyframe);
	if (frameref) {
		frameref->Release();
	}

	if (!bodyframe) {
		return;
	}

	IBody* body[BODY_COUNT] = { 0 };
	bodyframe->GetAndRefreshBodyData(BODY_COUNT, body);
	for (int i = 0; i < BODY_COUNT; i++) {
		body[i]->get_IsTracked(&tracked);
		if (tracked) {
			if (extract_joint_locations) {
				body[i]->GetJoints(JointType_Count, joints.data());
			}
			if (extract_joint_orientations) {
				body[i]->GetJointOrientations(JointType_Count, orientations.data());
			}
			break;
		}
	}

	if (bodyframe) {
		bodyframe->Release();
	}
}

void
KinectExtractor::GetKinectData() {
	if (!sensor) {
		return;
	}
	IMultiSourceFrame* frame = NULL;
	if (SUCCEEDED(reader->AcquireLatestFrame(&frame))) {
		if (extract_depth) {
			_GetDepthData(frame);
		}
		if (extract_rgb) {
			_GetRgbData(frame);
		}
		if (extract_joint_locations || extract_joint_orientations) {
			_GetBodyData(frame);
		}
	}
	if (frame) {
		frame->Release();
	}
}

const CameraSpacePoint&
KinectExtractor::GetJointLocation(const JointType joint)
{
	return joints[joint].Position;
}

Vector4TupleVector
KinectExtractor::GetDownsampledColorData()
{
	Vector4TupleVector downsampled;
	downsampled.resize(DEPTH_WIDTH * DEPTH_HEIGHT);
	int index = 0;
	for (int i = 0; i < DEPTH_HEIGHT; i++) {
		for (int j = 0; j < DEPTH_WIDTH; j++) {
			const ColorSpacePoint& point = depth2rgb[index];
			if (point.X < 0 || point.Y < 0 || point.X > COLOR_WIDTH || point.Y > COLOR_HEIGHT) {
				downsampled[index] = std::make_tuple(0.f, 0.f, 0.f, 0.f);
			}
			else {
				int idx = (int)point.X + COLOR_WIDTH * (int)point.Y;
				float r = rgbimage[4 * idx + 0] / 255.f;
				float g = rgbimage[4 * idx + 1] / 255.f;
				float b = rgbimage[4 * idx + 2] / 255.f;
				downsampled[index] = std::make_tuple(r, g, b, 1.f);
			}
			// Don't copy alpha channel
			index++;
		}
	}
	return downsampled;
}

PointTupleVector
KinectExtractor::GetDepthLocations()
{
	PointTupleVector cleaned;
	cleaned.reserve(depth2xyz.size());
	for (const auto item : depth2xyz) {
		if (item.X == std::numeric_limits<float>::infinity() ||
			item.Y == std::numeric_limits<float>::infinity() ||
			item.Z == std::numeric_limits<float>::infinity())
		{
			continue;
		}
		cleaned.push_back(std::make_tuple(item.X, item.Y, item.Z));
	}
	cleaned.shrink_to_fit();
	return cleaned;
}