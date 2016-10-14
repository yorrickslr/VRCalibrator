/* note on transitions:
pos.m[0][3]		X	away from screen
pos.m[1][3]		Y	to ceiling
pos.m[2][3]		Z	to left from screen
*/

#include <iostream>
#include <fstream>
#include <string>
#include <array>

#include <kinect.h>
#include <openvr.h>

#include "samples.hpp"


template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


void errexit(IKinectSensor* sensor, std::string message) {
	std::cerr << "ERROR: " << message << std::endl;
	sensor->Close();
	SafeRelease(sensor);
	exit(0);
}


int main(int argc, char** argv) {
	// initialize OpenVR
	vr::EVRInitError error = vr::VRInitError_None;
	vr::IVRSystem* hmd = vr::VR_Init(&error, vr::VRApplication_Background);
	if (error != vr::VRInitError_None) {
		hmd = NULL;
		std::cerr << "ERROR while initializing openVR" << std::endl;
		return 0;
	}

	// initialize KinectSDK
	HRESULT hr;
	IKinectSensor* sensor = NULL;
	hr = GetDefaultKinectSensor(&sensor);
	if (FAILED(hr))
		errexit(sensor, "cannot find default kinect");
	hr = sensor->Open();
	if (FAILED(hr))
		errexit(sensor, "cannot open stream");
	IBodyFrameSource* source = NULL;
	hr = sensor->get_BodyFrameSource(&source);
	if (FAILED(hr))
		errexit(sensor, "cannot get frame source");
	IBodyFrameReader* reader = NULL;
	hr = source->OpenReader(&reader);
	if (FAILED(hr))
		errexit(sensor, "cannot open frame reader");
	source->Release();
	source = NULL;
	IBodyFrame* frame = NULL;

	// samples
	Samples samples;

	// positions
	vr::HmdMatrix34_t openvr_pos;
	CameraSpacePoint kinect_pos;

	std::cout << "Initialization finished..." << std::endl;

	while (true) {
		//acquire Kinect position
		hr = reader->AcquireLatestFrame(&frame);
		if (FAILED(hr)) {
			SafeRelease(frame);
			continue;
		}
		IBody* bodies[BODY_COUNT] = { 0 };
		hr = frame->GetAndRefreshBodyData(BODY_COUNT, bodies);
		if (FAILED(hr))
			errexit(sensor, "cannot get body data");
		for (IBody* body : bodies) {
			BOOLEAN tracked;
			body->get_IsTracked(&tracked);
			if (!tracked)
				continue;
			Joint joints[JointType_Count];
			body->GetJoints(JointType_Count, joints);
			kinect_pos = joints[11].Position;
		}
		SafeRelease(frame);

		//acquire openvr position and handle events
		vr::VREvent_t event;
		vr::TrackedDevicePose_t pose;
		while (hmd->PollNextEventWithPose(vr::TrackingUniverseStanding, &event, sizeof(event), &pose)) {
			if (hmd->GetTrackedDeviceClass(event.trackedDeviceIndex) == vr::TrackedDeviceClass_Controller) {
				if (event.eventType == vr::VREvent_TrackedDeviceDeactivated) {
					std::cout << "WARNING: device signal lost" << std::endl;
				}
				else if (event.eventType == vr::VREvent_TrackedDeviceActivated) {
					std::cout << "INFORMATION: device found" << std::endl;
				}
				else if (event.data.controller.button == vr::k_EButton_ApplicationMenu) {
					if (event.eventType == vr::VREvent_ButtonPress) {
						openvr_pos = pose.mDeviceToAbsoluteTracking;
						if (samples.add(openvr_pos, kinect_pos))
							std::cout << "added sample " << samples.length << std::endl;
						else {
							std::cout << "did not add sample:" << std::endl;
							std::cout << "    Openvr - " << Vec3(openvr_pos) << std::endl;
							std::cout << "    Kinect - " << Vec3(kinect_pos) << std::endl;
						}
					}
				}
				else if (event.data.controller.button == vr::k_EButton_Grip) {
					goto exit;
				}
			}
		}
	}

exit:
	std::cout << "\n---------- OpenVR ---------   SAMPLES   --------- Kinect ----------" << std::endl;
	for (std::pair<Vec3, Vec3> point : samples.data) {
		std::cout << point.first << "\t\t" << point.second << std::endl;
	}
	
	sensor->Close();
	vr::VR_Shutdown();

	std::cout << "\nPress enter to calibrate..." << std::endl;
	std::cin.get();
	
	Mat4 calibration = samples.calibrate2();

	std::ofstream file;
	file.open("calibration.txt");
	file << calibration;
	file.close();

/*
	Samples samples;
	glm::vec3 openvr_vec, kinect_vec;

	Mat4 calibration;

	openvr_vec = glm::vec3(0,0,0);
	kinect_vec = glm::vec3(0,-1,-2);
	samples.add(openvr_vec, kinect_vec);

	openvr_vec = glm::vec3(0,1,0);
	kinect_vec = glm::vec3(0,0,-2);
	samples.add(openvr_vec, kinect_vec);

	openvr_vec = glm::vec3(0, 0, -1);
	kinect_vec = glm::vec3(-1, -1, -2);
	samples.add(openvr_vec, kinect_vec);

	calibration = samples.calibrate2();
*/
	Mat4 openvr = Mat4();
	Mat4 kinect;
	Mat4 diff;
	Mat4 temp;

	std::cout << "Empty diff:" << std::endl;
	std::cout << diff << std::endl;

	openvr.translate(samples.data[0].first.data);
	kinect.translate(samples.data[0].second.data);

	temp = kinect * calibration;
	for (int i = 0; i < 4; i++) {
		for (int k = 0; k < 4; k++) {
			diff.data[i][k] = abs(openvr.data[i][k] - temp.data[i][k]);
		}
	}

	std::cout << "Difference 1:" << std::endl;
	std::cout << diff << std::endl << std::endl;

	openvr = Mat4();
	kinect = Mat4();

	openvr.translate(samples.data[1].first.data);
	kinect.translate(samples.data[1].second.data);

	temp = kinect * calibration;
	for (int i = 0; i < 4; i++) {
		for (int k = 0; k < 4; k++) {
			diff.data[i][k] = abs(openvr.data[i][k] - temp.data[i][k]);
		}
	}

	std::cout << "Difference 2:" << std::endl;
	std::cout << diff << std::endl << std::endl;

	openvr = Mat4();
	kinect = Mat4();

	openvr.translate(samples.data[2].first.data);
	kinect.translate(samples.data[2].second.data);

	temp = kinect * calibration;
	for (int i = 0; i < 4; i++) {
		for (int k = 0; k < 4; k++) {
			diff.data[i][k] = abs(openvr.data[i][k] - temp.data[i][k]);
		}
	}

	std::cout << "Difference 3:" << std::endl;
	std::cout << diff << std::endl << std::endl;

	std::cout << "test" << std::endl;
	Mat4 a, b;
	a.data[3][0] = 42;
	std::cout << a << std::endl;
	std::cout << b << std::endl;
	std::cout << a*b << std::endl;
	
	/*
	openvr = Mat4();
	kinect = Mat4();
	openvr.translate(samples.data[0].first.data);
	kinect.translate(samples.data[0].second.data);
	std::cout << openvr << std::endl;
	std::cout << kinect << std::endl;
	temp = kinect * calibration;
	std::cout << std::endl;
	std::cout << temp << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;
	*/

	return 0;
}