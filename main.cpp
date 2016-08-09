/* note on transitions:
pos.m[0][3]		X	away from screen
pos.m[1][3]		Y	to ceiling
pos.m[2][3]		Z	to left from screen
*/

#include <iostream>
#include <string>
#include <array>

#include <kinect.h>
#include <openvr.h>


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
		std::cerr << "ERROR while initializing" << std::endl;
		return 0;
	}
	vr::TrackedDevicePose_t devices[vr::k_unMaxTrackedDeviceCount];

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
	
	// positions
	vr::HmdMatrix34_t openvr_pos;
	CameraSpacePoint kinect_pos;

	while(true) {
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

		// acquire OpenVR position
		vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0, devices, vr::k_unMaxTrackedDeviceCount);
		for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
			if (devices[i].bPoseIsValid && devices[i].bDeviceIsConnected) {
				if (vr::VRSystem()->GetTrackedDeviceClass(i) == 2) {
					openvr_pos = devices[i].mDeviceToAbsoluteTracking;
					break;
				}
			}
		}

		//output both Y coordinates
		std::cout << "OpenVR = " << openvr_pos.m[1][3] << std::endl;
		std::cout << "Kinect = " << kinect_pos.Y << std::endl;
	}

	vr::VR_Shutdown();
	return 0;
}