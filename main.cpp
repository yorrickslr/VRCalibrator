#include <iostream>
#include <string>
#include <array>

#include <openvr.h>


int main(int argc, char** argv) {
	vr::EVRInitError error = vr::VRInitError_None;
	vr::IVRSystem* hmd = vr::VR_Init(&error, vr::VRApplication_Background);
	if (error != vr::VRInitError_None) {
		hmd = NULL;
		std::cerr << "ERROR while initializing" << std::endl;
	}
	
	vr::TrackedDevicePose_t devices[vr::k_unMaxTrackedDeviceCount];
	for(int j=0; j<1000; j++) {
		vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0, devices, vr::k_unMaxTrackedDeviceCount);
		for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
			if (devices[i].bPoseIsValid && devices[i].bDeviceIsConnected) {
				if (vr::VRSystem()->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_HMD) {
					std::cout << "device is HMD" << std::endl;
					vr::HmdMatrix34_t pos = devices[i].mDeviceToAbsoluteTracking;
					std::cout << pos.m[0][0] << " " << pos.m[0][1] << " " << pos.m[0][2] << " " << pos.m[0][3] << std::endl;
					std::cout << pos.m[1][0] << " " << pos.m[1][1] << " " << pos.m[1][2] << " " << pos.m[1][3] << std::endl;
					std::cout << pos.m[2][0] << " " << pos.m[2][1] << " " << pos.m[2][2] << " " << pos.m[2][3] << std::endl;
					std::cout << 1.0 << " " << 1.0 << " " << 1.0 << " " << 1.0 << std::endl;
				}
				else if(vr::VRSystem()->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_Controller) {
					std::cout << "device is controller" << std::endl;
					vr::HmdMatrix34_t pos = devices[i].mDeviceToAbsoluteTracking;
					std::cout << pos.m[0][0] << " " << pos.m[0][1] << " " << pos.m[0][2] << " " << pos.m[0][3] << std::endl;
					std::cout << pos.m[1][0] << " " << pos.m[1][1] << " " << pos.m[1][2] << " " << pos.m[1][3] << std::endl;
					std::cout << pos.m[2][0] << " " << pos.m[2][1] << " " << pos.m[2][2] << " " << pos.m[2][3] << std::endl;
					std::cout << 1.0 << " " << 1.0 << " " << 1.0 << " " << 1.0 << std::endl;
				}
				else {
					std::cout << "device unknown" << std::endl;
					vr::HmdMatrix34_t pos = devices[i].mDeviceToAbsoluteTracking;
					std::cout << pos.m[0][0] << " " << pos.m[0][1] << " " << pos.m[0][2] << " " << pos.m[0][3] << std::endl;
					std::cout << pos.m[1][0] << " " << pos.m[1][1] << " " << pos.m[1][2] << " " << pos.m[1][3] << std::endl;
					std::cout << pos.m[2][0] << " " << pos.m[2][1] << " " << pos.m[2][2] << " " << pos.m[2][3] << std::endl;
					std::cout << 1.0 << " " << 1.0 << " " << 1.0 << " " << 1.0 << std::endl;
				}
			}
		}
	}

	vr::VR_Shutdown();
	return 0;
}