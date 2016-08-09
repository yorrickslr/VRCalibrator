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
		return 0;
	}

	std::cout.precision(8);
	std::cout.fixed;
	
	vr::TrackedDevicePose_t devices[vr::k_unMaxTrackedDeviceCount];
	for(int j=0; j<1000; j++) {
		vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0, devices, vr::k_unMaxTrackedDeviceCount);
		for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
			if (devices[i].bPoseIsValid && devices[i].bDeviceIsConnected) {
				if (vr::VRSystem()->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_HMD) {
					std::cout << "device is HMD" << std::endl;
					vr::HmdMatrix34_t pos = devices[i].mDeviceToAbsoluteTracking;
					std::cout << pos.m[0][0] << "\t" << pos.m[0][1] << "\t" << pos.m[0][2] << "\t" << pos.m[0][3] << std::endl;
					std::cout << pos.m[1][0] << "\t" << pos.m[1][1] << "\t" << pos.m[1][2] << "\t" << pos.m[1][3] << std::endl;
					std::cout << pos.m[2][0] << "\t" << pos.m[2][1] << "\t" << pos.m[2][2] << "\t" << pos.m[2][3] << std::endl;
					std::cout << "1.0000000" << "\t" << "1.0000000" << "\t" << "1.0000000" << "\t" << "1.0000000" << std::endl;
					std::cout << std::endl;
				}
				else if(vr::VRSystem()->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_Controller) {
					std::cout << "device is controller" << std::endl;
					vr::HmdMatrix34_t pos = devices[i].mDeviceToAbsoluteTracking;
					std::cout << pos.m[0][0] << "\t" << pos.m[0][1] << "\t" << pos.m[0][2] << "\t" << pos.m[0][3] << std::endl;
					std::cout << pos.m[1][0] << "\t" << pos.m[1][1] << "\t" << pos.m[1][2] << "\t" << pos.m[1][3] << std::endl;
					std::cout << pos.m[2][0] << "\t" << pos.m[2][1] << "\t" << pos.m[2][2] << "\t" << pos.m[2][3] << std::endl;
					std::cout << "1.0000000" << "\t" << "1.0000000" << "\t" << "1.0000000" << "\t" << "1.0000000" << std::endl;
					std::cout << std::endl;
				}
				else {
					std::cout << "device unknown" << std::endl;
					vr::HmdMatrix34_t pos = devices[i].mDeviceToAbsoluteTracking;
					std::cout << pos.m[0][0] << "\t" << pos.m[0][1] << "\t" << pos.m[0][2] << "\t" << pos.m[0][3] << std::endl;
					std::cout << pos.m[1][0] << "\t" << pos.m[1][1] << "\t" << pos.m[1][2] << "\t" << pos.m[1][3] << std::endl;
					std::cout << pos.m[2][0] << "\t" << pos.m[2][1] << "\t" << pos.m[2][2] << "\t" << pos.m[2][3] << std::endl;
					std::cout << "1.0000000" << "\t" << "1.0000000" << "\t" << "1.0000000" << "\t" << "1.0000000" << std::endl;
					std::cout << std::endl;
				}
			}
		}
	}

	vr::VR_Shutdown();
	return 0;
}