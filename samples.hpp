#pragma once

#ifndef VRCALIBRATOR_BBOX
#define VRCALIBRATOR_BBOX

#include <vector>
#include <array>
#include <utility>

struct Vec3 {
	Vec3(vr::HmdMatrix34_t const& mat) {
		data.assign(mat.m[0][3]);
		data.assign(mat.m[1][3]);
		data.assign(mat.m[2][3]);
	}
	Vec3(CameraSpacePoint const& mat) {
		data.assign(mat.X);
		data.assign(mat.Y);
		data.assign(mat.Z);
	}
	bool containsZero() {
		if (data[0] == 0.0)
			return 1;
		if (data[1] == 0.0)
			return 1;
		if (data[2] == 0.0)
			return 1;
		return 0;
	}
	friend std::ostream& operator<< (std::ostream& stream, const Vec3& vec) {
		std::cout.precision(5);
		return stream << std::fixed << vec.data[0] << ", " << vec.data[1] << ", " << vec.data[2];
	}
	bool operator== (Vec3 const& vec) {
		if (data[0] != vec.data[0])
			return 0;
		if (data[1] != vec.data[1])
			return 0;
		if (data[2] != vec.data[2])
			return 0;
		return 1;
	}

	std::array<float, 3> data;
};

struct Samples {
	bool add(vr::HmdMatrix34_t const& openvr_pos, CameraSpacePoint const& kinect_pos) {
		Vec3 openvr_vec = Vec3(openvr_pos);
		Vec3 kinect_vec = Vec3(kinect_pos);
		if (openvr_vec.containsZero() || kinect_vec.containsZero())
			return 0;
		if (length>0 && openvr_vec == data.back().first)
			return 0;
		if (length>0 && kinect_vec == data.back().second)
			return 0;
		std::pair<Vec3, Vec3> temp = std::pair<Vec3, Vec3>(openvr_vec, kinect_vec);
		data.push_back(temp);
		length++;
		return 1;
	}
	bool pop() {
		if (length == 0)
			return 0;
		data.pop_back();
		length--;
		return 1;
	}
	bool check() {
		// TODO check if pairs contain same values and delete them
		return 1;
	}

	std::vector<std::pair<Vec3, Vec3>> data;
	int length = 0;
};

#endif