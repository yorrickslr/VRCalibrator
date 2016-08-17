#pragma once

#ifndef VRCALIBRATOR_BBOX
#define VRCALIBRATOR_BBOX

#include <vector>
#include <array>
#include <utility>

struct Vec3 {
	Vec3(vr::HmdMatrix34_t const& mat) {
		data.at(0) = mat.m[0][3];
		data.at(1) = mat.m[1][3];
		data.at(2) = mat.m[2][3];
	}
	Vec3(CameraSpacePoint const& mat) {
		data.at(0) = mat.X;
		data.at(1) = mat.Y;
		data.at(2) = mat.Z;
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
	unsigned length = 0;
};

struct Mat4 {
	Mat4() {
		make_identity();
	}
	Mat4(Vec3 const& vec) {
		make_identity();
		data.at(12) = vec.data.at(0);
		data.at(13) = vec.data.at(1);
		data.at(14) = vec.data.at(2);
	}
	void print() {
		std::cout.precision(5);
		std::cout << data.at(0) << " " << data.at(4) << " " << data.at(8) << " " << data.at(12) << std::endl;
		std::cout << data.at(1) << " " << data.at(5) << " " << data.at(9) << " " << data.at(13) << std::endl;
		std::cout << data.at(2) << " " << data.at(6) << " " << data.at(10) << " " << data.at(14) << std::endl;
		std::cout << data.at(3) << " " << data.at(7) << " " << data.at(11) << " " << data.at(15) << std::endl;
	}
	void make_identity() {
		data.fill(0.0);
		data.at(0) = 1.0;
		data.at(5) = 1.0;
		data.at(10) = 1.0;
		data.at(15) = 1.0;
	}
	friend Mat4 operator*(Mat4 const& a, Mat4 const& b) {
		// TODO matrix multiplication
		Mat4 result;
		result.data.at(0) = 42.42;
		return result;
	}

	std::array<float, 16> data;
};

#endif