#pragma once

#ifndef VRCALIBRATOR_BBOX
#define VRCALIBRATOR_BBOX

#include <vector>
#include <array>
#include <utility>
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtx/string_cast.hpp>
#include <gtx/matrix_decompose.hpp>

struct Mat4;
struct Vec3;
struct Samples;


struct Vec3 {
	Vec3() {}
	Vec3(vr::HmdMatrix34_t const& mat) {
		data[0] = mat.m[0][3];
		data[1] = mat.m[1][3];
		data[2] = mat.m[2][3];
	}
	Vec3(CameraSpacePoint const& mat) {
		data[0] = mat.X;
		data[1] = mat.Y;
		data[2] = mat.Z;
	}
	Vec3(glm::vec3 const& vec) {
		data = vec;
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
	friend Vec3 operator- (Vec3 const& a, Vec3 const& b) {
		return a.data - b.data;
	}
	friend Vec3 operator+  (Vec3 const& a, Vec3 const& b) {
		return a.data + b.data;
	}
	friend Vec3 operator*(Vec3 const& a, Vec3 const& b) {
		return a.data * b.data;
	}

	glm::vec3 data;
};

struct Mat4 {
	Mat4() {}
	Mat4(glm::mat4 const& mat) {
		data = mat;
	}
	Mat4(Vec3 const& vec) {
		data = glm::translate(data, vec.data);
	}
	void translate(Vec3 const& vec) {
		data = glm::translate(data, vec.data);
	}
	void print() {
		std::cout.precision(5);
		std::cout << data[0][0] << " " << data[1][0] << " " << data[2][0] << " " << data[3][0] << std::endl;
		std::cout << data[0][1] << " " << data[1][1] << " " << data[2][1] << " " << data[3][1] << std::endl;
		std::cout << data[0][2] << " " << data[1][2] << " " << data[2][2] << " " << data[3][2] << std::endl;
		std::cout << data[0][3] << " " << data[1][3] << " " << data[2][3] << " " << data[3][3] << std::endl;
	}
	friend Mat4 operator*(Mat4 const& a, Mat4 const& b) {
		return a.data*b.data;
	}

	glm::mat4 data;
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
	Mat4 calibrate() {
		Mat4 calibration, temp;

		Vec3 openvr_a = data[0].first;
		Vec3 openvr_b = data[1].first;
		Vec3 kinect_a = data[0].second;
		Vec3 kinect_b = data[1].second;
		Vec3 kinect_to_openvr = openvr_a - kinect_a;
		kinect_a = kinect_a + kinect_to_openvr;
		kinect_b = kinect_b + kinect_to_openvr;
		calibration.translate(kinect_to_openvr);

		// DEBUG
		std::string message;
		std::cout << "kinect_to_openvr = " << kinect_to_openvr << std::endl;
		if (kinect_a == openvr_a) {
			message = "\t ~~~ success ~~~";
		}
		else {
			message = "\t ~~~ failure ~~~";
		}
		std::cout << "kinect_a to openvr: " << kinect_a << message << std::endl;
		if (kinect_b == openvr_b) {
			message = "\t ~~~ success ~~~";
		}
		else {
			message = "\t ~~~ failure ~~~";
		}
		std::cout << "kinect_b to openvr: " << kinect_b << message << std::endl;
		calibration.print();


		return Mat4{};
	}

	std::vector<std::pair<Vec3, Vec3>> data;
	unsigned length = 0;
};

#endif