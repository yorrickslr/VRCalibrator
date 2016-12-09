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
#include <gtx/vector_angle.hpp>
#include <armadillo>

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
	Vec3(arma::fvec3 const& vec) {
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
	void operator+=(Vec3 const& vec) {
		data += vec.data;
	}
	Vec3 operator-() {
		return -data;
	}

	arma::fvec3 data;
};

struct Mat4 {
	Mat4() {}
	Mat4(arma::fmat44 const& mat) {
		data = mat;
	}
	Mat4(Vec3 const& vec) {
		set_translation(vec.data);
	}
	void set_translation(arma::fvec3 const& vec) {
		data.at(3, 0) = vec.at(0);
		data.at(3, 1) = vec.at(1);
		data.at(3, 2) = vec.at(2);
	}
	void set_rotation(arma::fmat const& mat) {
		for (int i = 0; i < 3; i++) {
			for (int k = 0; k < 3; k++) {
				data.at(k, i) = mat.at(k, i);
			}
		}
	}
	friend std::ostream& operator<< (std::ostream& stream, Mat4 const& mat) {
		stream << mat.data.at(0, 0) << " " << mat.data.at(1, 0) << " " << mat.data.at(2, 0) << " " << mat.data.at(3, 0) << std::endl;
		stream << mat.data.at(0, 1) << " " << mat.data.at(1, 1) << " " << mat.data.at(2, 1) << " " << mat.data.at(3, 1) << std::endl;
		stream << mat.data.at(0, 2) << " " << mat.data.at(1, 2) << " " << mat.data.at(2, 2) << " " << mat.data.at(3, 2) << std::endl;
		stream << mat.data.at(0, 3) << " " << mat.data.at(1, 3) << " " << mat.data.at(2, 3) << " " << mat.data.at(3, 3) << std::endl;
		return stream;
	}
	friend Mat4 operator*(Mat4 const& a, Mat4 const& b) {
		return a.data*b.data;
	}

	arma::fmat data;
};

struct Samples {
	bool add(vr::HmdMatrix34_t const& openvr_pos, CameraSpacePoint const& kinect_pos) {
		Vec3 openvr_vec = Vec3(openvr_pos);
		Vec3 kinect_vec = Vec3(kinect_pos);
		//kinect_vec.data[0] *= -1;
		//kinect_vec.data[2] *= -1;
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
	void add(Vec3 const& openvr_vec, Vec3 const& kinect_vec) {
		std::pair<Vec3, Vec3> temp = std::pair<Vec3, Vec3>(openvr_vec, kinect_vec);
		data.push_back(temp);
		length++;
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

	arma::fmat44 calibrate() {
		arma::fvec openvr_centroid(3, arma::fill::zeros);
		arma::fvec kinect_centroid(3, arma::fill::zeros);

		for (auto i : data) {
			openvr_centroid.at(0) += i.first.data.at(0);
			openvr_centroid.at(1) += i.first.data.at(1);
			openvr_centroid.at(2) += i.first.data.at(2);
			kinect_centroid.at(0) += i.second.data.at(0);
			kinect_centroid.at(1) += i.second.data.at(1);
			kinect_centroid.at(2) += i.second.data.at(2);
		}
		openvr_centroid.at(0) /= length;
		openvr_centroid.at(1) /= length;
		openvr_centroid.at(2) /= length;
		kinect_centroid.at(0) /= length;
		kinect_centroid.at(1) /= length;
		kinect_centroid.at(2) /= length;

		arma::fmat openvr_centered(length, 3);
		arma::fmat kinect_centered(length, 3);

		for (int i = 0; i < length; i++) {
			openvr_centered.at(i, 0) = data.at(i).first.data.at(0) - openvr_centroid.at(0);
			openvr_centered.at(i, 1) = data.at(i).first.data.at(1) - openvr_centroid.at(1);
			openvr_centered.at(i, 2) = data.at(i).first.data.at(2) - openvr_centroid.at(2);

			kinect_centered.at(i, 0) = data.at(i).second.data.at(0) - kinect_centroid.at(0);
			kinect_centered.at(i, 1) = data.at(i).second.data.at(1) - kinect_centroid.at(1);
			kinect_centered.at(i, 2) = data.at(i).second.data.at(2) - kinect_centroid.at(2);
		}

		arma::fmat M;
		M = openvr_centered.t() * kinect_centered;

		arma::fmat U;
		arma::fvec S;
		arma::fmat Vt;
		arma::svd(U, S, Vt, M);

		arma::fmat R = Vt * U.t();

		// handle special reflection case
		if (arma::det(R) < 0) {
			std::cout << "INFORMATION: reflection detected!" << std::endl;
			Vt.at(0, 2) *= -1;
			Vt.at(1, 2) *= -1;
			Vt.at(2, 2) *= -1;
			R = Vt * U.t();
		}

		R = R.i();

		kinect_centroid = R * kinect_centroid;

		kinect_centroid.insert_rows(3, 1);
		openvr_centroid.insert_rows(3, 1);

		arma::fmat44 offset(arma::fill::eye);
		offset.at(0, 3) = openvr_centroid.at(0) - kinect_centroid.at(0);
		offset.at(1, 3) = openvr_centroid.at(1) - kinect_centroid.at(1);
		offset.at(2, 3) = openvr_centroid.at(2) - kinect_centroid.at(2);
		offset.at(3, 3) = 1;

		R.insert_rows(3, 1);
		R.insert_cols(3, 1);
		R.at(3, 3) = 1;

		arma::fmat44 result = offset * R;

		return result;
	}

	std::vector<std::pair<Vec3, Vec3>> data;
	unsigned length = 0;
};

#endif