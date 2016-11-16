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
	/*
	Mat4 calibrate() {
		Mat4 calibration;

		Vec3 openvr_a = data[0].first;
		Vec3 openvr_b = data[1].first;
		Vec3 kinect_a = data[0].second;
		Vec3 kinect_b = data[1].second;

		// step 1
		Vec3 kinect_to_openvr = openvr_a - kinect_a;
		kinect_a = kinect_a + kinect_to_openvr;
		kinect_b = kinect_b + kinect_to_openvr;
		calibration.translate(kinect_to_openvr);

		// step 2
		Vec3 to_null = -openvr_a;
		openvr_a += to_null;
		openvr_b += to_null;
		kinect_a += to_null;
		kinect_b += to_null;
		calibration.translate(to_null);

		// step 3
		Vec3 rotation_axis = glm::cross(glm::normalize(openvr_b.data), glm::normalize(kinect_b.data));
		std::cout << "rotation axis: " << rotation_axis << std::endl;
		float deg = glm::angle(glm::normalize(openvr_b.data), glm::normalize(kinect_b.data));
		std::cout << "rotation degree: " << deg << std::endl;
		calibration.rotate(deg, rotation_axis.data);

		// step 4
		calibration.translate(-to_null);

		return calibration;
	}*/

	Mat4 calibrate2() {
		arma::fvec3 openvr_centroid;
		arma::fvec3 kinect_centroid;

		std::cout << "Length " << length << std::endl;

		for (auto i : data) {
			openvr_centroid[0] += i.second.data[0];
			openvr_centroid[1] += i.second.data[1];
			openvr_centroid[2] += i.second.data[2];
			kinect_centroid[0] += i.first.data[0];
			kinect_centroid[1] += i.first.data[1];
			kinect_centroid[2] += i.first.data[2];
		}
		openvr_centroid[0] /= length;
		openvr_centroid[1] /= length;
		openvr_centroid[2] /= length;
		kinect_centroid[0] /= length;
		kinect_centroid[1] /= length;
		kinect_centroid[2] /= length;

		std::cout << "OpenVR centroid:" << std::endl;
		std::cout << openvr_centroid.t() << std::endl;

		std::cout << "Kinect centroid:" << std::endl;
		std::cout << kinect_centroid.t() << std::endl;

		arma::fmat openvr_centered(length, 3);
		arma::fmat kinect_centered(length, 3);

		for (int i = 0; i < length; i++) {
			openvr_centered.at(i, 0) = data.at(i).second.data[0] - openvr_centroid.at(0);
			openvr_centered.at(i, 1) = data.at(i).second.data[1] - openvr_centroid.at(1);
			openvr_centered.at(i, 2) = data.at(i).second.data[2] - openvr_centroid.at(2);

			kinect_centered.at(i, 0) = data.at(i).first.data[0] - kinect_centroid.at(0);
			kinect_centered.at(i, 1) = data.at(i).first.data[1] - kinect_centroid.at(1);
			kinect_centered.at(i, 2) = data.at(i).first.data[2] - kinect_centroid.at(2);
		}
		
		std::cout << "OpenVR centered:" << std::endl;
		std::cout << openvr_centered << std::endl;

		std::cout << "Kinect centered:" << std::endl;
		std::cout << kinect_centered << std::endl;

		arma::fmat openvr_transposed(3, length);
		openvr_transposed = openvr_centered.t();

		arma::fmat M;
		M = openvr_transposed * kinect_centered;

		std::cout << "M:" << std::endl;
		std::cout << M << std::endl;

		arma::fmat U;
		arma::fvec S;
		arma::fmat Vt;
		arma::svd(U, S, Vt, M);

		std::cout << "U:" << std::endl;
		std::cout << U << std::endl;
		std::cout << "S:" << std::endl;
		std::cout << S << std::endl;
		std::cout << "Vt:" << std::endl;
		std::cout << Vt.t() << std::endl;

		arma::fmat R = Vt * U.t();

		std::cout << "R:" << std::endl;
		std::cout << R << std::endl;

		// handle special reflection case
		if (arma::det(R) < 0) {
			std::cout << "INFORMATION: reflection detected!" << std::endl;
			Vt.at(0, 2) *= -1;
			Vt.at(1, 2) *= -1;
			Vt.at(2, 2) *= -1;
			std::cout << "Vt:" << std::endl;
			std::cout << Vt.t() << std::endl;
			R = Vt * U.t();
			std::cout << "R:" << std::endl;
			std::cout << R << std::endl;
		}
/*
		R = R.i();

		std::cout << "inverted R:" << std::endl;
		std::cout << R << std::endl;
*/
		arma::vec rotated_kinect_sample0 = arma::vec( arma::zeros(3));
		for (int i = 0; i < 3; ++i) {
			rotated_kinect_sample0[i] = data[0].second.data[i];
		}

		//NOTE: KINECT AND OPENVR SAMPLES ARE SWAPPED RIGHT NOW
		rotated_kinect_sample0 = R * (rotated_kinect_sample0);


		auto t = -R * openvr_centroid + kinect_centroid;

		std::cout << "Own translation:(\n";
		
		arma::fvec3 offset;

		for (int i = 0; i < 3; ++i) {
			std::cout << data[0].first.data[i] - kinect_centroid[i];
			offset[i] = data[0].first.data[i] - kinect_centroid[i];
			
			if (i != 2) {
				std::cout << ", ";
			}
		}
		std::cout << ");\n";
		
		std::cout << "offset: " << std::endl;
		std::cout << offset << std::endl;

		std::cout << "t:" << std::endl;
		std::cout << t << std::endl;

		Mat4 result;
		Mat4 translation;

		result.set_rotation(R);
		
		std::cout << "Result with rotation:" << std::endl;
		std::cout << result << std::endl;

		translation.set_translation(offset);

		std::cout << " result * translation" << std::endl;
		std::cout << result * translation << std::endl;

		std::cout << " translation * result" << std::endl;
		std::cout << translation * result << std::endl;

		result = translation * result;



		return result;
	}

	arma::fmat44 calibrate3() {
		std::cout << "length: " << length << std::endl;
		std::cout << "data.size(): " << data.size() << std::endl;

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

		std::cout << "M:" << std::endl;
		std::cout << M << std::endl;

		arma::fmat U;
		arma::fvec S;
		arma::fmat Vt;
		arma::svd(U, S, Vt, M);

		arma::fmat R = Vt * U.t();

		std::cout << "R:" << std::endl;
		std::cout << R << std::endl;

		// handle special reflection case
		if (arma::det(R) < 0) {
			std::cout << "INFORMATION: reflection detected!" << std::endl;
			Vt.at(0, 2) *= -1;
			Vt.at(1, 2) *= -1;
			Vt.at(2, 2) *= -1;
			std::cout << "Vt:" << std::endl;
			std::cout << Vt.t() << std::endl;
			R = Vt * U.t();
			std::cout << "R:" << std::endl;
			std::cout << R << std::endl;
		}

		R = R.i();

		std::cout << "OpenVR centroid:" << std::endl;
		std::cout << openvr_centroid << std::endl;

		std::cout << "Kinect centroid:" << std::endl;
		std::cout << kinect_centroid << std::endl;

		kinect_centroid = R * kinect_centroid;

		std::cout << "rotated kinect centroid:" << std::endl;
		std::cout << kinect_centroid << std::endl;

		kinect_centroid.insert_rows(3, 1);
		openvr_centroid.insert_rows(3, 1);

		arma::fmat44 offset(arma::fill::eye);
		offset.at(0, 3) = openvr_centroid.at(0) - kinect_centroid.at(0);
		offset.at(1, 3) = openvr_centroid.at(1) - kinect_centroid.at(1);
		offset.at(2, 3) = openvr_centroid.at(2) - kinect_centroid.at(2);
		offset.at(3, 3) = 1;

		std::cout << "offset:" << std::endl;
		std::cout << offset << std::endl;

		R.insert_rows(3, 1);
		R.insert_cols(3, 1);
		R.at(3, 3) = 1;


		arma::fmat44 result = offset * R;
		std::cout << "result:" << std::endl;
		std::cout << result << std::endl;

		return result;
	}

	std::vector<std::pair<Vec3, Vec3>> data;
	unsigned length = 0;
};

#endif