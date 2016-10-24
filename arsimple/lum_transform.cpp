/*Copyright 2016 Google Inc.
Copyright 2016 Lukas Murmann

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.*/

#include "lum_transform.h"
#include "lum_constants.h"
#include <Eigen/Dense>

namespace lum {
	float deg2rad(float deg) {
		return deg / 180.0f * (float)M_PI;
	}
	float rad2deg(float rad) {
		return rad /(float) M_PI * 180.0f;
	}
	float radmod2pi(float p) {
		while (p > M_PIf * 2) {
			p -= M_PIf * 2;
		}
		while (p < 0) {
			p += M_PIf * 2;
		}
		return p;
	}


	Eigen::Matrix4f planeBasis(const Eigen::Vector4f& plane) {
		using namespace Eigen;
		// first, find two vectors that are orthogonal to plane normal
		Vector3f n = plane.topRows(3);
		float d = -plane(3);
		Vector3f v1 = Vector3f::Random();
		v1 = v1 - n * n.dot(v1);
		Vector3f v2 = n.cross(v1);

		Eigen::Matrix4f ret;
		ret.setZero();
		ret.block(0, 3, 3, 1) = n * d; // origin
		ret(3,3) = 1;
		ret.block(0, 0, 3, 1) = v1;
		ret.block(0, 1, 3, 1) = v2;
		ret.block(0, 2, 3, 1) = n;
		return ret;
	}

    Eigen::Matrix4f rotateX(float angle) {
        Eigen::Matrix4f ret;
        ret.setIdentity();
        ret.topLeftCorner(3, 3) = Eigen::AngleAxis<float> {angle, Eigen::Vector3f::UnitX()}.matrix();
        return ret;
    }
    Eigen::Matrix4f rotateY(float angle) {
        Eigen::Matrix4f ret;
        ret.setIdentity();
        ret.topLeftCorner(3, 3) = Eigen::AngleAxis<float> {angle, Eigen::Vector3f::UnitY()}.matrix();
        return ret;
    }
    Eigen::Matrix4f rotateZ(float angle) {
        Eigen::Matrix4f ret;
        ret.setIdentity();
        ret.topLeftCorner(3, 3) = Eigen::AngleAxis<float> {angle, Eigen::Vector3f::UnitZ()}.matrix();
        return ret;
    }

    Eigen::Matrix4f scaleX(float sx) {
        Eigen::Matrix4f ret;
        ret.setIdentity();
        ret(0,0) = sx;
        return ret;
    }
    Eigen::Matrix4f scaleY(float sy) {
        Eigen::Matrix4f ret;
        ret.setIdentity();
        ret(1,1) = sy;
        return ret;
    }
    Eigen::Matrix4f scaleZ(float sz) {
        Eigen::Matrix4f ret;
        ret.setIdentity();
        ret(2,2) = sz;
        return ret;
    }

    Eigen::Matrix4f scaleXYZ(float sx, float sy, float sz) {
        Eigen::Matrix4f ret;
        ret.setIdentity();
        ret(0,0) = sx;
        ret(1,1) = sy;
        ret(2,2) = sz;
        return ret;
    }
    Eigen::Matrix4f scaleXYZ(float s) {
        return scaleXYZ(s, s, s);
    }

    Eigen::Matrix4f translateX(float x) {
        Eigen::Matrix4f ret;
        ret.setIdentity();
        ret(0, 3) = x;
        return ret;
    }
    Eigen::Matrix4f translateY(float y) {
        Eigen::Matrix4f ret;
        ret.setIdentity();
        ret(1, 3) = y;
        return ret;
    }
    Eigen::Matrix4f translateZ(float z) {
        Eigen::Matrix4f ret;
        ret.setIdentity();
        ret(2, 3) = z;
        return ret;
    }
    Eigen::Matrix4f translateXYZ(float x, float y, float z) {
        Eigen::Matrix4f ret;
        ret.setIdentity();
        ret.topRightCorner(3, 1) = Eigen::Vector3f(x, y, z);
        return ret;
    }
    Eigen::Matrix4f rotateTo(float x, float y, float z) {
        Eigen::Vector3f target_dir(x, y, z);
        Eigen::Vector3f current_dir(0, 1, 0); // assume y == up vector;
        Eigen::Vector3f axis = current_dir.cross(target_dir);
        float cosangle = current_dir.dot(target_dir) / target_dir.norm();
        float angle = acos(cosangle);
        Eigen::AngleAxisf aa;
        aa.angle() = angle;
        aa.axis() = axis;

        Eigen::Matrix4f ret;
        ret.setIdentity();
        ret.topLeftCorner(3,3) = aa.matrix();
        return ret;
    }


    Eigen::Matrix4f fromTo(const Eigen::Vector3f & from, const Eigen::Vector3f & to) {
        Eigen::Matrix4f ret;

        Eigen::Vector3f diff = to - from;
        float s = diff.norm();
        diff.normalize();

        Eigen::Matrix4f R = rotateTo(diff(0), diff(1), diff(2));
        Eigen::Matrix4f S = scaleY(s);
        Eigen::Matrix4f T = translateXYZ(from(0), from(1), from(2));

        return T * R * S;
    }

	Eigen::Matrix3f angleAxis(const Eigen::Vector3f& from, float alpha) {
		float s = sin(alpha);
		float c = cos(alpha);
		float t = 1 - c;

		Eigen::Vector3f n = from.normalized();

		float x = n(0);
		float y = n(1);
		float z = n(2);

		Eigen::Matrix3f ret;
		ret <<
			t*x*x + c, t*x*y - s*z, t*x*z + s*y,
			t*x*y + s*z, t*y*y + c, t*y*z - s*x,
			t*x*z - s*y, t*y*z + s*x, t*z*z + c;
		return ret;
	}

	float fovyTofovx(float fovy, float aspect) {
		float nearplane_h_half = tanf(fovy / 2.0f);
		float nearplane_w_half = aspect * nearplane_h_half;
		return 2.0f*atanf(nearplane_w_half);
	}
	float fovxTofovy(float fovx, float aspect) {
		float nearplane_w_half = tanf(fovx / 2.0f);
		float nearplane_h_half = nearplane_w_half / aspect;
		return 2.0f*atanf(nearplane_h_half);
	}

	Eigen::Matrix4f projectionMatrix(float fovy,
		float aspect,
		float n,
		float f) {
		Eigen::Matrix4f ret;
		ret.setZero();
		assert(n <= 0); // please make both clip planes negative.
		assert(f < 0); // clip planes should be specified in view coordinates

		// t == top of near clipping plane
		// r == right of near clipping plane
		// see fundamentals of computer graphics 3rd ed. ~ p172

		float tanfoo = tanf(fovy / 2);
		float t = tanfoo * -n;
		float r = aspect * t;

		assert(t >= 0);
		assert(r >= 0);

		float z_range = n - f;

		ret(0, 0) = 1 / (aspect * tanfoo);
		ret(1, 1) = 1 / tanfoo;
		ret(2, 2) = (f + n) / z_range;
		ret(3, 2) = -1;
		ret(2, 3) = -2 * f*n / z_range;

		return ret;
	}

	Eigen::Matrix4f orthoMatrix(float width, float height,
		float nearclip, float farclip) {
		Eigen::Matrix4f ret;

		ret.setIdentity();
		ret(0, 0) = 2 / width;
		ret(1, 1) = -2 / height;
        ret(0, 3) = -1;
        ret(1, 3) = +1; 
		ret(2, 2) = -2 / (farclip - nearclip);
		ret(2, 3) = -(farclip + nearclip) / (farclip - nearclip);
		return ret;
	}


	Eigen::Matrix3f crossProduct(const Eigen::Vector3f& c) {
		Eigen::Matrix3f Mat;
		Mat << 0, -c(2), c(1), 
			   c(2), 0, -c(0), 
			   -c(1), c(0), 0;
		return Mat;
	}
	Eigen::Matrix3f rodrigues(const Eigen::Vector3f axis, float angle) {
		Eigen::Matrix3f cp = crossProduct(axis);
		Eigen::Matrix3f cpsq = cp * cp;
	    float norm = axis.norm();
		float sqnorm = axis.squaredNorm();


		Eigen::Matrix3f ret = Eigen::Matrix3f::Identity();
		if (norm < 1e-6) {
			return  ret;
		}
		ret += cp / norm * sin(norm * angle) + cpsq / sqnorm * (1 - cos(norm * angle));
		return ret;
	}

	void angleAxisToEuler(float x, float y, float z, float angle, float* yaw, float* pitch, float* roll) {
		float s = sinf(angle);
		float c = cosf(angle);
		float t = 1 - c;


		//  if axis is not already normalised then uncomment this
		// double magnitude = Math.sqrt(x*x + y*y + z*z);
		// if (magnitude==0) throw error;
		// x /= magnitude;
		// y /= magnitude;
		// z /= magnitude;
		float heading, attitude, bank;
		if ((x*y*t + z*s) > 0.998f) { // north pole singularity detected
			heading = 2 * atan2f(x*sinf(angle / 2), cosf(angle / 2));
			attitude = (float)M_PI / 2.0f;
			bank = 0;
			return;
		}
		if ((x*y*t + z*s) < -0.998) { // south pole singularity detected
			heading = -2 * atan2f(x*sinf(angle / 2), cosf(angle / 2));
			attitude = -(float)M_PI / 2.0f;
			bank = 0;
			return;
		}
		*yaw = atan2f(y * s - x * z * t, 1 - (y*y + z*z) * t);
		*pitch = asinf(x * y * t + z * s);
		*roll = atan2f(x * s - y * z * t, 1 - (x*x + z*z) * t);
	}

    mat4f lookAt(const vec3f& eye, const vec3f& center, const vec3f& up) {
        // z is negative forward
        vec3f z = (eye - center).normalized();
        float along = z.dot(up);
        vec3f y = (up - z*along).normalized(); // ensure up is orthogonal to viewing dir
        vec3f x = y.cross(z);

        // the x, y, and z vectors define the orthonormal coordinate system
        // the affine part defines the overall translation
        mat4f view;

        view.row(0) = vec4f(x(0), x(1), x(2), -x.dot(eye));
        view.row(1) = vec4f(y(0), y(1), y(2), -y.dot(eye));
        view.row(2) = vec4f(z(0), z(1), z(2), -z.dot(eye));
        view.row(3) = vec4f(0, 0, 0, 1);

        return view;
    }

}