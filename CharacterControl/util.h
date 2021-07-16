#ifndef _UTIL_H_
#define _UTIL_H_

#include <glm/common.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <PxPhysicsAPI.h>

// forward declaration
physx::PxVec3 Glm2PxVec3(const glm::vec3& input);
physx::PxQuat Glm2PxQuat(const glm::quat& rot_in);
glm::quat Px2GlmQuat(const physx::PxQuat& q);
glm::vec3 Px2GlmVec3(const physx::PxVec3& vec);
glm::vec3 GetTranslationFromModel(glm::mat4 input);
glm::quat GetQuatFromModel(glm::mat4 input);
glm::vec3 GetScalarFromModel(glm::mat4 input);
glm::quat RotateBetweenVectors(glm::vec3 u, glm::vec3 v);
float Deg2Rad(float degree);
float Rad2Deg(float radian);

inline physx::PxVec3 Glm2PxVec3(const glm::vec3& input)
{
	return physx::PxVec3(input.x, input.y, input.z);
}

inline physx::PxQuat Glm2PxQuat(const glm::quat& rot_in)
{
	auto rot = rot_in;
	glm::quat rot_n = glm::normalize(rot);
	return physx::PxQuat(rot_n.x, rot_n.y, rot_n.z, rot_n.w);
}

inline glm::quat Px2GlmQuat(const physx::PxQuat& q)
{
	return glm::quat(q.w, q.x, q.y, q.z);
}

inline glm::vec3 Px2GlmVec3(const physx::PxVec3& vec)
{
	return glm::vec3(vec.x, vec.y, vec.z);
}


inline glm::vec3 GetTranslationFromModel(glm::mat4 input)
{
	float* arr = glm::value_ptr(input);
	return glm::vec3(arr[12], arr[13], arr[14]);
}

inline glm::quat GetQuatFromModel(glm::mat4 input)
{
	float* arr = glm::value_ptr(input);
	float sx, sy, sz;

	sx = glm::length(glm::vec3(arr[0], arr[4], arr[8]));
	sy = glm::length(glm::vec3(arr[1], arr[5], arr[9]));
	sz = glm::length(glm::vec3(arr[2], arr[6], arr[10]));

	float rot_arr[16] = { 0 };
	rot_arr[0] = arr[0] / sx; rot_arr[1] = arr[1] / sy; rot_arr[2] = arr[2] / sz;
	rot_arr[4] = arr[4] / sx; rot_arr[5] = arr[5] / sy; rot_arr[6] = arr[6] / sz;
	rot_arr[8] = arr[8] / sx; rot_arr[9] = arr[9] / sy; rot_arr[10] = arr[10] / sz;
	rot_arr[15] = 1;

	return glm::quat(glm::make_mat4(rot_arr));
}

inline glm::vec3 GetScalarFromModel(glm::mat4 input)
{
	float* arr = glm::value_ptr(input);
	float sx, sy, sz;

	sx = glm::length(glm::vec3(arr[0], arr[4], arr[8]));
	sy = glm::length(glm::vec3(arr[1], arr[5], arr[9]));
	sz = glm::length(glm::vec3(arr[2], arr[6], arr[10]));

	return glm::vec3(sx, sy, sz);
}

inline glm::quat RotateBetweenVectors(glm::vec3 u, glm::vec3 v)
{
	u = glm::normalize(u);
	v = glm::normalize(v);

	float cosine = glm::dot(u, v);
	glm::vec3 rotation_axis;

	if (cosine < -1 + 0.001f)
	{
		rotation_axis = glm::cross(glm::vec3(0, 0, 1), u);
		if (glm::length2(rotation_axis) < 0.01f)
		{
			rotation_axis = glm::cross(glm::vec3(1, 0, 0), u);
		}
		rotation_axis = normalize(rotation_axis);
		return glm::angleAxis(glm::radians(180.0f), rotation_axis);
	}

	rotation_axis = glm::cross(u, v);
	float s = sqrt((1 + cosine) * 2);
	float invs = 1 / s;

	return glm::quat(
		s * 0.5f,
		rotation_axis.x * invs,
		rotation_axis.y * invs,
		rotation_axis.z * invs
	);

}

inline float Deg2Rad(float degree)
{
	return degree * physx::PxPi / 180.f;
}

inline float Rad2Deg(float radian)
{
	return radian * 180.f / physx::PxPi;
}


#endif
