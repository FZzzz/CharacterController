#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <glm\glm.hpp>
#include <memory>
#include "GameObject.h"


struct CameraDesc
{
	float		fov;
	float		screen_width;
	float		screen_height;
	float		near_plane;
	float		far_plane;
	glm::vec3	target_position;
	glm::mat4	projection;
	glm::mat4	lookAt;
	glm::vec3	camera_front;
	glm::vec3	camera_up;
	glm::vec3	position;
	float		move_speed;
	GLuint		ubo; //TODO: Clean this!
	
};

class Camera
{
public:
	Camera(CameraDesc desc);
	Camera(glm::mat4 projection, glm::mat4 lookAt, glm::vec3 position, GLuint ubo);
	~Camera();

	void Zoom(float change);
	void Move(glm::vec3 direction);
	void Rotate(glm::vec3 euler);
	void Update();
	
	// public atrribute
	glm::mat4	m_cameraMat;
	glm::mat4	m_projection;
	glm::mat4	m_lookAt;
	glm::vec3	m_front;
	glm::vec3	m_up;
	glm::vec3	m_position;
	glm::vec2	m_ctrl_prev_pos;
	glm::vec2	m_ctrl_cur_pos;
	float		m_move_speed;
	bool		m_ctrl_hold;
	bool		m_lock_on_mode;
	GLuint		m_camera_ubo;
	//std::weak_ptr<GameObject> lookAtObj;
	
private:
	float		m_near_plane;
	float		m_far_plane;
	float		m_rotate_radius;
	float		m_camera_height;
	float		m_theta = 0.0f;
	float		m_fov;
	float		m_screen_width;
	float		m_screen_height;
	glm::vec3	m_target_position;
};

#endif
