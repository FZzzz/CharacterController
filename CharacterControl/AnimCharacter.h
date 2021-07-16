#ifndef _ANIM_CHARACTER_H_
#define _ANIM_CHARACTER_H_

#include "GameObject.h"
#include "Mesh.h"
//#include "MotionParser/Character.h"
#include "Transform.h"
#include "Animation.h"
#include "Joint.h"
#include <vector>
#include <memory>

// forward declaration
class Bone;
class AnimCharacter;

using NameJointMap = std::unordered_map<std::string, std::shared_ptr<Joint>>;

/*
* Bone is the implied connection between joints
*/
class Bone : public GameObject
{
public:

	Bone(std::shared_ptr<Joint> start_joint, std::shared_ptr<Joint> end_joint);
	~Bone();

	void Initialize();
	void Initialize(Transform& trans);
	void Update();
	
	std::shared_ptr<Joint> getStartJoint() { return m_start_joint; };
	std::shared_ptr<Joint> getEndJoint() { return m_end_joint; };

	// bone config
	float length;

private:
	std::shared_ptr<Joint> m_start_joint;
	std::shared_ptr<Joint> m_end_joint;
	   	
};

class AnimCharacter
{
public:
	AnimCharacter();
	~AnimCharacter();

	void Initialize(glm::vec3 position);
	void Update();
	
	void setName(std::string name);
	void setRoot(std::shared_ptr<Joint> root);
	void setJoints(const std::vector<std::shared_ptr<Joint>>& joints);
	void setBones(const std::vector<std::shared_ptr<Bone>>& bones);
	void setAnimation(std::shared_ptr<AnimationState> animation);
	
	inline std::string									getName()			{ return m_name; };
	inline const std::vector<std::shared_ptr<Joint>>&	getJoints()			{ return m_joints; };
	inline const std::vector<std::shared_ptr<Bone>>&	getBones()			{ return m_bones; };
	inline std::shared_ptr<AnimationState>				getAnimation()		{ return m_animation; };
	inline std::shared_ptr<Joint>						getRoot()			{ return m_root; };
	inline const NameJointMap&							getNameJointMap()	{ return m_name_joint_map; };

	// Object status
	bool m_initialized;

private:


#ifdef _DEBUG
	std::vector<std::shared_ptr<GameObject>> m_debug_objs;
#endif

	// Basic class member
	std::string							m_name;

	// root transform
	Transform							m_transform;

	// Character skeletal data
	std::vector<std::shared_ptr<Bone>>	m_bones;
	std::shared_ptr<Joint>				m_root;
	std::vector<std::shared_ptr<Joint>> m_joints;
	NameJointMap						m_name_joint_map;
	std::shared_ptr<AnimationState>		m_animation;


	// Inner setup functions
	void SetUpJointMeshes();
	void SetUpBoneMeshes();

	// Sub-function to update joint information
	void UpdateJointTransform();


};

#endif