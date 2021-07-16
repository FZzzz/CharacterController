#include "AnimCharacter.h"
#include "GLFWApp.h"
#include "common.h"
#include "util.h"

// Bone
Bone::Bone(std::shared_ptr<Joint> start_joint, std::shared_ptr<Joint> end_joint) : 
	m_start_joint(start_joint), m_end_joint(end_joint), length(0)
{
	setName("Bone");
}

Bone::~Bone()
{
}

void Bone::Initialize()
{
	GameObject::Initialize();
}

void Bone::Initialize(Transform & trans)
{
	m_transform = trans;
	GameObject::Initialize();
}

void Bone::Update()
{
	
	glm::vec3 offset = m_end_joint->m_transform.m_translation;
	glm::vec3 scale = glm::vec3(glm::length(offset), 1, 1);

	// Compute bone rotation
	glm::vec3 norm_dir = glm::normalize(offset);
	glm::vec3 u = glm::vec3(1, 0, 0);
	auto rot_quat = RotateBetweenVectors(u, norm_dir);

	m_transform.m_scale = scale;
	m_transform.setRotation(rot_quat);
	//m_transform.setRotation(m_start_joint->m_transform.getQuaternion());
	//m_transform.m_translation = m_start_joint->m_transform.m_translation;
	

	GameObject::Update();
}

// AnimCharacter
AnimCharacter::AnimCharacter() 
	: m_initialized(false), m_name("AnimCharacter"), m_root(nullptr), m_animation(nullptr)
{
	m_name_joint_map.clear();
}


AnimCharacter::~AnimCharacter()
{
}

void AnimCharacter::Initialize(glm::vec3 position)
{
	m_transform.m_translation = position;
	m_root->m_transform.setParent(&m_transform);
	SetUpJointMeshes();
	SetUpBoneMeshes();
	// Use first frame as rest pose
	UpdateJointTransform();
	// Initialize joint model matrix
	for (auto it = m_joints.begin(); it != m_joints.end(); ++it)
	{
		auto joint = (*it);
		joint->Update();
	}

	m_initialized = true;
}

void AnimCharacter::Update()
{
	//m_transform.Update();
	// synchronize joint and keyframe
	UpdateJointTransform();
	m_transform.Update();
#ifdef _DEBUG
	for (size_t i = 0; i < m_debug_objs.size(); ++i)
	{
		m_debug_objs[i]->m_transform.m_translation = m_bones[i]->m_transform.getTranslationWorld();
		m_debug_objs[i]->m_transform.setRotation(m_bones[i]->m_transform.getRotationWorld());
	}
#endif
	m_animation->Step();
}

void AnimCharacter::setName(std::string name)
{
	m_name = name;
}

void AnimCharacter::setRoot(std::shared_ptr<Joint> root)
{
	m_root = root;
}

void AnimCharacter::setJoints(const std::vector<std::shared_ptr<Joint>>& joints)
{
	m_joints = joints;
	for (size_t i = 0; i < m_joints.size(); ++i)
	{
		m_name_joint_map[m_joints[i]->getName()] = m_joints[i];
	}
}

void AnimCharacter::setBones(const std::vector<std::shared_ptr<Bone>>& bones)
{
	m_bones = bones;
}

void AnimCharacter::setAnimation(std::shared_ptr<AnimationState> animation)
{
	m_animation = animation;
}

void AnimCharacter::SetUpJointMeshes()
{
	auto resource_manager = GLFWApp::getInstance()->getResourceManager();
	//const auto mesh_list = resource_manager->getMeshes();

	// TODO: remember to replace this with findMesh(name)
	auto mesh = resource_manager->FindMeshByName("Coordinate");

	for (auto it = m_joints.begin(); it != m_joints.end(); ++it)
	{
		(*it)->setMesh(mesh);
		resource_manager->AddGameObject(std::static_pointer_cast<GameObject>(*it));
	}

}

void AnimCharacter::SetUpBoneMeshes()
{
	auto resource_manager = GLFWApp::getInstance()->getResourceManager();
	//const auto mesh_list = resource_manager->getMeshes();

	// TODO: remember to replace this with findMesh(name)
	auto mesh = resource_manager->FindMeshByName("AnimCapsule");
	if (mesh == nullptr)
		return;
	for (size_t i = 0; i < m_bones.size(); ++i)
	{
		m_bones[i]->setMesh(mesh);
		resource_manager->AddGameObject(std::static_pointer_cast<GameObject>(m_bones[i]));

#ifdef _DEBUG
		{
			glm::vec3 debug_scalar = glm::vec3(1, 1, 1);
			auto debug_obj = std::make_shared<GameObject>();
			Transform trans(m_bones[i]->m_transform.getTranslationWorld(), m_bones[i]->m_transform.getRotationWorld(), debug_scalar);
			debug_obj->setName(m_bones[i]->getName() + "_BoneCoord");
			debug_obj->setMesh(resource_manager->FindMeshByName("Coordinate"));
			debug_obj->Initialize(trans);
			m_debug_objs.push_back(debug_obj);
			resource_manager->AddGameObject(debug_obj);
		}
#endif

	}


}

void AnimCharacter::UpdateJointTransform()
{
	const auto& current_frame = m_animation->getCurrentFrame();

	for (auto it = m_joints.begin(); it != m_joints.end(); ++it)
	{
		auto joint = (*it);
		const auto channel_value = current_frame.joint_channel_map.at(*it);
		const auto frame_data = current_frame.joint_framedata_map.at(*it);

		if (frame_data.movable)
			joint->m_transform.m_translation = frame_data.translation;

		joint->m_transform.setRotation(frame_data.quaternion);
	}

}
