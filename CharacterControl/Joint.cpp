#include "Joint.h"
#include "GLFWApp.h"

// Joint
Joint::Joint(std::string name) : m_offset_from_parent(glm::vec3(0,0,0)), m_num_channels(0), m_parent(nullptr)
{
	GameObject::setName(name);
	for (int i = 0; i < 6; ++i)
		m_joint_limit[i] = { 0,0 };
}

Joint::Joint(const Joint &other)
{
	m_transform = other.m_transform;
	m_transform.setParent(nullptr);
	m_mesh = nullptr;

	GameObject::setName("Clone" + other.m_name);
	//m_name = "Clone " + other.m_name;
	m_parent = nullptr;
	
	m_offset_from_parent = other.m_offset_from_parent;
	for (int i = 0; i < 6; ++i)
		m_joint_limit[i] = other.m_joint_limit[i];
	m_channel_order = other.m_channel_order;
	m_num_channels = other.m_num_channels;
	
	m_childs.clear();
}

Joint::~Joint()
{
}

void Joint::Initialize()
{
	GameObject::Initialize();
}

void Joint::Initialize(Transform& trans)
{
	auto resource_manager = GLFWApp::getInstance()->getResourceManager();
	const auto mesh_list = resource_manager->getMeshes();
	// TODO: remember to replace this with findMesh(name) / setMesh()
	auto mesh = mesh_list[2];

	GameObject::Initialize(trans.m_translation);
}

void Joint::setParent(std::shared_ptr<Joint> parent)
{
	m_parent = parent;
	m_transform.setParent(&(parent->m_transform));
}

void Joint::AddChild(std::shared_ptr<Joint> child)
{
	m_childs.push_back(child);
}
