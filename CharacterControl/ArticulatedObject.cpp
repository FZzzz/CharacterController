#include "ArticulatedObject.h"
#include <PxArticulation.h>
#include "GLFWApp.h"


ArticulationLink::ArticulationLink() 
	: m_rigid_body(nullptr), m_shape(nullptr), m_internal_joint(nullptr), m_parent(nullptr)
{
}

ArticulationLink::~ArticulationLink()
{
	if(m_rigid_body && m_rigid_body->isReleasable())
		m_rigid_body->release();
	if(m_shape && m_shape->isReleasable())
		m_shape->release();
	if(m_internal_joint && m_internal_joint->isReleasable())
		m_internal_joint->release();

	m_parent = nullptr;

}

ArticulatedObject::ArticulatedObject() : m_root(nullptr)
{
	if(m_root)
		delete m_root;
	for (size_t i = 0; i < m_links.size(); ++i)
	{
		delete m_links[i];
	}
	m_links.clear();
}

ArticulatedObject::~ArticulatedObject()
{
}

ArticulationLink* ArticulatedObject::CreateLink(ArticulationLink* parent, PxTransform transform, std::string name="")
{
	// If parent is null then it is root
	PxPhysics* physics = GLFWApp::getInstance()->getPhysxManager()->getPhysics();
	ArticulationLink* link = new ArticulationLink();
	link->m_name = name;
	link->m_rigid_body = physics->createRigidDynamic(transform);
	link->m_parent = parent;
	
	m_links.push_back(link);

	return link;
}

/* Not allow to explicitly add rigidbodies
void ArticulatedObject::AddRigidbody(physx::PxRigidDynamic* actor)
{
	m_rigidbodies.push_back(actor);
}
*/

// This will be decrepcated soon. (Joints shouldn't be explicitly added to this class)
// TODO: Find a better way to manage joints
/*
void ArticulatedObject::AddD6Joints(physx::PxD6Joint* joint)
{
	m_joints.push_back(joint);
}
*/

void ArticulatedObject::setLinkRoot(ArticulationLink * link)
{
	m_root = link;
}

void ArticulatedObject::AddActorsToScene()
{
	auto physx_manager = GLFWApp::getInstance()->getPhysxManager();
	for (size_t i = 0; i < m_links.size(); ++i)
	{
		physx_manager->getScene()->addActor(*(m_links[i]->m_rigid_body));
	}
}
