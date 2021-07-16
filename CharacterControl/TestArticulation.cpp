#include "TestArticulation.h"
#include "util.h"

TestArticulation::TestArticulation() : 
	m_physx_manager(nullptr), m_physics(nullptr), m_articulation(nullptr), m_material(nullptr),
	m_target_angle(0),
	m_stiffness(100.f),
	m_damping(0.1f),
	m_forceLimit(PX_MAX_F32)
				
{
}

TestArticulation::~TestArticulation()
{
}

void TestArticulation::Initialize()
{
	m_physx_manager = GLFWApp::getInstance()->getPhysxManager();
	m_physics = m_physx_manager->getPhysics();
	m_material = m_physx_manager->getMaterial();
}

void TestArticulation::Update()
{
	if (!m_control_joint)
		return;
	// Target (P control)
	m_target_angle += Deg2Rad(0.1f);
	m_control_joint->setDrive(PxArticulationAxis::eSWING2, m_stiffness, m_damping, m_forceLimit, PxArticulationDriveType::eFORCE);
	m_control_joint->setDriveTarget(PxArticulationAxis::eSWING2, m_target_angle);
	// Velocity (D control)
	//m_control_joint->setDriveVelocity();
	
	m_cache = m_articulation->createCache();
	m_articulation->copyInternalStateToCache(*m_cache, PxArticulationCache::eALL);
}

void TestArticulation::CreatePhysXJointsArticulation()
{
	float height = 4.0f;

	PxQuat rot(30.0f * PxPi / 180.0f, PxVec3(0, 0, 1));
	PxQuat rot_inv(-30.0f * PxPi / 180.0f, PxVec3(0, 0, 1));
	PxQuat rot2(60.0f * PxPi / 180.0f, PxVec3(0, 0, 1));

	PxTransform relativePose(Glm2PxQuat(RotateBetweenVectors(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0))));

	//m_articulation = m_physx_manager->getPhysics()->createArticulation();
	m_articulation = m_physx_manager->getPhysics()->createArticulationReducedCoordinate();

	PxTransform base_anchor_pose(PxVec3(10, height, 0));

	PxArticulationLink* base = m_articulation->createLink(nullptr, PxTransform(PxVec3(10.f, height - 0.5f, 0.f)));
	PxRigidActorExt::createExclusiveShape(*base, PxBoxGeometry(0.5f, 0.5f, 1.0f), *m_material);
	PxRigidBodyExt::updateMassAndInertia(*base, 3.f);

	PxFixedJoint* base_anchor =
		PxFixedJointCreate(*m_physics, nullptr, base_anchor_pose, base, PxTransform(PxVec3(0, 0.0f, 0)));
	
	m_articulation->setSolverIterationCounts(32);

	PxArticulationLink* link1 = m_articulation->createLink(base, PxTransform(PxVec3(0, -1.1f, 0.f)+ base->getGlobalPose().p));
	PxShape* shape = PxRigidActorExt::createExclusiveShape(*link1, PxBoxGeometry(0.5f, 0.5f, 1.0f), *m_material);
	PxRigidBodyExt::updateMassAndInertia(*link1, 3.f);
	//shape->setLocalPose(relativePose);


	PxVec3 parent_anchor_pose(PxVec3(0.0f, -0.55f, -0.0f));
	
	// set first joint
	PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>(link1->getInboundJoint());
	joint->setParentPose(PxTransform(parent_anchor_pose));
	joint->setChildPose(PxTransform(PxVec3(0.f, 0.55f, 0.f)));
	joint->setJointType(PxArticulationJointType::eFIX);
		
	// link 2
	PxArticulationLink* link2 = m_articulation->createLink(link1, PxTransform(PxVec3(0.f, -1.65f, 0.f)+ link1->getGlobalPose().p, Glm2PxQuat(RotateBetweenVectors(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0)))));
	shape = PxRigidActorExt::createExclusiveShape(*link2, PxBoxGeometry(0.5f, 0.5f, 1.0f), *m_material);
	PxRigidBodyExt::updateMassAndInertia(*link2, 3.f);
	//shape->setLocalPose(relativePose);

	// joint 2
	joint = static_cast<PxArticulationJointReducedCoordinate*>(link2->getInboundJoint());
	joint->setParentPose(PxTransform(PxVec3(0.f,-0.55f,0.f)+ parent_anchor_pose, Glm2PxQuat(RotateBetweenVectors(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0)))));
	joint->setChildPose(PxTransform(PxVec3(0.f, 0.55f, 0.f)));
	joint->setJointType(PxArticulationJointType::eSPHERICAL);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
	joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
	joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eLIMITED);
	joint->setLimit(PxArticulationAxis::eSWING2, Deg2Rad(-60.f), Deg2Rad(60.0f));
	
	joint->setDrive(PxArticulationAxis::eSWING2, m_stiffness, m_damping, m_forceLimit, PxArticulationDriveType::eFORCE);
	m_target_angle = Deg2Rad(0.0f);
	joint->setDriveTarget(PxArticulationAxis::eSWING2, m_target_angle);
	
	m_control_joint = joint;

	m_physx_manager->getScene()->addArticulation(*m_articulation);
	
	/*
	PxRigidDynamic* capsule_actor = m_physics->createRigidDynamic(PxTransform(PxVec3(0, 5, 0)));
	
	PxShape* aShape = PxRigidActorExt::createExclusiveShape(
		*capsule_actor, 
		PxCapsuleGeometry(0.5f, 1.0f), *m_physx_manager->getMaterial());
	//aShape->setLocalPose(relativePose);
	m_physx_manager->getScene()->addActor(*capsule_actor);
	*/
	m_links.push_back(base);
	m_links.push_back(link1);
	m_links.push_back(link2);

	m_cache = m_articulation->createCache();
	m_articulation->copyInternalStateToCache(*m_cache, PxArticulationCache::eALL);
}

void TestArticulation::CreateArticulationSimLink()
{
	float height = 4.0f;

	PxQuat rot(30.0f * PxPi / 180.0f, PxVec3(0, 0, 1));
	PxQuat rot_inv(-30.0f * PxPi / 180.0f, PxVec3(0, 0, 1));
	PxQuat rot2(60.0f * PxPi / 180.0f, PxVec3(0, 0, 1));
	PxVec3 parent_anchor_pose(PxVec3(0.0f, -0.55f, -0.0f));

	m_articulation = m_physx_manager->getPhysics()->createArticulationReducedCoordinate();
	m_articulation->setSolverIterationCounts(32);

	PxTransform base_anchor_pose(PxVec3(10, height, 0));
	
	auto base = std::make_shared<SimLink>(nullptr, PxTransform(PxVec3(10.f, height - 0.5f, 0.f)));
	base->Initialize(m_articulation);
	base->CreateShape(PxBoxGeometry(0.5f, 0.5f, 1.0f), m_material);
	
	PxFixedJoint* base_anchor =
		PxFixedJointCreate(*m_physics, nullptr, base_anchor_pose, base->getLink(), PxTransform(PxVec3(0, 0.0f, 0)));

	auto link1 = std::make_shared<SimLink>(base, PxTransform(PxVec3(0, -1.1f, 0.f) + base->getLink()->getGlobalPose().p));
	link1->Initialize(m_articulation);
	link1->CreateShape(PxBoxGeometry(0.5f, 0.5f, 1.0f), m_material);
	link1->UpdateMassAndInertia(3.f);
	link1->SetUpInboundJointParentPose(PxTransform(parent_anchor_pose));
	link1->SetUpInboundJointChildPose(PxTransform(PxVec3(0.f, 0.55f, 0.f)));
	link1->SetUpInboundJointType(PxArticulationJointType::eFIX);
	
	auto link2 = std::make_shared<SimLink>(link1, PxTransform(PxVec3(0, -1.65f, 0.f) + link1->getLink()->getGlobalPose().p, Glm2PxQuat(RotateBetweenVectors(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0)))));
	link2->Initialize(m_articulation);
	link2->CreateShape(PxBoxGeometry(0.5f, 0.5f, 1.0f), m_material);
	link2->UpdateMassAndInertia(3.f);
	link2->SetUpInboundJointParentPose(PxTransform(PxVec3(0.f, -0.55f, 0.f) + parent_anchor_pose, Glm2PxQuat(RotateBetweenVectors(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0)))));
	link2->SetUpInboundJointChildPose(PxTransform(PxVec3(0.f, 0.55f, 0.f)));
	link2->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
	link2->SetUpInboundJointMotion(PxArticulationAxis::eTWIST,	PxArticulationMotion::eFREE);
	link2->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
	link2->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eLIMITED);
	link2->SetUpInboundJointLimit(PxArticulationAxis::eSWING2, Deg2Rad(-60.f), Deg2Rad(60.0f));

	link2->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, m_stiffness, m_damping, m_forceLimit, PxArticulationDriveType::eFORCE);
	m_target_angle = Deg2Rad(0.0f);
	link2->SetUpInboundJointDriveTarget(PxArticulationAxis::eSWING2, m_target_angle);

	m_control_joint = link2->getInboundJoint();
	m_physx_manager->getScene()->addArticulation(*m_articulation);

	m_links.push_back(base->getLink());
	m_links.push_back(link1->getLink());
	m_links.push_back(link2->getLink());

	/*
	PxArticulationLink* link2 = m_articulation->createLink(link1, PxTransform(PxVec3(0.f, -1.65f, 0.f)+ link1->getGlobalPose().p, Glm2PxQuat(RotateBetweenVectors(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0)))));
	shape = PxRigidActorExt::createExclusiveShape(*link2, PxBoxGeometry(0.5f, 0.5f, 1.0f), *m_material);
	PxRigidBodyExt::updateMassAndInertia(*link2, 3.f);
	//shape->setLocalPose(relativePose);

	// joint 2
	joint = static_cast<PxArticulationJointReducedCoordinate*>(link2->getInboundJoint());
	joint->setParentPose(PxTransform(PxVec3(0.f,-0.55f,0.f)+ parent_anchor_pose, Glm2PxQuat(RotateBetweenVectors(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0)))));
	joint->setChildPose(PxTransform(PxVec3(0.f, 0.55f, 0.f)));
	joint->setJointType(PxArticulationJointType::eSPHERICAL);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
	joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
	joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eLIMITED);
	joint->setLimit(PxArticulationAxis::eSWING2, Deg2Rad(-60.f), Deg2Rad(60.0f));
	*/
/*
	PxArticulationLink* link1 = m_articulation->createLink(base, PxTransform(PxVec3(0, -1.1f, 0.f) + base->getGlobalPose().p));
	PxShape* shape = PxRigidActorExt::createExclusiveShape(*link1, PxBoxGeometry(0.5f, 0.5f, 1.0f), *m_material);
	PxRigidBodyExt::updateMassAndInertia(*link1, 3.f);
	*/
}

void TestArticulation::CreatePhysXJointsCustomized()
{
	float height = 10.0f;

	PxQuat rot(30.0f * PxPi / 180.0f, PxVec3(0, 0, 1));
	PxQuat rot2(60.0f * PxPi / 180.0f, PxVec3(0, 0, 1));

	PxTransform actor0_anchor_world = PxTransform(PxVec3(0.f, height, 0.f));
	

	float x_offset = 0.5f * PxSin(PxPi / 6.f);
	float y_offset = -0.5f * PxCos(PxPi / 6.f);

	float x_offset2 = 0.5f * PxSin(PxPi / 3.f);
	float y_offset2 = -0.5f * PxCos(PxPi / 3.f);

	//PxTransform base_trans = PxTransform(PxVec3(x, y, 0), rot2);
	PxTransform base_trans = PxTransform(PxVec3(0, height - 0.5f, 0));
	PxTransform local_trans = PxTransform(PxVec3(x_offset, -0.75f + y_offset, 0.f), rot);
	PxTransform local_trans2 = PxTransform(PxVec3(x_offset2, -0.75f + y_offset2, 0.f), rot2);

	auto base = m_physics->createRigidDynamic(base_trans);
	PxRigidActorExt::createExclusiveShape(*base, PxBoxGeometry(0.5f, 0.5f, 1.0f), *m_material);
	PxRigidBodyExt::updateMassAndInertia(*base, 3.f);
	m_physx_manager->getScene()->addActor(*base);
	/*
	PxSphericalJoint* base_anchor_joint =
		PxSphericalJointCreate(*m_physics, nullptr, base_anchor_pose, base, PxTransform(PxVec3(0, 0.5f, 0)));
	//base_anchor_joint->setLocalPose(PxJointActorIndex::eACTOR1, PxTransform(PxVec3(0, 0.5f, 0)));
	//base_anchor_joint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
	*/
	PxFixedJoint* base_anchor_joint = PxFixedJointCreate(
		*m_physics, 
		nullptr, 
		actor0_anchor_world,
		base, 
		PxTransform(PxVec3(0, 0.5f, 0))
	);
	
	auto link_1 = m_physics->createRigidDynamic(base->getGlobalPose().transform(local_trans));
	PxRigidActorExt::createExclusiveShape(*link_1, PxBoxGeometry(0.5f, 0.5f, 1.0f), *m_material);
	PxRigidBodyExt::updateMassAndInertia(*link_1, 3.f);
	m_physx_manager->getScene()->addActor(*link_1);
	
	auto joint = PxFixedJointCreate(
		*m_physics,
		base,
		PxTransform(PxVec3(0,-0.75f,0), rot),
		link_1,
		PxTransform(PxVec3(0, 0.5f, 0))
	);
	
	auto link_2 = m_physics->createRigidDynamic(link_1->getGlobalPose().transform(local_trans2));
	PxRigidActorExt::createExclusiveShape(*link_2, PxBoxGeometry(0.5f, 0.5f, 1.0f), *m_material);
	PxRigidBodyExt::updateMassAndInertia(*link_2, 3.f);
	m_physx_manager->getScene()->addActor(*link_2);

	joint = PxFixedJointCreate(
		*m_physics,
		link_1,
		PxTransform(PxVec3(0, -0.75f, 0), rot2),
		link_2,
		PxTransform(PxVec3(0, 0.5f, 0))
	);

	/*
	PxD6Joint* joint = PxD6JointCreate(
		*m_physics,
		base,
		base->getGlobalPose().transform(PxTransform(parent_anchor_pose, rot)),
		link_1,
		link_1->getGlobalPose().transform(PxTransform(PxVec3(0.f, 0.55f, 0.f)))
	);
	joint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
	joint->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
	joint->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
	*/
}

void TestArticulation::ComputeDofStarts()
{
	m_dof_starts[0] = 0; // root_link has no inbound joint
	m_dof_starts[1] = 0;
	m_dof_starts[2] = 0;

	for (PxU32 i = 1; i < 3; ++i)
	{
		PxU32 llIndex = m_links[i]->getLinkIndex();
		PxU32 dofs = m_links[i]->getInboundJointDof();

		m_dof_starts[llIndex] = dofs;
	}

	PxU32 count = 0;
	for (PxU32 i = 1; i < 3; ++i)
	{
		PxU32 dofs = m_dof_starts[i];
		count += dofs;
		m_dof_starts[i] = count;
	}
	return;
}

void TestArticulation::setDriveStiffness(float stiffness)
{
	m_stiffness = stiffness;
}

void TestArticulation::setDriveDamping(float damping)
{
	m_damping = damping;
}

void TestArticulation::setForceLimit(float forceLimit)
{
	m_forceLimit = forceLimit;
}

std::vector<float> TestArticulation::getCachedPosition(int link_index)
{
	assert(m_cache);

	std::vector<float> ret_vec;

	auto dofs = m_links[link_index]->getInboundJointDof();

	for (size_t i = 0; i < dofs; ++i)
	{
		PxU32 idx = m_dof_starts[link_index] + static_cast<PxU32>(i);
		ret_vec.push_back(m_cache->jointPosition[idx]);
	}

	return ret_vec;
}

std::vector<float> TestArticulation::getCachedForce(int link_index)
{
	assert(m_cache);

	std::vector<float> ret_vec;

	auto dofs = m_links[link_index]->getInboundJointDof();

	for (size_t i = 0; i < dofs; ++i)
	{
		PxU32 idx = m_dof_starts[link_index] + static_cast<PxU32>(i);
		ret_vec.push_back(m_cache->jointForce[idx]);
	}

	return ret_vec;
}

std::vector<float> TestArticulation::getCachedVelocity(int link_index)
{
	assert(m_cache);

	std::vector<float> ret_vec;

	auto dofs = m_links[link_index]->getInboundJointDof();

	for (size_t i = 0; i < dofs; ++i)
	{
		PxU32 idx = m_dof_starts[link_index] + static_cast<PxU32>(i);
		ret_vec.push_back(m_cache->jointVelocity[idx]);
	}

	return ret_vec;
}
