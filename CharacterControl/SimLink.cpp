#include "SimLink.h"
#include "GLFWApp.h"
#include "util.h"
// Don't include when tested
#include "imgui/imgui.h"

SimLink::SimLink(std::shared_ptr<SimLink> sim_parent, PxTransform link_global_pose) : 
	m_parent(nullptr),
	m_link(nullptr),
	m_link_pose(link_global_pose),
	m_joint(nullptr),
	m_stiffness(10.0f),
	m_damping(0.1f),
	m_drive_type(PxArticulationDriveType::eTARGET),
	m_shape(nullptr),
	m_material(nullptr),
	m_name(""),
	m_ref2sim(glm::mat4(1)),
	m_sim_curr_euler(glm::vec3(0, 0, 0)),
	m_sim_last_euler(glm::vec3(0, 0, 0)),
	m_ref_curr_euler(glm::vec3(0, 0, 0)),
	m_ref_last_euler(glm::vec3(0, 0, 0))	
{
	if (sim_parent)
		m_parent = sim_parent->m_link;
}

SimLink::~SimLink()
{
}

void SimLink::Update(float dt)//, glm::mat4 ref2sim)
{
	/*
	if (m_name != "head")
		return;
		*/
	// TODO
	// 1. Transform reference information to simulate coordinate system
	// 2. Decompose euler angles
	// 3. setDriveTarget(), setDriveVelocity()
	auto ref_joint_sp = m_ref_joint.lock();

	if (ref_joint_sp == nullptr || m_parent == nullptr || ref_joint_sp->getParent() == nullptr)
		return;
	
	// Get rotation matrix from model mat
	glm::mat4 sim_parent_inv = glm::inverse(glm::toMat4(Px2GlmQuat(m_parent->getGlobalPose().q)));
	glm::mat4 ref_parent_inv = glm::inverse(ref_joint_sp->getParent()->m_transform.getModelMatWorld());
	glm::mat4 ref_world = glm::toMat4(GetQuatFromModel(ref_joint_sp->m_transform.getModelMatWorld()));
	glm::mat4 sim_world = glm::toMat4(Px2GlmQuat(m_link->getGlobalPose().q));
	
	glm::mat4 ref_drive = ref_world * ref_parent_inv * m_ref_local_inv;
	glm::mat4 sim_drive = glm::inverse(m_ref2sim) * ref_drive * m_ref2sim;
	//glm::mat4 sim_drive = m_ref2sim * ref_world * sim_parent_inv * m_sim_local_inv;

	// Compute euler angles (Order XYZ?)
	glm::vec3 target_angle = glm::eulerAngles(glm::quat_cast(sim_drive));
		
	// Target at local space
	m_ref_curr_euler = target_angle;
	glm::vec3 ref_velocity = (target_angle - m_ref_last_euler) / dt;

	/* Test GUI */
	if(m_name == "head")
	{
		ImGui::Begin("Head drive test");
		//float target_v[3] = { Rad2Deg(target_angle.x), Rad2Deg(target_angle.y), Rad2Deg(target_angle.z) };
		float target_v[3] = { target_angle.x, target_angle.y, target_angle.z };
		ImGui::InputFloat3("Target angle", target_v, 3, ImGuiInputTextFlags_ReadOnly);
		//float cur_v[3] = { Rad2Deg(m_sim_curr_euler.x), Rad2Deg(m_sim_curr_euler.y), Rad2Deg(m_sim_curr_euler.z) };
		float cur_v[3] = { m_sim_curr_euler.x, m_sim_curr_euler.y, m_sim_curr_euler.z };
		ImGui::InputFloat3("Current angle", cur_v, 3, ImGuiInputTextFlags_ReadOnly);

		float q[4] = 
		{	m_link->getGlobalPose().q.x , 
			m_link->getGlobalPose().q.y ,
			m_link->getGlobalPose().q.z,
			m_link->getGlobalPose().q.w
		};
		ImGui::InputFloat4("Parent Q", q, 3, ImGuiInputTextFlags_ReadOnly);
		ImGui::End();
	}
	
	// Set drive target using target_angles
	SetUpInboundJointDriveTarget(PxArticulationAxis::eTWIST,	target_angle.x);
	SetUpInboundJointDriveTarget(PxArticulationAxis::eSWING1,	target_angle.y);
	SetUpInboundJointDriveTarget(PxArticulationAxis::eSWING2,	target_angle.z);
		
	// Set drive target velocity
	//SetUpInboundJointDriveVelocity(PxArticulationAxis::eTWIST,	ref_velocity.x);
	//SetUpInboundJointDriveVelocity(PxArticulationAxis::eSWING1,	ref_velocity.y);
	//SetUpInboundJointDriveVelocity(PxArticulationAxis::eSWING2,	ref_velocity.z);

	// Set drive target velocity using ???
	// Need record last position? velocity = (cur - last)

	// Torque = k_p * (sim_angle - ref_angle) + k_d * (sim_ang_vel - ref_ang_vel)

	// Record last euler angles 
	m_sim_last_euler = m_sim_curr_euler;
	m_ref_last_euler = m_sim_last_euler;
}

void SimLink::Initialize(PxArticulationReducedCoordinate* const articulation)
{
	m_link = articulation->createLink(m_parent, m_link_pose);
#ifdef _DEBUG
	assert(m_link);
#endif
	m_joint = static_cast<PxArticulationJointReducedCoordinate*>(m_link->getInboundJoint());
	
	if (!m_parent)
		return;

	glm::mat4 parent_rot_inv = glm::inverse(glm::toMat4(Px2GlmQuat(m_parent->getGlobalPose().q)));
	m_sim_local = parent_rot_inv * glm::toMat4(Px2GlmQuat(m_link->getGlobalPose().q));
	m_sim_local_inv = glm::inverse(m_sim_local);

	m_sim_curr_euler = glm::vec3(0, 0, 0);
	m_sim_last_euler = glm::vec3(0, 0, 0);

	return;
}

void SimLink::ResetPose()
{
	if (!m_ref_joint.lock())
		return;
	auto ref_joint_sp = m_ref_joint.lock();
	
	/* Root joint should not be modified */
	if (ref_joint_sp->getParent() == nullptr)
		return;
	if (m_parent == nullptr)
		return;
	// Reset Inbound joint parent pose and (linkPose?)
	glm::mat4 ref_parent_rot_inv = glm::inverse(ref_joint_sp->getParent()->m_transform.getModelMatWorld());
	glm::mat4 ref_rot_glm = glm::toMat4(GetQuatFromModel(ref_joint_sp->m_transform.getModelMatWorld()));
	glm::mat4 sim_rot_glm = glm::toMat4(Px2GlmQuat(m_link->getGlobalPose().q));

	//glm::mat4 ref_local_rot = m_ref_local_invRot /* *m_sim_init_pose_inv*/ * ref_parent_rot_inv * ref_rot_glm;

}

void SimLink::SetUpLinkPose(PxTransform link_global_pose)
{
	m_link_pose = link_global_pose;
}

void SimLink::CreateShape(const PxGeometry & geometry, PxMaterial* material=nullptr)
{
	m_material = material;
#ifdef _DEBUG
	assert(m_material);
#endif
	m_shape = PxRigidActorExt::createExclusiveShape(
		*m_link,
		geometry,
		*m_material
	);
}

void SimLink::UpdateMassAndInertia(PxReal density)
{
#ifdef _DEBUG
	assert(m_link);
#endif
	PxRigidBodyExt::updateMassAndInertia(*m_link, density);
}

void SimLink::SetUpInboundJointParentPose(const physx::PxTransform & pose)
{
#ifdef _DEBUG
	assert(m_link && m_joint);
#endif
	m_joint->setParentPose(pose);
}

void SimLink::SetUpInboundJointChildPose(const physx::PxTransform & pose)
{
#ifdef _DEBUG
	assert(m_link && m_joint);
#endif
	m_joint->setChildPose(pose);
}

void SimLink::SetUpInboundJointType(PxArticulationJointType::Enum type)
{
	if (!m_joint)
	{
		std::cout << "joint is null\n";
		return;
	}
	m_joint->setJointType(type);

}

void SimLink::SetUpInboundJointMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion)
{
	if (!m_joint)
	{
		std::cout << "No Joint Available (set joint motion)\n";
		return;
	}
	
	m_joint->setMotion(axis, motion);
}

void SimLink::SetUpInboundJointLimit(PxArticulationAxis::Enum axis, PxReal low_limit, PxReal high_limit)
{
	if (!m_joint)
	{
		std::cout << "No Joint Available (set joint motion)\n";
		return;
	}

	m_joint->setLimit(axis, low_limit, high_limit);
}

void SimLink::SetUpInboundJointDrive(PxArticulationAxis::Enum axis, PxReal stiffness, PxReal damping, PxReal forceLimit, PxArticulationDriveType::Enum driveType=PxArticulationDriveType::eFORCE)
{
	if (!m_joint)
	{
		std::cout << "No Joint Available (set joint motion)\n";
		return;
	}

	m_stiffness = stiffness;
	m_damping = damping;
	m_joint->setDrive(axis, stiffness, damping, forceLimit, driveType);
}

void SimLink::SetUpInboundJointDriveTarget(PxArticulationAxis::Enum drive_axis, float targetAngle)
{
	if (!m_joint)
	{
		std::cout << "No Joint Available (set joint motion)\n";
		return;
	}

	m_joint->setDriveTarget(drive_axis, targetAngle);
}

void SimLink::SetUpInboundJointDriveVelocity(PxArticulationAxis::Enum drive_axis, float targetVelocity)
{
	if (!m_joint)
	{
		std::cout << "No Joint Available (set joint motion)\n";
		return;
	}

	m_joint->setDriveVelocity(drive_axis, targetVelocity);
}

void SimLink::setReferenceJoint(std::weak_ptr<Joint> ref_joint)
{
	m_ref_joint = ref_joint;
	auto sp = m_ref_joint.lock();
	
	if (!sp)
		return;

	auto ref_parent = sp->getParent();
	m_ref_local = 
		(ref_parent) 
		? glm::inverse(ref_parent->m_transform.getModelMatWorld()) * sp->m_transform.getModelMatWorld() 
		: sp->m_transform.getModelMatWorld();

	m_ref_local_inv = glm::inverse(m_ref_local);

	auto sim_world_rot = glm::toMat4(Px2GlmQuat(m_link->getGlobalPose().q));

	/* sim_world * glm::inverse(ref_world) */
	m_ref2sim = sim_world_rot * glm::inverse(sp->m_transform.getRotationMatWorld());


}

void SimLink::setName(std::string name)
{
	m_name = name;
}


