#ifndef _SIM_LINK_H_
#define _SIM_LINK_H_

#include <memory>
#include <PxPhysicsAPI.h>
#include "PhysXManager.h"
#include "Joint.h"
#include "common.h"

class SimLink;

class SimLink
{
public:
	SimLink(std::shared_ptr<SimLink> parent, PxTransform link_global_pose);
	~SimLink();

	void Initialize(PxArticulationReducedCoordinate* const articulation); // specify which articulation this link belongs to
	void ResetPose();

	void Update(float dt);// , glm::mat4 ref2sim);
	   
	/* Link Configuration */
	void SetUpLinkPose(PxTransform link_global_pose);
	void CreateShape(const PxGeometry& geometry, PxMaterial* material);
	void UpdateMassAndInertia(PxReal density);
	//void SetUpMaterial(PxMaterial& material);

	/* 
	*  Inbound Joint Configuration 
	*  Inbound joint is the joint connecting parent and this link
	*/
	void SetUpInboundJointParentPose(const physx::PxTransform &pose);
	void SetUpInboundJointChildPose(const physx::PxTransform &pose);
	void SetUpInboundJointType(PxArticulationJointType::Enum type);
	void SetUpInboundJointMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion);
	void SetUpInboundJointLimit(PxArticulationAxis::Enum axis, PxReal low_limit, PxReal high_limit);

	/* Joint Drive */
	void SetUpInboundJointDrive(PxArticulationAxis::Enum axis, PxReal stiffness, PxReal damping, PxReal forceLimit, PxArticulationDriveType::Enum driveType);
	void SetUpInboundJointDriveTarget(PxArticulationAxis::Enum drive_axis, float targetAngle);
	void SetUpInboundJointDriveVelocity(PxArticulationAxis::Enum drive_axis, float targetVelocity);
	
	/* Reference Joint*/
	void setReferenceJoint(std::weak_ptr<Joint> ref_joint);

	// setter
	void setName(std::string name);

	// getter
	inline std::string								getName()			{ return m_name; };
	inline PxArticulationLink*						getLink()			{ return m_link; };
	inline PxArticulationLink*						getParentLink()		{ return m_parent; };
	inline PxArticulationJointReducedCoordinate*	getInboundJoint()	{ return m_joint; };
	inline PxReal									getJointStifness()	{ return m_stiffness; };
	inline PxReal									getJointDamping()	{ return m_damping; };
	inline PxArticulationDriveType::Enum			getDriveType()		{ return m_drive_type; };
	inline PxTransform								getGlobalPose()		{ return m_link->getGlobalPose(); };

private:
	PxArticulationLink* m_parent;
	PxArticulationLink* m_link;
	
	/* Joint configuration */
	PxArticulationJointReducedCoordinate*	m_joint;
	PxReal									m_stiffness;	// P gain
	PxReal									m_damping;		// D gain
	PxArticulationDriveType::Enum			m_drive_type;	// Drive type (default force drive)
	
	/* Reference joint */
	std::weak_ptr<Joint>	m_ref_joint;

	/* Shape configuration */
	PxShape*	m_shape;
	PxMaterial* m_material;

	PxTransform m_link_pose;
	//PxTransform m_shape_relative_trans;
	PxTransform m_joint_parent_pose;
	PxQuat		m_joint_parent_rotate;
	PxTransform m_joint_child_pose;

	/* Simulation use */
	glm::mat4	m_ref2sim;
	
	glm::mat4	m_sim_local;
	glm::mat4	m_sim_local_inv;
	
	glm::mat4	m_ref_local;
	glm::mat4	m_ref_local_inv;
	
	//glm::vec3	m_sim_init_euler;
	glm::vec3	m_sim_last_euler;	
	glm::vec3	m_sim_curr_euler;
	glm::vec3	m_ref_last_euler;
	glm::vec3	m_ref_curr_euler;

	// PxMaterial* m_material;
	// Link information
	std::string m_name;


};

#endif