#ifndef _TEST_ARTICULATION_H_
#define _TEST_ARTICULATION_H_

#include "GLFWApp.h"
#include "PhysXManager.h"
#include "Joint.h"
#include "SimLink.h"
#include <vector>

//using namespace physx;

class TestArticulation
{
public:
	TestArticulation();
	~TestArticulation();

	void Initialize();
	void Update();
	void CreatePhysXJointsArticulation();
	void CreateArticulationSimLink();
	void CreatePhysXJointsCustomized();
	void ComputeDofStarts();

	void setDriveStiffness(float stiffness);
	void setDriveDamping(float damping);
	void setForceLimit(float forceLimit);

	std::vector<float> getCachedPosition(int link_index);
	std::vector<float> getCachedForce(int link_index);
	std::vector<float> getCachedVelocity(int link_index);
	inline float getTargetAngle() { return m_target_angle; };
	inline float getDriveStiffness() { return m_stiffness; };
	inline float getDriveDamping() { return m_damping; };
	inline float getForceLimit() { return m_forceLimit; };
	inline physx::PxU32 getInboundJointDof(int link_index) { return m_links[link_index]->getInboundJointDof(); };
	

private:
	std::shared_ptr<PhysXManager>			m_physx_manager;
	physx::PxPhysics*								m_physics;
	physx::PxArticulationReducedCoordinate*		m_articulation;
	physx::PxMaterial*								m_material;
	physx::PxArticulationCache*					m_cache;
	std::vector<physx::PxArticulationLink*>		m_links;
	std::vector<std::shared_ptr<SimLink>>					m_sim_links;

	PxArticulationJointReducedCoordinate*	m_control_joint;
	
	float m_target_angle, prev_target_angle;
	float m_stiffness;
	float m_damping;
	float m_forceLimit;
	physx::PxU32 m_dof_starts[3]; // dof_starts[totalLinkCount];

};

#endif