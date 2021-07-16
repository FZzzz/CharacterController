#ifndef _SIM_CHARACTER_H_
#define _SIM_CHARACTER_H_

#include <vector>
#include <memory>
#include <map>
#include <PxPhysicsAPI.h>
#include "GameObject.h"
#include "AnimCharacter.h"
#include "ArticulatedObject.h"
#include "SimLink.h"

class SimCharacter
{
public:
	SimCharacter();
	~SimCharacter();
	
	void Initialize(std::shared_ptr<AnimCharacter> ref_character);

	/*
	*  If desired initial pose is not T-pose(the first frame in .bvh)
	*  Use InitializePose() to set to the desired initial pose
	*  TODO: May be able to pop out to GUI
	*/
	void InitializePose(uint32_t pose_start);


	void Update(float dt);
	void Release();

	// getter
	Transform& getTransform() { return m_transform; };

private:
	
	void SetUpCharacter();
	void CreatePhysxJoint();
	void CreateMeshes();
	void CreateMeshesEx();
	void CreateMeshes_Dirty();
	void CreatePhysxJointEx();
	void CreatePhysxJoint_dirty();
	
	
	// Subrountine for link creation
	PxArticulationLink* CreateArticulationLink_dirty(PxArticulationLink* parent, PxTransform link_pose, const PxGeometry &geometry);
	PxArticulationJointReducedCoordinate* CreateArticulationJoint_dirty();

	std::shared_ptr<AnimCharacter>		m_ref_character;
	uint32_t							m_ref_start;
	
	// Cache data from animated character
	// But this won't be used after set up PhysX joints
	// Shall we remove this? (dirty work)
	std::shared_ptr<Joint>					m_root;
	std::vector<std::shared_ptr<Bone>>		m_bones;
	std::vector<std::shared_ptr<Joint>>		m_joints;
	std::vector<std::shared_ptr<SimLink>>	m_sim_links;

#ifdef USE_REDUCED_COORDINATE_ARTICULATION
	physx::PxArticulationReducedCoordinate* m_articulation;
#else
	physx::PxArticulation* m_articulation;
#endif

	ArticulatedObject_sp							m_articulated_obj;
	// Don't use std::vector for PxJoint --> clear will call dtor
	// But physx disable dtor, they suggest to use release function
	std::vector<physx::PxArticulationLink*>			m_px_articulation_links;
	std::vector<physx::PxArticulationJointBase*>	m_px_articulation_joints;
	
	std::vector<std::shared_ptr<GameObject>>		m_renderable_objs;

#ifdef _DEBUG
	std::vector<std::shared_ptr<GameObject>>		m_debug_objs;
#endif

	glm::vec3										m_visual_offset;

	//std::vector<physx::PxCapsuleGeometry> m_colliders;
	physx::PxMaterial*								m_material;
	Transform										m_transform;

	glm::mat4										m_ref2sim;


	int												m_start_count;
};

#endif
