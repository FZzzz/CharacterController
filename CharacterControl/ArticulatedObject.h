#ifndef _ARTICULATED_OBJ_H_
#define _ARTICULATED_OBJ_H_

/*
** A customized articulation class: similar to PxArticulation, but more flexible 
**                                => Don't need that much features
*/

#include <vector>
#include <unordered_map>
#include <memory>
#include <PxPhysicsAPI.h>
#include <PxActor.h>

using namespace physx;

class ArticulationLink;
class ArticulatedObject;
	
using ArticulatedObject_sp = std::shared_ptr<ArticulatedObject>;

class ArticulationLink
{
public:
	ArticulationLink();
	~ArticulationLink();
	PxRigidDynamic* m_rigid_body;
	PxShape* m_shape;
	PxD6Joint* m_internal_joint;
	ArticulationLink* m_parent;
	std::string m_name;
};


class ArticulatedObject
{
public:
	ArticulatedObject();
	~ArticulatedObject();

	ArticulationLink* CreateLink(ArticulationLink* parent, PxTransform transform, std::string name);
	//void AddRigidbody(physx::PxRigidDynamic*);
	//void AddD6Joints(physx::PxD6Joint*);

	void setLinkRoot(ArticulationLink* link);

	void AddActorsToScene();

	inline const std::vector<ArticulationLink*>& getLinks() { return m_links; };
	//inline const std::vector<physx::PxRigidDynamic*>& getRigidbodies() { return m_rigidbodies; };
	//inline const std::vector<physx::PxD6Joint*>& getJoints() { return m_joints; };

private:
	
	ArticulationLink* m_root;

	std::vector<ArticulationLink*> m_links;
	//std::vector<physx::PxRigidDynamic*> m_rigidbodies;
	//std::vector<physx::PxD6Joint*> m_joints;

};

#endif