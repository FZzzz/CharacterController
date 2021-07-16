#include "SimCharacter.h"
#include <unordered_map>
#include <stack>
#include <PxArticulation.h>
#include "GLFWApp.h"
#include "util.h"
#include <glm/gtc/quaternion.hpp>

using namespace physx;

SimCharacter::SimCharacter() 
	: m_material(nullptr), m_articulation(nullptr), m_visual_offset(glm::vec3(-5,0,0)), m_ref2sim(glm::mat4(1)),
	m_start_count(0)
{
}


SimCharacter::~SimCharacter()
{
	for (size_t i = 0; i < m_px_articulation_links.size(); ++i)
	{
		/*
		if(m_articulation_links[i])
			m_articulation_links[i]->release();
		*/
	}
	for (size_t i = 0; i < m_px_articulation_joints.size(); ++i)
	{
		/*
		if(m_articulation_joints[i])
			m_articulation_joints[i]->release();
		*/
	}
	/*
	if(m_articulation)
		m_articulation->release();
	*/
}

void SimCharacter::Initialize(std::shared_ptr<AnimCharacter> ref_character)
{
	m_ref_character = ref_character;
	SetUpCharacter();
	//CreatePhysxJoint();
	CreatePhysxJoint_dirty();
	//CreateMeshes();
	CreateMeshes_Dirty();
}

void SimCharacter::Update(float dt)
{
	// Fetch simulation result
	// Update transform
	// Compute target configuration (ref_character)
	// PD controller 

	// Simulate
	if(m_articulation->isSleeping() || !m_ref_character->getAnimation()->isPausing())
	{
		m_articulation->wakeUp();
	}
	for (size_t i = 0; i < m_sim_links.size(); ++i)
	{
		m_sim_links[i]->Update(dt);//, m_ref2sim);
	}


	// make mesh follows simulate result
	for (size_t i = 0; i < m_renderable_objs.size(); ++i)
	{
		auto link_pose = m_sim_links[i]->getLink()->getGlobalPose();
		auto translation = Px2GlmVec3(link_pose.p) + m_visual_offset;
		auto quaternion = Px2GlmQuat(link_pose.q);
		
		m_renderable_objs[i]->m_transform.m_translation = translation;
		m_renderable_objs[i]->m_transform.setRotation(quaternion);

#ifdef _DEBUG
		m_debug_objs[i]->m_transform.m_translation = translation;
		m_debug_objs[i]->m_transform.setRotation(quaternion);
#endif

	}
	
}

void SimCharacter::Release()
{
	//GameObject::Release();
}

void SimCharacter::SetUpCharacter()
{
	if (m_ref_character == nullptr)
		return;

	auto ref_joints = m_ref_character->getJoints();
	// foreach joint create a copied one for SimCharacter

	std::unordered_map<std::shared_ptr<Joint>, std::shared_ptr<Joint>> joint_map;

	for (auto it = ref_joints.cbegin(); it != ref_joints.cend(); ++it)
	{
		const Joint& ref_joint = **it;
		auto joint = std::make_shared<Joint>(ref_joint);
		m_joints.push_back(joint);
		joint_map.emplace((*it), joint);
	}

	// set parent and childs
	for (auto it = joint_map.cbegin(); it != joint_map.cend(); ++it)
	{
		auto ref_joint = it->first;
		auto joint = it->second;

		if (ref_joint->getParent() != nullptr)
			joint->setParent(joint_map.at(ref_joint->getParent()));
		else
			joint->setParent(nullptr);

		auto ref_childs = ref_joint->getChilds();
		for (auto i = 0; i < ref_childs.size(); ++i)
		{
			joint->AddChild(joint_map.at(ref_childs[i]));
		}

	}

	auto ref_bones = m_ref_character->getBones();

	// set bones
	for (auto it = ref_bones.cbegin(); it != ref_bones.cend(); ++it)
	{	
		auto start_joint = joint_map.at((*it)->getStartJoint());
		auto end_joint = joint_map.at((*it)->getEndJoint());
		
		auto bone = std::make_shared<Bone>(start_joint, end_joint);
		bone->length = (*it)->length;
		m_bones.push_back(bone);
	}

	m_root = joint_map.at(m_ref_character->getRoot());
}

void SimCharacter::CreatePhysxJoint()
{

	std::cout << "Create PHYSX JOINT" << std::endl;

	auto physx_manager = GLFWApp::getInstance()->getPhysxManager();
	auto physics = GLFWApp::getInstance()->getPhysxManager()->getPhysics();

#if USE_REDUCED_COORDINATE_ARTICULATION
	auto articulation = physics->createArticulationReducedCoordinate();
#else
	auto articulation = physics->createArticulation();
#endif	
	articulation->setSolverIterationCounts(4);
	
	auto root_pos = m_root->m_transform.m_translation;
	
	//auto node = m_root;
	auto prev = nullptr;

	std::map<std::shared_ptr<Joint>, PxArticulationLink*> joint_link_map;
	std::stack<std::shared_ptr<Joint>> dfs_stack;
		
	std::unordered_map<std::shared_ptr<Joint>, bool> visited;
	for (size_t i = 0; i < m_joints.size(); ++i)
	{
		visited.emplace(m_joints[i], false);
	}

	auto root_pose = m_root->m_transform.getModelMatWorld();
	auto root_global_trans = GetTranslationFromModel(root_pose);

	// Setup root
	auto root_link = articulation->createLink(
		nullptr, 
		PxTransform(Glm2PxVec3(root_global_trans))
	);
	
	PxRigidActorExt::createExclusiveShape(
		*root_link, 
		PxBoxGeometry(0.01f, 0.01f, 0.01f), 
		*(physx_manager->getMaterial())
	);
	
	PxFixedJoint* fixed_root_joint = PxFixedJointCreate(
		*physics,
		nullptr,
		root_link->getGlobalPose(),
		root_link,
		PxTransform(0, 0, 0)
	);
	
	PxRigidBodyExt::updateMassAndInertia(*root_link, 3.f);
	root_link->setMass(3.f);
	root_link->setAngularDamping(0.15f);
	root_link->setMaxContactImpulse(100.0f);
	root_link->setMaxDepenetrationVelocity(100.0f);
	
#if _DEBUG
	{
		const char* name = m_root->getName().c_str();
		// setName doesnt work why?
		static_cast<PxActor*>(root_link)->setName(name);
	}
#endif

	dfs_stack.push(m_root);
	
	auto prev_link = root_link;
	joint_link_map.emplace(m_root, root_link);

	std::cout << m_root->getName() << "\n";
	//std::cout << static_cast<PxActor*>(root_link)->getName() << "\n";
	
	int count = 0;
	// traverse every joints in the skeleton
	while (!dfs_stack.empty() && dfs_stack.top()!= nullptr)
	{
		auto node = dfs_stack.top();
		dfs_stack.pop();
		
		/*
			A joint link is the connection from parent joint to this joint
			After creat link, we cache this link to our joint_link_map
		*/
		if (!visited[node] && node->getParent() != nullptr)
		{
			std::cout << node->getName() << " Parent: " << node->getParent()->getName() << "\n";
			
			auto node_pose = node->m_transform.getModelMatWorld();
			auto trans = GetTranslationFromModel(node_pose);
			auto scalar = GetScalarFromModel(node_pose);

			auto parent_pose = node->getParent()->m_transform.getModelMatWorld();
			auto parent_trans = GetTranslationFromModel(parent_pose);

			auto v = parent_trans - trans;
			auto global_rot = (glm::length2(v) > 0.00001f) ? RotateBetweenVectors(glm::vec3(1, 0, 0), v) : glm::quat();

			auto link_trans = 0.5f * (trans + parent_trans);
			auto link_rot = Glm2PxQuat(global_rot); 

			prev_link = joint_link_map[node->getParent()];
			
			auto link = articulation->createLink(
				prev_link, 
				PxTransform(Glm2PxVec3(link_trans), link_rot)
			);
			//PxRigidBodyExt::updateMassAndInertia(*link, 3.f);
			link->setMass(3.f);
			/*
			link->setAngularDamping(0.15f);
			link->setMaxContactImpulse(100.0f);
			link->setMaxDepenetrationVelocity(100.0f);
			*/
			m_px_articulation_links.push_back(link);
			float pose_offset = 0;
			
			// prevent inner collision crashes
			if (glm::length(node->m_transform.m_translation) > 0.0001f )//&& node->getName().find("End") == std::string::npos)
			{
				float scalar = glm::length(trans - parent_trans) / 2.0f;
				float set_y = 0.15f;
				float set_z = 0.15f;
				float set_x = scalar * 0.8f;
				//float set_y = scalar * 0.05f;
				//float set_z = scalar * 0.05f;
				float volume_scalar = scalar * scalar * scalar;
				
				PxRigidActorExt::createExclusiveShape(
					*link,
					PxBoxGeometry(PxVec3(set_x, set_y, set_z)),
					*(physx_manager->getMaterial())
				);
				
				/*
				PxRigidActorExt::createExclusiveShape(
					*link,
					PxCapsuleGeometry(set_y, set_x),
					*(physx_manager->getMaterial())
				);
				*/


				// Update physical property if big enough
				//PxRigidBodyExt::updateMassAndInertia(*link, 3.f);
				float cubic_scalar = scalar * scalar * scalar;
				cubic_scalar = (cubic_scalar < 0.1f) ? 0.1f: cubic_scalar;
				//PxRigidBodyExt::updateMassAndInertia(*link, 3.f);
				link->setMass(3.f);

#if _DEBUG
				//float length = 0.8f * glm::length(scalar);
				//std::cout << length << std::endl;
#endif 
				pose_offset = scalar;
			}
			/*
			else
			{
				PxRigidActorExt::createExclusiveShape(
					*link,
					PxBoxGeometry(PxVec3(0.001f, 0.001f, 0.001f)),
					*(physx_manager->getMaterial())
				);

				PxRigidBodyExt::updateMassAndInertia(*link, 0.003f);
			}
			*/
			
					
#if _DEBUG
			// Check if link is not null
			{
				const char* name = node->getName().c_str();
				static_cast<PxActor*>(link)->setName(name);
				assert(link);
			}
#endif		
			joint_link_map.emplace(node, link);
			// Set Joints
			// Find grandparent's pose and compute parent_offset
			float parent_offset = 0;
			auto grand_joint = node->getParent() ? node->getParent()->getParent() : nullptr;
						
			if (grand_joint)
			{
				auto grand_pose = grand_joint->m_transform.getModelMatWorld();
				auto grand_trans = GetTranslationFromModel(grand_pose);
				parent_offset = glm::length(parent_trans - grand_trans) / 2.0f;
			}
			parent_offset = -parent_offset;

			// Create joint that links parent and current link
			// Need local rotation of "link" at setParentPose()
			
			auto node_rot = node->m_transform.getQuaternion();
			
			glm::mat4 M_p = glm::mat4(1.0f);
			glm::mat4 M_p_inv = glm::mat4(1.0f); 
			if (node->getParent())
			{
				M_p = glm::toMat4(GetQuatFromModel(node->getParent()->m_transform.getModelMatWorld()));
				M_p_inv = glm::inverse(M_p);
			}

			glm::mat4 M_l = M_p_inv * glm::toMat4(GetQuatFromModel(node->m_transform.getModelMatWorld()));
			glm::quat local_rot = glm::quat(M_l);

			auto parent_pose_rot = Glm2PxQuat(local_rot);
#ifdef USE_REDUCED_COORDINATE_ARTICULATION
			auto joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
			joint->setJointType(PxArticulationJointType::eSPHERICAL);
			joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
			joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
			joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLOCKED);
			joint->setMaxJointVelocity(100.f);
#else
			auto joint = static_cast<PxArticulationJoint*>(link->getInboundJoint());
			joint->setDamping(15.0f);
			joint->setDriveType(PxArticulationJointDriveType::eERROR);
#endif
			//joint->setParentPose(PxTransform(PxVec3(0.f, parent_offset, 0.f), parent_pose_rot));
			//joint->setChildPose(PxTransform(PxVec3(0.f, pose_offset, 0.f)));
			if (glm::abs(parent_offset) < 0.001f)
			{
				if (parent_offset > 0)
					parent_offset = 0.001f;
				else
					parent_offset = -0.001f;
			}

			/*
			if (glm::abs(pose_offset) < 0.001f)
				pose_offset = 0.001f * (glm::abs(pose_offset) / pose_offset);
			*/

			joint->setParentPose(PxTransform(PxVec3(parent_offset, 0.f, 0.f), parent_pose_rot));
			joint->setChildPose(PxTransform(PxVec3(pose_offset, 0.f, 0.f)));
			
			m_px_articulation_joints.push_back(joint);

			float mass = link->getMass();
			dfs_stack.push(m_root);
			
			visited[node] = true;
/*
#if _DEBUG
			if (node->getName().find("End_site") != std::string::npos)
			{
				break;
			}
#endif
*/
		}
		
		for (auto it = node->getChilds().begin(); it != node->getChilds().end(); ++it)
		{
			auto child = (*it);
			if (!visited[child])
			{
				dfs_stack.push(child);
			}
		}
	}//end of while
	
	physx_manager->getScene()->addArticulation(*articulation);
	//physx_manager->getScene()->setSolverArticulationBatchSize(8);
	m_articulation = articulation;

	for (size_t i = 0; i < m_px_articulation_links.size(); ++i)
	{
		std::cout << m_px_articulation_links[i]->getMass() << std::endl;
	}
	
	std::cout << "\nJoint Creation Finished" << std::endl;
}

void SimCharacter::CreateMeshes()
{
	auto resource_manager = GLFWApp::getInstance()->getResourceManager();
	auto mesh = resource_manager->getMeshes()[3];
	
	for (size_t i=0; i < m_px_articulation_links.size(); ++i)
	{
		auto link_pose = m_px_articulation_links[i]->getGlobalPose();
		PxShape* shape = nullptr;
		m_px_articulation_links[i]->getShapes(&shape, 1);

		if (shape)
		{
			auto translation = Px2GlmVec3(link_pose.p) + m_visual_offset;
			auto quaternion = Px2GlmQuat(link_pose.q);

			//PxCapsuleGeometry geometry;
			//shape->getCapsuleGeometry(geometry);

			PxBoxGeometry geometry;
			shape->getBoxGeometry(geometry);
			//auto scalar = glm::vec3(geometry.halfHeight, geometry.radius, geometry.radius); // change this
			auto scalar = Px2GlmVec3(geometry.halfExtents);
			Transform trans(translation, quaternion, scalar);

			auto obj = std::make_shared<GameObject>();
			obj->Initialize(trans);
			obj->setMesh(mesh);

			m_renderable_objs.push_back(obj);
			resource_manager->AddGameObject(obj);
		}
	}
}

void SimCharacter::CreateMeshesEx()
{
	auto resource_manager = GLFWApp::getInstance()->getResourceManager();
	auto mesh = resource_manager->getMeshes()[3];

	auto links = m_articulated_obj->getLinks();

	for (size_t i = 0; i < links.size(); ++i)
	{
		auto link_pose = links[i]->m_rigid_body->getGlobalPose();
		PxShape* shape = nullptr;
		shape = links[i]->m_shape;

		if (shape)
		{
			auto translation = Px2GlmVec3(link_pose.p) + m_visual_offset;
			auto quaternion = Px2GlmQuat(link_pose.q);

			//PxCapsuleGeometry geometry;
			//shape->getCapsuleGeometry(geometry);

			PxBoxGeometry geometry;
			shape->getBoxGeometry(geometry);
			//auto scalar = glm::vec3(geometry.halfHeight, geometry.radius, geometry.radius); // change this
			auto scalar = Px2GlmVec3(geometry.halfExtents);
			Transform trans(translation, quaternion, scalar);

			auto obj = std::make_shared<GameObject>();
			obj->Initialize(trans);
			obj->setMesh(mesh);

			m_renderable_objs.push_back(obj);
			resource_manager->AddGameObject(obj);
		}
	}
}

void SimCharacter::CreateMeshes_Dirty()
{
	auto resource_manager = GLFWApp::getInstance()->getResourceManager();
	auto box_mesh		= resource_manager->FindMeshByName("Box");
	auto sphere_mesh	= resource_manager->FindMeshByName("Capsule");
	auto capsule_mesh	= resource_manager->FindMeshByName("Sphere");
	auto coord_mesh		= resource_manager->FindMeshByName("Coordinate");
	
	if ( box_mesh == nullptr || sphere_mesh == nullptr || capsule_mesh == nullptr)
	{
		std::cout << "SimCharacter::CreateMeshes_Dirty " << "Cannot find mesh!\n" << std::endl;
		return;
	}

	for (size_t i = 0; i < m_sim_links.size(); ++i)
	{
		auto link_pose = m_sim_links[i]->getGlobalPose();
		PxShape* shape = nullptr;
		m_sim_links[i]->getLink()->getShapes(&shape, 1);

		if (shape)
		{
			auto translation = Px2GlmVec3(link_pose.p) + m_visual_offset;
			auto quaternion = Px2GlmQuat(link_pose.q);

			//PxCapsuleGeometry geometry;
			//shape->getCapsuleGeometry(geometry);

			//PxBoxGeometry geometry;
			
			//shape->getBoxGeometry(geometry);
			//auto scalar = glm::vec3(geometry.halfHeight, geometry.radius, geometry.radius); // change this
			//auto scalar = Px2GlmVec3(geometry.halfExtents);
			glm::vec3 scalar(1, 1, 1);

			auto obj = std::make_shared<GameObject>();
			obj->setName(m_sim_links[i]->getName() + "_Mesh");
			
			auto geo_type = shape->getGeometryType();
			
			switch (geo_type)
			{
			case PxGeometryType::Enum::eBOX:
			{
				PxBoxGeometry box_geo;
				shape->getBoxGeometry(box_geo);
				scalar = glm::vec3(Px2GlmVec3(box_geo.halfExtents));
				obj->setMesh(box_mesh);
				break;
			}
			case PxGeometryType::Enum::eCAPSULE:
			{
				PxCapsuleGeometry capsule_geo;
				shape->getCapsuleGeometry(capsule_geo);
				scalar = glm::vec3(capsule_geo.halfHeight, capsule_geo.radius, capsule_geo.radius);
				obj->setMesh(capsule_mesh);
				break;
			}
			case PxGeometryType::Enum::eSPHERE:
			{
				PxSphereGeometry sphere_geo;
				shape->getSphereGeometry(sphere_geo);
				scalar = glm::vec3(sphere_geo.radius, sphere_geo.radius, sphere_geo.radius);
				obj->setMesh(sphere_mesh);
				break;
			}
			}

			Transform trans(translation, quaternion, scalar);
			obj->Initialize(trans);

#ifdef _DEBUG
			glm::vec3 debug_scalar(.1f, .1f, .1f);
			auto debug_obj = std::make_shared<GameObject>();
			debug_obj->setName(m_sim_links[i]->getName() + "_Coord");
			debug_obj->setMesh(coord_mesh);
			Transform debug_trans(translation, quaternion, debug_scalar);
			debug_obj->Initialize(debug_trans);
			m_debug_objs.push_back(debug_obj);
			resource_manager->AddGameObject(debug_obj);
#endif
			
			m_renderable_objs.push_back(obj);
			resource_manager->AddGameObject(obj);
		}
	}
	return;
}

void SimCharacter::CreatePhysxJointEx()
{
	std::cout << "Create PHYSX JOINT EX" << std::endl;

	auto physx_manager = GLFWApp::getInstance()->getPhysxManager();
	auto physics = GLFWApp::getInstance()->getPhysxManager()->getPhysics();
	auto root_pos = m_root->m_transform.m_translation;
	
	auto prev = nullptr;

	std::map<std::shared_ptr<Joint>, ArticulationLink*> joint_link_map;
	std::stack<std::shared_ptr<Joint>> dfs_stack;

	std::unordered_map<std::shared_ptr<Joint>, bool> visited;
	for (size_t i = 0; i < m_joints.size(); ++i)
	{
		visited.emplace(m_joints[i], false);
	}

	auto root_pose = m_root->m_transform.getModelMatWorld();
	auto root_global_trans = GetTranslationFromModel(root_pose);
	
	m_articulated_obj = std::make_shared<ArticulatedObject>();
	
	// Setup root
	auto root_link = m_articulated_obj->CreateLink(nullptr, PxTransform(Glm2PxVec3(root_global_trans)), "Root");
	root_link->m_shape = PxRigidActorExt::createExclusiveShape(
		*(root_link->m_rigid_body),
		PxBoxGeometry(0.01f, 0.01f, 0.01f),
		*(physx_manager->getMaterial())
	);
	root_link->m_parent = nullptr;
	m_articulated_obj->setLinkRoot(root_link);
	
	/*
	PxFixedJoint* fixed_root_joint = PxFixedJointCreate(
		*physics,
		nullptr,
		root_link->m_rigid_body->getGlobalPose(),
		root_link->m_rigid_body,
		PxTransform(0, 0, 0)
	);
	*/
	//PxRigidBodyExt::updateMassAndInertia(*root_link, 3.f);

#if _DEBUG
	{
		const char* name = m_root->getName().c_str();
		// setName doesnt work why?
		root_link->m_rigid_body->setName(name);
		//static_cast<PxActor*>(root_link)->m_rigid_body->setName(name);
	}
#endif

	dfs_stack.push(m_root);

	auto prev_link = root_link;
	joint_link_map.emplace(m_root, root_link);

	std::cout << m_root->getName() << "\n";
	//std::cout << static_cast<PxActor*>(root_link)->getName() << "\n";

	int count = 0;
	// traverse every joints in the skeleton
	while (!dfs_stack.empty() && dfs_stack.top() != nullptr)
	{
		// node : std::shared_ptr<Joint>
		auto node = dfs_stack.top();
		dfs_stack.pop();

		/*
			A joint link is the connection from parent joint to this joint
			After creat link, we cache this link to our joint_link_map
		*/
		if (!visited[node] && node->getParent() != nullptr)
		{
#ifdef _DEBUG
			//std::cout << node->getName() << ", Parent: " << node->getParent()->getName() << "\n";
#endif

			auto node_pose_world = node->m_transform.getModelMatWorld();
			auto node_position_world = GetTranslationFromModel(node_pose_world);

			auto parent_pose_world = node->getParent()->m_transform.getModelMatWorld();
			auto parent_position_world = GetTranslationFromModel(parent_pose_world);

			auto v = parent_position_world - node_position_world;
			auto global_rot = (glm::length2(v) > 0.0000001f) ? RotateBetweenVectors(glm::vec3(1, 0, 0), v) : glm::quat();

			auto link_trans = 0.5f * (node_position_world + parent_position_world);
			auto link_rot = Glm2PxQuat(global_rot);

			prev_link = joint_link_map[node->getParent()];

			auto link = m_articulated_obj->CreateLink(
				prev_link,
				PxTransform(Glm2PxVec3(link_trans), link_rot),
				node->getName()
			);
			
			float pose_offset = 0;

			// prevent crashes when the colliders are too close
			// some people really like to use empty node to represent the hierarchy
			if (glm::length(node->m_transform.m_translation) > 0.0001f)
			{
				float scalar = glm::length(v) / 2.0f;
				float set_y = 0.15f;
				float set_z = 0.15f;
				float set_x = scalar * 0.8f;
				//float set_y = scalar * 0.05f;
				//float set_z = scalar * 0.05f;
				float volume_scalar = scalar * scalar * scalar;

				link->m_shape = PxRigidActorExt::createExclusiveShape(
					*(link->m_rigid_body),
					PxBoxGeometry(PxVec3(set_x, set_y, set_z)),
					*(physx_manager->getMaterial())
				);

				// Update physical property if big enough
				PxRigidBodyExt::updateMassAndInertia(*(link->m_rigid_body), 3.f);
				pose_offset = scalar;
			}
			/*
			else
			{
				link->m_shape = PxRigidActorExt::createExclusiveShape(
					*(link->m_rigid_body),
					PxBoxGeometry(PxVec3(0.001f, 0.001f, 0.001f)),
					*(physx_manager->getMaterial())
				);
			}*/
			

#if _DEBUG
			assert(link);
#endif		
			joint_link_map.emplace(node, link);
			// Set Joints
			// Find grandparent's pose and compute parent_offset
			float parent_offset = 0;
			auto grand_joint = node->getParent() ? node->getParent()->getParent() : nullptr;
			glm::vec3 a(0, 0, 0);

			if (grand_joint)
			{
				auto grand_pose = grand_joint->m_transform.getModelMatWorld();
				auto grand_trans = GetTranslationFromModel(grand_pose);
				parent_offset = glm::length(parent_position_world - grand_trans) / 2.0f;
				parent_offset = -parent_offset;
				a = grand_trans - parent_position_world;
			}

			glm::quat joint_orientation_glm;// = (glm::length(a) > 0.00001f) ? RotateBetweenVectors(a, v) : RotateBetweenVectors(glm::vec3(1, 0, 0), v);
			if (glm::length2(a) < 0.000001f)
			{
				if (glm::length2(v) < 0.000001f)
				{
					joint_orientation_glm = glm::quat();
				}
				else
				{
					joint_orientation_glm = RotateBetweenVectors(glm::vec3(1, 0, 0), v);
				}
				
			}
			else
			{
				// Normal condition
				if (glm::length2(v) > 0.000001f)
				{
					joint_orientation_glm = RotateBetweenVectors(a, v);
				}
				else
				{
					joint_orientation_glm = RotateBetweenVectors(a, glm::vec3(1, 0, 0));
				}
			}
			
			//auto joint_orientation_glm = (glm::length(a)> 0.00001f)? RotateBetweenVectors(a, v) : RotateBetweenVectors(glm::vec3(1,0,0),v);
			auto joint_orientation = Glm2PxQuat(joint_orientation_glm);
			// Create joint that links parent and current link
			// Need local rotation of "link" at setParentPose()

			auto node_rot = node->m_transform.getQuaternion();
			auto node_euler = node->m_transform.getEulerAngles();
			auto parent_pose_rot = Glm2PxQuat(node_rot);
			//auto node_mat = node->m_transform.getModelMatWorld();
			//auto parent_pose_quat_glm = GetQuatFromModel(node_mat);
			//auto parent_pose_rot = Glm2PxQuat(parent_pose_quat_glm);
			//auto joint = static_cast<PxArticulationJoint*>(link->getInboundJoint());
			auto parent_rigidbody = link->m_parent->m_rigid_body;
			auto current_rigidbody = link->m_rigid_body;
			
			auto joint = PxFixedJointCreate(
				*physics,
				parent_rigidbody,
				PxTransform(PxVec3(parent_offset, 0.f, 0.f), joint_orientation),
				current_rigidbody,
				PxTransform(PxVec3(pose_offset, 0.f, 0.f))
			);
			dfs_stack.push(m_root);
			visited[node] = true;
		} // if-end

		for (auto it = node->getChilds().begin(); it != node->getChilds().end(); ++it)
		{
			auto child = (*it);
			if (!visited[child])
			{
				dfs_stack.push(child);
			}
		}
		
		if (count == 9)
			break;
		count++;
		
	}//end of while

	//physx_manager->getScene()->addArticulation(*articulation);
	//physx_manager->getScene()->setSolverArticulationBatchSize(8);
	//m_articulation = articulation;
	m_articulated_obj->AddActorsToScene();

	std::cout << "\nJoint Creation Finished" << std::endl;
}

void SimCharacter::CreatePhysxJoint_dirty()
{
	auto physx_manager = GLFWApp::getInstance()->getPhysxManager();
	auto physics = GLFWApp::getInstance()->getPhysxManager()->getPhysics();

#if USE_REDUCED_COORDINATE_ARTICULATION
	auto articulation = physics->createArticulationReducedCoordinate();
#else
	auto articulation = physics->createArticulation();
#endif	
	articulation->setSolverIterationCounts(32);

	if (!articulation)
		return;

	m_articulation = articulation;

	auto name_joint_map = m_ref_character->getNameJointMap();
	
	glm::mat4 root_pose = m_root->m_transform.getModelMatWorld();
	PxVec3 cur_trans = PxVec3(0.0f, 10.f, 0.f);//Glm2PxVec3(root_global_trans);
	PxQuat cur_rot = Glm2PxQuat(RotateBetweenVectors(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0)));
	PxQuat local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(-85.f), glm::vec3(0, 0, 1)));
	PxMaterial* material = physx_manager->getMaterial();

	std::shared_ptr<SimLink> root_link = std::make_shared<SimLink>(nullptr, PxTransform(cur_trans));
	root_link->Initialize(articulation);
	root_link->CreateShape(PxSphereGeometry(0.1f), material);
	root_link->setName("hip");
	m_sim_links.push_back(root_link);
	
	/*
	PxFixedJoint* fixed_root_joint = PxFixedJointCreate(
		*physics,
		nullptr,
		root_link->getGlobalPose(),
		root_link->getLink(),
		PxTransform(0, 0, 0)
	);
	*/
	//auto ref_root = m_ref_character->getRoot();
	//glm::mat4 ref_root_rot_inv = glm::inverse(glm::toMat4(GetQuatFromModel(ref_root->m_transform.getModelMatWorld())));
	//glm::mat4 sim_root_rot = glm::toMat4(Px2GlmQuat(root_link->getLink()->getGlobalPose().q));

	//m_ref2sim = sim_root_rot * ref_root_rot_inv;
	//m_ref2sim = glm::toMat4(RotateBetweenVectors(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0)));

	// Joint setting
	{
		PxReal kp = 1000.f;
		PxReal kd = 100.f;
		PxArticulationDriveType::Enum driveType = PxArticulationDriveType::eFORCE;

		/* T-pose initialization */
		// abdomen
		cur_trans = cur_trans + PxVec3(0.f, 1.2f, 0.f) + PxVec3(0.f, 0.1f, 0.f);
		
		std::shared_ptr<SimLink> abdomen = std::make_shared<SimLink>(root_link, PxTransform(cur_trans, cur_rot));
		abdomen->Initialize(articulation);
		abdomen->CreateShape(PxCapsuleGeometry(0.2f, 1.f), material);
		abdomen->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		abdomen->SetUpInboundJointParentPose(PxTransform(PxVec3(0.f, 0.1f, 0.f), cur_rot));
		abdomen->SetUpInboundJointChildPose(PxTransform(PxVec3(-1.2f, 0.f, 0.f)));
		abdomen->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		abdomen->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		abdomen->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		abdomen->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		abdomen->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		abdomen->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		abdomen->setReferenceJoint(name_joint_map["abdomen"]);
		abdomen->setName("abdomen");
		m_sim_links.push_back(abdomen);

		// chest
		// 0, 0.36f, 0.f
		cur_trans = cur_trans + PxVec3(0.f, 2.4f, 0.f);

		std::shared_ptr<SimLink> chest = std::make_shared<SimLink>(abdomen, PxTransform(cur_trans, cur_rot));
		chest->Initialize(articulation);
		chest->CreateShape(PxCapsuleGeometry(0.2f, 1.f), material);
		chest->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		chest->SetUpInboundJointParentPose(PxTransform(PxVec3(1.2f, 0.f, 0.f)));
		chest->SetUpInboundJointChildPose(PxTransform(PxVec3(-1.2f, 0.f, 0.f)));
		chest->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		chest->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		chest->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		chest->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		chest->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		chest->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		chest->setReferenceJoint(name_joint_map["chest"]);
		chest->setName("chest");
		m_sim_links.push_back(chest);

		// neck
		// 0.f, 0.54f, 0.f
		cur_trans = cur_trans + PxVec3(0.f, 1.8f, 0.f);
		std::shared_ptr<SimLink> neck = std::make_shared<SimLink>(chest, PxTransform(cur_trans, cur_rot));
		neck->Initialize(articulation);
		neck->CreateShape(PxCapsuleGeometry(0.2f, 0.4f), material);
		neck->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		neck->SetUpInboundJointParentPose(PxTransform(PxVec3(1.2f, 0.f, 0.f)));
		neck->SetUpInboundJointChildPose(PxTransform(PxVec3(-0.6f, 0.f, 0.f)));
		neck->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		neck->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		neck->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		neck->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		neck->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		neck->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		neck->setReferenceJoint(name_joint_map["neck"]);
		neck->setName("neck");
		m_sim_links.push_back(neck);

		// head
		// PxVec3(0.f, 0.66f, 0.f)
		cur_trans = cur_trans + PxVec3(0.f, 1.2f, 0.f);
		std::shared_ptr<SimLink> head = std::make_shared<SimLink>(neck, PxTransform(cur_trans, cur_rot));
		head->Initialize(articulation);
		head->CreateShape(PxCapsuleGeometry(0.2f, 0.4f), material);
		head->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		head->SetUpInboundJointParentPose(PxTransform(PxVec3(0.6f, 0.f, 0.f)));
		head->SetUpInboundJointChildPose(PxTransform(PxVec3(-0.6f, 0.f, 0.f)));
		head->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		head->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		head->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		head->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		head->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		head->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		head->setReferenceJoint(name_joint_map["head"]);
		head->setName("head");
		m_sim_links.push_back(head);

		// rCollar
		cur_trans = chest->getLink()->getGlobalPose().p + PxVec3(-0.6f * PxCos(Deg2Rad(-5.0f)) - 0.3f, 0.6f * PxSin(Deg2Rad(-5.0f)) + 1.2f, 0);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(5.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(-85.f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> rCollar = std::make_shared<SimLink>(chest, PxTransform(cur_trans, cur_rot));
		rCollar->Initialize(articulation);
		rCollar->CreateShape(PxCapsuleGeometry(0.2f, 0.4f), material);
		rCollar->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		rCollar->SetUpInboundJointParentPose(PxTransform(PxVec3(1.2f, 0.3f, 0.f), local_rot));
		rCollar->SetUpInboundJointChildPose(PxTransform(PxVec3(0.6f, 0.f, 0.f)));
		rCollar->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		rCollar->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		rCollar->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		rCollar->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		rCollar->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		rCollar->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		rCollar->setReferenceJoint(name_joint_map["rCollar"]);
		rCollar->setName("rCollar");
		m_sim_links.push_back(rCollar);

		// rShldr
		cur_trans = rCollar->getLink()->getGlobalPose().p +
			PxVec3(-1.2f * PxCos(Deg2Rad(-2.5f)) - 0.6f* PxCos(Deg2Rad(-5.0f)), 1.2f*PxSin(Deg2Rad(-2.5f)) + 0.6f* PxSin(Deg2Rad(-5.0f)), 0);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(2.5f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(-2.5f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> rShldr = std::make_shared<SimLink>(rCollar, PxTransform(cur_trans, cur_rot));
		rShldr->Initialize(articulation);
		rShldr->CreateShape(PxCapsuleGeometry(0.2f, 1.0f), material);
		rShldr->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		rShldr->SetUpInboundJointParentPose(PxTransform(PxVec3(-0.6f, 0.f, 0.f), local_rot));
		rShldr->SetUpInboundJointChildPose(PxTransform(PxVec3(1.2f, 0.f, 0.f)));
		rShldr->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		rShldr->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		rShldr->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		rShldr->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		rShldr->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		rShldr->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		rShldr->setReferenceJoint(name_joint_map["rShldr"]);
		rShldr->setName("rShldr");
		m_sim_links.push_back(rShldr);

		// rForeArm
		cur_trans = rShldr->getLink()->getGlobalPose().p +
			PxVec3(-1.2f * PxCos(Deg2Rad(0.f)) - 1.2f * PxCos(Deg2Rad(-2.5f)), 1.2f * PxSin(Deg2Rad(0.f)) + 1.2f * PxSin(Deg2Rad(-2.5f)), 0);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(0.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(-2.5f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> rForeArm = std::make_shared<SimLink>(rShldr, PxTransform(cur_trans, cur_rot));
		rForeArm->Initialize(articulation);
		rForeArm->CreateShape(PxCapsuleGeometry(0.2f, 1.0f), material);
		rForeArm->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		rForeArm->SetUpInboundJointParentPose(PxTransform(PxVec3(-1.2f, 0.f, 0.f), local_rot));
		rForeArm->SetUpInboundJointChildPose(PxTransform(PxVec3(1.2f, 0.f, 0.f)));
		rForeArm->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		rForeArm->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		rForeArm->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		rForeArm->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		rForeArm->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		rForeArm->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		rForeArm->setReferenceJoint(name_joint_map["rHand"]);
		rForeArm->setName("rForeArm");
		m_sim_links.push_back(rForeArm);


		// rHand
		cur_trans = rForeArm->getLink()->getGlobalPose().p +
			PxVec3(-0.6f * PxCos(Deg2Rad(0.f)) - 1.2f * PxCos(Deg2Rad(0.f)), 0.6f * PxSin(Deg2Rad(0.f)) + 1.2f * PxSin(Deg2Rad(0.f)), 0);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(0.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(0.f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> rHand = std::make_shared<SimLink>(rForeArm, PxTransform(cur_trans, cur_rot));
		rHand->Initialize(articulation);
		rHand->CreateShape(PxCapsuleGeometry(0.2f, 0.4f), material);
		rHand->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		rHand->SetUpInboundJointParentPose(PxTransform(PxVec3(-1.2f, 0.f, 0.f), local_rot));
		rHand->SetUpInboundJointChildPose(PxTransform(PxVec3(0.6f, 0.f, 0.f)));
		rHand->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		rHand->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		rHand->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		rHand->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		rHand->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		rHand->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		rHand->setReferenceJoint(name_joint_map["rThumb1"]);
		rHand->setName("rHand");
		m_sim_links.push_back(rHand);


		// lCollar
		cur_trans =
			PxVec3(
				-1.f * rCollar->getLink()->getGlobalPose().p.x,
				rCollar->getLink()->getGlobalPose().p.y,
				rCollar->getLink()->getGlobalPose().p.z
			);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(-5.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(-95.f), glm::vec3(0, 0, 1)));
		//cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(120.f), glm::vec3(0,0,1)));

		std::shared_ptr<SimLink> lCollar = std::make_shared<SimLink>(chest, PxTransform(cur_trans, cur_rot));
		lCollar->Initialize(articulation);
		lCollar->CreateShape(PxCapsuleGeometry(0.2f, 0.4f), material);
		lCollar->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		lCollar->SetUpInboundJointParentPose(PxTransform(PxVec3(1.2f, -0.3f, 0.f), local_rot));
		lCollar->SetUpInboundJointChildPose(PxTransform(PxVec3(-0.6f, 0.f, 0.f)));
		lCollar->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		lCollar->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		lCollar->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		lCollar->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		lCollar->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		lCollar->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		lCollar->setReferenceJoint(name_joint_map["lCollar"]);
		lCollar->setName("lCollar");
		m_sim_links.push_back(lCollar);


		// lShldr
		cur_trans =
			PxVec3(-1.f * rShldr->getGlobalPose().p.x, rShldr->getGlobalPose().p.y, rShldr->getGlobalPose().p.z);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(-2.5f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(2.5f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> lShldr = std::make_shared<SimLink>(lCollar, PxTransform(cur_trans, cur_rot));
		lShldr->Initialize(articulation);
		lShldr->CreateShape(PxCapsuleGeometry(0.2f, 1.0f), material);
		lShldr->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		lShldr->SetUpInboundJointParentPose(PxTransform(PxVec3(0.6f, 0.f, 0.f), local_rot));
		lShldr->SetUpInboundJointChildPose(PxTransform(PxVec3(-1.2f, 0.f, 0.f)));
		lShldr->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		lShldr->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		lShldr->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		lShldr->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		lShldr->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		lShldr->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		lShldr->setReferenceJoint(name_joint_map["lShldr"]);
		lShldr->setName("lShldr");
		m_sim_links.push_back(lShldr);


		// lForeArm
		cur_trans =
			PxVec3(-1.f * rForeArm->getGlobalPose().p.x, rForeArm->getGlobalPose().p.y, rForeArm->getGlobalPose().p.z);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(0.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(2.5f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> lForeArm = std::make_shared<SimLink>(lShldr, PxTransform(cur_trans, cur_rot));
		lForeArm->Initialize(articulation);
		lForeArm->CreateShape(PxCapsuleGeometry(0.2f, 1.0f), material);
		lForeArm->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		lForeArm->SetUpInboundJointParentPose(PxTransform(PxVec3(1.2f, 0.f, 0.f), local_rot));
		lForeArm->SetUpInboundJointChildPose(PxTransform(PxVec3(-1.2f, 0.f, 0.f)));
		lForeArm->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		lForeArm->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		lForeArm->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		lForeArm->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		lForeArm->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		lForeArm->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		lForeArm->setReferenceJoint(name_joint_map["lForeArm"]);
		lForeArm->setName("lForeArm");
		m_sim_links.push_back(lForeArm);


		// lHand
		cur_trans =
			PxVec3(-1.f * rHand->getGlobalPose().p.x, rHand->getGlobalPose().p.y, rHand->getGlobalPose().p.z);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(0.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(0.f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> lHand = std::make_shared<SimLink>(lForeArm, PxTransform(cur_trans, cur_rot));
		lHand->Initialize(articulation);
		lHand->CreateShape(PxCapsuleGeometry(0.2f, 0.4f), material);
		lHand->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		lHand->SetUpInboundJointParentPose(PxTransform(PxVec3(1.2f, 0.f, 0.f), local_rot));
		lHand->SetUpInboundJointChildPose(PxTransform(PxVec3(-0.6f, 0.f, 0.f)));
		lHand->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		lHand->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		lHand->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		lHand->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		lHand->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		lHand->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		lHand->setReferenceJoint(name_joint_map["lHand"]);
		lHand->setName("lHand");
		m_sim_links.push_back(lHand);
		
		
		// rButtock
		cur_trans = root_link->getGlobalPose().p
			+ PxVec3(-0.3f*PxCos(Deg2Rad(30.0f)) - 0.1f*PxCos(Deg2Rad(30.0f)), -0.3f*PxSin(Deg2Rad(30.0f)) - 0.1f*PxSin(Deg2Rad(30.0f)), 0);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(30.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(30.f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> rButtock = std::make_shared<SimLink>(root_link, PxTransform(cur_trans, cur_rot));
		rButtock->Initialize(articulation);
		rButtock->CreateShape(PxCapsuleGeometry(0.2f, 0.1f), material);
		rButtock->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		rButtock->SetUpInboundJointParentPose(PxTransform(PxVec3(-0.1f*PxCos(Deg2Rad(30.0f)), -0.1f*PxSin(Deg2Rad(30.0f)), 0.f), local_rot));
		rButtock->SetUpInboundJointChildPose(PxTransform(PxVec3(0.3f, 0.f, 0.f)));
		rButtock->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		rButtock->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		rButtock->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		rButtock->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		rButtock->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		rButtock->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		rButtock->setReferenceJoint(name_joint_map["rButtock"]);
		rButtock->setName("rButtock");
		m_sim_links.push_back(rButtock);


		// rThigh
		cur_trans = rButtock->getGlobalPose().p
			+ PxVec3(-1.8f * PxCos(Deg2Rad(85.0f)) - 0.3f * PxCos(Deg2Rad(30.0f)), -1.8f * PxSin(Deg2Rad(85.0f)) - 0.3f * PxSin(Deg2Rad(30.0f)), 0);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(85.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(55.f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> rThigh = std::make_shared<SimLink>(rButtock, PxTransform(cur_trans, cur_rot));
		rThigh->Initialize(articulation);
		rThigh->CreateShape(PxCapsuleGeometry(0.2f, 1.6f), material);
		rThigh->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		rThigh->SetUpInboundJointParentPose(PxTransform(PxVec3(-0.3f, 0.f, 0.f), local_rot));
		rThigh->SetUpInboundJointChildPose(PxTransform(PxVec3(1.8f, 0.f, 0.f)));
		rThigh->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		rThigh->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		rThigh->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		rThigh->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		rThigh->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		rThigh->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		rThigh->setReferenceJoint(name_joint_map["rThigh"]);
		rThigh->setName("rThigh");
		m_sim_links.push_back(rThigh);


		// rShin
		cur_trans = rThigh->getGlobalPose().p
			+ PxVec3(-1.8f * PxCos(Deg2Rad(90.0f)) - 1.8f * PxCos(Deg2Rad(85.0f)), -1.8f * PxSin(Deg2Rad(90.0f)) - 1.8f * PxSin(Deg2Rad(85.0f)), 0);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(90.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(5.f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> rShin = std::make_shared<SimLink>(rThigh, PxTransform(cur_trans, cur_rot));
		rShin->Initialize(articulation);
		rShin->CreateShape(PxCapsuleGeometry(0.2f, 1.6f), material);
		rShin->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		rShin->SetUpInboundJointParentPose(PxTransform(PxVec3(-1.8f, 0.f, 0.f), local_rot));
		rShin->SetUpInboundJointChildPose(PxTransform(PxVec3(1.8f, 0.f, 0.f)));
		rShin->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		rShin->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		rShin->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		rShin->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		rShin->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		rShin->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		rShin->setReferenceJoint(name_joint_map["rShin"]);
		rShin->setName("rShin");
		m_sim_links.push_back(rShin);


		// rFoot
		cur_trans = rShin->getGlobalPose().p
			+ PxVec3(-1.8f * PxCos(Deg2Rad(90.0f)), -0.1f - 1.8f * PxSin(Deg2Rad(90.0f)), 0.15f);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(0.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(-90.f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> rFoot = std::make_shared<SimLink>(rShin, PxTransform(cur_trans, cur_rot));
		rFoot->Initialize(articulation);
		rFoot->CreateShape(PxBoxGeometry(0.25f, 0.1f, 0.5f), material);
		rFoot->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		rFoot->SetUpInboundJointParentPose(PxTransform(PxVec3(-1.8f, 0.f, 0.f), local_rot));
		rFoot->SetUpInboundJointChildPose(PxTransform(PxVec3(0.f, 0.1f, -0.15f)));
		rFoot->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		rFoot->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		rFoot->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		rFoot->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		rFoot->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		rFoot->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		rFoot->setReferenceJoint(name_joint_map["rFoot"]);
		rFoot->setName("rFoot");
		m_sim_links.push_back(rFoot);


		// lButtock
		cur_trans =
			PxVec3(-1.f * rButtock->getGlobalPose().p.x, rButtock->getGlobalPose().p.y, rButtock->getGlobalPose().p.z);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(150.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(150.f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> lButtock = std::make_shared<SimLink>(root_link, PxTransform(cur_trans, cur_rot));
		lButtock->Initialize(articulation);
		lButtock->CreateShape(PxCapsuleGeometry(0.2f, 0.1f), material);
		lButtock->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		lButtock->SetUpInboundJointParentPose(PxTransform(PxVec3(-0.1f * PxCos(Deg2Rad(150.0f)), -0.1f * PxSin(Deg2Rad(150.0f)), 0.f), local_rot));
		lButtock->SetUpInboundJointChildPose(PxTransform(PxVec3(0.3f, 0.f, 0.f)));
		lButtock->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		lButtock->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		lButtock->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		lButtock->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		lButtock->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		lButtock->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		lButtock->setReferenceJoint(name_joint_map["lButtock"]);
		lButtock->setName("lButtock");
		m_sim_links.push_back(lButtock);


		// lThigh
		cur_trans =
			PxVec3(-1.f * rThigh->getGlobalPose().p.x, rThigh->getGlobalPose().p.y, rThigh->getGlobalPose().p.z);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(95.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(-55.f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> lThigh = std::make_shared<SimLink>(lButtock, PxTransform(cur_trans, cur_rot));
		lThigh->Initialize(articulation);
		lThigh->CreateShape(PxCapsuleGeometry(0.2f, 1.6f), material);
		lThigh->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		lThigh->SetUpInboundJointParentPose(PxTransform(PxVec3(-0.3f, 0.f, 0.f), local_rot));
		lThigh->SetUpInboundJointChildPose(PxTransform(PxVec3(1.8f, 0.f, 0.f)));
		lThigh->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		lThigh->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		lThigh->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		lThigh->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		lThigh->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		lThigh->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		lThigh->setReferenceJoint(name_joint_map["lThigh"]);
		lThigh->setName("lThigh");
		m_sim_links.push_back(lThigh);


		// lShin
		cur_trans =
			PxVec3(-1.f * rShin->getGlobalPose().p.x, rShin->getGlobalPose().p.y, rShin->getGlobalPose().p.z);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(90.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(-5.f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> lShin = std::make_shared<SimLink>(lThigh, PxTransform(cur_trans, cur_rot));
		lShin->Initialize(articulation);
		lShin->CreateShape(PxCapsuleGeometry(0.2f, 1.6f), material);
		lShin->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		lShin->SetUpInboundJointParentPose(PxTransform(PxVec3(-1.8f, 0.f, 0.f), local_rot));
		lShin->SetUpInboundJointChildPose(PxTransform(PxVec3(1.8f, 0.f, 0.f)));
		lShin->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		lShin->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		lShin->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		lShin->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		lShin->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		lShin->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		lShin->setReferenceJoint(name_joint_map["lShin"]);
		lShin->setName("lShin");
		m_sim_links.push_back(lShin);


		// lFoot
		cur_trans =
			PxVec3(-1.f * rFoot->getGlobalPose().p.x, rFoot->getGlobalPose().p.y, rFoot->getGlobalPose().p.z);
		cur_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(0.f), glm::vec3(0, 0, 1)));
		local_rot = Glm2PxQuat(glm::angleAxis(Deg2Rad(-90.f), glm::vec3(0, 0, 1)));

		std::shared_ptr<SimLink> lFoot = std::make_shared<SimLink>(lShin, PxTransform(cur_trans, cur_rot));
		lFoot->Initialize(articulation);
		lFoot->CreateShape(PxBoxGeometry(0.25f, 0.1f, 0.5f), material);
		lFoot->SetUpInboundJointType(PxArticulationJointType::eSPHERICAL);
		lFoot->SetUpInboundJointParentPose(PxTransform(PxVec3(-1.8f, 0.f, 0.f), local_rot));
		lFoot->SetUpInboundJointChildPose(PxTransform(PxVec3(0.f, 0.1f, -0.15f)));
		lFoot->SetUpInboundJointMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
		lFoot->SetUpInboundJointMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
		lFoot->SetUpInboundJointMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		lFoot->SetUpInboundJointDrive(PxArticulationAxis::eTWIST, kp, kd, PX_MAX_F32, driveType);
		lFoot->SetUpInboundJointDrive(PxArticulationAxis::eSWING1, kp, kd, PX_MAX_F32, driveType);
		lFoot->SetUpInboundJointDrive(PxArticulationAxis::eSWING2, kp, kd, PX_MAX_F32, driveType);
		lFoot->setReferenceJoint(name_joint_map["lFoot"]);
		lFoot->setName("lFoot");
		m_sim_links.push_back(lFoot);
		
	}

	physx_manager->getScene()->addArticulation(*m_articulation);

}

void SimCharacter::InitializePose(uint32_t pose_start)
{
	auto anim = m_ref_character->getAnimation();
	m_ref_start = pose_start;
	anim->setFrameStart(pose_start);

	// TODO: Reset to desired start pose
	// Option 1: Drive to target
	// Option 2: Reset poses (Recompute)
	
	for (size_t i = 0; i < m_sim_links.size(); ++i)
	{
		m_sim_links[i]->ResetPose();
	}
}

PxArticulationLink* SimCharacter::CreateArticulationLink_dirty(
	PxArticulationLink* parent, 
	PxTransform link_pose,
	const PxGeometry &geometry)
{
	if (!m_articulation)
		return nullptr;
	auto physx_manager = GLFWApp::getInstance()->getPhysxManager();
	PxMaterial* material = physx_manager->getMaterial();
	PxArticulationLink* link = m_articulation->createLink(
		parent,
		link_pose
	);

	PxShape* shape = PxRigidActorExt::createExclusiveShape(*link, geometry, *material);
	
	/*
	// create Y-axis capsule
	if (geometry.getType() == PxGeometryType::eCAPSULE)
	{
		const PxTransform relativePose(Glm2PxQuat(RotateBetweenVectors(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0))));
		shape->setLocalPose(relativePose);
	}
	*/
		
	return link;
}

PxArticulationJointReducedCoordinate* SimCharacter::CreateArticulationJoint_dirty()
{
	return nullptr;
}
