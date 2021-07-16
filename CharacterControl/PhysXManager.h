#ifndef _PHYSX_WRAPPER_
#define _PHYSX_WRAPPER_

#include <PxPhysicsAPI.h>
#include <vector>

using namespace physx;

class PhysXManager
{
public:
	PhysXManager();
	~PhysXManager();

	void InitPhysics(bool interactive);
	void CleanUpPhysics();

	void StepPhysics(const float &deltaTime);

	void CreateMaterial(float static_friction, float dynamic_friction, float restitution);
	void CreatePlane();


	inline PxPhysics*	getPhysics()	{ return m_physics; };
	inline PxMaterial*	getMaterial()	{ return m_material; };
	inline PxScene*		getScene()		{ return m_scene; };
	inline float		getDeltaTime()	{ return m_dt; };

private:
	PxDefaultAllocator		m_allocator;
	PxDefaultErrorCallback	m_errorCallback;

	PxFoundation*			m_foundation = nullptr;
	PxPhysics*				m_physics = nullptr;

	PxDefaultCpuDispatcher*	m_dispatcher = nullptr;
	PxCudaContextManager*	m_cuda_context_manager = nullptr;
	PxScene*				m_scene = nullptr;
	
	PxMaterial*				m_material = nullptr;

	PxPvd*                  m_pvd = nullptr;

	std::vector<PxScene*>	m_scenes; 

	float					m_dt;

};

#endif
