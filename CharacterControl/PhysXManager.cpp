#include "PhysXManager.h"
#include <iostream>

#define PVD_HOST "127.0.0.1"

PhysXManager::PhysXManager() :  m_foundation(nullptr), 
								m_physics(nullptr), m_dispatcher(nullptr), m_cuda_context_manager(nullptr),
								m_scene(nullptr), //m_material(nullptr),
								m_pvd(nullptr), m_dt(1.f / 120.f)
{
}

PhysXManager::~PhysXManager()
{
}

void PhysXManager::InitPhysics(bool interactive)
{
	m_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, m_allocator, m_errorCallback);

	PxTolerancesScale tolerance;
	/*
	tolerance.length = 100;
	tolerance.speed = 981;
	*/

#ifdef _DEBUG
	/*PhysX Visual Debugger*/
	m_pvd = PxCreatePvd(*m_foundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	//PxPvdTransport* transport = PxDefaultPvdFileTransportCreate("pvd_test.pxd2");
	m_pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	/*PhysX*/
	m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_foundation, tolerance, true, m_pvd);
	PxInitExtensions(*m_physics, m_pvd);
#else
	m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_foundation, tolerance, false, nullptr);
	PxInitExtensions(*m_physics, nullptr);
#endif

	/*CUDA*/
	//PxCudaContextManagerDesc cuda_context_desc;
	//auto cuda_context_manager = PxCreateCudaContextManager(*m_foundation, cuda_context_desc, PxGetProfilerCallback());
	
	/* PhysX Scene */
	PxSceneDesc sceneDesc(m_physics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, 0.0f, 0.0f);
	sceneDesc.solverType = PxSolverType::eTGS;
	m_dispatcher = PxDefaultCpuDispatcherCreate(16);
	sceneDesc.cpuDispatcher = m_dispatcher;
	if (!m_dispatcher)
		std::cout << "Dispathcer create failed\n";
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	sceneDesc.cudaContextManager = m_cuda_context_manager;

	sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
	sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;

	m_scene = m_physics->createScene(sceneDesc);
#ifdef _DEBUG
	PxPvdSceneClient* pvdClient = m_scene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
#endif
	
}

void PhysXManager::CleanUpPhysics()
{
	m_scene->release();
	m_dispatcher->release();
	m_physics->release();
#ifdef _DEBUG
	if (m_pvd)
	{
		PxPvdTransport* transport = m_pvd->getTransport();
		m_pvd->release();
		transport->release();
	}	
	m_foundation->release();
#endif
}

void PhysXManager::StepPhysics(const float &deltaTime)
{
	m_scene->simulate(deltaTime);
	m_scene->fetchResults(true);
}

void PhysXManager::CreateMaterial(float static_friction, float dynamic_friction, float restitution)
{
	//m_material = m_physics->createMaterial(0.5f, 0.5f, 0.6f);
	m_material = m_physics->createMaterial(static_friction, dynamic_friction, restitution);
}

void PhysXManager::CreatePlane()
{
	/*plane creation*/
	PxRigidStatic* groundPlane = PxCreatePlane(*m_physics, PxPlane(0, 1, 0, 0), *m_material);
	m_scene->addActor(*groundPlane);
}
