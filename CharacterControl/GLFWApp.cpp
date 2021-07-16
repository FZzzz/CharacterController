#include "GLFWApp.h"
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <glm/gtc/matrix_transform.hpp>
#include "imgui/imgui_impl_glfw_gl3.h"
#ifdef _WIN32
#undef APIENTRY
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#include <GLFW/glfw3native.h>
#include <chrono>
#endif

#include "GameObject.h"
#include "Cube.h"
#include "Floor.h"
#include "AnimCharacter.h"
#include "SimCharacter.h"
//#include "TestArticulation.h"

void Error_callback(int error, const char* description);
void Key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void CursorPos_callback(GLFWwindow* window, double xpos, double ypos);
void MouseButton_callback(GLFWwindow* window, int button, int action, int mod);
void Scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void Frame_Status_GUI();
void Object_Viewer_GUI();
void Animated_Character_GUI();
void Mouse_Status_GUI();
void Camera_Controller_GUI();
void TestArticulation_Link_GUI();

GLFWApp* GLFWApp::appInstance;

GLFWApp::GLFWApp() :
	m_frames_proccessed(0),
	m_previousTime(0),
	m_currentTime(0),
	m_accumulated_time(0),
	m_time_threshold(0.1),
	m_glsl_version("#version 150"),
	m_app_status(true),
	m_resource_manager(nullptr),
	m_gui_manager(nullptr),
	m_mainCamera(nullptr),
	m_renderer(nullptr),
	m_test_articulation(nullptr),
	m_stepping(false)
{
}

GLFWApp::~GLFWApp()
{
	m_resource_manager.reset();
	m_mainCamera.reset();
	m_renderer.reset();
}

bool GLFWApp::Initialize(int width , int height , const std::string &title)
{
	if (!glfwInit())
	{
		return false;
	}

	glfwSetErrorCallback(Error_callback);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_SAMPLES, 4);

	m_window = glfwCreateWindow(width, height, title.c_str() , NULL , NULL);
	m_window_width = width;
	m_window_height = height;
	
	if (!m_window)
	{
		std::cout << "Window Creation Failed!" << std::endl;
		return false;
	}

	glfwSetWindowPos(m_window, 100, 100);

	glfwMakeContextCurrent(m_window);
	glfwSetKeyCallback(m_window, Key_callback);
	glfwSetInputMode(m_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	glfwSetCursorPosCallback(m_window, CursorPos_callback);
	glfwSetMouseButtonCallback(m_window, MouseButton_callback);
	glfwSetScrollCallback(m_window, Scroll_callback);
	glfwSwapInterval(0);

	//initialize glew
	glewExperimental = true;
	GLenum status = glewInit();
	if (status != GLEW_OK)
	{
		std::cout << "GLEW INIT Failed! " << std::endl;
		return false;
	}
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_BLEND);
	//glEnable(GL_CULL_FACE);
	//glCullFace(GL_BACK);
	//glFrontFace(GL_CCW);
	//glEnable(GL_CULL_FACE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glCullFace(GL_BACK);


	/*PhysX Initialization*/
	InitPhysics(true);
	
	/*
		ResourceManager Creation
	*/
	m_resource_manager = std::make_shared<ResourceManager>();
	
	const float f_width = static_cast<float>(width);
	const float f_height = static_cast<float>(height);
	
	// Shader
	std::shared_ptr<Shader> shader = std::make_shared<Shader>();
	shader->SetupShader("resources/shader/shadow_mapping_vs_adv.glsl",
		"resources/shader/shadow_mapping_fs.glsl");

	std::shared_ptr<Shader> debug_shader = std::make_shared<Shader>();
	debug_shader->SetupShader("resources/shader/debug_coordinate_vs.glsl",
		"resources/shader/debug_coordinate_fs.glsl");

	auto mat_uniform = glGetUniformBlockIndex(shader->getProgram(), "Matrices");
	GLuint ubo;
	glGenBuffers(1, &ubo);
	glBindBuffer(GL_UNIFORM_BUFFER, ubo);
	glBufferData(GL_UNIFORM_BUFFER, 3 * sizeof(glm::mat4), NULL, GL_STATIC_DRAW);
	glBindBuffer(GL_UNIFORM_BUFFER, 0);
	// define the range of the buffer that links to a uniform binding point
	glBindBufferRange(GL_UNIFORM_BUFFER, 0, ubo, 0, 3 * sizeof(glm::mat4));

	// Camera Setting
	{
		CameraDesc camera_desc;
		camera_desc.fov = 45.0f;
		camera_desc.screen_width = f_width;
		camera_desc.screen_height = f_height;
		camera_desc.near_plane = 0.001f;
		camera_desc.far_plane = 1000.0f;
		camera_desc.camera_front = glm::vec3(0, 0, -1.0f);
		camera_desc.camera_up = glm::vec3(0, 1.f, 0);
		camera_desc.position = glm::vec3(0.0f, 20.0f, 60.0f);
		camera_desc.move_speed = 1.f;
		camera_desc.target_position = glm::vec3(0, camera_desc.position.y, 0);
		camera_desc.projection = glm::perspective(camera_desc.fov, f_width / f_height, 0.1f, 1000.0f);
		camera_desc.lookAt = glm::lookAt(camera_desc.position, camera_desc.target_position, glm::vec3(0.0f, 1.0f, 0.0f));
		camera_desc.ubo = ubo;

		//Main Camera setting
		m_mainCamera = std::make_shared<Camera>(camera_desc);
		m_resource_manager->SetMainCamera(m_mainCamera);

		//Renderer setting
		m_renderer = std::make_shared<Renderer>(m_resource_manager);
		m_renderer->Initialize(m_mainCamera, width, height);
		m_renderer->SetMainCamera(m_mainCamera);

		glBindBuffer(GL_UNIFORM_BUFFER, ubo);
		glBufferSubData(
			GL_UNIFORM_BUFFER,
			0,
			sizeof(glm::mat4),
			glm::value_ptr(m_mainCamera->m_projection));
		glBufferSubData(
			GL_UNIFORM_BUFFER,
			2 * sizeof(glm::mat4),
			sizeof(glm::mat4),
			glm::value_ptr(m_renderer->getLightMat()));
		glBindBuffer(GL_UNIFORM_BUFFER, 0);

		// Initialize asset importer
		m_importer = std::make_shared<AssetImporter>();
	}
	//Objects setting
	//std::shared_ptr<GameObject> obj = std::make_shared<GameObject>();	
	{
		//default mesh
		auto load_status = IMPORT_STATUS::IMPORT_FAILED;
		auto mesh = m_importer->LoadMesh("resources/models/monkey.obj", shader, load_status);
		// Original
		/* 
		bool load_status = false;
		std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
		mesh->Initialize(shader);
		load_status = mesh->LoadFromFileAssimp("resources/models/capsule.obj");
		*/
		if (load_status == IMPORT_STATUS::IMPORT_FAILED)
		{
			std::cout << "Load Failed\n";
			SignalFail();
		}
	}

	// Joint
	{
		auto load_status = IMPORT_STATUS::IMPORT_FAILED;
		auto mesh = m_importer->LoadMesh("resources/models/coordinate.obj", debug_shader, load_status);
		mesh->setName("Coordinate");
		if (load_status == IMPORT_STATUS::IMPORT_FAILED)
		{
			std::cout << "Load Failed\n";
			SignalFail();
		}
	}

	{
		auto load_status = IMPORT_STATUS::IMPORT_FAILED;
		auto mesh = m_importer->LoadMesh("resources/models/capsule_mesh.obj", shader, load_status);
		mesh->setName("AnimCapsule");
		if (load_status == IMPORT_STATUS::IMPORT_FAILED)
		{
			std::cout << "Load Failed\n";
			SignalFail();
		}
	}

	// PhysX box geometry Primitive
	{
		auto load_status = IMPORT_STATUS::IMPORT_FAILED;
		auto mesh = m_importer->LoadMesh("resources/models/box_mesh.obj", shader, load_status);
		mesh->setName("Box");
		if (load_status == IMPORT_STATUS::IMPORT_FAILED)
		{
			std::cout << "Load Failed\n";
			SignalFail();
		}
	}

	// PhysX capsule geometry Primitive
	{
		auto load_status = IMPORT_STATUS::IMPORT_FAILED;
		auto mesh = m_importer->LoadMesh("resources/models/physx_capsule.obj", shader, load_status);
		mesh->setName("Capsule");
		if (load_status == IMPORT_STATUS::IMPORT_FAILED)
		{
			std::cout << "Load Failed\n";
			SignalFail();
		}
	}

	// PhysX sphere geometry Primitive
	{
		auto load_status = IMPORT_STATUS::IMPORT_FAILED;
		auto mesh = m_importer->LoadMesh("resources/models/sphere_mesh.obj", shader, load_status);
		mesh->setName("Sphere");
		if (load_status == IMPORT_STATUS::IMPORT_FAILED)
		{
			std::cout << "Load Failed\n";
			SignalFail();
		}
	}


	
	// Load Animation
	{
		auto load_status = IMPORT_STATUS::IMPORT_FAILED;
		auto anim_character = m_importer->LoadBVH("resources/mocap/02_01.bvh", load_status);
		if (load_status == IMPORT_STATUS::IMPORT_SUCCESS)
		{
			anim_character->Initialize(glm::vec3(10, -12, 0));
			anim_character->setName("Bob");
			m_resource_manager->AddAnimCharacter(anim_character);
			m_anim_character = anim_character;
		}		
	}
	  
	// SimCharacter
	//for(int i=0; i<4;++i)
	{
		auto sim_character = std::make_shared<SimCharacter>();
		sim_character->Initialize(m_anim_character);
		sim_character->InitializePose(0);
		m_resource_manager->AddSimCharacter(sim_character);

		//sim_character->InitializePose();
	}
	
	/*
	{
		m_test_articulation = std::make_shared<TestArticulation>();
		m_test_articulation->Initialize();
		m_test_articulation->setDriveStiffness(300.f);
		m_test_articulation->setDriveDamping(0.1f);
		m_test_articulation->setForceLimit(PX_MAX_F32);
		//test_articulation->CreatePhysXJointsCustomized();
		//m_test_articulation->CreatePhysXJointsArticulation();
		m_test_articulation->CreateArticulationSimLink();
		m_test_articulation->ComputeDofStarts();
	}
	*/
		
	//Terrain Initilization
	std::shared_ptr<Floor> plane_terrain = std::make_shared<Floor>();
	plane_terrain->Initialize(glm::vec3(0, 0, 0), shader);
	m_resource_manager->AddGameObject(std::move(static_cast<std::shared_ptr<GameObject>>(plane_terrain)));
	
	//CreateMonkeys(10000, OBJECT_FLAG_ENUM::OBJECT_STATIC);
	
	/*
		End of resources setting
	*/

	/*
		UI settings
	*/
	m_gui_manager = std::make_shared<GUIManager>();
	m_gui_manager->Initialize(m_window);
	//m_gui_manager->AddGUIFunction(std::function<void()>(Frame_Status_GUI));
	m_gui_manager->AddGUIFunction(std::bind(Frame_Status_GUI));
	m_gui_manager->AddGUIFunction(std::bind(Object_Viewer_GUI));
	m_gui_manager->AddGUIFunction(std::bind(Animated_Character_GUI));
	m_gui_manager->AddGUIFunction(std::bind(TestArticulation_Link_GUI));
	m_gui_manager->AddGUIFunction(std::bind(Mouse_Status_GUI));
	m_gui_manager->AddGUIFunction(std::bind(Camera_Controller_GUI));
	//SetUpImGui();
	/*
	std::cout << "Setup finished\n";
	std::cout << "Number of Shader: " << m_resManager->getShaderList().size() << std::endl;
	/*
	std::cout << "Float size: " << sizeof(float) << "\n";
	std::cout << "Vec3 size: " << sizeof(glm::vec3) << "\n";
	std::cout << "GameObject size: " << sizeof(GameObject) << "\n";
	std::cout << "Mesh size: " << sizeof(Mesh) << "\n";
	std::cout << "Transform size: " << sizeof(Transform) << "\n";
	*/

	/*
	m_resManager->ArrangeStaticObjects();
	*/
	return true;
}

void GLFWApp::Run()
{

	while (!glfwWindowShouldClose(m_window))
	{
		t0 = std::chrono::high_resolution_clock::now();
		ImGui_ImplGlfwGL3_NewFrame();

		//Frame_Status_GUI();
		//Object_Viewer_GUI();
		
		ClearBuffer();
		t1 = std::chrono::high_resolution_clock::now();
		Update();
		t2 = std::chrono::high_resolution_clock::now();
		Render();
		t3 = std::chrono::high_resolution_clock::now();

		glfwSwapBuffers(m_window);
		glfwPollEvents();
		t4 = std::chrono::high_resolution_clock::now();;
	}
	
	ImGui_ImplGlfwGL3_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();
}

void GLFWApp::ClearBuffer()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.35f , 0.35f , 0.35f , 0);
}

void GLFWApp::ReleaseResources()
{
	m_resource_manager->ShutDown();
	m_physx_manager->CleanUpPhysics();
}

void GLFWApp::SwitchRenderMode()
{
	m_renderer->renderQuad = !m_renderer->renderQuad;
}

void GLFWApp::CreateMonkeys(int num, OBJECT_FLAG_ENUM type)
{
	auto mesh_list = m_resource_manager->getMeshes();
	//static int name_count = 0;
	//static std::string name_pattern = "Object";
	//test add gameobject
	for (int i = 0; i < num; ++i)
	{

		float x = static_cast<float>(rand() % 10);
		float y = static_cast<float>(rand() % 10);
		float z = static_cast<float>(rand() % 10);

		auto obj = std::make_shared<GameObject>();
		obj->Initialize(glm::vec3(x, y, z));
		obj->setObjectType(type);
		obj->setMesh(mesh_list[1]);
		//obj->m_name = name_pattern + std::to_string(name_count);

		//auto mesh_list = m_resManager->getMeshList();
		//attach mesh onto game object
		//obj->setMesh(mesh_list[1]);

		obj->m_transform.m_translation = glm::vec3(x, y, z);
		if (type == OBJECT_FLAG_ENUM::OBJECT_STATIC)
			m_resource_manager->AddStaticObject(std::move(obj));
		else
			m_resource_manager->AddGameObject(std::move(obj));
		//name_count++;
	}
}

void GLFWApp::SignalFail()
{
	m_app_status = false;
}

float GLFWApp::getElapsedTime()
{
	return static_cast<float>(glfwGetTime());
}

void GLFWApp::NextStep()
{
	m_stepping = true;
}

void Error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

void Key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	static GLFWApp* const instance = GLFWApp::getInstance();
	static auto res_manager = instance->getResourceManager();
	static auto camera = res_manager->getMainCamera();
	if (action == GLFW_RELEASE)
	{
		switch (key)
		{
		case GLFW_KEY_ESCAPE:
		{
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		}

		case GLFW_KEY_H:
		{
			instance->SwitchRenderMode();
			break;
		}

		case GLFW_KEY_G:
		{
			/*
			static int name_count = 0;
			static std::string name_pattern = "Object";
			//test add gameobject
			for (int i = 0; i < 10000; ++i)
			{
				
				float x = static_cast<float>(rand() % 10);
				float y = static_cast<float>(rand() % 10);
				float z = static_cast<float>(rand() % 10);
				
				std::shared_ptr<GameObject> obj = std::make_shared<Cube>();

				obj->Initialize(glm::vec3(x,y,z));
				obj->setName(name_pattern + std::to_string(name_count));
				obj->m_transform.m_translation = glm::vec3(x, y, z);

				instance->getResourceManager()->AddGameObject(std::move(obj));
				name_count++;
			}
			*/
			break;
		}

		case GLFW_KEY_SPACE:
		{
			auto resource_manager = GLFWApp::getInstance()->getResourceManager();
			auto anim_characters = resource_manager->getAnimCharacters();
			for (auto it = anim_characters.cbegin();
				it != anim_characters.cend();
				++it)
			{
				auto character = *it;
				character->getAnimation()->Pause();
			}
			break;
		}

		case GLFW_KEY_F:
		{
			//GLFWApp::getInstance()->NextStep();
			//auto resource_manager = GLFWApp::getInstance()->getResourceManager();
			//auto anim 
			break;
		}
		
		case GLFW_KEY_BACKSPACE:
		{
			for (int i = 0; i < 1; ++i)
			{
				//test memory release
				instance->getResourceManager()->RemoveObjectFromLast();
			}
			break;
		}

		}
	}
	else if (action == GLFW_PRESS || action == GLFW_REPEAT)
	{
		switch (key)
		{
		case GLFW_KEY_W:
		{
			//camera->Zoom(-0.25f);
			camera->Move(camera->m_front);
			break;
		}

		case GLFW_KEY_S:
		{
			camera->Move(-1.f * camera->m_front);
			break;
		}
		case GLFW_KEY_Q:
		{
			//camera->Zoom(-0.25f);
			camera->Move(-1.f * camera->m_up);
			break;
		}

		case GLFW_KEY_E:
		{
			camera->Move(camera->m_up);
			break;
		}
		case GLFW_KEY_A:
		{
			//camera->Rotate(0.2f / 30.0f);
			glm::vec3 right = glm::normalize(glm::cross(camera->m_front, camera->m_up));
			camera->Move(-1.f*right);
			break;
		}

		case GLFW_KEY_D:
		{
			//camera->Rotate(-0.2f / 30.0f);
			glm::vec3 right = glm::normalize(glm::cross(camera->m_front, camera->m_up));
			camera->Move(right);
			break;
		}

		case GLFW_KEY_X:
		{
			auto resource_manager = GLFWApp::getInstance()->getResourceManager();
			auto anim_characters = resource_manager->getAnimCharacters();
			for (auto it = anim_characters.cbegin();
				it != anim_characters.cend();
				++it)
			{
				auto character = *it;
				character->getAnimation()->setCurrentFrame(character->getAnimation()->getCurrentFrameTime() + 1);
			}
			break;
		}

		case GLFW_KEY_Z:
		{
			auto resource_manager = GLFWApp::getInstance()->getResourceManager();
			auto anim_characters = resource_manager->getAnimCharacters();
			for (auto it = anim_characters.cbegin();
				it != anim_characters.cend();
				++it)
			{
				auto character = *it;
				character->getAnimation()->setCurrentFrame(character->getAnimation()->getCurrentFrameTime() - 1);
			}
			break;
		}
		}
	}

}

void CursorPos_callback(GLFWwindow * window, double xpos, double ypos)
{
	glfwGetCursorPos(window, &xpos, &ypos);

	auto camera = GLFWApp::getInstance()->getResourceManager()->getMainCamera();
	if (camera->m_ctrl_hold)
	{
		camera->m_ctrl_cur_pos = glm::vec2(xpos, ypos);
		/* Rotate Camera */
		glm::vec2 offset = camera->m_ctrl_cur_pos - camera->m_ctrl_prev_pos;
		// Use this offset to rotate camera's (yaw, pitch)
		float rotate_speed = 0.001f;

		camera->Rotate(rotate_speed * glm::vec3(offset.y, offset.x, 0));
		camera->m_ctrl_prev_pos = camera->m_ctrl_cur_pos;
	}


}

void MouseButton_callback(GLFWwindow * window, int button, int action, int mod)
{
	auto camera = GLFWApp::getInstance()->getResourceManager()->getMainCamera();
	if (camera == nullptr)
		return;

	double xpos = 0, ypos = 0;
	glfwGetCursorPos(window, &xpos, &ypos);
	std::cout << action << std::endl;
	if (action == GLFW_PRESS)
	{
		switch (button)
		{
		case GLFW_MOUSE_BUTTON_RIGHT:
		{
			camera->m_ctrl_hold = true;
			camera->m_ctrl_prev_pos = glm::vec2(xpos, ypos);
			break;
		}
		}
	}
	else if (action == GLFW_RELEASE)
	{
		switch (button)
		{
		case GLFW_MOUSE_BUTTON_RIGHT:
		{
			camera->m_ctrl_hold = false;
			break;
		}
		}
	}
}

void Scroll_callback(GLFWwindow * window, double xoffset, double yoffset)
{
	auto camera = GLFWApp::getInstance()->getResourceManager()->getMainCamera();
	float change = -1.f * yoffset;//
	camera->Zoom(change);
}

void GLFWApp::Render()
{
	m_renderer->Render();
	//GUI rendering
	m_gui_manager->Render();
	//ImGui::Render();
	//ImGui_ImplGlfwGL3_RenderDrawData(ImGui::GetDrawData());
}

void GLFWApp::Update()
{
	const float dt = m_anim_character->getAnimation()->getFrameTime();
	m_physx_manager->StepPhysics(dt);
	m_previousTime = m_currentTime;
	m_currentTime = static_cast<float>(glfwGetTime());

	m_accumulated_time = 
		(m_accumulated_time >= m_time_threshold)
		? 0.f
		: m_accumulated_time + m_currentTime - m_previousTime;

	/* AnimCharacter */
	auto anim_characters = m_resource_manager->getAnimCharacters();
	if (anim_characters.size() > 0 && m_accumulated_time > m_time_threshold)
	{
		for (auto it = anim_characters.cbegin(); it != anim_characters.cend(); ++it)
		{
			(*it)->Update();
		}
	}
	

	/* SimCharacter */
	auto sim_characters = m_resource_manager->getSimCharacters();
	for (auto it = sim_characters.cbegin(); it != sim_characters.cend(); ++it)
	{
		if(*it != nullptr)
			(*it)->Update(dt);
	}

	/*Test*/
	if (m_test_articulation)
		m_test_articulation->Update();

	/* GameObject */
	for (auto it = m_resource_manager->getObjects().begin(); 
		it != m_resource_manager->getObjects().end(); ++it)
	{
		(*it)->Update();
	}

	m_mainCamera->Update();
	// GUI update
	m_gui_manager->Update();
}

void GLFWApp::SetUpImGui()
{
	//Set up imgui binding
	ImGui::CreateContext();
	ImGuiIO &io = ImGui::GetIO(); (void)io;
	ImGui_ImplGlfwGL3_Init(m_window, false);
	ImGui::StyleColorsDark();
}

void GLFWApp::InitPhysics(bool interactive)
{
	if (m_physx_manager == nullptr)
		m_physx_manager = std::make_shared<PhysXManager>();
	
	m_physx_manager->InitPhysics(interactive);
	float static_friction = 0.8f;
	float dynamic_friction = 0.5f;
	float restitution = 0.2f;
	m_physx_manager->CreateMaterial(static_friction, dynamic_friction, restitution);
	m_physx_manager->CreatePlane();
}

void Frame_Status_GUI()
{
	auto resource_manager = GLFWApp::getInstance()->getResourceManager();
	auto camera = resource_manager->getMainCamera();
	// If we don't call ImGui::Begin()/ImGui::End() the widgets automatically appears in a window called "Debug".
	ImGui::Begin("Frame Status");
	static float f = 0.0f;
	static int counter = 0;
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::NewLine();
	ImGui::Text("Object Array Size: %u", resource_manager->getObjects().size());
	ImGui::Text("Mesh Array Size: %u", resource_manager->getMeshes().size());
	ImGui::Text("Shader Array Size: %u", resource_manager->getShaders().size());
	//ImGui::Text("IMGUI time: %.2lf ms", (t0-t4).count()/1000000.0);
	//ImGui::Text("Update time: %.2lf ms", (t2-t1).count()/1000000.0);
	//ImGui::Text("Render time: %.2lf ms", (t3-t2).count()/1000000.0);
	//ImGui::Text("SWAP & PollEvent time: %.2lf ms", (t4-t3).count()/1000000.0);

	//ImGui::Text("Camera Position: %.2f, %.2f, %.2f", camera->m_position.x, camera->m_position.y, camera->m_position.z);

	ImGui::End();
}

void Object_Viewer_GUI()
{
	ImGui::Begin("Object List");
	auto resource_manager = GLFWApp::getInstance()->getResourceManager();
	
	static int update_count = 0;
	static std::string name_list = "";
	if (update_count % 60 == 0)
	{
		update_count = 0;
		name_list.clear();
		for (auto it = resource_manager->getObjects().begin();
			it != resource_manager->getObjects().end(); ++it)
		{
			name_list += (*it)->getName() + "\n";
		}
	}
	ImGui::Text("%s", name_list.data());
	ImGui::NewLine();
	
	update_count++;
	ImGui::End();
}

void Animated_Character_GUI()
{
	ImGui::Begin("Animated Character Panel");

	auto resource_manager = GLFWApp::getInstance()->getResourceManager();
	auto anim_characters = resource_manager->getAnimCharacters();
	
	enum Channel
	{
		Channel_RX,
		Channel_RY,
		Channel_RZ
	};

	for (auto it = anim_characters.cbegin();
		it != anim_characters.cend();
		++it)
	{
		auto character = *it;
		auto anim = character->getAnimation();
		auto& root_transform = character->getRoot()->m_transform;
		if (ImGui::CollapsingHeader(character->getName().c_str()))
		{
			float v[3] = { root_transform.m_translation.x, root_transform.m_translation.y, root_transform.m_translation.z };
			ImGui::InputFloat3("Root Position", v, 3, ImGuiInputTextFlags_ReadOnly);

			int current_frame_number = static_cast<int>(anim->getCurrentFrameTime());
			ImGui::SliderInt("Frame number", &current_frame_number, 0, anim->getKeyFrameSize() - 1);
			anim->setCurrentFrame(static_cast<size_t>(current_frame_number));

			bool repeat = anim->isRepeat();
			ImGui::Checkbox("Repeated", &repeat);
			anim->setRepeat(repeat);

			const float frame_time = anim->getFrameTime();
			ImGui::Text("Per frame time: %.4fs", frame_time);
		}
	}
	
	ImGui::End();
}

void Mouse_Status_GUI()
{
	GLFWwindow* window = GLFWApp::getInstance()->getGLFWwindow();

	double xpos = 0, ypos = 0;
	glfwGetCursorPos(window, &xpos, &ypos);
	ImGui::Begin("Cursor Position");
	//ImGui::SetWindowPos("Cursor Position", ImVec2(10.f, 10.f), ImGuiCond_::ImGuiCond_Always);
	float v[2] = { static_cast<float>(xpos), static_cast<float>(ypos) };
	ImGui::InputFloat2("Cursor Position", v, 3, ImGuiInputTextFlags_ReadOnly);
	ImGui::End();
}

void Camera_Controller_GUI()
{
	auto camera = GLFWApp::getInstance()->getResourceManager()->getMainCamera();
	ImGui::Begin("Camera Controller");
	ImGui::SliderFloat("Frame number", &(camera->m_move_speed), 0.f, 100.f);
	ImGui::End();
}

void TestArticulation_Link_GUI()
{
	auto resource_manager = GLFWApp::getInstance()->getResourceManager();
	auto test_articulation = GLFWApp::getInstance()->getTestArticulation();
	if (!test_articulation)
		return;

	ImGui::Begin("TestArticulation");
	ImGui::Text("Target angle (Swing2): %.3f degree", test_articulation->getTargetAngle() * 180.f / MATH_UTIL_PI);
	
	auto joint_1_cache = test_articulation->getCachedPosition(1);
	for (size_t i = 0; i < joint_1_cache.size(); ++i)
	{
		ImGui::Text("Cache joint 1 position: %.5f", joint_1_cache[i]);
	}
	ImGui::NewLine();

	auto joint_2_cache = test_articulation->getCachedPosition(2);
	for (size_t i = 0; i < joint_2_cache.size(); ++i)
	{
		ImGui::Text("Cache joint 2 position: %.5f", joint_2_cache[i]);
	}
	ImGui::NewLine();
	
	ImGui::End();
}

