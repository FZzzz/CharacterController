#ifndef _GLFWAPP_H
#define _GLFWAPP_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <string>
#include <glm/glm.hpp>
#include <memory>
#include <chrono>
#include "imgui/imgui.h"
#include "GameObject.h"
#include "ResourceManager.h"
#include "Camera.h"
#include "Renderer.h"
#include "AnimCharacter.h"
#include "AssetImporter.h"
#include "GUIManager.h"
#include "PhysXManager.h"
#include "TestArticulation.h"

// forward declaration
class GameObject;
class ResourceManager;
class Renderer;
class AnimCharacter;
class AssetImporter;
class GUIManager;
class TestArticulation;

class GLFWApp
{
public:
	GLFWApp();
	~GLFWApp();
	
	bool Initialize(int width , int height , const std::string &title);
	void Run();
	void ClearBuffer();
	void ReleaseResources();
	void SwitchRenderMode();
	void CreateMonkeys(int num, OBJECT_FLAG_ENUM type);

	void NextStep();

	
	static GLFWApp* getInstance() {
		if (!appInstance)
			appInstance = new GLFWApp();
		return appInstance; 
	};
		
	/*virtual functions*/
	virtual float getElapsedTime();

	// setters

	// getters
	inline std::shared_ptr<ResourceManager>		getResourceManager() { return m_resource_manager; }
	inline std::shared_ptr<GUIManager>			getGUIManager() { return m_gui_manager; }
	inline const std::shared_ptr<Renderer>		getRenderer() { return m_renderer; }
	inline const std::shared_ptr<PhysXManager>	getPhysxManager() { return m_physx_manager; }
	inline bool									getAppStatus() { return m_app_status; };
	inline std::shared_ptr<TestArticulation>	getTestArticulation() { return m_test_articulation; };
	inline int									getWindowWidth() { return m_window_width; };
	inline int									getWindowHeight() { return m_window_height; };
	inline GLFWwindow*							getGLFWwindow() { return m_window; };


private:
	
	void Render();
	void Update();

	void SetUpImGui();
	void InitPhysics(bool interactive);

	GLFWwindow* m_window;
	static GLFWApp* appInstance;
	double m_previousTime , m_currentTime , m_deltaTime, m_accumulated_time, m_time_threshold;
	int m_frames_proccessed;
	char m_glsl_version[32];

	GLFWcursor* m_mouseCursors[ImGuiMouseCursor_COUNT];

	/*Importer*/
	std::shared_ptr<AssetImporter> m_importer;

	/*Manager*/
	std::shared_ptr<ResourceManager> m_resource_manager;
	std::shared_ptr<GUIManager> m_gui_manager;
	
	/*Resources*/
	std::shared_ptr<Camera> m_mainCamera;
	std::shared_ptr<Renderer> m_renderer;
	
	/* Timer */
	std::chrono::high_resolution_clock::time_point t0, t1, t2, t3, t4;

	/*PhysX*/
	std::shared_ptr<PhysXManager> m_physx_manager;

	/*Others*/
	std::shared_ptr<AnimCharacter> m_anim_character;
	
	/*Test*/
	std::shared_ptr<TestArticulation> m_test_articulation;
	
	/*GUIs*/
	//void Frame_Status_GUI();
	//void Object_Viewer_GUI();

	void SignalFail();

	bool	m_app_status;
	int		m_window_width;
	int		m_window_height;
	bool	m_stepping;
};

#endif
