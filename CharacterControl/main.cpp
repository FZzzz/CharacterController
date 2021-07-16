#include "GLFWApp.h"
#include <iostream>
#define _CRTDBG_MAP_ALLOC 
#include <crtdbg.h>  

// Replace _NORMAL_BLOCK with _CLIENT_BLOCK if you want the
// allocations to be of _CLIENT_BLOCK type

int main()
{
	std::string file_path;
	//Create GLFWApp
	std::unique_ptr<GLFWApp> demoApp(GLFWApp::getInstance());

	demoApp->Initialize(1600, 900, "GLEngine");

	/*
		Add text to App
	*/
	if (demoApp->getAppStatus())
	{
		demoApp->Run();
	}
	

	//release resources
	demoApp->ReleaseResources();

	demoApp.reset();
#ifdef _DEBUG
	//system("pause");
	//_CrtDumpMemoryLeaks();
#else 
	std::cout << "Program Stopped" << std::endl;
#endif
	return 0;
}
