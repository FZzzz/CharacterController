#ifndef _RENDERABLEOBJECT_H_
#define _RENDERABLEOBJECT_H_

/*
	RenderableObject: One Mesh, One Texture, One Shader
	MAYBE DO: 
		1. Multiple Texture Support 
		2. Map<Texture, Shader>
*/

#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <memory>
#include "Shader.h"
#include "GLFunctions.h"

#define MAX_NUM_OF_INDICES 2147483647
/*
enum RENDER_ENUM
{
	RENDER_NORMAL,
	RENDER_INSTANCED
};
*/

class Mesh : public std::enable_shared_from_this<Mesh>
{
public:
	Mesh();
	Mesh(const Mesh&);
	~Mesh();

	bool Initialize(std::shared_ptr<Shader> shader);
	void Release();

	void Render();
	void Update();

	bool LoadFromFile(std::string path);
	bool LoadFromFileAssimp(std::string path);
	void UploadToGPU();
	void SetupGLBuffers();
	void ComputeSmoothNormal(unsigned int normal_type);
	

	//set functions
	void setPositions(std::vector<glm::vec3>& pos);
	void setNormals(std::vector<glm::vec3>& normals);
	void setIndices(std::vector<unsigned int>& indices);
	void setTexCoord(std::vector<glm::vec2>& texcoords);

	void setShader(std::shared_ptr<Shader> shader);
	//void setRenderOption(RENDER_ENUM render_option);

	// Problem: Dangerous!!!!!!!
	inline std::vector<unsigned int>& getIndices() { return m_indices; };
	inline std::vector<glm::vec3>& getPositions() { return m_positions; };

	inline const std::shared_ptr<Shader>& getShader() { return m_shader; };
	inline const GLuint& getShaderProgram() { return m_shader->getProgram(); };
	inline const size_t getNumberOfIndices() { return m_indices.size(); };
	inline const GLuint& getVAO() { return m_vao; };
	inline const GLuint& getEBO() { return m_ebo; };
	
	
	GLuint textureID;
	
protected: // for inheritance? why the fucking protected?
	
	std::vector<glm::vec3> m_positions;
	std::vector<glm::vec3> m_normals;
	std::vector<unsigned int> m_indices;
	std::vector<glm::vec3> m_colors;
	std::vector<glm::vec2> m_texcoord;

	std::shared_ptr<Shader> m_shader;
	GLuint m_vao; //vertex buffer
	GLuint m_vbo[3]; //pos, normal, text
	GLuint m_ebo; // element buffer -- indices

	std::vector<unsigned int> m_triangleNormalIndex;

	//bool m_animated;
	bool m_useSmoothNormal;

private:
	
	//RENDER_ENUM m_render_option;
	
	void ComputeVNormalByFace();
	void ComputeVNormalByEdge();

	void RegistToManager();

};

#endif
