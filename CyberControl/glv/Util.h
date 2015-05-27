
#ifndef __GL4Demo__Util__
#define __GL4Demo__Util__

#include <iostream>
#include <string>
#include <vector>


#include <GL\glew.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace glv{
	class Util{
	public:
		static GLuint loadCubeMap(std::vector<const char *> filenames);
		static GLuint loadTexture(const char * filename);
		static void* loadImage(const char * filename);

		static void computeTangentBasis(std::vector<glm::vec3>& vertices, std::vector<glm::vec2>& uvs, std::vector<glm::vec3>& normals, std::vector<glm::vec3>& tangents, std::vector<glm::vec3>& bitangents);

		static std::string& rtrim(std::string& str);

		static void print(glm::vec3 vector);
		static void print(glm::mat4 &matrix);
		static void print(glm::mat3 matrix);
	};
}

#endif /* defined(__GL4Demo__Util__) */
