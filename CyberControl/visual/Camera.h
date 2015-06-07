#include "GLCommon.h"
#include "OpenGL.h"
#include "math/math3d.h"
#include "math/mathgl.h"

struct Camera
{
	vec3 position;
	vec3 rotation;

	mat4 projection;
};

void CameraCreate(Camera &camera, float x, float y, float z);
void CameraPerspective(Camera &camera, float fov, float aspect, float zNear, float zFar);
void CameraRotate(Camera &camera, float x, float y, float z);
void CameraMove(Camera &camera, float x, float y, float z);
void CameraSetup(GLuint program, const Camera &camera, const mat4 &model);