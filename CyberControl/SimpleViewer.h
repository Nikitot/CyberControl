
#ifndef __SFM__SimpleViewer__
#define __SFM__SimpleViewer__

#include <iostream>
#include <vector>

#include "glv/Viewer.h"
#include "glv/Shader.h"
#include "glv/Sphere.h"

class SimpleViewer : public glv::Viewer {
public:
    SimpleViewer();
    ~SimpleViewer();
    
    void setVertexData( std::vector<glm::vec3>& points );
    
protected:
    glv::Shader shader;
    glv::Sphere sphere;
    GLuint vertexBufferId;
    int noOfPoints;
    
    bool preLoop();
    void loop( double last_time, double current_time );
    bool postLoop();
};

#endif /* defined(__SFM__SimpleViewer__) */
