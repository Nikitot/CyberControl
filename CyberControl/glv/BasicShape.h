
#ifndef __OpenGLViewer__BasicShape__
#define __OpenGLViewer__BasicShape__

#include <iostream>
#include "GLVHeader.h"

using namespace std;

namespace glv {
    class BasicShape {
    protected:
        GLuint bufferIds[6];
        int noOfElements;
        
    public:
        BasicShape();
        ~BasicShape();
        virtual bool init( float size );
        virtual void draw();
    };
};
#endif /* defined(__OpenGLViewer__BasicShape__) */
