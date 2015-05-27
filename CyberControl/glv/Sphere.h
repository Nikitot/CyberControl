
#ifndef __OpenGLViewer__Sphere__
#define __OpenGLViewer__Sphere__

#include <iostream>
#include "BasicShape.h"

using namespace std;

namespace glv {
    class Sphere : public BasicShape {
        
    public:
        Sphere();
        bool init( unsigned int slices, unsigned int parallels, float size );
        bool init( float size );
    };
};


#endif /* defined(__OpenGLViewer__Sphere__) */
