#ifndef __OpenGLViewer__Cube__
#define __OpenGLViewer__Cube__

#include <iostream>
#include "BasicShape.h"

using namespace std;

namespace glv {
    class Cube : public BasicShape {
        
        public:
            Cube();
            bool init( float size );
    };
};

#endif /* defined(__OpenGLViewer__Cube__) */
