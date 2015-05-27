
#pragma warning(disable: 4996)

#include "SimpleViewer.h"
#include "glv/Util.h"
#include "stdafx.h"

SimpleViewer::SimpleViewer(){
    
}

SimpleViewer::~SimpleViewer(){
    
}

void SimpleViewer::setVertexData(std::vector<glm::vec3> &points) {
    noOfPoints = static_cast<int>(points.size() );
    
    glGenBuffers( 1, &vertexBufferId );
	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * points.size(), &points[0], GL_STATIC_DRAW);
}

bool SimpleViewer::preLoop(){
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glShadeModel( GL_SMOOTH );
    glEnable( GL_DEPTH_TEST );
    glDepthFunc( GL_LESS );
    
    shader.init("./shaders/objectPass.vsh", "./shaders/objectPass.fsh" );

    float scale = 10.0f;
	glm::vec3 matScale = glm::vec3(-scale, scale, -scale);
	model = glm::scale(matScale) * glm::mat4(1.0);
    projection  = glm::perspective(70.0f, 4.0f/3.0f, 0.1f, 100.0f );
    view        = glm::lookAt( glm::vec3(0.0f, 0.0f, cameraDistance), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f) );
    
	Mat image =	imread("./texture.jpg");
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, windowWidth, windowHeight, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

    shader.bind();
    
	sphere.init(1.0f);
    
    return true;
}

//void SimpleViewer::drawScene(cv::Mat& image) {
//	glEnableVertexAttribArray(0);
//	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);
//	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
//
//	glEnableVertexAttribArray(1);
//	glBindBuffer(GL_ARRAY_BUFFER, uvBufferId);
//	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
//
//	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
//
//	glDisableVertexAttribArray(0);
//	glDisableVertexAttribArray(1);
//}

void SimpleViewer::loop( double last_time, double current_time ){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glm::mat4 rotated_view = view * arcball.createViewRotationMatrix();
    
    glm::vec3 translation_vec( rotated_view[3][0], rotated_view[3][1], rotated_view[3][2] );
    translation_vec = glm::normalize( translation_vec ) * cameraDistance;
    
    rotated_view[3][0] = translation_vec.x;
    rotated_view[3][1] = translation_vec.y;
    rotated_view[3][2] = translation_vec.z;
    
    glm::mat4 mvp = projection * rotated_view * model;
    shader.setUniform( "mvp", mvp );
    
    glEnableVertexAttribArray( 0 );
    glBindBuffer( GL_ARRAY_BUFFER, vertexBufferId );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0 );
    
    glDrawArrays( GL_POINTS, 0, noOfPoints);
}

bool SimpleViewer::postLoop() {
    shader.unbind();
    
    return true;
}