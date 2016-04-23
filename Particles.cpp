/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Particles.cpp
 * Author: swl
 * 
 * Created on April 15, 2016, 12:16 PM
 */

#include "Particles.h"

Particles::Particles() 
{
    int nx = 10;
    int ny = 10;
    int nz = 10;
    float d = 0.1;

    kernel_size = d * 1.4;
    radius = d * 0.45;
    k = 0.001;
    n = 4;
    q = 0.2;
    epsilon = 104;
    nIters = 10;
    rest_density = 1 / (d * d *d);
    dt = 0.1;
    gravity = glm::dvec3(0, 0, -9.81);

    for(int x=0; x<nx; x++)
    {
        for(int y=0; y<ny; y++)
        {
            for(int z=0; z<nz; z++)
            {
                Particle par;
                par.p = glm::dvec3((x+0.5-nx*0.5)*d, (y+0.5)*d-1.0, (z+0.5-nz*0.5)*d);
                particles.push_back(par);
            }
        }
    }
}

void Particles::step() //simulation loop
{
    for(Particle &par : particles) {
        par.v = par.v + (extForce(par.p) * dt); //apply forces
        par.newp = par.p + (dt * par.v); //predict position
    }
    for(Particle &par : particles) {
        //findNeighbors will use par.newp and update par.neighbors
        findNeighbors(par);
    }
    int iter = 0;
    while (iter < nIters) {
        for(Particle &par : particles) {
            calcLambda(par); //lambda constraint force
        }
        for(Particle &par : particles) {
            //calculate deltap
            calcDeltaP(par);
            //collisions
        }
        for(Particle &par : particles) {
            //update new position
            par.newp += par.deltap;
        }
        iter++;
    }
    for(Particle &par : particles) {
        //update velocity
        //apply vorticity
        //update position
    }

}

glm::dvec3 Particles::extForce(glm::dvec3 position) 
//find forces and return extForce at position
{
    return glm::dvec3(0, 0, -9.81);
}

void Particles::findNeighbors(Particle &par)
//calculate neighbors and update par's neighbors
{

}

void Particles::calcLambda(Particle &par)
//calculate lambda and update par's lambda
{

}

void Particles::calcDeltaP(Particle &par)
//calculate lambda and update par's lambda
{

}

void Particles::render() const
{
    GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat mat_shininess[] = { 50.0 };
    GLfloat light_position[] = { 10.0, 10.0, 10.0, 0.0 };
    glShadeModel (GL_SMOOTH);
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glColor3f(0.2, 0.5, 0.8);
    glColorMaterial(GL_FRONT, GL_SPECULAR);
    glColor3f(0.9, 0.9, 0.9);
    glColorMaterial(GL_FRONT, GL_AMBIENT);
    glColor3f(0.2, 0.5, 0.8);
    
    for(const Particle &par : particles)
    {    
        
        glPushMatrix();
        glTranslatef(par.p.x, par.p.y, par.p.z);
        glutSolidSphere(0.05, 10, 10);
        glPopMatrix();
    }
    
    glPopAttrib();
}

