/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Particles.h
 * Author: swl
 *
 * Created on April 15, 2016, 12:16 PM
 */

#ifndef PARTICLES_H
#define PARTICLES_H

#include <glm/glm.hpp>
#include <vector>
#if defined(__APPLE_CC__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <math.h>
#endif

typedef struct Particle
{
    glm::dvec3 p;
    glm::dvec3 newp;
    glm::dvec3 v;
    std::vector<Particle> neighbors;
    float lambda; //constraint force
    glm::dvec3 deltap;

} Particle;

class Particles {
public:
	  float kernel_size;
   	float radius;
   	float k;
   	int n;
   	float q;
   	int epsilon;
   	int nIters;
   	float rest_density;
    double dt;
    glm::dvec3 gravity;

    Particles();
    void render() const;
    float calcPoly(glm::dvec3 r, float h);
    void step(); // simulate one frame
    glm::dvec3 extForce(glm::dvec3 position);
    void findNeighbors(Particle &par);
    void calcLambda(Particle &par);
    void calcDeltaP(Particle &par);
    glm::dvec3 calcSpiky(glm::dvec3 pos);
    
    std::vector<Particle> particles;
};

#endif /* PARTICLES_H */

