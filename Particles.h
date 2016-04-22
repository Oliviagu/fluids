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
    float dt;
    glm::dvec3 gravity;

    Particles();
    void render() const;
    void step(); // simulate one frame
    glm::dvec3 extForce(glm::dvec3 position);
    void findNeighbors(Particles::Particle &par);
    void calcLambda(Particles::Particle &par);

    struct Particle
    {
        glm::dvec3 p;
        glm::dvec3 newp;
        glm::dvec3 v;
        std::vector<Particle> neighbors;
        float lambda; //constraint force

    };
    
    std::vector<Particle> particles;
};

#endif /* PARTICLES_H */

