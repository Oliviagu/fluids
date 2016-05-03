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

#include <string>
#include <glm/glm.hpp>
#include <vector>
#include <map> 
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

    bool collisions[6];
    bool collision; 
    std::vector<Particle *> neighbors;
    float lambda; //constraint force
    glm::dvec3 deltap;
    int cellId[3];

} Particle;

class Particles {
public:
    Particles(float most_bottom[3], float cube_width, float cube_length, float cube_height);
	  float kernel_size;
   	float radius;
   	float k;
   	int n;
   	float q;
   	double epsilon;
   	int nIters;
   	double rest_density;
    double dt;
    float bottom_pt[3];
    float box_width;
    float box_length;
    float box_height;

    void render() const;
    void step(); // simulate one frame
    glm::dvec3 extForce(glm::dvec3 position);
    void findCellId(Particle &par);
    std::string createStringCellId(int (&pos) [3]);
    void createCellIdList(std::map<std::string, std::vector<Particle *>>  &cell_id_map);
    void findNeighbors(Particle &par, std::map<std::string, std::vector<Particle *>>  &cell_id_map);
    void calcLambda(Particle &par);
    void calcDeltaP(Particle &par);
    double dvec3_length(glm::dvec3 p);
    void calcCollision(Particle &par);
    void calcVorticity(Particle &par);
    void calcViscosity(Particle &par);
    double calcPoly(glm::dvec3 r, float h);
    glm::dvec3 calcSpiky(glm::dvec3 pos, float h);
    
    std::vector<Particle> particles;
};

#endif /* PARTICLES_H */

