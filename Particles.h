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
    std::vector<Particle *> neighbors;
    float lambda; //constraint force
    glm::dvec3 deltap;
    int cellId;

} Particle;

class Particles {
public:
    Particles (int cube_width, int cube_length, int cube_height);
    int cube_width_num_cells;
    int cube_length_num_cells;
    int cube_height_num_cells;
	  float kernel_size;
   	float radius;
   	float k;
   	int n;
   	float q;
   	double epsilon;
   	int nIters;
   	double rest_density;
    double dt;

    Particles();
    void render() const;
    void step(); // simulate one frame
    glm::dvec3 extForce(glm::dvec3 position);
    void createCellIdList(std::map<int, std::vector<Particle *>>  &cell_id_map);
    void findNeighbors(Particle &par, std::map<int, std::vector<Particle *>>  &cell_id_map);
    void calcLambda(Particle &par);
    int findCellId(glm::dvec3 position);
    void calcDeltaP(Particle &par);
    void calcVorticity(Particle &par);
    void calcViscosity(Particle &par);
    double calcPoly(glm::dvec3 r, float h);
    glm::dvec3 calcSpiky(glm::dvec3 pos, float h);
    
    std::vector<Particle> particles;
};

#endif /* PARTICLES_H */

