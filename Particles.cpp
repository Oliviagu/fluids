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
 
Particles::Particles(int cube_width, int cube_length, int cube_height) 
{
    int nx = 5;
    int ny = 5;
    int nz = 5;
    float d = 0.1;

    kernel_size = d * 1.4;
    //TODO weilun doesn't use this radius when rendering
    //radius = d * 0.45;
    radius = 0.05;

    cube_width_num_cells = cube_width / (radius * 2);
    cube_length_num_cells = cube_length / (radius * 2);
    cube_height_num_cells = cube_height / (radius * 2);

    k = 0.001;
    n = 4;
    q = 0.2;
    epsilon = 104;
    nIters = 10;
    rest_density = 1 / (d * d *d);
    dt = 0.1;

    for(int x=0; x<nx; x++)
    {
        for(int y=0; y<ny; y++)
        {
            for(int z=0; z<nz; z++)
            {
                Particle par;
                par.p = glm::dvec3((x+0.5-nx*0.5)*d, (y+0.5)*d-1.0, (z+0.5-nz*0.5)*d);
                par.newp = par.p;
                par.v = glm::dvec3(0, 0, 0);
                par.neighbors = {&par};
                par.lambda = 0;
                par.deltap = glm::dvec3(0, 0, 0);
                par.cellId = -1;
                particles.push_back(par);
            }
        }
    }
}

//Kernel Functions
double Particles::calcPoly(glm::dvec3 r, float h) 
{
    double temp = pow(h, 2.0) - pow(r.length(), 2.0);
    return (315 / (64 * M_PI * pow(h, 9.0))) * pow(temp, 3.0);
}

void Particles::step() //simulation loop
{
    printf("STEP\n");
    for(Particle &par : particles) {
        par.v = par.v + (extForce(par.p) * dt); //apply forces
        par.newp = par.p + (dt * par.v); //predict position
    }
    std::map<int, std::vector<Particle *>>  cell_id_list; 
    createCellIdList(cell_id_list);
    for(Particle &par : particles) {
        //findNeighbors will use par.newp and update par.neighbors
        findNeighbors(par, cell_id_list);
    }
    int iter = 0;
    while (iter < nIters) {
        for(Particle &par : particles) {
            calcLambda(par); //lambda constraint force
        }
        for(Particle &par : particles) {
            //calculate deltap
            calcDeltaP(par);
            par.newp += par.deltap;
            //collisions
            calcCollision(par);

        }
        iter++;
    }
    for(Particle &par : particles) {
        //update velocity
        par.v = (1.0 / dt) * (par.newp - par.p);
        //apply vorticity
        calcVorticity(par);
        //apply viscosity
        calcViscosity(par);
        //update position
        par.p = par.newp;
    }

}

glm::dvec3 Particles::extForce(glm::dvec3 position) 
//find forces and return extForce at position
{
    return glm::dvec3(0, 0, -9.81);
}

void Particles::createCellIdList(std::map<int, std::vector<Particle *>>  &cell_id_map) {  
    for (Particle &par : particles) {
        printf("create cell Id par.newp x : %f , y : %f, z : %f\n", par.newp.x, par.newp.y, par.newp.z);
        glm::dvec3 par_cell = glm::dvec3((int) (par.newp.x / (2 * radius)), (int) (par.newp.y / (2 * radius)), (int) (par.newp.z / (2 * radius)));
        printf("par_cell x : %f , y : %f , z : %f\n", par_cell.x, par_cell.y, par_cell.z);
        par.cellId = findCellId(par_cell);
        printf("par.cellId %d\n", par.cellId);
        if (cell_id_map.find(par.cellId) != cell_id_map.end()) {
            cell_id_map.insert(std::pair<int, std::vector<Particle *>>(par.cellId, std::vector<Particle *> ()));
        }
        cell_id_map[par.cellId].push_back(&par);
    }
}

int Particles::findCellId(glm::dvec3 position)
{
    int x = position.x + (cube_width_num_cells / 2);
    int y = position.y + (cube_height_num_cells / 2);
    int z = position.z + (cube_length_num_cells / 2);
    printf("cube dimension width %d height %d length %d\n", cube_width_num_cells, cube_height_num_cells, cube_length_num_cells);
    printf("findcellId x %d y %d z %d \n", x, y, z);
    if (x < 0 or x > cube_width_num_cells) {
      return -1;
    }
    if (y < 0 or y > cube_height_num_cells) {
      return -1;
    }
    if (z < 0 or z > cube_length_num_cells) {
      return -1;
    }
    return cube_width_num_cells * y + x + z * (cube_width_num_cells * cube_height_num_cells);
}


void Particles::findNeighbors(Particle &par, std::map<int, std::vector<Particle *>>  &cell_id_map)
//calculate neighbors and update par's neighbors
{
  //par.neighbor already has self in the initialization
  for (int x = -1; x <= 1; x += 1) {
    for (int y = -1; y <= 1; y += 1) {
      for (int z = -1; z <= 1; z += 1) {
        glm::dvec3 par_cell = glm::dvec3((int) (par.newp.x / (2 * radius)), (int) (par.newp.y / (2 * radius)), (int) (par.newp.z / (2 * radius)));
        int neighbor = findCellId(glm::dvec3(par_cell.x + x, par_cell.y + y, par_cell.z + z));
        if (neighbor != -1) {
          if (cell_id_map.find(neighbor) != cell_id_map.end()) {
            par.neighbors.insert(par.neighbors.end(), cell_id_map[neighbor].begin(), cell_id_map[neighbor].end());
          }
        }
      }
    }
  }
}

void Particles::calcLambda(Particle &par)
//calculate lambda and update par's lambda
{
    //calculate Ci = pi/rest_density - 1
    float Ci;
    float pi;
    for (Particle * neighbor : par.neighbors) {
        pi += calcPoly(par.p - neighbor->p, kernel_size);
    }
    Ci = pi/rest_density - 1;

    //calculate pkCi
    float pkCi;
    double iSum; //pkCi for when k = i
    glm::dvec3 iSumVec;
    float jSum;
    for (Particle * neighbor : par.neighbors) {
        iSumVec += calcSpiky(par.p - neighbor->p, kernel_size);
        //TODO with respect to p_k
        if (&par != neighbor) {
            float jSumTemp = (float) (1/rest_density) * -calcSpiky(par.p - neighbor->p, kernel_size).length();
            jSum += pow(jSumTemp, 2.0);
        }
    }
    iSumVec = (1/rest_density) * iSumVec;
    iSum += pow(iSumVec.length(), 2.0);
    pkCi = iSum + jSum + epsilon;

    par.lambda = -(Ci / pkCi);
    //TODO ask Olivia about new interpretation on calcLambda
    //calculate Ci = pi/rest_density - 1
//    float Ci;
//    float pi;
//    for (Particle * neighbor : par.neighbors) {
//        pi += calcPoly(par.p - neighbor->p, kernel_size);
//    }
//    Ci = pi/rest_density - 1;
//    double gradient_constraint_neighbors = epsilon;
//    for (Particle * neighbor : par.neighbors) {
//        glm::dvec3 gradient_constraint_fn = glm::dvec3(0.0, 0.0, 0.0);
//        if (&par == neighbor) {
//          for (Particle * next_neighbor : par.neighbors) {
//            gradient_constraint_fn += calcSpiky(par.p - next_neighbor->p, kernel_size);
//          }
//        } else {
//          gradient_constraint_fn = -1.0 * calcSpiky(par.p - neighbor->p, kernel_size);
//        }
//        gradient_constraint_fn = (1.0 / rest_density) * gradient_constraint_fn;
//        gradient_constraint_neighbors += pow(gradient_constraint_fn.length(), 2.0);
//    }
//    par.lambda = -1.0 * (Ci / gradient_constraint_neighbors);

}

void Particles::calcDeltaP(Particle &par)
//calculate lambda and update par's lambda
{
  double sum = 0;
  glm::vec3 deltaP = glm::vec3(0,0,0);
  for(Particle *other_particle : par.neighbors) {
    double new_lambda = other_particle->lambda + par.lambda;
    deltaP += calcSpiky(par.p - other_particle->p, kernel_size) * new_lambda * (1.0 / rest_density);

    
  }
  par.deltap =  deltaP;
}

void Particles::calcCollision(Particle &par){
  printf("collsion particle x : %f , y: %f, z : %f \n", par.newp.x, par.newp.y, par.newp.z); 
    if (par.newp.x > 1.0){
        par.newp.x = 1.0;
    }
    if (par.newp.y > 1.0){
        par.newp.y = 1.0;
    }
    if (par.newp.z > 1.0){
        par.newp.z = 1.0;
    }
    if (par.newp.x < -1.0){
        par.newp.x = -1.0;
    }
    if (par.newp.y < -1.0){
        par.newp.y = -1.0;
    }
    if (par.newp.z < -1.0){
        par.newp.z = -1.0;
    }
}

glm::dvec3 Particles::calcSpiky(glm::dvec3 p, float h){
  double constant =  45/M_PI * pow(h,6) * pow(h - p.length(),2)/ p.length() ;
  return p * constant;
}

void Particles::calcVorticity(Particle &par)
//calculate lambda and update par's lambda
{
    glm::dvec3 vorticity = glm::dvec3(0.0, 0.0, 0.0);
    for (Particle *neighbor : par.neighbors) {
        //TODO calcSpiky with respect to neighbor
        vorticity += cross((neighbor->v - par.v), calcSpiky(par.p - neighbor->p, kernel_size));
    }
    glm::dvec3 gradient_vorticity = pow(pow(vorticity.x, 2) + pow(vorticity.y, 2) + pow(vorticity.z, 2), -0.5) * vorticity;
    glm::dvec3 N = gradient_vorticity * (1.0 / (double) gradient_vorticity.length()); 
    glm::dvec3 f_vorticity = epsilon * cross(N, vorticity);
    par.v += (f_vorticity * dt); //apply forces
}
void Particles::calcViscosity(Particle &par)
//calculate lambda and update par's lambda
{
    glm::dvec3 viscosity = glm::dvec3(0.0, 0.0, 0.0);
    for (Particle *neighbor : par.neighbors) {
        //TODO calcSpiky with respect to neighbor
        viscosity += (neighbor->v - par.v) * calcPoly(par.p - neighbor->p, kernel_size);
    }
    par.v += 0.01 * viscosity; 
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
    
    printf("RENDER\n");
    for(const Particle &par : particles)
    {    
        printf("particle x : %f , y: %f, z : %f \n", par.p.x, par.p.y, par.p.z); 
        printf("particle num neighbors : %lu \n", par.neighbors.size()); 
        glPushMatrix();
        glTranslatef(par.p.x, par.p.y, par.p.z);
        glutSolidSphere(radius, 10, 10);
        glPopMatrix();
    }
    
    glPopAttrib();
}

