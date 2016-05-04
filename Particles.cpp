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

inline float clip(const float& n, const float& lower, const float& upper) 
{
    return glm::max(lower, glm::min(n, upper));
}

Particles::Particles(float most_bottom[3], float cube_width, float cube_length, float cube_height, float most_bottom_obstacle[3], float obstacle_width, float obstacle_length, float obstacle_height) 
{
    bottom_pt[0] = most_bottom[0];
    bottom_pt[1] = most_bottom[1];
    bottom_pt[2] = most_bottom[2];
    box_width = cube_width;
    box_length = cube_length;
    box_height = cube_height;
    obstacle_bottom_pt[0] = most_bottom_obstacle[0];
    obstacle_bottom_pt[1] = most_bottom_obstacle[1];
    obstacle_bottom_pt[2] = most_bottom_obstacle[2];
    printf("obstacle box x %f y %f z %f\n", obstacle_bottom_pt[0], 
        obstacle_bottom_pt[1], obstacle_bottom_pt[2]);
    printf("other obstacle box x %f y %f z %f\n", obstacle_bottom_pt[0] + obstacle_width, 
        obstacle_bottom_pt[1] + obstacle_height, obstacle_bottom_pt[2] + obstacle_length);
    obstacle_box_width = obstacle_width;
    obstacle_box_length = obstacle_length;
    obstacle_box_height = obstacle_height;

    int nx = 5;
    int ny = 5;
    int nz = 5;
    float d = 0.1;

    kernel_size = d * 1.4;
    //TODO weilun doesn't use this radius when rendering
    //radius = d * 0.45;
    radius = 0.05;


    k = 0.001;
    n = 4;
    q = 0.2;
    epsilon = 1000;
    nIters = 20;
    rest_density = 1 / (d * d * d);
    dt = 0.01;

    for(int x=0; x<nx; x++)
    {
        for(int y=0; y<ny; y++)
        {
            for(int z=0; z<nz; z++)
            {
                Particle par;
                par.p = glm::dvec3((x+0.5-nx*0.5)*d, (y+0.5)*d-1.0, (z+0.5-nz*0.5)*d);
                par.newp = par.p;
                printf("initial par p x %f y %f z %f\n", par.newp.x, par.newp.y, par.newp.z);
                par.prev_newp = par.p;
                par.v = glm::dvec3(0, 0, 0);
                par.neighbors = {};
                par.lambda = 0;
                par.deltap = glm::dvec3(0, 0, 0);
                particles.push_back(par);
            }
        }
    }
}

//Kernel Functions
double Particles::calcPoly(glm::dvec3 r, float h) 
{
  if (dvec3_length(r) < 0) {
    return 0.;
  }
  if (dvec3_length(r) > h) {
    return 0.;
  }
    double temp = pow(h, 2.0) - pow(dvec3_length(r), 2.0);
    double value = (315. / (64. * M_PI * pow(h, 9.0))) * pow(temp, 3.0);
    return value;
}


void Particles::step() //simulation loop
{
    for(Particle &par : particles) {
        par.neighbors = {};
        par.v = par.v + (extForce(par.p) * dt); //apply forces
        //printf("%f\n", par.v.y);
        par.newp = par.p + (dt * par.v); //predict position
        par.prev_newp = par.p;
    }
    std::map<std::string, std::vector<Particle *>>  cell_id_list; 
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
            //collisions
            calcCollision(par);

        }
        for(Particle &par : particles) {
            par.prev_newp = par.newp;
            par.newp += par.deltap;
            //printf("updated newp  newp x %f y %f z %f\n", par.newp.x, par.newp.y, par.newp.z);
        }
        iter++;
    }
    for(Particle &par : particles) {
        //update velocity
        par.v = (1.0 / dt) * (par.newp - par.p);
//        //apply vorticity
//        calcVorticity(par);
//        //apply viscosity
//        calcViscosity(par);
        //update position
        par.p = par.newp;
    }

}

glm::dvec3 Particles::extForce(glm::dvec3 position) 
//find forces and return extForce at position
{
//    return glm::dvec3(0, 0, 0);
    return glm::dvec3(0, -9.81, 0);
}

void Particles::createCellIdList(std::map<std::string, std::vector<Particle *>>  &cell_id_map) {  
    for (Particle &par : particles) {
        findCellId(par);
        std::string id = createStringCellId(par.cellId);
        if (cell_id_map.find(id) == cell_id_map.end()) {
            //not found
            cell_id_map.insert(std::pair<std::string, std::vector<Particle *>>(id, std::vector<Particle *> ()));
        }
        cell_id_map[id].push_back(&par);
    }
}

void Particles::findCellId(Particle& par)
{
    glm::dvec3 pos = par.newp;
    par.cellId[0] = (int) (pos.x / (2. * kernel_size));
    par.cellId[1] = (int) (pos.y / (2. * kernel_size));
    par.cellId[2] = (int) (pos.z / (2. * kernel_size)); 
}

std::string Particles::createStringCellId(int (&pos) [3])
{
    return std::to_string(pos[0]) + "," + std::to_string(pos[1]) + "," + std::to_string(pos[2]);
}



void Particles::findNeighbors(Particle &par, std::map<std::string, std::vector<Particle *>>  &cell_id_map)
//calculate neighbors and update par's neighbors
{
  for (int x = -1; x <= 1; x += 1) {
    for (int y = -1; y <= 1; y += 1) {
      for (int z = -1; z <= 1; z += 1) {
        int neighbor[3] = {par.cellId[0] + x, par.cellId[1] + y, par.cellId[2] + z};
        std::string id = createStringCellId(neighbor);
        if (cell_id_map.find(id) != cell_id_map.end()) {
          //found
            par.neighbors.insert(par.neighbors.end(), cell_id_map[id].begin(), cell_id_map[id].end());
        }
      }
    }
  }
}

void Particles::calcLambda(Particle &par)
//calculate lambda and update par's lambda
{
    //calculate Ci = pi/rest_density - 1
    float Ci = 0.;
    float pi = 0.;

    for (Particle * neighbor : par.neighbors) {
        pi += calcPoly(par.newp - neighbor->newp, kernel_size);
    }
    Ci = -((pi/rest_density) - 1);

    //calculate pkCi
    float pkCi = 0.;
    double iSum = 0.; //pkCi for when k = i
    glm::dvec3 iSumVec(0.,0.,0.);
    float jSum = 0.;
    for (Particle * neighbor : par.neighbors) {
        iSumVec += calcSpiky(par.newp - neighbor->newp, kernel_size);
        //TODO with respect to p_k
        if (&par != neighbor) {
            glm::dvec3 l = calcSpiky(par.newp - neighbor->newp, kernel_size);
            glm::dvec3 jSumTemp = (float) (1/rest_density) * -1.0 * l;
            jSum += pow(dvec3_length(jSumTemp), 2.0);

        }

    }
    //printf("%lu\n", par.neighbors.size());

//    printf("Own particle: %d \n", ownParticle); 
    iSumVec = (1/rest_density) * iSumVec;
    iSum += pow(dvec3_length(iSumVec), 2.0);
    pkCi = iSum + jSum + epsilon;

    //printf("jSum: %f\n", jSum); 
    //printf("iSum: %f\n", iSum); 

    par.lambda =  -1 * (Ci / pkCi);

}

void Particles::calcDeltaP(Particle &par)
//calculate lambda and update par's lambda
{

  glm::vec3 deltaP = glm::vec3(0,0,0);
  for(Particle *other_particle : par.neighbors) {
    double new_lambda = other_particle->lambda + par.lambda;
    deltaP += calcSpiky(par.newp - other_particle->newp, kernel_size) * new_lambda ;

    
  }
  par.deltap =  deltaP * (float) (1.0 / rest_density);
  //printf("deltap x %f y %f z %f\n", par.deltap.x, par.deltap.y, par.deltap.z);
}


void Particles::calcCollision(Particle &par) {
  printf("collision  newp x %f y %f z %f\n", par.newp.x, par.newp.y, par.newp.z);
    float col = 0.0014f;
    if (par.newp.x > bottom_pt[0] + box_width){
        par.newp.x = bottom_pt[0] + box_width - col;
    }
    if (par.newp.y > bottom_pt[1] + box_height){
        par.newp.y = bottom_pt[1] + box_height - col;
    }
    if (par.newp.z > bottom_pt[2] + box_length){
        par.newp.z = bottom_pt[2] + box_length - col;
    }

    if (par.newp.x < bottom_pt[0]){
        par.newp.x = bottom_pt[0] + col;
    }
    if (par.newp.y < bottom_pt[1]){
        par.newp.y = bottom_pt[1] + col;
    }
    if (par.newp.z < bottom_pt[2]){
        par.newp.z = bottom_pt[2] + col;
    }
   
    bool obstacle_x = (par.newp.x >= obstacle_bottom_pt[0]) and (par.newp.x <= obstacle_bottom_pt[0] + obstacle_box_width);
    bool obstacle_y = (par.newp.y >= obstacle_bottom_pt[1]) and (par.newp.y <= obstacle_bottom_pt[1] + obstacle_box_height);
    bool obstacle_z = (par.newp.z >= obstacle_bottom_pt[2]) and (par.newp.z <= obstacle_bottom_pt[2] + obstacle_box_length); 

    auto plane_intersect = [] (float o, float p, float d) -> float {
      printf("num %f denom %f\n", p-o, d);
      return (p - o) / d;
    };

    if (obstacle_x and obstacle_y and obstacle_z) {
      if (obstacle_x){
        float t_left = plane_intersect(par.prev_newp.x, obstacle_bottom_pt[0], par.v.x);
        float t_right = plane_intersect(par.prev_newp.x, obstacle_bottom_pt[0] + obstacle_box_width, par.v.x);
        if (t_left >= 0 and t_right >= 0) {
          if (t_left < t_right) {
            par.newp.x = par.prev_newp.x + t_left * par.v.x - col;
          } else {
            par.newp.x = par.prev_newp.x + t_right * par.v.x + col;
          }
        } else {
          printf("X error newp x %f\n", par.newp.x);
          printf("X ERROR t_left %f t_right %f \n", t_left, t_right);
        }
      }
      if (obstacle_y){
          float t_left = plane_intersect(par.prev_newp.y, obstacle_bottom_pt[1], par.v.y);
          float t_right = plane_intersect(par.prev_newp.y, obstacle_bottom_pt[1] + obstacle_box_height, par.v.y);
          if (t_left >= 0 and t_right >= 0) {
            if (t_left < t_right) {
              printf(" Y bottom ERROR\n");
            } else {
              par.newp.y = par.prev_newp.y + (t_right * par.v.y) + col;
            }
          } else {
            printf("y error newp y %f\n", par.newp.y);
            printf("Y ERROR t_left %f t_right %f \n", t_left, t_right);
          }
      }
      if (obstacle_z){
          float t_left = plane_intersect(par.prev_newp.z, obstacle_bottom_pt[2], par.v.z);
          float t_right = plane_intersect(par.prev_newp.z, obstacle_bottom_pt[2] + obstacle_box_length, par.v.z);
          if (t_left >= 0 and t_right >= 0) {
            if (t_left < t_right) {
              par.newp.z = par.prev_newp.z + t_left * par.v.z - col;
            } else {
              par.newp.z = par.prev_newp.z + t_right * par.v.z + col;
            }
          } else {
            printf("z error newp z %f\n", par.newp.z);
            printf("Z ERROR t_left %f t_right %f \n", t_left, t_right);
          }
      }
      //printf("collision x %f y %f z %f\n", par.newp.x, par.newp.y, par.newp.z);
  }

}

double Particles::dvec3_length(glm::dvec3 p) {
  double value = sqrt(pow(p.x,2) + pow(p.y,2) + pow(p.z,2));
  return value;
}

glm::dvec3 Particles::calcSpiky(glm::dvec3 p, float h){

  if (dvec3_length(p) <= 0) {
    return glm::dvec3(0, 0, 0);
  }
  if (dvec3_length(p) > h) {
    return glm::dvec3(0, 0, 0);
  }

  double constant =  (15.0 * pow(h - dvec3_length(p),2.)) /(M_PI * pow(h,6.));
  glm::dvec3 val =  (p / dvec3_length(p)) * constant;
  //printf("calcSpiky x %f y %f z %f\n", val.x, val.y, val.z);
  return val;
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
    glm::dvec3 N = gradient_vorticity * (1.0 / (double) dvec3_length(gradient_vorticity)); 
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
    GLfloat mat_shininess[] = { 75.0 };
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
        glutSolidSphere(radius, 10, 10);
        glPopMatrix();
    }
    
    glPopAttrib();
}

