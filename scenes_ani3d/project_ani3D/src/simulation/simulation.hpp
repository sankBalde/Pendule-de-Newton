#pragma once

#include "cgp/cgp.hpp"


struct particle_structure
{
    cgp::vec3 p; // Position
    cgp::vec3 v; // Speed

    cgp::vec3 c; // Color
    float r;     // Radius
    float m;     // mass
};


//void simulate(std::vector<particle_structure>& particles, float dt);

void simulate(std::vector<particle_structure>& particles, float dt_arg, const std::vector<cgp::vec3>& attach_points, float length_of_fil);