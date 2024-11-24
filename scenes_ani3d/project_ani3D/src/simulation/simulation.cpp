#include "simulation.hpp"

using namespace cgp;


#include <vector>


void handle_collision_sphere_sphere(std::vector<particle_structure>& particles, particle_structure& particle)
{
	size_t const N = particles.size();

	for (size_t k = 0; k < N; ++k)
	{
		particle_structure& other = particles[k];

		// Ignorer la collision avec soi-même
		if (&particle == &other)
			continue;

		vec3 const p12 = particle.p - other.p;
		float const dist = norm(p12);
		float const r_sum = particle.r + other.r;

		// Vérifier si les sphères se chevauchent
		if (dist < r_sum)
		{
			// Calculer le vecteur normal
			vec3 const normal = p12 / dist;

			// Calculer la profondeur de pénétration
			float const penetration_depth = r_sum - dist;

			// Corriger les positions pour éviter l'interpénétration
			particle.p = particle.p + 0.5f * penetration_depth * normal;
			other.p = other.p - 0.5f * penetration_depth * normal;

			// Calculer la composante de vitesse le long de la direction normale
			float v_rel = dot(particle.v - other.v, normal);

			// Appliquer la force de restitution (élastique)
			if (v_rel < 0) // Assurer que la collision est en approche et non en éloignement
			{
				float e = 0.9f; // Coefficient de restitution (1.0 pour une collision parfaitement élastique)
				float j = -(1 + e) * v_rel / (1 / particle.m + 1 / other.m);

				// Mettre à jour les vitesses des particules en fonction de l'impulsion
				particle.v = particle.v + (j / particle.m) * normal;
				other.v = other.v - (j / other.m) * normal;
			}
		}
	}
}


void apply_tension_force(particle_structure& particle, const vec3& attach_point, float length_of_fil) {
    // Calculer le vecteur qui va du point d'attache à la particule
    vec3 tension_dir = particle.p - attach_point;
    float current_length = norm(tension_dir);

    // Normaliser le vecteur direction de la tension
    tension_dir /= current_length;

    // Calculer la force de tension pour maintenir la longueur du fil constante
    float stretch_amount = current_length - length_of_fil;

    // Intensité de la force de tension (ajuster ce coefficient pour régler l'effet)
    float k_tension = 50.0f; // Coefficient de raideur du fil

    // Appliquer la force de tension qui va dans la direction opposée de l'étirement
    vec3 tension_force = -k_tension * stretch_amount * tension_dir;

    // Appliquer cette force à la vitesse de la particule (pour simuler une accélération)
    particle.v += tension_force / particle.m;
}

void simulate(std::vector<particle_structure>& particles, float dt_arg, const std::vector<vec3>& attach_points, float length_of_fil) {
    size_t const N_substep = 10;
    float const dt = dt_arg / N_substep;
    for (size_t k_substep = 0; k_substep < N_substep; ++k_substep) {
        vec3 const g = { 0,0,-9.81f };
        size_t const N = particles.size();

        // Appliquer les forces de collision et la gravité
        for (size_t k = 0; k < N; ++k) {
            particle_structure& particle = particles[k];

            handle_collision_sphere_sphere(particles, particle);
        }

        // Calculer et appliquer la force de tension pour chaque particule
        for (size_t k = 0; k < N; ++k) {
            apply_tension_force(particles[k], attach_points[k], length_of_fil);
        }

        // Mettre à jour la vitesse avec la force de gravité et la friction
        for (size_t k = 0; k < N; ++k) {
            particle_structure& particle = particles[k];
            vec3 const f = particle.m * g;
            particle.v = (1 - 0.9f * dt) * particle.v + dt * f / particle.m;
        }

        // Mettre à jour la position en fonction de la vitesse
        for (size_t k = 0; k < N; ++k) {
            particle_structure& particle = particles[k];
            particle.p = particle.p + dt * particle.v;
        }
    }
}

