#include "scene.hpp"


using namespace cgp;




void scene_structure::initialize()
{
	camera_control.initialize(inputs, window);
	camera_control.set_rotation_axis_z();
	camera_control.look_at({ 3.0f, 2.0f, 2.0f }, {0,0,0}, {0,0,1});
	global_frame.initialize_data_on_gpu(mesh_primitive_frame());

	timer.event_period = 0.5f;

	sphere.initialize_data_on_gpu(mesh_primitive_sphere());
	sphere.material.alpha = 128.0f;
	sphere.material.color = {0.8f, 0.8f, 0.8f};

	// Initialisation du nombre de boules par défaut
	gui.num_spheres = 5;
	previous_num_spheres = gui.num_spheres;

	update_bar_and_supports();
}

void scene_structure::update_bar_and_supports()
{
	// Calcul de la longueur totale de la barre en fonction du nombre de sphères
	float bar_length = (gui.num_spheres - 1) * 0.2f;

	// Mise à jour de la barre métallique
	barre.clear();
	barre.initialize_data_on_gpu(mesh_primitive_cylinder(0.02f, {0, 0, 0}, {bar_length, 0, 0}));
	barre.material.alpha = 128.0f;
	barre.material.color = {0.7f, 0.7f, 0.7f};

	// Mise à jour des supports verticaux
	support_gauche.clear();
	support_droite.clear();
	support_gauche.initialize_data_on_gpu(mesh_primitive_cylinder(0.02f, {0, 0, 0}, {0.0f, 0.5f, -1.0f}));
	support_droite.initialize_data_on_gpu(mesh_primitive_cylinder(0.02f, {bar_length, 0, 0}, {bar_length, 0.5f, -1.0f}));
	support_gauche.material.color = {0.7f, 0.7f, 0.7f};
	support_droite.material.color = {0.7f, 0.7f, 0.7f};

	// Mise à jour des supports inclinés
	support_gauche_incline.clear();
	support_droite_incline.clear();
	support_gauche_incline.initialize_data_on_gpu(mesh_primitive_cylinder(0.02f, {0, 0, 0.0f}, {-0.5f, -0.5f, -1.0f}));
	support_droite_incline.initialize_data_on_gpu(mesh_primitive_cylinder(0.02f, {bar_length, 0, 0.0f}, {bar_length + 0.5f, -0.5f, -1.0f}));
	support_gauche_incline.material.color = {0.7f, 0.7f, 0.7f};
	support_droite_incline.material.color = {0.7f, 0.7f, 0.7f};

	// Ajout du plan pour simuler le sol avec une taille ajustée
	float sol_length = bar_length + 1.0f; // Ajustement de la taille en fonction de la barre
	sol.clear();
	sol.initialize_data_on_gpu(mesh_primitive_quadrangle(
		{-sol_length, -1.0f, -1.2f}, {sol_length, -1.0f, -1.2f},
		{sol_length, 1.0f, -1.2f}, {-sol_length, 1.0f, -1.2f}
	));
	sol.material.color = {0.6f, 0.6f, 0.6f}; // Gris clair pour le sol
}



void scene_structure::display_frame()
{
	// Set the light to the current position of the camera
	environment.light = camera_control.camera_model.position();

	if (gui.display_frame)
		draw(global_frame, environment);

	timer.update();

	sol.model.translation = {0.4f, 0.0f, 0.19f};
	// Affichage du sol
	draw(sol, environment);

	// Affichage de la barre métallique
	barre.model.translation = {0, 0, 0}; // Position au-dessus des sphères
	draw(barre, environment);

	support_gauche.model.translation = {0, 0, 0};
	draw(support_gauche, environment);

	support_droite.model.translation = {0, 0, 0};
	draw(support_droite, environment);

	// Supports inclinés
	support_gauche_incline.model.translation = {0, 0, 0};
	draw(support_gauche_incline, environment);

	support_droite_incline.model.translation = {0, 0, 0};
	draw(support_droite_incline, environment);

	// Create a new particle if needed
	emit_particle();

	// Call the simulation of the particle system
	float const dt = 0.01f * timer.scale;
	simulate(particles, dt, attach_points, length_of_fil);

	// Display the result
	sphere_display();

}

void scene_structure::sphere_display()
{
	// Display the particles as spheres
	size_t const N = particles.size();
	for (size_t k = 0; k < N; ++k)
	{
		particle_structure const& particle = particles[k];
		//sphere.material.color = particle.c;
		sphere.material.color = particle.c * 0.5f;
		sphere.model.translation = particle.p;
		sphere.model.scaling = particle.r;


		draw(sphere, environment);
		numarray<vec3> line_data = { attach_points[k], particle.p };
		segments_dra.initialize_data_on_gpu(line_data);
		segments_dra.color = {0.6f, 0.6f, 0.6f};

		draw(segments_dra, environment);
		segments_dra.clear();
	}


}

void scene_structure::emit_particle()
{
	// Emit particle with random velocity
	//  Assume first that all particles have the same radius and mass
	static numarray<vec3> const color_lut = { {1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1} };


	float spacing = 0.2f; // distance entre deux fils

	if (gui.reset_animation)
	{
		particles.clear();
		attach_points.clear();
		gui.reset_animation = false;
	}

	//if (timer.event && gui.add_sphere) {
	if (particles.size() < gui.num_spheres) {

		for (int i = particles.size(); i < gui.num_spheres; ++i) {
			particle_structure particle;
			float angle = (i < gui.num_particles_to_emit) ? - M_PI / 2 : 0.0f; // Angle initial pour soulever les sphères

			particle.p = {
				i * spacing + length_of_fil * std::sin(angle),
				0.0f,
				-length_of_fil * std::cos(angle)
			};

			particle.r = 0.1f;
			particle.c = color_lut[int(rand_uniform() * color_lut.size())];
			particle.v = (i < gui.num_particles_to_emit) ? vec3{0.5f, 0, 0} : vec3{0.0f, 0, 0};  // Vitesse initiale nulle sauf pour les spheres à lancer
			particle.m = 1.5f;

			// Définition des points d'attache pour chaque pendule
			vec3 attach_point = {i * spacing, 0, 0}; // Position d'attache fixe au-dessus de la sphère
			attach_points.push_back(attach_point);
			particles.push_back(particle);
		}
	}
}


void scene_structure::display_gui()
{
	ImGui::Checkbox("Frame", &gui.display_frame);

	if (ImGui::Button("Reset animation")) {
		gui.reset_animation = true;
	} else {
		gui.reset_animation = false;
	}

	ImGui::InputInt("Number of spheres", &gui.num_spheres);
	if (gui.num_spheres < 1) gui.num_spheres = 1; // Minimum de 1 boule

	if (gui.num_spheres != previous_num_spheres) {
		gui.reset_animation = true;
		previous_num_spheres = gui.num_spheres;
		if (gui.num_spheres > 5)
			update_bar_and_supports();
	}

	// Ajouter l'input pour le nombre de boules à lancer
	ImGui::InputInt("Number of particles to emit", &gui.num_particles_to_emit);
	if (gui.num_particles_to_emit < 1) gui.num_particles_to_emit = 1;
}

void scene_structure::mouse_move_event()
{
	if (!inputs.keyboard.shift)
		camera_control.action_mouse_move(environment.camera_view);
}
void scene_structure::mouse_click_event()
{
	camera_control.action_mouse_click(environment.camera_view);

}
void scene_structure::keyboard_event()
{
	camera_control.action_keyboard(environment.camera_view);
}
void scene_structure::idle_frame()
{
	camera_control.idle_frame(environment.camera_view);
}

