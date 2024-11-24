#pragma once


#include "cgp/cgp.hpp"
#include "environment.hpp"

#include "simulation/simulation.hpp"

using cgp::mesh_drawable;


struct gui_parameters {
	bool display_frame = false;
	bool reset_animation = true;
	int num_spheres = 1;
	int num_particles_to_emit = 1;

};

// The structure of the custom scene
struct scene_structure : scene_inputs_generic {
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	camera_controller_orbit_euler camera_control;
	camera_projection_perspective camera_projection;
	window_structure window;

	mesh_drawable global_frame;          // The standard global frame
	environment_structure environment;   // Standard environment controler
	input_devices inputs;                // Storage for inputs status (mouse, keyboard, window dimension)
	gui_parameters gui;                  // Standard GUI element storage
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	cgp::timer_event_periodic timer;
	std::vector<particle_structure> particles;
	std::vector<cgp::vec3> attach_points;
	cgp::mesh_drawable sphere;
	cgp::mesh_drawable barre;
	cgp::mesh_drawable support_gauche;
	cgp::mesh_drawable support_droite;

	cgp::mesh_drawable support_gauche_incline;
	cgp::mesh_drawable support_droite_incline;
	cgp::mesh_drawable sol;

	cgp::curve_drawable segments_dra;
	float length_of_fil = 0.9f;  // Longueur fixe du fil
	int previous_num_spheres = 5;




	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();    // Standard initialization to be called before the animation loop
	void display_frame(); // The frame display to be called within the animation loop
	void display_gui();   // The display of the GUI, also called within the animation loop

	void mouse_move_event();
	void mouse_click_event();
	void keyboard_event();
	void idle_frame();

	void emit_particle();
	void simulation_step(float dt);
	void sphere_display();
	void update_bar_and_supports();
};





