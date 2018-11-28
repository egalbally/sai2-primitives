/*======================================================================================
 * 05-kuka_screwing.cpp
 *
 * Example of a controller for a Kuka arm made with the screwing alignment primitive
 * ScrewingAlignment controller is used to align a bottle cap with a bottle before using
 * the RedundantArmMotion primitive to perform the screwing action.
 *
 * Daniela Deschamps, Spring 2018
 *
 *======================================================================================*/


/* --------------------------------------------------------------------------------------
   Include Required Libraries and Files
-----------------------------------------------------------------------------------------*/
#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>

#include "primitives/RedundantArmMotion.h"
#include "primitives/ScrewingAlignment.h"
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of Sai2Graphics
#include <signal.h>
#include <stdlib.h> //includes capability for exit

/* --------------------------------------------------------------------------------------
	Simulation and Control Loop Setup
-------------------------------------------------------------------------------------*/

bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";
const string bottle_file = "resources/bottle.urdf";
const string bottle_name = "Bottle";
const string camera_name = "camera";

// simulation and control loop
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* bottle, ForceSensorSim* fsensor, Simulation::Sai2Simulation* sim);

// control link and position in link
const string link_name = "link6";
const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0,0.0,0.20);
const Eigen::Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.05);
Eigen::Vector3d sensed_force;
Eigen::Vector3d sensed_moment;

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;


/* --------------------------------------------------------------------------------------
   State Machine Setup
-----------------------------------------------------------------------------------------*/

enum ControllerState {
	APPROACH,
	ALIGNMENT,
	SCREWING,
	DONE
};

enum ControllerStatus {
	RUNNING,
	STABILIZING,
	FINISHED,
	FAILED
};

ControllerStatus approach(Sai2Primitives::RedundantArmMotion*, Eigen::Matrix3d, Eigen::Vector3d, Eigen::VectorXd, Simulation::Sai2Simulation*);
ControllerStatus alignment(Sai2Model::Sai2Model*, Sai2Primitives::RedundantArmMotion*, Sai2Primitives::ScrewingAlignment*, Eigen::Matrix3d, Eigen::VectorXd, Simulation::Sai2Simulation*, Eigen::Affine3d);
ControllerStatus screwing(Sai2Model::Sai2Model*, Sai2Primitives::RedundantArmMotion*, Sai2Primitives::ScrewingAlignment*, Eigen::Vector3d, Eigen::Matrix3d, Eigen::VectorXd, Simulation::Sai2Simulation*, Eigen::Affine3d);
ControllerStatus done(Sai2Primitives::RedundantArmMotion*, Eigen::Vector3d, Eigen::VectorXd, Simulation::Sai2Simulation*);
ControllerState controller_state_;

double theta_deg = 0;
unsigned long long controller_counter = 0;
unsigned long long curr_control_time = 0;
bool force_flag = false;		// robot has not yet encountered a z force greater than threshold
Eigen::Matrix3d target_orientation;

#define APPROACH_ANGLE 	15.0	// Angle about X with which to approach bottle
#define FORCE_THRESH	1 		// threshold force to establish whether contact has been made with bottle
#define MAX_ROTATION 	360 	// max rotation in degrees
// ---------------------------------------------------------------------------------------


/* =======================================================================================
   MAIN LOOP
========================================================================================== */

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_file << endl;

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, false);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.05); //might need to increase friction when screws are added?
	sim->setCoeffFrictionDynamic(0);

	// load robots
	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();
	auto robot = new Sai2Model::Sai2Model(robot_file, false, world_gravity, sim->getRobotBaseTransform(robot_name));

	sim->getJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// load bottle
	auto bottle = new Sai2Model::Sai2Model(bottle_file, false, world_gravity, sim->getRobotBaseTransform(bottle_name));

	// load simulated force sensor
	Eigen::Affine3d T_sensor = Eigen::Affine3d::Identity();
	T_sensor.translation() = sensor_pos_in_link;
	auto fsensor = new ForceSensorSim(robot_name, link_name, T_sensor, robot);
	auto fsensor_display = new ForceSensorDisplay(fsensor, graphics);


	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

	double last_cursorx, last_cursory;

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, bottle, fsensor, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

    	fsensor_display->update();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->updateGraphics(bottle_name, bottle);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();

		// move scene camera as required
    	// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
    	Eigen::Vector3d cam_depth_axis;
    	cam_depth_axis = camera_lookat - camera_pos;
    	cam_depth_axis.normalize();
    	Eigen::Vector3d cam_up_axis;
    	// cam_up_axis = camera_vertical;
    	// cam_up_axis.normalize();
    	cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
	    Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
    	cam_roll_axis.normalize();
    	Eigen::Vector3d cam_lookat_axis = camera_lookat;
    	cam_lookat_axis.normalize();
    	if (fTransXp) {
	    	camera_pos = camera_pos + 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_roll_axis;
	    }
	    if (fTransXn) {
	    	camera_pos = camera_pos - 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_roll_axis;
	    }
	    if (fTransYp) {
	    	// camera_pos = camera_pos + 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos + 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_up_axis;
	    }
	    if (fTransYn) {
	    	// camera_pos = camera_pos - 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos - 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_up_axis;
	    }
	    if (fTransZp) {
	    	camera_pos = camera_pos + 0.1*cam_depth_axis;
	    	camera_lookat = camera_lookat + 0.1*cam_depth_axis;
	    }	    
	    if (fTransZn) {
	    	camera_pos = camera_pos - 0.1*cam_depth_axis;
	    	camera_lookat = camera_lookat - 0.1*cam_depth_axis;
	    }
	    if (fRotPanTilt) {
	    	// get current cursor position
	    	double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
	    }
	    graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
	    glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}


/* =======================================================================================
   CONTROL LOOP
========================================================================================== */
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	
	robot->updateModel();
	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);

	Eigen::Affine3d control_frame_in_link = Eigen::Affine3d::Identity();
	control_frame_in_link.translation() = pos_in_link;
	Eigen::Affine3d sensor_frame_in_link = Eigen::Affine3d::Identity();
	sensor_frame_in_link.translation() = pos_in_link;

	// Motion arm primitive - for initial alignment
	Sai2Primitives::RedundantArmMotion* motion_primitive = new Sai2Primitives::RedundantArmMotion(robot, link_name, pos_in_link);
	// Eigen::VectorXd motion_primitive_torques;
	motion_primitive->enableGravComp();

	// Screwing primitive 
	Sai2Primitives::ScrewingAlignment* screwing_primitive = new Sai2Primitives::ScrewingAlignment(robot, link_name, control_frame_in_link, sensor_frame_in_link);
	Eigen::VectorXd screwing_primitive_torques;
	screwing_primitive->enableGravComp();

	Eigen::Matrix3d current_orientation;
	Eigen::Vector3d current_position;	
	Eigen::Vector3d current_pos1;	
	Eigen::Matrix3d initial_orientation;
	Eigen::Vector3d initial_position;
	robot->rotation(initial_orientation, motion_primitive->_link_name);
	robot->position(initial_position, motion_primitive->_link_name, motion_primitive->_control_frame.translation());

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	timer.initializeTimer(1000000); // 1 ms pause before starting loop


	unsigned long long screw_counter = 0;
	bool setup_flag = false;		// triggered when setup of position and orientation are complete
	double theta_deg = 0;

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;
		double time = controller_counter/control_freq;

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		// update tasks model
		screwing_primitive->updatePrimitiveModel(); //ADDED
		motion_primitive->updatePrimitiveModel();

		// update sensed values (need to put them back in sensor frame)
		Eigen::Matrix3d R_link;
		robot->rotation(R_link, link_name);
		Eigen::Matrix3d R_sensor = R_link*sensor_frame_in_link.rotation();
		screwing_primitive->updateSensedForceAndMoment(- R_sensor.transpose() * sensed_force, - R_sensor.transpose() * sensed_moment);


		switch (controller_state_){
			case APPROACH:
				// cout << "APPROACH" << endl;
				if (approach(motion_primitive, initial_orientation, initial_position, command_torques, sim) == FINISHED){
					cout << endl;
					cout << "APPROACH FINISHED" << endl;
					cout << "---" << endl;
					controller_state_= ALIGNMENT;
				}		
				break;
			case ALIGNMENT:
				// cout << "ALIGNMENT" << endl;
				if (alignment(robot, motion_primitive, screwing_primitive, initial_orientation, command_torques, sim, sensor_frame_in_link) == FINISHED)
				{
					cout << "ALIGNMENT FINISHED" << endl;
					cout << "---" << endl;
					robot->position(current_position, motion_primitive->_link_name, motion_primitive->_control_frame.translation());
					curr_control_time = 0;
					controller_state_ = SCREWING;
				}
				break;
			case SCREWING:
				// cout << "START SCREW" << endl;
				current_pos1 = current_position;
				robot->position(current_position, motion_primitive->_link_name, motion_primitive->_control_frame.translation());
				current_position = current_pos1;

				if (screwing(robot, motion_primitive, screwing_primitive, current_position, initial_orientation, command_torques, sim, sensor_frame_in_link) == FINISHED)
				{
					cout << "SCREWING FINISHED" << endl;
					cout << "---" << endl;
					robot->position(current_position, motion_primitive->_link_name, motion_primitive->_control_frame.translation());
					controller_state_ = DONE;
				}
				break;
			case DONE:
				if (done(motion_primitive, current_position, command_torques, sim) == FINISHED){
					break;
				}
				break;				
			default:
				cout << "Invalid controller state. Stopping controller." << endl;
				robot->position(current_position, motion_primitive->_link_name, motion_primitive->_control_frame.translation());
				motion_primitive->_desired_position = current_position;
				motion_primitive->computeTorques(command_torques);
				sim->setJointTorques(robot_name, command_torques);	
				break;
		}

		// update counter and timer
		controller_counter++;
		curr_control_time++;
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Control Loop run time  : " << end_time << " seconds\n";
    std::cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}


/* =======================================================================================
   STATE MACHINE CASES
   -----------------------
   * Approach: 	Brings end-effector to a point close to the bottle at a pre-set approach
   				angle. The point is hard-coded for simulation, but should be determined by
   				robot vision in real life.
   				  -> RedundantArmMotion.cpp

   * Alignment:	Brings cap in contact with bottle, and uses force and moment control to
   				align it over the lip of the bottle.
   				  -> ScrewingAlignment.cpp

   * Screwing:  Rotates the cap on the bottle until a target orientation is reached. The
   				target orientation criteria should be replaced by a force criteria once
   				threads are added.
   				  -> RedundantArmMotion.cpp

   * Done:		Stops robot motion by having it maintain its current position.
   				  -> RedundantArmMotion.cpp
========================================================================================== */

ControllerStatus approach(Sai2Primitives::RedundantArmMotion* motion_primitive, Eigen::Matrix3d initial_orientation, Eigen::Vector3d initial_position, Eigen::VectorXd command_torques, Simulation::Sai2Simulation* sim) {
	// orientation part
	if(theta_deg <= APPROACH_ANGLE)
	{
		Eigen::Matrix3d R;
		double theta = -M_PI/2.0/500.0 * (controller_counter);
		theta_deg = -theta*180/M_PI;
		R <<      1     ,      0      ,      0     ,
	   		      0     ,  cos(theta) , -sin(theta),
	       		  0     ,  sin(theta) ,  cos(theta);

		motion_primitive->_desired_orientation = R*initial_orientation;
	}

	// position part - would be replaced by point determined by robot vision
	Eigen::Vector3d approach_point;
	approach_point <<  -0.1,
					  -0.19,
					  -0.24;

	motion_primitive->_desired_position = initial_position + approach_point;
	motion_primitive->_desired_velocity = Eigen::Vector3d(0.5,0.5,0.5);

	// torques
	motion_primitive->computeTorques(command_torques);
	sim->setJointTorques(robot_name, command_torques);

	if (controller_counter >= 1500){
		theta_deg = 0;
		return FINISHED;
	}

	return RUNNING;
}

ControllerStatus alignment(Sai2Model::Sai2Model* robot, Sai2Primitives::RedundantArmMotion* motion_primitive, Sai2Primitives::ScrewingAlignment* screwing_primitive, Eigen::Matrix3d initial_orientation, Eigen::VectorXd command_torques, Simulation::Sai2Simulation* sim, Eigen::Affine3d sensor_frame_in_link){
	screwing_primitive->_desired_orientation = initial_orientation;

	Eigen::Matrix3d R_link;
	robot->rotation(R_link, link_name);
	Eigen::Matrix3d R_sensor = R_link*sensor_frame_in_link.rotation();
	screwing_primitive->updateSensedForceAndMoment(- R_sensor.transpose() * sensed_force, - R_sensor.transpose() * sensed_moment);

	if(sensed_force[2] > FORCE_THRESH && force_flag == false)
	{
		force_flag = true;
		cout << endl;
		cout << "Robot has reached the bottle" << endl;
		cout << endl;
	}

	if (force_flag == true){
		screwing_primitive->_desired_normal_force = 20;

	}
	else{
		screwing_primitive->_desired_normal_force = 0;
		// screwing_primitive->_desired_velocity = Eigen::Vector3d(0.0, 0.0, -0.2);
	}

	screwing_primitive->computeTorques(command_torques, controller_counter % 500 == 0);
	sim->setJointTorques(robot_name, command_torques);

	Eigen::Matrix3d current_orientation;
	robot->rotation(current_orientation, motion_primitive->_link_name);

	Eigen::Vector3d orientation_threshold;
	orientation_threshold << 0.05,
							 0.05,
							 -0.999;

	Eigen::Vector3d z_orientation = current_orientation.col(2);

	if (z_orientation[0] < orientation_threshold[0] && z_orientation[1] < orientation_threshold[1] && z_orientation[2] < orientation_threshold[2]){
		return FINISHED;
	}

	return RUNNING;
}

ControllerStatus screwing(Sai2Model::Sai2Model* robot, Sai2Primitives::RedundantArmMotion* motion_primitive, Sai2Primitives::ScrewingAlignment* screwing_primitive, Eigen::Vector3d current_position, Eigen::Matrix3d initial_orientation, Eigen::VectorXd command_torques, Simulation::Sai2Simulation* sim, Eigen::Affine3d sensor_frame_in_link){
	
		Eigen::Matrix3d R;
		Eigen::Vector3d T;
		Eigen::Vector3d V;
		double theta;

		if (curr_control_time < 2500){
			motion_primitive->_desired_orientation = initial_orientation;
		}
		else if (theta_deg < MAX_ROTATION)
		{
			theta = -M_PI/2.0/2500.0 * (curr_control_time - 2500);
			theta_deg = -theta*180/M_PI;
			R << cos(theta) , -sin(theta), 0,
		    	 sin(theta) , cos(theta) , 0,
		        	  0     ,      0     , 1;

			motion_primitive->_desired_orientation = R*initial_orientation;
			target_orientation = R*initial_orientation;
		}
		else if (curr_control_time < 16000) //extra buffer time to allow slow simulation to reach target orientation
		{
			motion_primitive->_desired_orientation = target_orientation;				
		}
		else
		{
			return FINISHED;
		}

		motion_primitive->_desired_position = current_position;
		motion_primitive->_desired_velocity = Eigen::Vector3d::Zero();
		motion_primitive->computeTorques(command_torques);
		sim->setJointTorques(robot_name, command_torques);

	return RUNNING;
}

ControllerStatus done(Sai2Primitives::RedundantArmMotion* motion_primitive, Eigen::Vector3d current_position, Eigen::VectorXd command_torques, Simulation::Sai2Simulation* sim){
	motion_primitive->_desired_position = current_position;
	motion_primitive->computeTorques(command_torques);
	sim->setJointTorques(robot_name, command_torques);
	cout << endl;
	cout << "------- DONE -------" << endl;
	sleep(1);
	// exit(EXIT_FAILURE);
	fSimulationRunning = false;
	return FINISHED;
}



/* =======================================================================================
   SIMULATION SETUP
   -----------------------
   * Simulation loop
   * Window initialization
   * Window error
   * Mouse click commands
========================================================================================== */

void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* bottle, ForceSensorSim* fsensor, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// bottle controller
	Eigen::Vector2d bottle_qd = Eigen::Vector2d::Zero();
	Eigen::Vector2d bottle_torques = Eigen::Vector2d::Zero();

	// create a timer
	unsigned long long sim_counter = 0;
	double sim_freq = 2000.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(sim_freq); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		double time = sim_counter/sim_freq;

		// force sensor update
		fsensor->update(sim);
		fsensor->getForce(sensed_force);
		fsensor->getMoment(sensed_moment);

		// bottle controller
		sim->getJointPositions (bottle_name, bottle->_q);
		sim->getJointVelocities (bottle_name, bottle->_dq);
	 	bottle->updateKinematics();

		//desired velocity of the bottle - set to 0
	 		// bottle_qd(0) = 5.0/180.0*M_PI*sin(2*M_PI*0.12*time);
	 		// bottle_qd(1) = 7.0/180.0*M_PI*sin(2*M_PI*0.08*time);
	 	bottle_qd(0) = 0;	//desired velocity of the bottle - set to 0
	 	bottle_qd(1) = 0;

	 	bottle_torques = -1000.0* (bottle->_q - bottle_qd) - 75.0*bottle->_dq;

		sim->setJointTorques (bottle_name, bottle_torques);

		// integrate forward
		sim->integrate(0.0005);

		sim_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
    std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 1.0 * screenH;
    int windowH = 0.7 * screenH;
    int windowPosY = (screenH - windowH*2) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - Bottle Cap Screwing Demo", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
    switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
    }
}

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
		switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}





/* =======================================================================================
   EXTRA SCRAPS OF CODE - expand side bar to see it (should be deleted)
========================================================================================== */

	// ------------------ APPROACH -----------------//
		// initial approach setup using motion primitive, assuming small approach angle and position inaccuracies based on robot vision
		
		// if(controller_counter < 3000)
		// {
		// 	// orientation part
		// 	if(theta_deg <= APPROACH_ANGLE)
		// 	{
		// 		Eigen::Matrix3d R;
		// 		double theta = -M_PI/2.0/500.0 * (controller_counter);
		// 		theta_deg = -theta*180/M_PI;
		// 		R <<      1     ,      0      ,      0     ,
		// 	   		      0     ,  cos(theta) , -sin(theta),
		// 	       		  0     ,  sin(theta) ,  cos(theta);

		// 		motion_primitive->_desired_orientation = R*initial_orientation;
		// 		cout << "......." << endl;
		// 		cout << theta << endl;
		// 		cout << endl;
		// 		cout << theta_deg << endl;
		// 	}

		// 	// position part - would be replaced by point determined by robot vision
		// 	Eigen::Vector3d approach_point;
		// 	approach_point <<  0,
		// 					  -0.11,
		// 					  -0.18;

		// 	motion_primitive->_desired_position = initial_position + approach_point;

		// 	// torques
		// 	motion_primitive->computeTorques(motion_primitive_torques);
		// 	command_torques = motion_primitive_torques;
		// 	sim->setJointTorques(robot_name, command_torques);
		// }
		
	// -----------------------------------------------------------------//	
		// else
		// {

			// screwing_primitive->_desired_orientation = initial_orientation;

			// screwing_primitive->updateSensedForceAndMoment(- R_sensor.transpose() * sensed_force, - R_sensor.transpose() * sensed_moment);


			// screwing_primitive->computeTorques(screwing_primitive_torques, controller_counter % 500 == 0);
			// command_torques = screwing_primitive_torques;
			// sim->setJointTorques(robot_name, command_torques);

			// // criteria for clunk: orientation is flat and moments are zero?


			// robot->rotation(current_orientation, motion_primitive->_link_name);

			// Eigen::Vector3d orientation_threshold;
			// orientation_threshold << 0.05,
			// 						 0.05,
			// 						 -0.999;

			// Eigen::Vector3d z_orientation = current_orientation.col(2);

			// if (z_orientation[0] < orientation_threshold[0] && z_orientation[1] < orientation_threshold[1] && z_orientation[2] < orientation_threshold[2]){
			// 	cout << "TARGET REACHED!" << endl;
			// }


		// if(controller_counter % 500 == 0)
		// 	{ 
		// 		cout << "-------------" << endl;
		// 		cout << "sensed_force" << endl;
		// 		cout << sensed_force << endl; 
		// 		cout << "sensed_moment" << endl;
		// 		cout << sensed_moment << endl; 
		// 		cout << "current_orientation" << endl;
		// 		cout << current_orientation << endl;

		// 	}











		// #define MAX_ROTATION 	270 	// max rotation in degrees
		// #define FORCE_THRESH 	1 		// threshold force to begin screwing

		// if(sensed_force[2] > FORCE_THRESH && force_flag == false)
		// {
		// 	force_flag = true;
		// }

		// Eigen::Matrix3d R;
		// Eigen::Vector3d T;
		// Eigen::Vector3d V;
		// // double theta = -M_PI/2.0/1000.0 * (screw_counter);
		// double theta = -M_PI/2.0/1000.0 * controller_counter;
		// // double theta_deg = theta*180/M_PI;
		// // double theta = APPROACH_ANGLE * M_PI/180.0;
		// // R <<      1     ,      0      ,      0     ,
		// //    	      0     ,  cos(theta) , -sin(theta),
		// //        	  0     ,  sin(theta) ,  cos(theta);

		// 	// R << cos(theta) , -sin(theta), 0,
		//  //    	 sin(theta) , cos(theta) , 0,
		//  //        	  0     ,      0     , 1;
		//     R << cos(theta) , 0 , sin(theta),
		// 	          0     , 1 ,     0     ,
		// 	    -sin(theta) , 0 , cos(theta);

		// T << 0,
		// 	 -0.1,
		// 	 0.5;

	 //    V << 0,
	 //    	 0,
	 //    	 0.1;

		// // screwing_primitive->_desired_position = T + initial_position;
		// // screwing_primitive->_desired_velocity = V;
		// screwing_primitive->_desired_orientation = R*initial_orientation;
		// // screwing_primitive->_desired_angular_velocity = Eigen::Vector3d::Zero();

		// robot->rotation(current_orientation, screwing_primitive->_link_name);


		// if(-theta_deg <= MAX_ROTATION && force_flag == true )

	 //    if (-theta_deg <= MAX_ROTATION)
		// {

		// 	R << cos(theta) , -sin(theta), 0,
		//     	 sin(theta) , cos(theta) , 0,
		//         	  0     ,      0     , 1;

		// 	screwing_primitive->_desired_orientation = R*initial_orientation;
		// 	screw_counter ++;

		// }
		// else
		// {
		// 	screwing_primitive->_desired_orientation = initial_orientation;
		// }
		// screwing_primitive->_desired_angular_velocity = Eigen::Vector3d::Zero();

