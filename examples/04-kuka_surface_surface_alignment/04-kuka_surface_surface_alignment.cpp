/*
 * Example of a controller for a Kuka arm made with the surface surface alignment primitive
 *
 */

/* --------------------------------------------------------------------------------------
   Include Required Libraries and Files
-----------------------------------------------------------------------------------------*/
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <math.h>
#include <stdio.h>
#include <time.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>

// #include "primitives/RedundantArmMotion.h"
#include "primitives/SurfaceSurfaceAlignment.h"
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of Sai2Graphics

#include <signal.h>

/* ----------------------------------------------------------------------------------
	Simulation, Control Loop and Record data Setup
-------------------------------------------------------------------------------------*/
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";
const string plate_file = "resources/plate.urdf";
const string plate_name = "Plate";
const string frame_viz_name = "frameOrigin_viz";

const string camera_name = "camera";

// simulation and control loop
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* plate, ForceSensorSim* fsensor, Simulation::Sai2Simulation* sim);

// control link and position in link
const string link_name = "link6";
// const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0,0.0,0.15);
// const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0,0.2,0.15);
const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.23,0.0,0.15);
const Eigen::Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.05);
Eigen::Vector3d sensed_force;
Eigen::Vector3d sensed_moment;
Eigen::Vector3d pos_ball;
Eigen::Quaterniond frame_viz_ori_Q = Eigen::Quaterniond::Identity();
Eigen::Matrix3d frame_viz_ori_R;
double curr_time;

//---*
// Recording data
//
// Get current date/time, format is YYYY-MM-DD_HH-mm-ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tstruct);

    return buf;
}
// file name for recording data
const string record_file = "../../../src/dataPlotter/Data/dataSurfAlign_pos_" + to_string(pos_in_link[2]) + "_" + currentDateTime() + ".csv";
// helper function to combine data into vector
void recordData(double curr_time, int dof, Eigen::Vector3d sensed_force, Eigen::Vector3d sensed_moment, Eigen::VectorXd command_torques);
// helper function to record data to CSV file
void recordToCSV(Eigen::VectorXd &v, const std::string &filename);
//---*

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
	sim->setCoeffFrictionStatic(0.4);

	// load robots
	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();
	auto robot = new Sai2Model::Sai2Model(robot_file, false,sim->getRobotBaseTransform(robot_name), world_gravity);

	sim->getJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// load plate
	auto plate = new Sai2Model::Sai2Model(plate_file, false, sim->getRobotBaseTransform(plate_name), world_gravity);

	// load simulated force sensor
	Eigen::Affine3d T_sensor = Eigen::Affine3d::Identity();
	T_sensor.translation() = sensor_pos_in_link;
	auto fsensor = new ForceSensorSim(robot_name, link_name, T_sensor, robot);
	auto fsensor_display = new ForceSensorDisplay(fsensor, graphics);

	//Display frame on viz ball
	auto vis_world_ptr = graphics->_world;
	chai3d::cGenericObject* object_ptr;
	for (uint i = 0; i < vis_world_ptr->getNumChildren(); i++) {
		auto temp_ptr = vis_world_ptr->getChild(i);
		if (strcmp(temp_ptr->m_name.c_str(), frame_viz_name.c_str()) == 0) {
			object_ptr = temp_ptr;
		}
	}
	object_ptr->setShowFrame(true, false);

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

	double last_cursorx, last_cursory;

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, plate, fsensor, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

    	fsensor_display->update();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		// Eigen::Quaterniond frame_viz_ori_Q = Eigen::Quaterniond::Identity();
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->updateGraphics(plate_name, plate);
		graphics->updateObjectGraphics(frame_viz_name, pos_ball, frame_viz_ori_Q);
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

/* ----------------------------------------------------------------------------------
	Utility functions
-------------------------------------------------------------------------------------*/
void recordData(double curr_time, int dof, Eigen::Vector3d sensed_force, Eigen::Vector3d sensed_moment, Eigen::VectorXd command_torques)
{
	// 1 value for time, 3 values for forces, 3 values for moments, and dof values for command torques  
	Eigen::VectorXd data = Eigen::VectorXd::Zero(7 + dof);
	data(0) = curr_time;
	
	for (int i = 0; i < 3; i++) {
		data(1 + i) = sensed_force(i);
		data(4 + i) = sensed_moment(i);
	}
	for (int i = 0; i < dof; i++) {
		data(7 + i) = command_torques(i);
	}
	recordToCSV(data, record_file);
}
//------------------------------------------------------------------------------
void recordToCSV(Eigen::VectorXd &v, const std::string &filename)
{
    std::ofstream file(filename,  std::ofstream::out | std::ofstream::app);
    if (file.is_open())
    {
        for(int i=0; i < v.rows(); ++i)
        {
            file << v(i);
            if(i!=v.rows()-1) file << ", ";
        }
        file << "\n";
    }
    //    file.close();
}

/* =======================================================================================
   CONTROL LOOP
========================================================================================== */
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {

	robot->updateModel();
	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);
	Eigen::MatrixXd N_prec = Eigen::MatrixXd::Identity(dof,dof);

	Eigen::Affine3d control_frame_in_link = Eigen::Affine3d::Identity();
	control_frame_in_link.translation() = pos_in_link;
	Eigen::Affine3d sensor_frame_in_link = Eigen::Affine3d::Identity();
	sensor_frame_in_link.translation() = pos_in_link;

	// Motion arm primitive
	Sai2Primitives::SurfaceSurfaceAlignment* surf_alignment_primitive = new Sai2Primitives::SurfaceSurfaceAlignment(robot, link_name, control_frame_in_link, sensor_frame_in_link);
	Eigen::VectorXd surf_alignment_primitive_torques;
	surf_alignment_primitive->enableGravComp();

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	unsigned long long controller_counter = 0;

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		// update tasks model
		N_prec = Eigen::MatrixXd::Identity(dof,dof);
		surf_alignment_primitive->updatePrimitiveModel(N_prec);

		// update ball visualizer position (control point located at: end_effector + pos_in_link)
		robot->position(pos_ball, link_name, pos_in_link);
		robot->rotation(frame_viz_ori_R, link_name);
		frame_viz_ori_Q = frame_viz_ori_R;

		// pos_ball<< surf_alignment_primitive->_control_frame.translation(); //not moving
		// pos_ball << 0, 0.15*sin(2*M_PI*curr_time/3), 0.4;
		// cout << pos_ball << endl;
		// -------------------------------------------
		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		// update sensed values (need to put them back in sensor frame)
		Eigen::Matrix3d R_link;
		robot->rotation(R_link, link_name);
		Eigen::Matrix3d R_sensor = R_link*sensor_frame_in_link.rotation();
		surf_alignment_primitive->updateSensedForceAndMoment(- R_sensor.transpose() * sensed_force, - R_sensor.transpose() * sensed_moment);

		// torques
		surf_alignment_primitive->computeTorques(surf_alignment_primitive_torques);

		//------ Final torques
		command_torques = surf_alignment_primitive_torques;
		// command_torques.setZero();

		// -------------------------------------------
		sim->setJointTorques(robot_name, command_torques);
		

		// -------------------------------------------
		if(controller_counter % 500 == 0)
		{
			// cout << time << endl;
			// cout << endl;
		}

		controller_counter++;

		// -------------------------------------------
		// update last time
		last_time = curr_time;

		//*
		// Recording data
		//*
		// if (curr_time <= 8.0)
		// {
		// 	recordData(curr_time, dof, sensed_force, sensed_moment, command_torques);
		// }
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Control Loop run time  : " << end_time << " seconds\n";
    std::cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}

/* =======================================================================================
   SIMULATION SETUP
   -----------------------
   * Simulation loop
   * Window initialization
   * Window error
   * Mouse click commands
========================================================================================== */
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* plate, ForceSensorSim* fsensor, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// plate controller
	Eigen::Vector2d plate_qd = Eigen::Vector2d::Zero();
	Eigen::Vector2d plate_torques = Eigen::Vector2d::Zero();

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

		// plate controller
		sim->getJointPositions(plate_name, plate->_q);
		sim->getJointVelocities(plate_name, plate->_dq);
		plate->updateKinematics();

		plate_qd(0) = 5.0/180.0*M_PI*sin(2*M_PI*0.12*time);
		plate_qd(1) = 7.0/180.0*M_PI*sin(2*M_PI*0.08*time);

		plate_torques = -1000.0*(plate->_q - plate_qd) - 75.0*plate->_dq;

		sim->setJointTorques(plate_name, plate_torques);

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


//------------------------------------------------------------------------------
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
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW2", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

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