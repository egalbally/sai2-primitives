/*======================================================================================
 * hw2-p2-main-sol.cpp
 *
 * Implement operational space motion tracking for a 7-DOF redundant kuka 
 * by completing the "FILL ME IN" section 
 *
 * Elena Galbally, Spring 2019
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

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> 

using namespace std;

const string world_fname = "resources/hw2/world_2_iiwa.urdf";
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";
const string ee_link_name = "link6";
const string camera_name = "camera_top";

/* --------------------------------------------------------------------------------------
	Simulation Loop Initialization
-------------------------------------------------------------------------------------*/
// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);


/* =======================================================================================
   MAIN LOOP
========================================================================================== */
int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);

	// set initial condition
	robot->_q << 125.9/180.0*M_PI,
				39.2/180.0*M_PI,
				-49.2/180.0*M_PI,
				70.0/180.0*M_PI,
				-62.4/180.0*M_PI,
				80.2/180.0*M_PI,
				187.2/180.0*M_PI;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
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
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); //200Hz timer
	double last_time = timer.elapsedTime(); //secs

	// cache variables
	bool fTimerDidSleep = true;
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->dof());

	// additional cache variables
	Eigen::VectorXd g(robot->dof()); //joint space gravity vector
	Eigen::MatrixXd J0(6, robot->dof()); //end effector basic Jacobian
	Eigen::MatrixXd L0(6, 6); //Lambda_0 at end effector
	Eigen::VectorXd p(6); //gravity vector at end-effector
	const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(robot->dof(), robot->dof()); // n x n identity matrix
	Eigen::MatrixXd Jbar(robot->dof(), 6);	//A-weighted generalized inverse of J0
	Eigen::MatrixXd Nbar(robot->dof(), robot->dof()); //I - Jbar*J0, null space projection matrix for Jbar
	Eigen::Vector3d ee_pos; //end effector position
	Eigen::Matrix3d ee_rot_mat; //end effector rotation
	robot->rotation(ee_rot_mat, ee_link_name); // initialize
	Eigen::Quaterniond ee_rot_lambda(ee_rot_mat); // end effector rotation in quaternion form
	Eigen::VectorXd ee_error(6); //end effector operational space instantaneous error
	Eigen::VectorXd v0(6); //end effector velocity
	Eigen::VectorXd v0d(6); //end effector desired velocity
	Eigen::VectorXd dv0d(6); //end effector desired acceleration

	// gains
	double kpj = 50;
	double kvj = 20;
	double kpx = 50;
	double kvx = 20;

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		/* ------------------------------------------------------------------------------------
			FILL ME IN: set joint torques
		-------------------------------------------------------------------------------------*/

		// ---------------------------------------------------------------------------------
		// (1) Update current robot configuration parameters and desired trajectory values 
		//----------------------------------------------------------------------------------

		// Update joint space gravity
		robot->gravityVector(g);

		// Update current end effector position, orientation
		robot->position(ee_pos, ee_link_name, Eigen::Vector3d::Zero());
		robot->rotation(ee_rot_mat, ee_link_name);

		// Update operational space dynamics
		robot->J_0(J0, ee_link_name, Eigen::Vector3d::Zero());
		L0 = (J0 * robot->_M_inv * J0.transpose()).inverse();
		Jbar = robot->_M_inv * J0.transpose() * L0;
		Nbar = In - Jbar*J0;
		p = Jbar.transpose()*g;

		// Update current end effector velocity (v,w)
		v0 = J0*robot->_dq; 

		// Update desired end-effector position, velocity and acceleration
		double R = 0.1; //m
		double T = 5.0; //sec
		double xd = 0.0;
		double yd = 0.5 + R*cos(2.0*M_PI*curr_time/T);
		double zd = 0.6 + R/2.0 - R/2.0*cos(4.0*M_PI*curr_time/T);

		Eigen::Quaterniond lambdad (
			1/sqrt(2)*sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T)),
			1/sqrt(2)*cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T)),
			1/sqrt(2)*sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T)),
			1/sqrt(2)*cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T))
		);

		double dxd = 0.0;
		double dyd = -2.0*M_PI*R/T*sin(2.0*M_PI*curr_time/T);
		double dzd = 2.0*M_PI*R/T*sin(4.0*M_PI*curr_time/T);

		Eigen::Quaterniond dlambdad (
			-pow(M_PI, 2)/(2*T*sqrt(2.0))*cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T))*sin(2.0*M_PI*curr_time/T),
			pow(M_PI, 2)/(2*T*sqrt(2.0))*sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T))*sin(2.0*M_PI*curr_time/T),
			-pow(M_PI, 2)/(2*T*sqrt(2.0))*cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T))*sin(2.0*M_PI*curr_time/T),
			pow(M_PI, 2)/(2*T*sqrt(2.0))*sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T))*sin(2.0*M_PI*curr_time/T)
		);

		double ddxd = 0.0;
		double ddyd = -4.0*M_PI*M_PI/T*R/T*cos(2.0*M_PI*curr_time/T);
		double ddzd = 8.0*M_PI*M_PI/T*R/T*cos(4.0*M_PI*curr_time/T);

		Eigen::Quaterniond ddlambdad(
			1/sqrt(2)*(-sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(-pow(M_PI, 2)/(2.0*T)*sin(2.0*M_PI*curr_time/T), 2) - cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(M_PI, 3)/(T*T)*cos(2.0*M_PI*curr_time/T)),
			1/sqrt(2)*(-cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(-pow(M_PI, 2)/(2.0*T)*sin(2.0*M_PI*curr_time/T), 2) + sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(M_PI, 3)/(T*T)*cos(2.0*M_PI*curr_time/T)),
			1/sqrt(2)*(-sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(-pow(M_PI, 2)/(2.0*T)*sin(2.0*M_PI*curr_time/T), 2) - cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(M_PI, 3)/(T*T)*cos(2.0*M_PI*curr_time/T)),
			1/sqrt(2)*(-cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(-pow(M_PI, 2)/(2.0*T)*sin(2.0*M_PI*curr_time/T), 2) + sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(M_PI, 3)/(T*T)*cos(2.0*M_PI*curr_time/T))
		);

		// --------------------------------------------------------------------
		// (2) Compute desired operational space trajectory values and errors 
		//---------------------------------------------------------------------

		// - Convert end effector rotation matrix to quaternion for computing 
		//   the end effector orientation error and desired op space v, dv 

		// 			*** Note *** They way eigen does this conversion leads to some sign issues (see pg.17 of the course reader)
		//			So, we have to check if the biggest element in the quaternion has changed sign to make 
		//			sure the conversion from rotation matrix to quaternion is correct

		Eigen::Quaterniond temp_ee_rot_lambda = Eigen::Quaterniond(ee_rot_mat); // new rotation quaternion
		Eigen::VectorXd::Index max_ind = 0;
		Eigen::VectorXd temp_vec = temp_ee_rot_lambda.coeffs().cwiseAbs(); // get absolute values
		temp_vec.cwiseMax(max_ind); // guaranteed to be > 0.5

		// ---------------------------------- sign check ---------------------------------//
		if ((temp_ee_rot_lambda.coeffs())[max_ind]*(ee_rot_lambda.coeffs())[max_ind] < 0) // Flip sign
		{	
			// ee_rot_lambda = -temp_ee_rot_lambda; <-- Eigen does not allow this unfortunately
			ee_rot_lambda = Eigen::Quaterniond(
				-temp_ee_rot_lambda.w(),
				-temp_ee_rot_lambda.x(),
				-temp_ee_rot_lambda.y(),
				-temp_ee_rot_lambda.z() );
		} 
		else // do NOT flip sign (eigen did it right)
		{
			ee_rot_lambda = temp_ee_rot_lambda;
		}
		// ---------------------------------- sign check end ---------------------------------//

		// - Compute desired operational space velocity and acceleration 
		v0d << dxd, dyd, dzd, 2*(dlambdad * ee_rot_lambda.conjugate()).vec();
		dv0d << ddxd, ddyd, ddzd, 2*(ddlambdad * ee_rot_lambda.conjugate()).vec();

		// - Compute current position and orientation error 
		//			*** Note *** For the orientation  --> delta_phi = -Er+*labmdad)
		//			remember from hw1: Er+*labmdad = 2 * labmdad * lambda.conjugate()
		ee_error << (ee_pos - Eigen::Vector3d(xd, yd, zd)), -2*(lambdad * ee_rot_lambda.conjugate()).vec();


		// ---------------------------------------------------------------------------------
		// (3) Compute joint torques
		//----------------------------------------------------------------------------------
		tau = J0.transpose()*(L0*(dv0d - kpx*ee_error - kvx*(v0 - v0d)) + p) +  // PD control in op. space to track trajectory
			Nbar.transpose()*robot->_M*(-kvj*robot->_dq + g); // joint space damping and gravity comp. in the null space

		sim->setJointTorques(robot_name, tau);
		// -------------------------------------------

		// update last time
		last_time = curr_time;
	}
}


/* =======================================================================================
   SIMULATION SETUP
   -----------------------
   * Simulation loop
   * Window initialization
   * Window error
   * Mouse click commands
========================================================================================== */
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer
	double last_time = timer.elapsedTime(); //secs

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update last time
		last_time = curr_time;
	}
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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW2 - SOLUTION", NULL, NULL);
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
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
}
