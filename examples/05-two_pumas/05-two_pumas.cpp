/*======================================================================================
 * 05-two_pumas.cpp
 *
 *
 * Elena Galbally & Adrian Piedra (based on: https://github.com/manips-sai/cs327a/blob/split_sai2/hw4_sol/p1-main-sol.cpp)
 *
 *======================================================================================*/

/* --------------------------------------------------------------------------------------
   Include Required Libraries and Files
-----------------------------------------------------------------------------------------*/
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <math.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>

#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"
#include "tasks/PosOriTask.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const double time_slowdown_factor = 10;

const string world_fname = "resources/world_puma.urdf";
const string robot_fname = "resources/puma_gripper.urdf";
const string robot1_name = "Puma1";
const string robot2_name = "Puma2";
const string object_name = "CoordObject";
const string object_fname = "resources/object.urdf";
const string object_link_name = "object";
const string camera_name = "camera_front";
const string ee_link_name = "end-effector";
const string gripper_joint_name = "gripper";

Eigen::Vector3d sensed_force1;
Eigen::Vector3d sensed_force2;
Eigen::Vector3d sensed_moment1;
Eigen::Vector3d sensed_moment2;

/* ----------------------------------------------------------------------------------
	Simulation and Control Loop Setup
-------------------------------------------------------------------------------------*/

// state machine setup
enum ControlMode {
	CONTROL_GRASP_STABILIZE = 0,
	CONTROL_AUGMENTED_OBJECT
};

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* object_model, Simulation::Sai2Simulation* sim);
void simulation(ForceSensorSim* fsensor1, ForceSensorSim* fsensor2, Simulation::Sai2Simulation* sim);

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
	auto robot1 = new Sai2Model::Sai2Model(robot_fname, false);
	auto robot2 = new Sai2Model::Sai2Model(robot_fname, false);

	// load object
	auto coobject = new Sai2Model::Sai2Model(object_fname, false);

	// load simulated force sensor
	Eigen::Affine3d T_sensor = Eigen::Affine3d::Identity();
	// T_sensor.translation() = Eigen::Vector3d(0.0, 0.0, 0.1725);
	auto fsensor1 = new ForceSensorSim(robot1_name, ee_link_name, T_sensor, robot1);
	auto fsensor2 = new ForceSensorSim(robot2_name, ee_link_name, T_sensor, robot2);
	
	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);
	// set co-efficient of restition to zero to avoid bounce
    // see issue: https://github.com/manips-sai/sai2-simulation/issues/1
    sim->setCollisionRestitution(0.0);
    sim->setCoeffFrictionStatic(0.5);
    sim->setCoeffFrictionDynamic(0.5);
    
    // set joint damping on grippers: TODO: the values needed seem to much larger than physical!
    auto base_1 = sim->_world->getBaseNode(robot1_name);
    auto gripper_1 = base_1->getJoint(gripper_joint_name);
    gripper_1->setDamping(10.0);
    gripper_1->setJointLimits(-0.005, 0.068, 0.005);
    auto base_2 = sim->_world->getBaseNode(robot2_name);
    auto gripper_2 = base_2->getJoint(gripper_joint_name);
    gripper_2->setDamping(10.0);
    gripper_2->setJointLimits(-0.005, 0.068, 0.005);

    // set initial conditions
	robot1->_q << 90/180.0*M_PI,
				-22.5/180.0*M_PI,
				212/180.0*M_PI,
				90.0/180.0*M_PI,
				100/180.0*M_PI,
				180/180.0*M_PI,
				0.0;
	sim->setJointPositions(robot1_name, robot1->_q);
	robot1->updateModel();
	robot2->_q << 90/180.0*M_PI,
				202.5/180.0*M_PI,
				-28/180.0*M_PI,
				-90.0/180.0*M_PI,
				97/180.0*M_PI,
				180/180.0*M_PI,
				0.0;
	sim->setJointPositions(robot2_name, robot2->_q);
	robot2->updateModel();
	Eigen::Affine3d ee_trans;
	// robot1->transform(ee_trans, ee_link_name);
	// cout << ee_trans.translation().transpose() << endl;
	// cout << ee_trans.rotation() << endl;

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation thread first
    fSimulationRunning = true;
	thread sim_thread(simulation, fsensor1, fsensor2, sim);

	// next start the control thread
	thread ctrl_thread(control, robot1, robot2, coobject, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot1_name, robot1);
		graphics->updateGraphics(robot2_name, robot2);
		graphics->updateGraphics(object_name, coobject);
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

/* ----------------------------------------------------------------------------------
	Utility functions
-------------------------------------------------------------------------------------*/

// Calculate the cross product matrix
Eigen::Matrix3d getCrossProductMat(const Eigen::Vector3d& t) {
	Eigen::Matrix3d ret;
	ret <<  0, -t(2), t(1),
			t(2), 0, -t(0),
			-t(1), t(0), 0;
	return ret;
}


/* =======================================================================================
   CONTROL LOOP
========================================================================================== */
void control(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* object_model, Simulation::Sai2Simulation* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer
	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs

	Sai2Primitives::PosOriTask* posori_task1 = new Sai2Primitives::PosOriTask(robot1, ee_link_name, Eigen::Vector3d::Zero());
	Sai2Primitives::PosOriTask* posori_task2 = new Sai2Primitives::PosOriTask(robot2, ee_link_name, Eigen::Vector3d::Zero());

	// load robot global frame to robot base transformations: todo: move out to a function
	Eigen::Affine3d robot1_base_frame = sim->getRobotBaseTransform(robot1_name);
	Eigen::Affine3d robot2_base_frame = sim->getRobotBaseTransform(robot2_name);

	// cache variables
	bool fTimerDidSleep = true;
	bool fTorqueUseDefaults = false; // set true when torques are overriden for the first time
	Eigen::VectorXd tau1 = Eigen::VectorXd::Zero(robot1->dof());
	Eigen::VectorXd tau2 = Eigen::VectorXd::Zero(robot2->dof());

	Eigen::Affine3d object_com_frame;
	Eigen::MatrixXd object_inertia(6,6);
	Eigen::MatrixXd object_j(6,6);
	Eigen::VectorXd object_p(6);

	Eigen::VectorXd robot1_g(robot1->dof());
	Eigen::VectorXd robot2_g(robot2->dof());
	Eigen::MatrixXd robot1_j0_ee(6, robot1->dof());
	Eigen::MatrixXd robot2_j0_ee(6, robot2->dof());

	Eigen::Affine3d robot1_ee_frame_local;
	Eigen::Affine3d robot2_ee_frame_local;
	Eigen::Vector3d robot1_object_com_frame_local;
	Eigen::Vector3d robot2_object_com_frame_local;
	Eigen::MatrixXd robot1_j0_objcom(6, robot1->dof());
	Eigen::MatrixXd robot2_j0_objcom(6, robot2->dof());
	Eigen::MatrixXd robot1_j0_objectcom_bar(robot1->dof(), 6);
	Eigen::MatrixXd robot2_j0_objectcom_bar(robot2->dof(), 6);
	Eigen::MatrixXd robot1_objcom_inertia(6,6);
	Eigen::MatrixXd robot2_objcom_inertia(6,6);
	Eigen::VectorXd robot1_objectcom_p(6);
	Eigen::VectorXd robot2_objectcom_p(6);

	Eigen::MatrixXd augmented_object_inertia(6,6);
	Eigen::VectorXd augmented_object_p(6);

	Eigen::MatrixXd G(2*6, 2*6);
	Eigen::MatrixXd W(6, 2*6);

	Eigen::Vector3d obj_des_pos;
	Eigen::Vector3d obj_ori_error;
	Eigen::VectorXd obj_task_err(6);
	Eigen::VectorXd force_des_vec(12);
	Eigen::VectorXd force_ee_vec(12);

	Eigen::VectorXd force_obj_act_vec(12);
	Eigen::VectorXd force_ee_act_vec(12);

	double kp = 30;
	double kv = 10;

	// control mode
	ControlMode control_mode = CONTROL_GRASP_STABILIZE; // 0 = grasp stabilizing controller, 1 = augmented object controller

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time;

        // read joint positions, velocities
        sim->getJointPositions(robot1_name, robot1->_q);
        sim->getJointVelocities(robot1_name, robot1->_dq);
        robot1->updateModel();
        sim->getJointPositions(robot2_name, robot2->_q);
        sim->getJointVelocities(robot2_name, robot2->_dq);
        robot2->updateModel();

        // read object position
        sim->getJointPositions(object_name, object_model->_q);
        sim->getJointVelocities(object_name, object_model->_dq);
        object_model->updateModel();

        // update object dynamics
		// - find object COM frame in global frame
		object_model->transform(object_com_frame, object_link_name);
		// - obtain object inertia matrix in world frame
		object_model->J_0(object_j, object_link_name, Eigen::Vector3d::Zero());
		object_inertia = (object_j*object_model->_M_inv*object_j.transpose()).inverse();
		// - obtain object p
		object_p << 0, 0, 9.8*0.5, 0, 0, 0;

        // ---------------------------------------------------------------------------------
        /* ---------------------- Manipulator Jacobians ---------------------- */
		// - Obtain object COM location in robot1 end effector frame
		robot1->transform(robot1_ee_frame_local, ee_link_name);
		robot1_object_com_frame_local = ((robot1_base_frame*robot1_ee_frame_local).inverse() * object_com_frame).translation();

		// - Obtain object COM location in robot2 end effector frame
		robot2->transform(robot2_ee_frame_local, ee_link_name);
		robot2_object_com_frame_local = ((robot2_base_frame*robot2_ee_frame_local).inverse() * object_com_frame).translation();

		// - Obtain robot1 EE Jacobian at the object COM in world frame
		robot1->J_0(robot1_j0_objcom, ee_link_name, robot1_object_com_frame_local);
		
		// - Obtain robot1 EE Jacobian at the object COM in world frame
		robot1->J_0(robot2_j0_objcom, ee_link_name, robot2_object_com_frame_local);

		/* ---------------------- Augmented object model --------------------- */
		robot1_objcom_inertia = (robot1_j0_objcom*robot1->_M_inv*robot1_j0_objcom.transpose()).inverse();
		robot2_objcom_inertia = (robot2_j0_objcom*robot2->_M_inv*robot2_j0_objcom.transpose()).inverse();

		// - Add it all up
		augmented_object_inertia = object_inertia + robot1_objcom_inertia + robot2_objcom_inertia;

		// - Obtain robot1 g, j_objectcom_bar, p
		robot1->gravityVector(robot1_g);
		robot1_j0_objectcom_bar = robot1->_M_inv*robot1_j0_objcom.transpose()*robot1_objcom_inertia;
		robot1_objectcom_p = robot1_j0_objectcom_bar.transpose()*robot1_g;

		// - Obtain robot2 g, j_objectcom_bar, p
		robot2->gravityVector(robot2_g);
		robot2_j0_objectcom_bar = robot2->_M_inv*robot2_j0_objcom.transpose()*robot2_objcom_inertia;
		robot2_objectcom_p = robot2_j0_objectcom_bar.transpose()*robot2_g;

		// - Add it all up
		augmented_object_p = object_p + robot1_objectcom_p + robot2_objectcom_p;

		/* ---------------------- Grasp matrix ---------------------- */
		// NOTE: we calculate everything in the world frame. But any frame is good enough as
		// long as all quantities are calculated in the same frame.

		// - Get W
		W.setZero();
		W.block(0,0,3,3) = Eigen::Matrix3d::Identity();
		W.block(0,6,3,3) = Eigen::Matrix3d::Identity();
		W.block(3,3,3,3) = Eigen::Matrix3d::Identity();
		W.block(3,9,3,3) = Eigen::Matrix3d::Identity();
		// - - set r1: from object COM to robot 1 end effector
		W.block(3,0,3,3) = getCrossProductMat((robot1_base_frame*robot1_ee_frame_local).translation() - object_com_frame.translation());
		// - - set r2: from object COM to robot 2 end effector
		W.block(3,6,3,3) = getCrossProductMat((robot2_base_frame*robot2_ee_frame_local).translation() - object_com_frame.translation());

		// - Get G
		G.setZero();
		G.block(0,0,6,2*6) = W;
		// - set tension component
		Eigen::Vector3d e12 = (robot2_base_frame*robot2_ee_frame_local).translation() - (robot1_base_frame*robot1_ee_frame_local).translation();
		e12.normalize();
		G.block(6,0,1,3) = -0.5*e12.transpose(); // for tension between the end-effectors, use -e12, for compression, use e12
		G.block(6,6,1,3) = 0.5*e12.transpose();
		// - set moment components at end effector normal to the ee-ee axis
		Eigen::Vector3d e1_z = (robot1_ee_frame_local).linear().col(2);// set to end effector z-axis
		Eigen::Vector3d e1_y = e1_z.cross(e12);
		G.block(7,3,1,3) = e1_z.transpose();
		G.block(8,3,1,3) = e1_y.transpose();
		Eigen::Vector3d e2_z = (robot2_ee_frame_local).linear().col(2);// set to end effector z-axis
		Eigen::Vector3d e2_y = e2_z.cross(e12);
		G.block(9,9,1,3) = e2_z.transpose();
		G.block(10,9,1,3) = e2_y.transpose();
		// - set twist along ee-ee link axis
		G.block(11,3,1,3) = -0.5*e12.transpose();
		G.block(11,9,1,3) = 0.5*e12.transpose();

		// update sensed values (need to put them back in sensor frame)
		Eigen::Affine3d sensor_frame_in_link = Eigen::Affine3d::Identity();
		
		Eigen::Matrix3d R_link1;
		robot1->rotation(R_link1, ee_link_name);
		Eigen::Matrix3d R_sensor1 = R_link1*sensor_frame_in_link.rotation();
		posori_task1->updateSensedForceAndMoment(- R_sensor1.transpose() * sensed_force1, - R_sensor1.transpose() * sensed_moment1);

		Eigen::Matrix3d R_link2;
		robot2->rotation(R_link2, ee_link_name);
		Eigen::Matrix3d R_sensor2 = R_link2*sensor_frame_in_link.rotation();
		posori_task2->updateSensedForceAndMoment(- R_sensor2.transpose() * sensed_force2, - R_sensor2.transpose() * sensed_moment2);

		/* ---------------------- Force control ---------------------- */
		if (control_mode == CONTROL_AUGMENTED_OBJECT) {
			
			// - desired object position and current task error
			// obj_des_pos << 0, 0.4,0.15*sin(2*M_PI*curr_time/3);
			// obj_des_pos << 0, 0.15*sin(2*M_PI*curr_time/3), 0.4;
			obj_des_pos << 0.15*cos(2*M_PI*curr_time/3), 0.15*sin(2*M_PI*curr_time/3), 0.4;
			// obj_des_pos << 0, 0, 0;

			Sai2Model::orientationError(obj_ori_error, Eigen::Matrix3d::Identity(), object_com_frame.linear());
			obj_task_err << (object_com_frame.translation() - obj_des_pos), obj_ori_error;

			// - obtain desired force at augmented object location
			force_des_vec.setZero();
			force_des_vec.head(6) = object_p + augmented_object_inertia*(-kv*object_j*object_model->_dq - kp*(obj_task_err));
			force_des_vec[6] = -15.0; // 15N compression // 20N causes a bus error in Sai2-simulation!

			// - solve for ee forces
			force_ee_vec = G.lu().solve(force_des_vec);

			// - set joint torques
			robot1->J_0(robot1_j0_ee, ee_link_name, Eigen::Vector3d::Zero());
			robot2->J_0(robot2_j0_ee, ee_link_name, Eigen::Vector3d::Zero());
			tau1 = robot1_j0_ee.transpose()*force_ee_vec.head(6) + robot1_g;
			tau2 = robot2_j0_ee.transpose()*force_ee_vec.tail(6) + robot2_g;

		} else if (control_mode == CONTROL_GRASP_STABILIZE) { // initial grasp stabilization
			robot1->J_0(robot1_j0_ee, ee_link_name, Eigen::Vector3d::Zero());
			robot2->J_0(robot2_j0_ee, ee_link_name, Eigen::Vector3d::Zero());
			tau1 = robot1_j0_ee.transpose()*(object_p/2) + robot1->_M*(-10.0*robot1->_dq) + robot1_g;
			tau2 = robot2_j0_ee.transpose()*(object_p/2) + robot2->_M*(-10.0*robot2->_dq) + robot2_g;
			static uint grasp1Counter = 0;
			static uint grasp2Counter = 0;
			if (robot1->_dq[6] < 0.1) {
				grasp1Counter += 1;
			} else {
				grasp1Counter = 0;
			}
			if (robot2->_dq[6] < 0.1) {
				grasp2Counter += 1;
			} else {
				grasp2Counter = 0;
			}
			if (grasp1Counter > 40 && grasp2Counter > 40) {
				cout << "----- Switch Control Mode to Augmented Object Model -----" << endl;
				control_mode = CONTROL_AUGMENTED_OBJECT;
			}
		}

		// -------------------------------------------
		// set constant gripper forces.
		tau1[6] = 15;
		tau2[6] = 15;

        // Default values if torques are exceeded:
        bool fTorqueOverride = false; // to avoid robot blow-ups
        const double tau1_max = 200;
        const double tau2_max = 200;
        if (!fTorqueUseDefaults) {
        	if (tau1.cwiseAbs().maxCoeff() > tau1_max || tau2.cwiseAbs().maxCoeff() > tau2_max) {
	        	fTorqueOverride = true;
	        	cerr << "Torque overriden. User asked torques beyond safe limit: \n";
	        	cerr << "Robot 1: " << tau1.transpose() << "\n";
	        	cerr << "Robot 2: " << tau2.transpose() << "\n";
	        	fTorqueUseDefaults = true;
	        }
	        // Also default values if object is dropped
	        const double object_thickness = 0.05;
	        bool fRobot1DroppedObject = robot1->_q[6] > object_thickness/2;
	        bool fRobot2DroppedObject = robot2->_q[6] > object_thickness/2;
	        if (fRobot1DroppedObject || fRobot2DroppedObject) {
	        	cerr << "Torque overriden. Robot 1 or 2 dropped object. \n";
	        	fTorqueUseDefaults = true;
	        }
        }
        else {
        	robot1->gravityVector(tau1);
			tau1 = tau1 + robot1->_M*(-10.0*robot1->_dq);
			tau1 = (tau1.array() >= tau1_max).select(tau1_max*Eigen::VectorXd::Ones(robot1->dof()), tau1);
			tau1 = (tau1.array() <= -tau1_max).select(-tau1_max*Eigen::VectorXd::Ones(robot1->dof()), tau1);
			robot2->gravityVector(tau2);
			tau2 = tau2 + robot2->_M*(-10.0*robot2->_dq);
			tau2 = (tau2.array() >= tau2_max).select(tau2_max*Eigen::VectorXd::Ones(robot2->dof()), tau2);
			tau2 = (tau2.array() <= -tau2_max).select(-tau2_max*Eigen::VectorXd::Ones(robot2->dof()), tau2);
        }

		sim->setJointTorques(robot1_name, tau1);
		sim->setJointTorques(robot2_name, tau2);
		
		// compute the actual internal object forces from sensed force and moment
		force_ee_act_vec << sensed_force1, sensed_force2, sensed_moment1, sensed_moment2;
		force_obj_act_vec = G * force_ee_act_vec;

		// cout << force_obj_act_vec[6] << endl;

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
void simulation(ForceSensorSim* fsensor1, ForceSensorSim* fsensor2, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(10000); //10000Hz timer

	// sleep for a few moments to let graphics start
	// std::this_thread::sleep_for(std::chrono::seconds(1));	
	
	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs
	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();
		// if (timer.elapsedCycles() % 10000 == 0) {
		// 	cout << "Simulation loop frequency: " << timer.elapsedCycles()/timer.elapsedTime() << endl; 
		// }

		// force sensor update
		fsensor1->update(sim);
		fsensor1->getForce(sensed_force1);
		fsensor1->getMoment(sensed_moment1);

		// force sensor update
		fsensor2->update(sim);
		fsensor2->getForce(sensed_force2);
		fsensor2->getMoment(sensed_moment2);

		// integrate forward
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time; 
		// sim->integrate(loop_dt);
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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW4", NULL, NULL);
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