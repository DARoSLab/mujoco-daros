#ifndef SIMULATION_BRIDGE_H
#define SIMULATION_BRIDGE_H

#include<stdbool.h> 

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "stdio.h"
#include "stdlib.h"
#include <string>
#include <chrono>
#include <iostream>


using namespace std;

class MujocoSimulationBridge{

    public:
    
   // virtual void control_callback(const mjModel* m, mjData* d) = 0;  // Virtual function to be overridden

  
    MujocoSimulationBridge(const string mj_xml_file){

        _mjModel=mj_loadXML((mj_xml_file).c_str(), NULL, error, 1000);
        _mjData=mj_makeData(_mjModel);
        // mj_resetDataKeyframe(_model, _mjDat, 0);
        
    }
    virtual ~MujocoSimulationBridge(){
        cout<<"Mujoco sim destructor"<<endl;
        mjv_freeScene(&mj_scene);
        mjr_freeContext(&mj_context);

        mj_deleteData(_mjData);
        mj_deleteModel(_mjModel);
        
        glfwTerminate();
    }
    
    void init_mj_viz(){
        if( !glfwInit() )
            mju_error("Could not initialize GLFW");

        glfwWindowHint(GLFW_SAMPLES, 4);
        mj_window = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
        
        if (!mj_window) {
            mju_error("Could not create GLFW window!");
            glfwTerminate();
            std::exit(1);
        }
        
        
        glfwMakeContextCurrent(mj_window);
        glfwSwapInterval(1);

        mjv_defaultCamera(&mj_cam);
        mjv_defaultOption(&viz_opt);
        mjv_defaultScene(&mj_scene);
        mjr_defaultContext(&mj_context);
        mjv_makeScene(_mjModel, &mj_scene, 2000);                
        mjr_makeContext(_mjModel, &mj_context, mjFONTSCALE_150);

        // mj_cam.type = mjCAMERA_TRACKING;  // Tracking camera (optional)
        // mj_cam.trackbodyid = 0;           // The body id to track (e.g., the robot base)
        // mj_cam.distance = 3.0;            // Distance from the target
        // mj_cam.azimuth = 90;              // Azimuth angle
        // mj_cam.elevation = -20;           // Elevation angle
        // mj_cam.lookat[0] = _mjDat->qpos[0];    // X position to look at (center of the robot)
        // mj_cam.lookat[1] = _mjDat->qpos[1];    // Y position to look at
        // mj_cam.lookat[2] = _mjDat->qpos[2] + 1.0; // Z position to look at (e.g., robot's height + offset)
        mj_cam.azimuth = 90;
        mj_cam.elevation = -30;
        mj_cam.distance = 5.0;  // Adjust based on your scene
        mj_cam.lookat[0] = 0.0;
        mj_cam.lookat[1] = 0.0;
        mj_cam.lookat[2] = 1.0;

        ;
        // glfwSetWindowUserPointer(mj_window, this);
        // glfwSetCursorPosCallback(mj_window, mouse_move_callback);
        // glfwSetScrollCallback(mj_window, scroll);
         glfwSetWindowUserPointer(mj_window, this);

        //  Set the scroll callback
        glfwSetScrollCallback(mj_window, scrollCallback);

    }

    void run(){
        init_mj_viz();
        // mjcb_control = [](const mjModel* m, mjData* d) {  // Lambda to redirect to the correct control callback
        //     MujocoSimulationBridge* instance = reinterpret_cast<MujocoSimulationBridge*>(d->userdata);
        //     instance->control_callback(m, d);
        // };

        _mjModel->opt.gravity[0] = 0.0;    // Gravity in the X-axis
        _mjModel->opt.gravity[1] = 0.0;    // Gravity in the Y-axis
        _mjModel->opt.gravity[2] = -9.81; 

        //set contact options
        _mjModel->opt.jacobian = mjJAC_DENSE;
        _mjModel->opt.cone = mjCONE_ELLIPTIC; 

         while( !glfwWindowShouldClose(mj_window))
        {
            // advance interactive simulation for 1/60 sec
            //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
            //  this loop will finish on time for the next frame to be rendered at 60 fps.
            //  Otherwise add a cpu timer and exit this loop when it is time to render.
            mjtNum simstart = _mjData->time;
            while( _mjData->time - simstart < 1.0/60.0 )
            {
                _onestep_simulation();
            }

            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(mj_window, &viewport.width, &viewport.height);

            mjv_updateScene(_mjModel, _mjData, &viz_opt, NULL, &mj_cam, mjCAT_ALL, &mj_scene);
            mjr_render(viewport, &mj_scene, &mj_context);

            glfwSwapBuffers(mj_window);

            glfwPollEvents();
            // Get current mouse position
            double xpos, ypos;
            glfwGetCursorPos(mj_window, &xpos, &ypos);

            // Call the function to handle rotation and panning
            handleMouseMovement(mj_window, xpos, ypos, _mjModel, mj_scene, mj_cam);
            glfwSwapBuffers(mj_window);
           
        }
    }


void handleMouseMovement(GLFWwindow* window, double xpos, double ypos, mjModel* m, mjvScene& scn, mjvCamera& cam) {
    static double lastx = 0, lasty = 0;
    static bool button_left = false, button_right = false;

    // Update the state of the mouse buttons
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        button_left = true;
    } else {
        button_left = false;
    }

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        button_right = true;
    } else {
        button_right = false;
    }

    // If no buttons are pressed, do nothing
    if (!button_left && !button_right) {
        lastx = xpos;
        lasty = ypos;
        return;
    }

    // Compute the mouse displacement
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // Get the current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // Determine action based on the mouse button pressed
    mjtMouse action;
    if (button_right) {
        action = mjMOUSE_MOVE_V; // Vertical pan when right button is pressed
    } else if (button_left) {
        action = mjMOUSE_ROTATE_V; // Rotate when left button is pressed
    }

    // Move the camera based on mouse input
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void handleScroll(GLFWwindow* window, double xoffset, double yoffset, mjModel* m, mjvScene& scn, mjvCamera& cam) {
    // Adjust the camera zoom
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}
    
protected:
    mjModel* _mjModel=nullptr;
    mjData* _mjData=nullptr;
        
    char error[1000];

    mjvCamera mj_cam;                      
    mjvOption viz_opt;                      
    mjvScene mj_scene;                       
    mjrContext mj_context;
    GLFWwindow* mj_window;


    // Update State
    virtual void _onestep_simulation() = 0;
    // Update System State and find control input
private:

        bool button_left = false;
        bool button_middle = false;
        bool button_right =  false;
        double lastx = 0;
        double lasty = 0;

        static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset){
            MujocoSimulationBridge* instance = static_cast<MujocoSimulationBridge*>(glfwGetWindowUserPointer(window));
            instance->handleScroll(window, xoffset, yoffset, instance->_mjModel, instance->mj_scene, instance->mj_cam);
        }
};

#endif
