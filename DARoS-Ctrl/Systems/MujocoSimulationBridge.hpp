#ifndef SIMULATION_BRIDGE_H
#define SIMULATION_BRIDGE_H

#include <stdbool.h> 
#include <Configuration.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "stdio.h"
#include "stdlib.h"
#include <string>
#include <chrono>
#include <iostream>
#include <System.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ParamHandler/ParamHandler.hpp>

#include "array_safety_daros.h"
namespace mju = ::mujoco::sample_util;
using namespace std;

enum SimMode{
  PLAY = 0,
  RECORD = 1
};

// Command for recording
// ffmpeg -f rawvideo -pixel_format rgb24 -video_size 1200x900 -framerate 60 -i rgb.out -vf "vflip,format=yuv420p" video.mp4
// ffmpeg -f rawvideo -pixel_format rgb24 -video_size 2560x1440 -framerate 60 -i rgb.out -vf "vflip,format=yuv420p" video.mp4

class MujocoSimulationBridge{
  public:
    MujocoSimulationBridge(System<double> * sys, const std::string & setup_file):_system(sys) {
      _ReadConfig(setup_file);
      _mjModel=mj_loadXML(_mj_xml_file.c_str(), NULL, error, 1000);
      _mjData=mj_makeData(_mjModel);
      mj_forward(_mjModel, _mjData);
      mj_resetDataKeyframe(_mjModel, _mjData, 0);
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
        if( !glfwInit() ) mju_error("Could not initialize GLFW");

        if(_sim_mode == SimMode::RECORD){
          glfwWindowHint(GLFW_VISIBLE, 0);
          glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
        }else{
          glfwWindowHint(GLFW_SAMPLES, 4);
        }
        mj_window = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
        // mj_window = glfwCreateWindow(800, 800, "Invisible window", NULL, NULL);
        
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

        // glfwSetCursorPosCallback(mj_window, mouse_move_callback);
        glfwSetWindowUserPointer(mj_window, this);

        //  Set the scroll callback
        glfwSetScrollCallback(mj_window, scrollCallback);

    }

    void run(){
      init_mj_viz();

      _mjModel->opt.gravity[0] = 0.0;    // Gravity in the X-axis
      _mjModel->opt.gravity[1] = 0.0;    // Gravity in the Y-axis
      _mjModel->opt.gravity[2] = -9.81; 

      //set contact options
      _mjModel->opt.jacobian = mjJAC_DENSE;
      _mjModel->opt.cone = mjCONE_ELLIPTIC; 
      
      double frametime = 0;
      int framecount = 0;
      double fps = 60;

      // get size of active renderbuffer
      mjrRect viewport =  mjr_maxViewport(&mj_context);
      int W = viewport.width;
      int H = viewport.height;

      printf("Resolution: %dx%d. Please use these number when converting.....\n", mj_context.offWidth, mj_context.offHeight);
      // allocate rgb and depth buffers
      unsigned char* rgb = (unsigned char*)std::malloc(3*W*H);
      float* depth = (float*)std::malloc(sizeof(float)*W*H);
      if (!rgb || !depth) { mju_error("Could not allocate buffers"); }

      // create output rgb file
      std::FILE* fp = std::fopen("rgb.out", "wb");
      if (!fp) { mju_error("Could not open rgbfile for writing"); }

      // Off screen rendering
      if(_sim_mode == SimMode::RECORD){
        // set rendering to offscreen buffer
        mjr_setBuffer(mjFB_OFFSCREEN, &mj_context);
        if (mj_context.currentBuffer!=mjFB_OFFSCREEN) {
          std::printf("Warning: offscreen rendering not supported, using default/window framebuffer\n");
        }
     }
            
      while(!glfwWindowShouldClose(mj_window) && _mjData->time< _sim_duration) {
        // render new frame if it is time (or first frame)
        if ((_mjData->time-frametime) > 1./fps || frametime==0) {
          if(_sim_mode == SimMode::RECORD){ // Record mode .....................
            // update abstract scene
            mjv_updateScene(_mjModel, _mjData, &viz_opt, NULL, &mj_cam, mjCAT_ALL, &mj_scene);

            // render scene in offscreen buffer
            mjr_render(viewport, &mj_scene, &mj_context);

            // add time stamp in upper-left corner
            char stamp[50];
            mju::sprintf_arr(stamp, "Time = %.3f", _mjData->time);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, stamp, NULL, &mj_context);

            // read rgb and depth buffers
            mjr_readPixels(rgb, depth, viewport, &mj_context);

            // write rgb image to file
            std::fwrite(rgb, 3, W*H, fp);

            // print every 10 frames: '.' if ok, 'x' if OpenGL error
            if (((framecount++)%10)==0) {
              if (mjr_getError()) { std::printf("x");} 
              else {  std::printf("."); }
            }
            frametime = _mjData->time;
            // printf("Time: %f\n", _mjData->time);
            // printf("record\n");
          } else if(_sim_mode == SimMode::PLAY){ // Play mode ...............
            // printf("play\n");
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
            
            #ifdef __APPLE__
            glfwSwapBuffers(mj_window);
            #endif
            frametime = _mjData->time;
          }
        } // end of if ((_mjData->time-frametime) > 1./fps || frametime==0)
        _onestep_forward();
      } // end of while

      std::fclose(fp);
      std::free(rgb);
      std::free(depth);
    
    } // end of run


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
    void _onestep_forward() {
      if(_mjData->time>=_ctrl_time){
        _ctrl_time += _system->getCtrlDt();
        _UpdateSystemObserver();

        _system->runCtrl();

        _UpdateControlCommand();
        _UpdateSystemVisualInfo();
      }
      mj_step(_mjModel, _mjData);
    }

    void _ReadConfig(const std::string & file_name) {
      ParamHandler param_handler(file_name);
      std::string mujoco_file_localpath;
      param_handler.getString("Mujoco_XML_FileName", mujoco_file_localpath);
      this->_mj_xml_file = THIS_COM + mujoco_file_localpath;

      double input;
      param_handler.getValue("sim_mode", input);
      this->_sim_mode = static_cast<SimMode>(input);
      param_handler.getValue("sim_duration", input);
      this->_sim_duration = input;
    }

    mjModel* _mjModel=nullptr;
    mjData* _mjData=nullptr;
        
    char error[1000];

    mjvCamera mj_cam;                      
    mjvOption viz_opt;                      
    mjvScene mj_scene;                       
    mjrContext mj_context;
    GLFWwindow* mj_window;

    // Update Sensor Data 
    virtual void _UpdateSystemObserver() = 0;

    // Write Control Command to Sim
    virtual void _UpdateControlCommand() = 0;

    // Update System's Visualization Data
    virtual void _UpdateSystemVisualInfo() = 0;

    double _ctrl_time = 0.0;

    System<double> * _system = nullptr;

  private:
    SimMode _sim_mode;
    double _sim_duration;
    std::string _mj_xml_file;
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
