#pragma once

#include <shape_msgs/Mesh.h>
#include <string>
#include <geometry_msgs/Pose.h>

bool isGazeboRunning();

// Replace all occurrences of a substring within a string
std::string replaceAll(std::string str, const std::string& from, const std::string& to);

// Load the content of a file into a string
std::string loadFile(const std::string& filename);

// Get the absolute path of a mesh given its name
std::string getMeshAbsolutePath(const std::string& name);

// Convert Pose into string
std::string convertPoseToStr(const geometry_msgs::Pose& pose);

// Load a mesh for use in RViz, given the name of the mesh and an optional scaling factor
shape_msgs::Mesh loadMeshRviz(const std::string& name, double scale);

// Load a mesh for use in Gazebo, given the name of the mesh and an optional scaling factor
void loadMeshGazebo(const std::string& name, double scale, const geometry_msgs::Pose& pose);
