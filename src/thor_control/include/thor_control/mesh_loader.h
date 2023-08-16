#pragma once

#include <shape_msgs/Mesh.h>
#include <string>

shape_msgs::Mesh loadMesh(const std::string& path, double scale_factor);
