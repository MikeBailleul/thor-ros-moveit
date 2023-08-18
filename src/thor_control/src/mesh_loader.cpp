#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <shape_msgs/Mesh.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <gazebo/transport/transport.hh>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

bool isGazeboRunning() {
    if (std::system("pgrep gzserver")) {
        return false; // Command returns non-zero if gazebo is not running
    }
    return true;
}

std::string replaceAll(std::string str, const std::string& from, const std::string& to) {
    size_t startPos = 0;
    while ((startPos = str.find(from, startPos)) != std::string::npos) {
        str.replace(startPos, from.length(), to);
        startPos += to.length();
    }
    return str;
}

std::string loadFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Failed to open file: " << filename << '\n';
        return "";
    }
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return content;
}

std::string getMeshAbsolutePath(const std::string& name) {
    std::string package_path = ros::package::getPath("thor_control");
    return package_path + "/meshes/" + name + ".stl";
}

std::string getSdfAbsolutePath(const std::string& name) {
    std::string package_path = ros::package::getPath("thor_control");
    return package_path + "/sdf/" + name + ".sdf";
}

std::string convertPoseToStr(const geometry_msgs::Pose& pose) {
    std::ostringstream pose_stream;
    pose_stream << pose.position.x << " " 
                << pose.position.y << " " 
                << pose.position.z << " ";

    // Convert quaternion to Euler angles in radians
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // angles in radians

    pose_stream << roll << " " 
                << pitch << " " 
                << yaw;

    return pose_stream.str();
}


shape_msgs::Mesh loadMeshRviz(const std::string& name, double scale) {
    std::string meshPath = getMeshAbsolutePath(name);
    Assimp::Importer importer;
    shape_msgs::Mesh mesh;

    const aiScene* scene = importer.ReadFile(meshPath, aiProcess_Triangulate);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        ROS_ERROR("Failed to load mesh from path: %s, error: %s", meshPath.c_str(), importer.GetErrorString());
        return mesh;
    }

    aiMesh* ai_mesh = scene->mMeshes[0];

    // scale down
    for (unsigned int i = 0; i < ai_mesh->mNumVertices; ++i) {
        geometry_msgs::Point vertex;
        vertex.x = ai_mesh->mVertices[i].x * scale;
        vertex.y = ai_mesh->mVertices[i].y * scale;
        vertex.z = ai_mesh->mVertices[i].z * scale;
        mesh.vertices.push_back(vertex);
    }

    for (unsigned int i = 0; i < ai_mesh->mNumVertices; ++i) {
        geometry_msgs::Point vertex;
        vertex.x = ai_mesh->mVertices[i].x;
        vertex.y = ai_mesh->mVertices[i].y;
        vertex.z = ai_mesh->mVertices[i].z;
        mesh.vertices.push_back(vertex);
    }

    for (unsigned int i = 0; i < ai_mesh->mNumFaces; ++i) {
        if (ai_mesh->mFaces[i].mNumIndices != 3) continue;

        shape_msgs::MeshTriangle triangle;
        triangle.vertex_indices[0] = ai_mesh->mFaces[i].mIndices[0];
        triangle.vertex_indices[1] = ai_mesh->mFaces[i].mIndices[1];
        triangle.vertex_indices[2] = ai_mesh->mFaces[i].mIndices[2];
        mesh.triangles.push_back(triangle);
    }

    return mesh;
}

void loadMeshGazebo(const std::string& name, double scale, const geometry_msgs::Pose& pose) {
    if (isGazeboRunning()) {
        std::string meshPath = getMeshAbsolutePath(name);
        std::string sdfPath = getSdfAbsolutePath(name);
        std::string sdfTemplate = loadFile(sdfPath);

        std::string sdfString = replaceAll(sdfTemplate, "{object_name}", name);
        sdfString = replaceAll(sdfString, "{object_path}", meshPath);
        sdfString = replaceAll(sdfString, "{object_scale}", std::to_string(scale));
        sdfString = replaceAll(sdfString, "{object_pose}", convertPoseToStr(pose));

        // create temp file to store sdf constructed template
        std::string tempFileName = "/tmp/temp_constructed.sdf";
        std::ofstream sdfTempFile(tempFileName);
        sdfTempFile << sdfString;
        sdfTempFile.close();
        
        std::string cliDelete = "gz model -m '" + name +  "' -d";
        system(cliDelete.c_str());

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::string cliAdd = "gz model -m '" + name +  "' -f " + tempFileName;
        system(cliAdd.c_str());

        // Delete the temporary file
        std::remove(tempFileName.c_str());
    }
}
