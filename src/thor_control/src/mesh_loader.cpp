#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <shape_msgs/Mesh.h>
#include <ros/ros.h>
#include <ros/package.h>

shape_msgs::Mesh loadMesh(const std::string& path, double scale_factor = 1.0) {
    std::string package_path = ros::package::getPath("thor_control");
    std::string mesh_path = package_path + path;
    Assimp::Importer importer;
    shape_msgs::Mesh mesh;

    const aiScene* scene = importer.ReadFile(mesh_path, aiProcess_Triangulate);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        ROS_ERROR("Failed to load mesh from path: %s, error: %s", path.c_str(), importer.GetErrorString());
        return mesh;
    }

    aiMesh* ai_mesh = scene->mMeshes[0];

    // scale down
    for (unsigned int i = 0; i < ai_mesh->mNumVertices; ++i) {
        geometry_msgs::Point vertex;
        vertex.x = ai_mesh->mVertices[i].x * scale_factor;
        vertex.y = ai_mesh->mVertices[i].y * scale_factor;
        vertex.z = ai_mesh->mVertices[i].z * scale_factor;
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