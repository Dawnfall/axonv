#pragma once

#include <Eigen/Core>
#include <string>

const double EPS = 1e-6f;

const std::string SURFACE_CLOUD_ID = "surface_cloud";
const std::string PATH_CLOUD_ID = "path_cloud";
const std::string NORMALS_CLOUD_ID = "normals_cloud";

const int NORMALS_LEVEL = 10;
const double NORMALS_SCALE = 0.05;

const int SURFACE_POINT_SIZE = 5;
const int PATH_POINT_SIZE = 10;

const int KD_TREE_K_CLOSEST = 150;

const float SELECTED_SPHERE_SIZE = 0.01f;

const Eigen::Vector3i BG_COLOR{ 10,10,10 };
const Eigen::Vector3i RED_COLOR{ 255,0,0 };
const Eigen::Vector3i GRAY_COLOR{ 180,180,180 };
