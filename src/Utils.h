#pragma once

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Constants.h"
#include <pcl/kdtree/kdtree_flann.h>

using Vec3f = Eigen::Vector3f;
using Vec3d = Eigen::Vector3d;
using Vec4f = Eigen::Vector4f;
using Mat3d = Eigen::Matrix3d;

using Point3 = pcl::PointXYZ;
using Point2 = pcl::PointXY;
using PointCloud3 = pcl::PointCloud<Point3>;
using PointCloud2 = pcl::PointCloud<Point2>;

using KDTree3 = pcl::KdTreeFLANN<Point3>;
using KDTree2 = pcl::KdTreeFLANN<Point2>;

using Visual = pcl::visualization::PCLVisualizer;

using PCColorHandler = pcl::visualization::PointCloudColorHandlerGenericField<Point3>; //TODO: maybe not correct usage
using PcColorHandlerCustom = pcl::visualization::PointCloudColorHandlerCustom<Point3>;

static inline double distanceSquared(const Vec3d& a, const Vec3d& b)
{
	return (a - b).squaredNorm();
}

static inline double cross2(const Vec3d& a, const Vec3d& b) //TODO... this returns (a.cross(b)).z in 3D if z==0
{
	return a.x() * b.y() - a.y() * b.x();
}

static inline bool nearlyEqual(const Vec3d& a,const Vec3d& b, double eps = EPS)
{
	return (a - b).squaredNorm() <= eps * eps;
}

static inline Point3 ToPoint3(const Vec3d& vec)
{
	return Point3{ (float)vec.x(),(float)vec.y(),(float)vec.z() };
}

static inline Point2 ToPoint2(const Point3& p)
{
	return Point2{ p.x,p.y };
}

static inline Point2 ToPoint2(const Vec3d& p)
{
	return Point2{ (float)p.x(),(float)p.y() };
}

static inline pcl::Normal ToNormal(const Vec3d& vec)
{
	return pcl::Normal{ (float)vec.x(),(float)vec.y(),(float)vec.z() };
}