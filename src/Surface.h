#pragma once
#include "Utils.h"

#include "Constants.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

#include "ConvexHull.h"

struct SurfacePoint
{
	Vec3d Pos;
	Vec3d Normal;
};

using SurfaceRow = std::vector<SurfacePoint>;
using SurfacePath = std::vector<SurfaceRow>;

class Surface
{
public:
	void Set(PointCloud::Ptr cloud,  Visual::Ptr viewer);
	SurfacePath CalculateSurfacePath(const ConvexHull& hull, double lineOffset)const;

	SurfacePoint LiftToSurface(const Vec3d& point, double z_guess = std::numeric_limits<double>::quiet_NaN())const;
	SurfacePoint GetNextAlong2DPath(const SurfacePoint& p, const Vec3d& dirXY, double stepSize)const;
	SurfacePoint GetNextRowPoint(const SurfacePoint& p, const Vec3d& tangent, double offset, double crossProductSignage)const;

	SurfaceRow GetNextRowPath(const SurfaceRow& row, const ConvexHull& hull, double lineOffset, double crossProductSignage)const;

	SurfaceRow ElevateEdge(const Edge& edge, double stepSize)const;
	SurfaceRow ElevateEdge2(const Edge& edge, double stepSize)const;

private:
	PointCloud::ConstPtr m_cloud;
	pcl::KdTreeFLANN<Point3> m_kdTree;
	Vec4f m_centroid; // cached (used as z-guess fallback)

	PCColorHandler::Ptr m_pcHandler = nullptr;
};

