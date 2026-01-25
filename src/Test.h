#pragma once

#include "Utils.h"
#include "ConvexHull.h"
#include "Surface.h"

inline PointCloud3::Ptr CreateSomePointCloud(int nX, int nY, float dX, float dY)
{
    auto cloud = std::make_shared<PointCloud3>();
    cloud->points.reserve(static_cast<std::size_t>(nX) * static_cast<std::size_t>(nY));

    for (int j = 0; j < nY; ++j) 
	{
        for (int i = 0; i < nX; ++i)
		 {
            float x = i * dX;
            float y = j * dY;
            float z = std::sin(x) + std::cos(y); //example

            cloud->points.push_back({x, y, z});
        }
    }

    cloud->width  = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}


inline SurfacePath TestNRows(const Surface& surface, const ConvexHull& hull, double lineOffset, int n)
{
	SurfacePath path;
	double trajectoryStep = lineOffset / 3.0f;
	SurfaceRow row = surface.ElevateEdge(hull.GetLongestEdge(), trajectoryStep);
	path.emplace_back(row);

	double alternate = -1.0f;
	for (int i = 0; i < n; i++)
	{
		path.emplace_back(surface.GetNextRowPath(path.back(), hull, lineOffset, alternate));
		alternate *= -1;
	}
	return path;
}
