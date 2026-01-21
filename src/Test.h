#pragma once

#include "Utils.h"
#include "ConvexHull.h"
#include "Surface.h"


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
