#pragma once

#include "Constants.h"
#include "Utils.h"

struct Line
{
	Vec3d Point;
	Vec3d Dir;

	void OffsetCCW(double step)
	{
		Vec3d offsetDir{ -Dir.y(),Dir.x(),Dir.z() };
		Point += offsetDir * step;
	}
};

struct Edge
{
	Vec3d a;
	Vec3d b;

	Line GetLine()const
	{
		Vec3d v = b - a;
		return Line{ a,v.normalized() };
	}

	double GetNorm()const { return (b - a).norm(); }
};

enum class HitType
{
	NONE,
	TANGENT,
	SECANT
};

struct LinePolyHit
{
	HitType hit;
	Vec3d p0, p1;
};

//points are 3D but z == 0 
//polygon is CCW

class ConvexHull
{
public:
	ConvexHull(PointCloud3::Ptr areaPoints);

	double convexHull2DSignedArea()const;
	Edge GetLongestEdge()const;
	LinePolyHit IntersectLineConvexHull(const Line& line)const;
	bool PointInConvexPolygonCCW(const Vec3d& p)const;

private:
	std::vector<Vec3d> m_polygon;
};
