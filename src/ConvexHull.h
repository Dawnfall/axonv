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

class ConvexHull
{
public:
	ConvexHull();

	void Prepare();

	double convexHull2DSignedArea()const;
	Edge GetLongestEdge()const;
	bool PointInConvexPolygonCCW(const Vec3d& p)const;

	void SelectPoint(const Point3& point, Visual::Ptr viewer);
	void Clear(Visual::Ptr viewer);

private:

	std::vector<Vec3d> m_polygon;
	PointCloud3::Ptr m_selectedPoints = nullptr;
};
