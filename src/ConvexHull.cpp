#include "ConvexHull.h"

#include <pcl/surface/convex_hull.h>

// Returns convex hull polygon in XY as CCW , assumes its a valid area
ConvexHull::ConvexHull(PointCloud3::Ptr areaPoints)
{
	pcl::ConvexHull<Point3> hull;
	hull.setInputCloud(areaPoints);
	hull.setDimension(2);

	PointCloud3 hullPoints;
	std::vector<pcl::Vertices> hullPolygons;
	hull.reconstruct(hullPoints, hullPolygons);

	if (hullPolygons.empty())
		throw std::runtime_error("Hull reconstruction failed.");

	const auto& ring = hullPolygons[0].vertices;
	m_polygon.reserve(ring.size());
	for (uint32_t idx : ring)
	{
		const auto& hp = hullPoints.points.at(idx);
		m_polygon.emplace_back(hp.x, hp.y, 0.0f);
	}

	// Remove last if equals first
	if (m_polygon.size() >= 2 && nearlyEqual(m_polygon.front(), m_polygon.back()))
		m_polygon.pop_back();

	// Enforce CCW
	const float signedArea = convexHull2DSignedArea();
	if (signedArea < 0.0f)
		std::reverse(m_polygon.begin(), m_polygon.end());
}

double ConvexHull::convexHull2DSignedArea()const
{
	if (m_polygon.size() < 3)
		return 0.0f;
	double signedArea = 0.0f;
	for (size_t i0 = 0; i0 < m_polygon.size(); ++i0)
	{
		const size_t i1 = (i0 + 1) % m_polygon.size();
		signedArea += m_polygon[i0].x() * m_polygon[i1].y() - m_polygon[i1].x() * m_polygon[i0].y();
	}
	return 0.5f * signedArea; // signed
}

Edge ConvexHull::GetLongestEdge()const
{
	size_t bestIndex0 = 0;
	double bestDistanceSq = -1.0;

	for (size_t i0 = 0; i0 < m_polygon.size(); ++i0)
	{
		const size_t i1 = (i0 + 1) % m_polygon.size();
		const double distSq = distanceSquared(m_polygon[i0], m_polygon[i1]);
		if (distSq > bestDistanceSq)
		{
			bestDistanceSq = distSq;
			bestIndex0 = i0;
		}
	}
	const size_t bestIndex1 = (bestIndex0 + 1) % m_polygon.size();
	return Edge{ m_polygon[bestIndex0] ,m_polygon[bestIndex1] };
}

// TODO... need a bit more digging through this
LinePolyHit ConvexHull::IntersectLineConvexHull(const Line& line)const //polygon is CCW
{
	LinePolyHit out;
	out.hit = HitType::NONE;

	if (m_polygon.size() < 3)
		return out;
	if (line.Dir.squaredNorm() < EPS * EPS)
		return out;

	double tEnter = -std::numeric_limits<double>::infinity();
	double tExit = std::numeric_limits<double>::infinity();

	for (size_t i0 = 0; i0 < m_polygon.size(); ++i0)
	{
		size_t i1 = (i0 + 1) % m_polygon.size();
		Vec3d e = m_polygon[i1] - m_polygon[i0];

		// Inequality: s * cross(e, (P + tD) - vi) >= 0
		double A = cross2(e, line.Dir);         // coefficient of t
		double B = cross2(e, line.Point - m_polygon[i0]);  // constant term

		if (std::fabs(A) <= EPS)
		{
			// Parallel to this edge's supporting line: either always ok or impossible.
			if (B < -EPS)
				return out; // outside => no intersection
			continue;                 // no constraint
		}

		const double tHit = -B / A; // where it crosses the boundary line

		if (A > 0)
		{
			// A t + B >= 0  => t >= tHit
			if (tHit > tEnter)
				tEnter = tHit;
		}
		else
		{
			// => t <= tHit
			if (tHit < tExit)
				tExit = tHit;
		}

		if (tEnter - tExit > EPS)
		{
			out.hit = HitType::NONE;
			return out;
		}
	}

	// If we get here, the line intersects the convex polygon.
	if (std::isfinite(tEnter) && std::isfinite(tExit) && std::fabs(tEnter - tExit) <= 10.f * EPS)
	{
		out.hit = HitType::TANGENT;
		const double t = 0.5f * (tEnter + tExit);
		out.p0 = line.Point + t * line.Dir;
		out.p1 = out.p0;
	}
	else
	{
		out.hit = HitType::SECANT;
		out.p0 = line.Point + tEnter * line.Dir;
		out.p1 = line.Point + tExit * line.Dir;
	}

	return out;
}

bool ConvexHull::PointInConvexPolygonCCW(const Vec3d& p)const //TODO..
{
	for (size_t i = 0; i < m_polygon.size(); ++i)
	{
		const Vec3d& a = m_polygon[i];
		const Vec3d& b = m_polygon[(i + 1) % m_polygon.size()];
		if (cross2(b - a, p - a) < -EPS)
			return false;
	}
	return true;
}
