#include "ConvexHull.h"

#include <pcl/surface/convex_hull.h>

// Returns convex hull polygon in XY as CCW , assumes its a valid area
ConvexHull::ConvexHull()
{
	m_selectedPoints = std::make_shared<PointCloud3>();
}

void ConvexHull::Prepare()
{
	pcl::ConvexHull<Point3> hull;
	hull.setInputCloud(m_selectedPoints);
	hull.setDimension(2);

	PointCloud3 hullPoints;
	std::vector<pcl::Vertices> hullPolygons;
	hull.reconstruct(hullPoints, hullPolygons);

	if (hullPolygons.empty())
	{
		std::cout << "Hull reconstruction failed.\n";
		return;
	}

	const auto& ring = hullPolygons[0].vertices;
	m_polygon.reserve(ring.size());
	for (uint32_t idx : ring)
	{
		const auto& hp = hullPoints.points.at(idx);
		m_polygon.emplace_back(Vec3d{ hp.x, hp.y, 0.0f });
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

bool ConvexHull::PointInConvexPolygonCCW(const Vec3d& p)const
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

void ConvexHull::Clear(Visual::Ptr viewer)
{
	viewer->removeAllShapes();
	m_selectedPoints->clear();
	m_polygon.clear();

}

void ConvexHull::SelectPoint(const Point3& point, Visual::Ptr viewer)
{
	const std::string id = "s" + std::to_string(m_selectedPoints->points.size());
	m_selectedPoints->push_back(point);
	viewer->addSphere(point, SELECTED_SPHERE_SIZE , RED_COLOR.x(), RED_COLOR.y(), RED_COLOR.z(), id);
}