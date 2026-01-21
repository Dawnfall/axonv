#include "Surface.h"

#include "Test.h"

void Surface::Set(PointCloud::Ptr cloud, Visual::Ptr viewer)
{
	m_cloud = cloud;

	m_pcHandler = std::make_shared<PCColorHandler>(m_cloud, "z");

	m_kdTree.setInputCloud(cloud); 
	pcl::compute3DCentroid(*cloud, m_centroid);

	viewer->addPointCloud<Point3>(m_cloud, *m_pcHandler, SURFACE_CLOUD_ID);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, SURFACE_POINT_SIZE, SURFACE_CLOUD_ID);
}

SurfacePath Surface::CalculateSurfacePath(const ConvexHull& convexHull,double lineOffset)
{
	SurfacePath path;

	if (!m_cloud || m_cloud->empty())
		return path;

	double trajectoryStep = lineOffset / 3.0f;
	SurfaceRow row = ElevateEdge(convexHull.GetLongestEdge(), trajectoryStep);
	path.emplace_back(row);

	double alternate = -1.0f;
	while (!path.back().empty())
	{
		path.emplace_back(GetNextRowPath(path.back(), convexHull, lineOffset, alternate));
		alternate *= -1;
	}
	return path;
}

// Project (x,y) to the surface and estimate normal using local plane fitting.
SurfacePoint Surface::LiftToSurface(Vec3d point, double z_guess) const
{
	// If caller didn't provide a z guess, fall back to cloud centroid z
	const double z0 = std::isnan(z_guess) ? m_centroid.z() : z_guess;

	// Query point for neighbor search (3D KD-tree); z is only a bias
	Point3 q{ (float)point.x(), (float)point.y(), (float)z0 };

	std::vector<int> indices(KD_TREE_K_CLOSEST);
	std::vector<float> sqr_dist(KD_TREE_K_CLOSEST);
	m_kdTree.nearestKSearch(q, KD_TREE_K_CLOSEST, indices, sqr_dist);

	// --- centroid of neighbors ---
	Vec3d centroid(0.0, 0.0, 0.0);
	for (int id : indices)
	{
		const auto& p = (*m_cloud)[id];
		centroid += Vec3d(p.x, p.y, p.z);
	}
	centroid /= indices.size();

	// --- covariance ---
	Mat3d cov = Mat3d::Zero();
	for (int id : indices)
	{
		const auto& p = (*m_cloud)[id];
		Vec3d d{ p.x - centroid.x(), p.y - centroid.y(), p.z - centroid.z() };
		cov += d * d.transpose();
	}
	cov /= indices.size();

	// --- PCA: smallest eigenvector = normal ---
	Eigen::SelfAdjointEigenSolver<Mat3d> es(cov);
	if (es.info() != Eigen::Success)
		throw std::runtime_error("Eigen decomposition failed");

	Vec3d n = es.eigenvectors().col(0);
	n.normalize();

	// Enforce consistent orientation (height-field assumption)
	if (n.z() < 0.f)
		n = -n;

	// --- XY-preserving lift: solve plane equation for z ---
	// Plane through centroid with normal n: n · (X - centroid) = 0
	const double x = point.x();
	const double y = point.y();
	const double nz = n.z();

	// Under your assumptions, nz should not be near 0.
	// If you still want a minimal guard without "edge case clutter", you can assert:
	// assert(std::abs(nz) > 1e-6f);

	const double z = centroid.z() - (n.x() * (x - centroid.x()) + n.y() * (y - centroid.y())) / nz;

	return SurfacePoint{ Vec3d{x, y, z}, n };
}

SurfacePoint Surface::GetNextAlong2DPath(const SurfacePoint& p, Vec3d dirXY, double stepSize)const
{
	Vec3d dirProjOnNormal = dirXY.dot(p.Normal) * p.Normal;
	Vec3d dirTrajectory = (dirXY - dirProjOnNormal).normalized();

	Vec3d nextPoint = p.Pos + dirTrajectory * stepSize;

	return LiftToSurface(nextPoint);
}

SurfacePoint Surface::GetNextRowPoint(const SurfacePoint& p, const Vec3d& tangent, double offset, double crossProductSignage)const
{
	Vec3d moveDir = tangent.cross(p.Normal).normalized() * crossProductSignage; // cross product will alternate row to row
	Vec3d newPos = p.Pos + moveDir * offset;

	return LiftToSurface(newPos, newPos.z());
}

SurfaceRow Surface::GetNextRowPath(const SurfaceRow& row, const ConvexHull& hull, double lineOffset, double crossProductSignage)const
{
	SurfaceRow nextRow;
	const size_t n = row.size();
	if (n < 2)
		return nextRow;

	nextRow.reserve(n);

	for (size_t i = n; i > 0;) //next row is inverted
	{
		--i; // n-1 -> 0
		const auto& cur = row[i];

		Vec3d tangent = (i > 0)?
			(cur.Pos - row[i - 1].Pos): // backward tangent
			(row[1].Pos - cur.Pos) // forward tangent
			;
		tangent.normalize();

		SurfacePoint p = GetNextRowPoint(cur, tangent, lineOffset, crossProductSignage);
		if (hull.PointInConvexPolygonCCW(p.Pos))
			nextRow.emplace_back(p);
	}

	return nextRow;
}

SurfaceRow Surface::ElevateEdge(const Edge& edge, double stepSize)const
{
	SurfaceRow row;
	double totalLen = (edge.b - edge.a).norm();
	Vec3d dir = (edge.b - edge.a).normalized();

	int t = edge.GetNorm() / stepSize;
	for (int i = 0; i < t; ++i)
	{
		Vec3d newPoint = edge.a + dir * stepSize * i; //first row is generated in 2D and lifted to 3D
		row.push_back(LiftToSurface(newPoint));
	}
	return row;
}

SurfaceRow Surface::ElevateEdge2(const Edge& edge, double stepSize)const //bad way
{
	double totalLenXY = (edge.b - edge.a).norm();
	Vec3d dirXY = (edge.b - edge.a).normalized();

	SurfaceRow row;
	SurfacePoint startSurfacePoint = LiftToSurface(edge.a);
	row.push_back(startSurfacePoint);

	while (true)
	{
		const Vec3d& point = row.back().Pos;
		const Vec3d& normal = row.back().Normal;

		Vec3d trajectory = dirXY - normal * dirXY.dot(normal); //trajectory will drift in x,y
		trajectory.normalize();

		Vec3d nextPoint = point + trajectory * stepSize;
		double distanceXY = (Vec3d(nextPoint.x(), nextPoint.y(), 0.0) - edge.a).dot(dirXY);
		if (distanceXY > totalLenXY)
			break;

		row.push_back(LiftToSurface(nextPoint, nextPoint.z()));
	}
	return row;
}