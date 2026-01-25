#include "Surface.h"

#include <algorithm>

void Surface::Set(PointCloud3::Ptr cloud, Visual::Ptr viewer)
{
	m_cloud = cloud;

	m_pcHandler = std::make_shared<PCColorHandler>(m_cloud, "z");

	//m_kdTree.setInputCloud(cloud); 

	m_cloud2D = std::make_shared<PointCloud2>();
	m_cloud2D->reserve(m_cloud->size());
	for (const auto& p : m_cloud->points)
		m_cloud2D->push_back(ToPoint2(p));
	m_kdTree2D.setInputCloud(m_cloud2D);

	viewer->addPointCloud<Point3>(m_cloud, *m_pcHandler, SURFACE_CLOUD_ID);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, SURFACE_POINT_SIZE, SURFACE_CLOUD_ID);
}

SurfacePath Surface::CalculateSurfacePath(const ConvexHull& convexHull,double lineOffset)const
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

//PCA algorithm
SurfacePoint Surface::LiftToSurface(const Vec3d& point) const
{
	std::vector<int> indices;
	indices.reserve(KD_TREE_K_CLOSEST);
	std::vector<float> sqrDist;
	sqrDist.reserve(KD_TREE_K_CLOSEST);

	Point2 point2D = ToPoint2(point);

	//m_kdTree2D.radiusSearch(point2D, radius, indices, sqrDist); //switch to radius maybe
	m_kdTree2D.nearestKSearch(point2D, KD_TREE_K_CLOSEST, indices, sqrDist);

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

	// Enforce consistent orientation
	if (n.z() < 0.f)
		n = -n;

	double x = point.x();
	double y = point.y();
	double nz = n.z();

	//solve for z
	//double z = centroid.z() - (n.x() * (x - centroid.x()) + n.y() * (y - centroid.y())) / nz;

	double z;
	if (std::abs(nz) > 1e-2) //problems if nz is close to 0...we switch to least squares
	{
		z = centroid.z() - (n.x() * (x - centroid.x()) + n.y() * (y - centroid.y())) / nz;
	}
	else
	{
		// Least squares z = ax + by + c
		Eigen::MatrixXd A(indices.size(), 3);
		Eigen::VectorXd b(indices.size());

		for (size_t i = 0; i < indices.size(); ++i)
		{
			const auto& p = (*m_cloud)[indices[i]];
			A(i, 0) = p.x;
			A(i, 1) = p.y;
			A(i, 2) = 1.0;
			b(i) = p.z;
		}

		Eigen::Vector3d abc = A.colPivHouseholderQr().solve(b);
		z = abc[0] * x + abc[1] * y + abc[2];
	}



	return SurfacePoint{ Vec3d{x, y, z}, n };
}

SurfacePoint Surface::GetNextAlong2DPath(const SurfacePoint& p, const Vec3d& dirXY, double stepSize)const
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

	return LiftToSurface(newPos);
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

		row.push_back(LiftToSurface(nextPoint));
	}
	return row;
}