#pragma once
#include "Utils.h"

class Path
{
public:

	inline void Init(Visual::Ptr viewer)
	{
		m_pathCloud = std::make_shared<PointCloud>();
		m_pathHandler = std::make_shared<PcColorHandlerCustom>(m_pathCloud, GRAY_COLOR.x(), GRAY_COLOR.y(), GRAY_COLOR.z());
		m_normalCloud = std::make_shared<pcl::PointCloud<pcl::Normal>>();

		viewer->addPointCloud(m_pathCloud, *m_pathHandler, PATH_CLOUD_ID);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PATH_POINT_SIZE, PATH_CLOUD_ID);
	}

	inline void Clear(Visual::Ptr viewer)
	{
		m_pathCloud->clear();
		m_normalCloud->clear();
	}

	inline void Set(const SurfacePath& path)
	{
		for (const auto& row : path)
			for (const auto& surfacePoint : row)
			{
				m_pathCloud->push_back(ToPoint3(surfacePoint.Pos));
				m_normalCloud->push_back(ToNormal(surfacePoint.Normal));
			}
	}

	inline void Update(Visual::Ptr viewer)
	{
		viewer->updatePointCloud<Point3>(m_pathCloud, *m_pathHandler, PATH_CLOUD_ID);
		viewer->removePointCloud(NORMALS_CLOUD_ID);
		viewer->addPointCloudNormals<Point3, pcl::Normal>(
			m_pathCloud, m_normalCloud,
			NORMALS_LEVEL, NORMALS_SCALE,
			NORMALS_CLOUD_ID
		);
	}

	inline void PrintPath()const
	{
		std::cout << "Points:\n";

		for (size_t i = 0; i < m_pathCloud->size(); i++)
		{
			std::cout << "Pos: " << (*m_pathCloud)[i] << " ; ";
			std::cout << "Nor: " << (*m_normalCloud)[i] << "\n";
		}
	}

private:

	PointCloud::Ptr m_pathCloud;
	pcl::PointCloud<pcl::Normal>::Ptr m_normalCloud;
	PcColorHandlerCustom::Ptr m_pathHandler = nullptr;
};