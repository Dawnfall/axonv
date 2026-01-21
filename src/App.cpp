#include "App.h"
#include <pcl/io/pcd_io.h>

#include "Utils.h"
#include "Constants.h"
#include "Surface.h"
#include "ConvexHull.h"

#include "Test.h"
App::App()
{
	m_viewer = std::make_shared<Visual>("AxonV");
	m_viewer->setBackgroundColor(BG_COLOR.x(), BG_COLOR.y(), BG_COLOR.z());

	m_selectedPoints = std::make_shared<PointCloud>();
	m_selectedPointsHandler = std::make_shared<PcColorHandlerCustom>(m_selectedPoints, RED_COLOR.x(), RED_COLOR.y(), RED_COLOR.z());

	m_viewer->addPointCloud(m_selectedPoints, *m_selectedPointsHandler, SELECTED_CLOUD_ID);
	m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, SELECTED_POINT_SIZE, SELECTED_CLOUD_ID);

	m_path.Init(m_viewer);

	//m_viewer->initCameraParameters();
	//m_viewer->resetCamera();
	m_viewer->registerPointPickingCallback(
		[this](const pcl::visualization::PointPickingEvent& e)
		{
			int idx = e.getPointIndex();
			if (idx == -1)
				return; // nothing picked

			float x, y, z;
			e.getPoint(x, y, z);

			m_selectedPoints->push_back(Point3{ x, y, z });
			std::cout << x << " " << y << " " << z << std::endl;
			m_viewer->updatePointCloud(m_selectedPoints, *m_selectedPointsHandler, SELECTED_CLOUD_ID);
		});

	m_viewer->registerKeyboardCallback(
		[this](const pcl::visualization::KeyboardEvent& event) {
			if (event.keyDown())
			{
				if (event.getKeySym() == "r")
				{
					m_selectedPoints->clear();
					m_viewer->updatePointCloud<Point3>(m_selectedPoints, *m_selectedPointsHandler, SELECTED_CLOUD_ID);

					m_path.Clear(m_viewer);
					m_path.Update(m_viewer);
				}
				else if (event.getKeySym() == "c")
				{
					m_path.Clear(m_viewer);

					ConvexHull hull(m_selectedPoints);
					SurfacePath surPath = m_surface.CalculateSurfacePath(hull, m_lineOffset); //TODO: set here
					m_path.Set(surPath);

					m_path.Update(m_viewer);
				}
			}
		});
}

void App::Init(const std::string& pcPath, double lineOffset)
{
	m_lineOffset = lineOffset;

	PointCloud::Ptr pointCloud = std::make_shared<PointCloud>();
	if (pcl::io::loadPCDFile<Point3>(pcPath, *pointCloud) < 0)
	{
		throw std::runtime_error("Failed to load PCD file: " + pcPath);
	}

	m_surface.Set(pointCloud, m_viewer);
}

void App::Run()
{
	while (!m_viewer->wasStopped())
	{
		m_viewer->spinOnce(16);
	}
}