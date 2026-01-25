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

	m_path.Init(m_viewer);

	m_viewer->registerPointPickingCallback([this](const pcl::visualization::PointPickingEvent& e)
		{
			if (int idx = e.getPointIndex(); idx == -1) // nothing picked
				return;
			if (auto cloudID = e.getCloudName(); cloudID != SURFACE_CLOUD_ID)
				return;

			Point3 selectedPoint;
			e.getPoint(selectedPoint.x, selectedPoint.y, selectedPoint.z);

			m_hull.SelectPoint(selectedPoint,m_viewer);
		});

	m_viewer->registerKeyboardCallback(
		[this](const pcl::visualization::KeyboardEvent& event) {
			if (event.keyDown())
			{
				if (event.getKeySym() == "r")
				{
					m_viewer->removeAllShapes();
					m_hull.Clear(m_viewer);

					m_path.Clear(m_viewer);
					m_path.Update(m_viewer);
				}
				else if (event.getKeySym() == "c")
				{
					m_path.Clear(m_viewer);

					m_hull.Prepare();
					SurfacePath surPath = m_surface.CalculateSurfacePath(m_hull, m_lineOffset); //TODO: set here
					m_path.Set(surPath);

					m_path.Update(m_viewer);
				}
			}
		});
}

void App::Init(const std::string& pcPath, double lineOffset)
{
	m_lineOffset = lineOffset;

	PointCloud3::Ptr pointCloud = nullptr;
	if (pcPath == "random")
		pointCloud = CreateSomePointCloud(100, 100, 0.1f, 0.1f);
	else
	{
		pointCloud = std::make_shared<PointCloud3>();
		if (pcl::io::loadPCDFile<Point3>(pcPath, *pointCloud) < 0)
		{
			throw std::runtime_error("Failed to load PCD file: " + pcPath);
		}
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