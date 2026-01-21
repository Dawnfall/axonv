#pragma once
//#include "Constants.h"
//#include "ConvexHull.h"
#include "Utils.h"
#include "Surface.h"
#include "Path.h"
#include <string>

class App
{
public:
	App();

	void Init(const std::string& pcPath, double lineOffset);
	void Run();

private:

	Visual::Ptr m_viewer = nullptr;

	Surface m_surface;
	Path m_path;

	PointCloud::Ptr m_selectedPoints = nullptr;
	PcColorHandlerCustom::Ptr m_selectedPointsHandler = nullptr;

	float m_lineOffset = 0.01f;
};