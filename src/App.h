#pragma once
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

	ConvexHull m_hull;
	Surface m_surface;
	Path m_path;

	float m_lineOffset = 0.01f;
};