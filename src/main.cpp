#include "App.h"
#include "Constants.h"

//algorithm to create zig zag trajectory over surface represented by point cloud
//input: point cloud, points representing area over which we want zig zag, offset between zig zag rows
//output: trajectory of points representing zig zag path(x,y,z and normal)
//assumptions:
//     - area points define a convex hull which defines the area, which is simple(no holes/overlaps)
//     - point cloud represents a simple,no hole,no overlaping(on z) surface where 
//       You can assume that the surface changes gradually, with slow
//       variations in gradient relative to the distance between rows.
// 
//1.Get convex hull from area points 
//2.starting row...choose biggest edge  
//3.make first row from that edge by creating a trajectory of points by folowing this line but over 3D surface
//4.iterate to get next row: 
//      - go through all current row points in reverse
//      - get normal and tangent at that point
//      - from normal and tangent calculate offset vector
//      - fit that offset point to the plane
//      - if point outside of convex hull discard it
//      - once next row has no points->break
//end result are zig zag lines where 3D distances are 

int main(int argc, char* argv[]) //run with path lineOffset
{
	//set here if want default
	std::string pcPath = "data/input.pcd";
	double lineOffset = 0.03;

	if (argc == 3) //assumes valid if given
	{
		pcPath = argv[1];
		lineOffset = std::stod(argv[2]);
	}

	App app;
	app.Init(pcPath,lineOffset);
	app.Run();
}