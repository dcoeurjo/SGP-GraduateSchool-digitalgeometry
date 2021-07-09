#include <iostream>
#include <vector>
#include <array>
#include <utility>
#include <random>

#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>
#include "DGtal/geometry/tools/QuickHull.h"

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

using namespace DGtal;
using namespace Z3i;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;

typedef DGtal::ConvexHullIntegralKernel< 3 > Kernel3D;
typedef DGtal::QuickHull< Kernel3D >         QuickHull3D;

float deltah=0.005;
float deltac=63.0/64.0;
int width=10;
int nbpts=100;
std::default_random_engine generator;

void oneStep()
{
  std::normal_distribution<double> distribution(0,width/4.0);

  QuickHull3D hull;
  std::vector<Point> pointsgrid;
  for(auto i=-width;i <= width;++i)
    for(auto j=-width;j <= width;++j)
      for(auto k=-width;k <= width;++k)
        pointsgrid.push_back(Point(i,j,k));
        
  std::vector<Point> points;
  std::cout << "Digitzing shape" << std::endl;
  Domain dom(Point(-width,-width,-width),Point(width,width,width));
  
  for(auto i=0; i < nbpts; ++i)
  {
    Point p(distribution(generator),distribution(generator),distribution(generator));
    if (!dom.isInside(p)) continue;
    points.push_back(p);
  }
  
  std::cout << "Computing convex hull" << std::endl;
  hull.setInput( points );
  hull.computeConvexHull();
  std::cout << "#points="    << hull.nbPoints()
           << " #vertices=" << hull.nbVertices()
           << " #facets="   << hull.nbFacets() << std::endl;
  
  std::vector< RealPoint > vertices;
  hull.getVertexPositions( vertices );
  std::vector< std::vector< std::size_t > > facets;
  hull.getFacetVertices( facets );
  
  auto pp = polyscope::registerSurfaceMesh("Convex hull", vertices, facets);
  polyscope::registerPointCloud("Grid", pointsgrid);

}

double h=0.25;
void mycallback()
{
  ImGui::SliderInt("width", &width, 0, 100);
  ImGui::SliderInt("nbpts", &nbpts, 0, 1000);
  if (ImGui::Button("Run"))
  {
    oneStep();
  }
  if (ImGui::Button("Screenshots"))
  {
    for(auto hh=h; hh > 0.01; hh=hh-deltah)
    {
      oneStep();
      polyscope::screenshot();
      polyscope::refresh();
    }
  }
  if (ImGui::Button("Screenshots (mult)"))
  {
    for(auto hh=h; hh > 0.01; hh *= deltac)
    {
      oneStep();
      polyscope::screenshot();
      polyscope::refresh();
    }
  }
}

int main()
{
  polyscope::init();
  polyscope::state::userCallback = mycallback;
  polyscope::show();
  return 0;
}
