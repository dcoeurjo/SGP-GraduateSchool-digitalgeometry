#include <iostream>
#include <vector>
#include <array>
#include <utility>

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

float h=0.25;

void oneStep(double myh)
{
  auto params = SH3::defaultParameters();
  params( "polynomial", "sphere1" )( "gridstep", myh )
        ( "minAABB", -1.25 )( "maxAABB", 1.25 );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  
  QuickHull3D hull;
  
  std::vector<Point> points;
  std::cout << "Digitzing shape" << std::endl;
  auto domain = digitized_shape->getDomain();
  for(auto &p: domain)
    if (digitized_shape->operator()(p))
      points.push_back(p);
  
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
  
  polyscope::registerSurfaceMesh("Convex hull", vertices, facets)->rescaleToUnit();
}

float deltah=0.005;
float deltac=31.0/32.0;
void mycallback()
{
  ImGui::SliderFloat("h", &h, 0.0, 0.5);
  ImGui::SliderFloat("deltah", &deltah, 0.0, 0.1);
  ImGui::SliderFloat("deltac", &deltac, 0.0, 1.0);
  if (ImGui::Button("Run"))
  {
    oneStep(h);
  }
  if (ImGui::Button("Screenshots"))
  {
    for(auto hh=h; hh > 0.01; hh=hh-deltah)
    {
      oneStep(hh);
      polyscope::screenshot();
      polyscope::refresh();
    }
  }
  if (ImGui::Button("Screenshots (mult)"))
  {
    for(auto hh=h; hh > 0.01; hh *= deltac)
    {
      std::cout << "gridstep = " << hh << std::endl;
      oneStep(hh);
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
