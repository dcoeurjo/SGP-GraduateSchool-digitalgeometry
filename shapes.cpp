#include <iostream>
#include <vector>
#include <array>
#include <utility>

#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include "deps/CLI11/CLI11.hpp"


using namespace DGtal;
using namespace Z3i;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;

bool toggle=false;
int cpt=0;
bool screenshots=false;


void oneStep(double h)
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params( "polynomial", "distel" )( "gridstep", h );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K               = SH3::getKSpace( params );
  auto binary_image    = SH3::makeBinaryImage( digitized_shape, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  auto embedder        = SH3::getCellEmbedder( K );
  SH3::Cell2Index c2i;
  auto surfels         = SH3::getSurfelRange( surface, params );
  auto primalSurface   = SH3::makePrimalPolygonalSurface(c2i, surface);
  
  //Need to convert the faces
  std::vector<std::vector<std::size_t>> faces;
  for(auto &face: primalSurface->allFaces())
    faces.push_back(primalSurface->verticesAroundFace( face ));
  auto digsurf = polyscope::registerSurfaceMesh("Primal surface", primalSurface->positions(), faces);
  digsurf->rescaleToUnit();
  digsurf->setEdgeWidth(std::sqrt(h));  //fade-out
  digsurf->setEdgeColor({1.,1.,1.});
}

float h=0.5;
float dh=0.01;
void myCallback()
{
  ++cpt;
  
  ImGui::SliderFloat("h", &h, 0.001, 1.0);
  ImGui::SliderFloat("deltaradius", &dh, 0.0, 1.0);
  ImGui::Checkbox("Screenshots",&screenshots);
  if (ImGui::Button("Go"))
    oneStep(h);
  
  if (ImGui::Button("Toggle"))
    toggle = !toggle;
  
  if((toggle) )//&& (cpt%2==1))
  {
    oneStep(h);
    h-=dh;
    if (h<0.001)     toggle = !toggle;
    
    if (screenshots)
      polyscope::screenshot();
  }
  
}

int main(int argc, char **argv)
{
  polyscope::init();
  
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return 0;
}
