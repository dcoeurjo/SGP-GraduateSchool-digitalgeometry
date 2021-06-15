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

CountedPtr<SH3::BinaryImage> binary_image;
CountedPtr<SH3::DigitalSurface > surface;
SH3::KSpace K;
SH3::SurfelRange surfels;
Parameters params;
polyscope::SurfaceMesh *digsurf;
float rad=1.0;

void oneStep(double rad)
{
  //Computing some differential quantities
  params("r-radius", rad);
  auto normalsII = SHG3::getIINormalVectors(binary_image, surfels, params);
  auto Mcurv     = SHG3::getIIMeanCurvatures(binary_image, surfels, params);
  
  //Attaching quantities
  digsurf->addFaceVectorQuantity("II normal vectors", normalsII);
  digsurf->addFaceScalarQuantity("II mean curvature", Mcurv);
}

float drad=0.1;
void myCallback()
{
  ++cpt;
  
  ImGui::SliderFloat("radius", &rad, 1.0, 10.0);
  ImGui::SliderFloat("deltaradius", &drad, 0.0, 1.0);
  if (ImGui::Button("Go"))
    oneStep(rad);
  
  if (ImGui::Button("Toggle"))
    toggle = !toggle;
  
  if((toggle) )//&& (cpt%2==1))
  {
    oneStep(rad);
    rad+=drad;
    if (rad>10.0)     toggle = !toggle;

    if (screenshots)
      polyscope::screenshot();
  }
  
}

int main(int argc, char **argv)
{
  polyscope::init();
  
  CLI::App app{"DT demo"};
  std::string filename;
  app.add_option("-i,--input,1", filename, "Input VOL file")->required()->check(CLI::ExistingFile);
  CLI11_PARSE(app,argc,argv);
  
  params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  binary_image = SH3::makeBinaryImage(filename, params );
  K            = SH3::getKSpace( binary_image );
  surface      = SH3::makeDigitalSurface( binary_image, K, params );
  surfels      = SH3::getSurfelRange( surface, params );
  auto embedder     = SH3::getCellEmbedder( K );
  
  //Need to convert the faces
  std::vector<std::vector<size_t>> faces;
  std::vector<RealPoint> positions;
  unsigned int cpt=0;
  for(auto &surfel: surfels)
  {
    auto verts = SH3::getPrimalVertices(K, surfel, false );
    for(auto &v: verts)
      positions.push_back(embedder(v));
    
    std::vector<size_t> face={cpt, cpt+1, cpt+2,cpt+3};
    cpt+=4;
    faces.push_back(face);
  }
  digsurf = polyscope::registerSurfaceMesh("Primal",positions, faces)->setEdgeWidth(1.0)->setEdgeColor({1.,1.,1.});

  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return 0;
}
