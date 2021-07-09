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


void oneStepAll(double h)
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params( "polynomial", "goursat" )( "gridstep", h );
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
  digsurf->rescaleToUnit(); digsurf->setEdgeWidth(h*h);  digsurf->setEdgeColor({1.,1.,1.});
  
  //Computing some differential quantities
  params("r-radius", 5*std::pow(h,-2.0/3.0));
  auto Mcurv     = SHG3::getIIMeanCurvatures(binary_image, surfels, params);
  auto normalsII = SHG3::getIINormalVectors(binary_image, surfels, params);
  auto KTensor   = SHG3::getIIPrincipalCurvaturesAndDirections(binary_image, surfels, params);  //Recomputing...
 
  std::vector<double> Gcurv(surfels.size()),k1(surfels.size()),k2(surfels.size());
  std::vector<RealVector> d1(surfels.size()),d2(surfels.size());
  auto i=0;
  for(auto &t: KTensor) //AOS->SOA
  {
    k1[i]    = std::get<0>(t);
    k2[i]    = std::get<1>(t);
    d1[i]    = std::get<2>(t);
    d2[i]    = std::get<3>(t);
    Gcurv[i] = k1[i]*k2[i];
    ++i;
  }
  
  //Attaching quantities
  digsurf->addFaceVectorQuantity("II normal vectors", normalsII, polyscope::VectorType::AMBIENT);
  digsurf->addFaceScalarQuantity("II mean curvature", Mcurv);
  digsurf->addFaceScalarQuantity("II Gaussian curvature", Gcurv);
  digsurf->addFaceScalarQuantity("II k1 curvature", k1);
  digsurf->addFaceScalarQuantity("II k2 curvature", k2);
  digsurf->addFaceVectorQuantity("II first principal direction", d1, polyscope::VectorType::AMBIENT);
  digsurf->addFaceVectorQuantity("II second principal direction", d2, polyscope::VectorType::AMBIENT);
}


void oneStep(double h)
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params( "polynomial", "goursat" )( "gridstep", h );
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
  digsurf->setEdgeWidth(h*h);  //fade-out
  digsurf->setEdgeColor({1.,1.,1.});
  
  //Computing some differential quantities
  params("r-radius", 3*std::pow(h,-2.0/3.0));
  auto Mcurv     = SHG3::getIIMeanCurvatures(binary_image, surfels, params);
  
  //Attaching quantities
  //digsurf->addFaceVectorQuantity("II normal vectors", normalsII);
  digsurf->addFaceScalarQuantity("II mean curvature", Mcurv);
}

float h=1.0;
float dh=0.01;
void myCallback()
{
  ++cpt;
  
  ImGui::SliderFloat("h", &h, 0.001, 1.0);
  ImGui::SliderFloat("deltaradius", &dh, 0.0, 1.0);
  ImGui::Checkbox("Screenshots",&screenshots);
  if (ImGui::Button("Go"))
    oneStep(h);
  
  if (ImGui::Button("All estimators"))
    oneStepAll(h);
  
  
  if (ImGui::Button("Toggle"))
    toggle = !toggle;
  
  if((toggle) )//&& (cpt%2==1))
  {
    oneStep(h);
    h-=dh;
    if (h<0.0001)     toggle = !toggle;
    
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
