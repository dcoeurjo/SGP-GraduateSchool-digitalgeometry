#include <iostream>
#include <vector>

#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>

#include <DGtal/images/SimpleThresholdForegroundPredicate.h>
#include <DGtal/geometry/volumes/distance/DistanceTransformation.h>
#include <DGtal/geometry/volumes/distance/VoronoiMap.h>

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include "deps/CLI11/CLI11.hpp"


using namespace DGtal;
using namespace Z3i;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
int slice;
int axis;
bool negate=false;

CountedPtr<SH3::BinaryImage> binary_image;

typedef functors::SimpleThresholdForegroundPredicate<SH3::BinaryImage> Predicate;
typedef DistanceTransformation< Z3i::Space, Predicate, Z3i::L2Metric> DT;
typedef DistanceTransformation< Z3i::Space, functors::NotPointPredicate<Predicate>, Z3i::L2Metric>  DTOUTSIDE;

DT *distancemap_inside;
DTOUTSIDE *distancemap_outside;


template<typename Image>
void registerDTSlice(const Image& image, const Domain &domain,
                     Z3i::Integer slice, size_t axis, bool inner)
{
  std::vector<Point> grid;
  std::vector<double> values;
  for(auto y = domain.lowerBound()[1]; y <= domain.upperBound()[1] ; ++y )
    for(auto z = domain.lowerBound()[2]; z <= domain.upperBound()[2] ; ++z )
    {
      double val;
      switch (axis) {
        case 0:
          val =image(Point(slice, y, z));grid.push_back(Point(slice, y, z));
          break;
        case 1: val=image(Point(y,slice, z));grid.push_back(Point(y,slice, z));break;
        case 2: val = image(Point(y, z, slice));grid.push_back(Point(y,z,slice));break;
        default:
          break;
      }
      values.push_back(val);
    }
  auto pspcl= polyscope::registerPointCloud("Slice", grid);
  auto q=pspcl->addScalarQuantity("DT values", values);
  if (inner) //For inner balls, we display them with non scaled radius
    pspcl->setPointRadiusQuantity(q,false);
}



void myCallback()
{
  ImGui::SliderInt("Slice", &slice, binary_image->domain().lowerBound()[0], binary_image->domain().upperBound()[0]);
  ImGui::RadioButton("X axis", &axis, 0); ImGui::SameLine();
  ImGui::RadioButton("Y axis", &axis, 1); ImGui::SameLine();
  ImGui::RadioButton("Z axis", &axis, 2);
  
  ImGui::Checkbox("Outside DT", &negate);
  if (ImGui::Button("Update slice dt"))
  {
    //DT
    Z3i::L2Metric l2;
    if (negate)
      registerDTSlice(*distancemap_outside, binary_image->domain(), slice,axis, false);
    else
      registerDTSlice(*distancemap_inside, binary_image->domain(), slice,axis,true);
  }
}

int main(int argc, char **argv)
{
  polyscope::init();
  
  CLI::App app{"DT demo"};
  std::string filename;
  app.add_option("-i,--input,1", filename, "Input VOL file")->required()->check(CLI::ExistingFile);
  CLI11_PARSE(app,argc,argv);


  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  auto h=1.; //gridstep
  params("surfaceComponents", "All");
  
  binary_image = SH3::makeBinaryImage(filename, params );
  auto K            = SH3::getKSpace( binary_image );
  auto surface      = SH3::makeDigitalSurface( binary_image, K, params );
  auto surfels      = SH3::getSurfelRange( surface, params );
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
  auto primalSurf = polyscope::registerSurfaceMesh("Primal surface",positions, faces)->setEdgeWidth(1.0)->setEdgeColor({1.,1.,1.});

  //Preparing predicates
  Predicate binaryshape(*binary_image, 0);
  functors::NotPointPredicate<Predicate> negpred(binaryshape);
  distancemap_inside = new DT(binary_image->domain(), binaryshape, Z3i::l2Metric);
  distancemap_outside = new DTOUTSIDE(binary_image->domain(), negpred, Z3i::l2Metric);
  
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return 0;
}
