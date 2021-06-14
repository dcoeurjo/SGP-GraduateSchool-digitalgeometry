#include <iostream>
#include <vector>

#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>

#include <DGtal/images/SimpleThresholdForegroundPredicate.h>
#include <DGtal/geometry/volumes/distance/DistanceTransformation.h>
#include <DGtal/geometry/volumes/distance/VoronoiMap.h>
#include <DGtal/geometry/volumes/distance/ReducedMedialAxis.h>

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

typedef VoronoiMap< Z3i::Space, Predicate, Z3i::L2Metric> VoroMap;
typedef VoronoiMap< Z3i::Space, functors::NotPointPredicate<Predicate>, Z3i::L2Metric>  VoroMapOutside;

VoroMap *voronoiMap;
VoroMapOutside *voronoiMapOutside;


template<typename Image>
void registerDTSlice(const Image& image, const Domain &domain,
                     Z3i::Integer slice, size_t axis, bool inner)
{
  std::vector<Point> grid;
  std::vector<double> values;
  std::vector<unsigned int> sites;
    Point p;
  for(auto y = domain.lowerBound()[1]; y <= domain.upperBound()[1] ; ++y )
    for(auto z = domain.lowerBound()[2]; z <= domain.upperBound()[2] ; ++z )
    {
      switch (axis) {
        case 0: p=Point(slice, y, z);break;
        case 1: p=Point(y,slice, z);break;
        case 2: p=Point(y, z, slice);break;
        default:
          break;
      }
      Point q = image(p);
      if (p!=q)
      {
        grid.push_back(p);  //Grid
        sites.push_back(13*(q[0] + q[1]*451 + q[2]*311) % 1024); //Voronoi cells
        values.push_back((p-q).norm()); //Distance
      }
    }
  auto pspcl= polyscope::registerPointCloud("Slice", grid);
  auto q=pspcl->addScalarQuantity("DT values", values);
  pspcl->addScalarQuantity("Sites", sites);
  if (inner) //For inner balls, we display them with non scaled radius
    pspcl->setPointRadiusQuantity(q,false);
}


bool toggle=false;
int cpt=0;

void myCallback()
{
  ++cpt;
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
      registerDTSlice(*voronoiMapOutside, binary_image->domain(), slice,axis, false);
    else
      registerDTSlice(*voronoiMap, binary_image->domain(), slice,axis,true);
  }
  
  if (ImGui::Button("RDMA"))
  {
    ImageContainerBySTLVector<Domain, uint64_t> squareDT(binary_image->domain());
    for(auto p: binary_image->domain())
    {
      auto q = voronoiMap->operator()(p);
      if (p!=q)
      {
        squareDT.setValue(p, (p-q).squaredNorm());
      }
    }
    typedef PowerMap<ImageContainerBySTLVector<Domain, uint64_t>, L2PowerMetric> PowerMapType;
    PowerMapType powermap(binary_image->domain(), squareDT, Z3i::l2PowerMetric);
    auto rdma = ReducedMedialAxis<PowerMapType>::getReducedMedialAxisFromPowerMap(powermap);
    std::vector<Point> centers;
    std::vector<double> radii;
    for(auto &p: rdma.domain())
    {
      if (rdma(p) != 0)
      {
        centers.push_back(p);
        radii.push_back(std::sqrt(rdma(p)) - 0.5);
      }
    }
    auto psrdma = polyscope::registerPointCloud("RDMA", centers);
    auto q= psrdma->addScalarQuantity("radius", radii);
    psrdma->setPointRadiusQuantity(q,false);
  }
  
  if (ImGui::Button("Toggle"))
    toggle = !toggle;
  
  if((toggle) )//&& (cpt%2==1))
  {
    slice++;
    if (slice > binary_image->domain().upperBound()[axis])
      slice = binary_image->domain().lowerBound()[axis];
    registerDTSlice(*voronoiMapOutside, binary_image->domain(), slice,axis, false);
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
  voronoiMap = new VoroMap(binary_image->domain(), binaryshape, Z3i::l2Metric);
  voronoiMapOutside = new VoroMapOutside(binary_image->domain(), negpred, Z3i::l2Metric);
  
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return 0;
}
