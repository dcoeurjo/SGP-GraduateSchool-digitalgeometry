#include <iostream>
#include <vector>
#include <array>
#include <utility>

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
bool negate=true;
bool subSample=false;

CountedPtr<SH3::BinaryImage> binary_image;

typedef functors::SimpleThresholdForegroundPredicate<SH3::BinaryImage> Predicate;
typedef DistanceTransformation< Z3i::Space, Predicate, Z3i::L2Metric> DT;
typedef DistanceTransformation< Z3i::Space, functors::NotPointPredicate<Predicate>, Z3i::L2Metric>  DTOUTSIDE;

typedef VoronoiMap< Z3i::Space, Predicate, Z3i::L2Metric> VoroMap;
typedef VoronoiMap< Z3i::Space, functors::NotPointPredicate<Predicate>, Z3i::L2Metric>  VoroMapOutside;

VoroMap *voronoiMap;
VoroMapOutside *voronoiMapOutside;


template<typename Image>
void registerSlice(const Image& image, const Domain &domain,
                     Z3i::Integer slice, size_t axis, bool inner,
                   bool subSample)
{
  Point p;
  auto dim1 = (axis+1)%3;
  auto dim2 = (axis+2)%3;
  auto width1 = domain.upperBound()[dim1] - domain.lowerBound()[dim1] + 1;
  auto width2 = domain.upperBound()[dim2] - domain.lowerBound()[dim2] + 1;
  unsigned int cpt=0u;
  
  if (subSample)
  {
    width1 /= 2;
    width2 /= 2;
  }
  
  std::vector<Point> vertices(width1*width2);
  std::vector<double> values(width1*width2);
  std::vector<unsigned int> sites(width1*width2);
  std::vector<std::array<unsigned int,4>> faces(width1*width2);

  std::vector<Point> grid;
  std::vector<double> radii;
  auto cptF=0;
  for(auto v = domain.lowerBound()[dim2]; v <= domain.upperBound()[dim2] ; ++v )
  {
    for(auto u = domain.lowerBound()[dim1]; u <= domain.upperBound()[dim1] ; ++u )
    {
      switch (axis) {
        case 0: p=Point(slice, u, v);break;
        case 1: p=Point(u,slice, v);break;
        case 2: p=Point(u, v, slice);break;
      }
      Point q = image(p); //Voronoi site
      vertices[cpt] = p;
      if ((u <  domain.upperBound()[dim1] ) && (v < domain.upperBound()[dim2]))
      {
        if ((subSample) && ( u+1<domain.upperBound()[dim1] ) && (v+1 < domain.upperBound()[dim2]))
          faces[cpt] = {cpt, cpt + 1, cpt + width1+1 , cpt+width1};
        else
          faces[cpt] = {cpt, cpt + 1, cpt + width1+1 , cpt+width1};

      }
      values[cpt]=((p-q).norm()); //Distance
      sites[cpt]=(13*(q[0] + q[1]*451 + q[2]*311) % 1024); //Voronoi cells
      if ((inner)&& (p!=q))
      {
        grid.push_back(p);  //Grid
        radii.push_back((p-q).norm()); //Distance
      }
      ++cpt;
      if (subSample)
        ++u;
    }
    if (subSample)
      ++v;
  }
  
  auto psm = polyscope::registerSurfaceMesh("Slice", vertices, faces);
  psm->addVertexDistanceQuantity("DT", values);
  psm->addFaceScalarQuantity("Sites", sites);
  if (inner)
  {
    auto pspcl= polyscope::registerPointCloud("Balls", grid);
    auto q=pspcl->addScalarQuantity("DT values", radii);
    pspcl->setPointRadiusQuantity(q,false);
  }
}


bool toggle=false;
bool toggleScale=false;
bool toggle1D=false;
int cpt=0;
bool screenshots=false;
float scaleAxis=1.0;

void myCallback()
{
  ++cpt;
  ImGui::SliderInt("Slice", &slice, binary_image->domain().lowerBound()[0], binary_image->domain().upperBound()[0]);
  ImGui::RadioButton("X axis", &axis, 0); ImGui::SameLine();
  ImGui::RadioButton("Y axis", &axis, 1); ImGui::SameLine();
  ImGui::RadioButton("Z axis", &axis, 2);
  
  ImGui::Checkbox("Outside DT", &negate);
  ImGui::Checkbox("Screenshots", &screenshots);
  ImGui::Checkbox("SubSample (for OpenGL prefs)", &subSample);
  if (ImGui::Button("Update slice dt"))
  {
    //DT
    Z3i::L2Metric l2;
    if (negate)
      registerSlice(*voronoiMapOutside, binary_image->domain(), slice,axis, false,subSample);
    else
      registerSlice(*voronoiMap, binary_image->domain(), slice,axis,true,subSample);
  }
  
  ImGui::SliderFloat("scale Axis", &scaleAxis, 1.0,2.0);
  if (ImGui::Button("RDMA"))
  {
    ImageContainerBySTLVector<Domain, uint64_t> squareDT(binary_image->domain());
    for(auto p: binary_image->domain())
    {
      auto q = voronoiMap->operator()(p);
      if (p!=q)
        squareDT.setValue(p, scaleAxis*scaleAxis*(p-q).squaredNorm());
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
        radii.push_back(1.0/scaleAxis*std::sqrt(rdma(p)) );
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
    registerSlice(*voronoiMapOutside, binary_image->domain(), slice,axis, false,subSample);
  
    if (screenshots)
      polyscope::screenshot();
  }
  
  if (ImGui::Button("Toggle 1D"))
    toggle1D = !toggle1D;
  
  if((toggle1D) )//&& (cpt%2==1))
  {
    slice++;
    if (slice > binary_image->domain().upperBound()[axis])
      toggle1D = !toggle1D;
    registerSlice(*voronoiMapOutside, binary_image->domain(), slice,axis, false,subSample);
    
    if (screenshots)
      polyscope::screenshot();
  }
  
  if (ImGui::Button("Toggle Scale"))
    toggleScale = !toggleScale;
  if((toggleScale) )//&& (cpt%2==1))
  {
    
    if (scaleAxis > 10.0)
      toggleScale = !toggleScale;
    ImageContainerBySTLVector<Domain, uint64_t> squareDT(binary_image->domain());
    for(auto p: binary_image->domain())
    {
      auto q = voronoiMap->operator()(p);
      if (p!=q)
        squareDT.setValue(p, scaleAxis*scaleAxis*(p-q).squaredNorm());
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
        radii.push_back(1.0/scaleAxis*std::sqrt(rdma(p)) );
      }
    }
    auto psrdma = polyscope::registerPointCloud("RDMA", centers);
    auto q= psrdma->addScalarQuantity("radius", radii);
    psrdma->setPointRadiusQuantity(q,false);
    scaleAxis += 0.05;
    
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
