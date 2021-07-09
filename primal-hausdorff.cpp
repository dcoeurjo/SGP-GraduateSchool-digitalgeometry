#include <iostream>
#include <sstream>
#include <vector>
#include <array>
#include <utility>

#include "deps/CLI11/CLI11.hpp"

#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>


#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"


using namespace DGtal;
using namespace Z3i;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;

typedef PolygonalSurface< RealPoint > PolySurf;
typedef PolySurf::Vertex Vertex;
typedef PolySurf::Face   Face;

CountedPtr< PolySurf > dual_surface;

/// Register to polyscope the boundary surfels of a given binary image
/// \a bimage.
void registerDigitalSurface( CountedPtr< SH3::BinaryImage > bimage,
                             std::string name, double h )
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params( "closed", 1)("surfaceComponents", "AnyBig")( "gridstep", h );
  auto K            = SH3::getKSpace( bimage );
  auto surface      = SH3::makeDigitalSurface( bimage, K, params );
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
      positions.push_back(embedder(v)*h);
    
    std::vector<size_t> face={cpt, cpt+1, cpt+2,cpt+3};
    cpt+=4;
    faces.push_back(face);
  }
  auto primalSurf = polyscope::registerSurfaceMesh( name, positions, faces)
    ->setSurfaceColor( {0.11, 0.4, 0.9} )
    ->setEdgeWidth(2.0)
    ->setEdgeColor({1.,1.,1.})
    ->setTransparency( 0.5 );
;
}

/// Register to polyscope the boundary surfels of a given binary image
/// \a bimage.
void registerPolygonalSurface
( CountedPtr< SH3::PolygonalSurface > dual_surface,
  const std::vector< RealPoint > & positions,
  std::string name )
{
  std::vector< std::vector< std::size_t> > faces;
  for ( Face f = 0; f < dual_surface->nbFaces(); f++ )
    faces.push_back( dual_surface->verticesAroundFace( f ) );
  auto dualSurf = polyscope::registerSurfaceMesh( name, positions, faces);
  //    ->setEdgeWidth(0.0)->setEdgeColor({1.,0.,0.});
}


// Polyscope GUI Callback
void mycallback()
{
}

// main program
int main( int argc, char* argv[] )
{
  polyscope::init();

  CLI::App app{"Homotopic Thinning demo"};
  std::string poly_string;
  double      gridstep1, gridstep2;
  app.add_option("-p,--polynomial,1", poly_string, "a shape described as a polynomial like x^2+y^2-z^2-5")->required();
  app.add_option("-1,--gridstep1,1", gridstep1, "specifies the first digitization grid step" );
  app.add_option("-2,--gridstep2,1", gridstep2, "specifies the first digitization grid step" );
  CLI11_PARSE(app,argc,argv);

  for ( ; gridstep1 >= gridstep2; gridstep1 /= 2.0 )
    {
      // Read voxel object and hands surfaces to polyscope
      auto params = SH3::defaultParameters();
      params( "polynomial", poly_string )( "gridstep", gridstep1 );
      std::cout << "gridstep=" << gridstep1 << std::endl;
      params( "minAABB", -10.0 );
      params( "maxAABB",  10.0 );
      params( "offset",    1.0 );
      params( "closed",    1   );
      auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
      auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
      auto K               = SH3::getKSpace( params );
      auto binary_image    = SH3::makeBinaryImage( digitized_shape,
                                                   SH3::Domain(K.lowerBound(),K.upperBound()),
                                                   params );
      std::ostringstream sname;
      sname << "Primal surface " << gridstep1; 
      registerDigitalSurface( binary_image, sname.str(), gridstep1 );
    }
  
  auto params = SH3::defaultParameters();
  params( "polynomial", poly_string )( "gridstep", gridstep2 );
  std::cout << "gridstep=" << gridstep2 << std::endl;
  params( "minAABB", -10.0 );
  params( "maxAABB",  10.0 );
  params( "offset",    1.0 );
  params( "closed",    1   );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K               = SH3::getKSpace( params );
  auto binary_image    = SH3::makeBinaryImage( digitized_shape,
                                               SH3::Domain(K.lowerBound(),K.upperBound()),
                                               params );
  auto surface         = SH3::makeLightDigitalSurface( binary_image, K, params );
  auto surfels         = SH3::getSurfelRange( surface, params );
  auto positions       = SHG3::getPositions( implicit_shape, K, surfels, params );
  auto dual_surface    = SH3::makeDualPolygonalSurface( surface );
  registerPolygonalSurface( dual_surface, positions, "True surface"  );
  // Give the hand to polyscope
  polyscope::state::userCallback = mycallback;
  polyscope::show();
  return 0;
}
