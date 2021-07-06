#include <iostream>
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
                             std::string name )
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  auto h=1.; //gridstep
  params( "closed", 1)("surfaceComponents", "AnyBig");
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
      positions.push_back(embedder(v));
    
    std::vector<size_t> face={cpt, cpt+1, cpt+2,cpt+3};
    cpt+=4;
    faces.push_back(face);
  }
  auto primalSurf = polyscope::registerSurfaceMesh( name, positions, faces)
    ->setEdgeWidth(1.0)->setEdgeColor({1.,1.,1.});
}

/// Register to polyscope the boundary surfels of a given binary image
/// \a bimage.
void registerPolygonalSurface
( CountedPtr< SH3::PolygonalSurface > dual_surface,
  std::string name )
{
  std::vector< std::vector< std::size_t> > faces;
  std::vector< RealPoint > positions;
  for ( Vertex v = 0; v < dual_surface->nbVertices(); v++ )
    positions.push_back( dual_surface->position( v ) );
  for ( Face f = 0; f < dual_surface->nbFaces(); f++ )
    faces.push_back( dual_surface->verticesAroundFace( f ) );
  auto dualSurf = polyscope::registerSurfaceMesh( name, positions, faces)
    ->setEdgeWidth(2.0)->setEdgeColor({1.,0.,0.});
}

// Removes a peel of simple points onto voxel object.
// bool oneStep()
// {
//   DigitalSet::Iterator it, itE;
//   DigitalSet & S = object->pointSet();
//   std::queue< Point > Q;
//   for ( auto&& p : S )
//     if ( object->isSimple( p ) )
//       Q.push( p );
//   int nb_simple = 0;
//   while ( ! Q.empty() )
//     {
//       const auto p = Q.front();
//       Q.pop();
//       if ( object->isSimple( p ) )
//         {
//           S.erase( p );
//           binary_image->setValue( p, false );
//           ++nb_simple;
//         }
//     }
//   trace.info() << "Removed " << nb_simple << " / " << S.size()
//                << " points." << std::endl;
//   registerDigitalSurface( binary_image, "Thinned object" );
//   return nb_simple == 0;
// }

// Polyscope GUI Callback
void mycallback()
{
  // if (ImGui::Button("Run"))
  //   {
  //     oneStep();
  //   }
  // if (ImGui::Button("Screenshots"))
  //   {
  //     bool finished = false;
  //     while ( ! finished )
  //       {
  //         finished = oneStep();
  //         polyscope::screenshot();
  //         polyscope::refresh();
  //       }
  //   }
}

// main program
int main( int argc, char* argv[] )
{
  polyscope::init();

  CLI::App app{"Homotopic Thinning demo"};
  std::string poly_string;
  double      gridstep;
  app.add_option("-p,--polynomial,1", poly_string, "a shape described as a polynomial like x^2+y^2-z^2-5")->required();
  app.add_option("-g,--gridstep,1", gridstep, "specifies the digitization grid step" );
  CLI11_PARSE(app,argc,argv);

  // Read voxel object and hands surfaces to polyscope
  auto params = SH3::defaultParameters();
  params( "polynomial", poly_string )( "gridstep", gridstep );
  std::cout << "gridstep=" << gridstep << std::endl;
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
  registerDigitalSurface( binary_image, "Primal surface" );
  params( "surfelAdjacency", 0 );
  auto primal_surface0  = SH3::makeLightDigitalSurface( binary_image, K, params );
  params( "surfelAdjacency", 1 );
  auto primal_surface1  = SH3::makeLightDigitalSurface( binary_image, K, params );
  // auto grayscale_image = SH3::makeGrayScaleImage
  //   ( binary_image,
  //     [] ( bool v ) { return v ? (unsigned char) 1 : (unsigned char) 0; } );
  auto dual_surface0 = SH3::makeDualPolygonalSurface( primal_surface0 );
  auto dual_surface1 = SH3::makeDualPolygonalSurface( primal_surface1 );
  registerPolygonalSurface( dual_surface0, "Dual surface interior adjacency" );
  registerPolygonalSurface( dual_surface1, "Dual surface exterior adjacency" );
  // std::vector< std::vector< std::size_t> > faces;
  // std::vector< RealPoint > positions;
  // for ( Vertex v = 0; v < dual_surface->nbVertices(); v++ )
  //   positions.push_back( dual_surface->position( v ) );
  // for ( Face f = 0; f < dual_surface->nbFaces(); f++ )
  //   faces.push_back( dual_surface->verticesAroundFace( f ) );
  // auto dualSurf = polyscope::registerSurfaceMesh( "Dual surface", positions, faces)
  //   ->setEdgeWidth(1.0)->setEdgeColor({1.,1.,1.});
  // Give the hand to polyscope
  polyscope::state::userCallback = mycallback;
  polyscope::show();
  return 0;
}
