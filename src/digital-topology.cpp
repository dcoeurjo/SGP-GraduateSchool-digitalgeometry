#include <iostream>
#include <vector>

#include <DGtal/base/Common.h>
#include <DGtal/io/boards/Board2D.h>
#include <DGtal/helpers/StdDefs.h>

// #include "deps/CLI11/CLI11.hpp"

int main(int argc, char **argv)
{
  using namespace DGtal;
  using namespace Z2i;
  Board2D board;
  Domain domain( Point(-5,-5), Point(5,5) );

  {
    // Build curve
    DigitalSet curve_set( domain );
    std::vector< Point > V =
      { Point(-3,0), Point(-3,1), Point(-3,2),
        Point(-2,2), Point(-2,3), Point(-1,3) };
    for ( auto p : V ) {
      curve_set.insert( Point(  p[ 0 ],  p[ 1 ] ) );
      curve_set.insert( Point( -p[ 1 ],  p[ 0 ] ) );
      curve_set.insert( Point( -p[ 0 ], -p[ 1 ] ) );
      curve_set.insert( Point(  p[ 1 ], -p[ 0 ] ) );
    }
    // Build complement
    DigitalSet comp_curve_set( domain );
    for ( auto p : domain )
      if ( curve_set.find( p ) == curve_set.end() )
        comp_curve_set.insertNew( p );
    Adj4 adj4; Adj8 adj8;
    DigitalTopology< Adj4, Adj8 > dig_topo_4_8( adj4, adj8 );
    DigitalTopology< Adj8, Adj4 > dig_topo_8_4( adj8, adj4 );
    Object4_8 curve_4( dig_topo_4_8, curve_set );
    Object8_4 comp_curve_4( dig_topo_8_4, comp_curve_set );
    board << domain
          << SetMode( curve_4.className(), "DrawAdjacencies" )
          << CustomStyle( curve_4.className() + "/DrawAdjacencies",
                          new CustomColors( Color::Black, Color::Green ) )
          << curve_4
          << CustomStyle( curve_4.className() + "/DrawAdjacencies",
                          new CustomColors( Color::Black, Color( 230, 230, 255, 255 ) ) )
          << comp_curve_4;
    board.saveEPS( "curve_4_8.eps" );
  }

  {
    // Build curve
    DigitalSet curve_set( domain );
    std::vector< Point > V =
      { Point(-3,0), Point(-3,1), Point(-2,2), Point(-1,3) };
    for ( auto p : V ) {
      curve_set.insert( Point(  p[ 0 ],  p[ 1 ] ) );
      curve_set.insert( Point( -p[ 1 ],  p[ 0 ] ) );
      curve_set.insert( Point( -p[ 0 ], -p[ 1 ] ) );
      curve_set.insert( Point(  p[ 1 ], -p[ 0 ] ) );
    }
    // Build complement
    DigitalSet comp_curve_set( domain );
    for ( auto p : domain )
      if ( curve_set.find( p ) == curve_set.end() )
        comp_curve_set.insertNew( p );
    Adj4 adj4; Adj8 adj8;
    DigitalTopology< Adj4, Adj8 > dig_topo_4_8( adj4, adj8 );
    DigitalTopology< Adj8, Adj4 > dig_topo_8_4( adj8, adj4 );
    Object8_4 curve_8( dig_topo_8_4, curve_set );
    Object4_8 comp_curve_8( dig_topo_4_8, comp_curve_set );
    Object8_4 bad_comp_curve( dig_topo_8_4, comp_curve_set );
    board << domain
          << SetMode( curve_8.className(), "DrawAdjacencies" )
          << CustomStyle( curve_8.className() + "/DrawAdjacencies",
                          new CustomColors( Color::Black, Color::Green ) )
          << curve_8
          << CustomStyle( curve_8.className() + "/DrawAdjacencies",
                          new CustomColors( Color::Black, Color( 230, 230, 255, 255 ) ) )
          << comp_curve_8;
    board.saveEPS( "curve_8_4.eps" );
    board << domain
          << SetMode( curve_8.className(), "DrawAdjacencies" )
          << CustomStyle( curve_8.className() + "/DrawAdjacencies",
                          new CustomColors( Color::Black, Color::Red ) )
          << curve_8
          << CustomStyle( curve_8.className() + "/DrawAdjacencies",
                          new CustomColors( Color::Black, Color( 230, 230, 255, 255 ) ) )
          << bad_comp_curve;
    board.saveEPS( "curve_8_8.eps" );
  }
  return 0;
}
