#include <iostream>
#include <vector>
#include <array>
#include <utility>

#include <DGtal/base/Common.h>
#include <DGtal/io/boards/Board2D.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>
#include "DGtal/geometry/curves/ArithmeticalDSL.h"

#include "deps/CLI11/CLI11.hpp"


using namespace DGtal;
using namespace Z2i;

struct Style : public DrawableWithBoard2D
{
  Color myPenColor;
  Color myFillColor;
  Style( const Color & penColor,
                         const Color & fillColor )
  : myPenColor( penColor ), myFillColor( fillColor )
  {}
  virtual void setStyle( Board2D & aboard) const
  {
    aboard.setFillColor( myFillColor); // specifies the fill color.
    aboard.setPenColor( myPenColor );  // specifies the pen color.
  }
};

int main(int argc, char **argv)
{
  
  Board2D board;
  Domain domain(Point(0,0),Point(30,10));
  
  // Construct a standard DSL from a, b, mu
  NaiveDSL<Integer> line( 2, 5, 0 );
  
  board << domain;
  
  Point a(0,0);
  Point b(28,10);
  board.setLineWidth(2.0);
  board.setPenColor(Color::Red);
  board.setLineStyle(LibBoard::Shape::SolidStyle);
  board.drawLine(a[0], a[1], b[0], b[1]);
  board.saveEPS("drawDSS-1.eps");

  
  board << SetMode( b.className(), "Paving" );
  // Draw the DSL points between firstPoint and lastPoint
  for ( auto it = line.begin(a); (*it)[0]<= 30; ++it )
    board << *it; //Draw the point
  board.setLineWidth(2.0);
  board.setPenColor(Color::Red);
  board.setLineStyle(LibBoard::Shape::SolidStyle);
  board.drawLine(a[0], a[1], b[0], b[1]);
  board.saveEPS("drawDSS-2.eps");
  
  
  board.clear();
  board << domain;
  // Draw the DSL points between firstPoint and lastPoint
  auto mycol=[](unsigned int i){       std::cout<<i<<" "<<i/5<<std::endl;
    switch(i / 5){
      case 0: return Color::Red;
      case 1: return Color::Blue;
      case 3: return Color::Gray;
      case 4: return Color::Green;
      case 5: return Color::Purple;
      case 6: return Color::Lime;
      case 7: return Color::Yellow;
    }  };
  auto cpt=0;
  for ( auto it = line.begin(a); (*it)[0]<= 30; ++it, ++cpt )
  {
    auto color = mycol(cpt);
    std::cout<<color<<std::endl;
    board.setFillColor( Color::Blue );
    board << CustomStyle( it->  className()+"/Paving", new Style( color,color ) )
          << *it; //Draw the point
   }
  board.saveEPS("drawDSS-3.eps");
  
  return 0;
}
