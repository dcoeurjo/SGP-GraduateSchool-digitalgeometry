# SGP-GraduateSchool-digitalgeometry

This repository contains material for the "Digtial Geometry" talk at
the
[Graduate School](https://sgp2021.github.io/program/#graduate-school) of the
[Symposium on Geometry Processing](https://sgp2021.github.io). It
contains the slides of the lecture and C++ codes used to generate the
images / animations (using [DGtal](https://dgtal.org)).

![](https://raw.githubusercontent.com/dcoeurjo/SGP-GraduateSchool-digitalgeometry/main/img/bunny-geodesics.png)


# How to build the examples


Just clone this repository and its submodules:

```
git clone https://github.com/dcoeurjo/SGP-GraduateSchool-digitalgeometry.git
git submodule update  --recursive --init
```


Then you can compile the example file using (illustrated using linux/macOS
Makefile target, check you `cmake` Generators for alternative platform):

```
mkdir build
cd build
cmake ..  -DCMAKE_BUILD_TYPE=Release
make
```

*Note*: to compile DGtal, you would need [boost](boost.org) (only
 headers) and  [zlib](https://www.zlib.net). The project heavily uses [polyscope](http://polyscope.run) for the visualization and UI.


## Authors


* [David Coeurjolly](http://perso.liris.cnrs.fr/david.coeurjolly)
* [Jacques-Olivier Lachaud](http://www.lama.univ-savoie.fr/pagesmembres/lachaud/People/LACHAUD-JO/person.html)
