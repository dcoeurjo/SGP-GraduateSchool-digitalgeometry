# SGP-GraduateSchool-digitalgeometry
[![C/C++ CI](https://github.com/dcoeurjo/SGP-GraduateSchool-digitalgeometry/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/dcoeurjo/SGP-GraduateSchool-digitalgeometry/actions/workflows/c-cpp.yml)

This repository contains material for the "Digtial Geometry" talk at
the
[Graduate School](https://sgp2021.github.io/program/#graduate-school) of the
[Symposium on Geometry Processing](https://sgp2021.github.io). It
contains the slides of the lecture and C++ codes used to generate the
images / animations (using [DGtal](https://dgtal.org)).

![](https://raw.githubusercontent.com/dcoeurjo/SGP-GraduateSchool-digitalgeometry/main/img/bunny-geodesics.png)

# Abstract

Digital Geometry is about the processing of topological and geometrical objects defined in regular lattices (e.g. collection of voxels in 3d). Whereas representing quantities on regular, hierarchical or adaptive grids is a classical approach to spatially discretize a domain, processing the geometry of such objects requires us to revisit classical results from continuous or discrete mathematics. In this course, we will review tools and results that have been designed specifically to the geometry processing in Z^d. More precisely, we will present how processing regularly spaced data with integer coordinate embeddings may impact computational geometry algorithms, and how stability results (multigrid convergence) of differential quantities estimators (curvature tensor, Laplace-Beltrami,..) on boundaries of digital objects can be designed. Finally, we will present some elements of discrete calculus on digital surfaces. Lastly, we will briefly give a demo of the DGtal library (dgtal.org) which contains a wide class of algorithms dedicated to the processing of such specific data.

# How to build the examples


Just clone this repository and its submodules:

```
git clone https://github.com/dcoeurjo/SGP-GraduateSchool-digitalgeometry.git
git submodule update  --recursive --init
```


Then you can compile the files using `cmake/make` (using linux/macOS
Makefile target, check you `cmake -h` Generators for alternative platform):

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
