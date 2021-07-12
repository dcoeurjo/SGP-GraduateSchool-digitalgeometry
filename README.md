# SGP-GraduateSchool-digitalgeometry
[![C/C++ CI](https://github.com/dcoeurjo/SGP-GraduateSchool-digitalgeometry/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/dcoeurjo/SGP-GraduateSchool-digitalgeometry/actions/workflows/c-cpp.yml)

This repository contains material for the "Digital Geometry" talk at
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

# Slides

* [Keynote](https://perso.liris.cnrs.fr/david.coeurjolly/talk/digital-geometry/GraduateSchool-DigitalGeometry.key)
* [PDF export](https://perso.liris.cnrs.fr/david.coeurjolly/talk/digital-geometry/GraduateSchool-DigitalGeometry.pdf)
 
# Authors

* [David Coeurjolly](http://perso.liris.cnrs.fr/david.coeurjolly)
* [Jacques-Olivier Lachaud](http://www.lama.univ-savoie.fr/pagesmembres/lachaud/People/LACHAUD-JO/person.html)

# Annotated bibliography

| WARNING: Work in progress |
| --- |

## Topology, preserving homotopy

* [Bertrand94] Bertrand, Gilles. "Simple points, topological numbers and geodesic neighborhoods in cubic grids." Pattern recognition letters 15.10 (1994): 1003-1011.
  - homotopic thinning on voxel shapes baded on simple points in 2D/3D
  - easy definitions based on connected components of foreground / background

* [CB08] Couprie, Michel, and Gilles Bertrand. "New characterizations of simple points in 2D, 3D, and 4D discrete spaces." IEEE Transactions on Pattern Analysis and Machine Intelligence 31.4 (2008): 637-648.
  - simple points up to 4D
 
* [BC94] Bertrand, Gilles, and Michel Couprie. "On parallel thinning algorithms: minimal non-simple sets, P-simple points and critical kernels." Journal of Mathematical Imaging and Vision 35.1 (2009): 23-35.
  - recall on critical kernels,
  - how they induce P-simple points, i.e. points that can be removed in parallel while preserving homotopy

* [YLJ18] Yan, Yajie, David Letscher, and Tao Ju. "Voxel cores: Efficient, robust, and provably good approximation of 3d medial axes." ACM Transactions on Graphics (TOG) 37.4 (2018): 1-13.
  - homotopy equivalence between a (smooth enough) shape and its voxelization,
  - definition of an homotopic medial axis (voxel core).

## geometry of digitized surface vs continuous surface, multigrid convergence

* [LT16] Lachaud, Jacques-Olivier, and Boris Thibert. "Properties of gauss digitized shapes and digital surface integration." Journal of Mathematical Imaging and Vision 54.2 (2016): 162-180.
  - hausdorff closeness between both continuous surface and digitized surface
  - locii of non-injectivity of projection operator, reduces to zero asymptotically
  - locii of non-manifoldness of digitized surface, reduces to zero asymptotically
  - multigrid convergence of digital integral  

* [CLL14] Coeurjolly, David, Jacques-Olivier Lachaud, and Jérémy Levallois. "Multigrid convergent principal curvature estimators in digital geometry." Computer Vision and Image Understanding 129 (2014): 27-41.
  - digital integral invariant estimators (II)
  - multigrid convergence of principal curvatures and principal directions II estimators

* [LTC17] Lachaud, Jacques-Olivier, David Coeurjolly, and Jérémy Levallois. "Robust and convergent curvature and normal estimators with digital integral invariants." Modern Approaches to Discrete Curvature. Springer, Cham, 2017. 293-348.
  - extension of multigrid convergence results of [CLL14] to more general surfaces
  - multigrid convergence of normal vector estimator

* [LRT19] Lachaud, Jacques-Olivier, Pascal Romon, and Boris Thibert. "Corrected curvature measures." (2019).
  - new mean and Gaussian curvature estimators based on corrected normal current
  - valid for any piecewise C^1-smooth surface (even with singularities), like digital surfaces, but also like polyhedral surfaces
  - stability theorem + multigrid convergence for digital surface + multigrid convergent normal estimator
  
* [LRTC20] Lachaud, Jacques-Olivier, Pascal Romon, Boris Thibert, and David Coeurjolly. "Interpolated corrected curvature measures for polygonal surfaces." Computer Graphics Forum. Vol. 39. No. 5. 2020.
  - simple analytic formula of corrected normal current for triangulated and quadrangulated surfaces

