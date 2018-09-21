//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include <array>

#include "Types.h"


namespace destroyer {

/*

 ThreeDimVector class

 Three dimensional array container class.

*/

template <class T>
class ThreeDimVector
{
public:
    // Constructor requires the extents in each dimension.
    ThreeDimVector(int size_i, int size_j, int size_k);
    ~ThreeDimVector();

    void set(int i, int j, int k, T val);
    T get(int i, int j, int k) const;

private:
    T* container_;
    int size_i_;
    int size_j_;
    int size_k_;
};

template<class T>
ThreeDimVector<T>::ThreeDimVector(int size_i, int size_j, int size_k) {
    // Allocate linear array using raw pointer.
    // Note that contained values are uninitialized.
    container_ = new T[size_i * size_j * size_k];
    size_i_ = size_i;
    size_j_ = size_j;
    size_k_ = size_k;
}

template<class T>
ThreeDimVector<T>::~ThreeDimVector() {
    // De-allocate array.
    // Note that contained items are not de-allocated: they are responsible for their own garbage collection.
    delete [] container_;
}

template<class T>
void ThreeDimVector<T>::set(int i, int j, int k, T val) {
    auto linear_index = (i * size_j_ * size_k_) + (j * size_k_) + (k * 1);
    container_[linear_index] = val;
}

template<class T>
T ThreeDimVector<T>::get(int i, int j, int k) const {
    auto linear_index = (i * size_j_ * size_k_) + (j * size_k_) + (k * 1);
    return container_[linear_index];
}



/*

 BCCLatticeGenerator class

 Fills an empty TetMesh with a lattice of tetrahedrons comprising the triangulation of the
 body centered cubic lattice spanning the specified bounding box. Edge length of tetrahedra is specified.

 The BCC lattice consists of nodes at every point of a Cartesian grid along with the nodes located at the cell
 centers. These node locations may be viewed as belonging to two interlaced grids. The BCC lattice is the Delaunay
 complex of the interlaced grid nodes, and thus possesses all properties of a Delaunay tetrahedralization.

*/

// Minimum number of BCC Lattice cells for each dimension
#define MIN_GRID_COUNT 4.0

class BCCLatticeGenerator
{
    using GridPtr = typename std::unique_ptr<ThreeDimVector<TetNodeRef> >;

public:

    // Constructor should be supplied with the pointer to an instantiated (and empty) TetMesh,
    // the target edge length for the tetrahedra, and the world space bounding box.
    BCCLatticeGenerator(TetMeshPtr tet_mesh, Real edge_length,
                        Real min_x, Real min_y, Real min_z,
                        Real max_x, Real max_y, Real max_z);

    ~BCCLatticeGenerator() = default;

    // Tesselate the bounding box domain. The contained TetMesh will be filled with a tetrahedral mesh
    // essentially identical to the Delaunay tetrahedralization of the BCC nodes within the prescribed bounds.
    void FillTetMesh();

private:
    void AdjustTesselatedSpace(Real min_x, Real min_y, Real min_z, Real max_x, Real max_y, Real max_z);
    GridPtr GenerateGridNodes();
    void GenerateTetrahedraTemplate(int origin_x, int origin_y, int origin_z);

private:
    TetMeshPtr tet_mesh_;
    Real yz_edge_length_;
    Real x_edge_length_;
    std::array<Real,3> lower_corner_;
    std::array<int,3> element_count_;
    GridPtr grid_;
};

}; // namespace destroyer
