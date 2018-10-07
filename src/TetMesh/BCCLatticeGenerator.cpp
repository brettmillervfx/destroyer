//
// Created by Brett Miller on 8/15/18.
//


#include "TetMesh/BCCLatticeGenerator.h"

#include <cmath>
#include <algorithm>

#include "TetMesh/TetMesh.h"


namespace destroyer {

/*

 BCCLatticeGenerator uses the method described in "Tetrahedral mesh generation based on space indicator functions",
 Friess, et al. By rotating the standard BCC lattice 45 degrees around its x axis, we dramatically simplify filling the
 structure by defining a template of tetrahedrons that easily tile on the resulting irregular lattice of nodes.

*/

BCCLatticeGenerator::BCCLatticeGenerator(TetMeshPtr tet_mesh, Real edge_length,
                                         Real min_x, Real min_y, Real min_z,
                                         Real max_x, Real max_y, Real max_z) {

    tet_mesh_ = tet_mesh;

    // The transformed lattice of nodes is irregular: the normally cubic grid is forshortened
    // in the x axis. The lengths of the grid intervals result in tetrahedral edges of the desired
    // length. See the Friess paper for illustrations.
    yz_edge_length_ = (sqrt(2.0)*edge_length) / 2.0;
    x_edge_length_ = edge_length / 2.0;

    // Check for valid bounds and valid edgeLength and correct if necessary.
    AdjustTesselatedSpace(min_x, min_y, min_z, max_x, max_y, max_z);

}

void BCCLatticeGenerator::FillTetMesh() {

    // Lay in nodes of the two offset grids.
    grid_ = GenerateGridNodes();

    // Advance through the grid, building a template of six tetrahedra at each iteration.
    // Adjacent i rows are offset by 1 to line up tets.
    for (Index i = 0; i<element_count_[0]-5; i+=2)
        for (Index j = 0; j<element_count_[1]-1; j++)
            for (Index k = 0; k<element_count_[2]-1; k++) {
                auto ioffset = (j + k) % 2;
                GenerateTetrahedraTemplate(i + ioffset, j, k);
            }

}

void BCCLatticeGenerator::AdjustTesselatedSpace(Real min_x, Real min_y, Real min_z,
                                                Real max_x, Real max_y, Real max_z) {

    // Adjust the bounding box by expanding it to fit an integer number of cells in
    // each dimension, then pad it to ensure that the grid has the minimum number of cells
    // to allow a full template stamp. Calculate the index extents in each dimension.

    // Get the length of the bounding box and the center.
    auto length_x = max_x - min_x;
    auto length_y = max_y - min_y;
    auto length_z = max_z - min_z;
    auto center_x = min_x + (length_x/2.0);
    auto center_y = min_y + (length_y/2.0);
    auto center_z = min_z + (length_z/2.0);

    // We want at least a minimal grid to allow for sufficient overlap.
    // Find the correct bounding box dimensions to allow optimal fit.
    auto adjusted_length_x = x_edge_length_ * std::max(MIN_GRID_COUNT,ceil(length_x / x_edge_length_)+5);
    auto adjusted_length_y = yz_edge_length_ * std::max(MIN_GRID_COUNT,ceil(length_y / yz_edge_length_)+2);
    auto adjusted_length_z = yz_edge_length_ * std::max(MIN_GRID_COUNT,ceil(length_z / yz_edge_length_)+2);

    // Adjust lower corner to adjusted bounds.
    lower_corner_[0] = center_x - (adjusted_length_x/2.0);
    lower_corner_[1] = center_y - (adjusted_length_y/2.0);
    lower_corner_[2] = center_z - (adjusted_length_z/2.0);

    // Register the number of nodes in each dimension of the offset grids.
    element_count_[0] = static_cast<Index>(adjusted_length_x/x_edge_length_)+1;
    element_count_[1] = static_cast<Index>(adjusted_length_y/yz_edge_length_)+1;
    element_count_[2] = static_cast<Index>(adjusted_length_z/yz_edge_length_)+1;

}

BCCLatticeGenerator::GridPtr BCCLatticeGenerator::GenerateGridNodes() {

    // Build a 3D grid of nodes at the (irregular) intervals prescribed by the transformed BCC lattice and
    // add those nodes to the TetMesh object.

    // Allocate the three dimensional point reference.
    auto grid = GridPtr (new ThreeDimVector<TetNodeRef>(element_count_[0], element_count_[1], element_count_[2]));

    // Iterate the space, adding nodes to tetMesh and registering spatial pointers.
    for (Index i = 0; i<element_count_[0]; i++)
        for (Index j = 0; j<element_count_[1]; j++)
            for (Index k = 0; k<element_count_[2]; k++) {
                auto posX = lower_corner_[0]+(i * x_edge_length_);
                auto posY = lower_corner_[1]+(j * yz_edge_length_);
                auto posZ = lower_corner_[2]+(k * yz_edge_length_);
                grid->set(i, j, k, tet_mesh_->AddNode(posX, posY, posZ));
            }

    return grid;

}

void BCCLatticeGenerator::GenerateTetrahedraTemplate(Index originx, Index originy, Index originz) {

    // Stamp the tetrahedral tile into the TetMesh at the specified origin nodes. See the paper
    // for an illustration of the template.

    // Get pointers to the nodes of the template.
    TetNodeRef node_a = grid_->get(originx, originy, originz+1);
    TetNodeRef node_b = grid_->get(originx+1, originy, originz);
    TetNodeRef node_c = grid_->get(originx+2, originy, originz+1);
    TetNodeRef node_d = grid_->get(originx+3, originy, originz);
    TetNodeRef node_e = grid_->get(originx+1, originy+1, originz+1);
    TetNodeRef node_f = grid_->get(originx+2, originy+1, originz);
    TetNodeRef node_g = grid_->get(originx+3, originy+1, originz+1);
    TetNodeRef node_h = grid_->get(originx+4, originy+1, originz);

    // The 6 tets of the template use these nodes.
    tet_mesh_->AddTetrahedron(node_b, node_e, node_a, node_c);
    tet_mesh_->AddTetrahedron(node_f, node_e, node_b, node_c);
    tet_mesh_->AddTetrahedron(node_d, node_f, node_b, node_c);
    tet_mesh_->AddTetrahedron(node_d, node_g, node_f, node_c);
    tet_mesh_->AddTetrahedron(node_d, node_g, node_h, node_f);
    tet_mesh_->AddTetrahedron(node_g, node_f, node_e, node_c);

}

}; // namespace destroyer