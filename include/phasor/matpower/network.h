
#ifndef PHASOR_PARSER_MATPOWER_NETWORK_H__
#define PHASOR_PARSER_MATPOWER_NETWORK_H__

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>

#include "branch.h"
#include "bus.h"
#include "gen.h"
#include "gencost.h"

namespace phasor::matpower {

struct Network {
  BranchData branch;
  BusData bus;
  GeneratorData gen;
  GeneratorCostData gencost;

  int version = 2;
  double baseMVA = 100.0;

  Eigen::MatrixXd makeISF() const {
    // todo: verify that buses are ordered numerically.

    // Set line status
    Eigen::VectorXd status = branch[BranchIndex::BR_STATUS];

    // Useful system properties
    size_t n_l_all = branch.getCol(0).size();
    size_t n_l = (status.array() > 0).count();
    size_t n_b = bus.getCol(0).size();

    // Series susceptance
    Eigen::VectorXd br_x(n_l);
    size_t index = 0;
    for (size_t i = 0; i < n_l_all; ++i)
      if (status[i] > 0)
        br_x[index++] = branch[BranchIndex::BR_X][i];

    // Set tap ratio
    Eigen::VectorXd tap(n_l);
    index = 0;
    for (size_t i = 0; i < n_l_all; ++i)
      if (status[i] > 0)
        tap[index++] = branch[BranchIndex::TAP][i];
    for (double &val : tap) if (val == 0) val = 1;

    // Scale susceptance by tap ratio
    Eigen::VectorXd b = 1 / br_x.array() / tap.array();

    // Construct connection matrix Cft = Cf - Ct for line and from - to buses
    Eigen::VectorXd f_bus(n_l);
    Eigen::VectorXd t_bus(n_l);
    index = 0;
    for (size_t i = 0; i < n_l_all; ++i) {
      if (status[i] > 0) {
        f_bus[index] = branch[BranchIndex::F_BUS][i] - 1; // remember to reindex to 0
        t_bus[index] = branch[BranchIndex::T_BUS][i] - 1;
        index++;
      }
    }

    // Create Bf matrix (flow matrix)
    typedef Eigen::Triplet<double> Triplet;
    std::vector<Triplet> tripletList;
    for (size_t i = 0; i < n_l; ++i) {
      tripletList.emplace_back(i, f_bus[i], b[i]);
      tripletList.emplace_back(i, t_bus[i], -b[i]);
    }
    Eigen::SparseMatrix<double> Bf(n_l, n_b);
    Bf.setFromTriplets(tripletList.begin(), tripletList.end());

    // Create Bbus matrix (graph incidence matrix)
    tripletList.clear();
    for (size_t i = 0; i < n_l; ++i) {
      tripletList.emplace_back(i, f_bus[i], 1);
      tripletList.emplace_back(i, t_bus[i], -1);
    }
    Eigen::SparseMatrix<double> Cft(n_l, n_b);
    Cft.setFromTriplets(tripletList.begin(), tripletList.end());

    Eigen::SparseMatrix<double> Bbus = Cft.transpose() * Bf;

    // Assume Pfinj = Pbusinj = 0
    // todo: reevaluate this assumption.

    // Construct ISF
    // Determine reference bus (0 if none specified)
    size_t reference_index = 0;
    for (size_t i = 0; i < n_b; ++i) if (bus[BusIndex::BUS_TYPE][i] == 3) reference_index = i;

    Eigen::SparseMatrix<double> Bf_ref(n_l, n_b - 1);
    Bf_ref.leftCols(reference_index) = Bf.leftCols(reference_index);
    Bf_ref.rightCols(n_b - reference_index - 1) = Bf.rightCols(n_b - reference_index - 1);

    Eigen::SparseMatrix<double> Bbus_ref(n_b, n_b - 1);
    Bbus_ref.leftCols(reference_index) = Bbus.leftCols(reference_index);
    Bbus_ref.rightCols(n_b - reference_index - 1) = Bbus.rightCols(n_b - reference_index - 1);\

    Eigen::SparseMatrix<double> BbusTBbus = Bbus_ref.transpose() * Bbus_ref;

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldl_solver;
    ldl_solver.compute(BbusTBbus);

    Eigen::SparseMatrix<double> Bbus_ref_transpose = Bbus_ref.transpose();

    return Bf_ref * ldl_solver.solve(Bbus_ref_transpose);
  }
};

} // namespace phasor

#endif // PHASOR_PARSER_MATPOWER_NETWORK_H__