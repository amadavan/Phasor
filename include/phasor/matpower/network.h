
#ifndef PHASOR_PARSER_MATPOWER_NETWORK_H__
#define PHASOR_PARSER_MATPOWER_NETWORK_H__

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#ifdef ENABLE_SUITESPARSE
#include <Eigen/CholmodSupport>
#endif
#include <utility>

#include "branch.h"
#include "bus.h"
#include "gen.h"
#include "gencost.h"

namespace phasor::matpower {

struct Network {
 public:
  BranchData branch;
  BusData bus;
  GeneratorData gen;
  GeneratorCostData gencost;

  int version = 2;
  double baseMVA = 100.0;

  Network(BranchData _branch,
          BusData _bus,
          GeneratorData _gen,
          GeneratorCostData _gencost,
          int _version,
          double _baseMVA)
      : branch(std::move(_branch)),
        bus(std::move(_bus)),
        gen(std::move(_gen)),
        gencost(std::move(_gencost)),
        version(_version),
        baseMVA(_baseMVA) {}

  [[nodiscard]] bool isInternalOrder() const {
    Eigen::VectorXd current_order = bus[BusIndex::BUS_I];
    for (size_t i = 0; i < current_order.size(); ++i) {
      size_t current_index = current_order[i];
      if (current_index != i + 1) return false;
    }
    return true;
  }

  [[nodiscard]] bool isExternalOrder() const {
    return bus_order_ == BusOrder::External;
  }

  void toInternalOrder() {
    if (isInternalOrder()) return;

    // Get number of buses
    size_t n_b = bus.getCol(0).size();
    size_t n_l_all = branch.getCol(0).size();
    size_t n_g = gen.getCol(0).size();

    // Store external order state
    external_order_ = bus[BusIndex::BUS_I];

    Eigen::VectorXd branch_fbus = branch[BranchIndex::F_BUS];
    Eigen::VectorXd branch_tbus = branch[BranchIndex::T_BUS];
    Eigen::VectorXd gen_index = gen[GenIndex::GEN_BUS];

    for (size_t i = 0; i < n_b; ++i) {
      int internal_index = i + 1;
      int external_index = external_order_[i];
      if (external_index != i) {
        // Update from/to bus indices for branches
        for (size_t j = 0; j < n_l_all; ++j) {
          if (branch_fbus[j] == external_index)
            branch.data_.coeffRef(j, static_cast<size_t>(BranchIndex::F_BUS)) = internal_index;
          if (branch_tbus[j] == external_index)
            branch.data_.coeffRef(j, static_cast<size_t>(BranchIndex::T_BUS)) = internal_index;
        }

        // Update generation bus index
        for (size_t j = 0; j < n_g; ++j)
          if (gen_index[j] == external_index)
            gen.data_.coeffRef(j, static_cast<size_t>(GenIndex::GEN_BUS)) = internal_index;

        // Update actual bus index
        bus.data_.coeffRef(i, static_cast<size_t>(BusIndex::BUS_I)) = internal_index;
      }
    }

    bus_order_ = BusOrder::Internal;
  }

  void toExternalOrder() {
    if (isExternalOrder()) return;

    // Get number of buses
    size_t n_b = bus.getCol(0).size();
    size_t n_l_all = branch.getCol(0).size();
    size_t n_g = gen.getCol(0).size();

    Eigen::VectorXd branch_fbus = branch[BranchIndex::F_BUS];
    Eigen::VectorXd branch_tbus = branch[BranchIndex::T_BUS];
    Eigen::VectorXd gen_index = gen[GenIndex::GEN_BUS];

    for (size_t i = 0; i < n_b; ++i) {
      int internal_index = i + 1;
      int external_index = external_order_[i];
      if (external_index != i) {
        // Update from/to bus indices for branches
        for (size_t j = 0; j < n_l_all; ++j) {
          if (branch_fbus[j] == internal_index)
            branch.data_.coeffRef(j, static_cast<size_t>(BranchIndex::F_BUS)) = external_index;
          if (branch_tbus[j] == internal_index)
            branch.data_.coeffRef(j, static_cast<size_t>(BranchIndex::T_BUS)) = external_index;
        }

        // Update generation bus index
        for (size_t j = 0; j < n_g; ++j)
          if (gen_index[j] == internal_index)
            gen.data_.coeffRef(j, static_cast<size_t>(GenIndex::GEN_BUS)) = external_index;

        // Update actual bus index
        bus.data_.coeffRef(i, static_cast<size_t>(BusIndex::BUS_I)) = external_index;
      }
    }

    bus_order_ = BusOrder::External;
  }

  /**
   * Create the Bbus and Bf matrices for DC opf.
   *
   * Emulates the makeBDC function from MATLAB. Crucially, this assumes that Pbusinj and Pfinj are negligible. This
   * constructs matrices Bbus and Bf, such that:
   * Pbus = Bbus * theta,
   * Pf = Bf * theta,
   * where Pbus and Pf correspond to nodal power injections and line flows, respectively.
   *
   * todo: Reevaluate the Pbusinj = Pfinj = 0 assumption.
   *
   * @return A tuple containing the factors for nodal power injections and line flows.
   */
  [[nodiscard]] std::tuple<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> makeBDC() const {
    if (!isInternalOrder())
      throw std::runtime_error("Buses need to be ordered consecutively. Please call toInternalOrder before makeISF.");

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

    return std::make_tuple(Bbus, Bf);
  }

  [[nodiscard]] Eigen::MatrixXd makeISF() const {
    if (!isInternalOrder())
      throw std::runtime_error("Buses need to be ordered consecutively. Please call toInternalOrder before makeISF.");

    // Set line status
    Eigen::VectorXd status = branch[BranchIndex::BR_STATUS];

    // Useful system properties
    size_t n_l_all = branch.getCol(0).size();
    size_t n_l = (status.array() > 0).count();
    size_t n_b = bus.getCol(0).size();

    auto[Bbus, Bf] = makeBDC();

    // Assume Pfinj = Pbusinj = 0
    // todo: reevaluate this assumption.

    // Construct ISF
    // Determine reference bus (0 if none specified)
    size_t reference_index = 0;
    for (size_t i = 0; i < n_b; ++i) if (bus[BusIndex::BUS_TYPE][i] == 3) reference_index = i;

    Eigen::SparseMatrix<double> Bf_ref(n_l, n_b - 1);
    Bf_ref.reserve(Bf.nonZeros());
    for (size_t i = 0; i < n_b - 1; ++i) {
      Bf_ref.startVec(i);
      if (i < reference_index)
        for (Eigen::SparseMatrix<double>::InnerIterator it(Bf, i); it; ++it)
          Bf_ref.insertBack(it.row(), i) = it.value();
      else
        for (Eigen::SparseMatrix<double>::InnerIterator it(Bf, i + 1); it; ++it)
          Bf_ref.insertBack(it.row(), i) = it.value();
    }
    Bf_ref.finalize();

    Eigen::SparseMatrix<double> Bbus_ref(n_b, n_b - 1);
    Bbus_ref.reserve(Bbus.nonZeros());
    for (size_t i = 0; i < n_b - 1; ++i) {
      Bbus_ref.startVec(i);
      if (i < reference_index)
        for (Eigen::SparseMatrix<double>::InnerIterator it(Bbus, i); it; ++it)
          Bbus_ref.insertBack(it.row(), i) = it.value();
      else
        for (Eigen::SparseMatrix<double>::InnerIterator it(Bbus, i + 1); it; ++it)
          Bbus_ref.insertBack(it.row(), i) = it.value();
    }
    Bbus_ref.finalize();

    Eigen::SparseMatrix<double> BbusTBbus = Bbus_ref.transpose() * Bbus_ref;
#ifdef ENABLE_SUITESPARSE
    Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> Bbus_solver;
#else
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> Bbus_solver;
#endif
    Bbus_solver.compute(BbusTBbus);

    if (Bbus_solver.info() != Eigen::Success) {
      std::cout << "phasor::Network::makeBDC failed. Unable to compute the ISF matrix.";
    }

    Eigen::SparseMatrix<double> Bbus_ref_transpose = Bbus_ref.transpose();

    return Bf_ref * Bbus_solver.solve(Bbus_ref_transpose);
  }

 private:
  enum class BusOrder {
    External,
    Internal
  };

  BusOrder bus_order_ = BusOrder::External;
  Eigen::VectorXd external_order_;
};

} // namespace phasor

#endif // PHASOR_PARSER_MATPOWER_NETWORK_H__