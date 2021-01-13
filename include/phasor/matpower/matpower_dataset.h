//
// Created by Avinash Madavan on 12/18/20
//

#ifndef PHASOR_INCLUDE_PARSER_MATPOWER_MATPOWER_DATASET_H__
#define PHASOR_INCLUDE_PARSER_MATPOWER_MATPOWER_DATASET_H__

#include <Eigen/Core>
#include <utility>

namespace phasor::matpower {
class MatpowerDataset {
 public:
  /// Load matpower dataset from string
  MatpowerDataset(Eigen::MatrixXd data) : data_(std::move(data)) {};

  /**
   * Access variable as array.
   *
   * Given a bus/branch/generator index get the associated values of the
   * system. This method provides a direct access to the network's power
   * information, such as access the matpower bus information for all buses,
   */
  Eigen::VectorXd operator[](const size_t index) const {
    return getRow(index);
  }

  [[nodiscard]] Eigen::VectorXd getRow(const size_t index) const {
    return data_.row(index);
  }

  [[nodiscard]] Eigen::VectorXd getCol(const size_t index) const {
    return data_.col(index);
  }

 private:
  Eigen::MatrixXd data_;

  friend struct Network;
};
} // namespace phasor

#endif // PHASOR_INCLUDE_PARSER_MATPOWER_MATPOWER_DATASET_H__