#ifndef PHASOR_INCLUDE_PARSER_MATPOWER_GENCOST_H__
#define PHASOR_INCLUDE_PARSER_MATPOWER_GENCOST_H__

#include <utility>

#include "matpower_dataset.h"
#include "matpower_index.h"

namespace phasor::matpower {
class GeneratorCostData : public MatpowerDataset {
 public:
  GeneratorCostData(Eigen::MatrixXd data) : MatpowerDataset(std::move(data)) {}

  Eigen::VectorXd operator[](const matpower::GeneratorCostIndex index) const {
    return getCol(static_cast<size_t>(index));
  }
};
} // namespace phasor

#endif // PHASOR_INCLUDE_PARSER_MATPOWER_GENCOST_H__