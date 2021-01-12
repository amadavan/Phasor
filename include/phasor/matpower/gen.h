#ifndef PHASOR_INCLUDE_PARSER_MATPOWER_GEN_H__
#define PHASOR_INCLUDE_PARSER_MATPOWER_GEN_H__

#include <utility>

#include "matpower_dataset.h"
#include "matpower_index.h"

namespace phasor {
class GeneratorData : public MatpowerDataset {
 public:
  GeneratorData(Eigen::MatrixXd data) : MatpowerDataset(std::move(data)) {}

  Eigen::VectorXd operator[](const matpower::GenIndex index) const {
    return getCol(static_cast<size_t>(index));
  }
};
} // namespace phasor

#endif // PHASOR_INCLUDE_PARSER_MATPOWER_GEN_H__