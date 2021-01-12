#ifndef PHASOR_INCLUDE_PARSER_MATPOWER_BUS_H__
#define PHASOR_INCLUDE_PARSER_MATPOWER_BUS_H__

#include <utility>
#include <vector>

#include "matpower_dataset.h"
#include "matpower_index.h"

namespace phasor {
class BusData : public MatpowerDataset {
 public:
  BusData(Eigen::MatrixXd data) : MatpowerDataset(std::move(data)) {}

  Eigen::VectorXd operator[](const matpower::BusIndex index) const {
    return getCol(static_cast<size_t>(index));
  }
};
} // namespace phasor

#endif // PHASOR_INCLUDE_PARSER_MATPOWER_BUS_H__