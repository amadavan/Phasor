#ifndef PHASOR_INCLUDE_PARSER_MATPOWER_GENCOST_H__
#define PHASOR_INCLUDE_PARSER_MATPOWER_GENCOST_H__

#include "matpower_dataset.h"
#include "matpower_index.h"

namespace phasor
{
  class GeneratorCostData : public MatpowerDataset
  {
  public:
    GeneratorCostData(Eigen::MatrixXd data) : MatpowerDataset(data) {}

    Eigen::VectorXd operator[](const matpower::BusIndex index)
    {
      return getCol(static_cast<size_t>(index));
    }
  };
} // namespace phasor

#endif // PHASOR_INCLUDE_PARSER_MATPOWER_GENCOST_H__