#ifndef PHASOR_INCLUDE_PARSER_MATPOWER_GEN_H__
#define PHASOR_INCLUDE_PARSER_MATPOWER_GEN_H__

#include "matpower_dataset.h"
#include "matpower_index.h"

namespace phasor
{
  class GeneratorData : public MatpowerDataset
  {
  public:
    GeneratorData(Eigen::MatrixXd data) : MatpowerDataset(data) {}

    Eigen::VectorXd operator[](const matpower::GeneratorCostIndex index)
    {
      return getCol(static_cast<size_t>(index));
    }
  };
} // namespace phasor

#endif // PHASOR_INCLUDE_PARSER_MATPOWER_GEN_H__