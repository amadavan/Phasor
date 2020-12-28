#ifndef PHASOR_INCLUDE_PARSER_MATPOWER_BRANCH_H__
#define PHASOR_INCLUDE_PARSER_MATPOWER_BRANCH_H__

#include "matpower_dataset.h"
#include "matpower_index.h"

namespace phasor
{
  class BranchData : public MatpowerDataset
  {
  public:
    BranchData(Eigen::MatrixXd data) : MatpowerDataset(data) {}

    Eigen::VectorXd operator[](const matpower::BranchIndex index)
    {
      return getCol(static_cast<size_t>(index));
    }
  };
} // namespace phasor

#endif // PHASOR_INCLUDE_PARSER_MATPOWER_BRANCH_H__