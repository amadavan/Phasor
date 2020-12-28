
#ifndef PHASOR_PARSER_MATPOWER_NETWORK_H__
#define PHASOR_PARSER_MATPOWER_NETWORK_H__

#include <Eigen/Core>

#include "branch.h"
#include "bus.h"
#include "gen.h"
#include "gencost.h"

namespace phasor::matpower
{

  struct Network
  {
    BranchData branch;
    BusData bus;
    GeneratorData gen;
    GeneratorCostData gencost;
    
    int version = 2;
    double baseMVA = 100.0;
  };

} // namespace phasor

#endif // PHASOR_PARSER_MATPOWER_NETWORK_H__