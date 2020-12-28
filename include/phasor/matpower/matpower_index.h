
#ifndef PHASOR_INCLUDE_PARSER_MATPOWER_MATPOWER_INDEX_H__
#define PHASOR_INCLUDE_PARSER_MATPOWER_MATPOWER_INDEX_H__

#ifndef MATPOWER_VERSION
#define MATPOWER_VERSION 2
#endif

namespace phasor::matpower
{

  namespace v1
  {
    enum class BusIndex : size_t
    {
      BUS_I,    /// bus number
      BUS_TYPE, /// bus type (1 = PQ, 2 = PV, 3 = ref, 4 = isolated)
      PD,       /// real power demand (MW)
      QD,       /// reactive power demand (MW)
      GS,       /// shunt conductance (MW, demanded at V = 1.0 p.u.)
      BS,       /// shunt susceptance (MW, demanded at V = 1.0 p.u.)
      BUS_AREA, /// area number (positive integer)
      VM,       /// voltage magnitude (p.u.)
      VA,       /// voltage angle (degrees)
      BASE_KV,  /// base voltage (kV)
      ZONE,     /// loss zone (positive integer)
      VMAX,     /// maximum voltage magnitude (p.u.)
      VMIN      /// minimum voltage magnitude (p.u.)
    };

    enum class GenIndex : size_t
    {
      GEN_BUS,    /// bus number
      PG,         /// real power output (MW)
      QG,         /// reactive power output (MVAr)
      QMAX,       /// maximum reactive power output (MVAr)
      QMIN,       /// minimum reactive power output (MVAr)
      VG,         /// voltage magnitude setpoint (p.u.)
      MBASE,      /// total MVA base of machine, defaults tobaseMVA
      GEN_STATUS, /// machine status, >0 = machine in-service, ≤0 = machine out-of-service
      PMAX,       /// maximum real power output (MW)
      PMIN,       /// minimum real power output (MW)
    };

    enum class BranchIndex : size_t
    {
      F_BUS,     /// “from” bus number
      T_BUS,     /// “to” bus number
      BR_R,      /// resistance (p.u.)
      BR_X,      /// reactance (p.u.)
      BR_B,      /// total line charging susceptance (p.u.)
      RATE_A,    /// MVA rating A (long term rating), set to 0 for unlimited
      RATE_B,    /// MVA rating B (short term rating), set to 0 for unlimited
      RATE_C,    /// MVA rating C (emergency rating), set to 0 for unlimited
      TAP,       /// transformer off nominal turns ratio,
                 /// if non-zero (taps at “from”bus,  impedance at “to” bus,
                 /// i.e.  ifr=x=b= 0,tap=|Vf||Vt|;tap= 0 used to indicate
                 /// transmission line rather than transformer,i.e.
                 /// mathematically equivalent to transformer withtap= 1)
      SHIFT,     /// transformer phase shift angle (degrees), positive⇒delay
      BR_STATUS, /// initial branch status, 1 = in-service, 0 = out-of-service
      ANGMIN,    /// minimum angle difference,θf−θt(degrees)
      ANGMAX,    /// maximum angle difference,θf−θt(degrees
    };

    enum class GeneratorCostIndex : size_t
    {
      MODEL,    /// cost model, 1 = piecewise linear, 2 = polynomial
      STARTUP,  /// startup cost in US dollars
      SHUTDOWN, /// shutdown cost in US dollars
      NCOST,    /// number N=n+1 of data points defining ann-segment
                /// piecewise linear cost function, or of coefficients
                /// defining ann-th order polynomial cost function
      COST,     /// parameters defining total cost functionf(p) begin in
                /// this column,units offandpare$/hr and MW (or MVAr),
                /// respectively
                /// (MODEL= 1)⇒ p1,f1,p2,f2,...,pN,fN
                ///   wherep1< p2<···< pNand the costf(p) is defined by the
                ///     coordinates (p1,f1), (p2,f2), . . . , (pN,fN) of the
                ///     end/break-points of the piecewise linear cost
                /// (MODEL= 2)⇒ cn,...,c1,c0N coefficients of n-th order
                /// polynomial cost function, startingwith highest order,
                /// where cost isf(p) =cnpn+···+c1p+c0
    };
  }

  namespace v2
  {
    enum class BusIndex : size_t
    {
      BUS_I,    /// bus number
      BUS_TYPE, /// bus type (1 = PQ, 2 = PV, 3 = ref, 4 = isolated)
      PD,       /// real power demand (MW)
      QD,       /// reactive power demand (MW)
      GS,       /// shunt conductance (MW, demanded at V = 1.0 p.u.)
      BS,       /// shunt susceptance (MW, demanded at V = 1.0 p.u.)
      BUS_AREA, /// area number (positive integer)
      VM,       /// voltage magnitude (p.u.)
      VA,       /// voltage angle (degrees)
      BASE_KV,  /// base voltage (kV)
      ZONE,     /// loss zone (positive integer)
      VMAX,     /// maximum voltage magnitude (p.u.)
      VMIN      /// minimum voltage magnitude (p.u.)
    };

    enum class GenIndex : size_t
    {
      GEN_BUS,    /// bus number
      PG,         /// real power output (MW)
      QG,         /// reactive power output (MVAr)
      QMAX,       /// maximum reactive power output (MVAr)
      QMIN,       /// minimum reactive power output (MVAr)
      VG,         /// voltage magnitude setpoint (p.u.)
      MBASE,      /// total MVA base of machine, defaults tobaseMVA
      GEN_STATUS, /// machine status, >0 = machine in-service, ≤0 = machine out-of-service
      PMAX,       /// maximum real power output (MW)
      PMIN,       /// minimum real power output (MW)
      PC1,        /// lower real power output of PQ capability curve (MW)
      PC2,        /// upper real power output of PQ capability curve (MW)
      QC1MIN,     /// minimum reactive power output at PC1(MVAr)
      QC1MAX,     /// maximum reactive power output at PC1(MVAr)
      QC2MIN,     /// minimum reactive power output at PC2(MVAr)
      QC2MAX,     /// maximum reactive power output at PC2(MVAr)
      RAMP_AGC,   /// ramp rate for load following/AGC (MW/min)
      RAMP_10,    /// ramp rate for 10 minute reserves (MW)
      RAMP_30,    /// ramp rate for 30 minute reserves (MW)
      RAMP_Q,     /// ramp rate for reactive power (2 sec timescale) (MVAr/min)
      APF         /// area participation factor
    };

    enum class BranchIndex : size_t
    {
      F_BUS,     /// “from” bus number
      T_BUS,     /// “to” bus number
      BR_R,      /// resistance (p.u.)
      BR_X,      /// reactance (p.u.)
      BR_B,      /// total line charging susceptance (p.u.)
      RATE_A,    /// MVA rating A (long term rating), set to 0 for unlimited
      RATE_B,    /// MVA rating B (short term rating), set to 0 for unlimited
      RATE_C,    /// MVA rating C (emergency rating), set to 0 for unlimited
      TAP,       /// transformer off nominal turns ratio,
                 /// if non-zero (taps at “from”bus,  impedance at “to” bus,
                 /// i.e.  ifr=x=b= 0,tap=|Vf||Vt|;tap= 0 used to indicate
                 /// transmission line rather than transformer,i.e.
                 /// mathematically equivalent to transformer withtap= 1)
      SHIFT,     /// transformer phase shift angle (degrees), positive⇒delay
      BR_STATUS, /// initial branch status, 1 = in-service, 0 = out-of-service
      ANGMIN,    /// minimum angle difference,θf−θt(degrees)
      ANGMAX,    /// maximum angle difference,θf−θt(degrees
    };

    enum class GeneratorCostIndex : size_t
    {
      MODEL,    /// cost model, 1 = piecewise linear, 2 = polynomial
      STARTUP,  /// startup cost in US dollars
      SHUTDOWN, /// shutdown cost in US dollars
      NCOST,    /// number N=n+1 of data points defining ann-segment
                /// piecewise linear cost function, or of coefficients
                /// defining ann-th order polynomial cost function
      COST,     /// parameters defining total cost functionf(p) begin in
                /// this column,units offandpare$/hr and MW (or MVAr),
                /// respectively
                /// (MODEL= 1)⇒ p1,f1,p2,f2,...,pN,fN
                ///   wherep1< p2<···< pNand the costf(p) is defined by the
                ///     coordinates (p1,f1), (p2,f2), . . . , (pN,fN) of the
                ///     end/break-points of the piecewise linear cost
                /// (MODEL= 2)⇒ cn,...,c1,c0N coefficients of n-th order
                /// polynomial cost function, startingwith highest order,
                /// where cost isf(p) =cnpn+···+c1p+c0
    };
  }

#if MATPOWER_VERSION == 1
  using namespace v1;
#else
  using namespace v2;
#endif

} // namespace phasor::matpower

#endif // PHASOR_INCLUDE_PARSER_MATPOWER_MATPOWER_INDEX_H__