// compile command: 
// g++ -ggdb3 -I/usr/include/log4cxx `pkg-config --cflags roboptim-core` src/generatedFiles/@FUNCTION_NAME@.cc `pkg-config --libs roboptim-core` -o bin/@FUNCTION_NAME@
#include <iostream>
#include <boost/mpl/vector.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>
#include <roboptim/core/differentiable-function.hh>
#include <roboptim/core/linear-function.hh>
#include <roboptim/core/function/constant.hh>
#include <roboptim/core/solver.hh>
#include <roboptim/core/solver-factory.hh>
#include <roboptim/core/result.hh>

typedef roboptim::Solver <
  roboptim::GenericDifferentiableFunction< roboptim::EigenMatrixDense>,
  boost::mpl::vector<
    roboptim::GenericLinearFunction<roboptim::EigenMatrixDense>,
    roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense>
    >
  >
  solver_t;

static const double pi = boost::math::constants::pi<double>();


template <typename T>
class CostFunction : public roboptim::GenericLinearFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericLinearFunction<T>);
  
  explicit CostFunction (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
CostFunction<T>::CostFunction (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericLinearFunction<T>
    (96, 1, "CostFunction_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
CostFunction<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.0;
}

template <typename T>
void
CostFunction<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_1 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_1 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_1<T>::LiftConstraint_1 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_1_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_1<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = cos(q_07) - 1.0*w_01_01;
}

template <typename T>
void
LiftConstraint_1<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = -1.0*sin(q_07); 
			 grad[7] = -1.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_2 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_2 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_2<T>::LiftConstraint_2 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_2_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_2<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = sin(q_07) - 1.0*w_01_02;
}

template <typename T>
void
LiftConstraint_2<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = cos(q_07); 
			 grad[7] = 0.0; 
			 grad[8] = -1.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_3 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_3 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_3<T>::LiftConstraint_3 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_3_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_3<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = cos(q_06) - 1.0*w_01_03;
}

template <typename T>
void
LiftConstraint_3<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = -1.0*sin(q_06); 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = -1.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_4 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_4 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_4<T>::LiftConstraint_4 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_4_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_4<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = sin(q_02) - 1.0*w_01_04;
}

template <typename T>
void
LiftConstraint_4<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = cos(q_02); 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = -1.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_5 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_5 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_5<T>::LiftConstraint_5 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_5_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_5<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = cos(q_01) - 1.0*w_01_05;
}

template <typename T>
void
LiftConstraint_5<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = -1.0*sin(q_01); 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = -1.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_6 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_6 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_6<T>::LiftConstraint_6 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_6_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_6<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = sin(q_06) - 1.0*w_01_06;
}

template <typename T>
void
LiftConstraint_6<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = cos(q_06); 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = -1.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_7 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_7 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_7<T>::LiftConstraint_7 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_7_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_7<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = cos(q_05) - 1.0*w_01_07;
}

template <typename T>
void
LiftConstraint_7<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = -1.0*sin(q_05); 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = -1.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_8 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_8 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_8<T>::LiftConstraint_8 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_8_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_8<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = sin(q_01) - 1.0*w_01_08;
}

template <typename T>
void
LiftConstraint_8<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = cos(q_01); 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = -1.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_9 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_9 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_9<T>::LiftConstraint_9 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_9_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_9<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = sin(q_05) - 1.0*w_01_09;
}

template <typename T>
void
LiftConstraint_9<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = cos(q_05); 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = -1.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_10 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_10 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_10<T>::LiftConstraint_10 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_10_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_10<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = cos(q_02) - 1.0*w_01_10;
}

template <typename T>
void
LiftConstraint_10<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = -1.0*sin(q_02); 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = -1.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_11 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_11 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_11<T>::LiftConstraint_11 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_11_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_11<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = cos(q_04) - 1.0*w_01_11;
}

template <typename T>
void
LiftConstraint_11<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = -1.0*sin(q_04); 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = -1.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_12 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_12 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_12<T>::LiftConstraint_12 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_12_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_12<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = sin(q_03 - 1.5707963267948966192313216916398) - 1.0*w_01_12;
}

template <typename T>
void
LiftConstraint_12<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = cos(q_03 - 1.5707963267948966192313216916398); 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = -1.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_13 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_13 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_13<T>::LiftConstraint_13 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_13_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_13<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = sin(q_04) - 1.0*w_01_13;
}

template <typename T>
void
LiftConstraint_13<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = cos(q_04); 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = -1.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_14 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_14 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_14<T>::LiftConstraint_14 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_14_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_14<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = cos(q_03 - 1.5707963267948966192313216916398) - 1.0*w_01_14;
}

template <typename T>
void
LiftConstraint_14<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = -1.0*sin(q_03 - 1.5707963267948966192313216916398); 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = -1.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_15 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_15 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_15<T>::LiftConstraint_15 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_15_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_15<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_05*w_01_10 - 1.0*w_02_01;
}

template <typename T>
void
LiftConstraint_15<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = w_01_10; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = w_01_05; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = -1.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_16 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_16 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_16<T>::LiftConstraint_16 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_16_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_16<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_03*(w_01_04 + 3.7493994566546433353612919702563e-33) - 1.0*w_02_02;
}

template <typename T>
void
LiftConstraint_16<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = w_01_04 + 3.7493994566546433353612919702563e-33; 
			 grad[10] = w_01_03; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = -1.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_17 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_17 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_17<T>::LiftConstraint_17 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_17_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_17<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_06*(w_01_04 + 3.7493994566546433353612919702563e-33) - 1.0*w_02_03;
}

template <typename T>
void
LiftConstraint_17<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = w_01_06; 
			 grad[11] = 0.0; 
			 grad[12] = w_01_04 + 3.7493994566546433353612919702563e-33; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = -1.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_18 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_18 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_18<T>::LiftConstraint_18 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_18_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_18<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_04*w_01_05 - 1.0*w_02_04;
}

template <typename T>
void
LiftConstraint_18<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.00000000000000006123233995736766035868820147292*w_01_05; 
			 grad[11] = 0.00000000000000006123233995736766035868820147292*w_01_04; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = -1.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_19 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_19 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_19<T>::LiftConstraint_19 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_19_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_19<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_08*w_01_10 - 1.0*w_02_05;
}

template <typename T>
void
LiftConstraint_19<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = w_01_10; 
			 grad[15] = 0.0; 
			 grad[16] = w_01_08; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = -1.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_20 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_20 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_20<T>::LiftConstraint_20 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_20_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_20<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_04*w_01_08 - 1.0*w_02_06;
}

template <typename T>
void
LiftConstraint_20<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.00000000000000006123233995736766035868820147292*w_01_08; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.00000000000000006123233995736766035868820147292*w_01_04; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = -1.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_21 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_21 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_21<T>::LiftConstraint_21 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_21_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_21<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_10*w_01_12 - 1.0*w_02_07;
}

template <typename T>
void
LiftConstraint_21<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = w_01_12; 
			 grad[17] = 0.0; 
			 grad[18] = w_01_10; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = -1.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_22 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_22 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_22<T>::LiftConstraint_22 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_22_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_22<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_04*w_01_14 - 1.0*w_02_08;
}

template <typename T>
void
LiftConstraint_22<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.00000000000000006123233995736766035868820147292*w_01_14; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.00000000000000006123233995736766035868820147292*w_01_04; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = -1.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_23 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_23 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_23<T>::LiftConstraint_23 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_23_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_23<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_05*w_01_14 - 1.0*w_02_09;
}

template <typename T>
void
LiftConstraint_23<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = w_01_14; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = w_01_05; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = -1.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_24 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_24 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_24<T>::LiftConstraint_24 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_24_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_24<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_08*w_01_14 - 1.0*w_02_10;
}

template <typename T>
void
LiftConstraint_24<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = w_01_14; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = w_01_08; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = -1.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_25 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_25 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_25<T>::LiftConstraint_25 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_25_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_25<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_05*w_01_10 - 1.0*w_02_11;
}

template <typename T>
void
LiftConstraint_25<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.00000000000000006123233995736766035868820147292*w_01_10; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.00000000000000006123233995736766035868820147292*w_01_05; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = -1.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_26 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_26 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_26<T>::LiftConstraint_26 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_26_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_26<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_08*w_01_10 - 1.0*w_02_12;
}

template <typename T>
void
LiftConstraint_26<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.00000000000000006123233995736766035868820147292*w_01_10; 
			 grad[15] = 0.0; 
			 grad[16] = 0.00000000000000006123233995736766035868820147292*w_01_08; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = -1.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_27 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_27 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_27<T>::LiftConstraint_27 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_27_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_27<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_04*w_01_05 - 1.0*w_02_13;
}

template <typename T>
void
LiftConstraint_27<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = w_01_05; 
			 grad[11] = w_01_04; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = -1.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_28 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_28 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_28<T>::LiftConstraint_28 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_28_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_28<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_04*w_01_08 - 1.0*w_02_14;
}

template <typename T>
void
LiftConstraint_28<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = w_01_08; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = w_01_04; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = -1.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_29 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_29 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_29<T>::LiftConstraint_29 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_29_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_29<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_10*w_01_14 - 1.0*w_02_15;
}

template <typename T>
void
LiftConstraint_29<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = w_01_14; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = w_01_10; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = -1.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_30 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_30 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_30<T>::LiftConstraint_30 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_30_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_30<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_04*w_01_12 - 1.0*w_02_16;
}

template <typename T>
void
LiftConstraint_30<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.00000000000000006123233995736766035868820147292*w_01_12; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.00000000000000006123233995736766035868820147292*w_01_04; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = -1.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_31 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_31 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_31<T>::LiftConstraint_31 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_31_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_31<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_05*w_01_12 - 1.0*w_02_17;
}

template <typename T>
void
LiftConstraint_31<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = w_01_12; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = w_01_05; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = -1.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_32 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_32 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_32<T>::LiftConstraint_32 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_32_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_32<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_08*w_01_12 - 1.0*w_02_18;
}

template <typename T>
void
LiftConstraint_32<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = w_01_12; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = w_01_08; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = -1.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_33 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_33 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_33<T>::LiftConstraint_33 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_33_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_33<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_03_01 - 1.0*w_01_03*(w_02_04 - 0.00000000000000006123233995736766035868820147292*w_01_05 + w_02_05);
}

template <typename T>
void
LiftConstraint_33<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.00000000000000006123233995736766035868820147292*w_01_05 - 1.0*w_02_04 - 1.0*w_02_05; 
			 grad[10] = 0.0; 
			 grad[11] = 0.00000000000000006123233995736766035868820147292*w_01_03; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = -1.0*w_01_03; 
			 grad[25] = -1.0*w_01_03; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = -1.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_34 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_34 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_34<T>::LiftConstraint_34 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_34_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_34<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_03_02 - 1.0*w_01_06*(w_02_04 - 0.00000000000000006123233995736766035868820147292*w_01_05 + w_02_05);
}

template <typename T>
void
LiftConstraint_34<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.00000000000000006123233995736766035868820147292*w_01_06; 
			 grad[12] = 0.00000000000000006123233995736766035868820147292*w_01_05 - 1.0*w_02_04 - 1.0*w_02_05; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = -1.0*w_01_06; 
			 grad[25] = -1.0*w_01_06; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = -1.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_35 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_35 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_35<T>::LiftConstraint_35 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_35_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_35<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_03*(0.00000000000000006123233995736766035868820147292*w_01_08 + w_02_01 - 1.0*w_02_06) - 1.0*w_03_03;
}

template <typename T>
void
LiftConstraint_35<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.00000000000000006123233995736766035868820147292*w_01_08 + w_02_01 - 1.0*w_02_06; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.00000000000000006123233995736766035868820147292*w_01_03; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = w_01_03; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = -1.0*w_01_03; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = -1.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_36 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_36 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_36<T>::LiftConstraint_36 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_36_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_36<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_06*(0.00000000000000006123233995736766035868820147292*w_01_08 + w_02_01 - 1.0*w_02_06) - 1.0*w_03_04;
}

template <typename T>
void
LiftConstraint_36<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.00000000000000006123233995736766035868820147292*w_01_08 + w_02_01 - 1.0*w_02_06; 
			 grad[13] = 0.0; 
			 grad[14] = 0.00000000000000006123233995736766035868820147292*w_01_06; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = w_01_06; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = -1.0*w_01_06; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = -1.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_37 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_37 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_37<T>::LiftConstraint_37 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_37_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_37<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_11*(w_02_07 - 0.00000000000000006123233995736766035868820147292*w_01_14 + w_02_08) - 1.0*w_03_05;
}

template <typename T>
void
LiftConstraint_37<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = w_02_07 - 0.00000000000000006123233995736766035868820147292*w_01_14 + w_02_08; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = -0.00000000000000006123233995736766035868820147292*w_01_11; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = w_01_11; 
			 grad[28] = w_01_11; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = -1.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_38 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_38 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_38<T>::LiftConstraint_38 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_38_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_38<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_12*(w_02_11 - 1.0*w_02_14) - 1.0*w_03_06;
}

template <typename T>
void
LiftConstraint_38<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = w_02_11 - 1.0*w_02_14; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = w_01_12; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = -1.0*w_01_12; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = -1.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_39 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_39 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_39<T>::LiftConstraint_39 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_39_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_39<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_03_07 - 0.00000000000000006123233995736766035868820147292*w_01_14*(w_02_04 + w_02_05);
}

template <typename T>
void
LiftConstraint_39<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = - 0.00000000000000006123233995736766035868820147292*w_02_04 - 0.00000000000000006123233995736766035868820147292*w_02_05; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = -0.00000000000000006123233995736766035868820147292*w_01_14; 
			 grad[25] = -0.00000000000000006123233995736766035868820147292*w_01_14; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = -1.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_40 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_40 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_40<T>::LiftConstraint_40 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_40_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_40<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_12*(w_02_12 + w_02_13) - 1.0*w_03_08;
}

template <typename T>
void
LiftConstraint_40<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = w_02_12 + w_02_13; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = w_01_12; 
			 grad[33] = w_01_12; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = -1.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_41 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_41 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_41<T>::LiftConstraint_41 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_41_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_41<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_14*(w_02_01 - 1.0*w_02_06) - 1.0*w_03_09;
}

template <typename T>
void
LiftConstraint_41<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.00000000000000006123233995736766035868820147292*w_02_01 - 0.00000000000000006123233995736766035868820147292*w_02_06; 
			 grad[21] = 0.00000000000000006123233995736766035868820147292*w_01_14; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = -0.00000000000000006123233995736766035868820147292*w_01_14; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = -1.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_42 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_42 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_42<T>::LiftConstraint_42 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_42_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_42<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_03_10 - 1.0*w_01_13*(0.00000000000000006123233995736766035868820147292*w_01_12 + w_02_15 - 1.0*w_02_16);
}

template <typename T>
void
LiftConstraint_42<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = -0.00000000000000006123233995736766035868820147292*w_01_13; 
			 grad[19] = w_02_16 - 1.0*w_02_15 - 0.00000000000000006123233995736766035868820147292*w_01_12; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = -1.0*w_01_13; 
			 grad[36] = w_01_13; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = -1.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_43 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_43 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_43<T>::LiftConstraint_43 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_43_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_43<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_03_11 - 1.0*w_01_11*(0.00000000000000006123233995736766035868820147292*w_01_12 + w_02_15 - 1.0*w_02_16);
}

template <typename T>
void
LiftConstraint_43<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = w_02_16 - 1.0*w_02_15 - 0.00000000000000006123233995736766035868820147292*w_01_12; 
			 grad[18] = -0.00000000000000006123233995736766035868820147292*w_01_11; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = -1.0*w_01_11; 
			 grad[36] = w_01_11; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = -1.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_44 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_44 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_44<T>::LiftConstraint_44 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_44_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_44<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_13*(w_02_07 - 0.00000000000000006123233995736766035868820147292*w_01_14 + w_02_08) - 1.0*w_03_12;
}

template <typename T>
void
LiftConstraint_44<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = w_02_07 - 0.00000000000000006123233995736766035868820147292*w_01_14 + w_02_08; 
			 grad[20] = -0.00000000000000006123233995736766035868820147292*w_01_13; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = w_01_13; 
			 grad[28] = w_01_13; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = -1.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_45 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_45 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_45<T>::LiftConstraint_45 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_45_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_45<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_14*(w_02_11 - 1.0*w_02_14) - 1.0*w_03_13;
}

template <typename T>
void
LiftConstraint_45<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = w_02_11 - 1.0*w_02_14; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = w_01_14; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = -1.0*w_01_14; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = -1.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_46 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_46 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_46<T>::LiftConstraint_46 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_46_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_46<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_14*(w_02_12 + w_02_13) - 1.0*w_03_14;
}

template <typename T>
void
LiftConstraint_46<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = w_02_12 + w_02_13; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = w_01_14; 
			 grad[33] = w_01_14; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = -1.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_47 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_47 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_47<T>::LiftConstraint_47 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_47_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_47<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_03_15 - 0.00000000000000006123233995736766035868820147292*w_01_12*(w_02_04 + w_02_05);
}

template <typename T>
void
LiftConstraint_47<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = - 0.00000000000000006123233995736766035868820147292*w_02_04 - 0.00000000000000006123233995736766035868820147292*w_02_05; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = -0.00000000000000006123233995736766035868820147292*w_01_12; 
			 grad[25] = -0.00000000000000006123233995736766035868820147292*w_01_12; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = -1.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_48 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_48 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_48<T>::LiftConstraint_48 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_48_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_48<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_12*(w_02_01 - 1.0*w_02_06) - 1.0*w_03_16;
}

template <typename T>
void
LiftConstraint_48<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.00000000000000006123233995736766035868820147292*w_02_01 - 0.00000000000000006123233995736766035868820147292*w_02_06; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.00000000000000006123233995736766035868820147292*w_01_12; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = -0.00000000000000006123233995736766035868820147292*w_01_12; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = -1.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_49 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_49 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_49<T>::LiftConstraint_49 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_49_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_49<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_07*(w_03_05 - 1.0*w_03_10) - 1.0*w_04_01;
}

template <typename T>
void
LiftConstraint_49<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_03_05 - 1.0*w_03_10; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = w_01_07; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = -1.0*w_01_07; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = -1.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_50 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_50 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_50<T>::LiftConstraint_50 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_50_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_50<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_09*(w_03_11 + w_03_12) - 1.0*w_04_02;
}

template <typename T>
void
LiftConstraint_50<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = w_03_11 + w_03_12; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = w_01_09; 
			 grad[50] = w_01_09; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = -1.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_51 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_51 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_51<T>::LiftConstraint_51 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_51_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_51<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_07*(w_03_11 + w_03_12) - 1.0*w_04_03;
}

template <typename T>
void
LiftConstraint_51<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_03_11 + w_03_12; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = w_01_07; 
			 grad[50] = w_01_07; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = -1.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_52 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_52 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_52<T>::LiftConstraint_52 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_52_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_52<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_09*(w_03_05 - 1.0*w_03_10) - 1.0*w_04_04;
}

template <typename T>
void
LiftConstraint_52<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = w_03_05 - 1.0*w_03_10; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = w_01_09; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = -1.0*w_01_09; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = -1.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_53 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_53 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_53<T>::LiftConstraint_53 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_53_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_53<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_04_05 - 1.0*w_01_11*(w_02_09 + w_03_06 - 1.0*w_03_07);
}

template <typename T>
void
LiftConstraint_53<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = w_03_07 - 1.0*w_03_06 - 1.0*w_02_09; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = -1.0*w_01_11; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = -1.0*w_01_11; 
			 grad[45] = w_01_11; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = -1.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_54 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_54 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_54<T>::LiftConstraint_54 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_54_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_54<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_04_06 - 1.0*w_01_11*(w_02_10 + w_03_08 - 1.0*w_03_09);
}

template <typename T>
void
LiftConstraint_54<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = w_03_09 - 1.0*w_03_08 - 1.0*w_02_10; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = -1.0*w_01_11; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = -1.0*w_01_11; 
			 grad[47] = w_01_11; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = -1.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_55 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_55 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_55<T>::LiftConstraint_55 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_55_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_55<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_13*(w_03_13 - 1.0*w_02_17 + w_03_15) - 1.0*w_04_07;
}

template <typename T>
void
LiftConstraint_55<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = w_03_13 - 1.0*w_02_17 + w_03_15; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = -1.0*w_01_13; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = w_01_13; 
			 grad[52] = 0.0; 
			 grad[53] = w_01_13; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = -1.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_56 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_56 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_56<T>::LiftConstraint_56 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_56_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_56<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_11*(w_03_13 - 1.0*w_02_17 + w_03_15) - 1.0*w_04_08;
}

template <typename T>
void
LiftConstraint_56<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = w_03_13 - 1.0*w_02_17 + w_03_15; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = -1.0*w_01_11; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = w_01_11; 
			 grad[52] = 0.0; 
			 grad[53] = w_01_11; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = -1.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_57 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_57 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_57<T>::LiftConstraint_57 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_57_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_57<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_04_09 - 1.0*w_01_13*(w_02_09 + w_03_06 - 1.0*w_03_07);
}

template <typename T>
void
LiftConstraint_57<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = w_03_07 - 1.0*w_03_06 - 1.0*w_02_09; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = -1.0*w_01_13; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = -1.0*w_01_13; 
			 grad[45] = w_01_13; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = -1.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_58 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_58 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_58<T>::LiftConstraint_58 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_58_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_58<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_13*(w_03_14 - 1.0*w_02_18 + w_03_16) - 1.0*w_04_10;
}

template <typename T>
void
LiftConstraint_58<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = w_03_14 - 1.0*w_02_18 + w_03_16; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = -1.0*w_01_13; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = w_01_13; 
			 grad[53] = 0.0; 
			 grad[54] = w_01_13; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = -1.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_59 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_59 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_59<T>::LiftConstraint_59 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_59_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_59<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_11*(w_03_14 - 1.0*w_02_18 + w_03_16) - 1.0*w_04_11;
}

template <typename T>
void
LiftConstraint_59<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = w_03_14 - 1.0*w_02_18 + w_03_16; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = -1.0*w_01_11; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = w_01_11; 
			 grad[53] = 0.0; 
			 grad[54] = w_01_11; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = -1.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_60 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_60 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_60<T>::LiftConstraint_60 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_60_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_60<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_04_12 - 1.0*w_01_13*(w_02_10 + w_03_08 - 1.0*w_03_09);
}

template <typename T>
void
LiftConstraint_60<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = w_03_09 - 1.0*w_03_08 - 1.0*w_02_10; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = -1.0*w_01_13; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = -1.0*w_01_13; 
			 grad[47] = w_01_13; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = -1.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_61 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_61 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_61<T>::LiftConstraint_61 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_61_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_61<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_02*(0.00000000000000006123233995736766035868820147292*w_01_04 + w_04_01 - 1.0*w_04_02 + 2.2958450216584678727157691077339e-49) - 1.0*w_05_01;
}

template <typename T>
void
LiftConstraint_61<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.00000000000000006123233995736766035868820147292*w_01_04 + w_04_01 - 1.0*w_04_02 + 2.2958450216584678727157691077339e-49; 
			 grad[9] = 0.0; 
			 grad[10] = 0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = w_01_02; 
			 grad[56] = -1.0*w_01_02; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = -1.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_62 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_62 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_62<T>::LiftConstraint_62 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_62_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_62<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_03*(w_04_03 + w_04_04) - 1.0*w_05_02;
}

template <typename T>
void
LiftConstraint_62<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = w_04_03 + w_04_04; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = w_01_03; 
			 grad[58] = w_01_03; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = -1.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_63 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_63 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_63<T>::LiftConstraint_63 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_63_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_63<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_07*(w_04_05 - 1.0*w_04_07) - 1.0*w_05_03;
}

template <typename T>
void
LiftConstraint_63<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_04_05 - 1.0*w_04_07; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = w_01_07; 
			 grad[60] = 0.0; 
			 grad[61] = -1.0*w_01_07; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = -1.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_64 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_64 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_64<T>::LiftConstraint_64 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_64_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_64<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_07*(w_04_06 - 1.0*w_04_10) - 1.0*w_05_04;
}

template <typename T>
void
LiftConstraint_64<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_04_06 - 1.0*w_04_10; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = w_01_07; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = -1.0*w_01_07; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = -1.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_65 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_65 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_65<T>::LiftConstraint_65 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_65_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_65<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_06*(w_04_03 + w_04_04) - 1.0*w_05_05;
}

template <typename T>
void
LiftConstraint_65<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = w_04_03 + w_04_04; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = w_01_06; 
			 grad[58] = w_01_06; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = -1.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_66 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_66 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_66<T>::LiftConstraint_66 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_66_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_66<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_03*(w_04_01 - 1.0*w_04_02) - 1.0*w_05_06;
}

template <typename T>
void
LiftConstraint_66<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.00000000000000006123233995736766035868820147292*w_04_01 - 0.00000000000000006123233995736766035868820147292*w_04_02; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.00000000000000006123233995736766035868820147292*w_01_03; 
			 grad[56] = -0.00000000000000006123233995736766035868820147292*w_01_03; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = -1.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_67 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_67 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_67<T>::LiftConstraint_67 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_67_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_67<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_06*(w_04_01 - 1.0*w_04_02) - 1.0*w_05_07;
}

template <typename T>
void
LiftConstraint_67<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.00000000000000006123233995736766035868820147292*w_04_01 - 0.00000000000000006123233995736766035868820147292*w_04_02; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.00000000000000006123233995736766035868820147292*w_01_06; 
			 grad[56] = -0.00000000000000006123233995736766035868820147292*w_01_06; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = -1.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_68 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_68 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_68<T>::LiftConstraint_68 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_68_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_68<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_09*(w_04_08 + w_04_09) - 1.0*w_05_08;
}

template <typename T>
void
LiftConstraint_68<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = w_04_08 + w_04_09; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = w_01_09; 
			 grad[63] = w_01_09; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = -1.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_69 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_69 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_69<T>::LiftConstraint_69 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_69_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_69<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_07*(w_04_08 + w_04_09) - 1.0*w_05_09;
}

template <typename T>
void
LiftConstraint_69<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_04_08 + w_04_09; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = w_01_07; 
			 grad[63] = w_01_07; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = -1.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_70 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_70 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_70<T>::LiftConstraint_70 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_70_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_70<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_09*(w_04_05 - 1.0*w_04_07) - 1.0*w_05_10;
}

template <typename T>
void
LiftConstraint_70<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = w_04_05 - 1.0*w_04_07; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = w_01_09; 
			 grad[60] = 0.0; 
			 grad[61] = -1.0*w_01_09; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = -1.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_71 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_71 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_71<T>::LiftConstraint_71 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_71_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_71<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_09*(w_04_11 + w_04_12) - 1.0*w_05_11;
}

template <typename T>
void
LiftConstraint_71<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = w_04_11 + w_04_12; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = w_01_09; 
			 grad[66] = w_01_09; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = -1.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_72 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_72 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_72<T>::LiftConstraint_72 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_72_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_72<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_07*(w_04_11 + w_04_12) - 1.0*w_05_12;
}

template <typename T>
void
LiftConstraint_72<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_04_11 + w_04_12; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = w_01_07; 
			 grad[66] = w_01_07; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = -1.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_73 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_73 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_73<T>::LiftConstraint_73 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_73_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_73<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_09*(w_04_06 - 1.0*w_04_10) - 1.0*w_05_13;
}

template <typename T>
void
LiftConstraint_73<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = w_04_06 - 1.0*w_04_10; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = w_01_09; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = -1.0*w_01_09; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = -1.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_74 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_74 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_74<T>::LiftConstraint_74 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_74_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_74<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_06_01 - 1.0*w_01_02*(0.00000000000000006123233995736766035868820147292*w_02_04 - 3.7493994566546440195890577538583e-33*w_01_05 + 0.00000000000000006123233995736766035868820147292*w_02_05 - 1.0*w_05_03 + w_05_08);
}

template <typename T>
void
LiftConstraint_74<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 3.7493994566546440195890577538583e-33*w_01_05 - 0.00000000000000006123233995736766035868820147292*w_02_04 - 0.00000000000000006123233995736766035868820147292*w_02_05 + w_05_03 - 1.0*w_05_08; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 3.7493994566546440195890577538583e-33*w_01_02; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = -0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[25] = -0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = w_01_02; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = -1.0*w_01_02; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = -1.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_75 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_75 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_75<T>::LiftConstraint_75 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_75_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_75<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_03*(w_05_09 + w_05_10) - 1.0*w_06_02;
}

template <typename T>
void
LiftConstraint_75<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = w_05_09 + w_05_10; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = w_01_03; 
			 grad[76] = w_01_03; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = -1.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_76 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_76 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_76<T>::LiftConstraint_76 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_76_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_76<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_02*(3.7493994566546440195890577538583e-33*w_01_08 + 0.00000000000000006123233995736766035868820147292*w_02_01 - 0.00000000000000006123233995736766035868820147292*w_02_06 + w_05_04 - 1.0*w_05_11) - 1.0*w_06_03;
}

template <typename T>
void
LiftConstraint_76<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 3.7493994566546440195890577538583e-33*w_01_08 + 0.00000000000000006123233995736766035868820147292*w_02_01 - 0.00000000000000006123233995736766035868820147292*w_02_06 + w_05_04 - 1.0*w_05_11; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 3.7493994566546440195890577538583e-33*w_01_02; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = -0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = w_01_02; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = -1.0*w_01_02; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = -1.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_77 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_77 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_77<T>::LiftConstraint_77 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_77_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_77<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_03*(w_05_12 + w_05_13) - 1.0*w_06_04;
}

template <typename T>
void
LiftConstraint_77<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = w_05_12 + w_05_13; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = w_01_03; 
			 grad[79] = w_01_03; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = -1.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_78 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_78 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_78<T>::LiftConstraint_78 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_78_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_78<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_02*(w_05_02 - 1.0*w_02_03 + w_05_07) - 1.0*w_06_05;
}

template <typename T>
void
LiftConstraint_78<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.00000000000000006123233995736766035868820147292*w_05_02 - 0.00000000000000006123233995736766035868820147292*w_02_03 + 0.00000000000000006123233995736766035868820147292*w_05_07; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = -0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = -1.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_79 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_79 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_79<T>::LiftConstraint_79 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_79_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_79<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_06_06 - 1.0*w_01_01*(w_02_02 + w_05_05 - 1.0*w_05_06);
}

template <typename T>
void
LiftConstraint_79<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = w_05_06 - 1.0*w_05_05 - 1.0*w_02_02; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = -1.0*w_01_01; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = -1.0*w_01_01; 
			 grad[72] = w_01_01; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = -1.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_80 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_80 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_80<T>::LiftConstraint_80 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_80_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_80<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_06*(w_05_09 + w_05_10) - 1.0*w_06_07;
}

template <typename T>
void
LiftConstraint_80<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = w_05_09 + w_05_10; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = w_01_06; 
			 grad[76] = w_01_06; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = -1.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_81 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_81 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_81<T>::LiftConstraint_81 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_81_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_81<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_03*(w_05_03 - 1.0*w_05_08) - 1.0*w_06_08;
}

template <typename T>
void
LiftConstraint_81<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.00000000000000006123233995736766035868820147292*w_05_03 - 0.00000000000000006123233995736766035868820147292*w_05_08; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.00000000000000006123233995736766035868820147292*w_01_03; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = -0.00000000000000006123233995736766035868820147292*w_01_03; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = -1.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_82 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_82 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_82<T>::LiftConstraint_82 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_82_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_82<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_06*(w_05_03 - 1.0*w_05_08) - 1.0*w_06_09;
}

template <typename T>
void
LiftConstraint_82<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.00000000000000006123233995736766035868820147292*w_05_03 - 0.00000000000000006123233995736766035868820147292*w_05_08; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.00000000000000006123233995736766035868820147292*w_01_06; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = -0.00000000000000006123233995736766035868820147292*w_01_06; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = -1.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_83 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_83 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_83<T>::LiftConstraint_83 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_83_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_83<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = w_01_06*(w_05_12 + w_05_13) - 1.0*w_06_10;
}

template <typename T>
void
LiftConstraint_83<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = w_05_12 + w_05_13; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = w_01_06; 
			 grad[79] = w_01_06; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = -1.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_84 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_84 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_84<T>::LiftConstraint_84 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_84_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_84<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_03*(w_05_04 - 1.0*w_05_11) - 1.0*w_06_11;
}

template <typename T>
void
LiftConstraint_84<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.00000000000000006123233995736766035868820147292*w_05_04 - 0.00000000000000006123233995736766035868820147292*w_05_11; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.00000000000000006123233995736766035868820147292*w_01_03; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = -0.00000000000000006123233995736766035868820147292*w_01_03; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = -1.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_85 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_85 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_85<T>::LiftConstraint_85 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_85_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_85<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_06*(w_05_04 - 1.0*w_05_11) - 1.0*w_06_12;
}

template <typename T>
void
LiftConstraint_85<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.00000000000000006123233995736766035868820147292*w_05_04 - 0.00000000000000006123233995736766035868820147292*w_05_11; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.00000000000000006123233995736766035868820147292*w_01_06; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = -0.00000000000000006123233995736766035868820147292*w_01_06; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = -1.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_86 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_86 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_86<T>::LiftConstraint_86 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_86_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_86<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_02*(w_06_02 - 1.0*w_03_02 + w_06_09) - 1.0*w_07_01;
}

template <typename T>
void
LiftConstraint_86<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.00000000000000006123233995736766035868820147292*w_06_02 - 0.00000000000000006123233995736766035868820147292*w_03_02 + 0.00000000000000006123233995736766035868820147292*w_06_09; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = -0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = -1.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_87 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_87 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_87<T>::LiftConstraint_87 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_87_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_87<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_07_02 - 1.0*w_01_01*(w_03_01 + w_06_07 - 1.0*w_06_08);
}

template <typename T>
void
LiftConstraint_87<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = w_06_08 - 1.0*w_06_07 - 1.0*w_03_01; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = -1.0*w_01_01; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = -1.0*w_01_01; 
			 grad[87] = w_01_01; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = -1.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_88 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_88 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_88<T>::LiftConstraint_88 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_88_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_88<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.00000000000000006123233995736766035868820147292*w_01_02*(w_06_04 - 1.0*w_03_04 + w_06_12) - 1.0*w_07_03;
}

template <typename T>
void
LiftConstraint_88<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.00000000000000006123233995736766035868820147292*w_06_04 - 0.00000000000000006123233995736766035868820147292*w_03_04 + 0.00000000000000006123233995736766035868820147292*w_06_12; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = -0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.00000000000000006123233995736766035868820147292*w_01_02; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = -1.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class LiftConstraint_89 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit LiftConstraint_89 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
LiftConstraint_89<T>::LiftConstraint_89 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (96, 1, "LiftConstraint_89_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
LiftConstraint_89<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = - 1.0*w_07_04 - 1.0*w_01_01*(w_03_03 + w_06_10 - 1.0*w_06_11);
}

template <typename T>
void
LiftConstraint_89<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = w_06_11 - 1.0*w_06_10 - 1.0*w_03_03; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = -1.0*w_01_01; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = -1.0*w_01_01; 
			 grad[90] = w_01_01; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = -1.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class EEConstraint_1 : public roboptim::GenericLinearFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericLinearFunction<T>);
  
  explicit EEConstraint_1 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
EEConstraint_1<T>::EEConstraint_1 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericLinearFunction<T>
    (96, 1, "EEConstraint_1_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
EEConstraint_1<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.4*w_01_05 - 1.0*EE_1_1 + 0.36*w_02_04 + 0.36*w_02_05 + 0.87*w_02_09 - 0.55*w_03_01 - 0.2*w_03_02 + 0.87*w_03_06 - 0.87*w_03_07 - 0.63*w_04_05 + 0.63*w_04_07 + 0.16*w_05_03 - 0.16*w_05_08 + 0.532*w_06_01 + 0.2*w_06_02 - 0.55*w_06_07 + 0.55*w_06_08 + 0.2*w_06_09 - 0.532*w_07_01 + 0.532*w_07_02 + 0.65;
}

template <typename T>
void
EEConstraint_1<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.4; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.36; 
			 grad[25] = 0.36; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.87; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = -0.55; 
			 grad[40] = -0.2; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = 0.0; 
			 grad[44] = 0.87; 
			 grad[45] = -0.87; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = -0.63; 
			 grad[60] = 0.0; 
			 grad[61] = 0.63; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.16; 
			 grad[70] = 0.0; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = -0.16; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.532; 
			 grad[81] = 0.2; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = -0.55; 
			 grad[87] = 0.55; 
			 grad[88] = 0.2; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = -0.532; 
			 grad[93] = 0.532; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class EEConstraint_2 : public roboptim::GenericLinearFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericLinearFunction<T>);
  
  explicit EEConstraint_2 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
EEConstraint_2<T>::EEConstraint_2 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericLinearFunction<T>
    (96, 1, "EEConstraint_2_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
EEConstraint_2<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.4*w_01_08 - 1.0*EE_1_2 - 0.36*w_02_01 + 0.36*w_02_06 + 0.87*w_02_10 - 0.55*w_03_03 - 0.2*w_03_04 + 0.87*w_03_08 - 0.87*w_03_09 - 0.63*w_04_06 + 0.63*w_04_10 + 0.16*w_05_04 - 0.16*w_05_11 + 0.532*w_06_03 + 0.2*w_06_04 - 0.55*w_06_10 + 0.55*w_06_11 + 0.2*w_06_12 - 0.532*w_07_03 + 0.532*w_07_04;
}

template <typename T>
void
EEConstraint_2<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.4; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = -0.36; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.36; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.87; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = -0.55; 
			 grad[42] = -0.2; 
			 grad[43] = 0.0; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.87; 
			 grad[47] = -0.87; 
			 grad[48] = 0.0; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.0; 
			 grad[56] = 0.0; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = -0.63; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.63; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.0; 
			 grad[68] = 0.0; 
			 grad[69] = 0.0; 
			 grad[70] = 0.16; 
			 grad[71] = 0.0; 
			 grad[72] = 0.0; 
			 grad[73] = 0.0; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = -0.16; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.532; 
			 grad[83] = 0.2; 
			 grad[84] = 0.0; 
			 grad[85] = 0.0; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = -0.55; 
			 grad[90] = 0.55; 
			 grad[91] = 0.2; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = -0.532; 
			 grad[95] = 0.532; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class EEConstraint_3 : public roboptim::GenericLinearFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericLinearFunction<T>);
  
  explicit EEConstraint_3 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_1_3;
};

template <typename T>
EEConstraint_3<T>::EEConstraint_3 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_1_3) throw ()
  : roboptim::GenericLinearFunction<T>
    (96, 1, "EEConstraint_3_testRobot"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_1_3 (EE_1_3)
{}

template <typename T>
void
EEConstraint_3<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];
  
	result[0] = 0.000000000000000053272135762909869812217942235113*w_01_14 - 0.36*w_01_04 - 1.0*EE_1_3 - 0.55*w_02_02 - 0.2*w_02_03 - 0.87*w_02_07 - 0.87*w_02_08 - 0.63*w_03_05 + 0.63*w_03_10 + 0.16*w_04_01 - 0.16*w_04_02 + 0.532*w_05_01 + 0.2*w_05_02 - 0.55*w_05_05 + 0.55*w_05_06 + 0.2*w_05_07 - 0.532*w_06_05 + 0.532*w_06_06 + 1.1;
}

template <typename T>
void
EEConstraint_3<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& q_07 = x[6];
	const double& w_01_01 = x[7];
	const double& w_01_02 = x[8];
	const double& w_01_03 = x[9];
	const double& w_01_04 = x[10];
	const double& w_01_05 = x[11];
	const double& w_01_06 = x[12];
	const double& w_01_07 = x[13];
	const double& w_01_08 = x[14];
	const double& w_01_09 = x[15];
	const double& w_01_10 = x[16];
	const double& w_01_11 = x[17];
	const double& w_01_12 = x[18];
	const double& w_01_13 = x[19];
	const double& w_01_14 = x[20];
	const double& w_02_01 = x[21];
	const double& w_02_02 = x[22];
	const double& w_02_03 = x[23];
	const double& w_02_04 = x[24];
	const double& w_02_05 = x[25];
	const double& w_02_06 = x[26];
	const double& w_02_07 = x[27];
	const double& w_02_08 = x[28];
	const double& w_02_09 = x[29];
	const double& w_02_10 = x[30];
	const double& w_02_11 = x[31];
	const double& w_02_12 = x[32];
	const double& w_02_13 = x[33];
	const double& w_02_14 = x[34];
	const double& w_02_15 = x[35];
	const double& w_02_16 = x[36];
	const double& w_02_17 = x[37];
	const double& w_02_18 = x[38];
	const double& w_03_01 = x[39];
	const double& w_03_02 = x[40];
	const double& w_03_03 = x[41];
	const double& w_03_04 = x[42];
	const double& w_03_05 = x[43];
	const double& w_03_06 = x[44];
	const double& w_03_07 = x[45];
	const double& w_03_08 = x[46];
	const double& w_03_09 = x[47];
	const double& w_03_10 = x[48];
	const double& w_03_11 = x[49];
	const double& w_03_12 = x[50];
	const double& w_03_13 = x[51];
	const double& w_03_14 = x[52];
	const double& w_03_15 = x[53];
	const double& w_03_16 = x[54];
	const double& w_04_01 = x[55];
	const double& w_04_02 = x[56];
	const double& w_04_03 = x[57];
	const double& w_04_04 = x[58];
	const double& w_04_05 = x[59];
	const double& w_04_06 = x[60];
	const double& w_04_07 = x[61];
	const double& w_04_08 = x[62];
	const double& w_04_09 = x[63];
	const double& w_04_10 = x[64];
	const double& w_04_11 = x[65];
	const double& w_04_12 = x[66];
	const double& w_05_01 = x[67];
	const double& w_05_02 = x[68];
	const double& w_05_03 = x[69];
	const double& w_05_04 = x[70];
	const double& w_05_05 = x[71];
	const double& w_05_06 = x[72];
	const double& w_05_07 = x[73];
	const double& w_05_08 = x[74];
	const double& w_05_09 = x[75];
	const double& w_05_10 = x[76];
	const double& w_05_11 = x[77];
	const double& w_05_12 = x[78];
	const double& w_05_13 = x[79];
	const double& w_06_01 = x[80];
	const double& w_06_02 = x[81];
	const double& w_06_03 = x[82];
	const double& w_06_04 = x[83];
	const double& w_06_05 = x[84];
	const double& w_06_06 = x[85];
	const double& w_06_07 = x[86];
	const double& w_06_08 = x[87];
	const double& w_06_09 = x[88];
	const double& w_06_10 = x[89];
	const double& w_06_11 = x[90];
	const double& w_06_12 = x[91];
	const double& w_07_01 = x[92];
	const double& w_07_02 = x[93];
	const double& w_07_03 = x[94];
	const double& w_07_04 = x[95];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = -0.36; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.000000000000000053272135762909869812217942235113; 
			 grad[21] = 0.0; 
			 grad[22] = -0.55; 
			 grad[23] = -0.2; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = -0.87; 
			 grad[28] = -0.87; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
			 grad[42] = 0.0; 
			 grad[43] = -0.63; 
			 grad[44] = 0.0; 
			 grad[45] = 0.0; 
			 grad[46] = 0.0; 
			 grad[47] = 0.0; 
			 grad[48] = 0.63; 
			 grad[49] = 0.0; 
			 grad[50] = 0.0; 
			 grad[51] = 0.0; 
			 grad[52] = 0.0; 
			 grad[53] = 0.0; 
			 grad[54] = 0.0; 
			 grad[55] = 0.16; 
			 grad[56] = -0.16; 
			 grad[57] = 0.0; 
			 grad[58] = 0.0; 
			 grad[59] = 0.0; 
			 grad[60] = 0.0; 
			 grad[61] = 0.0; 
			 grad[62] = 0.0; 
			 grad[63] = 0.0; 
			 grad[64] = 0.0; 
			 grad[65] = 0.0; 
			 grad[66] = 0.0; 
			 grad[67] = 0.532; 
			 grad[68] = 0.2; 
			 grad[69] = 0.0; 
			 grad[70] = 0.0; 
			 grad[71] = -0.55; 
			 grad[72] = 0.55; 
			 grad[73] = 0.2; 
			 grad[74] = 0.0; 
			 grad[75] = 0.0; 
			 grad[76] = 0.0; 
			 grad[77] = 0.0; 
			 grad[78] = 0.0; 
			 grad[79] = 0.0; 
			 grad[80] = 0.0; 
			 grad[81] = 0.0; 
			 grad[82] = 0.0; 
			 grad[83] = 0.0; 
			 grad[84] = -0.532; 
			 grad[85] = 0.532; 
			 grad[86] = 0.0; 
			 grad[87] = 0.0; 
			 grad[88] = 0.0; 
			 grad[89] = 0.0; 
			 grad[90] = 0.0; 
			 grad[91] = 0.0; 
			 grad[92] = 0.0; 
			 grad[93] = 0.0; 
			 grad[94] = 0.0; 
			 grad[95] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
int main ()
{
  // Set the starting point.
  roboptim::Function::vector_t start (96);
  start[0] = 0.5797;
	start[1] = 0.54986;
	start[2] = 0.14495;
	start[3] = 0.85303;
	start[4] = 0.62206;
	start[5] = 0.35095;
	start[6] = 0.51325;
	start[7] = 0.40181;
	start[8] = 0.075967;
	start[9] = 0.23992;
	start[10] = 0.0;
	start[11] = 0.0;
	start[12] = 0.0;
	start[13] = 0.0;
	start[14] = 0.0;
	start[15] = 0.0;
	start[16] = 0.0;
	start[17] = 0.0;
	start[18] = 0.0;
	start[19] = 0.0;
	start[20] = 0.0;
	start[21] = 0.0;
	start[22] = 0.0;
	start[23] = 0.0;
	start[24] = 0.0;
	start[25] = 0.0;
	start[26] = 0.0;
	start[27] = 0.0;
	start[28] = 0.0;
	start[29] = 0.0;
	start[30] = 0.0;
	start[31] = 0.0;
	start[32] = 0.0;
	start[33] = 0.0;
	start[34] = 0.0;
	start[35] = 0.0;
	start[36] = 0.0;
	start[37] = 0.0;
	start[38] = 0.0;
	start[39] = 0.0;
	start[40] = 0.0;
	start[41] = 0.0;
	start[42] = 0.0;
	start[43] = 0.0;
	start[44] = 0.0;
	start[45] = 0.0;
	start[46] = 0.0;
	start[47] = 0.0;
	start[48] = 0.0;
	start[49] = 0.0;
	start[50] = 0.0;
	start[51] = 0.0;
	start[52] = 0.0;
	start[53] = 0.0;
	start[54] = 0.0;
	start[55] = 0.0;
	start[56] = 0.0;
	start[57] = 0.0;
	start[58] = 0.0;
	start[59] = 0.0;
	start[60] = 0.0;
	start[61] = 0.0;
	start[62] = 0.0;
	start[63] = 0.0;
	start[64] = 0.0;
	start[65] = 0.0;
	start[66] = 0.0;
	start[67] = 0.0;
	start[68] = 0.0;
	start[69] = 0.0;
	start[70] = 0.0;
	start[71] = 0.0;
	start[72] = 0.0;
	start[73] = 0.0;
	start[74] = 0.0;
	start[75] = 0.0;
	start[76] = 0.0;
	start[77] = 0.0;
	start[78] = 0.0;
	start[79] = 0.0;
	start[80] = 0.0;
	start[81] = 0.0;
	start[82] = 0.0;
	start[83] = 0.0;
	start[84] = 0.0;
	start[85] = 0.0;
	start[86] = 0.0;
	start[87] = 0.0;
	start[88] = 0.0;
	start[89] = 0.0;
	start[90] = 0.0;
	start[91] = 0.0;
	start[92] = 0.0;
	start[93] = 0.0;
	start[94] = 0.0;
	start[95] = 0.0;

  double EE_1_1 = 0.53834;
	double EE_1_2 = 0.99613;
	double EE_1_3 = 0.078176;

  boost::shared_ptr<CostFunction<roboptim::EigenMatrixDense> > cost = boost::make_shared<CostFunction<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);

	boost::shared_ptr<LiftConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_1 = boost::make_shared<LiftConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_2 = boost::make_shared<LiftConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_3<roboptim::EigenMatrixDense> > cstrFunc_3 = boost::make_shared<LiftConstraint_3<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_4<roboptim::EigenMatrixDense> > cstrFunc_4 = boost::make_shared<LiftConstraint_4<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_5<roboptim::EigenMatrixDense> > cstrFunc_5 = boost::make_shared<LiftConstraint_5<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_6<roboptim::EigenMatrixDense> > cstrFunc_6 = boost::make_shared<LiftConstraint_6<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_7<roboptim::EigenMatrixDense> > cstrFunc_7 = boost::make_shared<LiftConstraint_7<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_8<roboptim::EigenMatrixDense> > cstrFunc_8 = boost::make_shared<LiftConstraint_8<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_9<roboptim::EigenMatrixDense> > cstrFunc_9 = boost::make_shared<LiftConstraint_9<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_10<roboptim::EigenMatrixDense> > cstrFunc_10 = boost::make_shared<LiftConstraint_10<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_11<roboptim::EigenMatrixDense> > cstrFunc_11 = boost::make_shared<LiftConstraint_11<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_12<roboptim::EigenMatrixDense> > cstrFunc_12 = boost::make_shared<LiftConstraint_12<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_13<roboptim::EigenMatrixDense> > cstrFunc_13 = boost::make_shared<LiftConstraint_13<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_14<roboptim::EigenMatrixDense> > cstrFunc_14 = boost::make_shared<LiftConstraint_14<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_15<roboptim::EigenMatrixDense> > cstrFunc_15 = boost::make_shared<LiftConstraint_15<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_16<roboptim::EigenMatrixDense> > cstrFunc_16 = boost::make_shared<LiftConstraint_16<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_17<roboptim::EigenMatrixDense> > cstrFunc_17 = boost::make_shared<LiftConstraint_17<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_18<roboptim::EigenMatrixDense> > cstrFunc_18 = boost::make_shared<LiftConstraint_18<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_19<roboptim::EigenMatrixDense> > cstrFunc_19 = boost::make_shared<LiftConstraint_19<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_20<roboptim::EigenMatrixDense> > cstrFunc_20 = boost::make_shared<LiftConstraint_20<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_21<roboptim::EigenMatrixDense> > cstrFunc_21 = boost::make_shared<LiftConstraint_21<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_22<roboptim::EigenMatrixDense> > cstrFunc_22 = boost::make_shared<LiftConstraint_22<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_23<roboptim::EigenMatrixDense> > cstrFunc_23 = boost::make_shared<LiftConstraint_23<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_24<roboptim::EigenMatrixDense> > cstrFunc_24 = boost::make_shared<LiftConstraint_24<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_25<roboptim::EigenMatrixDense> > cstrFunc_25 = boost::make_shared<LiftConstraint_25<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_26<roboptim::EigenMatrixDense> > cstrFunc_26 = boost::make_shared<LiftConstraint_26<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_27<roboptim::EigenMatrixDense> > cstrFunc_27 = boost::make_shared<LiftConstraint_27<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_28<roboptim::EigenMatrixDense> > cstrFunc_28 = boost::make_shared<LiftConstraint_28<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_29<roboptim::EigenMatrixDense> > cstrFunc_29 = boost::make_shared<LiftConstraint_29<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_30<roboptim::EigenMatrixDense> > cstrFunc_30 = boost::make_shared<LiftConstraint_30<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_31<roboptim::EigenMatrixDense> > cstrFunc_31 = boost::make_shared<LiftConstraint_31<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_32<roboptim::EigenMatrixDense> > cstrFunc_32 = boost::make_shared<LiftConstraint_32<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_33<roboptim::EigenMatrixDense> > cstrFunc_33 = boost::make_shared<LiftConstraint_33<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_34<roboptim::EigenMatrixDense> > cstrFunc_34 = boost::make_shared<LiftConstraint_34<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_35<roboptim::EigenMatrixDense> > cstrFunc_35 = boost::make_shared<LiftConstraint_35<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_36<roboptim::EigenMatrixDense> > cstrFunc_36 = boost::make_shared<LiftConstraint_36<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_37<roboptim::EigenMatrixDense> > cstrFunc_37 = boost::make_shared<LiftConstraint_37<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_38<roboptim::EigenMatrixDense> > cstrFunc_38 = boost::make_shared<LiftConstraint_38<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_39<roboptim::EigenMatrixDense> > cstrFunc_39 = boost::make_shared<LiftConstraint_39<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_40<roboptim::EigenMatrixDense> > cstrFunc_40 = boost::make_shared<LiftConstraint_40<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_41<roboptim::EigenMatrixDense> > cstrFunc_41 = boost::make_shared<LiftConstraint_41<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_42<roboptim::EigenMatrixDense> > cstrFunc_42 = boost::make_shared<LiftConstraint_42<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_43<roboptim::EigenMatrixDense> > cstrFunc_43 = boost::make_shared<LiftConstraint_43<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_44<roboptim::EigenMatrixDense> > cstrFunc_44 = boost::make_shared<LiftConstraint_44<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_45<roboptim::EigenMatrixDense> > cstrFunc_45 = boost::make_shared<LiftConstraint_45<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_46<roboptim::EigenMatrixDense> > cstrFunc_46 = boost::make_shared<LiftConstraint_46<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_47<roboptim::EigenMatrixDense> > cstrFunc_47 = boost::make_shared<LiftConstraint_47<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_48<roboptim::EigenMatrixDense> > cstrFunc_48 = boost::make_shared<LiftConstraint_48<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_49<roboptim::EigenMatrixDense> > cstrFunc_49 = boost::make_shared<LiftConstraint_49<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_50<roboptim::EigenMatrixDense> > cstrFunc_50 = boost::make_shared<LiftConstraint_50<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_51<roboptim::EigenMatrixDense> > cstrFunc_51 = boost::make_shared<LiftConstraint_51<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_52<roboptim::EigenMatrixDense> > cstrFunc_52 = boost::make_shared<LiftConstraint_52<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_53<roboptim::EigenMatrixDense> > cstrFunc_53 = boost::make_shared<LiftConstraint_53<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_54<roboptim::EigenMatrixDense> > cstrFunc_54 = boost::make_shared<LiftConstraint_54<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_55<roboptim::EigenMatrixDense> > cstrFunc_55 = boost::make_shared<LiftConstraint_55<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_56<roboptim::EigenMatrixDense> > cstrFunc_56 = boost::make_shared<LiftConstraint_56<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_57<roboptim::EigenMatrixDense> > cstrFunc_57 = boost::make_shared<LiftConstraint_57<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_58<roboptim::EigenMatrixDense> > cstrFunc_58 = boost::make_shared<LiftConstraint_58<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_59<roboptim::EigenMatrixDense> > cstrFunc_59 = boost::make_shared<LiftConstraint_59<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_60<roboptim::EigenMatrixDense> > cstrFunc_60 = boost::make_shared<LiftConstraint_60<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_61<roboptim::EigenMatrixDense> > cstrFunc_61 = boost::make_shared<LiftConstraint_61<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_62<roboptim::EigenMatrixDense> > cstrFunc_62 = boost::make_shared<LiftConstraint_62<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_63<roboptim::EigenMatrixDense> > cstrFunc_63 = boost::make_shared<LiftConstraint_63<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_64<roboptim::EigenMatrixDense> > cstrFunc_64 = boost::make_shared<LiftConstraint_64<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_65<roboptim::EigenMatrixDense> > cstrFunc_65 = boost::make_shared<LiftConstraint_65<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_66<roboptim::EigenMatrixDense> > cstrFunc_66 = boost::make_shared<LiftConstraint_66<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_67<roboptim::EigenMatrixDense> > cstrFunc_67 = boost::make_shared<LiftConstraint_67<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_68<roboptim::EigenMatrixDense> > cstrFunc_68 = boost::make_shared<LiftConstraint_68<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_69<roboptim::EigenMatrixDense> > cstrFunc_69 = boost::make_shared<LiftConstraint_69<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_70<roboptim::EigenMatrixDense> > cstrFunc_70 = boost::make_shared<LiftConstraint_70<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_71<roboptim::EigenMatrixDense> > cstrFunc_71 = boost::make_shared<LiftConstraint_71<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_72<roboptim::EigenMatrixDense> > cstrFunc_72 = boost::make_shared<LiftConstraint_72<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_73<roboptim::EigenMatrixDense> > cstrFunc_73 = boost::make_shared<LiftConstraint_73<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_74<roboptim::EigenMatrixDense> > cstrFunc_74 = boost::make_shared<LiftConstraint_74<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_75<roboptim::EigenMatrixDense> > cstrFunc_75 = boost::make_shared<LiftConstraint_75<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_76<roboptim::EigenMatrixDense> > cstrFunc_76 = boost::make_shared<LiftConstraint_76<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_77<roboptim::EigenMatrixDense> > cstrFunc_77 = boost::make_shared<LiftConstraint_77<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_78<roboptim::EigenMatrixDense> > cstrFunc_78 = boost::make_shared<LiftConstraint_78<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_79<roboptim::EigenMatrixDense> > cstrFunc_79 = boost::make_shared<LiftConstraint_79<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_80<roboptim::EigenMatrixDense> > cstrFunc_80 = boost::make_shared<LiftConstraint_80<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_81<roboptim::EigenMatrixDense> > cstrFunc_81 = boost::make_shared<LiftConstraint_81<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_82<roboptim::EigenMatrixDense> > cstrFunc_82 = boost::make_shared<LiftConstraint_82<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_83<roboptim::EigenMatrixDense> > cstrFunc_83 = boost::make_shared<LiftConstraint_83<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_84<roboptim::EigenMatrixDense> > cstrFunc_84 = boost::make_shared<LiftConstraint_84<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_85<roboptim::EigenMatrixDense> > cstrFunc_85 = boost::make_shared<LiftConstraint_85<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_86<roboptim::EigenMatrixDense> > cstrFunc_86 = boost::make_shared<LiftConstraint_86<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_87<roboptim::EigenMatrixDense> > cstrFunc_87 = boost::make_shared<LiftConstraint_87<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_88<roboptim::EigenMatrixDense> > cstrFunc_88 = boost::make_shared<LiftConstraint_88<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<LiftConstraint_89<roboptim::EigenMatrixDense> > cstrFunc_89 = boost::make_shared<LiftConstraint_89<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_90 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_91 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);
	boost::shared_ptr<EEConstraint_3<roboptim::EigenMatrixDense> > cstrFunc_92 = boost::make_shared<EEConstraint_3<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_1_3);

  //Create problem
  solver_t::problem_t pb (*cost);

  // Set bounds for all optimization parameters
  pb.argumentBounds ()[0] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[1] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[2] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[3] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[4] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[5] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[6] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[7] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[8] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[9] = roboptim::Function::makeInterval (-pi, pi);

  // Create constraints.
  {
		LiftConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_1), bounds, scales); 
	}
	{
		LiftConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_2), bounds, scales); 
	}
	{
		LiftConstraint_3<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_3), bounds, scales); 
	}
	{
		LiftConstraint_4<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_4), bounds, scales); 
	}
	{
		LiftConstraint_5<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_5), bounds, scales); 
	}
	{
		LiftConstraint_6<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_6), bounds, scales); 
	}
	{
		LiftConstraint_7<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_7), bounds, scales); 
	}
	{
		LiftConstraint_8<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_8), bounds, scales); 
	}
	{
		LiftConstraint_9<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_9), bounds, scales); 
	}
	{
		LiftConstraint_10<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_10), bounds, scales); 
	}
	{
		LiftConstraint_11<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_11), bounds, scales); 
	}
	{
		LiftConstraint_12<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_12), bounds, scales); 
	}
	{
		LiftConstraint_13<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_13), bounds, scales); 
	}
	{
		LiftConstraint_14<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_14), bounds, scales); 
	}
	{
		LiftConstraint_15<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_15), bounds, scales); 
	}
	{
		LiftConstraint_16<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_16), bounds, scales); 
	}
	{
		LiftConstraint_17<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_17), bounds, scales); 
	}
	{
		LiftConstraint_18<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_18), bounds, scales); 
	}
	{
		LiftConstraint_19<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_19), bounds, scales); 
	}
	{
		LiftConstraint_20<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_20), bounds, scales); 
	}
	{
		LiftConstraint_21<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_21), bounds, scales); 
	}
	{
		LiftConstraint_22<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_22), bounds, scales); 
	}
	{
		LiftConstraint_23<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_23), bounds, scales); 
	}
	{
		LiftConstraint_24<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_24), bounds, scales); 
	}
	{
		LiftConstraint_25<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_25), bounds, scales); 
	}
	{
		LiftConstraint_26<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_26), bounds, scales); 
	}
	{
		LiftConstraint_27<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_27), bounds, scales); 
	}
	{
		LiftConstraint_28<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_28), bounds, scales); 
	}
	{
		LiftConstraint_29<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_29), bounds, scales); 
	}
	{
		LiftConstraint_30<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_30), bounds, scales); 
	}
	{
		LiftConstraint_31<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_31), bounds, scales); 
	}
	{
		LiftConstraint_32<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_32), bounds, scales); 
	}
	{
		LiftConstraint_33<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_33), bounds, scales); 
	}
	{
		LiftConstraint_34<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_34), bounds, scales); 
	}
	{
		LiftConstraint_35<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_35), bounds, scales); 
	}
	{
		LiftConstraint_36<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_36), bounds, scales); 
	}
	{
		LiftConstraint_37<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_37), bounds, scales); 
	}
	{
		LiftConstraint_38<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_38), bounds, scales); 
	}
	{
		LiftConstraint_39<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_39), bounds, scales); 
	}
	{
		LiftConstraint_40<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_40), bounds, scales); 
	}
	{
		LiftConstraint_41<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_41), bounds, scales); 
	}
	{
		LiftConstraint_42<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_42), bounds, scales); 
	}
	{
		LiftConstraint_43<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_43), bounds, scales); 
	}
	{
		LiftConstraint_44<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_44), bounds, scales); 
	}
	{
		LiftConstraint_45<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_45), bounds, scales); 
	}
	{
		LiftConstraint_46<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_46), bounds, scales); 
	}
	{
		LiftConstraint_47<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_47), bounds, scales); 
	}
	{
		LiftConstraint_48<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_48), bounds, scales); 
	}
	{
		LiftConstraint_49<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_49), bounds, scales); 
	}
	{
		LiftConstraint_50<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_50), bounds, scales); 
	}
	{
		LiftConstraint_51<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_51), bounds, scales); 
	}
	{
		LiftConstraint_52<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_52), bounds, scales); 
	}
	{
		LiftConstraint_53<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_53), bounds, scales); 
	}
	{
		LiftConstraint_54<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_54), bounds, scales); 
	}
	{
		LiftConstraint_55<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_55), bounds, scales); 
	}
	{
		LiftConstraint_56<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_56), bounds, scales); 
	}
	{
		LiftConstraint_57<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_57), bounds, scales); 
	}
	{
		LiftConstraint_58<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_58), bounds, scales); 
	}
	{
		LiftConstraint_59<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_59), bounds, scales); 
	}
	{
		LiftConstraint_60<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_60), bounds, scales); 
	}
	{
		LiftConstraint_61<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_61), bounds, scales); 
	}
	{
		LiftConstraint_62<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_62), bounds, scales); 
	}
	{
		LiftConstraint_63<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_63), bounds, scales); 
	}
	{
		LiftConstraint_64<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_64), bounds, scales); 
	}
	{
		LiftConstraint_65<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_65), bounds, scales); 
	}
	{
		LiftConstraint_66<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_66), bounds, scales); 
	}
	{
		LiftConstraint_67<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_67), bounds, scales); 
	}
	{
		LiftConstraint_68<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_68), bounds, scales); 
	}
	{
		LiftConstraint_69<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_69), bounds, scales); 
	}
	{
		LiftConstraint_70<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_70), bounds, scales); 
	}
	{
		LiftConstraint_71<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_71), bounds, scales); 
	}
	{
		LiftConstraint_72<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_72), bounds, scales); 
	}
	{
		LiftConstraint_73<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_73), bounds, scales); 
	}
	{
		LiftConstraint_74<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_74), bounds, scales); 
	}
	{
		LiftConstraint_75<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_75), bounds, scales); 
	}
	{
		LiftConstraint_76<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_76), bounds, scales); 
	}
	{
		LiftConstraint_77<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_77), bounds, scales); 
	}
	{
		LiftConstraint_78<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_78), bounds, scales); 
	}
	{
		LiftConstraint_79<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_79), bounds, scales); 
	}
	{
		LiftConstraint_80<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_80), bounds, scales); 
	}
	{
		LiftConstraint_81<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_81), bounds, scales); 
	}
	{
		LiftConstraint_82<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_82), bounds, scales); 
	}
	{
		LiftConstraint_83<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_83), bounds, scales); 
	}
	{
		LiftConstraint_84<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_84), bounds, scales); 
	}
	{
		LiftConstraint_85<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_85), bounds, scales); 
	}
	{
		LiftConstraint_86<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_86), bounds, scales); 
	}
	{
		LiftConstraint_87<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_87), bounds, scales); 
	}
	{
		LiftConstraint_88<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_88), bounds, scales); 
	}
	{
		LiftConstraint_89<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_89), bounds, scales); 
	}
	{
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_90), bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_91), bounds, scales); 
	}
	{
		EEConstraint_3<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_92), bounds, scales); 
	}

  pb.startingPoint () = start;
  roboptim::SolverFactory<solver_t> factory ("cfsqp", pb);
  solver_t& solver = factory ();

  solver_t::result_t res = solver.minimum ();

  
  std::cout << solver << std::endl;

  // Process the result
  switch (res.which ())
    {
    case solver_t::SOLVER_VALUE:
      {
        // Get the result.
	roboptim::Result& result =
	  boost::get<roboptim::Result> (res);

        // Display the result.
	std::cout << "A solution has been found: " << std::endl
                  << result << std::endl;

        return 0;
      }

    case solver_t::SOLVER_VALUE_WARNINGS:
      {
        // Get the result.
        roboptim::ResultWithWarnings& result =
	  boost::get<roboptim::ResultWithWarnings> (res);

        // Display the result.
	std::cout << "A solution has been found: " << std::endl
                  << result << std::endl;

        return 0;
      }

    case solver_t::SOLVER_NO_SOLUTION:
    case solver_t::SOLVER_ERROR:
      {
	std::cout << "A solution should have been found. Failing..."
                  << std::endl
                  << boost::get<roboptim::SolverError> (res).what ()
                  << std::endl;

        return 2;
      }
    }

  return 0;
}
