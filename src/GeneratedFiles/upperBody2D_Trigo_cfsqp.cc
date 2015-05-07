// compile command: 
// g++ -ggdb3 -I/usr/include/log4cxx `pkg-config --cflags roboptim-core` src/generatedFiles/@FUNCTION_NAME@.cc `pkg-config --libs roboptim-core` -o bin/@FUNCTION_NAME@
#include <iostream>
#include <vector>
#include <boost/mpl/vector.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/timer/timer.hpp>
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
CostFunction<T>::CostFunction (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericLinearFunction<T>
    (36, 1, "CostFunction_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
CostFunction<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = 0.0;
}

template <typename T>
void
CostFunction<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_1<T>::LiftConstraint_1 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_1_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_1<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = pow(w_01_01,2) + pow(w_01_06,2) - 1.0;
}

template <typename T>
void
LiftConstraint_1<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 2.0*w_01_01; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 2.0*w_01_06; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_2<T>::LiftConstraint_2 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_2_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_2<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = pow(w_01_02,2) + pow(w_01_04,2) - 1.0;
}

template <typename T>
void
LiftConstraint_2<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 2.0*w_01_02; 
			 grad[2] = 0.0; 
			 grad[3] = 2.0*w_01_04; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_3<T>::LiftConstraint_3 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_3_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_3<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = pow(w_01_03,2) + pow(w_01_08,2) - 1.0;
}

template <typename T>
void
LiftConstraint_3<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 2.0*w_01_03; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 2.0*w_01_08; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_4<T>::LiftConstraint_4 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_4_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_4<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = pow(w_01_05,2) + pow(w_01_09,2) - 1.0;
}

template <typename T>
void
LiftConstraint_4<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 2.0*w_01_05; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 2.0*w_01_09; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_5<T>::LiftConstraint_5 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_5_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_5<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = pow(w_01_07,2) + pow(w_01_10,2) - 1.0;
}

template <typename T>
void
LiftConstraint_5<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 2.0*w_01_07; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 2.0*w_01_10; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_6<T>::LiftConstraint_6 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_6_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_6<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = pow(w_01_11,2) + pow(w_01_12,2) - 1.0;
}

template <typename T>
void
LiftConstraint_6<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

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
			 grad[10] = 2.0*w_01_11; 
			 grad[11] = 2.0*w_01_12; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_7<T>::LiftConstraint_7 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_7_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_7<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_02*w_01_11 - 1.0*w_02_01;
}

template <typename T>
void
LiftConstraint_7<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = w_01_11; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = w_01_02; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_8<T>::LiftConstraint_8 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_8_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_8<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_04*w_01_11 - 1.0*w_02_02;
}

template <typename T>
void
LiftConstraint_8<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_01_11; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = w_01_04; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_9<T>::LiftConstraint_9 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_9_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_9<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_02*w_01_12 - 1.0*w_02_03;
}

template <typename T>
void
LiftConstraint_9<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = w_01_12; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = w_01_02; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_10<T>::LiftConstraint_10 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_10_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_10<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_04*w_01_12 - 1.0*w_02_04;
}

template <typename T>
void
LiftConstraint_10<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_01_12; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = w_01_04; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_11<T>::LiftConstraint_11 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_11_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_11<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = - 1.0*w_03_01 - 1.0*w_01_05*(w_02_02 + w_02_03);
}

template <typename T>
void
LiftConstraint_11<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = -1.0*w_01_05; 
			 grad[14] = -1.0*w_01_05; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_12<T>::LiftConstraint_12 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_12_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_12<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_05*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_02;
}

template <typename T>
void
LiftConstraint_12<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = w_02_01 - 1.0*w_02_04; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = w_01_05; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = -1.0*w_01_05; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_13<T>::LiftConstraint_13 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_13_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_13<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = - 1.0*w_03_03 - 1.0*w_01_07*(w_02_02 + w_02_03);
}

template <typename T>
void
LiftConstraint_13<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = -1.0*w_01_07; 
			 grad[14] = -1.0*w_01_07; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_14<T>::LiftConstraint_14 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_14_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_14<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_07*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_04;
}

template <typename T>
void
LiftConstraint_14<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = w_02_01 - 1.0*w_02_04; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = w_01_07; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = -1.0*w_01_07; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_15<T>::LiftConstraint_15 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_15_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_15<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_09*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_05;
}

template <typename T>
void
LiftConstraint_15<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

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
			 grad[8] = w_02_01 - 1.0*w_02_04; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = w_01_09; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = -1.0*w_01_09; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_16<T>::LiftConstraint_16 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_16_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_16<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_09*(w_02_02 + w_02_03) - 1.0*w_03_06;
}

template <typename T>
void
LiftConstraint_16<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

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
			 grad[8] = w_02_02 + w_02_03; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_01_09; 
			 grad[14] = w_01_09; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_17<T>::LiftConstraint_17 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_17_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_17<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_10*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_07;
}

template <typename T>
void
LiftConstraint_17<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

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
			 grad[9] = w_02_01 - 1.0*w_02_04; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = w_01_10; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = -1.0*w_01_10; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_18<T>::LiftConstraint_18 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_18_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_18<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_10*(w_02_02 + w_02_03) - 1.0*w_03_08;
}

template <typename T>
void
LiftConstraint_18<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

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
			 grad[9] = w_02_02 + w_02_03; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_01_10; 
			 grad[14] = w_01_10; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_19<T>::LiftConstraint_19 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_19_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_19<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = - 1.0*w_03_09 - 1.0*w_01_09*(w_02_02 + w_02_03);
}

template <typename T>
void
LiftConstraint_19<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

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
			 grad[8] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = -1.0*w_01_09; 
			 grad[14] = -1.0*w_01_09; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_20<T>::LiftConstraint_20 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_20_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_20<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_05*(w_02_02 + w_02_03) - 1.0*w_03_10;
}

template <typename T>
void
LiftConstraint_20<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = w_02_02 + w_02_03; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_01_05; 
			 grad[14] = w_01_05; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_21<T>::LiftConstraint_21 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_21_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_21<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = - 1.0*w_03_11 - 1.0*w_01_10*(w_02_02 + w_02_03);
}

template <typename T>
void
LiftConstraint_21<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

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
			 grad[9] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = -1.0*w_01_10; 
			 grad[14] = -1.0*w_01_10; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_22<T>::LiftConstraint_22 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_22_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_22<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_07*(w_02_02 + w_02_03) - 1.0*w_03_12;
}

template <typename T>
void
LiftConstraint_22<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = w_02_02 + w_02_03; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_01_07; 
			 grad[14] = w_01_07; 
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
			 grad[27] = -1.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_23<T>::LiftConstraint_23 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_23_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_23<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_01*(w_03_01 - 1.0*w_03_05) - 1.0*w_04_01;
}

template <typename T>
void
LiftConstraint_23<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = w_03_01 - 1.0*w_03_05; 
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
			 grad[16] = w_01_01; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = -1.0*w_01_01; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_24<T>::LiftConstraint_24 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_24_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_24<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_01*(w_03_02 - 1.0*w_03_06) - 1.0*w_04_02;
}

template <typename T>
void
LiftConstraint_24<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = w_03_02 - 1.0*w_03_06; 
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
			 grad[17] = w_01_01; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = -1.0*w_01_01; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_25<T>::LiftConstraint_25 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_25_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_25<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_03*(w_03_03 - 1.0*w_03_07) - 1.0*w_04_03;
}

template <typename T>
void
LiftConstraint_25<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = w_03_03 - 1.0*w_03_07; 
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
			 grad[18] = w_01_03; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = -1.0*w_01_03; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_26<T>::LiftConstraint_26 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_26_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_26<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_03*(w_03_04 - 1.0*w_03_08) - 1.0*w_04_04;
}

template <typename T>
void
LiftConstraint_26<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = w_03_04 - 1.0*w_03_08; 
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
			 grad[19] = w_01_03; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = -1.0*w_01_03; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_27<T>::LiftConstraint_27 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_27_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_27<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_06*(w_03_02 + w_03_09) - 1.0*w_04_05;
}

template <typename T>
void
LiftConstraint_27<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_03_02 + w_03_09; 
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
			 grad[17] = w_01_06; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = w_01_06; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_28<T>::LiftConstraint_28 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_28_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_28<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_06*(w_03_05 + w_03_10) - 1.0*w_04_06;
}

template <typename T>
void
LiftConstraint_28<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_03_05 + w_03_10; 
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
			 grad[20] = w_01_06; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = w_01_06; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_29<T>::LiftConstraint_29 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_29_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_29<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_08*(w_03_04 + w_03_11) - 1.0*w_04_07;
}

template <typename T>
void
LiftConstraint_29<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

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
			 grad[7] = w_03_04 + w_03_11; 
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
			 grad[19] = w_01_08; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = w_01_08; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = -1.0; 
			 grad[35] = 0.0; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
LiftConstraint_30<T>::LiftConstraint_30 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (36, 1, "LiftConstraint_30_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
LiftConstraint_30<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = w_01_08*(w_03_07 + w_03_12) - 1.0*w_04_08;
}

template <typename T>
void
LiftConstraint_30<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

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
			 grad[7] = w_03_07 + w_03_12; 
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
			 grad[22] = w_01_08; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = w_01_08; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = -1.0; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
EEConstraint_1<T>::EEConstraint_1 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericLinearFunction<T>
    (36, 1, "EEConstraint_1_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
EEConstraint_1<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = 0.25*w_02_04 - 0.5*w_01_04 - 0.25*w_02_01 - 0.5*w_02_02 - 0.5*w_02_03 - 1.0*EE_1_1 + 0.25*w_03_01 - 0.25*w_03_05 + 0.3*w_04_01 - 0.3*w_04_05;
}

template <typename T>
void
EEConstraint_1<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = -0.5; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = -0.25; 
			 grad[13] = -0.5; 
			 grad[14] = -0.5; 
			 grad[15] = 0.25; 
			 grad[16] = 0.25; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = -0.25; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.3; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = -0.3; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
EEConstraint_2<T>::EEConstraint_2 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericLinearFunction<T>
    (36, 1, "EEConstraint_2_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
EEConstraint_2<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = 0.5*w_01_02 - 1.0*EE_1_2 + 0.5*w_02_01 - 0.25*w_02_02 - 0.25*w_02_03 - 0.5*w_02_04 + 0.25*w_03_02 - 0.25*w_03_06 + 0.3*w_04_02 - 0.3*w_04_06;
}

template <typename T>
void
EEConstraint_2<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.5; 
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
			 grad[12] = 0.5; 
			 grad[13] = -0.25; 
			 grad[14] = -0.25; 
			 grad[15] = -0.5; 
			 grad[16] = 0.0; 
			 grad[17] = 0.25; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = -0.25; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.3; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = -0.3; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
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
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
EEConstraint_3<T>::EEConstraint_3 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericLinearFunction<T>
    (36, 1, "EEConstraint_3_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
EEConstraint_3<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = 0.25*w_02_01 - 0.5*w_01_04 - 1.0*EE_2_1 - 0.5*w_02_02 - 0.5*w_02_03 - 0.25*w_02_04 + 0.25*w_03_03 - 0.25*w_03_07 + 0.3*w_04_03 - 0.3*w_04_07;
}

template <typename T>
void
EEConstraint_3<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = -0.5; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.25; 
			 grad[13] = -0.5; 
			 grad[14] = -0.5; 
			 grad[15] = -0.25; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.25; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = -0.25; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.3; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = -0.3; 
			 grad[35] = 0.0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class EEConstraint_4 : public roboptim::GenericLinearFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericLinearFunction<T>);
  
  explicit EEConstraint_4 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
EEConstraint_4<T>::EEConstraint_4 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericLinearFunction<T>
    (36, 1, "EEConstraint_4_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
EEConstraint_4<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];
  
	result[0] = 0.5*w_01_02 - 1.0*EE_2_2 + 0.5*w_02_01 + 0.25*w_02_02 + 0.25*w_02_03 - 0.5*w_02_04 + 0.25*w_03_04 - 0.25*w_03_08 + 0.3*w_04_04 - 0.3*w_04_08;
}

template <typename T>
void
EEConstraint_4<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& w_01_01 = x[0];
	const double& w_01_02 = x[1];
	const double& w_01_03 = x[2];
	const double& w_01_04 = x[3];
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_01_07 = x[6];
	const double& w_01_08 = x[7];
	const double& w_01_09 = x[8];
	const double& w_01_10 = x[9];
	const double& w_01_11 = x[10];
	const double& w_01_12 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_03_07 = x[22];
	const double& w_03_08 = x[23];
	const double& w_03_09 = x[24];
	const double& w_03_10 = x[25];
	const double& w_03_11 = x[26];
	const double& w_03_12 = x[27];
	const double& w_04_01 = x[28];
	const double& w_04_02 = x[29];
	const double& w_04_03 = x[30];
	const double& w_04_04 = x[31];
	const double& w_04_05 = x[32];
	const double& w_04_06 = x[33];
	const double& w_04_07 = x[34];
	const double& w_04_08 = x[35];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.5; 
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
			 grad[12] = 0.5; 
			 grad[13] = 0.25; 
			 grad[14] = 0.25; 
			 grad[15] = -0.5; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.25; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = -0.25; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.3; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = -0.3; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}

void solveFor (const std::vector<double>& initPos, const std::vector<double>& endEffectors)
{
  // Set the starting point.
  roboptim::Function::vector_t start ( initPos.size());
  for( int i = 0; i<initPos.size(); i++)
    start[i] = initPos[i];
  
  // Set the End Effector Goal Position
  double EE_1_1 = endEffectors[0];
	double EE_1_2 = endEffectors[1];
	double EE_2_1 = endEffectors[2];
	double EE_2_2 = endEffectors[3];

  boost::shared_ptr<CostFunction<roboptim::EigenMatrixDense> > cost = boost::make_shared<CostFunction<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);

	boost::shared_ptr<LiftConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_1 = boost::make_shared<LiftConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_2 = boost::make_shared<LiftConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_3<roboptim::EigenMatrixDense> > cstrFunc_3 = boost::make_shared<LiftConstraint_3<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_4<roboptim::EigenMatrixDense> > cstrFunc_4 = boost::make_shared<LiftConstraint_4<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_5<roboptim::EigenMatrixDense> > cstrFunc_5 = boost::make_shared<LiftConstraint_5<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_6<roboptim::EigenMatrixDense> > cstrFunc_6 = boost::make_shared<LiftConstraint_6<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_7<roboptim::EigenMatrixDense> > cstrFunc_7 = boost::make_shared<LiftConstraint_7<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_8<roboptim::EigenMatrixDense> > cstrFunc_8 = boost::make_shared<LiftConstraint_8<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_9<roboptim::EigenMatrixDense> > cstrFunc_9 = boost::make_shared<LiftConstraint_9<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_10<roboptim::EigenMatrixDense> > cstrFunc_10 = boost::make_shared<LiftConstraint_10<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_11<roboptim::EigenMatrixDense> > cstrFunc_11 = boost::make_shared<LiftConstraint_11<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_12<roboptim::EigenMatrixDense> > cstrFunc_12 = boost::make_shared<LiftConstraint_12<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_13<roboptim::EigenMatrixDense> > cstrFunc_13 = boost::make_shared<LiftConstraint_13<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_14<roboptim::EigenMatrixDense> > cstrFunc_14 = boost::make_shared<LiftConstraint_14<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_15<roboptim::EigenMatrixDense> > cstrFunc_15 = boost::make_shared<LiftConstraint_15<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_16<roboptim::EigenMatrixDense> > cstrFunc_16 = boost::make_shared<LiftConstraint_16<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_17<roboptim::EigenMatrixDense> > cstrFunc_17 = boost::make_shared<LiftConstraint_17<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_18<roboptim::EigenMatrixDense> > cstrFunc_18 = boost::make_shared<LiftConstraint_18<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_19<roboptim::EigenMatrixDense> > cstrFunc_19 = boost::make_shared<LiftConstraint_19<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_20<roboptim::EigenMatrixDense> > cstrFunc_20 = boost::make_shared<LiftConstraint_20<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_21<roboptim::EigenMatrixDense> > cstrFunc_21 = boost::make_shared<LiftConstraint_21<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_22<roboptim::EigenMatrixDense> > cstrFunc_22 = boost::make_shared<LiftConstraint_22<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_23<roboptim::EigenMatrixDense> > cstrFunc_23 = boost::make_shared<LiftConstraint_23<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_24<roboptim::EigenMatrixDense> > cstrFunc_24 = boost::make_shared<LiftConstraint_24<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_25<roboptim::EigenMatrixDense> > cstrFunc_25 = boost::make_shared<LiftConstraint_25<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_26<roboptim::EigenMatrixDense> > cstrFunc_26 = boost::make_shared<LiftConstraint_26<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_27<roboptim::EigenMatrixDense> > cstrFunc_27 = boost::make_shared<LiftConstraint_27<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_28<roboptim::EigenMatrixDense> > cstrFunc_28 = boost::make_shared<LiftConstraint_28<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_29<roboptim::EigenMatrixDense> > cstrFunc_29 = boost::make_shared<LiftConstraint_29<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_30<roboptim::EigenMatrixDense> > cstrFunc_30 = boost::make_shared<LiftConstraint_30<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_31 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_32 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<EEConstraint_3<roboptim::EigenMatrixDense> > cstrFunc_33 = boost::make_shared<EEConstraint_3<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<EEConstraint_4<roboptim::EigenMatrixDense> > cstrFunc_34 = boost::make_shared<EEConstraint_4<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);

  //Create problem
  solver_t::problem_t pb (*cost);

  // Set bounds for all optimization parameters
  pb.argumentBounds ()[0] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[1] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[2] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[3] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[4] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[5] = roboptim::Function::makeInterval (-pi, pi);

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
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_31), bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_32), bounds, scales); 
	}
	{
		EEConstraint_3<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_33), bounds, scales); 
	}
	{
		EEConstraint_4<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_34), bounds, scales); 
	}

  pb.startingPoint () = start;
  

  {
    roboptim::SolverFactory<solver_t> factoryTMP ("cfsqp", pb);
    boost::timer::auto_cpu_timer t;
    for( int i = 0; i<100; i++)
    {
      roboptim::SolverFactory<solver_t> factory ("cfsqp", pb);
      solver_t& solver = factory ();
      solver.solve();
    }
  }

  roboptim::SolverFactory<solver_t> factory ("cfsqp", pb);
  solver_t& solver = factory ();
  solver.solve();
  solver_t::result_t res = solver.minimum ();

  
  //std::cout << solver << std::endl;

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

        return;
      }

    case solver_t::SOLVER_VALUE_WARNINGS:
      {
        // Get the result.
        roboptim::ResultWithWarnings& result =
	  boost::get<roboptim::ResultWithWarnings> (res);

        // Display the result.
	std::cout << "A solution has been found: " << std::endl
                  << result << std::endl;

        return;
      }

    case solver_t::SOLVER_NO_SOLUTION:
    case solver_t::SOLVER_ERROR:
      {
	std::cout << "A solution should have been found. Failing..."
                  << std::endl
                  << boost::get<roboptim::SolverError> (res).what ()
                  << std::endl;

        return;
      }
    }

  return;
}

int main ()
{
  	{
		std::vector<double> start (36);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0035661;
		start[7] = 0.002538;
		start[8] = 0.0061938;
		start[9] = 0.0089607;
		start[10] = 0.0044753;
		start[11] = 0.0063124;
		start[12] = 0.0051136;
		start[13] = 0.0028119;
		start[14] = 0.0055288;
		start[15] = 0.0058496;
		start[16] = 0.0092788;
		start[17] = 0.0035272;
		start[18] = 0.0034631;
		start[19] = 0.001161;
		start[20] = 0.0076391;
		start[21] = 0.0065518;
		start[22] = 0.0020434;
		start[23] = 0.0050977;
		start[24] = 0.0055354;
		start[25] = 0.0010135;
		start[26] = 0.0098097;
		start[27] = 0.0071081;
		start[28] = 0.001325;
		start[29] = 0.0019594;
		start[30] = 0.0031013;
		start[31] = 0.0078869;
		start[32] = 0.0051586;
		start[33] = 0.0023515;
		start[34] = 0.001354;
		start[35] = 0.0088116;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0020865;
		start[7] = 0.0041138;
		start[8] = 0.0081229;
		start[9] = 0.0072383;
		start[10] = 0.0061655;
		start[11] = 0.0086181;
		start[12] = 0.0084101;
		start[13] = 0.0048235;
		start[14] = 0.0046955;
		start[15] = 0.0069356;
		start[16] = 0.0075789;
		start[17] = 0.00031255;
		start[18] = 0.0016242;
		start[19] = 0.0067412;
		start[20] = 0.0033163;
		start[21] = 0.004327;
		start[22] = 0.0063382;
		start[23] = 4.6717e-05;
		start[24] = 0.0049694;
		start[25] = 0.0038641;
		start[26] = 0.0053465;
		start[27] = 0.0069507;
		start[28] = 0.0078492;
		start[29] = 0.00061686;
		start[30] = 0.0090349;
		start[31] = 0.0060145;
		start[32] = 0.0071201;
		start[33] = 0.0013513;
		start[34] = 0.0061256;
		start[35] = 0.0079924;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.008068;
		start[7] = 0.00045541;
		start[8] = 0.0029304;
		start[9] = 0.0013214;
		start[10] = 1.8865e-05;
		start[11] = 0.0022079;
		start[12] = 0.0045613;
		start[13] = 0.00028624;
		start[14] = 0.008721;
		start[15] = 0.00078805;
		start[16] = 0.0062232;
		start[17] = 0.0010109;
		start[18] = 0.0043355;
		start[19] = 0.0052733;
		start[20] = 0.0083387;
		start[21] = 0.00078028;
		start[22] = 0.0057432;
		start[23] = 0.0026015;
		start[24] = 0.0088537;
		start[25] = 0.0018993;
		start[26] = 0.00055779;
		start[27] = 0.0050867;
		start[28] = 0.0032065;
		start[29] = 0.0088112;
		start[30] = 0.0092162;
		start[31] = 0.003922;
		start[32] = 0.0027677;
		start[33] = 0.0028604;
		start[34] = 0.0016333;
		start[35] = 0.0054497;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0073104;
		start[7] = 0.0073652;
		start[8] = 0.0081293;
		start[9] = 0.0018212;
		start[10] = 0.003675;
		start[11] = 0.0090135;
		start[12] = 0.0091766;
		start[13] = 0.0072961;
		start[14] = 0.0027624;
		start[15] = 0.0061038;
		start[16] = 0.0093701;
		start[17] = 0.0076702;
		start[18] = 0.0030371;
		start[19] = 0.0053972;
		start[20] = 0.005484;
		start[21] = 0.0065395;
		start[22] = 0.0029226;
		start[23] = 0.0037823;
		start[24] = 0.0079337;
		start[25] = 0.004914;
		start[26] = 0.009371;
		start[27] = 0.0074844;
		start[28] = 0.0057982;
		start[29] = 0.0019914;
		start[30] = 0.0058192;
		start[31] = 0.0053942;
		start[32] = 0.001982;
		start[33] = 0.0078061;
		start[34] = 0.0067042;
		start[35] = 0.00066121;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0006664;
		start[7] = 0.0056343;
		start[8] = 0.0059518;
		start[9] = 0.0071923;
		start[10] = 0.0046406;
		start[11] = 0.0093449;
		start[12] = 0.0062733;
		start[13] = 0.0055595;
		start[14] = 0.0069837;
		start[15] = 0.0025089;
		start[16] = 0.0035765;
		start[17] = 0.0045506;
		start[18] = 0.007874;
		start[19] = 0.0090234;
		start[20] = 0.006622;
		start[21] = 0.003447;
		start[22] = 0.0097919;
		start[23] = 0.0056038;
		start[24] = 0.0034273;
		start[25] = 0.0061807;
		start[26] = 0.0019259;
		start[27] = 0.0068669;
		start[28] = 0.0074552;
		start[29] = 0.0099865;
		start[30] = 0.0018407;
		start[31] = 0.00029354;
		start[32] = 0.0023073;
		start[33] = 0.001116;
		start[34] = 0.0058759;
		start[35] = 0.0025544;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0010056;
		start[7] = 0.005977;
		start[8] = 0.0072515;
		start[9] = 0.0061183;
		start[10] = 0.0069585;
		start[11] = 0.0052148;
		start[12] = 0.0064285;
		start[13] = 0.0064102;
		start[14] = 0.00018309;
		start[15] = 0.0072459;
		start[16] = 0.0034689;
		start[17] = 0.0073479;
		start[18] = 0.0038343;
		start[19] = 0.0041607;
		start[20] = 0.0028887;
		start[21] = 0.0037661;
		start[22] = 0.0066609;
		start[23] = 0.0094092;
		start[24] = 0.00045706;
		start[25] = 0.0011524;
		start[26] = 0.0031051;
		start[27] = 0.001903;
		start[28] = 0.00091963;
		start[29] = 0.00053608;
		start[30] = 0.0044555;
		start[31] = 0.0043019;
		start[32] = 8.744e-05;
		start[33] = 0.0080332;
		start[34] = 0.0025209;
		start[35] = 0.00069615;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0092895;
		start[7] = 0.0077639;
		start[8] = 0.0045648;
		start[9] = 0.0066917;
		start[10] = 0.003057;
		start[11] = 0.0060251;
		start[12] = 0.0066857;
		start[13] = 0.0095723;
		start[14] = 0.0088134;
		start[15] = 0.0002918;
		start[16] = 0.0069296;
		start[17] = 0.0025281;
		start[18] = 0.0004295;
		start[19] = 0.0029573;
		start[20] = 0.00093168;
		start[21] = 0.0069794;
		start[22] = 0.0029047;
		start[23] = 0.00051755;
		start[24] = 0.00042794;
		start[25] = 0.0098341;
		start[26] = 0.00048421;
		start[27] = 0.0044311;
		start[28] = 0.0057037;
		start[29] = 0.0061503;
		start[30] = 0.0034432;
		start[31] = 0.0091013;
		start[32] = 0.0018042;
		start[33] = 0.0030421;
		start[34] = 0.0072943;
		start[35] = 0.00075751;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.00083095;
		start[7] = 0.0072776;
		start[8] = 0.0023347;
		start[9] = 0.0040144;
		start[10] = 0.0091029;
		start[11] = 0.00055897;
		start[12] = 0.0023776;
		start[13] = 0.0011677;
		start[14] = 0.0078571;
		start[15] = 0.0074927;
		start[16] = 0.0035419;
		start[17] = 0.0091415;
		start[18] = 0.0095814;
		start[19] = 0.0078653;
		start[20] = 0.0029651;
		start[21] = 0.0063092;
		start[22] = 0.0026913;
		start[23] = 0.0016668;
		start[24] = 0.0052707;
		start[25] = 0.0030178;
		start[26] = 0.0050311;
		start[27] = 0.001033;
		start[28] = 0.0055085;
		start[29] = 6.1408e-05;
		start[30] = 0.0030555;
		start[31] = 0.0096266;
		start[32] = 0.0045264;
		start[33] = 0.0043199;
		start[34] = 0.0067683;
		start[35] = 0.0025261;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0033069;
		start[7] = 0.00018825;
		start[8] = 0.009246;
		start[9] = 0.0038306;
		start[10] = 0.00435;
		start[11] = 0.0039599;
		start[12] = 2.4585e-06;
		start[13] = 0.0059482;
		start[14] = 0.001579;
		start[15] = 0.0097445;
		start[16] = 0.0065733;
		start[17] = 0.0054206;
		start[18] = 0.0034704;
		start[19] = 0.0095391;
		start[20] = 0.0076656;
		start[21] = 0.0083795;
		start[22] = 0.0084386;
		start[23] = 0.0099239;
		start[24] = 0.0077476;
		start[25] = 0.00391;
		start[26] = 0.0014634;
		start[27] = 0.002758;
		start[28] = 0.0073111;
		start[29] = 0.0010797;
		start[30] = 0.00087045;
		start[31] = 0.0014857;
		start[32] = 0.0039102;
		start[33] = 0.0038647;
		start[34] = 0.0078364;
		start[35] = 0.0011451;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0058401;
		start[7] = 0.0017518;
		start[8] = 0.0017845;
		start[9] = 0.0030901;
		start[10] = 0.0091045;
		start[11] = 0.0016443;
		start[12] = 0.00038884;
		start[13] = 0.0056757;
		start[14] = 0.00092;
		start[15] = 0.0021422;
		start[16] = 0.0046042;
		start[17] = 0.0020993;
		start[18] = 0.0014412;
		start[19] = 0.0019608;
		start[20] = 1.0056e-05;
		start[21] = 0.0065465;
		start[22] = 0.0075818;
		start[23] = 0.0034876;
		start[24] = 0.004048;
		start[25] = 0.0032486;
		start[26] = 0.0047062;
		start[27] = 0.0012779;
		start[28] = 0.0018605;
		start[29] = 0.0091909;
		start[30] = 0.0087519;
		start[31] = 0.0062019;
		start[32] = 0.0048815;
		start[33] = 0.0065018;
		start[34] = 0.0036469;
		start[35] = 0.0079723;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0090011;
		start[7] = 0.0068658;
		start[8] = 0.0060991;
		start[9] = 0.0073637;
		start[10] = 0.0076799;
		start[11] = 0.0078052;
		start[12] = 0.0087429;
		start[13] = 0.0041836;
		start[14] = 0.0039066;
		start[15] = 0.008699;
		start[16] = 0.00040551;
		start[17] = 0.0089515;
		start[18] = 0.0098675;
		start[19] = 0.00072311;
		start[20] = 0.0032821;
		start[21] = 0.0055535;
		start[22] = 0.0020736;
		start[23] = 0.0061089;
		start[24] = 0.0098092;
		start[25] = 0.0096416;
		start[26] = 0.0031654;
		start[27] = 0.0097942;
		start[28] = 0.0025552;
		start[29] = 0.0044631;
		start[30] = 0.0093766;
		start[31] = 0.0033225;
		start[32] = 0.0051973;
		start[33] = 0.0061145;
		start[34] = 0.0033898;
		start[35] = 0.0059164;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0022316;
		start[7] = 0.0034878;
		start[8] = 0.0080232;
		start[9] = 0.003918;
		start[10] = 0.0064165;
		start[11] = 0.003928;
		start[12] = 0.0014442;
		start[13] = 0.0038825;
		start[14] = 0.0098156;
		start[15] = 0.009295;
		start[16] = 0.0069464;
		start[17] = 0.0053048;
		start[18] = 0.0068886;
		start[19] = 0.00068939;
		start[20] = 0.0097046;
		start[21] = 0.0080645;
		start[22] = 0.0079656;
		start[23] = 0.0094106;
		start[24] = 0.0093261;
		start[25] = 0.0084706;
		start[26] = 0.00052009;
		start[27] = 0.00087291;
		start[28] = 0.0098999;
		start[29] = 0.0093612;
		start[30] = 0.0069497;
		start[31] = 0.007694;
		start[32] = 0.0053744;
		start[33] = 0.0077006;
		start[34] = 0.0086476;
		start[35] = 0.0073948;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0091778;
		start[7] = 0.00053842;
		start[8] = 0.0092829;
		start[9] = 0.0088965;
		start[10] = 0.0040796;
		start[11] = 0.0087943;
		start[12] = 0.0077918;
		start[13] = 0.009657;
		start[14] = 0.005054;
		start[15] = 0.0042238;
		start[16] = 0.0064135;
		start[17] = 0.0091004;
		start[18] = 0.0032686;
		start[19] = 0.0034115;
		start[20] = 0.009813;
		start[21] = 0.0060948;
		start[22] = 0.0088887;
		start[23] = 0.0035284;
		start[24] = 0.0037301;
		start[25] = 0.0044788;
		start[26] = 0.0045804;
		start[27] = 0.0032307;
		start[28] = 0.0027147;
		start[29] = 0.00070685;
		start[30] = 0.0060915;
		start[31] = 0.0085966;
		start[32] = 0.0029494;
		start[33] = 0.0098373;
		start[34] = 0.0039607;
		start[35] = 0.0084906;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0032613;
		start[7] = 0.0096799;
		start[8] = 0.0061332;
		start[9] = 0.0092079;
		start[10] = 0.0036595;
		start[11] = 0.00016873;
		start[12] = 0.0051264;
		start[13] = 0.008913;
		start[14] = 0.00094776;
		start[15] = 0.0052344;
		start[16] = 0.0058714;
		start[17] = 0.0039866;
		start[18] = 0.0059317;
		start[19] = 0.0029028;
		start[20] = 0.0047676;
		start[21] = 0.001115;
		start[22] = 0.001233;
		start[23] = 0.007774;
		start[24] = 0.0065562;
		start[25] = 0.0061165;
		start[26] = 0.0027064;
		start[27] = 0.0047482;
		start[28] = 0.0023637;
		start[29] = 0.0010207;
		start[30] = 0.0094754;
		start[31] = 0.0083192;
		start[32] = 0.0044215;
		start[33] = 0.0048752;
		start[34] = 0.0023406;
		start[35] = 0.0050735;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.008343;
		start[7] = 0.00064176;
		start[8] = 0.0092079;
		start[9] = 0.0076275;
		start[10] = 0.002116;
		start[11] = 0.0075659;
		start[12] = 0.0099076;
		start[13] = 0.006517;
		start[14] = 0.003874;
		start[15] = 0.0039506;
		start[16] = 0.0090644;
		start[17] = 0.0058933;
		start[18] = 0.0092781;
		start[19] = 0.0023235;
		start[20] = 0.004345;
		start[21] = 0.0049495;
		start[22] = 0.0036757;
		start[23] = 0.0065082;
		start[24] = 0.0021677;
		start[25] = 0.0083446;
		start[26] = 0.0013721;
		start[27] = 0.00072385;
		start[28] = 0.0011344;
		start[29] = 0.0055763;
		start[30] = 0.0029965;
		start[31] = 0.00010579;
		start[32] = 0.0015366;
		start[33] = 0.0057186;
		start[34] = 0.0010991;
		start[35] = 0.0071477;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0082594;
		start[7] = 0.0065722;
		start[8] = 0.0026014;
		start[9] = 0.0099683;
		start[10] = 0.009351;
		start[11] = 0.007821;
		start[12] = 0.0059217;
		start[13] = 0.0029372;
		start[14] = 0.0060998;
		start[15] = 0.00091129;
		start[16] = 0.0026165;
		start[17] = 0.0045082;
		start[18] = 0.0025644;
		start[19] = 0.0022641;
		start[20] = 0.0001329;
		start[21] = 0.0020384;
		start[22] = 0.0054893;
		start[23] = 0.0090337;
		start[24] = 0.0068381;
		start[25] = 0.0035558;
		start[26] = 0.0026375;
		start[27] = 0.0085525;
		start[28] = 0.0013126;
		start[29] = 0.0032118;
		start[30] = 0.0022407;
		start[31] = 0.0022089;
		start[32] = 0.0085181;
		start[33] = 0.0010946;
		start[34] = 0.0072265;
		start[35] = 0.0098524;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0042271;
		start[7] = 0.0064765;
		start[8] = 0.0095199;
		start[9] = 0.0019906;
		start[10] = 0.0036192;
		start[11] = 0.0036517;
		start[12] = 0.0018887;
		start[13] = 0.0024399;
		start[14] = 0.00506;
		start[15] = 0.0073081;
		start[16] = 0.0043395;
		start[17] = 0.0039656;
		start[18] = 0.003037;
		start[19] = 0.004393;
		start[20] = 0.0013498;
		start[21] = 0.0018425;
		start[22] = 0.0055516;
		start[23] = 0.0026396;
		start[24] = 0.004839;
		start[25] = 0.0008947;
		start[26] = 0.0093483;
		start[27] = 0.0020677;
		start[28] = 0.0032234;
		start[29] = 0.0037925;
		start[30] = 0.0059063;
		start[31] = 0.008404;
		start[32] = 0.0047846;
		start[33] = 0.0054287;
		start[34] = 0.0096206;
		start[35] = 0.0025816;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0026691;
		start[7] = 0.0085185;
		start[8] = 0.00025512;
		start[9] = 0.0053143;
		start[10] = 0.0019313;
		start[11] = 0.0029248;
		start[12] = 0.009906;
		start[13] = 0.0067756;
		start[14] = 0.0068593;
		start[15] = 0.0087065;
		start[16] = 0.0025287;
		start[17] = 0.0002662;
		start[18] = 0.0089545;
		start[19] = 0.0039956;
		start[20] = 0.0031312;
		start[21] = 0.0070631;
		start[22] = 0.0082989;
		start[23] = 0.0045479;
		start[24] = 0.0065706;
		start[25] = 0.0036643;
		start[26] = 0.0099668;
		start[27] = 0.0019644;
		start[28] = 0.0064888;
		start[29] = 0.00055702;
		start[30] = 0.0030682;
		start[31] = 0.0036483;
		start[32] = 0.004577;
		start[33] = 0.001276;
		start[34] = 0.0084827;
		start[35] = 0.0093691;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0056703;
		start[7] = 0.0028067;
		start[8] = 0.0034256;
		start[9] = 0.0060528;
		start[10] = 0.0040226;
		start[11] = 0.0039095;
		start[12] = 0.004522;
		start[13] = 0.006817;
		start[14] = 0.002158;
		start[15] = 0.0034206;
		start[16] = 0.0069714;
		start[17] = 0.0092345;
		start[18] = 0.0068651;
		start[19] = 0.0063667;
		start[20] = 0.0071905;
		start[21] = 0.0036958;
		start[22] = 0.0062925;
		start[23] = 0.0066757;
		start[24] = 0.00099772;
		start[25] = 0.0021915;
		start[26] = 0.0016185;
		start[27] = 0.0020109;
		start[28] = 0.0085525;
		start[29] = 0.0055659;
		start[30] = 0.0043752;
		start[31] = 0.0044448;
		start[32] = 0.001368;
		start[33] = 0.0015272;
		start[34] = 0.007724;
		start[35] = 0.0097844;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0042869;
		start[7] = 0.0050195;
		start[8] = 0.0099804;
		start[9] = 0.0039799;
		start[10] = 0.0071442;
		start[11] = 0.0058386;
		start[12] = 0.0093357;
		start[13] = 0.0068268;
		start[14] = 0.0092919;
		start[15] = 0.0020053;
		start[16] = 0.0094604;
		start[17] = 0.0042924;
		start[18] = 0.0021404;
		start[19] = 0.002993;
		start[20] = 0.00082233;
		start[21] = 0.0023865;
		start[22] = 0.0030036;
		start[23] = 0.005309;
		start[24] = 0.0010708;
		start[25] = 0.0046947;
		start[26] = 0.0063086;
		start[27] = 0.0085789;
		start[28] = 0.0074723;
		start[29] = 0.0016344;
		start[30] = 0.00034329;
		start[31] = 0.0034635;
		start[32] = 0.0081255;
		start[33] = 0.0096992;
		start[34] = 0.0049947;
		start[35] = 0.0039275;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0050587;
		start[7] = 0.0071648;
		start[8] = 0.0092515;
		start[9] = 0.00063508;
		start[10] = 0.0037395;
		start[11] = 0.0072023;
		start[12] = 0.0042193;
		start[13] = 0.0037185;
		start[14] = 0.0015993;
		start[15] = 0.006378;
		start[16] = 0.0031815;
		start[17] = 0.0032849;
		start[18] = 0.0012619;
		start[19] = 0.0026169;
		start[20] = 0.0091947;
		start[21] = 0.0027462;
		start[22] = 0.0072336;
		start[23] = 0.0083571;
		start[24] = 0.0015019;
		start[25] = 0.00014208;
		start[26] = 0.0082357;
		start[27] = 0.005438;
		start[28] = 0.0092705;
		start[29] = 0.00080407;
		start[30] = 0.003577;
		start[31] = 0.0020701;
		start[32] = 0.0064434;
		start[33] = 0.0059488;
		start[34] = 0.0089452;
		start[35] = 0.0042345;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0028989;
		start[7] = 0.0053072;
		start[8] = 0.0047859;
		start[9] = 0.0024472;
		start[10] = 0.0095179;
		start[11] = 0.0020846;
		start[12] = 0.0052999;
		start[13] = 0.0070676;
		start[14] = 0.00070899;
		start[15] = 0.0052266;
		start[16] = 0.00068735;
		start[17] = 0.0057843;
		start[18] = 0.0076104;
		start[19] = 0.008913;
		start[20] = 0.0088832;
		start[21] = 0.0066631;
		start[22] = 0.0041318;
		start[23] = 0.0037937;
		start[24] = 0.0058848;
		start[25] = 0.0030847;
		start[26] = 0.0068322;
		start[27] = 0.00013122;
		start[28] = 0.0014201;
		start[29] = 0.0033752;
		start[30] = 0.00049788;
		start[31] = 0.0066341;
		start[32] = 0.0054902;
		start[33] = 0.0095583;
		start[34] = 0.0012675;
		start[35] = 0.00079585;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0036876;
		start[7] = 0.0056443;
		start[8] = 0.0018839;
		start[9] = 0.0042567;
		start[10] = 0.0020794;
		start[11] = 0.0039084;
		start[12] = 0.0045998;
		start[13] = 0.0098601;
		start[14] = 0.00072079;
		start[15] = 0.0075451;
		start[16] = 0.0033833;
		start[17] = 0.0083833;
		start[18] = 0.0039602;
		start[19] = 0.0011875;
		start[20] = 0.00017077;
		start[21] = 0.0033475;
		start[22] = 0.005484;
		start[23] = 0.0076336;
		start[24] = 0.0074298;
		start[25] = 0.0041919;
		start[26] = 0.0044661;
		start[27] = 0.0057499;
		start[28] = 0.0002902;
		start[29] = 7.17e-05;
		start[30] = 0.0047018;
		start[31] = 0.0010612;
		start[32] = 0.0096543;
		start[33] = 0.0093328;
		start[34] = 0.005319;
		start[35] = 0.0022565;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.007483;
		start[7] = 0.0087909;
		start[8] = 0.0022294;
		start[9] = 0.0041866;
		start[10] = 0.0049804;
		start[11] = 0.0064671;
		start[12] = 0.0092453;
		start[13] = 0.0034451;
		start[14] = 0.00090077;
		start[15] = 0.0039009;
		start[16] = 0.0020839;
		start[17] = 0.0088916;
		start[18] = 0.0097643;
		start[19] = 0.0083129;
		start[20] = 0.0043479;
		start[21] = 0.0078849;
		start[22] = 0.0040071;
		start[23] = 0.0078228;
		start[24] = 0.0088911;
		start[25] = 0.0032791;
		start[26] = 9.6413e-05;
		start[27] = 0.0015001;
		start[28] = 0.0071971;
		start[29] = 0.0048514;
		start[30] = 0.0053385;
		start[31] = 0.0087301;
		start[32] = 0.0007901;
		start[33] = 7.0071e-05;
		start[34] = 0.0063903;
		start[35] = 0.0084169;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0066983;
		start[7] = 0.0020275;
		start[8] = 0.0066369;
		start[9] = 0.0027714;
		start[10] = 0.0020444;
		start[11] = 0.0052132;
		start[12] = 0.0012808;
		start[13] = 0.0043102;
		start[14] = 0.0077234;
		start[15] = 0.0044492;
		start[16] = 0.0045656;
		start[17] = 0.0055576;
		start[18] = 0.0086779;
		start[19] = 0.0057946;
		start[20] = 0.0038815;
		start[21] = 0.0066483;
		start[22] = 0.0017089;
		start[23] = 0.0007124;
		start[24] = 0.0065103;
		start[25] = 0.0041859;
		start[26] = 0.004142;
		start[27] = 0.0072653;
		start[28] = 0.0038489;
		start[29] = 0.0032735;
		start[30] = 0.0057397;
		start[31] = 0.0049692;
		start[32] = 0.0015154;
		start[33] = 0.0026666;
		start[34] = 0.0023288;
		start[35] = 0.0048623;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0061758;
		start[7] = 0.0043365;
		start[8] = 0.0085313;
		start[9] = 0.0014835;
		start[10] = 0.0093653;
		start[11] = 0.0033655;
		start[12] = 0.005177;
		start[13] = 0.0096915;
		start[14] = 0.0033026;
		start[15] = 0.0099839;
		start[16] = 0.00023896;
		start[17] = 0.0083785;
		start[18] = 0.0088453;
		start[19] = 0.0047868;
		start[20] = 0.0075912;
		start[21] = 0.0018346;
		start[22] = 0.0041821;
		start[23] = 0.0045684;
		start[24] = 0.0098118;
		start[25] = 0.0086515;
		start[26] = 0.0082829;
		start[27] = 0.0064826;
		start[28] = 0.0091178;
		start[29] = 6.9441e-05;
		start[30] = 0.0027826;
		start[31] = 3.0292e-05;
		start[32] = 0.0013916;
		start[33] = 0.0054129;
		start[34] = 0.0017687;
		start[35] = 0.00065356;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0079566;
		start[7] = 0.006471;
		start[8] = 0.0092545;
		start[9] = 0.002578;
		start[10] = 0.0012506;
		start[11] = 0.0011217;
		start[12] = 0.00040601;
		start[13] = 0.0065637;
		start[14] = 0.0025259;
		start[15] = 0.0055573;
		start[16] = 0.0025516;
		start[17] = 0.0058715;
		start[18] = 0.0060085;
		start[19] = 0.00048108;
		start[20] = 0.0087601;
		start[21] = 0.0059865;
		start[22] = 0.0057375;
		start[23] = 0.0016953;
		start[24] = 0.00012393;
		start[25] = 0.0057181;
		start[26] = 0.0061419;
		start[27] = 0.0049551;
		start[28] = 0.0063708;
		start[29] = 0.0094564;
		start[30] = 0.00015431;
		start[31] = 0.001258;
		start[32] = 0.0097194;
		start[33] = 0.0020706;
		start[34] = 0.007797;
		start[35] = 0.0085715;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0018877;
		start[7] = 0.0043069;
		start[8] = 0.0025302;
		start[9] = 0.0041148;
		start[10] = 0.0073425;
		start[11] = 0.0066562;
		start[12] = 8.2428e-05;
		start[13] = 0.0070392;
		start[14] = 0.0023684;
		start[15] = 0.0064386;
		start[16] = 0.0035525;
		start[17] = 0.0043122;
		start[18] = 0.0082667;
		start[19] = 0.0016335;
		start[20] = 0.003366;
		start[21] = 0.007579;
		start[22] = 0.0072102;
		start[23] = 0.00041414;
		start[24] = 0.0015166;
		start[25] = 0.0037779;
		start[26] = 0.0034696;
		start[27] = 0.0033787;
		start[28] = 0.0035076;
		start[29] = 0.0085028;
		start[30] = 0.0086572;
		start[31] = 0.0087411;
		start[32] = 0.0025172;
		start[33] = 0.0045035;
		start[34] = 0.0024584;
		start[35] = 0.0057068;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0035171;
		start[7] = 0.00027113;
		start[8] = 0.00045015;
		start[9] = 0.00030698;
		start[10] = 0.0076084;
		start[11] = 0.00037548;
		start[12] = 0.0054832;
		start[13] = 0.0099665;
		start[14] = 0.0042837;
		start[15] = 0.0028081;
		start[16] = 0.0039465;
		start[17] = 0.0085199;
		start[18] = 0.0079304;
		start[19] = 0.0084535;
		start[20] = 0.0018276;
		start[21] = 0.0051507;
		start[22] = 0.004753;
		start[23] = 0.0079967;
		start[24] = 0.0060926;
		start[25] = 0.0084892;
		start[26] = 0.0078201;
		start[27] = 0.0020405;
		start[28] = 0.0024855;
		start[29] = 0.0076134;
		start[30] = 0.0079392;
		start[31] = 0.0094032;
		start[32] = 0.0073688;
		start[33] = 0.0057863;
		start[34] = 0.0035495;
		start[35] = 0.00076613;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0032044;
		start[7] = 0.0056385;
		start[8] = 0.0059121;
		start[9] = 0.00092952;
		start[10] = 0.0061;
		start[11] = 0.0083997;
		start[12] = 0.0043313;
		start[13] = 0.0053371;
		start[14] = 0.0025333;
		start[15] = 0.0094361;
		start[16] = 0.0086043;
		start[17] = 0.0035001;
		start[18] = 0.0046895;
		start[19] = 0.0010439;
		start[20] = 0.0072576;
		start[21] = 0.0010392;
		start[22] = 0.0081156;
		start[23] = 0.0063309;
		start[24] = 0.0014907;
		start[25] = 0.0019114;
		start[26] = 0.0016553;
		start[27] = 0.0084969;
		start[28] = 0.0035791;
		start[29] = 0.0014124;
		start[30] = 0.0022053;
		start[31] = 0.0043759;
		start[32] = 0.001926;
		start[33] = 0.0047168;
		start[34] = 0.0050549;
		start[35] = 0.0098483;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0044088;
		start[7] = 0.0071021;
		start[8] = 0.0093817;
		start[9] = 0.0019442;
		start[10] = 0.0017045;
		start[11] = 0.0065556;
		start[12] = 0.0071032;
		start[13] = 0.00027709;
		start[14] = 0.0086604;
		start[15] = 0.0032284;
		start[16] = 0.006893;
		start[17] = 0.0049633;
		start[18] = 0.0079466;
		start[19] = 0.0091024;
		start[20] = 0.0076778;
		start[21] = 0.00087508;
		start[22] = 0.0033124;
		start[23] = 0.0043894;
		start[24] = 0.0062302;
		start[25] = 0.0029941;
		start[26] = 0.00081659;
		start[27] = 0.0086757;
		start[28] = 0.0073431;
		start[29] = 0.00023972;
		start[30] = 0.0076018;
		start[31] = 0.00632;
		start[32] = 0.0062571;
		start[33] = 0.0072664;
		start[34] = 0.0059916;
		start[35] = 0.0097603;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0084977;
		start[7] = 0.0086062;
		start[8] = 0.0099495;
		start[9] = 0.008881;
		start[10] = 0.0067409;
		start[11] = 0.0062379;
		start[12] = 0.0091563;
		start[13] = 0.0090724;
		start[14] = 0.0070667;
		start[15] = 0.0045046;
		start[16] = 0.0048376;
		start[17] = 0.0034149;
		start[18] = 0.0059319;
		start[19] = 0.0022921;
		start[20] = 0.0094768;
		start[21] = 0.0019762;
		start[22] = 0.00018784;
		start[23] = 0.0051996;
		start[24] = 0.0030153;
		start[25] = 0.0088815;
		start[26] = 0.0072325;
		start[27] = 0.0099428;
		start[28] = 0.0066212;
		start[29] = 0.006691;
		start[30] = 0.0016741;
		start[31] = 0.0069423;
		start[32] = 0.0021607;
		start[33] = 0.0057265;
		start[34] = 0.0028557;
		start[35] = 0.0082281;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.00029331;
		start[7] = 0.0042906;
		start[8] = 0.0088993;
		start[9] = 0.0046978;
		start[10] = 0.0020747;
		start[11] = 0.0035307;
		start[12] = 0.008508;
		start[13] = 0.0013236;
		start[14] = 0.0071204;
		start[15] = 0.0087303;
		start[16] = 0.0012027;
		start[17] = 0.0096367;
		start[18] = 0.0012812;
		start[19] = 0.0029291;
		start[20] = 0.008244;
		start[21] = 0.0084354;
		start[22] = 0.0078428;
		start[23] = 0.006031;
		start[24] = 0.006026;
		start[25] = 0.0029662;
		start[26] = 0.0092688;
		start[27] = 0.0085275;
		start[28] = 0.0076301;
		start[29] = 5.7678e-05;
		start[30] = 2.5236e-05;
		start[31] = 0.0086817;
		start[32] = 0.00081403;
		start[33] = 0.0073957;
		start[34] = 0.006983;
		start[35] = 0.0020654;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0072149;
		start[7] = 0.0041376;
		start[8] = 0.0041311;
		start[9] = 0.0084749;
		start[10] = 0.0023681;
		start[11] = 0.0019424;
		start[12] = 0.0070922;
		start[13] = 0.0081222;
		start[14] = 0.0070166;
		start[15] = 0.003804;
		start[16] = 0.0016551;
		start[17] = 0.0024966;
		start[18] = 0.009031;
		start[19] = 0.0044741;
		start[20] = 0.0036764;
		start[21] = 0.0078147;
		start[22] = 0.005456;
		start[23] = 0.0041257;
		start[24] = 0.0045072;
		start[25] = 0.0074722;
		start[26] = 0.00018143;
		start[27] = 0.0053106;
		start[28] = 0.0040103;
		start[29] = 0.0024469;
		start[30] = 0.0015917;
		start[31] = 0.0046216;
		start[32] = 0.0073826;
		start[33] = 0.0092433;
		start[34] = 0.0091187;
		start[35] = 0.00024079;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0087543;
		start[7] = 0.0060622;
		start[8] = 0.0045211;
		start[9] = 0.0031939;
		start[10] = 0.00084531;
		start[11] = 0.0092666;
		start[12] = 0.0042509;
		start[13] = 0.0053525;
		start[14] = 0.0024293;
		start[15] = 0.0055434;
		start[16] = 0.0073537;
		start[17] = 0.0093108;
		start[18] = 0.0085858;
		start[19] = 0.0085915;
		start[20] = 0.0075285;
		start[21] = 0.0087912;
		start[22] = 0.0033443;
		start[23] = 0.0086609;
		start[24] = 0.0024769;
		start[25] = 0.008721;
		start[26] = 0.003792;
		start[27] = 0.0090035;
		start[28] = 0.0037418;
		start[29] = 0.00077421;
		start[30] = 0.0022854;
		start[31] = 0.0087912;
		start[32] = 0.0020104;
		start[33] = 0.0092973;
		start[34] = 0.0041371;
		start[35] = 0.0012162;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0046562;
		start[7] = 0.0075387;
		start[8] = 0.0062023;
		start[9] = 0.0062465;
		start[10] = 0.0088381;
		start[11] = 0.0095374;
		start[12] = 0.0077074;
		start[13] = 0.0067237;
		start[14] = 0.0096397;
		start[15] = 0.0010161;
		start[16] = 0.0071047;
		start[17] = 0.003175;
		start[18] = 0.0063818;
		start[19] = 0.0021194;
		start[20] = 0.0045414;
		start[21] = 0.005029;
		start[22] = 0.0072089;
		start[23] = 0.0054005;
		start[24] = 0.0015411;
		start[25] = 0.0012192;
		start[26] = 0.0053487;
		start[27] = 0.0049657;
		start[28] = 0.0029403;
		start[29] = 0.0083939;
		start[30] = 0.0011061;
		start[31] = 0.0016033;
		start[32] = 0.0051855;
		start[33] = 0.0015379;
		start[34] = 0.0023222;
		start[35] = 0.0090692;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0034684;
		start[7] = 0.0054264;
		start[8] = 0.0059261;
		start[9] = 0.003106;
		start[10] = 0.001833;
		start[11] = 4.8829e-05;
		start[12] = 0.00736;
		start[13] = 0.009071;
		start[14] = 0.0066159;
		start[15] = 0.0013279;
		start[16] = 0.0074561;
		start[17] = 0.0080324;
		start[18] = 0.0079986;
		start[19] = 0.0077475;
		start[20] = 0.0017865;
		start[21] = 0.004432;
		start[22] = 0.0081295;
		start[23] = 0.0036394;
		start[24] = 0.0080468;
		start[25] = 0.0081017;
		start[26] = 0.0086371;
		start[27] = 0.0018736;
		start[28] = 0.0056473;
		start[29] = 0.0049359;
		start[30] = 0.0062251;
		start[31] = 0.0080995;
		start[32] = 0.0074688;
		start[33] = 0.004196;
		start[34] = 0.0041338;
		start[35] = 0.0057731;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0046332;
		start[7] = 0.005902;
		start[8] = 0.0036493;
		start[9] = 0.0053601;
		start[10] = 0.00074857;
		start[11] = 0.0035777;
		start[12] = 0.0062922;
		start[13] = 0.0050167;
		start[14] = 0.0059716;
		start[15] = 0.0062385;
		start[16] = 0.0077751;
		start[17] = 0.0053831;
		start[18] = 0.00090848;
		start[19] = 0.0024964;
		start[20] = 0.0071021;
		start[21] = 0.0069102;
		start[22] = 0.0022381;
		start[23] = 0.0015516;
		start[24] = 0.0065265;
		start[25] = 0.0073843;
		start[26] = 0.0050365;
		start[27] = 0.00089088;
		start[28] = 0.003852;
		start[29] = 0.0066325;
		start[30] = 0.0058297;
		start[31] = 0.0072585;
		start[32] = 0.0030066;
		start[33] = 0.0097642;
		start[34] = 1.7562e-05;
		start[35] = 0.0096744;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0015399;
		start[7] = 0.0059479;
		start[8] = 0.0012204;
		start[9] = 6.4953e-05;
		start[10] = 0.0020838;
		start[11] = 0.0052978;
		start[12] = 0.0042849;
		start[13] = 0.008391;
		start[14] = 0.007403;
		start[15] = 0.0045134;
		start[16] = 0.0025234;
		start[17] = 0.0083797;
		start[18] = 0.002993;
		start[19] = 0.0046899;
		start[20] = 0.009154;
		start[21] = 0.0094648;
		start[22] = 0.004016;
		start[23] = 0.0064695;
		start[24] = 0.0064755;
		start[25] = 0.0098056;
		start[26] = 0.0029336;
		start[27] = 0.0085389;
		start[28] = 0.0021718;
		start[29] = 0.0047182;
		start[30] = 0.0028908;
		start[31] = 0.0014367;
		start[32] = 0.0070497;
		start[33] = 0.0048351;
		start[34] = 0.0026411;
		start[35] = 0.0067271;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.00069419;
		start[7] = 0.0066928;
		start[8] = 0.002325;
		start[9] = 0.0021725;
		start[10] = 0.0096241;
		start[11] = 0.0090942;
		start[12] = 0.0098316;
		start[13] = 0.0084857;
		start[14] = 0.0055915;
		start[15] = 0.0097802;
		start[16] = 0.0090107;
		start[17] = 0.0010828;
		start[18] = 0.0051537;
		start[19] = 0.0056739;
		start[20] = 0.0065552;
		start[21] = 0.0034519;
		start[22] = 0.0088694;
		start[23] = 0.0092248;
		start[24] = 0.00025688;
		start[25] = 0.00071846;
		start[26] = 0.0048187;
		start[27] = 0.00081926;
		start[28] = 0.0082733;
		start[29] = 0.009571;
		start[30] = 0.0094942;
		start[31] = 0.0069463;
		start[32] = 0.0074027;
		start[33] = 0.0030255;
		start[34] = 0.0067678;
		start[35] = 0.0090218;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.00030691;
		start[7] = 0.0054554;
		start[8] = 0.0093485;
		start[9] = 0.0042217;
		start[10] = 0.0097216;
		start[11] = 0.0098131;
		start[12] = 0.00051968;
		start[13] = 0.004919;
		start[14] = 0.0063831;
		start[15] = 0.0086055;
		start[16] = 0.00045645;
		start[17] = 0.00059365;
		start[18] = 0.007512;
		start[19] = 0.0014463;
		start[20] = 0.0061076;
		start[21] = 0.0051267;
		start[22] = 0.0020765;
		start[23] = 0.0011266;
		start[24] = 0.0025146;
		start[25] = 0.0010989;
		start[26] = 0.0099905;
		start[27] = 0.0045492;
		start[28] = 0.0087942;
		start[29] = 0.003209;
		start[30] = 0.0053;
		start[31] = 0.007178;
		start[32] = 0.0016924;
		start[33] = 0.0015481;
		start[34] = 0.0050257;
		start[35] = 0.00068495;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0090532;
		start[7] = 0.0058003;
		start[8] = 0.0073236;
		start[9] = 0.0033476;
		start[10] = 0.0039808;
		start[11] = 0.0061673;
		start[12] = 0.0041379;
		start[13] = 0.0067249;
		start[14] = 0.0091125;
		start[15] = 0.0088816;
		start[16] = 0.0013517;
		start[17] = 0.0074175;
		start[18] = 0.0025453;
		start[19] = 0.00369;
		start[20] = 0.0053141;
		start[21] = 0.0065776;
		start[22] = 0.0035725;
		start[23] = 0.0052819;
		start[24] = 0.0030772;
		start[25] = 0.0063676;
		start[26] = 9.5012e-05;
		start[27] = 0.002685;
		start[28] = 0.0033577;
		start[29] = 0.00092343;
		start[30] = 0.0095256;
		start[31] = 0.0089491;
		start[32] = 0.0094928;
		start[33] = 0.0017865;
		start[34] = 0.0024034;
		start[35] = 0.0063079;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0057159;
		start[7] = 0.0080627;
		start[8] = 0.0012843;
		start[9] = 0.0058473;
		start[10] = 0.0019102;
		start[11] = 0.0034836;
		start[12] = 0.0067219;
		start[13] = 0.0067626;
		start[14] = 0.0098189;
		start[15] = 0.0037992;
		start[16] = 0.0031081;
		start[17] = 0.0091034;
		start[18] = 0.0028081;
		start[19] = 0.0023574;
		start[20] = 0.0084874;
		start[21] = 0.0049359;
		start[22] = 0.008667;
		start[23] = 0.005072;
		start[24] = 0.009248;
		start[25] = 8.6502e-05;
		start[26] = 0.00040493;
		start[27] = 0.0079515;
		start[28] = 0.0058111;
		start[29] = 0.0040762;
		start[30] = 0.00041185;
		start[31] = 0.004402;
		start[32] = 0.0030547;
		start[33] = 0.0075046;
		start[34] = 0.00014127;
		start[35] = 0.0072261;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0014337;
		start[7] = 0.0079681;
		start[8] = 0.0066602;
		start[9] = 0.0046429;
		start[10] = 0.0039394;
		start[11] = 0.0076625;
		start[12] = 0.001199;
		start[13] = 0.0038999;
		start[14] = 0.0021702;
		start[15] = 0.0069428;
		start[16] = 0.002445;
		start[17] = 0.007999;
		start[18] = 0.003045;
		start[19] = 0.0085877;
		start[20] = 0.0070702;
		start[21] = 0.0082343;
		start[22] = 0.0011385;
		start[23] = 0.0062655;
		start[24] = 0.002326;
		start[25] = 0.0006553;
		start[26] = 0.0091598;
		start[27] = 0.0085073;
		start[28] = 0.0083583;
		start[29] = 0.00069452;
		start[30] = 0.0020648;
		start[31] = 0.003438;
		start[32] = 0.0029469;
		start[33] = 0.0059758;
		start[34] = 0.0096916;
		start[35] = 0.0023421;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0044952;
		start[7] = 0.0023864;
		start[8] = 0.0099448;
		start[9] = 0.0079518;
		start[10] = 0.00845;
		start[11] = 0.0078286;
		start[12] = 0.0029315;
		start[13] = 0.0096825;
		start[14] = 0.0098633;
		start[15] = 0.0020328;
		start[16] = 0.0068436;
		start[17] = 0.0043941;
		start[18] = 0.0024837;
		start[19] = 0.004292;
		start[20] = 0.0074154;
		start[21] = 0.00068815;
		start[22] = 0.0090114;
		start[23] = 0.0036936;
		start[24] = 0.0053143;
		start[25] = 0.0088524;
		start[26] = 0.0055807;
		start[27] = 0.0019481;
		start[28] = 0.0046637;
		start[29] = 0.0072107;
		start[30] = 0.0053105;
		start[31] = 0.0039141;
		start[32] = 0.0074157;
		start[33] = 0.0039828;
		start[34] = 0.0028637;
		start[35] = 0.0078277;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0023726;
		start[7] = 0.0098588;
		start[8] = 0.0068018;
		start[9] = 0.0057198;
		start[10] = 0.00042979;
		start[11] = 0.0015569;
		start[12] = 0.0052468;
		start[13] = 0.0090374;
		start[14] = 0.0012371;
		start[15] = 0.0032576;
		start[16] = 0.00085868;
		start[17] = 0.0034702;
		start[18] = 0.0010131;
		start[19] = 0.0062372;
		start[20] = 0.0098356;
		start[21] = 0.006475;
		start[22] = 0.0072293;
		start[23] = 0.0029635;
		start[24] = 0.0047983;
		start[25] = 5.8232e-05;
		start[26] = 0.0048132;
		start[27] = 0.0091646;
		start[28] = 0.0072586;
		start[29] = 0.0054053;
		start[30] = 0.0092258;
		start[31] = 0.0096514;
		start[32] = 0.0004167;
		start[33] = 0.0074345;
		start[34] = 0.0082334;
		start[35] = 0.0011846;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0061517;
		start[7] = 0.0010927;
		start[8] = 0.0067109;
		start[9] = 0.0052744;
		start[10] = 0.0083827;
		start[11] = 0.0020197;
		start[12] = 0.0039315;
		start[13] = 0.0068201;
		start[14] = 0.009268;
		start[15] = 0.0037246;
		start[16] = 0.008814;
		start[17] = 0.001567;
		start[18] = 0.0055699;
		start[19] = 0.0095147;
		start[20] = 0.00036557;
		start[21] = 0.0041383;
		start[22] = 0.0052459;
		start[23] = 0.0015502;
		start[24] = 0.0015966;
		start[25] = 0.0093456;
		start[26] = 0.0064936;
		start[27] = 0.001167;
		start[28] = 0.0065234;
		start[29] = 0.0046261;
		start[30] = 0.0078194;
		start[31] = 0.0026129;
		start[32] = 0.0047342;
		start[33] = 0.0023164;
		start[34] = 0.001477;
		start[35] = 0.0022638;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.009427;
		start[7] = 0.0014654;
		start[8] = 0.0050749;
		start[9] = 0.0052055;
		start[10] = 0.0052622;
		start[11] = 0.004089;
		start[12] = 0.0015764;
		start[13] = 0.0067058;
		start[14] = 0.0097526;
		start[15] = 0.0081802;
		start[16] = 0.0087546;
		start[17] = 0.0014325;
		start[18] = 0.0086621;
		start[19] = 0.0063991;
		start[20] = 0.0067149;
		start[21] = 0.0080837;
		start[22] = 0.0082232;
		start[23] = 0.0025866;
		start[24] = 0.0065393;
		start[25] = 0.0086245;
		start[26] = 0.0057465;
		start[27] = 0.0063233;
		start[28] = 0.0030103;
		start[29] = 0.0038538;
		start[30] = 0.0049884;
		start[31] = 0.0021597;
		start[32] = 0.0035239;
		start[33] = 0.0067656;
		start[34] = 0.0092302;
		start[35] = 0.0094735;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0041848;
		start[7] = 0.0099525;
		start[8] = 0.0087947;
		start[9] = 0.0050115;
		start[10] = 0.0060277;
		start[11] = 0.00020783;
		start[12] = 0.0086404;
		start[13] = 0.0015468;
		start[14] = 0.0075562;
		start[15] = 0.0098802;
		start[16] = 0.0069635;
		start[17] = 0.00063841;
		start[18] = 0.006057;
		start[19] = 0.0098761;
		start[20] = 0.0031613;
		start[21] = 0.0024114;
		start[22] = 0.0018863;
		start[23] = 0.0045959;
		start[24] = 0.0021383;
		start[25] = 0.0030593;
		start[26] = 0.0041003;
		start[27] = 0.0037769;
		start[28] = 0.0089151;
		start[29] = 0.0096759;
		start[30] = 0.0029258;
		start[31] = 0.0014095;
		start[32] = 0.0099127;
		start[33] = 0.0079028;
		start[34] = 0.001755;
		start[35] = 0.0014766;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 7.2363e-05;
		start[7] = 0.0014307;
		start[8] = 0.00068115;
		start[9] = 0.0075633;
		start[10] = 0.0066367;
		start[11] = 0.00085209;
		start[12] = 0.0034136;
		start[13] = 0.0033924;
		start[14] = 0.0050073;
		start[15] = 0.00081176;
		start[16] = 0.005423;
		start[17] = 0.0032613;
		start[18] = 0.001095;
		start[19] = 0.0012719;
		start[20] = 0.0040359;
		start[21] = 0.0034056;
		start[22] = 0.0029858;
		start[23] = 0.0086838;
		start[24] = 0.0095138;
		start[25] = 0.0047621;
		start[26] = 0.0054502;
		start[27] = 0.0019536;
		start[28] = 0.0055003;
		start[29] = 0.0016354;
		start[30] = 0.0096054;
		start[31] = 0.0016097;
		start[32] = 0.0050345;
		start[33] = 0.0059841;
		start[34] = 0.0082214;
		start[35] = 0.0038571;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0075647;
		start[7] = 0.0072285;
		start[8] = 0.009757;
		start[9] = 0.0065726;
		start[10] = 0.006843;
		start[11] = 0.0073456;
		start[12] = 0.0024712;
		start[13] = 0.0099028;
		start[14] = 0.0094528;
		start[15] = 0.0055362;
		start[16] = 0.0092536;
		start[17] = 0.0084602;
		start[18] = 0.00058416;
		start[19] = 0.0054253;
		start[20] = 0.0039186;
		start[21] = 0.0067107;
		start[22] = 0.0097466;
		start[23] = 0.0021544;
		start[24] = 0.0027619;
		start[25] = 0.0097689;
		start[26] = 0.0027602;
		start[27] = 0.0048398;
		start[28] = 0.0077568;
		start[29] = 0.0028966;
		start[30] = 0.0085198;
		start[31] = 0.0094576;
		start[32] = 0.0059563;
		start[33] = 0.0066422;
		start[34] = 0.0026908;
		start[35] = 0.00045909;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0086947;
		start[7] = 0.002419;
		start[8] = 0.0066342;
		start[9] = 0.0087191;
		start[10] = 0.0015838;
		start[11] = 0.0061678;
		start[12] = 0.0078571;
		start[13] = 0.0064065;
		start[14] = 0.0033446;
		start[15] = 0.0050516;
		start[16] = 0.0025768;
		start[17] = 0.003818;
		start[18] = 0.006518;
		start[19] = 0.00015225;
		start[20] = 0.0011013;
		start[21] = 0.0030141;
		start[22] = 0.0029383;
		start[23] = 0.0069719;
		start[24] = 0.00045525;
		start[25] = 0.0046723;
		start[26] = 0.0045161;
		start[27] = 0.0051635;
		start[28] = 0.0040916;
		start[29] = 0.0072089;
		start[30] = 0.0071722;
		start[31] = 0.0012884;
		start[32] = 0.0062225;
		start[33] = 0.0010278;
		start[34] = 0.0082552;
		start[35] = 0.0086987;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0093983;
		start[7] = 0.0011642;
		start[8] = 0.0033371;
		start[9] = 0.00073948;
		start[10] = 0.0044905;
		start[11] = 0.0064739;
		start[12] = 0.00027036;
		start[13] = 0.0075327;
		start[14] = 0.0030305;
		start[15] = 0.0041289;
		start[16] = 0.0077104;
		start[17] = 0.0091576;
		start[18] = 0.0040966;
		start[19] = 0.00045643;
		start[20] = 0.0064755;
		start[21] = 0.00096576;
		start[22] = 0.0043158;
		start[23] = 0.0038916;
		start[24] = 0.0093202;
		start[25] = 0.0072842;
		start[26] = 0.0067172;
		start[27] = 0.0013709;
		start[28] = 0.0087073;
		start[29] = 0.00057778;
		start[30] = 0.0026814;
		start[31] = 0.0068802;
		start[32] = 0.0092835;
		start[33] = 0.0038788;
		start[34] = 0.005315;
		start[35] = 0.0041035;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0057001;
		start[7] = 0.0051274;
		start[8] = 0.0058077;
		start[9] = 0.0098622;
		start[10] = 0.0096286;
		start[11] = 0.0029401;
		start[12] = 0.00020005;
		start[13] = 0.0030212;
		start[14] = 0.0086462;
		start[15] = 0.0064766;
		start[16] = 0.0079344;
		start[17] = 0.0060117;
		start[18] = 0.0071171;
		start[19] = 0.0088593;
		start[20] = 0.0050216;
		start[21] = 0.0044988;
		start[22] = 0.0065593;
		start[23] = 0.0072637;
		start[24] = 0.0017566;
		start[25] = 0.0099854;
		start[26] = 0.0013411;
		start[27] = 0.0067368;
		start[28] = 0.0093113;
		start[29] = 0.0042487;
		start[30] = 0.0040553;
		start[31] = 0.0022148;
		start[32] = 0.0094985;
		start[33] = 0.0058733;
		start[34] = 0.0015559;
		start[35] = 0.0012374;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0078246;
		start[7] = 0.0030472;
		start[8] = 0.0072073;
		start[9] = 0.0011831;
		start[10] = 0.0013172;
		start[11] = 0.0099143;
		start[12] = 0.001696;
		start[13] = 0.0070583;
		start[14] = 0.0061637;
		start[15] = 0.0062781;
		start[16] = 0.00019992;
		start[17] = 0.0089027;
		start[18] = 0.00082933;
		start[19] = 0.0088338;
		start[20] = 0.0013321;
		start[21] = 0.0071677;
		start[22] = 0.00021885;
		start[23] = 0.0010291;
		start[24] = 0.0081537;
		start[25] = 0.0059169;
		start[26] = 0.0025619;
		start[27] = 0.00016251;
		start[28] = 0.0086818;
		start[29] = 0.0023891;
		start[30] = 0.0096977;
		start[31] = 0.0052248;
		start[32] = 0.0038638;
		start[33] = 0.008023;
		start[34] = 0.0089489;
		start[35] = 0.00059872;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 2.3029e-05;
		start[7] = 0.0073676;
		start[8] = 0.004685;
		start[9] = 0.0021942;
		start[10] = 0.0068531;
		start[11] = 0.0099637;
		start[12] = 0.0092504;
		start[13] = 0.006579;
		start[14] = 0.0075864;
		start[15] = 0.0052939;
		start[16] = 0.0034047;
		start[17] = 0.00041028;
		start[18] = 0.0061794;
		start[19] = 0.0067299;
		start[20] = 0.0016815;
		start[21] = 0.0061044;
		start[22] = 0.0082201;
		start[23] = 0.0029232;
		start[24] = 0.002086;
		start[25] = 0.0073685;
		start[26] = 0.0098884;
		start[27] = 0.0069799;
		start[28] = 0.00056206;
		start[29] = 0.0080092;
		start[30] = 0.0080584;
		start[31] = 0.0040678;
		start[32] = 0.0080776;
		start[33] = 0.00918;
		start[34] = 0.0045471;
		start[35] = 0.0065509;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0028274;
		start[7] = 0.00028081;
		start[8] = 0.0095725;
		start[9] = 0.0075524;
		start[10] = 0.0061561;
		start[11] = 0.008921;
		start[12] = 0.00080965;
		start[13] = 0.0089546;
		start[14] = 0.0085308;
		start[15] = 0.0059341;
		start[16] = 0.0073739;
		start[17] = 0.003695;
		start[18] = 0.00046495;
		start[19] = 0.0028966;
		start[20] = 0.0035477;
		start[21] = 0.008807;
		start[22] = 0.0020715;
		start[23] = 0.0033297;
		start[24] = 0.0055327;
		start[25] = 0.0031395;
		start[26] = 0.0075599;
		start[27] = 0.009499;
		start[28] = 0.0095264;
		start[29] = 0.009813;
		start[30] = 0.0070699;
		start[31] = 0.006052;
		start[32] = 0.0058357;
		start[33] = 0.0056975;
		start[34] = 0.0047913;
		start[35] = 0.0025699;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0042209;
		start[7] = 0.0051389;
		start[8] = 0.0074954;
		start[9] = 0.0026616;
		start[10] = 0.00088149;
		start[11] = 0.0062661;
		start[12] = 0.0044674;
		start[13] = 0.0050454;
		start[14] = 0.007724;
		start[15] = 0.0089832;
		start[16] = 0.0027689;
		start[17] = 0.00062594;
		start[18] = 0.0092518;
		start[19] = 0.0084465;
		start[20] = 0.0058035;
		start[21] = 0.008759;
		start[22] = 0.0039423;
		start[23] = 0.0026765;
		start[24] = 0.0069659;
		start[25] = 0.0036112;
		start[26] = 0.0076338;
		start[27] = 0.0033171;
		start[28] = 0.0037315;
		start[29] = 0.0015291;
		start[30] = 0.0085614;
		start[31] = 0.0036352;
		start[32] = 0.0029087;
		start[33] = 0.0094429;
		start[34] = 0.00384;
		start[35] = 0.0088562;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0039339;
		start[7] = 0.0034533;
		start[8] = 0.0010206;
		start[9] = 0.0081289;
		start[10] = 0.0013868;
		start[11] = 0.0028634;
		start[12] = 0.00063738;
		start[13] = 0.009844;
		start[14] = 0.00040286;
		start[15] = 0.0021353;
		start[16] = 0.0048226;
		start[17] = 0.0039833;
		start[18] = 0.004463;
		start[19] = 6.5816e-05;
		start[20] = 0.0066076;
		start[21] = 0.0034387;
		start[22] = 0.0019991;
		start[23] = 0.0084246;
		start[24] = 0.0091452;
		start[25] = 0.0050249;
		start[26] = 0.0051597;
		start[27] = 0.0058731;
		start[28] = 0.0040959;
		start[29] = 0.0082103;
		start[30] = 0.0067859;
		start[31] = 0.0087418;
		start[32] = 0.009312;
		start[33] = 0.0081901;
		start[34] = 0.0090314;
		start[35] = 0.0032523;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0017031;
		start[7] = 0.002119;
		start[8] = 0.0069692;
		start[9] = 0.0057333;
		start[10] = 0.00052264;
		start[11] = 0.0022714;
		start[12] = 0.0086755;
		start[13] = 0.0035588;
		start[14] = 0.0069469;
		start[15] = 0.0052466;
		start[16] = 0.0030076;
		start[17] = 0.0044311;
		start[18] = 0.006033;
		start[19] = 0.0046822;
		start[20] = 0.0045933;
		start[21] = 0.0057874;
		start[22] = 0.0042665;
		start[23] = 0.0023542;
		start[24] = 0.0089171;
		start[25] = 0.0098687;
		start[26] = 0.00392;
		start[27] = 0.0087259;
		start[28] = 0.0014993;
		start[29] = 0.0043694;
		start[30] = 0.0029832;
		start[31] = 0.0087689;
		start[32] = 0.0034077;
		start[33] = 0.0043775;
		start[34] = 0.0036513;
		start[35] = 9.6687e-05;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0015077;
		start[7] = 0.0038193;
		start[8] = 0.0084739;
		start[9] = 0.0053419;
		start[10] = 0.0080328;
		start[11] = 0.0017701;
		start[12] = 0.0071247;
		start[13] = 0.006813;
		start[14] = 0.0025374;
		start[15] = 0.0072591;
		start[16] = 0.0016253;
		start[17] = 0.0025302;
		start[18] = 0.0034288;
		start[19] = 0.0095318;
		start[20] = 0.0038106;
		start[21] = 0.0086208;
		start[22] = 0.0057922;
		start[23] = 0.0052966;
		start[24] = 0.0099279;
		start[25] = 0.0039201;
		start[26] = 0.0071691;
		start[27] = 0.00082633;
		start[28] = 0.0007285;
		start[29] = 0.0039673;
		start[30] = 0.0036328;
		start[31] = 0.007763;
		start[32] = 0.0086346;
		start[33] = 0.0020828;
		start[34] = 0.0041098;
		start[35] = 0.0053338;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.00090619;
		start[7] = 0.0086387;
		start[8] = 0.001796;
		start[9] = 0.00084475;
		start[10] = 0.0012296;
		start[11] = 0.0062512;
		start[12] = 0.0059463;
		start[13] = 0.0088902;
		start[14] = 0.0047158;
		start[15] = 0.0065365;
		start[16] = 0.0052585;
		start[17] = 0.0076741;
		start[18] = 0.00088615;
		start[19] = 0.0024089;
		start[20] = 0.0046912;
		start[21] = 0.0047304;
		start[22] = 0.0030687;
		start[23] = 7.1447e-05;
		start[24] = 0.0080159;
		start[25] = 0.0029078;
		start[26] = 0.0066992;
		start[27] = 0.0013709;
		start[28] = 0.00064734;
		start[29] = 0.0041197;
		start[30] = 0.0019852;
		start[31] = 0.0025489;
		start[32] = 0.0079597;
		start[33] = 0.004105;
		start[34] = 0.0087486;
		start[35] = 0.0008998;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.00098212;
		start[7] = 0.0025607;
		start[8] = 0.0004294;
		start[9] = 0.00072984;
		start[10] = 0.003448;
		start[11] = 0.0073005;
		start[12] = 0.0056079;
		start[13] = 0.0066371;
		start[14] = 0.001837;
		start[15] = 0.0035313;
		start[16] = 0.0013957;
		start[17] = 0.0078062;
		start[18] = 0.0053018;
		start[19] = 0.0041093;
		start[20] = 0.0018431;
		start[21] = 0.0091909;
		start[22] = 0.0069;
		start[23] = 0.0081323;
		start[24] = 0.0039727;
		start[25] = 0.0015967;
		start[26] = 0.0089621;
		start[27] = 0.0063716;
		start[28] = 0.0025077;
		start[29] = 0.0073999;
		start[30] = 0.0012211;
		start[31] = 0.0074939;
		start[32] = 0.0058009;
		start[33] = 0.0091088;
		start[34] = 0.0056982;
		start[35] = 0.0038453;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0071688;
		start[7] = 0.0043997;
		start[8] = 0.0080858;
		start[9] = 0.0090765;
		start[10] = 0.0074584;
		start[11] = 0.0062106;
		start[12] = 0.001511;
		start[13] = 0.0018794;
		start[14] = 0.0045442;
		start[15] = 0.0073013;
		start[16] = 0.0080146;
		start[17] = 0.00027364;
		start[18] = 0.0026981;
		start[19] = 0.0046946;
		start[20] = 0.0032691;
		start[21] = 0.0097198;
		start[22] = 0.0005857;
		start[23] = 7.6591e-05;
		start[24] = 0.00089083;
		start[25] = 0.007859;
		start[26] = 0.0014697;
		start[27] = 0.003572;
		start[28] = 0.0033232;
		start[29] = 0.0079195;
		start[30] = 0.0083734;
		start[31] = 0.0067126;
		start[32] = 0.0054204;
		start[33] = 0.0099146;
		start[34] = 0.0015238;
		start[35] = 0.0068469;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0089902;
		start[7] = 0.0045066;
		start[8] = 0.0089264;
		start[9] = 0.0023294;
		start[10] = 0.0084073;
		start[11] = 0.00054922;
		start[12] = 0.0074437;
		start[13] = 0.0063413;
		start[14] = 0.0022391;
		start[15] = 0.0097164;
		start[16] = 0.0071124;
		start[17] = 0.00078153;
		start[18] = 0.0071944;
		start[19] = 0.0041311;
		start[20] = 0.009446;
		start[21] = 0.001627;
		start[22] = 0.0042904;
		start[23] = 0.00041826;
		start[24] = 0.0081999;
		start[25] = 0.0033676;
		start[26] = 0.00033179;
		start[27] = 0.004173;
		start[28] = 0.0027813;
		start[29] = 0.0096047;
		start[30] = 0.0048788;
		start[31] = 0.0004822;
		start[32] = 0.0052335;
		start[33] = 0.008981;
		start[34] = 0.0098757;
		start[35] = 0.0017477;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0047665;
		start[7] = 0.0064959;
		start[8] = 0.0015192;
		start[9] = 0.0049573;
		start[10] = 0.0027033;
		start[11] = 0.005788;
		start[12] = 0.0090988;
		start[13] = 0.0080136;
		start[14] = 0.003451;
		start[15] = 0.007318;
		start[16] = 0.0036571;
		start[17] = 0.0054209;
		start[18] = 0.0056132;
		start[19] = 0.0096163;
		start[20] = 0.001314;
		start[21] = 0.0068069;
		start[22] = 0.0060838;
		start[23] = 0.005126;
		start[24] = 0.0034462;
		start[25] = 3.6482e-05;
		start[26] = 0.0052355;
		start[27] = 0.0051977;
		start[28] = 0.0037533;
		start[29] = 0.0047057;
		start[30] = 0.0089375;
		start[31] = 0.0031022;
		start[32] = 0.0073877;
		start[33] = 0.0043862;
		start[34] = 0.0038693;
		start[35] = 0.0072147;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0060067;
		start[7] = 0.0033964;
		start[8] = 0.007466;
		start[9] = 0.0073532;
		start[10] = 0.0047604;
		start[11] = 0.0093002;
		start[12] = 0.0039619;
		start[13] = 3.6441e-05;
		start[14] = 0.0047879;
		start[15] = 0.0066869;
		start[16] = 0.0082678;
		start[17] = 0.0024418;
		start[18] = 0.0028572;
		start[19] = 0.0058027;
		start[20] = 0.000588;
		start[21] = 0.0089422;
		start[22] = 0.0063681;
		start[23] = 0.0046477;
		start[24] = 0.0050795;
		start[25] = 0.0044261;
		start[26] = 0.0012976;
		start[27] = 0.0078057;
		start[28] = 0.0019273;
		start[29] = 0.008105;
		start[30] = 0.0091776;
		start[31] = 0.0025504;
		start[32] = 0.0093236;
		start[33] = 0.0039131;
		start[34] = 0.0034254;
		start[35] = 0.0036524;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0037299;
		start[7] = 0.0051968;
		start[8] = 0.0070179;
		start[9] = 0.0028868;
		start[10] = 0.0086604;
		start[11] = 0.0082474;
		start[12] = 0.0036127;
		start[13] = 0.0065167;
		start[14] = 0.0037806;
		start[15] = 0.0045461;
		start[16] = 0.006332;
		start[17] = 0.0060793;
		start[18] = 0.00094165;
		start[19] = 0.0044197;
		start[20] = 0.0079346;
		start[21] = 0.0079698;
		start[22] = 0.0025832;
		start[23] = 0.0081173;
		start[24] = 0.0095629;
		start[25] = 0.0088686;
		start[26] = 0.001863;
		start[27] = 0.0011608;
		start[28] = 0.0026548;
		start[29] = 0.009526;
		start[30] = 0.00092818;
		start[31] = 0.0065985;
		start[32] = 0.00068062;
		start[33] = 0.0090305;
		start[34] = 0.0095944;
		start[35] = 0.0095589;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0086632;
		start[7] = 0.0011926;
		start[8] = 0.0055251;
		start[9] = 0.0075373;
		start[10] = 0.0013819;
		start[11] = 0.0069853;
		start[12] = 0.0078267;
		start[13] = 0.0079897;
		start[14] = 0.0016145;
		start[15] = 0.0081612;
		start[16] = 0.007393;
		start[17] = 0.0058325;
		start[18] = 0.006001;
		start[19] = 0.00026322;
		start[20] = 0.0070853;
		start[21] = 0.0013931;
		start[22] = 0.0092471;
		start[23] = 0.0092416;
		start[24] = 0.0029773;
		start[25] = 0.0054379;
		start[26] = 0.0021041;
		start[27] = 0.00764;
		start[28] = 0.0063659;
		start[29] = 0.0054558;
		start[30] = 0.0090138;
		start[31] = 0.0079271;
		start[32] = 0.0065675;
		start[33] = 0.00066188;
		start[34] = 0.0048458;
		start[35] = 0.0023855;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0051378;
		start[7] = 0.0072605;
		start[8] = 0.0096818;
		start[9] = 0.00065482;
		start[10] = 0.0085112;
		start[11] = 0.001131;
		start[12] = 0.0065032;
		start[13] = 0.0040155;
		start[14] = 0.0015188;
		start[15] = 0.00771;
		start[16] = 0.0085062;
		start[17] = 0.0027395;
		start[18] = 0.0006615;
		start[19] = 0.0083838;
		start[20] = 0.0061729;
		start[21] = 0.0052878;
		start[22] = 0.0024418;
		start[23] = 0.0026602;
		start[24] = 0.0086447;
		start[25] = 0.0025333;
		start[26] = 0.0093726;
		start[27] = 0.0087192;
		start[28] = 0.0021962;
		start[29] = 0.00046956;
		start[30] = 0.0090762;
		start[31] = 0.0085742;
		start[32] = 0.0056029;
		start[33] = 0.0039765;
		start[34] = 0.0047913;
		start[35] = 0.0061098;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0066404;
		start[7] = 0.0039542;
		start[8] = 0.0057462;
		start[9] = 0.0063549;
		start[10] = 0.0042851;
		start[11] = 0.0018312;
		start[12] = 0.0078547;
		start[13] = 0.0052576;
		start[14] = 0.0096668;
		start[15] = 0.0033033;
		start[16] = 0.0041508;
		start[17] = 4.1099e-05;
		start[18] = 0.0014793;
		start[19] = 0.0045767;
		start[20] = 0.00086114;
		start[21] = 0.00060595;
		start[22] = 0.002884;
		start[23] = 0.0013014;
		start[24] = 0.0049318;
		start[25] = 0.00013791;
		start[26] = 0.0032862;
		start[27] = 0.0065773;
		start[28] = 0.0050514;
		start[29] = 0.0083304;
		start[30] = 0.0034565;
		start[31] = 0.0025787;
		start[32] = 0.0038293;
		start[33] = 0.0039068;
		start[34] = 0.0023597;
		start[35] = 0.0098329;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0070755;
		start[7] = 0.0030466;
		start[8] = 0.0018865;
		start[9] = 0.0039594;
		start[10] = 0.0078527;
		start[11] = 0.0022743;
		start[12] = 0.0067314;
		start[13] = 0.0039716;
		start[14] = 0.006483;
		start[15] = 0.0006353;
		start[16] = 0.0039938;
		start[17] = 0.0089157;
		start[18] = 0.0087081;
		start[19] = 0.0040086;
		start[20] = 0.0035567;
		start[21] = 0.0076822;
		start[22] = 0.0077381;
		start[23] = 0.0053547;
		start[24] = 0.00072825;
		start[25] = 0.0025107;
		start[26] = 0.0066154;
		start[27] = 0.0047327;
		start[28] = 0.0088176;
		start[29] = 0.0071887;
		start[30] = 0.0095188;
		start[31] = 0.0098216;
		start[32] = 0.0018866;
		start[33] = 0.0018009;
		start[34] = 0.0036018;
		start[35] = 0.0059024;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0092469;
		start[7] = 2.755e-05;
		start[8] = 0.0092487;
		start[9] = 0.0017566;
		start[10] = 0.0003181;
		start[11] = 0.0045795;
		start[12] = 0.0093128;
		start[13] = 0.0017369;
		start[14] = 0.0034182;
		start[15] = 0.0046516;
		start[16] = 0.0083182;
		start[17] = 0.0016232;
		start[18] = 0.0095001;
		start[19] = 0.0030652;
		start[20] = 0.0062342;
		start[21] = 0.002325;
		start[22] = 0.00073186;
		start[23] = 0.0093391;
		start[24] = 0.0027435;
		start[25] = 0.0011707;
		start[26] = 0.0028537;
		start[27] = 0.00020418;
		start[28] = 0.0053091;
		start[29] = 0.0024842;
		start[30] = 0.0098155;
		start[31] = 0.00213;
		start[32] = 0.0024794;
		start[33] = 0.0027619;
		start[34] = 0.0078309;
		start[35] = 0.0041055;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0051187;
		start[7] = 0.0050658;
		start[8] = 0.00066084;
		start[9] = 0.007565;
		start[10] = 0.0054831;
		start[11] = 0.0093525;
		start[12] = 0.007081;
		start[13] = 0.0005493;
		start[14] = 0.0073667;
		start[15] = 0.00030853;
		start[16] = 0.0026734;
		start[17] = 0.0071974;
		start[18] = 0.0013981;
		start[19] = 0.0097674;
		start[20] = 0.0072973;
		start[21] = 0.0074205;
		start[22] = 0.0053804;
		start[23] = 0.0078962;
		start[24] = 0.0090868;
		start[25] = 0.0097642;
		start[26] = 0.0062402;
		start[27] = 0.0068158;
		start[28] = 0.0032301;
		start[29] = 0.0045479;
		start[30] = 0.0079763;
		start[31] = 0.0078354;
		start[32] = 0.0058841;
		start[33] = 0.0080292;
		start[34] = 0.0087965;
		start[35] = 0.0081071;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0099249;
		start[7] = 0.0057363;
		start[8] = 0.0049865;
		start[9] = 0.0003395;
		start[10] = 0.0080047;
		start[11] = 0.0088467;
		start[12] = 0.0056021;
		start[13] = 0.002016;
		start[14] = 0.005398;
		start[15] = 0.0053446;
		start[16] = 0.0015552;
		start[17] = 0.0034873;
		start[18] = 0.00042086;
		start[19] = 0.006397;
		start[20] = 0.0097511;
		start[21] = 0.0050116;
		start[22] = 0.007751;
		start[23] = 0.0012455;
		start[24] = 0.0044676;
		start[25] = 0.005513;
		start[26] = 0.0047521;
		start[27] = 0.00292;
		start[28] = 0.0058074;
		start[29] = 0.0057491;
		start[30] = 0.0093265;
		start[31] = 0.0079264;
		start[32] = 0.00094196;
		start[33] = 0.007662;
		start[34] = 0.0066101;
		start[35] = 0.0092112;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0026494;
		start[7] = 0.00452;
		start[8] = 0.0099546;
		start[9] = 0.0022402;
		start[10] = 0.004788;
		start[11] = 0.0028558;
		start[12] = 0.0065019;
		start[13] = 0.0091366;
		start[14] = 0.008274;
		start[15] = 0.0099764;
		start[16] = 0.0071376;
		start[17] = 0.0022409;
		start[18] = 0.0022912;
		start[19] = 0.0078643;
		start[20] = 0.0055024;
		start[21] = 0.0085814;
		start[22] = 0.0035436;
		start[23] = 0.0044944;
		start[24] = 0.0074327;
		start[25] = 0.0045632;
		start[26] = 0.0046132;
		start[27] = 0.00088775;
		start[28] = 6.8092e-05;
		start[29] = 0.0067806;
		start[30] = 0.001834;
		start[31] = 0.0044221;
		start[32] = 0.0080534;
		start[33] = 0.0073916;
		start[34] = 0.0065817;
		start[35] = 0.0094688;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0014454;
		start[7] = 0.0069532;
		start[8] = 0.0077359;
		start[9] = 0.0010038;
		start[10] = 0.0014616;
		start[11] = 0.0036076;
		start[12] = 0.0047179;
		start[13] = 0.0085191;
		start[14] = 0.007404;
		start[15] = 0.0029171;
		start[16] = 0.0071875;
		start[17] = 0.0055291;
		start[18] = 0.0056145;
		start[19] = 0.009552;
		start[20] = 0.0036311;
		start[21] = 0.0010614;
		start[22] = 0.0031264;
		start[23] = 0.0049754;
		start[24] = 0.0084388;
		start[25] = 0.0052128;
		start[26] = 0.004762;
		start[27] = 0.0021495;
		start[28] = 0.0016949;
		start[29] = 0.0094623;
		start[30] = 0.0079083;
		start[31] = 0.0044481;
		start[32] = 0.0017218;
		start[33] = 0.0047674;
		start[34] = 0.0091213;
		start[35] = 0.0039214;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0036384;
		start[7] = 0.0016568;
		start[8] = 0.005737;
		start[9] = 0.0034556;
		start[10] = 0.00059282;
		start[11] = 0.00020963;
		start[12] = 0.0060906;
		start[13] = 0.0099731;
		start[14] = 0.0016575;
		start[15] = 0.0066862;
		start[16] = 0.0084815;
		start[17] = 0.0046534;
		start[18] = 0.0017841;
		start[19] = 0.0045556;
		start[20] = 0.0036186;
		start[21] = 0.0044396;
		start[22] = 0.004641;
		start[23] = 0.0036965;
		start[24] = 0.0012073;
		start[25] = 0.0015685;
		start[26] = 0.0091057;
		start[27] = 0.0022484;
		start[28] = 0.0065615;
		start[29] = 0.0088465;
		start[30] = 0.0033335;
		start[31] = 0.0045958;
		start[32] = 0.0085167;
		start[33] = 0.0088471;
		start[34] = 0.0055247;
		start[35] = 0.0081184;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0070713;
		start[7] = 0.0042381;
		start[8] = 0.00071299;
		start[9] = 0.0011619;
		start[10] = 0.0040363;
		start[11] = 0.00021002;
		start[12] = 0.0028696;
		start[13] = 0.006757;
		start[14] = 0.0055496;
		start[15] = 0.003588;
		start[16] = 0.00012722;
		start[17] = 0.0070693;
		start[18] = 0.0081952;
		start[19] = 0.00049666;
		start[20] = 0.0036752;
		start[21] = 5.3333e-05;
		start[22] = 0.0089393;
		start[23] = 0.0083858;
		start[24] = 0.00038867;
		start[25] = 0.00776;
		start[26] = 0.0093501;
		start[27] = 0.0013791;
		start[28] = 0.00045642;
		start[29] = 0.0034928;
		start[30] = 0.000241;
		start[31] = 0.0034574;
		start[32] = 0.0055954;
		start[33] = 0.0098267;
		start[34] = 0.0080038;
		start[35] = 0.006474;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0081992;
		start[7] = 0.0038461;
		start[8] = 0.0054897;
		start[9] = 0.0065541;
		start[10] = 0.0078877;
		start[11] = 0.0029315;
		start[12] = 0.001156;
		start[13] = 0.002041;
		start[14] = 0.0099672;
		start[15] = 0.0061963;
		start[16] = 0.0073554;
		start[17] = 0.008702;
		start[18] = 0.0065883;
		start[19] = 0.00019082;
		start[20] = 0.007676;
		start[21] = 0.0072617;
		start[22] = 0.0056108;
		start[23] = 0.0084322;
		start[24] = 0.0070926;
		start[25] = 0.0097799;
		start[26] = 0.0061499;
		start[27] = 0.0081039;
		start[28] = 0.0069075;
		start[29] = 0.0097782;
		start[30] = 0.00086283;
		start[31] = 0.0099192;
		start[32] = 0.0019091;
		start[33] = 0.0054462;
		start[34] = 0.0081449;
		start[35] = 0.0091803;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0046497;
		start[7] = 0.006396;
		start[8] = 0.0028964;
		start[9] = 0.00031412;
		start[10] = 0.0092212;
		start[11] = 0.0034068;
		start[12] = 0.005469;
		start[13] = 0.0057572;
		start[14] = 0.005082;
		start[15] = 0.0029116;
		start[16] = 0.0025528;
		start[17] = 0.004483;
		start[18] = 0.0087837;
		start[19] = 0.0096975;
		start[20] = 0.00050401;
		start[21] = 0.0057155;
		start[22] = 0.0022533;
		start[23] = 0.0021163;
		start[24] = 0.0059064;
		start[25] = 0.0053415;
		start[26] = 0.0068218;
		start[27] = 0.007766;
		start[28] = 0.0072277;
		start[29] = 0.0056073;
		start[30] = 0.0067282;
		start[31] = 0.003365;
		start[32] = 0.0097538;
		start[33] = 0.008702;
		start[34] = 0.0082882;
		start[35] = 0.0094805;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0026049;
		start[7] = 0.006411;
		start[8] = 0.00069762;
		start[9] = 0.0045994;
		start[10] = 0.0012534;
		start[11] = 0.0004921;
		start[12] = 0.0088472;
		start[13] = 0.0058032;
		start[14] = 0.0022748;
		start[15] = 0.0071727;
		start[16] = 0.0093445;
		start[17] = 0.0042355;
		start[18] = 0.0043026;
		start[19] = 0.0061868;
		start[20] = 0.0067143;
		start[21] = 0.0029407;
		start[22] = 0.0028595;
		start[23] = 0.0003417;
		start[24] = 0.0037918;
		start[25] = 0.0070008;
		start[26] = 0.0012833;
		start[27] = 0.00074235;
		start[28] = 0.00089778;
		start[29] = 0.0062409;
		start[30] = 0.006173;
		start[31] = 0.0033769;
		start[32] = 0.0066676;
		start[33] = 0.0036773;
		start[34] = 0.0098073;
		start[35] = 0.0051168;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0035275;
		start[7] = 0.0068916;
		start[8] = 0.0062777;
		start[9] = 0.0099649;
		start[10] = 0.0097527;
		start[11] = 0.0076348;
		start[12] = 0.0087863;
		start[13] = 0.0098769;
		start[14] = 0.008399;
		start[15] = 0.0094983;
		start[16] = 0.0035086;
		start[17] = 0.0053428;
		start[18] = 0.0037579;
		start[19] = 0.0063177;
		start[20] = 0.0099828;
		start[21] = 0.0056302;
		start[22] = 0.00041953;
		start[23] = 0.0077787;
		start[24] = 0.0014304;
		start[25] = 0.0020808;
		start[26] = 0.0035371;
		start[27] = 0.0032878;
		start[28] = 0.0058305;
		start[29] = 0.0037405;
		start[30] = 0.0096386;
		start[31] = 0.0096623;
		start[32] = 0.0042153;
		start[33] = 0.0038032;
		start[34] = 0.0062579;
		start[35] = 0.0019867;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0081011;
		start[7] = 0.0074375;
		start[8] = 0.0025771;
		start[9] = 0.0024424;
		start[10] = 0.0082523;
		start[11] = 0.0025774;
		start[12] = 0.0012236;
		start[13] = 0.0071393;
		start[14] = 0.0082535;
		start[15] = 0.0096538;
		start[16] = 0.0059465;
		start[17] = 0.0063665;
		start[18] = 0.0096551;
		start[19] = 0.005666;
		start[20] = 0.0006482;
		start[21] = 0.00051085;
		start[22] = 0.0085031;
		start[23] = 0.008824;
		start[24] = 0.0094616;
		start[25] = 0.0092334;
		start[26] = 0.003797;
		start[27] = 0.008714;
		start[28] = 0.0057612;
		start[29] = 0.0018769;
		start[30] = 0.0099289;
		start[31] = 0.0089243;
		start[32] = 0.00021034;
		start[33] = 0.0042316;
		start[34] = 0.0083375;
		start[35] = 0.0065115;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0041995;
		start[7] = 0.0075025;
		start[8] = 0.00068049;
		start[9] = 0.0035529;
		start[10] = 0.0095758;
		start[11] = 0.0093982;
		start[12] = 0.0044342;
		start[13] = 0.00079958;
		start[14] = 0.0036932;
		start[15] = 0.00048372;
		start[16] = 0.0039771;
		start[17] = 0.0046242;
		start[18] = 0.0061344;
		start[19] = 0.0035605;
		start[20] = 0.007936;
		start[21] = 0.0099371;
		start[22] = 0.0089618;
		start[23] = 0.001264;
		start[24] = 0.0045528;
		start[25] = 0.00026299;
		start[26] = 0.0097622;
		start[27] = 0.0068897;
		start[28] = 0.0013493;
		start[29] = 0.0026342;
		start[30] = 0.0095583;
		start[31] = 0.0062909;
		start[32] = 0.0058115;
		start[33] = 0.0012573;
		start[34] = 0.0067813;
		start[35] = 0.0054961;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0052844;
		start[7] = 0.0070514;
		start[8] = 0.0022723;
		start[9] = 0.0084818;
		start[10] = 0.0050187;
		start[11] = 0.0060874;
		start[12] = 0.0056039;
		start[13] = 0.0094228;
		start[14] = 0.001204;
		start[15] = 0.0072011;
		start[16] = 0.0092629;
		start[17] = 0.0045113;
		start[18] = 0.007769;
		start[19] = 0.0023986;
		start[20] = 0.0030167;
		start[21] = 0.0093078;
		start[22] = 0.0046577;
		start[23] = 0.0023155;
		start[24] = 0.00081144;
		start[25] = 0.0044941;
		start[26] = 0.0045289;
		start[27] = 0.0088092;
		start[28] = 0.0096704;
		start[29] = 0.0036337;
		start[30] = 0.009836;
		start[31] = 0.00083969;
		start[32] = 0.00083943;
		start[33] = 0.0094877;
		start[34] = 0.0070639;
		start[35] = 0.0067111;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0047896;
		start[7] = 0.0014021;
		start[8] = 0.0072933;
		start[9] = 0.000111;
		start[10] = 0.008327;
		start[11] = 0.0089182;
		start[12] = 0.0014447;
		start[13] = 0.0010889;
		start[14] = 0.00028312;
		start[15] = 0.0023195;
		start[16] = 0.0086396;
		start[17] = 0.0029968;
		start[18] = 0.0087426;
		start[19] = 0.0065762;
		start[20] = 0.0055367;
		start[21] = 0.0090896;
		start[22] = 0.0050167;
		start[23] = 0.009667;
		start[24] = 0.0043167;
		start[25] = 0.0031655;
		start[26] = 0.00096349;
		start[27] = 0.0079775;
		start[28] = 0.0084553;
		start[29] = 0.0061813;
		start[30] = 0.0008777;
		start[31] = 0.0026863;
		start[32] = 0.0021428;
		start[33] = 0.0031498;
		start[34] = 0.0028589;
		start[35] = 0.0044302;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.009538;
		start[7] = 0.0038121;
		start[8] = 0.00061423;
		start[9] = 0.0091076;
		start[10] = 0.0054179;
		start[11] = 0.0028293;
		start[12] = 0.0052447;
		start[13] = 0.0096547;
		start[14] = 0.0070014;
		start[15] = 0.009469;
		start[16] = 0.0024276;
		start[17] = 0.0056061;
		start[18] = 0.0052687;
		start[19] = 0.008544;
		start[20] = 0.0037122;
		start[21] = 0.0093981;
		start[22] = 0.00014598;
		start[23] = 0.0058882;
		start[24] = 0.0053784;
		start[25] = 0.0019746;
		start[26] = 0.00054155;
		start[27] = 0.0014772;
		start[28] = 0.00012649;
		start[29] = 0.0060019;
		start[30] = 0.0055495;
		start[31] = 0.0059648;
		start[32] = 0.0022463;
		start[33] = 0.0082462;
		start[34] = 0.0029147;
		start[35] = 0.00067248;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.002643;
		start[7] = 0.0089417;
		start[8] = 0.0034281;
		start[9] = 0.0014614;
		start[10] = 0.0063292;
		start[11] = 0.0064008;
		start[12] = 0.0090281;
		start[13] = 0.0037024;
		start[14] = 0.0073236;
		start[15] = 0.0048638;
		start[16] = 0.0094427;
		start[17] = 0.0089443;
		start[18] = 0.0016603;
		start[19] = 0.0084935;
		start[20] = 0.0083544;
		start[21] = 0.0047101;
		start[22] = 0.0087247;
		start[23] = 0.008385;
		start[24] = 0.0029235;
		start[25] = 0.00050839;
		start[26] = 0.00061858;
		start[27] = 0.0019149;
		start[28] = 0.0089675;
		start[29] = 0.005804;
		start[30] = 0.0087566;
		start[31] = 0.0044434;
		start[32] = 0.0079836;
		start[33] = 0.0031719;
		start[34] = 0.00024942;
		start[35] = 0.0010174;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0031285;
		start[7] = 0.001509;
		start[8] = 0.0040524;
		start[9] = 0.0080727;
		start[10] = 0.0041537;
		start[11] = 0.0055616;
		start[12] = 0.0068653;
		start[13] = 0.0085985;
		start[14] = 0.0084227;
		start[15] = 0.0039901;
		start[16] = 0.0004621;
		start[17] = 0.0021526;
		start[18] = 0.0052677;
		start[19] = 0.0081718;
		start[20] = 0.0074032;
		start[21] = 0.0022074;
		start[22] = 0.0060993;
		start[23] = 0.008127;
		start[24] = 0.00083739;
		start[25] = 0.0076583;
		start[26] = 0.0023272;
		start[27] = 0.00063975;
		start[28] = 0.00086592;
		start[29] = 0.0028797;
		start[30] = 0.0021774;
		start[31] = 0.001479;
		start[32] = 0.0057772;
		start[33] = 0.0017945;
		start[34] = 1.837e-05;
		start[35] = 1.5797e-05;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0049977;
		start[7] = 0.0028367;
		start[8] = 0.0052073;
		start[9] = 0.0093545;
		start[10] = 0.002948;
		start[11] = 0.0011782;
		start[12] = 0.007682;
		start[13] = 0.0059879;
		start[14] = 0.0049118;
		start[15] = 0.0038212;
		start[16] = 0.0027675;
		start[17] = 0.0026295;
		start[18] = 0.0074045;
		start[19] = 0.0089845;
		start[20] = 0.0091075;
		start[21] = 0.0046554;
		start[22] = 0.0075096;
		start[23] = 0.0061367;
		start[24] = 0.0061928;
		start[25] = 0.0060765;
		start[26] = 0.0034666;
		start[27] = 0.0054759;
		start[28] = 0.0013215;
		start[29] = 0.0076688;
		start[30] = 0.006537;
		start[31] = 0.0054616;
		start[32] = 0.0080438;
		start[33] = 0.0011538;
		start[34] = 0.0013041;
		start[35] = 0.0078452;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0085936;
		start[7] = 0.0014658;
		start[8] = 0.0065435;
		start[9] = 0.0051861;
		start[10] = 5.4187e-05;
		start[11] = 0.0069641;
		start[12] = 0.0076404;
		start[13] = 0.0050483;
		start[14] = 0.0091;
		start[15] = 0.0015273;
		start[16] = 0.0022218;
		start[17] = 9.2813e-05;
		start[18] = 0.0081074;
		start[19] = 0.0093438;
		start[20] = 0.00085449;
		start[21] = 0.0064841;
		start[22] = 0.0010507;
		start[23] = 0.0063512;
		start[24] = 0.0029169;
		start[25] = 0.0088422;
		start[26] = 0.0034733;
		start[27] = 0.0073357;
		start[28] = 0.0099882;
		start[29] = 0.00095905;
		start[30] = 0.004481;
		start[31] = 0.0027751;
		start[32] = 0.00718;
		start[33] = 0.00093562;
		start[34] = 0.00057626;
		start[35] = 0.0073237;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0023768;
		start[7] = 0.0024454;
		start[8] = 0.0070488;
		start[9] = 0.0023357;
		start[10] = 0.0060283;
		start[11] = 0.00091966;
		start[12] = 0.0090191;
		start[13] = 0.0050808;
		start[14] = 0.0062501;
		start[15] = 0.0067226;
		start[16] = 0.0028086;
		start[17] = 0.0035517;
		start[18] = 0.0013674;
		start[19] = 0.0054211;
		start[20] = 0.0023316;
		start[21] = 0.0052868;
		start[22] = 0.0096777;
		start[23] = 8.1202e-05;
		start[24] = 0.004584;
		start[25] = 0.0073667;
		start[26] = 0.008893;
		start[27] = 0.0089095;
		start[28] = 0.0062817;
		start[29] = 0.0066407;
		start[30] = 0.0075863;
		start[31] = 0.0064013;
		start[32] = 0.0048308;
		start[33] = 0.0016719;
		start[34] = 0.0055802;
		start[35] = 0.0065226;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0015335;
		start[7] = 0.0082188;
		start[8] = 0.0047453;
		start[9] = 0.00052911;
		start[10] = 0.00092784;
		start[11] = 0.0025286;
		start[12] = 0.001635;
		start[13] = 0.0043367;
		start[14] = 8.5612e-05;
		start[15] = 0.0010077;
		start[16] = 0.0081855;
		start[17] = 0.0039235;
		start[18] = 0.0010814;
		start[19] = 0.0095485;
		start[20] = 0.00010799;
		start[21] = 0.0048942;
		start[22] = 0.0017409;
		start[23] = 0.0092096;
		start[24] = 0.00041253;
		start[25] = 0.0082655;
		start[26] = 0.0076503;
		start[27] = 0.0027643;
		start[28] = 0.0084532;
		start[29] = 0.0064237;
		start[30] = 0.0072273;
		start[31] = 0.0039146;
		start[32] = 0.0023881;
		start[33] = 0.0091417;
		start[34] = 0.0043563;
		start[35] = 0.0069247;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.00064019;
		start[7] = 0.0086107;
		start[8] = 0.0089368;
		start[9] = 0.0096459;
		start[10] = 0.0078351;
		start[11] = 0.0018905;
		start[12] = 0.0017797;
		start[13] = 0.0029248;
		start[14] = 0.0030589;
		start[15] = 0.00012408;
		start[16] = 0.0088397;
		start[17] = 0.0078462;
		start[18] = 0.0037829;
		start[19] = 0.0063666;
		start[20] = 0.0080833;
		start[21] = 0.00065941;
		start[22] = 0.0087169;
		start[23] = 0.0094992;
		start[24] = 0.0013072;
		start[25] = 0.0061029;
		start[26] = 0.0014627;
		start[27] = 0.0090838;
		start[28] = 0.006636;
		start[29] = 0.00086912;
		start[30] = 0.00073826;
		start[31] = 0.0090143;
		start[32] = 0.0019843;
		start[33] = 0.0021973;
		start[34] = 0.0067998;
		start[35] = 0.0053558;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0041561;
		start[7] = 0.00080067;
		start[8] = 0.0048553;
		start[9] = 0.0037659;
		start[10] = 0.0025318;
		start[11] = 0.006862;
		start[12] = 0.0015372;
		start[13] = 0.0092387;
		start[14] = 0.0062075;
		start[15] = 0.0079005;
		start[16] = 0.00065317;
		start[17] = 0.001598;
		start[18] = 0.0092172;
		start[19] = 0.0075706;
		start[20] = 0.0027871;
		start[21] = 0.0091638;
		start[22] = 0.00031341;
		start[23] = 0.0054402;
		start[24] = 0.0075005;
		start[25] = 0.0048817;
		start[26] = 0.0039766;
		start[27] = 0.00053829;
		start[28] = 0.0031952;
		start[29] = 0.0091468;
		start[30] = 0.0058274;
		start[31] = 0.0047278;
		start[32] = 0.0031798;
		start[33] = 0.0090804;
		start[34] = 0.0091281;
		start[35] = 0.0054928;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0084613;
		start[7] = 0.0028063;
		start[8] = 0.0024681;
		start[9] = 0.0078478;
		start[10] = 0.0060161;
		start[11] = 0.0023979;
		start[12] = 0.0041818;
		start[13] = 0.0015452;
		start[14] = 0.0016793;
		start[15] = 0.006759;
		start[16] = 0.0089811;
		start[17] = 0.0057368;
		start[18] = 0.0094528;
		start[19] = 0.0052989;
		start[20] = 0.0024416;
		start[21] = 0.0028315;
		start[22] = 0.0092213;
		start[23] = 0.0053994;
		start[24] = 0.0050222;
		start[25] = 0.0040987;
		start[26] = 0.0064082;
		start[27] = 0.0070698;
		start[28] = 0.0023813;
		start[29] = 0.0093401;
		start[30] = 0.0023447;
		start[31] = 0.0026021;
		start[32] = 0.0055511;
		start[33] = 0.0026998;
		start[34] = 0.0015872;
		start[35] = 0.0025978;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0084969;
		start[7] = 0.0012612;
		start[8] = 0.0077564;
		start[9] = 0.00025467;
		start[10] = 0.0061849;
		start[11] = 0.0027165;
		start[12] = 0.00026255;
		start[13] = 0.0040716;
		start[14] = 0.0070519;
		start[15] = 0.009316;
		start[16] = 0.00059698;
		start[17] = 0.000717;
		start[18] = 0.0023738;
		start[19] = 0.0012223;
		start[20] = 0.00073563;
		start[21] = 0.0017952;
		start[22] = 0.0087317;
		start[23] = 0.0085094;
		start[24] = 0.0085558;
		start[25] = 0.0055044;
		start[26] = 0.0043188;
		start[27] = 0.008914;
		start[28] = 0.00085005;
		start[29] = 0.0061566;
		start[30] = 0.0036071;
		start[31] = 0.0070281;
		start[32] = 0.0067103;
		start[33] = 0.0099629;
		start[34] = 0.0020305;
		start[35] = 0.0048589;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.00066183;
		start[7] = 0.0015804;
		start[8] = 0.0085322;
		start[9] = 0.00019368;
		start[10] = 0.0018175;
		start[11] = 0.0056658;
		start[12] = 0.0035398;
		start[13] = 0.0091192;
		start[14] = 0.0081505;
		start[15] = 0.0051139;
		start[16] = 0.003697;
		start[17] = 0.004598;
		start[18] = 0.0083935;
		start[19] = 0.0010151;
		start[20] = 0.0020374;
		start[21] = 0.0037828;
		start[22] = 0.0026449;
		start[23] = 0.00040303;
		start[24] = 0.0044466;
		start[25] = 0.006519;
		start[26] = 0.0092146;
		start[27] = 0.0033073;
		start[28] = 0.0064075;
		start[29] = 0.0070513;
		start[30] = 0.009856;
		start[31] = 0.0050805;
		start[32] = 0.0029017;
		start[33] = 0.00187;
		start[34] = 0.0078804;
		start[35] = 0.0043655;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (36);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.00056396;
		start[7] = 0.0091801;
		start[8] = 0.0010327;
		start[9] = 0.0066318;
		start[10] = 0.0022305;
		start[11] = 0.005154;
		start[12] = 0.0015271;
		start[13] = 0.0024526;
		start[14] = 0.0017725;
		start[15] = 0.00074101;
		start[16] = 0.0079955;
		start[17] = 0.0039429;
		start[18] = 0.0014699;
		start[19] = 0.0086915;
		start[20] = 0.0012135;
		start[21] = 0.0092189;
		start[22] = 0.0054276;
		start[23] = 0.00025901;
		start[24] = 0.00072291;
		start[25] = 0.0012475;
		start[26] = 0.0065454;
		start[27] = 0.005933;
		start[28] = 0.0028676;
		start[29] = 0.0011699;
		start[30] = 0.0048414;
		start[31] = 0.0090852;
		start[32] = 0.00085416;
		start[33] = 0.0007862;
		start[34] = 0.00082815;
		start[35] = 0.0045282;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}


  return 0;
}
