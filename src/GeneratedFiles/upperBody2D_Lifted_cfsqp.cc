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
    (42, 1, "CostFunction_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
    (42, 1, "LiftConstraint_1_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = cos(q_04) - 1.0*w_01_01;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = -1.0*sin(q_04); 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = -1.0; 
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
    (42, 1, "LiftConstraint_2_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = cos(q_01) - 1.0*w_01_02;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
    (42, 1, "LiftConstraint_3_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
    (42, 1, "LiftConstraint_4_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = sin(q_01) - 1.0*w_01_04;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
    (42, 1, "LiftConstraint_5_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = cos(q_03 + 1.5707963267948966192313216916398) - 1.0*w_01_05;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = -1.0*sin(q_03 + 1.5707963267948966192313216916398); 
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
    (42, 1, "LiftConstraint_6_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = sin(q_04) - 1.0*w_01_06;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
    (42, 1, "LiftConstraint_7_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = cos(q_05 - 1.5707963267948966192313216916398) - 1.0*w_01_07;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = -1.0*sin(q_05 - 1.5707963267948966192313216916398); 
			 grad[5] = 0.0; 
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
    (42, 1, "LiftConstraint_8_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = sin(q_06) - 1.0*w_01_08;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
    (42, 1, "LiftConstraint_9_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = sin(q_03 + 1.5707963267948966192313216916398) - 1.0*w_01_09;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = cos(q_03 + 1.5707963267948966192313216916398); 
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
    (42, 1, "LiftConstraint_10_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = sin(q_05 - 1.5707963267948966192313216916398) - 1.0*w_01_10;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = cos(q_05 - 1.5707963267948966192313216916398); 
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
    (42, 1, "LiftConstraint_11_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = cos(q_02) - 1.0*w_01_11;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
    (42, 1, "LiftConstraint_12_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = sin(q_02) - 1.0*w_01_12;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
    (42, 1, "LiftConstraint_13_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_02*w_01_11 - 1.0*w_02_01;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[7] = w_01_11; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = w_01_02; 
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
    (42, 1, "LiftConstraint_14_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_04*w_01_11 - 1.0*w_02_02;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[9] = w_01_11; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = w_01_04; 
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
    (42, 1, "LiftConstraint_15_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_02*w_01_12 - 1.0*w_02_03;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[7] = w_01_12; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = w_01_02; 
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
    (42, 1, "LiftConstraint_16_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_04*w_01_12 - 1.0*w_02_04;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[9] = w_01_12; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = w_01_04; 
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
    (42, 1, "LiftConstraint_17_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = - 1.0*w_03_01 - 1.0*w_01_05*(w_02_02 + w_02_03);
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[10] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = -1.0*w_01_05; 
			 grad[20] = -1.0*w_01_05; 
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
    (42, 1, "LiftConstraint_18_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_05*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_02;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[10] = w_02_01 - 1.0*w_02_04; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = w_01_05; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = -1.0*w_01_05; 
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
    (42, 1, "LiftConstraint_19_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = - 1.0*w_03_03 - 1.0*w_01_07*(w_02_02 + w_02_03);
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[12] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = -1.0*w_01_07; 
			 grad[20] = -1.0*w_01_07; 
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
    (42, 1, "LiftConstraint_20_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_07*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_04;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[12] = w_02_01 - 1.0*w_02_04; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = w_01_07; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = -1.0*w_01_07; 
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
    (42, 1, "LiftConstraint_21_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_09*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_05;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[14] = w_02_01 - 1.0*w_02_04; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = w_01_09; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = -1.0*w_01_09; 
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
    (42, 1, "LiftConstraint_22_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_09*(w_02_02 + w_02_03) - 1.0*w_03_06;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[14] = w_02_02 + w_02_03; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = w_01_09; 
			 grad[20] = w_01_09; 
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
    (42, 1, "LiftConstraint_23_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_10*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_07;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[15] = w_02_01 - 1.0*w_02_04; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = w_01_10; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = -1.0*w_01_10; 
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
    (42, 1, "LiftConstraint_24_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_10*(w_02_02 + w_02_03) - 1.0*w_03_08;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[15] = w_02_02 + w_02_03; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = w_01_10; 
			 grad[20] = w_01_10; 
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
    (42, 1, "LiftConstraint_25_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = - 1.0*w_03_09 - 1.0*w_01_09*(w_02_02 + w_02_03);
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[14] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = -1.0*w_01_09; 
			 grad[20] = -1.0*w_01_09; 
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
    (42, 1, "LiftConstraint_26_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_05*(w_02_02 + w_02_03) - 1.0*w_03_10;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[10] = w_02_02 + w_02_03; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = w_01_05; 
			 grad[20] = w_01_05; 
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
    (42, 1, "LiftConstraint_27_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = - 1.0*w_03_11 - 1.0*w_01_10*(w_02_02 + w_02_03);
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[15] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = -1.0*w_01_10; 
			 grad[20] = -1.0*w_01_10; 
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
    (42, 1, "LiftConstraint_28_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_07*(w_02_02 + w_02_03) - 1.0*w_03_12;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[12] = w_02_02 + w_02_03; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = w_01_07; 
			 grad[20] = w_01_07; 
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
    (42, 1, "LiftConstraint_29_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_01*(w_03_01 - 1.0*w_03_05) - 1.0*w_04_01;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = w_03_01 - 1.0*w_03_05; 
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
			 grad[22] = w_01_01; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = -1.0*w_01_01; 
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
    (42, 1, "LiftConstraint_30_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_01*(w_03_02 - 1.0*w_03_06) - 1.0*w_04_02;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = w_03_02 - 1.0*w_03_06; 
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
			 grad[23] = w_01_01; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = -1.0*w_01_01; 
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
LiftConstraint_31<T>::LiftConstraint_31 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (42, 1, "LiftConstraint_31_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_03*(w_03_03 - 1.0*w_03_07) - 1.0*w_04_03;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[8] = w_03_03 - 1.0*w_03_07; 
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
			 grad[24] = w_01_03; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = -1.0*w_01_03; 
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
LiftConstraint_32<T>::LiftConstraint_32 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (42, 1, "LiftConstraint_32_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_03*(w_03_04 - 1.0*w_03_08) - 1.0*w_04_04;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[8] = w_03_04 - 1.0*w_03_08; 
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
			 grad[25] = w_01_03; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = -1.0*w_01_03; 
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
LiftConstraint_33<T>::LiftConstraint_33 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (42, 1, "LiftConstraint_33_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_06*(w_03_02 + w_03_09) - 1.0*w_04_05;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[11] = w_03_02 + w_03_09; 
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
			 grad[23] = w_01_06; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = w_01_06; 
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
LiftConstraint_34<T>::LiftConstraint_34 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (42, 1, "LiftConstraint_34_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_06*(w_03_05 + w_03_10) - 1.0*w_04_06;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[11] = w_03_05 + w_03_10; 
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
			 grad[26] = w_01_06; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = w_01_06; 
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
LiftConstraint_35<T>::LiftConstraint_35 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (42, 1, "LiftConstraint_35_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_08*(w_03_04 + w_03_11) - 1.0*w_04_07;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[13] = w_03_04 + w_03_11; 
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
			 grad[25] = w_01_08; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = w_01_08; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = -1.0; 
			 grad[41] = 0.0; 
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
LiftConstraint_36<T>::LiftConstraint_36 (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (42, 1, "LiftConstraint_36_upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = w_01_08*(w_03_07 + w_03_12) - 1.0*w_04_08;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[13] = w_03_07 + w_03_12; 
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
			 grad[28] = w_01_08; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = w_01_08; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = -1.0; 
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
    (42, 1, "EEConstraint_1_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = 0.25*w_02_04 - 0.5*w_01_04 - 0.25*w_02_01 - 0.5*w_02_02 - 0.5*w_02_03 - 1.0*EE_1_1 + 0.25*w_03_01 - 0.25*w_03_05 + 0.3*w_04_01 - 0.3*w_04_05;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[9] = -0.5; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = -0.25; 
			 grad[19] = -0.5; 
			 grad[20] = -0.5; 
			 grad[21] = 0.25; 
			 grad[22] = 0.25; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = -0.25; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.3; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = -0.3; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
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
    (42, 1, "EEConstraint_2_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = 0.5*w_01_02 - 1.0*EE_1_2 + 0.5*w_02_01 - 0.25*w_02_02 - 0.25*w_02_03 - 0.5*w_02_04 + 0.25*w_03_02 - 0.25*w_03_06 + 0.3*w_04_02 - 0.3*w_04_06;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[7] = 0.5; 
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
			 grad[18] = 0.5; 
			 grad[19] = -0.25; 
			 grad[20] = -0.25; 
			 grad[21] = -0.5; 
			 grad[22] = 0.0; 
			 grad[23] = 0.25; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = -0.25; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.3; 
			 grad[36] = 0.0; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = -0.3; 
			 grad[40] = 0.0; 
			 grad[41] = 0.0; 
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
    (42, 1, "EEConstraint_3_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = 0.25*w_02_01 - 0.5*w_01_04 - 1.0*EE_2_1 - 0.5*w_02_02 - 0.5*w_02_03 - 0.25*w_02_04 + 0.25*w_03_03 - 0.25*w_03_07 + 0.3*w_04_03 - 0.3*w_04_07;
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
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[9] = -0.5; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.25; 
			 grad[19] = -0.5; 
			 grad[20] = -0.5; 
			 grad[21] = -0.25; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.25; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = -0.25; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.3; 
			 grad[37] = 0.0; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = -0.3; 
			 grad[41] = 0.0; 
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
    (42, 1, "EEConstraint_4_upperBody2D"),
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
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];
  
	result[0] = 0.5*w_01_02 - 1.0*EE_2_2 + 0.5*w_02_01 + 0.25*w_02_02 + 0.25*w_02_03 - 0.5*w_02_04 + 0.25*w_03_04 - 0.25*w_03_08 + 0.3*w_04_04 - 0.3*w_04_08;
}

template <typename T>
void
EEConstraint_4<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];
	const double& w_01_01 = x[6];
	const double& w_01_02 = x[7];
	const double& w_01_03 = x[8];
	const double& w_01_04 = x[9];
	const double& w_01_05 = x[10];
	const double& w_01_06 = x[11];
	const double& w_01_07 = x[12];
	const double& w_01_08 = x[13];
	const double& w_01_09 = x[14];
	const double& w_01_10 = x[15];
	const double& w_01_11 = x[16];
	const double& w_01_12 = x[17];
	const double& w_02_01 = x[18];
	const double& w_02_02 = x[19];
	const double& w_02_03 = x[20];
	const double& w_02_04 = x[21];
	const double& w_03_01 = x[22];
	const double& w_03_02 = x[23];
	const double& w_03_03 = x[24];
	const double& w_03_04 = x[25];
	const double& w_03_05 = x[26];
	const double& w_03_06 = x[27];
	const double& w_03_07 = x[28];
	const double& w_03_08 = x[29];
	const double& w_03_09 = x[30];
	const double& w_03_10 = x[31];
	const double& w_03_11 = x[32];
	const double& w_03_12 = x[33];
	const double& w_04_01 = x[34];
	const double& w_04_02 = x[35];
	const double& w_04_03 = x[36];
	const double& w_04_04 = x[37];
	const double& w_04_05 = x[38];
	const double& w_04_06 = x[39];
	const double& w_04_07 = x[40];
	const double& w_04_08 = x[41];

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
			 grad[7] = 0.5; 
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
			 grad[18] = 0.5; 
			 grad[19] = 0.25; 
			 grad[20] = 0.25; 
			 grad[21] = -0.5; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.25; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = -0.25; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
			 grad[37] = 0.3; 
			 grad[38] = 0.0; 
			 grad[39] = 0.0; 
			 grad[40] = 0.0; 
			 grad[41] = -0.3; 
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
	boost::shared_ptr<LiftConstraint_31<roboptim::EigenMatrixDense> > cstrFunc_31 = boost::make_shared<LiftConstraint_31<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_32<roboptim::EigenMatrixDense> > cstrFunc_32 = boost::make_shared<LiftConstraint_32<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_33<roboptim::EigenMatrixDense> > cstrFunc_33 = boost::make_shared<LiftConstraint_33<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_34<roboptim::EigenMatrixDense> > cstrFunc_34 = boost::make_shared<LiftConstraint_34<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_35<roboptim::EigenMatrixDense> > cstrFunc_35 = boost::make_shared<LiftConstraint_35<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<LiftConstraint_36<roboptim::EigenMatrixDense> > cstrFunc_36 = boost::make_shared<LiftConstraint_36<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_37 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_38 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<EEConstraint_3<roboptim::EigenMatrixDense> > cstrFunc_39 = boost::make_shared<EEConstraint_3<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<EEConstraint_4<roboptim::EigenMatrixDense> > cstrFunc_40 = boost::make_shared<EEConstraint_4<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);

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
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_37), bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_38), bounds, scales); 
	}
	{
		EEConstraint_3<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_39), bounds, scales); 
	}
	{
		EEConstraint_4<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_40), bounds, scales); 
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
		std::vector<double> start (42);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0021034;
		start[7] = 0.0086383;
		start[8] = 0.0082537;
		start[9] = 0.0057411;
		start[10] = 0.0005223;
		start[11] = 0.003836;
		start[12] = 0.00043719;
		start[13] = 0.0040469;
		start[14] = 0.0027179;
		start[15] = 0.0049307;
		start[16] = 0.0097692;
		start[17] = 0.0058988;
		start[18] = 0.0007484;
		start[19] = 0.0015535;
		start[20] = 0.0016402;
		start[21] = 0.0015528;
		start[22] = 0.0089469;
		start[23] = 0.0014119;
		start[24] = 0.002093;
		start[25] = 0.0053538;
		start[26] = 0.007599;
		start[27] = 0.0038507;
		start[28] = 0.0061224;
		start[29] = 0.00052949;
		start[30] = 0.0043482;
		start[31] = 0.0026029;
		start[32] = 0.0050072;
		start[33] = 0.0098949;
		start[34] = 0.0025796;
		start[35] = 0.0079453;
		start[36] = 0.0040422;
		start[37] = 0.0073383;
		start[38] = 0.0033549;
		start[39] = 1.1286e-05;
		start[40] = 0.0022033;
		start[41] = 0.0070656;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0088134;
		start[7] = 0.0095927;
		start[8] = 0.00016404;
		start[9] = 0.0080311;
		start[10] = 0.0012199;
		start[11] = 0.0036674;
		start[12] = 0.0074222;
		start[13] = 0.0012302;
		start[14] = 0.0049552;
		start[15] = 0.0021519;
		start[16] = 0.007253;
		start[17] = 0.006912;
		start[18] = 0.0049218;
		start[19] = 0.0048401;
		start[20] = 0.0054492;
		start[21] = 0.00052894;
		start[22] = 0.0084053;
		start[23] = 0.003477;
		start[24] = 0.0030422;
		start[25] = 0.0097751;
		start[26] = 0.0036311;
		start[27] = 0.0059622;
		start[28] = 0.0043362;
		start[29] = 0.0069084;
		start[30] = 0.0053961;
		start[31] = 0.0059152;
		start[32] = 0.0079209;
		start[33] = 0.0071713;
		start[34] = 0.00075405;
		start[35] = 0.0056647;
		start[36] = 0.0050168;
		start[37] = 0.0049598;
		start[38] = 0.0086583;
		start[39] = 0.00022213;
		start[40] = 0.0098558;
		start[41] = 0.0022585;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.008992;
		start[7] = 0.0046751;
		start[8] = 0.0056768;
		start[9] = 0.0083779;
		start[10] = 0.0051307;
		start[11] = 0.0043006;
		start[12] = 0.0027342;
		start[13] = 0.0023094;
		start[14] = 0.0052876;
		start[15] = 0.0037318;
		start[16] = 0.00096529;
		start[17] = 0.0044478;
		start[18] = 0.0091461;
		start[19] = 0.0035144;
		start[20] = 0.0057811;
		start[21] = 0.0012539;
		start[22] = 0.0012456;
		start[23] = 0.0048293;
		start[24] = 0.0078215;
		start[25] = 0.0048809;
		start[26] = 0.0063409;
		start[27] = 0.0030053;
		start[28] = 0.0097842;
		start[29] = 0.0097262;
		start[30] = 0.001599;
		start[31] = 0.00099752;
		start[32] = 0.0029957;
		start[33] = 0.0071419;
		start[34] = 0.009106;
		start[35] = 0.0096765;
		start[36] = 0.0046844;
		start[37] = 0.0027035;
		start[38] = 0.00061329;
		start[39] = 0.0054614;
		start[40] = 0.005647;
		start[41] = 0.0047029;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0030742;
		start[7] = 0.0015141;
		start[8] = 0.0046267;
		start[9] = 0.008484;
		start[10] = 0.0068156;
		start[11] = 0.0061365;
		start[12] = 0.0071575;
		start[13] = 0.0058741;
		start[14] = 0.0044224;
		start[15] = 0.0053004;
		start[16] = 0.0069032;
		start[17] = 0.0019175;
		start[18] = 0.0068677;
		start[19] = 0.0079559;
		start[20] = 0.00014535;
		start[21] = 0.0047577;
		start[22] = 0.0098921;
		start[23] = 0.0045097;
		start[24] = 0.0064483;
		start[25] = 0.007967;
		start[26] = 0.0067119;
		start[27] = 0.0038873;
		start[28] = 0.0057762;
		start[29] = 9.7725e-05;
		start[30] = 0.0037709;
		start[31] = 0.0079667;
		start[32] = 0.0025813;
		start[33] = 0.0088784;
		start[34] = 0.0027306;
		start[35] = 0.00013215;
		start[36] = 0.0039364;
		start[37] = 0.0053387;
		start[38] = 0.0016716;
		start[39] = 0.0052428;
		start[40] = 0.0069075;
		start[41] = 0.0070048;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0066522;
		start[7] = 0.0058073;
		start[8] = 0.0095292;
		start[9] = 0.0025824;
		start[10] = 0.00012071;
		start[11] = 0.0067134;
		start[12] = 0.0043775;
		start[13] = 0.0014528;
		start[14] = 0.0047265;
		start[15] = 0.0081588;
		start[16] = 0.0020857;
		start[17] = 9.1209e-05;
		start[18] = 0.0030133;
		start[19] = 0.0079834;
		start[20] = 0.0022437;
		start[21] = 0.0082935;
		start[22] = 0.002361;
		start[23] = 0.0074462;
		start[24] = 0.00036423;
		start[25] = 0.0021137;
		start[26] = 0.0027429;
		start[27] = 0.0015814;
		start[28] = 0.0072073;
		start[29] = 0.005198;
		start[30] = 0.0092811;
		start[31] = 0.0050599;
		start[32] = 0.0085836;
		start[33] = 0.0068755;
		start[34] = 0.0021947;
		start[35] = 0.0069847;
		start[36] = 0.005247;
		start[37] = 0.0071189;
		start[38] = 0.001876;
		start[39] = 0.0057272;
		start[40] = 0.005011;
		start[41] = 0.0086099;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0092244;
		start[7] = 0.0014125;
		start[8] = 0.0087301;
		start[9] = 0.0076251;
		start[10] = 0.002051;
		start[11] = 1.4191e-05;
		start[12] = 0.0094638;
		start[13] = 0.0075478;
		start[14] = 0.0090103;
		start[15] = 0.00017797;
		start[16] = 0.0052549;
		start[17] = 0.0083183;
		start[18] = 0.0065486;
		start[19] = 0.009293;
		start[20] = 0.0037896;
		start[21] = 0.0029441;
		start[22] = 0.0080965;
		start[23] = 0.0091344;
		start[24] = 0.0079262;
		start[25] = 0.0065684;
		start[26] = 0.0051496;
		start[27] = 0.0074538;
		start[28] = 0.00052126;
		start[29] = 0.00074513;
		start[30] = 0.0026539;
		start[31] = 0.0086164;
		start[32] = 0.005837;
		start[33] = 0.008977;
		start[34] = 0.0037169;
		start[35] = 0.0034499;
		start[36] = 0.0040676;
		start[37] = 0.0047135;
		start[38] = 0.0094905;
		start[39] = 0.0093676;
		start[40] = 0.0028043;
		start[41] = 0.0080876;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0040181;
		start[7] = 0.0092017;
		start[8] = 0.0057146;
		start[9] = 0.0095241;
		start[10] = 0.0019381;
		start[11] = 0.0029472;
		start[12] = 0.0076978;
		start[13] = 0.0017636;
		start[14] = 0.00018649;
		start[15] = 0.00913;
		start[16] = 3.7281e-07;
		start[17] = 0.0045076;
		start[18] = 0.0041758;
		start[19] = 0.0019026;
		start[20] = 0.0023542;
		start[21] = 0.0059923;
		start[22] = 0.0058656;
		start[23] = 0.0037394;
		start[24] = 0.0056177;
		start[25] = 0.00062876;
		start[26] = 0.00019419;
		start[27] = 0.00029621;
		start[28] = 0.00085877;
		start[29] = 0.0060904;
		start[30] = 0.0023249;
		start[31] = 0.0077511;
		start[32] = 0.0058592;
		start[33] = 0.0023608;
		start[34] = 0.0098615;
		start[35] = 0.0082933;
		start[36] = 0.0012742;
		start[37] = 0.003555;
		start[38] = 0.0001006;
		start[39] = 0.0060564;
		start[40] = 0.0085078;
		start[41] = 0.0036873;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.009544;
		start[7] = 0.0037732;
		start[8] = 0.0017461;
		start[9] = 0.0044534;
		start[10] = 0.0015526;
		start[11] = 0.0016988;
		start[12] = 0.0059648;
		start[13] = 0.0098629;
		start[14] = 0.003985;
		start[15] = 0.0070963;
		start[16] = 0.0028525;
		start[17] = 0.0054606;
		start[18] = 0.0031627;
		start[19] = 0.0039965;
		start[20] = 0.0051945;
		start[21] = 0.0059989;
		start[22] = 0.0011903;
		start[23] = 0.0098472;
		start[24] = 0.0060975;
		start[25] = 0.0073006;
		start[26] = 0.0080721;
		start[27] = 0.00046467;
		start[28] = 0.0065495;
		start[29] = 0.0014465;
		start[30] = 0.0050814;
		start[31] = 0.00077701;
		start[32] = 0.0056213;
		start[33] = 0.0019953;
		start[34] = 0.0031746;
		start[35] = 0.0086504;
		start[36] = 0.0016252;
		start[37] = 0.0078314;
		start[38] = 0.0073251;
		start[39] = 0.0044616;
		start[40] = 0.0096778;
		start[41] = 9.2041e-05;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0058886;
		start[7] = 0.0038891;
		start[8] = 0.009481;
		start[9] = 0.0023001;
		start[10] = 0.004408;
		start[11] = 0.0091957;
		start[12] = 0.0076885;
		start[13] = 0.0045114;
		start[14] = 0.0093733;
		start[15] = 0.0029159;
		start[16] = 0.0021486;
		start[17] = 0.0047567;
		start[18] = 0.0067164;
		start[19] = 0.0096921;
		start[20] = 0.0067873;
		start[21] = 0.0089383;
		start[22] = 0.0056667;
		start[23] = 0.0047191;
		start[24] = 0.0024763;
		start[25] = 0.0027185;
		start[26] = 0.0055154;
		start[27] = 0.00029949;
		start[28] = 0.0020964;
		start[29] = 0.009117;
		start[30] = 0.009041;
		start[31] = 0.0047226;
		start[32] = 0.0012428;
		start[33] = 0.00078607;
		start[34] = 0.0047344;
		start[35] = 0.0001734;
		start[36] = 0.0065802;
		start[37] = 0.0075204;
		start[38] = 0.0013333;
		start[39] = 0.0013617;
		start[40] = 0.0077338;
		start[41] = 0.00052698;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0038681;
		start[7] = 0.0023786;
		start[8] = 0.0095951;
		start[9] = 0.0039904;
		start[10] = 0.0077953;
		start[11] = 0.0044331;
		start[12] = 0.0096128;
		start[13] = 0.0046046;
		start[14] = 0.0027535;
		start[15] = 0.0024434;
		start[16] = 0.0066326;
		start[17] = 0.0030089;
		start[18] = 0.0037941;
		start[19] = 0.0019171;
		start[20] = 0.00982;
		start[21] = 0.00048416;
		start[22] = 0.0028728;
		start[23] = 0.0094756;
		start[24] = 0.0061909;
		start[25] = 0.0039779;
		start[26] = 0.0018868;
		start[27] = 0.0012525;
		start[28] = 0.007068;
		start[29] = 0.0072579;
		start[30] = 0.0097401;
		start[31] = 0.0045017;
		start[32] = 0.0093329;
		start[33] = 0.0051116;
		start[34] = 0.0060951;
		start[35] = 0.0065046;
		start[36] = 0.0016479;
		start[37] = 0.0062968;
		start[38] = 0.008832;
		start[39] = 0.0093663;
		start[40] = 0.0023071;
		start[41] = 0.0060984;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0097151;
		start[7] = 0.0082599;
		start[8] = 0.0077573;
		start[9] = 0.0037407;
		start[10] = 9.0432e-05;
		start[11] = 0.0065092;
		start[12] = 0.002952;
		start[13] = 0.0023992;
		start[14] = 0.009472;
		start[15] = 0.008833;
		start[16] = 0.0069504;
		start[17] = 0.00011837;
		start[18] = 0.0048333;
		start[19] = 0.0038676;
		start[20] = 0.0033638;
		start[21] = 0.00042791;
		start[22] = 0.0093115;
		start[23] = 0.0049365;
		start[24] = 0.0026756;
		start[25] = 0.0069435;
		start[26] = 0.0041491;
		start[27] = 0.0090687;
		start[28] = 0.0024909;
		start[29] = 0.0032501;
		start[30] = 0.0030183;
		start[31] = 0.0087793;
		start[32] = 0.0039475;
		start[33] = 0.0014261;
		start[34] = 0.0017283;
		start[35] = 0.00084515;
		start[36] = 0.00078573;
		start[37] = 0.001658;
		start[38] = 0.0091378;
		start[39] = 0.0022942;
		start[40] = 0.0075961;
		start[41] = 0.0049711;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0089854;
		start[7] = 0.0036785;
		start[8] = 0.0087748;
		start[9] = 0.00082372;
		start[10] = 0.0082867;
		start[11] = 0.0040989;
		start[12] = 0.009299;
		start[13] = 0.0029078;
		start[14] = 0.0031458;
		start[15] = 0.0097352;
		start[16] = 0.001843;
		start[17] = 0.0013987;
		start[18] = 0.0019381;
		start[19] = 0.0012226;
		start[20] = 0.0050696;
		start[21] = 0.0015315;
		start[22] = 0.0016463;
		start[23] = 0.0032233;
		start[24] = 0.0035494;
		start[25] = 0.0091551;
		start[26] = 0.0083379;
		start[27] = 0.00076495;
		start[28] = 0.008414;
		start[29] = 0.0029585;
		start[30] = 0.00076485;
		start[31] = 0.0029038;
		start[32] = 0.0061601;
		start[33] = 0.0012354;
		start[34] = 0.0086118;
		start[35] = 0.0044925;
		start[36] = 0.0075526;
		start[37] = 0.0064915;
		start[38] = 0.0021126;
		start[39] = 0.0030957;
		start[40] = 0.0044795;
		start[41] = 0.0073261;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0087504;
		start[7] = 0.0011307;
		start[8] = 0.0079698;
		start[9] = 0.00063838;
		start[10] = 0.0038619;
		start[11] = 0.0032328;
		start[12] = 0.0072822;
		start[13] = 0.0010241;
		start[14] = 0.0098052;
		start[15] = 0.0083519;
		start[16] = 0.0078744;
		start[17] = 0.0047054;
		start[18] = 0.00015131;
		start[19] = 0.0020648;
		start[20] = 0.0029705;
		start[21] = 0.0066842;
		start[22] = 0.0076958;
		start[23] = 0.0041196;
		start[24] = 0.0021042;
		start[25] = 0.00056853;
		start[26] = 0.0033064;
		start[27] = 0.0086538;
		start[28] = 0.0061254;
		start[29] = 0.0072949;
		start[30] = 0.0026481;
		start[31] = 0.0012799;
		start[32] = 0.0096703;
		start[33] = 0.0038913;
		start[34] = 0.0023286;
		start[35] = 0.009907;
		start[36] = 0.0040168;
		start[37] = 0.0099956;
		start[38] = 0.0087445;
		start[39] = 0.0064546;
		start[40] = 0.0058045;
		start[41] = 0.0012467;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0092654;
		start[7] = 0.0031171;
		start[8] = 0.0093395;
		start[9] = 0.004411;
		start[10] = 0.0068227;
		start[11] = 0.0073288;
		start[12] = 0.0085834;
		start[13] = 0.0022399;
		start[14] = 0.0092467;
		start[15] = 0.0097577;
		start[16] = 0.0019988;
		start[17] = 0.0018225;
		start[18] = 0.0086589;
		start[19] = 0.0077761;
		start[20] = 0.005756;
		start[21] = 0.0085278;
		start[22] = 0.0076032;
		start[23] = 0.00097367;
		start[24] = 0.00081963;
		start[25] = 0.0043098;
		start[26] = 0.0080566;
		start[27] = 0.002247;
		start[28] = 0.0064222;
		start[29] = 0.003066;
		start[30] = 0.0055666;
		start[31] = 0.0088505;
		start[32] = 0.0038383;
		start[33] = 0.0077627;
		start[34] = 0.0092965;
		start[35] = 0.0035137;
		start[36] = 0.0054806;
		start[37] = 0.0056448;
		start[38] = 0.0025153;
		start[39] = 0.0049184;
		start[40] = 0.0050314;
		start[41] = 0.0060672;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0032027;
		start[7] = 0.0044709;
		start[8] = 0.0054876;
		start[9] = 0.0054198;
		start[10] = 0.0025085;
		start[11] = 0.0084556;
		start[12] = 0.0048721;
		start[13] = 0.0080548;
		start[14] = 0.0037967;
		start[15] = 0.0018302;
		start[16] = 0.0052103;
		start[17] = 0.0017544;
		start[18] = 0.00074147;
		start[19] = 0.0097677;
		start[20] = 0.0024361;
		start[21] = 0.0024945;
		start[22] = 0.0083052;
		start[23] = 0.0085178;
		start[24] = 0.00035461;
		start[25] = 0.0044804;
		start[26] = 0.00093718;
		start[27] = 0.0099317;
		start[28] = 0.0060955;
		start[29] = 0.0047404;
		start[30] = 0.0018397;
		start[31] = 0.0012399;
		start[32] = 0.0054011;
		start[33] = 0.0089222;
		start[34] = 0.0049669;
		start[35] = 0.003659;
		start[36] = 0.0038482;
		start[37] = 0.0098839;
		start[38] = 0.00060339;
		start[39] = 0.0046367;
		start[40] = 0.0022532;
		start[41] = 0.0066478;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0084599;
		start[7] = 0.0056399;
		start[8] = 0.0076024;
		start[9] = 0.0068354;
		start[10] = 0.0057435;
		start[11] = 0.0029244;
		start[12] = 0.0020407;
		start[13] = 0.0029054;
		start[14] = 0.0042295;
		start[15] = 0.0014042;
		start[16] = 0.005022;
		start[17] = 0.0054305;
		start[18] = 0.0032247;
		start[19] = 0.0092309;
		start[20] = 0.0023955;
		start[21] = 0.0030365;
		start[22] = 0.0077915;
		start[23] = 0.001221;
		start[24] = 0.0056362;
		start[25] = 0.0088817;
		start[26] = 0.0086015;
		start[27] = 0.0018852;
		start[28] = 0.0069134;
		start[29] = 0.0014385;
		start[30] = 0.005807;
		start[31] = 0.005036;
		start[32] = 0.0011044;
		start[33] = 0.0047439;
		start[34] = 0.006456;
		start[35] = 0.003104;
		start[36] = 0.0031503;
		start[37] = 0.0014498;
		start[38] = 0.0080884;
		start[39] = 0.0037338;
		start[40] = 0.0059934;
		start[41] = 0.001413;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0078924;
		start[7] = 0.0064983;
		start[8] = 0.00753;
		start[9] = 0.0081901;
		start[10] = 0.0078345;
		start[11] = 0.0056477;
		start[12] = 0.0085923;
		start[13] = 0.0054978;
		start[14] = 0.0087416;
		start[15] = 0.0074807;
		start[16] = 0.0080764;
		start[17] = 0.0047394;
		start[18] = 0.0099571;
		start[19] = 0.00048362;
		start[20] = 0.0032937;
		start[21] = 0.0070214;
		start[22] = 0.0081996;
		start[23] = 0.0057928;
		start[24] = 0.0098338;
		start[25] = 0.0082114;
		start[26] = 0.004092;
		start[27] = 0.0075974;
		start[28] = 0.0050311;
		start[29] = 0.0045072;
		start[30] = 0.00202;
		start[31] = 0.00029707;
		start[32] = 0.0032088;
		start[33] = 0.002191;
		start[34] = 0.0092455;
		start[35] = 0.0078437;
		start[36] = 0.0037968;
		start[37] = 0.0024888;
		start[38] = 0.0031107;
		start[39] = 0.0089938;
		start[40] = 0.008559;
		start[41] = 0.0040371;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0060766;
		start[7] = 0.0047785;
		start[8] = 0.0084887;
		start[9] = 0.0078325;
		start[10] = 0.0038046;
		start[11] = 0.0032843;
		start[12] = 0.0081298;
		start[13] = 0.0038709;
		start[14] = 0.0078915;
		start[15] = 0.0055477;
		start[16] = 0.0054261;
		start[17] = 0.0053054;
		start[18] = 0.0039439;
		start[19] = 0.008025;
		start[20] = 0.0020251;
		start[21] = 0.00076322;
		start[22] = 0.007225;
		start[23] = 0.00085252;
		start[24] = 0.0065638;
		start[25] = 0.0039852;
		start[26] = 0.0028055;
		start[27] = 0.0053082;
		start[28] = 0.0089717;
		start[29] = 0.0073363;
		start[30] = 0.0088289;
		start[31] = 0.0066134;
		start[32] = 0.0041195;
		start[33] = 0.0031545;
		start[34] = 0.0039291;
		start[35] = 0.0082645;
		start[36] = 0.00396;
		start[37] = 0.0045599;
		start[38] = 0.0063578;
		start[39] = 0.0066969;
		start[40] = 0.0020852;
		start[41] = 0.0024931;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0068017;
		start[7] = 0.006619;
		start[8] = 0.0059855;
		start[9] = 0.0036655;
		start[10] = 0.0054821;
		start[11] = 0.0030505;
		start[12] = 0.002837;
		start[13] = 0.0080194;
		start[14] = 0.0088991;
		start[15] = 0.0066457;
		start[16] = 0.0060817;
		start[17] = 0.0003393;
		start[18] = 0.0015571;
		start[19] = 0.0069809;
		start[20] = 0.0092044;
		start[21] = 0.0053483;
		start[22] = 0.008922;
		start[23] = 0.00065595;
		start[24] = 0.004315;
		start[25] = 0.0031515;
		start[26] = 0.0053725;
		start[27] = 0.0070738;
		start[28] = 0.0034632;
		start[29] = 4.5968e-05;
		start[30] = 0.001245;
		start[31] = 0.0046947;
		start[32] = 0.0038235;
		start[33] = 0.0017672;
		start[34] = 0.0082428;
		start[35] = 0.0082321;
		start[36] = 0.001555;
		start[37] = 0.006708;
		start[38] = 0.00092767;
		start[39] = 0.00021524;
		start[40] = 0.0076837;
		start[41] = 0.0095718;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0043946;
		start[7] = 0.0034751;
		start[8] = 0.00097004;
		start[9] = 0.0062104;
		start[10] = 0.0038097;
		start[11] = 0.00063945;
		start[12] = 0.0049991;
		start[13] = 0.0078401;
		start[14] = 0.0033885;
		start[15] = 0.0026782;
		start[16] = 0.00013904;
		start[17] = 0.001075;
		start[18] = 0.0065138;
		start[19] = 0.009853;
		start[20] = 0.0036902;
		start[21] = 0.0097474;
		start[22] = 0.0013799;
		start[23] = 0.0075894;
		start[24] = 0.0075767;
		start[25] = 0.004532;
		start[26] = 0.002932;
		start[27] = 0.0061063;
		start[28] = 0.0030791;
		start[29] = 0.0062504;
		start[30] = 0.001246;
		start[31] = 0.0099383;
		start[32] = 0.0036766;
		start[33] = 0.0059547;
		start[34] = 0.0096497;
		start[35] = 0.00062242;
		start[36] = 0.0056917;
		start[37] = 0.0096797;
		start[38] = 0.0071808;
		start[39] = 0.0069913;
		start[40] = 0.0095926;
		start[41] = 0.007017;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.001513;
		start[7] = 0.0044436;
		start[8] = 0.0058612;
		start[9] = 0.0027729;
		start[10] = 0.0013621;
		start[11] = 0.0070224;
		start[12] = 0.0012055;
		start[13] = 0.0027658;
		start[14] = 0.0099075;
		start[15] = 0.0011171;
		start[16] = 0.0034126;
		start[17] = 0.0066622;
		start[18] = 0.006606;
		start[19] = 0.0088554;
		start[20] = 0.0026136;
		start[21] = 0.0077307;
		start[22] = 0.0087157;
		start[23] = 0.0045392;
		start[24] = 0.008244;
		start[25] = 0.0019883;
		start[26] = 0.0081799;
		start[27] = 2.9066e-05;
		start[28] = 0.0067848;
		start[29] = 0.0082587;
		start[30] = 0.0071268;
		start[31] = 0.0038917;
		start[32] = 0.00011703;
		start[33] = 0.0020709;
		start[34] = 0.0014817;
		start[35] = 0.0060616;
		start[36] = 0.0091769;
		start[37] = 0.0065307;
		start[38] = 0.0025948;
		start[39] = 0.008792;
		start[40] = 0.0067185;
		start[41] = 0.0040686;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0059519;
		start[7] = 0.0031476;
		start[8] = 0.0054932;
		start[9] = 0.0094913;
		start[10] = 0.0043526;
		start[11] = 0.0057748;
		start[12] = 0.0064626;
		start[13] = 0.0039486;
		start[14] = 0.0086829;
		start[15] = 0.0072453;
		start[16] = 0.00011892;
		start[17] = 0.0046319;
		start[18] = 0.0061168;
		start[19] = 0.0012043;
		start[20] = 0.00018092;
		start[21] = 0.00084492;
		start[22] = 0.0057989;
		start[23] = 0.0081136;
		start[24] = 0.0087248;
		start[25] = 0.0084243;
		start[26] = 0.0057011;
		start[27] = 0.0092988;
		start[28] = 0.0048303;
		start[29] = 0.0087102;
		start[30] = 0.0070357;
		start[31] = 0.00014524;
		start[32] = 0.0026128;
		start[33] = 0.0096301;
		start[34] = 0.0015502;
		start[35] = 0.0087611;
		start[36] = 0.0037088;
		start[37] = 0.0027359;
		start[38] = 0.0067824;
		start[39] = 0.0089571;
		start[40] = 0.00011711;
		start[41] = 0.0044955;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.001751;
		start[7] = 0.0028771;
		start[8] = 0.0036041;
		start[9] = 0.0075955;
		start[10] = 0.0047274;
		start[11] = 0.0045922;
		start[12] = 0.0026471;
		start[13] = 0.0016757;
		start[14] = 0.0080078;
		start[15] = 0.0079264;
		start[16] = 0.0023076;
		start[17] = 0.004252;
		start[18] = 0.009377;
		start[19] = 0.0071557;
		start[20] = 0.00092945;
		start[21] = 0.0093321;
		start[22] = 0.0037835;
		start[23] = 0.0018839;
		start[24] = 0.0058009;
		start[25] = 0.0062599;
		start[26] = 0.0043073;
		start[27] = 0.0076981;
		start[28] = 0.0011554;
		start[29] = 0.0094504;
		start[30] = 0.00086599;
		start[31] = 0.0017478;
		start[32] = 0.0039045;
		start[33] = 0.0065808;
		start[34] = 0.005617;
		start[35] = 0.0041922;
		start[36] = 0.00080648;
		start[37] = 0.0071724;
		start[38] = 0.0089358;
		start[39] = 0.0077155;
		start[40] = 0.0058756;
		start[41] = 0.0022956;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0063328;
		start[7] = 0.00080607;
		start[8] = 0.0025225;
		start[9] = 0.006356;
		start[10] = 0.0026589;
		start[11] = 0.0096118;
		start[12] = 0.0071369;
		start[13] = 0.0035381;
		start[14] = 0.0027992;
		start[15] = 0.00010764;
		start[16] = 0.0078777;
		start[17] = 0.0098089;
		start[18] = 0.00075138;
		start[19] = 0.0046721;
		start[20] = 0.00038428;
		start[21] = 0.0050389;
		start[22] = 0.0069806;
		start[23] = 0.0078967;
		start[24] = 0.007275;
		start[25] = 0.0065368;
		start[26] = 0.0097649;
		start[27] = 0.0025145;
		start[28] = 0.0045609;
		start[29] = 0.00071803;
		start[30] = 0.0092001;
		start[31] = 0.009965;
		start[32] = 0.0039572;
		start[33] = 0.00024498;
		start[34] = 0.0087261;
		start[35] = 0.0031626;
		start[36] = 0.0085721;
		start[37] = 0.0085131;
		start[38] = 0.0068918;
		start[39] = 0.0090495;
		start[40] = 0.0087316;
		start[41] = 0.0084016;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0080191;
		start[7] = 0.0055664;
		start[8] = 0.0075244;
		start[9] = 0.0091548;
		start[10] = 0.0040461;
		start[11] = 0.00090146;
		start[12] = 0.009361;
		start[13] = 0.0058463;
		start[14] = 0.0023365;
		start[15] = 0.0034403;
		start[16] = 0.005089;
		start[17] = 0.0077829;
		start[18] = 0.0014971;
		start[19] = 0.0054364;
		start[20] = 0.0038476;
		start[21] = 0.0016126;
		start[22] = 0.0092049;
		start[23] = 0.0016073;
		start[24] = 0.003961;
		start[25] = 0.0014898;
		start[26] = 0.0072847;
		start[27] = 0.0098853;
		start[28] = 0.00016927;
		start[29] = 0.0090358;
		start[30] = 0.0034285;
		start[31] = 0.0086096;
		start[32] = 0.009598;
		start[33] = 0.0055997;
		start[34] = 0.0013325;
		start[35] = 0.005748;
		start[36] = 0.0048542;
		start[37] = 0.00021933;
		start[38] = 0.0046376;
		start[39] = 0.0052805;
		start[40] = 0.0051453;
		start[41] = 0.0089856;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0015173;
		start[7] = 0.0085148;
		start[8] = 0.0019954;
		start[9] = 0.0035332;
		start[10] = 0.00042619;
		start[11] = 0.0015027;
		start[12] = 0.0023036;
		start[13] = 0.0080738;
		start[14] = 0.0047853;
		start[15] = 0.0050003;
		start[16] = 0.0070528;
		start[17] = 0.0011561;
		start[18] = 0.0053664;
		start[19] = 0.0052039;
		start[20] = 0.0095702;
		start[21] = 0.001794;
		start[22] = 0.0053453;
		start[23] = 0.0083851;
		start[24] = 0.0067063;
		start[25] = 0.0053805;
		start[26] = 0.0018866;
		start[27] = 0.0013175;
		start[28] = 0.0011992;
		start[29] = 0.0063768;
		start[30] = 0.0016558;
		start[31] = 0.0064778;
		start[32] = 0.00070909;
		start[33] = 0.0094722;
		start[34] = 0.004113;
		start[35] = 0.0062251;
		start[36] = 0.0017866;
		start[37] = 0.0060068;
		start[38] = 0.005816;
		start[39] = 0.00030731;
		start[40] = 0.0098003;
		start[41] = 0.0058275;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0062755;
		start[7] = 0.0027057;
		start[8] = 0.003234;
		start[9] = 0.0047575;
		start[10] = 0.0076951;
		start[11] = 0.008465;
		start[12] = 0.0092758;
		start[13] = 0.0074799;
		start[14] = 0.0085571;
		start[15] = 0.005146;
		start[16] = 0.0065945;
		start[17] = 0.00211;
		start[18] = 0.0023555;
		start[19] = 0.0063768;
		start[20] = 0.0059558;
		start[21] = 0.0090465;
		start[22] = 0.0091674;
		start[23] = 0.0045796;
		start[24] = 0.0046712;
		start[25] = 0.0017113;
		start[26] = 0.0017553;
		start[27] = 0.0083079;
		start[28] = 0.0014249;
		start[29] = 0.0082069;
		start[30] = 0.0067722;
		start[31] = 0.0028721;
		start[32] = 0.0076335;
		start[33] = 0.0066105;
		start[34] = 0.00062358;
		start[35] = 0.0094781;
		start[36] = 0.0096874;
		start[37] = 0.0023886;
		start[38] = 0.0026485;
		start[39] = 0.0062404;
		start[40] = 0.0023554;
		start[41] = 0.0076869;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0014148;
		start[7] = 0.0098138;
		start[8] = 0.0047748;
		start[9] = 0.0044342;
		start[10] = 0.0024391;
		start[11] = 0.0076177;
		start[12] = 0.0069423;
		start[13] = 0.0051161;
		start[14] = 0.0042039;
		start[15] = 0.00066888;
		start[16] = 0.0092106;
		start[17] = 0.0031285;
		start[18] = 0.0099576;
		start[19] = 0.0063178;
		start[20] = 0.0032369;
		start[21] = 0.0016081;
		start[22] = 0.0080708;
		start[23] = 0.0088216;
		start[24] = 0.0059002;
		start[25] = 0.0018957;
		start[26] = 0.0023785;
		start[27] = 0.0054118;
		start[28] = 0.0089048;
		start[29] = 0.0060846;
		start[30] = 0.0017152;
		start[31] = 0.0044385;
		start[32] = 0.0083174;
		start[33] = 0.0083514;
		start[34] = 0.0068567;
		start[35] = 0.0085774;
		start[36] = 0.0062584;
		start[37] = 0.0036646;
		start[38] = 0.0028402;
		start[39] = 0.0090292;
		start[40] = 0.0072356;
		start[41] = 0.0020545;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0020287;
		start[7] = 0.0014287;
		start[8] = 0.0040115;
		start[9] = 0.0008247;
		start[10] = 0.0071652;
		start[11] = 0.0064426;
		start[12] = 0.0075427;
		start[13] = 0.0019581;
		start[14] = 0.0026447;
		start[15] = 0.00064223;
		start[16] = 0.0054086;
		start[17] = 0.0053148;
		start[18] = 0.0028969;
		start[19] = 0.0051716;
		start[20] = 0.0083033;
		start[21] = 0.0085554;
		start[22] = 0.0092974;
		start[23] = 0.0085912;
		start[24] = 0.0039788;
		start[25] = 0.0015671;
		start[26] = 0.0094296;
		start[27] = 0.0096679;
		start[28] = 0.0026931;
		start[29] = 0.0095355;
		start[30] = 0.0095523;
		start[31] = 0.0075299;
		start[32] = 0.0078164;
		start[33] = 0.0057277;
		start[34] = 0.00015576;
		start[35] = 0.0043495;
		start[36] = 0.006313;
		start[37] = 0.0082416;
		start[38] = 0.0020063;
		start[39] = 0.004119;
		start[40] = 0.0096014;
		start[41] = 0.00065616;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0041242;
		start[7] = 0.007797;
		start[8] = 0.0055069;
		start[9] = 0.006843;
		start[10] = 0.0054999;
		start[11] = 0.0085925;
		start[12] = 0.0031886;
		start[13] = 0.0021169;
		start[14] = 0.004011;
		start[15] = 0.0081428;
		start[16] = 0.0063933;
		start[17] = 0.0037516;
		start[18] = 0.0034091;
		start[19] = 0.0022512;
		start[20] = 0.0073601;
		start[21] = 0.00019166;
		start[22] = 0.0011111;
		start[23] = 0.0025884;
		start[24] = 0.0061117;
		start[25] = 0.00064254;
		start[26] = 0.00067107;
		start[27] = 0.0043827;
		start[28] = 0.0099666;
		start[29] = 0.0044613;
		start[30] = 0.0018909;
		start[31] = 0.0011168;
		start[32] = 0.0027362;
		start[33] = 6.1126e-05;
		start[34] = 0.0025218;
		start[35] = 0.00092773;
		start[36] = 0.0078324;
		start[37] = 0.0095658;
		start[38] = 0.0069033;
		start[39] = 0.0011031;
		start[40] = 0.0089287;
		start[41] = 0.0096226;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0039645;
		start[7] = 0.0085354;
		start[8] = 0.0056203;
		start[9] = 0.0011708;
		start[10] = 0.0053799;
		start[11] = 0.0083782;
		start[12] = 0.0054909;
		start[13] = 0.0044455;
		start[14] = 0.0092539;
		start[15] = 0.0098794;
		start[16] = 0.0045211;
		start[17] = 0.0041889;
		start[18] = 0.0059507;
		start[19] = 0.0033945;
		start[20] = 0.0043776;
		start[21] = 0.00070939;
		start[22] = 0.0017966;
		start[23] = 0.0097132;
		start[24] = 0.005533;
		start[25] = 0.0046333;
		start[26] = 0.0038604;
		start[27] = 0.0060812;
		start[28] = 0.0047347;
		start[29] = 0.0069617;
		start[30] = 0.0067782;
		start[31] = 0.0066753;
		start[32] = 0.0073733;
		start[33] = 0.0018472;
		start[34] = 0.0029791;
		start[35] = 0.0031739;
		start[36] = 0.0085311;
		start[37] = 0.0086372;
		start[38] = 0.0043071;
		start[39] = 0.0097552;
		start[40] = 0.0076654;
		start[41] = 0.0035008;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0034283;
		start[7] = 0.0052173;
		start[8] = 0.0070525;
		start[9] = 0.0077214;
		start[10] = 0.009702;
		start[11] = 1.7289e-05;
		start[12] = 0.0082891;
		start[13] = 0.0072759;
		start[14] = 0.0091027;
		start[15] = 0.0047664;
		start[16] = 0.0064523;
		start[17] = 0.0037211;
		start[18] = 0.0042303;
		start[19] = 0.0081376;
		start[20] = 0.0014063;
		start[21] = 0.002434;
		start[22] = 0.0096521;
		start[23] = 0.0029022;
		start[24] = 0.0017896;
		start[25] = 0.00017768;
		start[26] = 0.0075677;
		start[27] = 0.0026083;
		start[28] = 0.004875;
		start[29] = 0.0095777;
		start[30] = 0.0053546;
		start[31] = 0.0010883;
		start[32] = 0.0069405;
		start[33] = 0.0031937;
		start[34] = 0.0037606;
		start[35] = 0.002214;
		start[36] = 0.0039484;
		start[37] = 0.0089857;
		start[38] = 0.0092366;
		start[39] = 0.0057637;
		start[40] = 0.00063542;
		start[41] = 0.002284;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0059592;
		start[7] = 0.0072098;
		start[8] = 0.0091445;
		start[9] = 0.004615;
		start[10] = 0.0016581;
		start[11] = 0.0034488;
		start[12] = 0.008115;
		start[13] = 0.0014276;
		start[14] = 0.0075049;
		start[15] = 0.0077399;
		start[16] = 0.0043891;
		start[17] = 0.00093282;
		start[18] = 0.003883;
		start[19] = 0.00035565;
		start[20] = 0.009344;
		start[21] = 0.007504;
		start[22] = 0.00585;
		start[23] = 0.0064137;
		start[24] = 0.0018499;
		start[25] = 0.0019184;
		start[26] = 0.0051579;
		start[27] = 0.0057533;
		start[28] = 0.0068104;
		start[29] = 0.0034015;
		start[30] = 0.0031754;
		start[31] = 0.0092398;
		start[32] = 0.0036005;
		start[33] = 0.0025568;
		start[34] = 0.0070978;
		start[35] = 0.0062128;
		start[36] = 0.0028597;
		start[37] = 0.0073842;
		start[38] = 0.0071806;
		start[39] = 0.00049596;
		start[40] = 0.0043498;
		start[41] = 0.008572;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0071331;
		start[7] = 0.0019484;
		start[8] = 0.0098999;
		start[9] = 0.0062821;
		start[10] = 0.00868;
		start[11] = 0.0014879;
		start[12] = 0.0069499;
		start[13] = 0.005891;
		start[14] = 0.0015419;
		start[15] = 0.0053578;
		start[16] = 0.0019372;
		start[17] = 0.0074078;
		start[18] = 0.0029103;
		start[19] = 0.0095821;
		start[20] = 0.008126;
		start[21] = 0.0022341;
		start[22] = 0.0020382;
		start[23] = 0.0093935;
		start[24] = 0.00050887;
		start[25] = 0.008137;
		start[26] = 0.00011781;
		start[27] = 0.00085728;
		start[28] = 0.0071634;
		start[29] = 0.0098278;
		start[30] = 0.0058052;
		start[31] = 0.00050931;
		start[32] = 0.006514;
		start[33] = 0.003312;
		start[34] = 0.0019625;
		start[35] = 0.001076;
		start[36] = 0.0053459;
		start[37] = 5.5727e-05;
		start[38] = 0.00068302;
		start[39] = 0.002772;
		start[40] = 0.0098436;
		start[41] = 0.0038873;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0059038;
		start[7] = 0.0058662;
		start[8] = 0.0033432;
		start[9] = 0.0064122;
		start[10] = 0.0015551;
		start[11] = 0.0066105;
		start[12] = 0.0071881;
		start[13] = 0.0026292;
		start[14] = 0.0045441;
		start[15] = 0.0083355;
		start[16] = 0.0052018;
		start[17] = 0.0083744;
		start[18] = 0.0069452;
		start[19] = 0.0019705;
		start[20] = 0.0004642;
		start[21] = 0.0083387;
		start[22] = 0.00094681;
		start[23] = 0.0047571;
		start[24] = 0.0016362;
		start[25] = 0.0032404;
		start[26] = 0.0009816;
		start[27] = 0.0099;
		start[28] = 0.0089393;
		start[29] = 0.0037398;
		start[30] = 0.00029375;
		start[31] = 0.0032981;
		start[32] = 0.0039833;
		start[33] = 0.0087095;
		start[34] = 0.0050664;
		start[35] = 0.0031936;
		start[36] = 0.0070691;
		start[37] = 0.0044403;
		start[38] = 0.0099665;
		start[39] = 0.0021752;
		start[40] = 0.0035928;
		start[41] = 0.0094379;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.006144;
		start[7] = 0.0066375;
		start[8] = 0.0047299;
		start[9] = 0.0044044;
		start[10] = 0.004295;
		start[11] = 0.0010625;
		start[12] = 0.0053948;
		start[13] = 0.0087203;
		start[14] = 0.0081628;
		start[15] = 0.0046275;
		start[16] = 0.005151;
		start[17] = 0.0045646;
		start[18] = 0.0019444;
		start[19] = 0.0086065;
		start[20] = 0.0001744;
		start[21] = 0.0034206;
		start[22] = 0.002642;
		start[23] = 0.0014719;
		start[24] = 0.0095048;
		start[25] = 0.0031873;
		start[26] = 0.0084183;
		start[27] = 0.0065851;
		start[28] = 0.0033464;
		start[29] = 0.00036626;
		start[30] = 0.0050588;
		start[31] = 0.0060926;
		start[32] = 0.0066536;
		start[33] = 0.00087219;
		start[34] = 0.0017136;
		start[35] = 0.0099471;
		start[36] = 0.0057204;
		start[37] = 0.0098279;
		start[38] = 0.0023566;
		start[39] = 0.0059815;
		start[40] = 0.0092258;
		start[41] = 0.0011702;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0039434;
		start[7] = 0.0079164;
		start[8] = 0.0090592;
		start[9] = 0.0017712;
		start[10] = 0.0011487;
		start[11] = 0.00030847;
		start[12] = 0.0031874;
		start[13] = 0.0085951;
		start[14] = 0.0033461;
		start[15] = 0.0047573;
		start[16] = 0.0094405;
		start[17] = 0.00033652;
		start[18] = 0.0063177;
		start[19] = 0.0081295;
		start[20] = 0.0050892;
		start[21] = 0.0097084;
		start[22] = 0.0086822;
		start[23] = 0.002388;
		start[24] = 0.0034422;
		start[25] = 0.00046482;
		start[26] = 0.0056775;
		start[27] = 0.0058478;
		start[28] = 0.00033052;
		start[29] = 0.0083684;
		start[30] = 0.0065876;
		start[31] = 0.0092259;
		start[32] = 0.0093722;
		start[33] = 0.0002185;
		start[34] = 0.00088849;
		start[35] = 0.0018665;
		start[36] = 0.0090338;
		start[37] = 0.0085996;
		start[38] = 0.0055618;
		start[39] = 0.0024298;
		start[40] = 0.0088093;
		start[41] = 0.006454;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0030398;
		start[7] = 0.0046881;
		start[8] = 0.0033444;
		start[9] = 0.0036277;
		start[10] = 0.0075794;
		start[11] = 0.00035253;
		start[12] = 0.008993;
		start[13] = 0.0019891;
		start[14] = 0.0097664;
		start[15] = 0.00076889;
		start[16] = 0.0021805;
		start[17] = 0.0014055;
		start[18] = 0.0038944;
		start[19] = 0.0071207;
		start[20] = 0.00032647;
		start[21] = 0.0064976;
		start[22] = 0.004911;
		start[23] = 0.0096851;
		start[24] = 0.0093368;
		start[25] = 0.0029679;
		start[26] = 0.0019853;
		start[27] = 0.0021922;
		start[28] = 0.0098046;
		start[29] = 0.00024148;
		start[30] = 0.0039415;
		start[31] = 0.0024687;
		start[32] = 0.0005973;
		start[33] = 0.0027872;
		start[34] = 0.00088529;
		start[35] = 0.00032039;
		start[36] = 0.0092794;
		start[37] = 0.0013795;
		start[38] = 0.0074245;
		start[39] = 0.0067181;
		start[40] = 0.009259;
		start[41] = 0.0020932;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0052873;
		start[7] = 0.0020288;
		start[8] = 0.00068214;
		start[9] = 0.0081821;
		start[10] = 0.0045113;
		start[11] = 0.0081256;
		start[12] = 0.0044083;
		start[13] = 0.0033261;
		start[14] = 0.0095406;
		start[15] = 0.0097121;
		start[16] = 0.0042096;
		start[17] = 0.0099367;
		start[18] = 0.0038145;
		start[19] = 0.0086605;
		start[20] = 0.0015678;
		start[21] = 0.0034971;
		start[22] = 0.0041371;
		start[23] = 0.0071808;
		start[24] = 0.005634;
		start[25] = 0.0035008;
		start[26] = 0.0029109;
		start[27] = 0.0048456;
		start[28] = 0.0086103;
		start[29] = 0.0089022;
		start[30] = 0.005336;
		start[31] = 0.0096251;
		start[32] = 0.0090647;
		start[33] = 0.0037398;
		start[34] = 0.0016023;
		start[35] = 0.0095722;
		start[36] = 0.0059532;
		start[37] = 0.0035544;
		start[38] = 0.0011449;
		start[39] = 0.0095684;
		start[40] = 6.7703e-05;
		start[41] = 0.0064837;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0019123;
		start[7] = 0.0073018;
		start[8] = 0.0041391;
		start[9] = 0.002785;
		start[10] = 0.0015963;
		start[11] = 0.0037056;
		start[12] = 0.00079935;
		start[13] = 0.0014668;
		start[14] = 0.0082214;
		start[15] = 0.004665;
		start[16] = 0.0091474;
		start[17] = 0.0017529;
		start[18] = 0.0078214;
		start[19] = 0.00049377;
		start[20] = 0.0025818;
		start[21] = 0.0072188;
		start[22] = 0.0029332;
		start[23] = 0.0040896;
		start[24] = 0.0028676;
		start[25] = 0.0014145;
		start[26] = 0.0094964;
		start[27] = 0.0042126;
		start[28] = 0.0090043;
		start[29] = 0.0077896;
		start[30] = 0.0092197;
		start[31] = 0.0056031;
		start[32] = 0.007788;
		start[33] = 0.00055745;
		start[34] = 0.0074246;
		start[35] = 0.0029157;
		start[36] = 0.00024893;
		start[37] = 0.000209;
		start[38] = 0.0027976;
		start[39] = 0.0053213;
		start[40] = 0.0075132;
		start[41] = 0.0015368;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0095662;
		start[7] = 0.0018023;
		start[8] = 0.0039874;
		start[9] = 0.0054164;
		start[10] = 0.0015591;
		start[11] = 0.009266;
		start[12] = 0.0019411;
		start[13] = 0.0049991;
		start[14] = 0.0018185;
		start[15] = 0.0040107;
		start[16] = 0.0067836;
		start[17] = 0.00062866;
		start[18] = 0.0099589;
		start[19] = 0.0029793;
		start[20] = 0.0067631;
		start[21] = 0.0085574;
		start[22] = 0.0018798;
		start[23] = 0.0085887;
		start[24] = 0.0069945;
		start[25] = 9.5252e-05;
		start[26] = 0.00993;
		start[27] = 0.0027055;
		start[28] = 0.0082261;
		start[29] = 0.0069925;
		start[30] = 0.0080895;
		start[31] = 0.0027867;
		start[32] = 0.0008493;
		start[33] = 0.0002241;
		start[34] = 0.0020143;
		start[35] = 0.0032668;
		start[36] = 0.0043262;
		start[37] = 0.0059559;
		start[38] = 0.0006597;
		start[39] = 0.0083615;
		start[40] = 0.0039661;
		start[41] = 0.0077782;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.00048063;
		start[7] = 0.00060349;
		start[8] = 0.0012245;
		start[9] = 0.0019724;
		start[10] = 0.0031942;
		start[11] = 0.007257;
		start[12] = 0.0055209;
		start[13] = 0.0043532;
		start[14] = 0.0031339;
		start[15] = 0.00024476;
		start[16] = 0.009191;
		start[17] = 0.0019523;
		start[18] = 0.0066367;
		start[19] = 0.0014817;
		start[20] = 0.0022315;
		start[21] = 0.0020297;
		start[22] = 0.0077753;
		start[23] = 0.0035645;
		start[24] = 0.006549;
		start[25] = 0.00046868;
		start[26] = 0.00044097;
		start[27] = 0.00038174;
		start[28] = 0.0058727;
		start[29] = 0.0066558;
		start[30] = 0.0096957;
		start[31] = 0.002004;
		start[32] = 0.0033353;
		start[33] = 0.0055144;
		start[34] = 0.0068626;
		start[35] = 0.00869;
		start[36] = 0.0012401;
		start[37] = 0.0014381;
		start[38] = 0.00082497;
		start[39] = 0.0054577;
		start[40] = 0.0020773;
		start[41] = 0.0023234;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0056321;
		start[7] = 0.0077582;
		start[8] = 0.0012364;
		start[9] = 0.0021773;
		start[10] = 0.0083337;
		start[11] = 0.0034089;
		start[12] = 0.0027074;
		start[13] = 0.0089088;
		start[14] = 0.0089583;
		start[15] = 0.0086576;
		start[16] = 0.0052733;
		start[17] = 0.0046163;
		start[18] = 0.0054708;
		start[19] = 0.008058;
		start[20] = 0.002766;
		start[21] = 0.001317;
		start[22] = 0.0075769;
		start[23] = 0.0011699;
		start[24] = 0.0076942;
		start[25] = 0.009142;
		start[26] = 0.0097466;
		start[27] = 0.0056204;
		start[28] = 0.0027015;
		start[29] = 0.0061783;
		start[30] = 0.0046609;
		start[31] = 0.0013178;
		start[32] = 0.0041731;
		start[33] = 0.0069383;
		start[34] = 0.0089285;
		start[35] = 0.00039122;
		start[36] = 0.0099279;
		start[37] = 0.0096633;
		start[38] = 0.0070276;
		start[39] = 0.0065847;
		start[40] = 0.0058806;
		start[41] = 0.0098727;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0082426;
		start[7] = 0.002849;
		start[8] = 0.0028305;
		start[9] = 0.0086502;
		start[10] = 0.004973;
		start[11] = 0.0039641;
		start[12] = 0.0059371;
		start[13] = 0.0064111;
		start[14] = 0.0045863;
		start[15] = 0.0045649;
		start[16] = 0.0091218;
		start[17] = 0.005523;
		start[18] = 0.0056476;
		start[19] = 0.0029351;
		start[20] = 0.0014493;
		start[21] = 0.0032103;
		start[22] = 0.0060957;
		start[23] = 0.007192;
		start[24] = 0.0089397;
		start[25] = 0.0030688;
		start[26] = 0.0090905;
		start[27] = 0.0055215;
		start[28] = 0.0011994;
		start[29] = 0.00045751;
		start[30] = 0.00042501;
		start[31] = 0.00038032;
		start[32] = 0.0027706;
		start[33] = 0.0095106;
		start[34] = 0.0050936;
		start[35] = 0.0082149;
		start[36] = 0.0079027;
		start[37] = 0.0064657;
		start[38] = 0.0015705;
		start[39] = 0.0040842;
		start[40] = 0.0070527;
		start[41] = 0.004466;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0025164;
		start[7] = 0.00044645;
		start[8] = 0.0095861;
		start[9] = 0.007283;
		start[10] = 0.0080286;
		start[11] = 0.0047155;
		start[12] = 0.0079277;
		start[13] = 0.00095988;
		start[14] = 0.0041696;
		start[15] = 0.00084094;
		start[16] = 0.0077048;
		start[17] = 0.0025881;
		start[18] = 0.0062474;
		start[19] = 0.0014512;
		start[20] = 0.0025162;
		start[21] = 0.0057628;
		start[22] = 8.4363e-05;
		start[23] = 0.00046156;
		start[24] = 0.0092417;
		start[25] = 0.0070774;
		start[26] = 0.0041213;
		start[27] = 0.0080268;
		start[28] = 0.0079838;
		start[29] = 0.0085314;
		start[30] = 0.0074906;
		start[31] = 0.0086385;
		start[32] = 0.0032732;
		start[33] = 0.0039832;
		start[34] = 0.0024458;
		start[35] = 0.0047284;
		start[36] = 0.0079712;
		start[37] = 0.0013249;
		start[38] = 0.0079062;
		start[39] = 0.0088159;
		start[40] = 0.00089599;
		start[41] = 0.00074665;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0045398;
		start[7] = 0.0051395;
		start[8] = 0.0087103;
		start[9] = 0.0028451;
		start[10] = 0.0074448;
		start[11] = 0.0096905;
		start[12] = 0.0085677;
		start[13] = 0.0069998;
		start[14] = 0.0022781;
		start[15] = 0.00044913;
		start[16] = 0.0033512;
		start[17] = 0.0090848;
		start[18] = 0.0071398;
		start[19] = 0.0096895;
		start[20] = 0.00018094;
		start[21] = 0.0033046;
		start[22] = 0.0050811;
		start[23] = 0.0048948;
		start[24] = 0.0056916;
		start[25] = 0.0095681;
		start[26] = 0.0094018;
		start[27] = 0.00013012;
		start[28] = 0.0054523;
		start[29] = 0.0019114;
		start[30] = 0.0048607;
		start[31] = 0.00020914;
		start[32] = 0.0066903;
		start[33] = 0.00046091;
		start[34] = 0.0018058;
		start[35] = 0.0035389;
		start[36] = 0.0087709;
		start[37] = 0.0024244;
		start[38] = 0.0089735;
		start[39] = 0.0087913;
		start[40] = 0.0070548;
		start[41] = 0.0015352;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0098959;
		start[7] = 0.0007357;
		start[8] = 0.0023246;
		start[9] = 0.0098923;
		start[10] = 0.0060488;
		start[11] = 0.0029281;
		start[12] = 0.00094956;
		start[13] = 0.00011406;
		start[14] = 0.0059257;
		start[15] = 0.0006195;
		start[16] = 0.0097769;
		start[17] = 0.0070831;
		start[18] = 0.0028972;
		start[19] = 0.0050571;
		start[20] = 0.0079229;
		start[21] = 0.0085427;
		start[22] = 0.002769;
		start[23] = 0.0089052;
		start[24] = 0.0051896;
		start[25] = 0.0075164;
		start[26] = 0.0054926;
		start[27] = 0.0011179;
		start[28] = 0.0090483;
		start[29] = 0.00070971;
		start[30] = 0.0072638;
		start[31] = 0.0078406;
		start[32] = 0.007387;
		start[33] = 0.0071184;
		start[34] = 0.0099886;
		start[35] = 0.0070081;
		start[36] = 0.0073831;
		start[37] = 0.0010477;
		start[38] = 0.0065277;
		start[39] = 0.0092089;
		start[40] = 0.0068877;
		start[41] = 0.0016798;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0079453;
		start[7] = 0.0077035;
		start[8] = 0.0034931;
		start[9] = 0.0036186;
		start[10] = 0.0039017;
		start[11] = 0.0082585;
		start[12] = 0.005898;
		start[13] = 0.0014541;
		start[14] = 0.00082093;
		start[15] = 0.0010798;
		start[16] = 0.0096214;
		start[17] = 0.0032375;
		start[18] = 0.0055497;
		start[19] = 0.0086992;
		start[20] = 0.004259;
		start[21] = 0.0058856;
		start[22] = 0.0067838;
		start[23] = 0.00034539;
		start[24] = 0.00094896;
		start[25] = 0.0076223;
		start[26] = 0.0065086;
		start[27] = 0.0016194;
		start[28] = 0.0095376;
		start[29] = 0.0034544;
		start[30] = 0.0097059;
		start[31] = 0.00058498;
		start[32] = 0.0016609;
		start[33] = 0.00024889;
		start[34] = 0.0068545;
		start[35] = 0.0051819;
		start[36] = 0.0080862;
		start[37] = 0.0098533;
		start[38] = 0.0077488;
		start[39] = 0.0073028;
		start[40] = 0.0004588;
		start[41] = 0.0096576;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0059207;
		start[7] = 0.0097516;
		start[8] = 0.0019247;
		start[9] = 0.0092989;
		start[10] = 0.0053738;
		start[11] = 0.0089475;
		start[12] = 0.007625;
		start[13] = 0.0010098;
		start[14] = 0.0040022;
		start[15] = 0.00085064;
		start[16] = 0.0013353;
		start[17] = 0.00049647;
		start[18] = 0.0056861;
		start[19] = 0.0015284;
		start[20] = 0.00054442;
		start[21] = 0.0040048;
		start[22] = 0.0047144;
		start[23] = 0.002042;
		start[24] = 0.0018297;
		start[25] = 0.0077344;
		start[26] = 0.006067;
		start[27] = 0.0091094;
		start[28] = 0.006103;
		start[29] = 0.0042667;
		start[30] = 0.0037965;
		start[31] = 0.0094422;
		start[32] = 0.0099737;
		start[33] = 0.0019832;
		start[34] = 0.0099117;
		start[35] = 0.0094847;
		start[36] = 0.0026645;
		start[37] = 0.0010504;
		start[38] = 0.0039157;
		start[39] = 0.0066998;
		start[40] = 0.0038136;
		start[41] = 0.0076011;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0043607;
		start[7] = 0.0057563;
		start[8] = 0.0031379;
		start[9] = 0.0053558;
		start[10] = 0.0082329;
		start[11] = 0.0013146;
		start[12] = 0.0065782;
		start[13] = 0.0075817;
		start[14] = 0.0052654;
		start[15] = 0.0018487;
		start[16] = 0.00052314;
		start[17] = 0.0042373;
		start[18] = 0.0028309;
		start[19] = 0.0029989;
		start[20] = 0.0044961;
		start[21] = 0.0028948;
		start[22] = 0.0069706;
		start[23] = 0.0095735;
		start[24] = 0.007239;
		start[25] = 0.0013091;
		start[26] = 0.0047329;
		start[27] = 0.0058322;
		start[28] = 0.0054351;
		start[29] = 0.00073844;
		start[30] = 0.0059319;
		start[31] = 0.0057278;
		start[32] = 0.0075302;
		start[33] = 0.00082365;
		start[34] = 0.0035229;
		start[35] = 0.00078196;
		start[36] = 0.0088202;
		start[37] = 0.0014554;
		start[38] = 0.0083565;
		start[39] = 0.0031395;
		start[40] = 0.0044742;
		start[41] = 0.0068333;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0086965;
		start[7] = 0.0028454;
		start[8] = 0.0022972;
		start[9] = 0.0053915;
		start[10] = 0.0037989;
		start[11] = 0.001008;
		start[12] = 0.008672;
		start[13] = 0.0042456;
		start[14] = 0.0014051;
		start[15] = 0.0041606;
		start[16] = 0.0043579;
		start[17] = 0.0020097;
		start[18] = 0.00067972;
		start[19] = 0.0097726;
		start[20] = 0.0074466;
		start[21] = 0.0054852;
		start[22] = 0.0091032;
		start[23] = 0.0017979;
		start[24] = 0.004015;
		start[25] = 0.0021929;
		start[26] = 0.0080705;
		start[27] = 0.0036113;
		start[28] = 0.0034262;
		start[29] = 0.0074622;
		start[30] = 0.0070635;
		start[31] = 0.0059239;
		start[32] = 0.0029506;
		start[33] = 0.0042823;
		start[34] = 0.0023308;
		start[35] = 0.0029715;
		start[36] = 0.0040059;
		start[37] = 0.0093926;
		start[38] = 0.0063054;
		start[39] = 0.009044;
		start[40] = 0.0082195;
		start[41] = 0.0025582;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0038574;
		start[7] = 0.0012534;
		start[8] = 0.0081074;
		start[9] = 0.0095877;
		start[10] = 0.0049424;
		start[11] = 0.00017477;
		start[12] = 0.0088918;
		start[13] = 0.009656;
		start[14] = 0.0062613;
		start[15] = 0.0045539;
		start[16] = 0.0053798;
		start[17] = 0.0037328;
		start[18] = 0.0088667;
		start[19] = 0.0095617;
		start[20] = 0.008502;
		start[21] = 0.0081772;
		start[22] = 0.0058824;
		start[23] = 0.0022467;
		start[24] = 0.0021887;
		start[25] = 0.0031629;
		start[26] = 0.007376;
		start[27] = 0.0046587;
		start[28] = 0.0026378;
		start[29] = 0.0035759;
		start[30] = 0.0026288;
		start[31] = 0.0025844;
		start[32] = 0.00054289;
		start[33] = 0.0064091;
		start[34] = 0.0088885;
		start[35] = 0.0076282;
		start[36] = 0.0088426;
		start[37] = 0.0094638;
		start[38] = 0.0029638;
		start[39] = 0.0015367;
		start[40] = 0.00088951;
		start[41] = 0.0050579;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0088606;
		start[7] = 0.00032979;
		start[8] = 0.0059226;
		start[9] = 0.0042166;
		start[10] = 0.0090914;
		start[11] = 0.003821;
		start[12] = 8.1722e-05;
		start[13] = 0.0046202;
		start[14] = 0.0069478;
		start[15] = 0.00038089;
		start[16] = 0.00074174;
		start[17] = 0.0028552;
		start[18] = 0.0076326;
		start[19] = 0.0011897;
		start[20] = 4.7172e-05;
		start[21] = 0.0094332;
		start[22] = 0.0082102;
		start[23] = 0.0046128;
		start[24] = 0.0099697;
		start[25] = 0.0034026;
		start[26] = 0.0029204;
		start[27] = 0.001714;
		start[28] = 0.0018951;
		start[29] = 0.0062557;
		start[30] = 0.0065839;
		start[31] = 0.0086896;
		start[32] = 0.0082588;
		start[33] = 0.00030444;
		start[34] = 0.0024542;
		start[35] = 0.0078803;
		start[36] = 0.0039808;
		start[37] = 0.00099571;
		start[38] = 0.0035125;
		start[39] = 0.0077465;
		start[40] = 0.0065439;
		start[41] = 0.00018134;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0015191;
		start[7] = 0.0072481;
		start[8] = 0.0033429;
		start[9] = 0.0040078;
		start[10] = 0.0073168;
		start[11] = 0.0040509;
		start[12] = 0.0080048;
		start[13] = 0.0025604;
		start[14] = 0.0026908;
		start[15] = 0.0013987;
		start[16] = 0.0057302;
		start[17] = 0.0059145;
		start[18] = 0.0028489;
		start[19] = 0.0037666;
		start[20] = 0.0039799;
		start[21] = 0.0074458;
		start[22] = 0.0043108;
		start[23] = 0.0089529;
		start[24] = 0.0050028;
		start[25] = 0.0090804;
		start[26] = 4.9788e-05;
		start[27] = 0.0024371;
		start[28] = 0.0075854;
		start[29] = 0.0041039;
		start[30] = 0.00085051;
		start[31] = 0.0028919;
		start[32] = 0.0031982;
		start[33] = 0.00018969;
		start[34] = 0.0042791;
		start[35] = 0.0039878;
		start[36] = 0.0079199;
		start[37] = 0.0034923;
		start[38] = 0.0039466;
		start[39] = 0.00081649;
		start[40] = 0.0079886;
		start[41] = 0.004781;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.00043465;
		start[7] = 0.0070315;
		start[8] = 0.0092783;
		start[9] = 0.0046637;
		start[10] = 0.0066261;
		start[11] = 0.0088576;
		start[12] = 0.0017528;
		start[13] = 0.0079984;
		start[14] = 0.0055479;
		start[15] = 0.0078686;
		start[16] = 0.0097195;
		start[17] = 0.001232;
		start[18] = 0.00088907;
		start[19] = 0.0057919;
		start[20] = 0.0088083;
		start[21] = 0.0043551;
		start[22] = 0.0058998;
		start[23] = 0.0091635;
		start[24] = 0.0016612;
		start[25] = 0.0092513;
		start[26] = 0.0056603;
		start[27] = 0.0059308;
		start[28] = 0.0071945;
		start[29] = 0.0090652;
		start[30] = 0.0088909;
		start[31] = 0.0013574;
		start[32] = 0.0024583;
		start[33] = 0.0036578;
		start[34] = 0.0034371;
		start[35] = 0.0085515;
		start[36] = 0.0070423;
		start[37] = 0.0066568;
		start[38] = 0.0052417;
		start[39] = 0.0084643;
		start[40] = 0.0014217;
		start[41] = 0.0043245;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0013391;
		start[7] = 0.0010737;
		start[8] = 0.0040344;
		start[9] = 0.0037044;
		start[10] = 0.0035197;
		start[11] = 0.0073531;
		start[12] = 0.003069;
		start[13] = 0.0031466;
		start[14] = 0.0004575;
		start[15] = 0.0019079;
		start[16] = 0.0081467;
		start[17] = 0.0097972;
		start[18] = 0.0038549;
		start[19] = 0.0073291;
		start[20] = 0.008819;
		start[21] = 0.00013484;
		start[22] = 0.0072783;
		start[23] = 0.0068452;
		start[24] = 0.0056444;
		start[25] = 0.0061727;
		start[26] = 0.0088214;
		start[27] = 0.007928;
		start[28] = 0.0075277;
		start[29] = 0.00057838;
		start[30] = 0.0099534;
		start[31] = 0.0021453;
		start[32] = 0.0076726;
		start[33] = 0.0098415;
		start[34] = 0.0015404;
		start[35] = 0.0015437;
		start[36] = 0.0031182;
		start[37] = 0.0014828;
		start[38] = 0.0080438;
		start[39] = 0.0051737;
		start[40] = 0.0035605;
		start[41] = 0.0027096;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0059354;
		start[7] = 0.0059522;
		start[8] = 0.0018235;
		start[9] = 0.0082046;
		start[10] = 0.0028321;
		start[11] = 0.0033254;
		start[12] = 0.0010672;
		start[13] = 0.0025053;
		start[14] = 0.0012847;
		start[15] = 0.0074058;
		start[16] = 0.0052453;
		start[17] = 0.00094593;
		start[18] = 0.0057318;
		start[19] = 0.0064721;
		start[20] = 0.0094127;
		start[21] = 0.00077441;
		start[22] = 0.0055395;
		start[23] = 0.0093474;
		start[24] = 0.00017781;
		start[25] = 0.0076645;
		start[26] = 0.0094782;
		start[27] = 0.0080952;
		start[28] = 0.0086227;
		start[29] = 0.00012762;
		start[30] = 0.0039885;
		start[31] = 0.0066424;
		start[32] = 0.0012914;
		start[33] = 0.0095338;
		start[34] = 0.0078981;
		start[35] = 0.0041246;
		start[36] = 0.0037764;
		start[37] = 0.003153;
		start[38] = 0.0026067;
		start[39] = 0.0060144;
		start[40] = 0.00096916;
		start[41] = 0.0077348;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.006271;
		start[7] = 0.008522;
		start[8] = 0.0089693;
		start[9] = 0.0065581;
		start[10] = 0.0042419;
		start[11] = 0.004295;
		start[12] = 0.004599;
		start[13] = 0.0041738;
		start[14] = 0.0029585;
		start[15] = 0.0080703;
		start[16] = 8.4168e-05;
		start[17] = 0.008538;
		start[18] = 0.0041943;
		start[19] = 0.0020267;
		start[20] = 0.0053434;
		start[21] = 0.0017716;
		start[22] = 0.0062817;
		start[23] = 0.0030927;
		start[24] = 0.00896;
		start[25] = 0.0036789;
		start[26] = 0.00853;
		start[27] = 0.0055533;
		start[28] = 0.0068268;
		start[29] = 0.0028199;
		start[30] = 0.0041698;
		start[31] = 0.0055984;
		start[32] = 0.0083424;
		start[33] = 0.00072476;
		start[34] = 0.0096617;
		start[35] = 0.0019247;
		start[36] = 0.0096411;
		start[37] = 0.0097179;
		start[38] = 0.0076774;
		start[39] = 0.0044994;
		start[40] = 0.0089475;
		start[41] = 0.0074993;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0016601;
		start[7] = 0.0084392;
		start[8] = 0.0015971;
		start[9] = 0.00087775;
		start[10] = 0.0067367;
		start[11] = 0.0065995;
		start[12] = 0.0056418;
		start[13] = 0.0073077;
		start[14] = 0.0075243;
		start[15] = 0.00074111;
		start[16] = 0.0064097;
		start[17] = 0.001372;
		start[18] = 0.0072161;
		start[19] = 0.0028631;
		start[20] = 0.008192;
		start[21] = 0.007368;
		start[22] = 0.0064368;
		start[23] = 0.0091805;
		start[24] = 0.0073703;
		start[25] = 0.0068461;
		start[26] = 0.0043072;
		start[27] = 0.0076435;
		start[28] = 0.0038029;
		start[29] = 0.0018785;
		start[30] = 0.0042643;
		start[31] = 0.0072114;
		start[32] = 0.0097554;
		start[33] = 0.0023819;
		start[34] = 0.0080975;
		start[35] = 0.0063476;
		start[36] = 0.0005141;
		start[37] = 0.0085843;
		start[38] = 0.006357;
		start[39] = 0.0029616;
		start[40] = 0.0078387;
		start[41] = 0.0066927;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0048788;
		start[7] = 0.0098271;
		start[8] = 0.0071745;
		start[9] = 0.0012629;
		start[10] = 0.008839;
		start[11] = 0.00494;
		start[12] = 0.0099953;
		start[13] = 0.0047959;
		start[14] = 0.0037571;
		start[15] = 0.0048307;
		start[16] = 0.0012014;
		start[17] = 0.0068851;
		start[18] = 0.0074758;
		start[19] = 0.0038943;
		start[20] = 0.007289;
		start[21] = 0.008531;
		start[22] = 0.0089644;
		start[23] = 0.0025563;
		start[24] = 0.0055296;
		start[25] = 0.0066073;
		start[26] = 0.0069453;
		start[27] = 0.0096045;
		start[28] = 0.0041571;
		start[29] = 0.0076207;
		start[30] = 0.00090048;
		start[31] = 0.0058262;
		start[32] = 0.00095332;
		start[33] = 0.0070085;
		start[34] = 0.0084379;
		start[35] = 0.0080969;
		start[36] = 0.0039682;
		start[37] = 0.003513;
		start[38] = 0.0068667;
		start[39] = 0.0022683;
		start[40] = 0.0015738;
		start[41] = 0.0027333;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0030401;
		start[7] = 0.0086163;
		start[8] = 0.0019876;
		start[9] = 0.00029333;
		start[10] = 0.0011536;
		start[11] = 0.0022662;
		start[12] = 0.0092675;
		start[13] = 0.0037785;
		start[14] = 0.008599;
		start[15] = 0.00032257;
		start[16] = 0.0027934;
		start[17] = 0.0037676;
		start[18] = 0.0035238;
		start[19] = 0.0011788;
		start[20] = 0.0065915;
		start[21] = 0.0062946;
		start[22] = 0.0056067;
		start[23] = 0.0067515;
		start[24] = 0.0028906;
		start[25] = 0.001398;
		start[26] = 0.0031244;
		start[27] = 0.0010071;
		start[28] = 0.0034971;
		start[29] = 0.0049734;
		start[30] = 0.0032884;
		start[31] = 0.0095137;
		start[32] = 0.00029334;
		start[33] = 0.0074951;
		start[34] = 0.0099087;
		start[35] = 0.0067768;
		start[36] = 0.0094684;
		start[37] = 0.0060455;
		start[38] = 0.0076949;
		start[39] = 0.0065328;
		start[40] = 0.0028457;
		start[41] = 0.0063023;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.006871;
		start[7] = 0.0019021;
		start[8] = 0.0033959;
		start[9] = 0.0097572;
		start[10] = 0.0013322;
		start[11] = 0.0080269;
		start[12] = 0.0080741;
		start[13] = 0.0087445;
		start[14] = 0.0062106;
		start[15] = 0.0093627;
		start[16] = 0.0022663;
		start[17] = 0.0042765;
		start[18] = 0.0083073;
		start[19] = 0.0024288;
		start[20] = 0.008299;
		start[21] = 0.0027554;
		start[22] = 0.003836;
		start[23] = 0.004387;
		start[24] = 0.0013967;
		start[25] = 0.0030867;
		start[26] = 0.0077241;
		start[27] = 0.00034535;
		start[28] = 0.0035212;
		start[29] = 0.0072497;
		start[30] = 0.0040851;
		start[31] = 0.0082569;
		start[32] = 0.00018455;
		start[33] = 0.0076472;
		start[34] = 0.0075844;
		start[35] = 0.0032586;
		start[36] = 0.0035156;
		start[37] = 0.0057575;
		start[38] = 0.0063338;
		start[39] = 0.00015095;
		start[40] = 0.0084026;
		start[41] = 0.00018182;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0017121;
		start[7] = 0.0010442;
		start[8] = 0.0048243;
		start[9] = 0.0036488;
		start[10] = 0.0036734;
		start[11] = 0.0048568;
		start[12] = 0.0042087;
		start[13] = 0.0032604;
		start[14] = 0.0042086;
		start[15] = 0.0069719;
		start[16] = 0.0014386;
		start[17] = 0.0090753;
		start[18] = 0.0081128;
		start[19] = 0.002709;
		start[20] = 0.0090501;
		start[21] = 0.0058411;
		start[22] = 0.0019865;
		start[23] = 0.0066828;
		start[24] = 0.0083814;
		start[25] = 0.0023402;
		start[26] = 0.0065305;
		start[27] = 0.0010346;
		start[28] = 0.0067388;
		start[29] = 0.003273;
		start[30] = 0.0086148;
		start[31] = 0.0055421;
		start[32] = 0.0085314;
		start[33] = 0.0089732;
		start[34] = 0.0017663;
		start[35] = 0.004282;
		start[36] = 0.0078405;
		start[37] = 0.0030958;
		start[38] = 0.00997;
		start[39] = 0.0011503;
		start[40] = 0.0055427;
		start[41] = 0.0047154;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0081215;
		start[7] = 0.0060958;
		start[8] = 0.0030965;
		start[9] = 0.0049324;
		start[10] = 0.0039179;
		start[11] = 0.001647;
		start[12] = 0.0068107;
		start[13] = 0.0087386;
		start[14] = 0.0099572;
		start[15] = 0.0035201;
		start[16] = 0.0056066;
		start[17] = 0.00050443;
		start[18] = 0.0044611;
		start[19] = 0.0098239;
		start[20] = 0.0021418;
		start[21] = 0.0043648;
		start[22] = 0.0055259;
		start[23] = 0.0022415;
		start[24] = 0.001107;
		start[25] = 0.0066885;
		start[26] = 0.0069748;
		start[27] = 0.00485;
		start[28] = 0.0034365;
		start[29] = 0.0095769;
		start[30] = 0.0076079;
		start[31] = 0.0073001;
		start[32] = 0.0063334;
		start[33] = 0.0044796;
		start[34] = 0.0048131;
		start[35] = 0.00083034;
		start[36] = 0.0049728;
		start[37] = 0.0021499;
		start[38] = 0.0056162;
		start[39] = 0.0023819;
		start[40] = 0.0038732;
		start[41] = 0.0079741;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 2.591e-05;
		start[7] = 0.0037843;
		start[8] = 0.0012457;
		start[9] = 0.00945;
		start[10] = 0.0073981;
		start[11] = 0.00053605;
		start[12] = 0.0059081;
		start[13] = 0.0079981;
		start[14] = 0.0011926;
		start[15] = 0.0014774;
		start[16] = 0.0026789;
		start[17] = 0.0075983;
		start[18] = 0.0075035;
		start[19] = 0.0069283;
		start[20] = 0.0054951;
		start[21] = 0.0055031;
		start[22] = 0.0023122;
		start[23] = 0.0055897;
		start[24] = 0.00040059;
		start[25] = 0.0053468;
		start[26] = 0.0023068;
		start[27] = 0.0021304;
		start[28] = 0.0088411;
		start[29] = 0.0095514;
		start[30] = 0.0035661;
		start[31] = 0.00093009;
		start[32] = 0.0074392;
		start[33] = 0.00039124;
		start[34] = 0.0036289;
		start[35] = 0.001977;
		start[36] = 3.8457e-05;
		start[37] = 0.0067909;
		start[38] = 0.0007902;
		start[39] = 0.003379;
		start[40] = 0.0026604;
		start[41] = 0.0079284;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0022407;
		start[7] = 0.0073051;
		start[8] = 0.0058755;
		start[9] = 0.0081002;
		start[10] = 0.003609;
		start[11] = 0.0014003;
		start[12] = 0.0068726;
		start[13] = 0.0046573;
		start[14] = 0.0053107;
		start[15] = 0.0055811;
		start[16] = 0.0062546;
		start[17] = 0.0089541;
		start[18] = 0.00092548;
		start[19] = 0.0073928;
		start[20] = 0.0088782;
		start[21] = 0.00011286;
		start[22] = 0.0078667;
		start[23] = 0.002689;
		start[24] = 0.0098302;
		start[25] = 0.0078536;
		start[26] = 0.0067781;
		start[27] = 0.0090956;
		start[28] = 0.0027936;
		start[29] = 0.0076694;
		start[30] = 0.00080118;
		start[31] = 0.003218;
		start[32] = 0.0053602;
		start[33] = 0.0093546;
		start[34] = 0.0048271;
		start[35] = 0.0019103;
		start[36] = 0.0063995;
		start[37] = 0.0052597;
		start[38] = 0.0048865;
		start[39] = 0.0032658;
		start[40] = 0.0017214;
		start[41] = 5.6792e-05;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0069269;
		start[7] = 0.004276;
		start[8] = 0.0095017;
		start[9] = 0.0084987;
		start[10] = 0.0010243;
		start[11] = 0.0058571;
		start[12] = 0.0083475;
		start[13] = 0.0045708;
		start[14] = 0.009704;
		start[15] = 0.0086306;
		start[16] = 0.0093084;
		start[17] = 0.0027775;
		start[18] = 0.0093914;
		start[19] = 0.0044201;
		start[20] = 0.0014995;
		start[21] = 0.0041899;
		start[22] = 0.009208;
		start[23] = 0.0041611;
		start[24] = 0.0055775;
		start[25] = 0.0014158;
		start[26] = 0.0077541;
		start[27] = 0.0094451;
		start[28] = 0.0079275;
		start[29] = 0.0047696;
		start[30] = 0.0056815;
		start[31] = 0.0088411;
		start[32] = 0.0061895;
		start[33] = 0.0076973;
		start[34] = 0.0011547;
		start[35] = 0.0018702;
		start[36] = 0.0029752;
		start[37] = 0.0081284;
		start[38] = 0.0040281;
		start[39] = 0.0051295;
		start[40] = 0.0095469;
		start[41] = 5.8478e-06;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.00010863;
		start[7] = 0.0019409;
		start[8] = 0.0053643;
		start[9] = 0.0084786;
		start[10] = 2.5302e-05;
		start[11] = 0.0086397;
		start[12] = 0.0017029;
		start[13] = 0.00068778;
		start[14] = 0.0083154;
		start[15] = 0.0029855;
		start[16] = 0.0024439;
		start[17] = 0.0041225;
		start[18] = 0.005815;
		start[19] = 0.0091668;
		start[20] = 0.0081292;
		start[21] = 0.0053939;
		start[22] = 0.00087156;
		start[23] = 0.0081304;
		start[24] = 0.0081822;
		start[25] = 0.0051697;
		start[26] = 0.0023962;
		start[27] = 0.0061026;
		start[28] = 0.0026805;
		start[29] = 0.0022976;
		start[30] = 0.0027249;
		start[31] = 0.0072343;
		start[32] = 0.0022311;
		start[33] = 0.0038919;
		start[34] = 0.0048098;
		start[35] = 0.0043615;
		start[36] = 0.00029429;
		start[37] = 0.0095353;
		start[38] = 0.0091635;
		start[39] = 0.0036443;
		start[40] = 0.0041465;
		start[41] = 0.0051453;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0050352;
		start[7] = 0.0066366;
		start[8] = 0.0025485;
		start[9] = 0.0048012;
		start[10] = 0.0052314;
		start[11] = 0.0064433;
		start[12] = 0.0056715;
		start[13] = 0.0051696;
		start[14] = 0.0052644;
		start[15] = 0.003487;
		start[16] = 0.0088689;
		start[17] = 0.0075095;
		start[18] = 0.0073734;
		start[19] = 0.0043133;
		start[20] = 0.0085816;
		start[21] = 0.00042794;
		start[22] = 0.0052502;
		start[23] = 0.0045583;
		start[24] = 0.0084414;
		start[25] = 0.0088648;
		start[26] = 0.0090489;
		start[27] = 0.0046657;
		start[28] = 0.0074343;
		start[29] = 0.0018897;
		start[30] = 0.0014903;
		start[31] = 0.007298;
		start[32] = 0.0082925;
		start[33] = 0.0097862;
		start[34] = 0.0026646;
		start[35] = 0.0041245;
		start[36] = 0.009894;
		start[37] = 0.0015132;
		start[38] = 0.0080724;
		start[39] = 0.0065978;
		start[40] = 0.0019945;
		start[41] = 0.0066521;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0078819;
		start[7] = 0.0059356;
		start[8] = 0.0071196;
		start[9] = 0.0085079;
		start[10] = 0.0027647;
		start[11] = 0.0062886;
		start[12] = 0.0051911;
		start[13] = 0.0090672;
		start[14] = 0.00044178;
		start[15] = 0.0093579;
		start[16] = 0.0030116;
		start[17] = 0.0046834;
		start[18] = 0.001133;
		start[19] = 0.0051446;
		start[20] = 0.0098034;
		start[21] = 0.0072238;
		start[22] = 0.00053469;
		start[23] = 0.0075538;
		start[24] = 0.0059769;
		start[25] = 0.0074245;
		start[26] = 0.0017474;
		start[27] = 0.0095397;
		start[28] = 0.0010855;
		start[29] = 0.0018568;
		start[30] = 0.0085919;
		start[31] = 0.0033038;
		start[32] = 0.0022432;
		start[33] = 0.0041739;
		start[34] = 0.0095117;
		start[35] = 0.0021364;
		start[36] = 0.0043194;
		start[37] = 0.00032107;
		start[38] = 0.0098713;
		start[39] = 0.0050006;
		start[40] = 0.0042299;
		start[41] = 0.0023315;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.00064235;
		start[7] = 0.0093336;
		start[8] = 0.0076026;
		start[9] = 0.0067695;
		start[10] = 0.0080462;
		start[11] = 0.0019152;
		start[12] = 0.0068539;
		start[13] = 0.0024387;
		start[14] = 0.0040246;
		start[15] = 0.0012313;
		start[16] = 0.0058287;
		start[17] = 0.0017584;
		start[18] = 0.0037632;
		start[19] = 0.0053486;
		start[20] = 0.0049172;
		start[21] = 0.0094153;
		start[22] = 0.0031824;
		start[23] = 0.00071566;
		start[24] = 0.0089639;
		start[25] = 0.0062926;
		start[26] = 0.0098485;
		start[27] = 0.0064347;
		start[28] = 0.0093388;
		start[29] = 0.0090609;
		start[30] = 0.0064037;
		start[31] = 0.0046197;
		start[32] = 0.0028708;
		start[33] = 0.0089885;
		start[34] = 0.0016658;
		start[35] = 0.0028483;
		start[36] = 0.0057973;
		start[37] = 0.0099372;
		start[38] = 0.00020972;
		start[39] = 0.0001835;
		start[40] = 0.0014989;
		start[41] = 0.0040825;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0001327;
		start[7] = 1.5901e-05;
		start[8] = 0.0031492;
		start[9] = 0.0023854;
		start[10] = 0.003592;
		start[11] = 0.007547;
		start[12] = 0.0090768;
		start[13] = 0.0025263;
		start[14] = 0.0052253;
		start[15] = 0.0037069;
		start[16] = 0.0089488;
		start[17] = 0.0086954;
		start[18] = 0.0041455;
		start[19] = 0.0067731;
		start[20] = 0.0066186;
		start[21] = 0.007361;
		start[22] = 0.0040501;
		start[23] = 0.0055728;
		start[24] = 0.0037589;
		start[25] = 0.0071023;
		start[26] = 0.0035619;
		start[27] = 0.0072644;
		start[28] = 0.0026555;
		start[29] = 0.0023047;
		start[30] = 0.0099135;
		start[31] = 0.0075382;
		start[32] = 0.0078816;
		start[33] = 0.00202;
		start[34] = 0.0073522;
		start[35] = 0.0062843;
		start[36] = 0.0095332;
		start[37] = 0.0054272;
		start[38] = 0.0020464;
		start[39] = 0.007616;
		start[40] = 0.0075454;
		start[41] = 0.0005678;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0032406;
		start[7] = 0.0056225;
		start[8] = 0.0027942;
		start[9] = 0.0060349;
		start[10] = 0.0012605;
		start[11] = 0.0034044;
		start[12] = 0.0056215;
		start[13] = 0.003269;
		start[14] = 0.0051398;
		start[15] = 0.0052423;
		start[16] = 0.0069683;
		start[17] = 0.0018301;
		start[18] = 0.0014868;
		start[19] = 0.0071552;
		start[20] = 0.005879;
		start[21] = 0.0014222;
		start[22] = 0.0055704;
		start[23] = 0.0028683;
		start[24] = 0.0068813;
		start[25] = 0.0064872;
		start[26] = 0.0011677;
		start[27] = 0.0034524;
		start[28] = 0.0041635;
		start[29] = 0.0089388;
		start[30] = 0.0029786;
		start[31] = 0.0054457;
		start[32] = 0.0040116;
		start[33] = 0.0073164;
		start[34] = 0.0074502;
		start[35] = 0.00036682;
		start[36] = 0.0062182;
		start[37] = 0.0098983;
		start[38] = 0.0098039;
		start[39] = 0.0040493;
		start[40] = 0.00039952;
		start[41] = 0.0032037;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0068482;
		start[7] = 0.0063673;
		start[8] = 0.0062968;
		start[9] = 0.0010491;
		start[10] = 0.0021006;
		start[11] = 0.0075906;
		start[12] = 0.0051568;
		start[13] = 0.0030186;
		start[14] = 0.0069027;
		start[15] = 0.0083122;
		start[16] = 0.0029683;
		start[17] = 0.0079089;
		start[18] = 0.00058846;
		start[19] = 0.0065372;
		start[20] = 0.0078073;
		start[21] = 0.00091839;
		start[22] = 0.0039389;
		start[23] = 0.0063352;
		start[24] = 0.0096709;
		start[25] = 0.0093392;
		start[26] = 0.00022616;
		start[27] = 0.0032183;
		start[28] = 0.0044907;
		start[29] = 0.0085408;
		start[30] = 0.0084083;
		start[31] = 0.0061274;
		start[32] = 0.0013784;
		start[33] = 0.0023766;
		start[34] = 0.0084164;
		start[35] = 0.0073745;
		start[36] = 0.0098846;
		start[37] = 0.0064688;
		start[38] = 0.0014959;
		start[39] = 0.0078928;
		start[40] = 0.004085;
		start[41] = 0.0041615;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0023596;
		start[7] = 0.0044237;
		start[8] = 0.0034885;
		start[9] = 0.0068564;
		start[10] = 0.0015004;
		start[11] = 0.0040168;
		start[12] = 6.1797e-05;
		start[13] = 0.0075214;
		start[14] = 0.0045213;
		start[15] = 0.004094;
		start[16] = 0.0033441;
		start[17] = 0.0039883;
		start[18] = 0.0039394;
		start[19] = 0.009759;
		start[20] = 0.0046007;
		start[21] = 0.0023066;
		start[22] = 0.0032934;
		start[23] = 0.0063046;
		start[24] = 0.0063042;
		start[25] = 0.0015701;
		start[26] = 0.00010153;
		start[27] = 0.0026661;
		start[28] = 0.0051211;
		start[29] = 0.0075412;
		start[30] = 0.0031513;
		start[31] = 0.0041649;
		start[32] = 0.0025024;
		start[33] = 0.0074076;
		start[34] = 0.0033446;
		start[35] = 0.0067747;
		start[36] = 0.002423;
		start[37] = 0.006964;
		start[38] = 0.0023536;
		start[39] = 0.0060903;
		start[40] = 0.0049634;
		start[41] = 0.0029262;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0024357;
		start[7] = 0.0011456;
		start[8] = 0.0045799;
		start[9] = 0.0019011;
		start[10] = 0.00050841;
		start[11] = 0.0011212;
		start[12] = 0.008814;
		start[13] = 0.0098455;
		start[14] = 0.0089911;
		start[15] = 0.0060025;
		start[16] = 0.008878;
		start[17] = 0.0090535;
		start[18] = 0.0076267;
		start[19] = 0.0073443;
		start[20] = 0.0052676;
		start[21] = 0.0070642;
		start[22] = 0.0030559;
		start[23] = 0.0064952;
		start[24] = 0.0035219;
		start[25] = 0.0065026;
		start[26] = 0.0020143;
		start[27] = 0.005912;
		start[28] = 0.0044766;
		start[29] = 0.0031605;
		start[30] = 0.0068027;
		start[31] = 0.0016495;
		start[32] = 0.0070481;
		start[33] = 0.0090422;
		start[34] = 0.0047822;
		start[35] = 6.4252e-05;
		start[36] = 0.003354;
		start[37] = 0.0035109;
		start[38] = 0.0064539;
		start[39] = 0.0088435;
		start[40] = 0.0027642;
		start[41] = 0.0068573;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0053934;
		start[7] = 0.00042742;
		start[8] = 0.0030482;
		start[9] = 0.0032677;
		start[10] = 0.0095757;
		start[11] = 0.0054382;
		start[12] = 0.0014717;
		start[13] = 0.0034876;
		start[14] = 0.0034911;
		start[15] = 0.0066366;
		start[16] = 0.0036407;
		start[17] = 0.0099651;
		start[18] = 0.0037316;
		start[19] = 0.0035999;
		start[20] = 0.0068857;
		start[21] = 0.0038716;
		start[22] = 0.0018173;
		start[23] = 0.0091422;
		start[24] = 0.0086377;
		start[25] = 0.0093784;
		start[26] = 0.0062154;
		start[27] = 0.0025774;
		start[28] = 0.00022002;
		start[29] = 0.0037511;
		start[30] = 0.0072243;
		start[31] = 0.0073424;
		start[32] = 0.0018162;
		start[33] = 0.0009861;
		start[34] = 0.0016801;
		start[35] = 0.005106;
		start[36] = 0.0051222;
		start[37] = 0.0061007;
		start[38] = 0.0042968;
		start[39] = 0.0081429;
		start[40] = 0.0099714;
		start[41] = 0.004736;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0029908;
		start[7] = 0.00032111;
		start[8] = 0.0074244;
		start[9] = 0.00034379;
		start[10] = 0.001604;
		start[11] = 0.00016464;
		start[12] = 0.0031218;
		start[13] = 0.0067574;
		start[14] = 0.0030938;
		start[15] = 0.0063383;
		start[16] = 0.0024286;
		start[17] = 0.0047149;
		start[18] = 0.0062485;
		start[19] = 0.0070382;
		start[20] = 0.0091151;
		start[21] = 0.0036895;
		start[22] = 0.0014228;
		start[23] = 0.0031213;
		start[24] = 0.0095554;
		start[25] = 0.0096903;
		start[26] = 0.00053929;
		start[27] = 0.0089627;
		start[28] = 0.0022852;
		start[29] = 0.0065189;
		start[30] = 0.0070799;
		start[31] = 0.0040815;
		start[32] = 0.0020884;
		start[33] = 0.0085335;
		start[34] = 0.0078837;
		start[35] = 0.0022723;
		start[36] = 0.0027212;
		start[37] = 0.0091321;
		start[38] = 0.0005894;
		start[39] = 0.0079844;
		start[40] = 0.0083546;
		start[41] = 0.0034722;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0014423;
		start[7] = 0.00077;
		start[8] = 0.0079977;
		start[9] = 0.0099284;
		start[10] = 0.0022967;
		start[11] = 0.0071032;
		start[12] = 0.0070778;
		start[13] = 0.003352;
		start[14] = 0.0084404;
		start[15] = 5.0976e-05;
		start[16] = 0.005491;
		start[17] = 0.00082097;
		start[18] = 0.0021485;
		start[19] = 0.0090861;
		start[20] = 0.0091018;
		start[21] = 0.00052489;
		start[22] = 0.0028643;
		start[23] = 0.0036088;
		start[24] = 0.0098566;
		start[25] = 0.0077617;
		start[26] = 0.0040861;
		start[27] = 0.0036048;
		start[28] = 0.0013018;
		start[29] = 0.0067814;
		start[30] = 0.0063657;
		start[31] = 0.0087146;
		start[32] = 0.0024594;
		start[33] = 0.0029991;
		start[34] = 0.0048374;
		start[35] = 0.00048932;
		start[36] = 0.0085454;
		start[37] = 0.0044802;
		start[38] = 0.0002868;
		start[39] = 0.0011355;
		start[40] = 0.0087659;
		start[41] = 0.0093748;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0012544;
		start[7] = 0.0057597;
		start[8] = 0.0029023;
		start[9] = 0.0008598;
		start[10] = 0.0096665;
		start[11] = 0.0074624;
		start[12] = 0.0013278;
		start[13] = 0.0028878;
		start[14] = 0.006427;
		start[15] = 0.0059711;
		start[16] = 0.0067667;
		start[17] = 0.0081442;
		start[18] = 0.0085582;
		start[19] = 0.0016241;
		start[20] = 0.0072458;
		start[21] = 0.0077169;
		start[22] = 0.0088297;
		start[23] = 0.0087549;
		start[24] = 0.0095702;
		start[25] = 0.0022976;
		start[26] = 0.0075425;
		start[27] = 0.0080817;
		start[28] = 0.0044712;
		start[29] = 0.005909;
		start[30] = 0.0045348;
		start[31] = 0.0069826;
		start[32] = 0.0076099;
		start[33] = 0.0077313;
		start[34] = 0.0010027;
		start[35] = 0.0068843;
		start[36] = 0.0077333;
		start[37] = 0.0058961;
		start[38] = 0.0042137;
		start[39] = 0.0098193;
		start[40] = 0.007378;
		start[41] = 0.00019567;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0025355;
		start[7] = 0.0038091;
		start[8] = 0.0057746;
		start[9] = 0.0024409;
		start[10] = 0.0088385;
		start[11] = 0.0065422;
		start[12] = 0.0080763;
		start[13] = 0.0035565;
		start[14] = 0.0019871;
		start[15] = 0.0041823;
		start[16] = 0.0067716;
		start[17] = 0.0015213;
		start[18] = 0.0064541;
		start[19] = 0.0052537;
		start[20] = 0.0069987;
		start[21] = 0.0082469;
		start[22] = 0.0057886;
		start[23] = 0.0023658;
		start[24] = 0.0042625;
		start[25] = 0.0098712;
		start[26] = 0.0014055;
		start[27] = 0.0043795;
		start[28] = 0.007128;
		start[29] = 0.00027846;
		start[30] = 0.007487;
		start[31] = 0.0043373;
		start[32] = 0.0081708;
		start[33] = 0.0079723;
		start[34] = 0.0045599;
		start[35] = 0.0041731;
		start[36] = 0.0092487;
		start[37] = 0.0054833;
		start[38] = 0.00092681;
		start[39] = 0.0072818;
		start[40] = 0.0034315;
		start[41] = 0.0082348;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0039837;
		start[7] = 0.0090012;
		start[8] = 0.0048423;
		start[9] = 0.0027463;
		start[10] = 0.0066123;
		start[11] = 0.0061905;
		start[12] = 0.0066122;
		start[13] = 0.0066853;
		start[14] = 0.0022404;
		start[15] = 0.0017939;
		start[16] = 0.0064137;
		start[17] = 0.00011857;
		start[18] = 0.0093079;
		start[19] = 0.0042436;
		start[20] = 0.00090164;
		start[21] = 0.00064692;
		start[22] = 0.0077793;
		start[23] = 0.0031406;
		start[24] = 0.0053961;
		start[25] = 0.0029287;
		start[26] = 0.0048032;
		start[27] = 0.0046099;
		start[28] = 0.003123;
		start[29] = 0.0071612;
		start[30] = 0.0063881;
		start[31] = 0.0076063;
		start[32] = 0.0088359;
		start[33] = 0.0090279;
		start[34] = 0.0038006;
		start[35] = 0.0096874;
		start[36] = 0.0030632;
		start[37] = 0.0079531;
		start[38] = 0.0049936;
		start[39] = 0.0051723;
		start[40] = 0.0018565;
		start[41] = 0.0083345;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 9.4528e-05;
		start[7] = 0.0066224;
		start[8] = 0.0065929;
		start[9] = 0.0080229;
		start[10] = 0.0046137;
		start[11] = 0.0049188;
		start[12] = 0.0041204;
		start[13] = 0.002176;
		start[14] = 0.0085197;
		start[15] = 0.0076772;
		start[16] = 0.0076111;
		start[17] = 0.0024715;
		start[18] = 0.0059225;
		start[19] = 0.0047608;
		start[20] = 0.0056704;
		start[21] = 0.0041493;
		start[22] = 0.0032272;
		start[23] = 0.0030623;
		start[24] = 0.005928;
		start[25] = 0.0044093;
		start[26] = 0.0059974;
		start[27] = 0.0042484;
		start[28] = 0.0062407;
		start[29] = 0.0018415;
		start[30] = 0.0034181;
		start[31] = 0.0067774;
		start[32] = 0.0089688;
		start[33] = 0.0049668;
		start[34] = 0.0018033;
		start[35] = 0.0012786;
		start[36] = 0.0065183;
		start[37] = 0.0089585;
		start[38] = 0.0085634;
		start[39] = 0.0035675;
		start[40] = 0.0084619;
		start[41] = 0.0089376;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.00081147;
		start[7] = 0.0071841;
		start[8] = 0.0027974;
		start[9] = 0.0095415;
		start[10] = 0.0046647;
		start[11] = 0.0094287;
		start[12] = 0.0042678;
		start[13] = 0.007699;
		start[14] = 0.007268;
		start[15] = 0.0056664;
		start[16] = 0.0068584;
		start[17] = 0.0034359;
		start[18] = 0.0066527;
		start[19] = 0.0032166;
		start[20] = 0.00097176;
		start[21] = 0.00062025;
		start[22] = 0.0045214;
		start[23] = 0.0063666;
		start[24] = 0.00031388;
		start[25] = 0.0036009;
		start[26] = 0.0017513;
		start[27] = 0.0021453;
		start[28] = 0.0082928;
		start[29] = 0.0046876;
		start[30] = 0.0052323;
		start[31] = 0.0093668;
		start[32] = 0.0069378;
		start[33] = 0.0058202;
		start[34] = 0.0098876;
		start[35] = 0.0079698;
		start[36] = 0.0084773;
		start[37] = 0.0059428;
		start[38] = 0.0067863;
		start[39] = 0.0082942;
		start[40] = 0.0010654;
		start[41] = 0.0022128;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0020018;
		start[7] = 0.0084257;
		start[8] = 0.0077539;
		start[9] = 0.0090486;
		start[10] = 0.0010494;
		start[11] = 0.008815;
		start[12] = 0.0066429;
		start[13] = 0.0058736;
		start[14] = 0.0065081;
		start[15] = 0.0085618;
		start[16] = 0.00066799;
		start[17] = 0.0055163;
		start[18] = 0.0094344;
		start[19] = 0.0069582;
		start[20] = 0.00062915;
		start[21] = 0.0032497;
		start[22] = 0.0084633;
		start[23] = 0.0045027;
		start[24] = 0.0008948;
		start[25] = 0.0059001;
		start[26] = 0.0046449;
		start[27] = 0.0047973;
		start[28] = 0.0050669;
		start[29] = 0.0019874;
		start[30] = 0.0046319;
		start[31] = 0.0064288;
		start[32] = 0.0083445;
		start[33] = 0.0051465;
		start[34] = 0.0059125;
		start[35] = 0.0059469;
		start[36] = 0.0075908;
		start[37] = 0.0073455;
		start[38] = 0.0060762;
		start[39] = 0.0055238;
		start[40] = 0.0027033;
		start[41] = 0.0033769;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0080762;
		start[7] = 0.002829;
		start[8] = 0.0064699;
		start[9] = 0.0049859;
		start[10] = 0.00064057;
		start[11] = 0.0054798;
		start[12] = 0.0080216;
		start[13] = 0.009665;
		start[14] = 0.0027402;
		start[15] = 0.0011394;
		start[16] = 0.0060191;
		start[17] = 0.0050871;
		start[18] = 0.0095224;
		start[19] = 0.009477;
		start[20] = 0.00047005;
		start[21] = 0.0038848;
		start[22] = 0.0093357;
		start[23] = 0.0099912;
		start[24] = 0.0079416;
		start[25] = 0.0036599;
		start[26] = 0.0059198;
		start[27] = 0.0040182;
		start[28] = 0.0050433;
		start[29] = 0.0029699;
		start[30] = 0.0017006;
		start[31] = 0.0043815;
		start[32] = 0.0034514;
		start[33] = 0.0075456;
		start[34] = 0.0055224;
		start[35] = 0.0084157;
		start[36] = 0.00095813;
		start[37] = 0.00064025;
		start[38] = 0.0073576;
		start[39] = 0.0032722;
		start[40] = 0.0066225;
		start[41] = 0.0022949;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0030471;
		start[7] = 0.0015964;
		start[8] = 0.0044032;
		start[9] = 0.0067207;
		start[10] = 7.4707e-05;
		start[11] = 0.0087812;
		start[12] = 0.00095111;
		start[13] = 0.003975;
		start[14] = 0.0068425;
		start[15] = 0.0015334;
		start[16] = 0.0077867;
		start[17] = 0.0012304;
		start[18] = 0.0028986;
		start[19] = 0.0065194;
		start[20] = 0.0094146;
		start[21] = 0.0066107;
		start[22] = 0.0066308;
		start[23] = 0.0070348;
		start[24] = 0.0062302;
		start[25] = 0.0015425;
		start[26] = 0.0073692;
		start[27] = 0.0071005;
		start[28] = 0.0076227;
		start[29] = 0.0068838;
		start[30] = 0.0048502;
		start[31] = 0.0044471;
		start[32] = 0.007483;
		start[33] = 0.007977;
		start[34] = 0.0098525;
		start[35] = 0.0041541;
		start[36] = 0.0051503;
		start[37] = 0.00664;
		start[38] = 0.0085734;
		start[39] = 0.00037745;
		start[40] = 0.001135;
		start[41] = 0.0069709;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0075293;
		start[7] = 0.0020991;
		start[8] = 0.0042198;
		start[9] = 0.0040578;
		start[10] = 0.0018139;
		start[11] = 0.0042955;
		start[12] = 0.0061744;
		start[13] = 0.0094064;
		start[14] = 0.0067209;
		start[15] = 0.0016391;
		start[16] = 0.0022568;
		start[17] = 0.0066573;
		start[18] = 0.0087536;
		start[19] = 0.008719;
		start[20] = 0.0091311;
		start[21] = 0.0066119;
		start[22] = 0.0016811;
		start[23] = 0.0055652;
		start[24] = 0.0081497;
		start[25] = 0.0094168;
		start[26] = 0.00023891;
		start[27] = 0.0079064;
		start[28] = 0.0064446;
		start[29] = 0.0039268;
		start[30] = 0.00039669;
		start[31] = 0.0042133;
		start[32] = 0.0035608;
		start[33] = 0.0098631;
		start[34] = 0.0029665;
		start[35] = 0.0031985;
		start[36] = 0.005485;
		start[37] = 0.0031385;
		start[38] = 0.007227;
		start[39] = 0.0029066;
		start[40] = 0.00086001;
		start[41] = 0.0081734;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0073038;
		start[7] = 0.0030337;
		start[8] = 0.0093634;
		start[9] = 0.0061001;
		start[10] = 0.0066221;
		start[11] = 0.0081722;
		start[12] = 0.0010514;
		start[13] = 0.0014281;
		start[14] = 0.00085173;
		start[15] = 0.007067;
		start[16] = 0.0084548;
		start[17] = 0.0030597;
		start[18] = 0.003122;
		start[19] = 0.0023256;
		start[20] = 0.0046399;
		start[21] = 0.003812;
		start[22] = 0.0019902;
		start[23] = 0.0071756;
		start[24] = 0.007941;
		start[25] = 0.005295;
		start[26] = 0.00023117;
		start[27] = 0.0024834;
		start[28] = 0.0080335;
		start[29] = 0.0077351;
		start[30] = 0.0089504;
		start[31] = 0.0031218;
		start[32] = 0.00056866;
		start[33] = 0.0037944;
		start[34] = 0.0066411;
		start[35] = 0.0023996;
		start[36] = 0.0093876;
		start[37] = 0.0043526;
		start[38] = 0.0057098;
		start[39] = 0.0047043;
		start[40] = 0.0087484;
		start[41] = 0.0085357;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0027959;
		start[7] = 0.0044039;
		start[8] = 0.003504;
		start[9] = 0.00087089;
		start[10] = 0.0089601;
		start[11] = 0.0054521;
		start[12] = 0.0097175;
		start[13] = 0.007276;
		start[14] = 0.00097037;
		start[15] = 0.007584;
		start[16] = 0.0090049;
		start[17] = 0.0054182;
		start[18] = 0.0014911;
		start[19] = 0.0084584;
		start[20] = 0.0090846;
		start[21] = 0.0063077;
		start[22] = 0.0035704;
		start[23] = 0.00062151;
		start[24] = 0.004689;
		start[25] = 0.0044236;
		start[26] = 0.0010435;
		start[27] = 0.002999;
		start[28] = 0.0052344;
		start[29] = 0.0023812;
		start[30] = 0.00066123;
		start[31] = 0.0096182;
		start[32] = 0.0069208;
		start[33] = 0.0066761;
		start[34] = 0.00018392;
		start[35] = 0.0059142;
		start[36] = 0.0039769;
		start[37] = 0.0087648;
		start[38] = 0.0047302;
		start[39] = 0.0036255;
		start[40] = 0.0073908;
		start[41] = 0.0098938;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		start[6] = 0.0049187;
		start[7] = 0.001214;
		start[8] = 0.0050844;
		start[9] = 0.0017288;
		start[10] = 0.00088829;
		start[11] = 0.0090224;
		start[12] = 0.0026928;
		start[13] = 0.0028067;
		start[14] = 0.0053154;
		start[15] = 0.00080971;
		start[16] = 0.0057022;
		start[17] = 0.0039118;
		start[18] = 0.0032182;
		start[19] = 0.0030323;
		start[20] = 0.0035733;
		start[21] = 0.0032421;
		start[22] = 0.0094986;
		start[23] = 0.00058142;
		start[24] = 0.0043892;
		start[25] = 0.00099871;
		start[26] = 0.0062628;
		start[27] = 0.0014387;
		start[28] = 0.0021841;
		start[29] = 0.0090238;
		start[30] = 0.008557;
		start[31] = 0.0016996;
		start[32] = 0.0072969;
		start[33] = 0.0092857;
		start[34] = 0.0028272;
		start[35] = 0.0076347;
		start[36] = 0.0064351;
		start[37] = 0.008937;
		start[38] = 0.0084073;
		start[39] = 0.0081657;
		start[40] = 0.0042508;
		start[41] = 0.0043663;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		start[6] = 0.0085158;
		start[7] = 0.0024497;
		start[8] = 0.0032698;
		start[9] = 0.00017855;
		start[10] = 0.0040317;
		start[11] = 0.0025168;
		start[12] = 0.0098374;
		start[13] = 0.0087797;
		start[14] = 0.0021081;
		start[15] = 0.0012954;
		start[16] = 0.0088725;
		start[17] = 0.005129;
		start[18] = 0.0065852;
		start[19] = 0.0058123;
		start[20] = 0.0088988;
		start[21] = 0.0084393;
		start[22] = 0.0024833;
		start[23] = 0.0036463;
		start[24] = 0.0050794;
		start[25] = 0.0088716;
		start[26] = 0.00049669;
		start[27] = 0.0030794;
		start[28] = 0.0051283;
		start[29] = 0.0065984;
		start[30] = 0.0070822;
		start[31] = 0.0063496;
		start[32] = 0.004858;
		start[33] = 0.0016974;
		start[34] = 0.0014078;
		start[35] = 0.0094294;
		start[36] = 0.0011442;
		start[37] = 0.0084249;
		start[38] = 0.0053626;
		start[39] = 0.0049872;
		start[40] = 0.0053723;
		start[41] = 0.0098557;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		start[6] = 0.0033053;
		start[7] = 0.0031941;
		start[8] = 0.0024919;
		start[9] = 0.00935;
		start[10] = 0.00028074;
		start[11] = 0.0060819;
		start[12] = 0.0090604;
		start[13] = 0.0091266;
		start[14] = 0.0093004;
		start[15] = 0.0029046;
		start[16] = 0.0049245;
		start[17] = 0.0049111;
		start[18] = 0.0031997;
		start[19] = 0.0070776;
		start[20] = 0.00072912;
		start[21] = 0.00029917;
		start[22] = 0.0038783;
		start[23] = 0.003793;
		start[24] = 0.0012269;
		start[25] = 0.0055805;
		start[26] = 0.0071691;
		start[27] = 0.0040046;
		start[28] = 0.0099388;
		start[29] = 0.0086213;
		start[30] = 0.0073976;
		start[31] = 0.0024596;
		start[32] = 0.0052521;
		start[33] = 0.0071703;
		start[34] = 0.0081705;
		start[35] = 0.0025207;
		start[36] = 0.0022687;
		start[37] = 0.0043104;
		start[38] = 0.0063988;
		start[39] = 0.0087574;
		start[40] = 0.0014792;
		start[41] = 0.0096681;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		start[6] = 0.0034662;
		start[7] = 0.0061634;
		start[8] = 0.0048593;
		start[9] = 0.0037228;
		start[10] = 0.0066153;
		start[11] = 0.0029996;
		start[12] = 0.0056705;
		start[13] = 0.0067003;
		start[14] = 0.0068914;
		start[15] = 0.0021285;
		start[16] = 0.0045723;
		start[17] = 0.0059079;
		start[18] = 0.00025153;
		start[19] = 0.001328;
		start[20] = 0.0054489;
		start[21] = 0.0066104;
		start[22] = 0.0098474;
		start[23] = 0.002646;
		start[24] = 0.00098513;
		start[25] = 0.0084782;
		start[26] = 0.00021795;
		start[27] = 0.0091951;
		start[28] = 0.0048486;
		start[29] = 0.0021519;
		start[30] = 0.0048436;
		start[31] = 0.00099582;
		start[32] = 0.0007338;
		start[33] = 0.0029883;
		start[34] = 0.0035462;
		start[35] = 0.0011408;
		start[36] = 0.005289;
		start[37] = 0.00516;
		start[38] = 0.0012642;
		start[39] = 0.0080868;
		start[40] = 0.0069759;
		start[41] = 0.0012943;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		start[6] = 0.0041282;
		start[7] = 0.00687;
		start[8] = 0.004886;
		start[9] = 0.0062672;
		start[10] = 0.007899;
		start[11] = 0.0094374;
		start[12] = 0.0080188;
		start[13] = 0.0013298;
		start[14] = 0.0086833;
		start[15] = 0.0051993;
		start[16] = 0.0014703;
		start[17] = 5.4164e-05;
		start[18] = 0.003377;
		start[19] = 0.0092732;
		start[20] = 0.0070536;
		start[21] = 0.0028501;
		start[22] = 0.0010358;
		start[23] = 0.003358;
		start[24] = 0.0057744;
		start[25] = 0.0077103;
		start[26] = 0.0053136;
		start[27] = 0.0056981;
		start[28] = 9.6214e-05;
		start[29] = 0.0013846;
		start[30] = 0.0050452;
		start[31] = 0.007414;
		start[32] = 0.00011479;
		start[33] = 0.0090251;
		start[34] = 0.0060849;
		start[35] = 0.0055741;
		start[36] = 0.0093796;
		start[37] = 0.0051326;
		start[38] = 0.0081621;
		start[39] = 0.0064561;
		start[40] = 0.0049873;
		start[41] = 0.0077195;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		start[6] = 0.0065862;
		start[7] = 0.0041888;
		start[8] = 0.0031898;
		start[9] = 0.0089687;
		start[10] = 0.0014759;
		start[11] = 0.0056667;
		start[12] = 0.004454;
		start[13] = 0.007041;
		start[14] = 0.0093628;
		start[15] = 0.0094944;
		start[16] = 0.0026055;
		start[17] = 0.0025843;
		start[18] = 0.0032663;
		start[19] = 0.0054224;
		start[20] = 0.0033233;
		start[21] = 0.0087584;
		start[22] = 0.0058528;
		start[23] = 0.0030712;
		start[24] = 0.0034685;
		start[25] = 0.0038144;
		start[26] = 0.0098181;
		start[27] = 0.0034042;
		start[28] = 0.0069546;
		start[29] = 0.0096187;
		start[30] = 0.0028501;
		start[31] = 0.005249;
		start[32] = 0.002466;
		start[33] = 0.0027683;
		start[34] = 0.0061596;
		start[35] = 0.0060043;
		start[36] = 0.0012462;
		start[37] = 0.0068608;
		start[38] = 0.0050679;
		start[39] = 0.0034545;
		start[40] = 0.007581;
		start[41] = 0.001903;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		start[6] = 0.0081336;
		start[7] = 0.0095399;
		start[8] = 0.0017358;
		start[9] = 0.0014515;
		start[10] = 0.0035531;
		start[11] = 0.0059709;
		start[12] = 0.0041464;
		start[13] = 0.0053799;
		start[14] = 0.0049533;
		start[15] = 0.004278;
		start[16] = 0.00215;
		start[17] = 0.003791;
		start[18] = 0.0085292;
		start[19] = 0.00023528;
		start[20] = 0.0052766;
		start[21] = 0.0052465;
		start[22] = 0.0040044;
		start[23] = 9.0926e-05;
		start[24] = 0.0088222;
		start[25] = 0.0094666;
		start[26] = 0.0096572;
		start[27] = 0.0028168;
		start[28] = 0.002803;
		start[29] = 0.00068362;
		start[30] = 0.0014698;
		start[31] = 0.0072755;
		start[32] = 0.0076407;
		start[33] = 0.0096717;
		start[34] = 0.0068109;
		start[35] = 0.0037748;
		start[36] = 0.0043527;
		start[37] = 0.0036388;
		start[38] = 0.0035125;
		start[39] = 0.0018132;
		start[40] = 0.0082927;
		start[41] = 0.0082914;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		start[6] = 0.0035039;
		start[7] = 0.0004416;
		start[8] = 0.0052897;
		start[9] = 0.009903;
		start[10] = 0.0010295;
		start[11] = 0.0083104;
		start[12] = 0.0087814;
		start[13] = 0.0079247;
		start[14] = 0.0079516;
		start[15] = 0.0042114;
		start[16] = 0.0063374;
		start[17] = 0.0095984;
		start[18] = 0.0051593;
		start[19] = 0.0014142;
		start[20] = 0.002647;
		start[21] = 0.00077655;
		start[22] = 0.0020008;
		start[23] = 0.0051474;
		start[24] = 0.0070671;
		start[25] = 0.0074786;
		start[26] = 0.0088418;
		start[27] = 0.0050368;
		start[28] = 0.0041427;
		start[29] = 0.0072374;
		start[30] = 0.0078363;
		start[31] = 0.0030976;
		start[32] = 0.0088317;
		start[33] = 0.0083466;
		start[34] = 0.004087;
		start[35] = 0.0077392;
		start[36] = 0.0040591;
		start[37] = 0.0016998;
		start[38] = 0.0047668;
		start[39] = 0.0073538;
		start[40] = 0.0037986;
		start[41] = 0.0015842;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		start[6] = 0.0044806;
		start[7] = 0.0062493;
		start[8] = 0.0073224;
		start[9] = 0.0019179;
		start[10] = 0.0037425;
		start[11] = 0.0028787;
		start[12] = 0.005879;
		start[13] = 0.005029;
		start[14] = 0.003767;
		start[15] = 0.0097658;
		start[16] = 0.0013902;
		start[17] = 0.0050071;
		start[18] = 0.0066858;
		start[19] = 0.0085285;
		start[20] = 0.0094543;
		start[21] = 0.0083624;
		start[22] = 0.0051617;
		start[23] = 0.0095568;
		start[24] = 0.0063501;
		start[25] = 0.0025614;
		start[26] = 0.0078572;
		start[27] = 0.00093757;
		start[28] = 0.0067424;
		start[29] = 0.0059824;
		start[30] = 0.0017874;
		start[31] = 0.0053505;
		start[32] = 0.0074656;
		start[33] = 0.0021632;
		start[34] = 0.0059259;
		start[35] = 0.0063511;
		start[36] = 0.0041744;
		start[37] = 0.0068781;
		start[38] = 0.0059228;
		start[39] = 0.00056189;
		start[40] = 0.0072329;
		start[41] = 0.0081707;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (42);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		start[6] = 0.0011268;
		start[7] = 0.0035;
		start[8] = 0.00068347;
		start[9] = 0.0030441;
		start[10] = 0.0010943;
		start[11] = 0.0097974;
		start[12] = 0.003766;
		start[13] = 0.0037028;
		start[14] = 0.0073644;
		start[15] = 0.0049445;
		start[16] = 0.0020628;
		start[17] = 0.00061924;
		start[18] = 0.0051732;
		start[19] = 0.0067624;
		start[20] = 0.0055584;
		start[21] = 0.0039334;
		start[22] = 0.0052011;
		start[23] = 0.00016598;
		start[24] = 0.0049488;
		start[25] = 0.0093856;
		start[26] = 0.0028482;
		start[27] = 0.0070119;
		start[28] = 0.0042815;
		start[29] = 0.005175;
		start[30] = 0.0070329;
		start[31] = 0.0052167;
		start[32] = 0.005712;
		start[33] = 0.0049021;
		start[34] = 0.003634;
		start[35] = 0.0096916;
		start[36] = 0.0079919;
		start[37] = 0.0016017;
		start[38] = 0.0013969;
		start[39] = 0.0039085;
		start[40] = 0.007397;
		start[41] = 0.0089428;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}


  return 0;
}
