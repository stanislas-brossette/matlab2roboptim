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
int main ()
{
  // Set the starting point.
  roboptim::Function::vector_t start (42);
  start[0] = 0.91065;
	start[1] = 0.18185;
	start[2] = 0.2638;
	start[3] = 0.14554;
	start[4] = 0.13607;
	start[5] = 0.86929;
	start[6] = 0.0;
	start[7] = 0.0;
	start[8] = 0.0;
	start[9] = 0.0;
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

  double EE_1_1 = 0.53834;
	double EE_1_2 = 0.99613;
	double EE_2_1 = 0.44268;
	double EE_2_2 = 0.10665;

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
