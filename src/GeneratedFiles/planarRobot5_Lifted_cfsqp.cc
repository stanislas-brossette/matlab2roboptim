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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
CostFunction<T>::CostFunction (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericLinearFunction<T>
    (37, 1, "CostFunction_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_1<T>::LiftConstraint_1 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_1_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = cos(q_05) - 1.0*w_01_01;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = -1.0*sin(q_05); 
			 grad[5] = -1.0; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_2<T>::LiftConstraint_2 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_2_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = sin(q_05) - 1.0*w_01_02;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = cos(q_05); 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_3<T>::LiftConstraint_3 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_3_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = cos(q_01) - 1.0*w_01_03;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_4<T>::LiftConstraint_4 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_4_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = cos(q_04) - 1.0*w_01_04;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_5<T>::LiftConstraint_5 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_5_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = sin(q_04) - 1.0*w_01_05;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_6<T>::LiftConstraint_6 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_6_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = sin(q_01) - 1.0*w_01_06;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_7<T>::LiftConstraint_7 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_7_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = cos(q_03) - 1.0*w_01_07;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = -1.0*sin(q_03); 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_8<T>::LiftConstraint_8 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_8_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = sin(q_03) - 1.0*w_01_08;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = cos(q_03); 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_9<T>::LiftConstraint_9 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_9_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = cos(q_02) - 1.0*w_01_09;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_10<T>::LiftConstraint_10 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_10_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = sin(q_02) - 1.0*w_01_10;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_11<T>::LiftConstraint_11 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_11_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_03*w_01_09 - 1.0*w_02_01;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[7] = w_01_09; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_01_03; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_12<T>::LiftConstraint_12 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_12_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_06*w_01_09 - 1.0*w_02_02;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[10] = w_01_09; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_01_06; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_13<T>::LiftConstraint_13 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_13_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_03*w_01_10 - 1.0*w_02_03;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[7] = w_01_10; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = w_01_03; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_14<T>::LiftConstraint_14 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_14_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_06*w_01_10 - 1.0*w_02_04;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[10] = w_01_10; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = w_01_06; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_15<T>::LiftConstraint_15 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_15_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_07*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_01;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[11] = w_02_01 - 1.0*w_02_04; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = w_01_07; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = -1.0*w_01_07; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_16<T>::LiftConstraint_16 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_16_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = - 1.0*w_03_02 - 1.0*w_01_08*(w_02_02 + w_02_03);
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[16] = -1.0*w_01_08; 
			 grad[17] = -1.0*w_01_08; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_17<T>::LiftConstraint_17 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_17_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_07*(w_02_02 + w_02_03) - 1.0*w_03_03;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[11] = w_02_02 + w_02_03; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = w_01_07; 
			 grad[17] = w_01_07; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_18<T>::LiftConstraint_18 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_18_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_08*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_04;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[15] = w_01_08; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = -1.0*w_01_08; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_19<T>::LiftConstraint_19 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_19_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = - 1.0*w_03_05 - 1.0*w_01_07*(w_02_02 + w_02_03);
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[11] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = -1.0*w_01_07; 
			 grad[17] = -1.0*w_01_07; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_20<T>::LiftConstraint_20 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_20_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_08*(w_02_02 + w_02_03) - 1.0*w_03_06;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[16] = w_01_08; 
			 grad[17] = w_01_08; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_21<T>::LiftConstraint_21 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_21_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_04*(w_03_01 + w_03_02) - 1.0*w_04_01;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[8] = w_03_01 + w_03_02; 
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
			 grad[19] = w_01_04; 
			 grad[20] = w_01_04; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_22<T>::LiftConstraint_22 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_22_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = - 1.0*w_04_02 - 1.0*w_01_05*(w_03_04 - 1.0*w_03_05);
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[9] = w_03_05 - 1.0*w_03_04; 
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
			 grad[22] = -1.0*w_01_05; 
			 grad[23] = w_01_05; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_23<T>::LiftConstraint_23 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_23_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_04*(w_03_03 + w_03_04) - 1.0*w_04_03;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[8] = w_03_03 + w_03_04; 
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
			 grad[21] = w_01_04; 
			 grad[22] = w_01_04; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_24<T>::LiftConstraint_24 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_24_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_05*(w_03_01 - 1.0*w_03_06) - 1.0*w_04_04;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[9] = w_03_01 - 1.0*w_03_06; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = w_01_05; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = -1.0*w_01_05; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_25<T>::LiftConstraint_25 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_25_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = - 1.0*w_04_05 - 1.0*w_01_04*(w_03_04 - 1.0*w_03_05);
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[8] = w_03_05 - 1.0*w_03_04; 
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
			 grad[22] = -1.0*w_01_04; 
			 grad[23] = w_01_04; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_26<T>::LiftConstraint_26 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_26_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_04*(w_03_01 - 1.0*w_03_06) - 1.0*w_04_06;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[8] = w_03_01 - 1.0*w_03_06; 
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
			 grad[19] = w_01_04; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = -1.0*w_01_04; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_27<T>::LiftConstraint_27 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_27_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_05*(w_03_01 + w_03_02) - 1.0*w_04_07;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[9] = w_03_01 + w_03_02; 
			 grad[10] = 0.0; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_28<T>::LiftConstraint_28 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_28_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_05*(w_03_03 + w_03_04) - 1.0*w_04_08;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[9] = w_03_03 + w_03_04; 
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
			 grad[21] = w_01_05; 
			 grad[22] = w_01_05; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_29<T>::LiftConstraint_29 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_29_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_01*(w_04_01 + w_04_02) - 1.0*w_05_01;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_04_01 + w_04_02; 
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
			 grad[25] = w_01_01; 
			 grad[26] = w_01_01; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_30<T>::LiftConstraint_30 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_30_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_02*(w_04_05 - 1.0*w_04_07) - 1.0*w_05_02;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = w_04_05 - 1.0*w_04_07; 
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
			 grad[29] = w_01_02; 
			 grad[30] = 0.0; 
			 grad[31] = -1.0*w_01_02; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = -1.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_31<T>::LiftConstraint_31 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_31_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_01*(w_04_03 + w_04_04) - 1.0*w_05_03;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_04_03 + w_04_04; 
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
			 grad[27] = w_01_01; 
			 grad[28] = w_01_01; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = -1.0; 
			 grad[36] = 0.0; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
LiftConstraint_32<T>::LiftConstraint_32 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (37, 1, "LiftConstraint_32_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_02*(w_04_06 - 1.0*w_04_08) - 1.0*w_05_04;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = w_04_06 - 1.0*w_04_08; 
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
			 grad[30] = w_01_02; 
			 grad[31] = 0.0; 
			 grad[32] = -1.0*w_01_02; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 0.0; 
			 grad[36] = -1.0; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
EEConstraint_1<T>::EEConstraint_1 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericLinearFunction<T>
    (37, 1, "EEConstraint_1_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_03 - 1.0*EE_1_1 + w_02_01 - 1.0*w_02_04 + w_03_01 + w_03_02 + w_04_01 + w_04_02 + w_05_01 + w_05_02;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[7] = 1.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 1.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = -1.0; 
			 grad[19] = 1.0; 
			 grad[20] = 1.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 1.0; 
			 grad[26] = 1.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 1.0; 
			 grad[34] = 1.0; 
			 grad[35] = 0.0; 
			 grad[36] = 0.0; 
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
			 const double& EE_1_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
};

template <typename T>
EEConstraint_2<T>::EEConstraint_2 (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericLinearFunction<T>
    (37, 1, "EEConstraint_2_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];
  
	result[0] = w_01_06 - 1.0*EE_1_2 + w_02_02 + w_02_03 + w_03_03 + w_03_04 + w_04_03 + w_04_04 + w_05_03 + w_05_04;
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
	const double& w_01_01 = x[5];
	const double& w_01_02 = x[6];
	const double& w_01_03 = x[7];
	const double& w_01_04 = x[8];
	const double& w_01_05 = x[9];
	const double& w_01_06 = x[10];
	const double& w_01_07 = x[11];
	const double& w_01_08 = x[12];
	const double& w_01_09 = x[13];
	const double& w_01_10 = x[14];
	const double& w_02_01 = x[15];
	const double& w_02_02 = x[16];
	const double& w_02_03 = x[17];
	const double& w_02_04 = x[18];
	const double& w_03_01 = x[19];
	const double& w_03_02 = x[20];
	const double& w_03_03 = x[21];
	const double& w_03_04 = x[22];
	const double& w_03_05 = x[23];
	const double& w_03_06 = x[24];
	const double& w_04_01 = x[25];
	const double& w_04_02 = x[26];
	const double& w_04_03 = x[27];
	const double& w_04_04 = x[28];
	const double& w_04_05 = x[29];
	const double& w_04_06 = x[30];
	const double& w_04_07 = x[31];
	const double& w_04_08 = x[32];
	const double& w_05_01 = x[33];
	const double& w_05_02 = x[34];
	const double& w_05_03 = x[35];
	const double& w_05_04 = x[36];

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
			 grad[10] = 1.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 1.0; 
			 grad[17] = 1.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 1.0; 
			 grad[22] = 1.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 1.0; 
			 grad[28] = 1.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
			 grad[32] = 0.0; 
			 grad[33] = 0.0; 
			 grad[34] = 0.0; 
			 grad[35] = 1.0; 
			 grad[36] = 1.0; 
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

  boost::shared_ptr<CostFunction<roboptim::EigenMatrixDense> > cost = boost::make_shared<CostFunction<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);

	boost::shared_ptr<LiftConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_1 = boost::make_shared<LiftConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_2 = boost::make_shared<LiftConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_3<roboptim::EigenMatrixDense> > cstrFunc_3 = boost::make_shared<LiftConstraint_3<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_4<roboptim::EigenMatrixDense> > cstrFunc_4 = boost::make_shared<LiftConstraint_4<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_5<roboptim::EigenMatrixDense> > cstrFunc_5 = boost::make_shared<LiftConstraint_5<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_6<roboptim::EigenMatrixDense> > cstrFunc_6 = boost::make_shared<LiftConstraint_6<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_7<roboptim::EigenMatrixDense> > cstrFunc_7 = boost::make_shared<LiftConstraint_7<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_8<roboptim::EigenMatrixDense> > cstrFunc_8 = boost::make_shared<LiftConstraint_8<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_9<roboptim::EigenMatrixDense> > cstrFunc_9 = boost::make_shared<LiftConstraint_9<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_10<roboptim::EigenMatrixDense> > cstrFunc_10 = boost::make_shared<LiftConstraint_10<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_11<roboptim::EigenMatrixDense> > cstrFunc_11 = boost::make_shared<LiftConstraint_11<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_12<roboptim::EigenMatrixDense> > cstrFunc_12 = boost::make_shared<LiftConstraint_12<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_13<roboptim::EigenMatrixDense> > cstrFunc_13 = boost::make_shared<LiftConstraint_13<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_14<roboptim::EigenMatrixDense> > cstrFunc_14 = boost::make_shared<LiftConstraint_14<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_15<roboptim::EigenMatrixDense> > cstrFunc_15 = boost::make_shared<LiftConstraint_15<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_16<roboptim::EigenMatrixDense> > cstrFunc_16 = boost::make_shared<LiftConstraint_16<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_17<roboptim::EigenMatrixDense> > cstrFunc_17 = boost::make_shared<LiftConstraint_17<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_18<roboptim::EigenMatrixDense> > cstrFunc_18 = boost::make_shared<LiftConstraint_18<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_19<roboptim::EigenMatrixDense> > cstrFunc_19 = boost::make_shared<LiftConstraint_19<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_20<roboptim::EigenMatrixDense> > cstrFunc_20 = boost::make_shared<LiftConstraint_20<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_21<roboptim::EigenMatrixDense> > cstrFunc_21 = boost::make_shared<LiftConstraint_21<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_22<roboptim::EigenMatrixDense> > cstrFunc_22 = boost::make_shared<LiftConstraint_22<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_23<roboptim::EigenMatrixDense> > cstrFunc_23 = boost::make_shared<LiftConstraint_23<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_24<roboptim::EigenMatrixDense> > cstrFunc_24 = boost::make_shared<LiftConstraint_24<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_25<roboptim::EigenMatrixDense> > cstrFunc_25 = boost::make_shared<LiftConstraint_25<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_26<roboptim::EigenMatrixDense> > cstrFunc_26 = boost::make_shared<LiftConstraint_26<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_27<roboptim::EigenMatrixDense> > cstrFunc_27 = boost::make_shared<LiftConstraint_27<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_28<roboptim::EigenMatrixDense> > cstrFunc_28 = boost::make_shared<LiftConstraint_28<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_29<roboptim::EigenMatrixDense> > cstrFunc_29 = boost::make_shared<LiftConstraint_29<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_30<roboptim::EigenMatrixDense> > cstrFunc_30 = boost::make_shared<LiftConstraint_30<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_31<roboptim::EigenMatrixDense> > cstrFunc_31 = boost::make_shared<LiftConstraint_31<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<LiftConstraint_32<roboptim::EigenMatrixDense> > cstrFunc_32 = boost::make_shared<LiftConstraint_32<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_33 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_34 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);

  //Create problem
  solver_t::problem_t pb (*cost);

  // Set bounds for all optimization parameters
  pb.argumentBounds ()[0] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[1] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[2] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[3] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[4] = roboptim::Function::makeInterval (-pi, pi);

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
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_33), bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
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
		std::vector<double> start (37);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0075618;
		start[6] = 0.0044075;
		start[7] = 0.0035013;
		start[8] = 0.0088387;
		start[9] = 0.0095987;
		start[10] = 0.005801;
		start[11] = 0.0051199;
		start[12] = 0.0031487;
		start[13] = 0.0032426;
		start[14] = 0.00072139;
		start[15] = 0.0043448;
		start[16] = 0.0059376;
		start[17] = 0.003455;
		start[18] = 0.0049701;
		start[19] = 0.0041044;
		start[20] = 0.0027811;
		start[21] = 0.0084351;
		start[22] = 0.00050843;
		start[23] = 0.0073275;
		start[24] = 0.0095045;
		start[25] = 0.0032418;
		start[26] = 0.0086568;
		start[27] = 0.0031797;
		start[28] = 0.0024246;
		start[29] = 0.0023645;
		start[30] = 0.0035566;
		start[31] = 0.0018432;
		start[32] = 0.005643;
		start[33] = 0.0084334;
		start[34] = 0.004485;
		start[35] = 0.0051584;
		start[36] = 0.007206;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0071886;
		start[6] = 0.00055654;
		start[7] = 0.0040133;
		start[8] = 0.0089119;
		start[9] = 0.0055103;
		start[10] = 0.0039955;
		start[11] = 0.009859;
		start[12] = 0.0077845;
		start[13] = 0.0095116;
		start[14] = 0.0060604;
		start[15] = 0.0024223;
		start[16] = 0.0057167;
		start[17] = 0.0071584;
		start[18] = 0.0055519;
		start[19] = 0.0038904;
		start[20] = 0.0070533;
		start[21] = 0.0077881;
		start[22] = 0.0032453;
		start[23] = 0.0095011;
		start[24] = 0.0002415;
		start[25] = 0.0033031;
		start[26] = 0.0029919;
		start[27] = 0.0071408;
		start[28] = 0.0041195;
		start[29] = 0.0091301;
		start[30] = 0.0063744;
		start[31] = 0.0044251;
		start[32] = 0.004546;
		start[33] = 0.0087605;
		start[34] = 0.0037873;
		start[35] = 0.0070683;
		start[36] = 0.0015596;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0091896;
		start[6] = 0.0014724;
		start[7] = 0.0032914;
		start[8] = 0.0033888;
		start[9] = 0.0091738;
		start[10] = 0.0024234;
		start[11] = 0.0076892;
		start[12] = 0.0012321;
		start[13] = 0.0065035;
		start[14] = 0.0036846;
		start[15] = 0.0096229;
		start[16] = 0.0037901;
		start[17] = 0.0058785;
		start[18] = 0.0042511;
		start[19] = 0.00029478;
		start[20] = 0.0026732;
		start[21] = 0.0034988;
		start[22] = 0.0083045;
		start[23] = 0.0037688;
		start[24] = 0.0093083;
		start[25] = 0.0053456;
		start[26] = 0.0071758;
		start[27] = 0.0041338;
		start[28] = 0.0048959;
		start[29] = 0.0055987;
		start[30] = 0.0053202;
		start[31] = 0.0050045;
		start[32] = 0.0004557;
		start[33] = 0.0084202;
		start[34] = 0.0080565;
		start[35] = 0.00052625;
		start[36] = 0.0089241;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0075134;
		start[6] = 0.0091671;
		start[7] = 0.00042693;
		start[8] = 0.0051199;
		start[9] = 0.0059551;
		start[10] = 0.00036217;
		start[11] = 0.00093293;
		start[12] = 0.0097136;
		start[13] = 0.0031535;
		start[14] = 0.0057795;
		start[15] = 0.00162;
		start[16] = 0.0071916;
		start[17] = 0.00024009;
		start[18] = 0.0034263;
		start[19] = 0.0048461;
		start[20] = 0.0098352;
		start[21] = 0.0053181;
		start[22] = 0.0063804;
		start[23] = 0.0035178;
		start[24] = 0.0098315;
		start[25] = 0.0013027;
		start[26] = 0.0027417;
		start[27] = 0.0072978;
		start[28] = 0.004585;
		start[29] = 0.0084704;
		start[30] = 0.0098724;
		start[31] = 0.0087371;
		start[32] = 0.0040414;
		start[33] = 0.0030915;
		start[34] = 0.0058897;
		start[35] = 0.0080157;
		start[36] = 0.0060866;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0086523;
		start[6] = 0.0094724;
		start[7] = 0.003238;
		start[8] = 0.0096692;
		start[9] = 0.0052;
		start[10] = 0.0073296;
		start[11] = 0.0064647;
		start[12] = 0.0068594;
		start[13] = 0.0037081;
		start[14] = 0.0070901;
		start[15] = 0.0055025;
		start[16] = 0.0045312;
		start[17] = 0.0070474;
		start[18] = 0.0072238;
		start[19] = 0.0018796;
		start[20] = 0.0089066;
		start[21] = 0.0023847;
		start[22] = 0.0015859;
		start[23] = 0.00038231;
		start[24] = 0.006116;
		start[25] = 0.0042646;
		start[26] = 0.0088776;
		start[27] = 0.005579;
		start[28] = 0.0036347;
		start[29] = 0.0037322;
		start[30] = 0.0053671;
		start[31] = 0.00062132;
		start[32] = 0.0060367;
		start[33] = 0.0011074;
		start[34] = 0.0018379;
		start[35] = 0.0050771;
		start[36] = 0.0050389;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0040833;
		start[6] = 0.0063258;
		start[7] = 0.0075851;
		start[8] = 0.0064336;
		start[9] = 0.0074219;
		start[10] = 0.0036587;
		start[11] = 0.0039783;
		start[12] = 0.0057242;
		start[13] = 0.00089276;
		start[14] = 0.0083108;
		start[15] = 0.00064567;
		start[16] = 0.0059007;
		start[17] = 0.0055164;
		start[18] = 0.0090891;
		start[19] = 0.002153;
		start[20] = 0.0029392;
		start[21] = 0.0086157;
		start[22] = 0.0090418;
		start[23] = 0.00075928;
		start[24] = 0.0086705;
		start[25] = 0.0092227;
		start[26] = 0.0089711;
		start[27] = 0.0067122;
		start[28] = 0.0038009;
		start[29] = 0.0021867;
		start[30] = 0.0037447;
		start[31] = 0.0058372;
		start[32] = 0.0024484;
		start[33] = 0.0093535;
		start[34] = 0.0041936;
		start[35] = 0.006505;
		start[36] = 0.00045561;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0034771;
		start[6] = 0.0029427;
		start[7] = 0.0030816;
		start[8] = 0.0010644;
		start[9] = 0.0085841;
		start[10] = 0.00026406;
		start[11] = 0.0013081;
		start[12] = 0.0080069;
		start[13] = 0.0041663;
		start[14] = 0.0095638;
		start[15] = 0.0052776;
		start[16] = 0.0098926;
		start[17] = 0.0095606;
		start[18] = 0.0057467;
		start[19] = 0.0032313;
		start[20] = 0.0091614;
		start[21] = 0.0083336;
		start[22] = 0.0026975;
		start[23] = 0.0090911;
		start[24] = 0.007759;
		start[25] = 0.0041899;
		start[26] = 0.0082271;
		start[27] = 0.0038536;
		start[28] = 0.0053129;
		start[29] = 0.00044461;
		start[30] = 0.009913;
		start[31] = 0.0047548;
		start[32] = 0.0092848;
		start[33] = 0.0068148;
		start[34] = 0.0045508;
		start[35] = 0.00092461;
		start[36] = 0.0097985;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0029085;
		start[6] = 0.0031816;
		start[7] = 0.0059562;
		start[8] = 0.0090917;
		start[9] = 0.0036284;
		start[10] = 0.0027641;
		start[11] = 0.005;
		start[12] = 0.0054439;
		start[13] = 0.009958;
		start[14] = 0.0070999;
		start[15] = 0.0076866;
		start[16] = 0.0070743;
		start[17] = 0.0072014;
		start[18] = 0.0071108;
		start[19] = 0.0085446;
		start[20] = 0.0029398;
		start[21] = 0.0084318;
		start[22] = 0.0097574;
		start[23] = 0.0066234;
		start[24] = 0.0041097;
		start[25] = 0.0017198;
		start[26] = 0.0045264;
		start[27] = 0.0090626;
		start[28] = 0.00521;
		start[29] = 0.0085454;
		start[30] = 0.0032482;
		start[31] = 0.0086325;
		start[32] = 0.005215;
		start[33] = 0.0091444;
		start[34] = 0.0027877;
		start[35] = 0.0092623;
		start[36] = 0.0086099;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0031386;
		start[6] = 0.0012496;
		start[7] = 0.0066915;
		start[8] = 0.0078368;
		start[9] = 0.0038583;
		start[10] = 0.0038706;
		start[11] = 0.0085167;
		start[12] = 0.0044907;
		start[13] = 0.0082858;
		start[14] = 0.0083559;
		start[15] = 0.008926;
		start[16] = 0.0075144;
		start[17] = 0.0067463;
		start[18] = 0.0012028;
		start[19] = 0.00029524;
		start[20] = 0.0061358;
		start[21] = 0.00732;
		start[22] = 0.0095249;
		start[23] = 0.0062638;
		start[24] = 0.0046883;
		start[25] = 0.0048613;
		start[26] = 0.0086631;
		start[27] = 0.0081772;
		start[28] = 0.0060838;
		start[29] = 0.0056821;
		start[30] = 0.0050695;
		start[31] = 0.0086499;
		start[32] = 0.0067217;
		start[33] = 0.0028858;
		start[34] = 0.0090585;
		start[35] = 0.00059456;
		start[36] = 0.005285;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0050824;
		start[6] = 0.0048797;
		start[7] = 0.0084973;
		start[8] = 0.0088379;
		start[9] = 0.0093147;
		start[10] = 0.0015049;
		start[11] = 0.00031263;
		start[12] = 0.0019592;
		start[13] = 0.0063697;
		start[14] = 0.0075084;
		start[15] = 0.0030883;
		start[16] = 0.0042567;
		start[17] = 0.0062737;
		start[18] = 0.0062378;
		start[19] = 0.0058815;
		start[20] = 0.0030747;
		start[21] = 0.0032527;
		start[22] = 0.009444;
		start[23] = 0.0086788;
		start[24] = 0.0063314;
		start[25] = 0.0055651;
		start[26] = 0.0036977;
		start[27] = 0.0053119;
		start[28] = 0.0069543;
		start[29] = 0.0073546;
		start[30] = 0.0064272;
		start[31] = 0.0077072;
		start[32] = 0.002246;
		start[33] = 0.0043183;
		start[34] = 0.0028055;
		start[35] = 0.008249;
		start[36] = 0.0056766;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0077772;
		start[6] = 0.0040253;
		start[7] = 0.0068557;
		start[8] = 0.008302;
		start[9] = 0.0081092;
		start[10] = 0.0084317;
		start[11] = 0.0034837;
		start[12] = 0.0093879;
		start[13] = 0.00072642;
		start[14] = 0.0019149;
		start[15] = 0.0054931;
		start[16] = 0.0047307;
		start[17] = 0.00075969;
		start[18] = 0.00027669;
		start[19] = 0.00051651;
		start[20] = 0.0039429;
		start[21] = 0.0097228;
		start[22] = 0.0035793;
		start[23] = 0.005298;
		start[24] = 0.0075438;
		start[25] = 0.0036119;
		start[26] = 0.0021564;
		start[27] = 0.000146;
		start[28] = 0.0067176;
		start[29] = 0.0018341;
		start[30] = 0.0053629;
		start[31] = 0.0026294;
		start[32] = 0.0032701;
		start[33] = 0.0097806;
		start[34] = 0.0019945;
		start[35] = 0.0080249;
		start[36] = 0.0045747;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0057246;
		start[6] = 0.0018583;
		start[7] = 0.0011937;
		start[8] = 0.002607;
		start[9] = 0.00618;
		start[10] = 0.0049701;
		start[11] = 0.0098821;
		start[12] = 0.0014307;
		start[13] = 0.00354;
		start[14] = 0.0052889;
		start[15] = 0.0068506;
		start[16] = 0.0051582;
		start[17] = 0.00090744;
		start[18] = 0.0095889;
		start[19] = 0.0044629;
		start[20] = 0.0032709;
		start[21] = 0.0003627;
		start[22] = 0.0051332;
		start[23] = 0.0080531;
		start[24] = 0.0093599;
		start[25] = 0.0048475;
		start[26] = 0.0046985;
		start[27] = 0.0084431;
		start[28] = 0.0012125;
		start[29] = 0.0015126;
		start[30] = 0.0066457;
		start[31] = 0.0081739;
		start[32] = 0.0081331;
		start[33] = 0.0087252;
		start[34] = 0.0041122;
		start[35] = 0.0011628;
		start[36] = 0.00025963;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0014026;
		start[6] = 0.0067693;
		start[7] = 0.0084136;
		start[8] = 0.0060434;
		start[9] = 0.0017025;
		start[10] = 0.0056785;
		start[11] = 0.0084481;
		start[12] = 0.0038764;
		start[13] = 0.0061434;
		start[14] = 0.0051144;
		start[15] = 0.0042207;
		start[16] = 0.001003;
		start[17] = 0.0077581;
		start[18] = 0.0026111;
		start[19] = 0.00055821;
		start[20] = 0.0036238;
		start[21] = 0.0020473;
		start[22] = 0.0068387;
		start[23] = 0.0012139;
		start[24] = 0.0049651;
		start[25] = 0.0065388;
		start[26] = 0.0053137;
		start[27] = 0.0095487;
		start[28] = 0.0071145;
		start[29] = 0.0084887;
		start[30] = 0.0046457;
		start[31] = 0.0083192;
		start[32] = 0.0055204;
		start[33] = 0.0036359;
		start[34] = 0.0067702;
		start[35] = 0.0076566;
		start[36] = 0.004623;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0061689;
		start[6] = 0.0019528;
		start[7] = 0.0067648;
		start[8] = 0.0019655;
		start[9] = 0.005662;
		start[10] = 0.0095098;
		start[11] = 0.0096236;
		start[12] = 0.0078087;
		start[13] = 0.0077141;
		start[14] = 0.004989;
		start[15] = 0.0011174;
		start[16] = 0.0091542;
		start[17] = 0.0085528;
		start[18] = 0.0019487;
		start[19] = 0.003791;
		start[20] = 0.0019956;
		start[21] = 0.0063321;
		start[22] = 0.0011098;
		start[23] = 0.0097089;
		start[24] = 0.0094332;
		start[25] = 0.0064386;
		start[26] = 0.0042854;
		start[27] = 0.0083614;
		start[28] = 0.0014461;
		start[29] = 0.0016621;
		start[30] = 0.0091448;
		start[31] = 0.0048142;
		start[32] = 0.0039917;
		start[33] = 0.0052;
		start[34] = 0.0091702;
		start[35] = 0.0066596;
		start[36] = 0.0010006;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0092896;
		start[6] = 0.0022597;
		start[7] = 0.0082805;
		start[8] = 0.0092452;
		start[9] = 0.0081052;
		start[10] = 0.0072285;
		start[11] = 0.0014263;
		start[12] = 0.0072537;
		start[13] = 0.007716;
		start[14] = 0.0068577;
		start[15] = 0.0094667;
		start[16] = 0.0071504;
		start[17] = 0.0024544;
		start[18] = 0.0018514;
		start[19] = 0.0026087;
		start[20] = 0.00078525;
		start[21] = 0.00031034;
		start[22] = 0.0048398;
		start[23] = 0.0040713;
		start[24] = 0.0046867;
		start[25] = 0.0062795;
		start[26] = 0.002927;
		start[27] = 0.0010545;
		start[28] = 0.005327;
		start[29] = 0.0091461;
		start[30] = 0.0061133;
		start[31] = 0.0088339;
		start[32] = 0.003228;
		start[33] = 0.0022381;
		start[34] = 0.0072978;
		start[35] = 0.0035171;
		start[36] = 0.0037543;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0029222;
		start[6] = 0.0033631;
		start[7] = 0.0083472;
		start[8] = 0.0093096;
		start[9] = 0.006604;
		start[10] = 0.0080044;
		start[11] = 0.0028981;
		start[12] = 0.0098012;
		start[13] = 0.0093466;
		start[14] = 0.0025289;
		start[15] = 0.0070201;
		start[16] = 0.002272;
		start[17] = 0.007026;
		start[18] = 0.0037239;
		start[19] = 0.0076436;
		start[20] = 0.0057882;
		start[21] = 0.0034662;
		start[22] = 0.006479;
		start[23] = 0.0079011;
		start[24] = 0.0021432;
		start[25] = 0.0096422;
		start[26] = 0.0018443;
		start[27] = 0.0068845;
		start[28] = 0.0087784;
		start[29] = 0.0096048;
		start[30] = 0.0044805;
		start[31] = 0.0066016;
		start[32] = 0.0017791;
		start[33] = 0.005702;
		start[34] = 0.0087536;
		start[35] = 0.0018896;
		start[36] = 0.0052063;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0041291;
		start[6] = 0.0039076;
		start[7] = 0.0046086;
		start[8] = 0.0084193;
		start[9] = 0.001055;
		start[10] = 0.0058381;
		start[11] = 0.0041764;
		start[12] = 0.0028588;
		start[13] = 0.0085288;
		start[14] = 0.0032296;
		start[15] = 0.00047341;
		start[16] = 0.0073675;
		start[17] = 0.00026057;
		start[18] = 5.9787e-05;
		start[19] = 0.004256;
		start[20] = 0.0011431;
		start[21] = 0.0011073;
		start[22] = 0.0060038;
		start[23] = 0.0037117;
		start[24] = 0.0019776;
		start[25] = 0.0054438;
		start[26] = 0.004056;
		start[27] = 0.0088659;
		start[28] = 0.0032527;
		start[29] = 0.0037013;
		start[30] = 0.0044072;
		start[31] = 0.0011084;
		start[32] = 0.0053054;
		start[33] = 0.0013922;
		start[34] = 0.0064837;
		start[35] = 0.009576;
		start[36] = 0.0066425;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.007958;
		start[6] = 0.0042383;
		start[7] = 0.0014659;
		start[8] = 0.0048306;
		start[9] = 0.0082986;
		start[10] = 0.0030703;
		start[11] = 0.0090024;
		start[12] = 0.0053633;
		start[13] = 0.00066751;
		start[14] = 0.001738;
		start[15] = 0.00067376;
		start[16] = 0.0082182;
		start[17] = 0.0028162;
		start[18] = 0.0080298;
		start[19] = 0.0012543;
		start[20] = 0.0038086;
		start[21] = 0.0020724;
		start[22] = 0.0018394;
		start[23] = 0.0073456;
		start[24] = 0.0075899;
		start[25] = 0.0097667;
		start[26] = 0.0068282;
		start[27] = 0.0082262;
		start[28] = 0.0016115;
		start[29] = 0.00072402;
		start[30] = 0.0047776;
		start[31] = 0.0036494;
		start[32] = 0.006526;
		start[33] = 0.0078138;
		start[34] = 0.0062347;
		start[35] = 0.0004276;
		start[36] = 0.0055382;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0082451;
		start[6] = 0.0052451;
		start[7] = 0.0097454;
		start[8] = 0.0053174;
		start[9] = 0.0094174;
		start[10] = 0.0077115;
		start[11] = 0.0098844;
		start[12] = 0.0054218;
		start[13] = 0.006945;
		start[14] = 0.0047198;
		start[15] = 0.0025475;
		start[16] = 0.0017543;
		start[17] = 0.0088876;
		start[18] = 0.0070466;
		start[19] = 0.00087997;
		start[20] = 0.0066118;
		start[21] = 0.0090939;
		start[22] = 0.0073394;
		start[23] = 0.0025502;
		start[24] = 0.0016306;
		start[25] = 0.0025286;
		start[26] = 0.0066503;
		start[27] = 0.0047792;
		start[28] = 0.0048753;
		start[29] = 0.0028724;
		start[30] = 0.0066566;
		start[31] = 0.0051219;
		start[32] = 0.006254;
		start[33] = 0.0047755;
		start[34] = 0.0033343;
		start[35] = 0.0089352;
		start[36] = 0.0036103;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0054395;
		start[6] = 0.0018453;
		start[7] = 0.0096466;
		start[8] = 0.0085783;
		start[9] = 0.0021144;
		start[10] = 0.0051695;
		start[11] = 0.00051135;
		start[12] = 0.004141;
		start[13] = 0.0033554;
		start[14] = 0.0034731;
		start[15] = 0.00049203;
		start[16] = 0.0023112;
		start[17] = 0.0021146;
		start[18] = 0.0020441;
		start[19] = 0.0070196;
		start[20] = 0.0018171;
		start[21] = 0.0032883;
		start[22] = 0.0025435;
		start[23] = 0.0051879;
		start[24] = 0.0056965;
		start[25] = 0.0024195;
		start[26] = 0.0015547;
		start[27] = 0.0085582;
		start[28] = 0.002731;
		start[29] = 0.0053711;
		start[30] = 0.0064063;
		start[31] = 0.0086702;
		start[32] = 0.0097227;
		start[33] = 0.005712;
		start[34] = 0.0076529;
		start[35] = 0.00709;
		start[36] = 0.0096601;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.006285;
		start[6] = 0.0054691;
		start[7] = 0.0056181;
		start[8] = 0.0020095;
		start[9] = 0.0067456;
		start[10] = 0.0018368;
		start[11] = 0.0064232;
		start[12] = 0.0043766;
		start[13] = 0.006716;
		start[14] = 0.0083998;
		start[15] = 0.0079081;
		start[16] = 0.0076595;
		start[17] = 0.0054849;
		start[18] = 0.0001098;
		start[19] = 0.0080932;
		start[20] = 0.0081424;
		start[21] = 0.0065102;
		start[22] = 0.00053845;
		start[23] = 0.0044349;
		start[24] = 0.0029846;
		start[25] = 0.0043247;
		start[26] = 0.0099552;
		start[27] = 0.0095743;
		start[28] = 0.0062437;
		start[29] = 0.0035492;
		start[30] = 0.0093716;
		start[31] = 0.0013842;
		start[32] = 0.0027318;
		start[33] = 0.0026875;
		start[34] = 0.0066967;
		start[35] = 0.00072724;
		start[36] = 0.0016711;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0052282;
		start[6] = 0.0020737;
		start[7] = 0.0066537;
		start[8] = 0.0095329;
		start[9] = 0.0041338;
		start[10] = 0.00046699;
		start[11] = 0.00463;
		start[12] = 0.0080869;
		start[13] = 0.0096446;
		start[14] = 0.0012791;
		start[15] = 0.0040793;
		start[16] = 0.0061149;
		start[17] = 0.0017117;
		start[18] = 0.0068058;
		start[19] = 0.0049806;
		start[20] = 0.0042399;
		start[21] = 0.009871;
		start[22] = 0.0044157;
		start[23] = 0.0078721;
		start[24] = 0.005458;
		start[25] = 0.0085975;
		start[26] = 0.0090495;
		start[27] = 0.0078577;
		start[28] = 0.0088118;
		start[29] = 0.00098249;
		start[30] = 0.0022576;
		start[31] = 0.0064983;
		start[32] = 0.0047171;
		start[33] = 0.0070165;
		start[34] = 0.0014804;
		start[35] = 0.0020089;
		start[36] = 0.0037684;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0033259;
		start[6] = 0.0034757;
		start[7] = 0.0061227;
		start[8] = 0.0050612;
		start[9] = 0.0025537;
		start[10] = 0.0099296;
		start[11] = 0.0053369;
		start[12] = 0.0031288;
		start[13] = 0.0040424;
		start[14] = 0.0078901;
		start[15] = 0.00021328;
		start[16] = 0.0011987;
		start[17] = 0.0028833;
		start[18] = 0.009021;
		start[19] = 0.00507;
		start[20] = 0.0091078;
		start[21] = 0.002698;
		start[22] = 0.0083406;
		start[23] = 0.0062136;
		start[24] = 0.00586;
		start[25] = 0.0062177;
		start[26] = 0.0035942;
		start[27] = 0.0047495;
		start[28] = 0.0031396;
		start[29] = 0.00066463;
		start[30] = 0.0024302;
		start[31] = 0.0060584;
		start[32] = 0.0043201;
		start[33] = 0.0043513;
		start[34] = 0.00094993;
		start[35] = 0.00059415;
		start[36] = 0.0087315;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0016631;
		start[6] = 0.0014157;
		start[7] = 0.0065512;
		start[8] = 0.00085736;
		start[9] = 0.0077493;
		start[10] = 0.0052352;
		start[11] = 0.0024222;
		start[12] = 0.0012827;
		start[13] = 0.0012153;
		start[14] = 0.00045482;
		start[15] = 0.0079032;
		start[16] = 0.00078209;
		start[17] = 0.0046165;
		start[18] = 0.0085332;
		start[19] = 0.0065343;
		start[20] = 0.0080172;
		start[21] = 0.0098509;
		start[22] = 0.0070775;
		start[23] = 0.0037247;
		start[24] = 0.0070427;
		start[25] = 0.0080482;
		start[26] = 0.00088457;
		start[27] = 0.00069213;
		start[28] = 0.0077317;
		start[29] = 0.0086502;
		start[30] = 0.0035375;
		start[31] = 0.0068464;
		start[32] = 0.0060443;
		start[33] = 0.0019549;
		start[34] = 0.0064865;
		start[35] = 0.0032912;
		start[36] = 0.0042392;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0080943;
		start[6] = 0.0070911;
		start[7] = 0.0030039;
		start[8] = 0.00099304;
		start[9] = 0.0035088;
		start[10] = 0.0086416;
		start[11] = 0.0088035;
		start[12] = 0.0098048;
		start[13] = 0.0043433;
		start[14] = 0.001898;
		start[15] = 0.0037124;
		start[16] = 0.0038662;
		start[17] = 0.0020051;
		start[18] = 0.008325;
		start[19] = 0.0035285;
		start[20] = 0.0043229;
		start[21] = 0.003378;
		start[22] = 0.001493;
		start[23] = 0.0022792;
		start[24] = 0.0062177;
		start[25] = 0.0077981;
		start[26] = 0.0023277;
		start[27] = 0.00023571;
		start[28] = 0.0058924;
		start[29] = 0.0015511;
		start[30] = 0.0013278;
		start[31] = 0.0060221;
		start[32] = 0.0072097;
		start[33] = 0.0073467;
		start[34] = 0.0088429;
		start[35] = 0.0084776;
		start[36] = 0.0027522;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0049281;
		start[6] = 0.0037047;
		start[7] = 0.0076494;
		start[8] = 0.0080456;
		start[9] = 0.005854;
		start[10] = 0.0067987;
		start[11] = 0.0022471;
		start[12] = 0.0021198;
		start[13] = 0.0068597;
		start[14] = 0.0089004;
		start[15] = 0.0025312;
		start[16] = 0.0084587;
		start[17] = 0.0012596;
		start[18] = 0.006387;
		start[19] = 0.0094857;
		start[20] = 0.0078807;
		start[21] = 0.0061011;
		start[22] = 0.0057083;
		start[23] = 5.6854e-05;
		start[24] = 0.0060323;
		start[25] = 0.0024821;
		start[26] = 0.0030827;
		start[27] = 0.0070319;
		start[28] = 0.0049412;
		start[29] = 0.0096722;
		start[30] = 0.00051533;
		start[31] = 0.0092717;
		start[32] = 0.0026631;
		start[33] = 0.0094418;
		start[34] = 0.0064392;
		start[35] = 0.0047357;
		start[36] = 0.0078761;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 8.2443e-05;
		start[6] = 0.0014752;
		start[7] = 0.0068572;
		start[8] = 0.0054012;
		start[9] = 0.005457;
		start[10] = 0.0048278;
		start[11] = 0.0010923;
		start[12] = 0.0056644;
		start[13] = 0.0034794;
		start[14] = 0.0082309;
		start[15] = 0.0047584;
		start[16] = 0.009796;
		start[17] = 0.00062654;
		start[18] = 0.0022006;
		start[19] = 0.0056997;
		start[20] = 0.0029432;
		start[21] = 0.0034979;
		start[22] = 0.0050229;
		start[23] = 0.0090152;
		start[24] = 0.0049153;
		start[25] = 0.0005487;
		start[26] = 0.0017658;
		start[27] = 0.004092;
		start[28] = 0.0065032;
		start[29] = 0.0048848;
		start[30] = 0.00937;
		start[31] = 0.0089934;
		start[32] = 0.0066009;
		start[33] = 0.0083138;
		start[34] = 0.0086278;
		start[35] = 0.0035363;
		start[36] = 0.0039576;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.00061532;
		start[6] = 0.0057658;
		start[7] = 0.0073442;
		start[8] = 0.0091077;
		start[9] = 0.0049829;
		start[10] = 0.0034164;
		start[11] = 0.0025481;
		start[12] = 0.0085957;
		start[13] = 0.0055717;
		start[14] = 0.0096744;
		start[15] = 0.0092848;
		start[16] = 0.0041162;
		start[17] = 0.00075333;
		start[18] = 0.0097171;
		start[19] = 0.0037259;
		start[20] = 0.0010553;
		start[21] = 0.0024638;
		start[22] = 0.0088685;
		start[23] = 0.0069367;
		start[24] = 0.0023996;
		start[25] = 0.0087493;
		start[26] = 0.007274;
		start[27] = 0.0072547;
		start[28] = 0.0065717;
		start[29] = 0.0070508;
		start[30] = 4.4752e-05;
		start[31] = 0.0083516;
		start[32] = 0.0062168;
		start[33] = 0.0029445;
		start[34] = 0.0055794;
		start[35] = 0.001113;
		start[36] = 0.0022235;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0053558;
		start[6] = 0.0059939;
		start[7] = 0.0088646;
		start[8] = 0.0015058;
		start[9] = 0.0099529;
		start[10] = 0.0022488;
		start[11] = 0.0095657;
		start[12] = 0.0016679;
		start[13] = 0.0066433;
		start[14] = 0.0033499;
		start[15] = 0.0051324;
		start[16] = 0.0046094;
		start[17] = 0.0066101;
		start[18] = 0.0012672;
		start[19] = 0.0054369;
		start[20] = 0.0013242;
		start[21] = 0.0041849;
		start[22] = 0.0062926;
		start[23] = 0.0051081;
		start[24] = 0.0093478;
		start[25] = 0.0061251;
		start[26] = 0.0099531;
		start[27] = 0.0096035;
		start[28] = 0.0062841;
		start[29] = 0.0024492;
		start[30] = 0.0043218;
		start[31] = 0.0041825;
		start[32] = 0.0090766;
		start[33] = 0.0012686;
		start[34] = 0.0098452;
		start[35] = 0.0022421;
		start[36] = 0.0072331;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0085714;
		start[6] = 0.0029079;
		start[7] = 0.0096003;
		start[8] = 0.003898;
		start[9] = 0.0061176;
		start[10] = 0.0077936;
		start[11] = 0.0030999;
		start[12] = 0.0097191;
		start[13] = 0.0081343;
		start[14] = 0.0095094;
		start[15] = 0.0042228;
		start[16] = 0.003701;
		start[17] = 0.0065347;
		start[18] = 0.0088168;
		start[19] = 0.0081539;
		start[20] = 0.005983;
		start[21] = 0.0050118;
		start[22] = 0.0041719;
		start[23] = 0.0048982;
		start[24] = 0.0092158;
		start[25] = 0.00531;
		start[26] = 0.0078697;
		start[27] = 0.00093595;
		start[28] = 0.0010069;
		start[29] = 0.0027104;
		start[30] = 0.0094743;
		start[31] = 0.0097554;
		start[32] = 0.0072193;
		start[33] = 0.0066684;
		start[34] = 0.0086503;
		start[35] = 0.0043314;
		start[36] = 0.0026045;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0042675;
		start[6] = 0.0085333;
		start[7] = 0.0080034;
		start[8] = 0.0089214;
		start[9] = 0.0013781;
		start[10] = 0.0077417;
		start[11] = 0.0030601;
		start[12] = 0.0020056;
		start[13] = 0.004203;
		start[14] = 0.00049853;
		start[15] = 0.0028418;
		start[16] = 0.00089626;
		start[17] = 0.0004148;
		start[18] = 0.003071;
		start[19] = 0.0083045;
		start[20] = 0.00010892;
		start[21] = 0.0085267;
		start[22] = 0.0034092;
		start[23] = 0.0035986;
		start[24] = 0.0039481;
		start[25] = 0.0067049;
		start[26] = 0.0013856;
		start[27] = 0.0039198;
		start[28] = 0.0031785;
		start[29] = 0.0082578;
		start[30] = 0.0010775;
		start[31] = 0.0093866;
		start[32] = 0.0041245;
		start[33] = 0.0077848;
		start[34] = 0.0043439;
		start[35] = 0.0018908;
		start[36] = 0.00079325;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0090996;
		start[6] = 0.003623;
		start[7] = 0.0087854;
		start[8] = 0.0052467;
		start[9] = 0.0042108;
		start[10] = 0.00889;
		start[11] = 0.0038131;
		start[12] = 0.0036632;
		start[13] = 0.0021856;
		start[14] = 0.0088618;
		start[15] = 0.0039781;
		start[16] = 0.0014217;
		start[17] = 0.0081595;
		start[18] = 0.0045931;
		start[19] = 0.0025914;
		start[20] = 0.0070362;
		start[21] = 0.0043962;
		start[22] = 0.00025883;
		start[23] = 0.0032168;
		start[24] = 0.004291;
		start[25] = 0.0027246;
		start[26] = 0.0016012;
		start[27] = 0.0079143;
		start[28] = 0.005477;
		start[29] = 0.0031045;
		start[30] = 0.0089532;
		start[31] = 0.0036626;
		start[32] = 0.0032902;
		start[33] = 0.0061476;
		start[34] = 0.0019605;
		start[35] = 0.003272;
		start[36] = 0.0017735;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0045833;
		start[6] = 0.0080097;
		start[7] = 0.0043543;
		start[8] = 0.0008303;
		start[9] = 0.0058177;
		start[10] = 0.00554;
		start[11] = 0.00035769;
		start[12] = 0.0016475;
		start[13] = 0.0092084;
		start[14] = 0.0038264;
		start[15] = 0.0051861;
		start[16] = 0.0031803;
		start[17] = 0.0053301;
		start[18] = 0.0068191;
		start[19] = 0.0033125;
		start[20] = 0.0020535;
		start[21] = 0.0066374;
		start[22] = 0.0094483;
		start[23] = 0.001347;
		start[24] = 0.0030696;
		start[25] = 0.0074377;
		start[26] = 0.0059754;
		start[27] = 0.00052299;
		start[28] = 0.006129;
		start[29] = 0.0024698;
		start[30] = 0.0035544;
		start[31] = 0.00022941;
		start[32] = 0.0058434;
		start[33] = 0.00087656;
		start[34] = 0.0067012;
		start[35] = 0.0057027;
		start[36] = 0.0091982;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0027189;
		start[6] = 0.008101;
		start[7] = 0.0013386;
		start[8] = 0.0010684;
		start[9] = 0.0080005;
		start[10] = 0.0030865;
		start[11] = 0.00081561;
		start[12] = 0.0097563;
		start[13] = 0.0023396;
		start[14] = 0.0018227;
		start[15] = 0.0084113;
		start[16] = 0.0024605;
		start[17] = 0.00091497;
		start[18] = 0.0098879;
		start[19] = 0.0045685;
		start[20] = 0.0056213;
		start[21] = 0.0035216;
		start[22] = 0.00053152;
		start[23] = 0.00012946;
		start[24] = 0.0092979;
		start[25] = 0.0044246;
		start[26] = 0.0070165;
		start[27] = 0.0094228;
		start[28] = 0.00016522;
		start[29] = 0.0026009;
		start[30] = 0.00477;
		start[31] = 0.0056811;
		start[32] = 0.0075896;
		start[33] = 0.0025142;
		start[34] = 0.0040364;
		start[35] = 0.00077325;
		start[36] = 0.0020194;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0088948;
		start[6] = 0.0065797;
		start[7] = 0.0063477;
		start[8] = 0.00015725;
		start[9] = 0.0018459;
		start[10] = 0.00024898;
		start[11] = 0.0079462;
		start[12] = 0.003014;
		start[13] = 0.0048648;
		start[14] = 0.0049314;
		start[15] = 0.00060259;
		start[16] = 0.0071407;
		start[17] = 4.3739e-06;
		start[18] = 0.0071789;
		start[19] = 0.0046076;
		start[20] = 0.0070321;
		start[21] = 0.007888;
		start[22] = 8.6675e-05;
		start[23] = 0.0025054;
		start[24] = 0.00093206;
		start[25] = 0.0078461;
		start[26] = 0.00016019;
		start[27] = 0.0083467;
		start[28] = 0.00069172;
		start[29] = 0.0019008;
		start[30] = 0.00077143;
		start[31] = 0.0075082;
		start[32] = 0.0014596;
		start[33] = 0.0039015;
		start[34] = 0.0045911;
		start[35] = 0.0019902;
		start[36] = 0.0032367;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0095829;
		start[6] = 0.0015797;
		start[7] = 0.0071597;
		start[8] = 0.0023494;
		start[9] = 0.0081118;
		start[10] = 0.0078425;
		start[11] = 0.0074285;
		start[12] = 0.0074256;
		start[13] = 0.00086362;
		start[14] = 0.0030432;
		start[15] = 0.0045787;
		start[16] = 0.0054428;
		start[17] = 0.0042495;
		start[18] = 0.0022475;
		start[19] = 0.0035638;
		start[20] = 0.00087843;
		start[21] = 0.0042039;
		start[22] = 0.00054499;
		start[23] = 0.009984;
		start[24] = 0.001898;
		start[25] = 0.0026899;
		start[26] = 0.0010898;
		start[27] = 0.0095047;
		start[28] = 0.0065746;
		start[29] = 0.008769;
		start[30] = 0.0091222;
		start[31] = 0.00028904;
		start[32] = 0.0045727;
		start[33] = 0.0042497;
		start[34] = 0.0048753;
		start[35] = 0.0090812;
		start[36] = 0.0090269;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0062924;
		start[6] = 0.0086531;
		start[7] = 0.0066729;
		start[8] = 0.0035129;
		start[9] = 0.0022418;
		start[10] = 0.0017102;
		start[11] = 4.0479e-05;
		start[12] = 0.004887;
		start[13] = 0.0048063;
		start[14] = 0.0027736;
		start[15] = 0.0076698;
		start[16] = 0.0075541;
		start[17] = 0.0029979;
		start[18] = 0.0048363;
		start[19] = 0.0039923;
		start[20] = 0.0037866;
		start[21] = 0.0050795;
		start[22] = 0.0076768;
		start[23] = 0.0038683;
		start[24] = 0.0064259;
		start[25] = 0.0030676;
		start[26] = 0.0041256;
		start[27] = 0.0012122;
		start[28] = 0.00718;
		start[29] = 0.0052281;
		start[30] = 0.003866;
		start[31] = 0.0050338;
		start[32] = 0.00070982;
		start[33] = 0.0059602;
		start[34] = 0.0063562;
		start[35] = 0.0034058;
		start[36] = 0.0022813;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0036583;
		start[6] = 0.00037036;
		start[7] = 0.0017946;
		start[8] = 0.0011835;
		start[9] = 0.0069284;
		start[10] = 0.0072415;
		start[11] = 3.0048e-06;
		start[12] = 0.0066586;
		start[13] = 0.0094985;
		start[14] = 0.0066398;
		start[15] = 0.00024489;
		start[16] = 0.00050229;
		start[17] = 0.004226;
		start[18] = 0.0014432;
		start[19] = 0.0033252;
		start[20] = 0.0027787;
		start[21] = 0.0096957;
		start[22] = 0.0026582;
		start[23] = 0.0040387;
		start[24] = 0.0026823;
		start[25] = 0.0078037;
		start[26] = 0.0071038;
		start[27] = 0.0075762;
		start[28] = 0.0023463;
		start[29] = 0.0042723;
		start[30] = 0.0015472;
		start[31] = 0.00017787;
		start[32] = 0.0056018;
		start[33] = 0.0039873;
		start[34] = 0.0017905;
		start[35] = 0.0078519;
		start[36] = 0.0030963;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0052677;
		start[6] = 0.0031005;
		start[7] = 0.0087627;
		start[8] = 0.00084063;
		start[9] = 0.0047814;
		start[10] = 0.0011735;
		start[11] = 0.0071102;
		start[12] = 0.0054697;
		start[13] = 0.0094891;
		start[14] = 0.0067582;
		start[15] = 0.0027694;
		start[16] = 0.0046582;
		start[17] = 0.006627;
		start[18] = 0.0062155;
		start[19] = 0.000966;
		start[20] = 0.0074107;
		start[21] = 0.0085447;
		start[22] = 0.0067709;
		start[23] = 0.007085;
		start[24] = 0.00096696;
		start[25] = 0.0038586;
		start[26] = 0.00054228;
		start[27] = 0.0078956;
		start[28] = 0.0018573;
		start[29] = 0.0034295;
		start[30] = 0.0008032;
		start[31] = 0.0081865;
		start[32] = 0.0016084;
		start[33] = 0.0078049;
		start[34] = 0.0097011;
		start[35] = 0.0037617;
		start[36] = 0.0080373;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0014923;
		start[6] = 0.0088094;
		start[7] = 0.0049563;
		start[8] = 0.0015577;
		start[9] = 0.0068044;
		start[10] = 0.0061572;
		start[11] = 0.0060682;
		start[12] = 0.0077108;
		start[13] = 0.002734;
		start[14] = 0.0097515;
		start[15] = 0.0045337;
		start[16] = 0.0042968;
		start[17] = 0.0049923;
		start[18] = 0.0005773;
		start[19] = 0.0044273;
		start[20] = 0.0023633;
		start[21] = 0.0019882;
		start[22] = 0.0051188;
		start[23] = 0.0063921;
		start[24] = 0.0013692;
		start[25] = 0.0026361;
		start[26] = 0.0071837;
		start[27] = 0.0050085;
		start[28] = 0.0016143;
		start[29] = 0.0036276;
		start[30] = 0.001207;
		start[31] = 0.0071931;
		start[32] = 0.0027257;
		start[33] = 0.006791;
		start[34] = 0.0067682;
		start[35] = 0.0026115;
		start[36] = 0.0048288;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0048248;
		start[6] = 0.0040659;
		start[7] = 0.0086911;
		start[8] = 0.0087485;
		start[9] = 0.0036472;
		start[10] = 0.0058476;
		start[11] = 0.0080545;
		start[12] = 0.0074723;
		start[13] = 0.0020336;
		start[14] = 0.001842;
		start[15] = 0.0024627;
		start[16] = 0.0087311;
		start[17] = 0.0043289;
		start[18] = 0.0029606;
		start[19] = 0.004341;
		start[20] = 0.0058753;
		start[21] = 0.0071495;
		start[22] = 0.0017808;
		start[23] = 0.0088981;
		start[24] = 0.0035446;
		start[25] = 0.0090139;
		start[26] = 0.0057334;
		start[27] = 0.0045146;
		start[28] = 0.0047874;
		start[29] = 0.0073668;
		start[30] = 0.0058613;
		start[31] = 0.007267;
		start[32] = 0.0039233;
		start[33] = 0.0088616;
		start[34] = 0.0096027;
		start[35] = 0.0057802;
		start[36] = 0.0032474;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.00042245;
		start[6] = 0.0039656;
		start[7] = 0.0073018;
		start[8] = 0.0060697;
		start[9] = 0.0071899;
		start[10] = 0.00251;
		start[11] = 0.00053338;
		start[12] = 0.0075905;
		start[13] = 0.0078593;
		start[14] = 0.00084634;
		start[15] = 0.0076615;
		start[16] = 0.0077985;
		start[17] = 0.007917;
		start[18] = 0.0081448;
		start[19] = 0.0059206;
		start[20] = 0.0049951;
		start[21] = 0.005803;
		start[22] = 0.0012466;
		start[23] = 0.0079098;
		start[24] = 0.0026755;
		start[25] = 0.0078036;
		start[26] = 0.0086662;
		start[27] = 0.0080784;
		start[28] = 0.0034129;
		start[29] = 0.0021671;
		start[30] = 0.0010631;
		start[31] = 0.0080343;
		start[32] = 0.0051221;
		start[33] = 0.0044735;
		start[34] = 0.0058474;
		start[35] = 0.003125;
		start[36] = 0.002041;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0063774;
		start[6] = 0.0048549;
		start[7] = 0.0010266;
		start[8] = 0.00063373;
		start[9] = 0.007221;
		start[10] = 0.0024237;
		start[11] = 0.0070192;
		start[12] = 0.0054698;
		start[13] = 0.0076925;
		start[14] = 0.0072912;
		start[15] = 0.0056078;
		start[16] = 0.0026875;
		start[17] = 0.0034634;
		start[18] = 0.0076405;
		start[19] = 0.005604;
		start[20] = 0.0011503;
		start[21] = 0.0059471;
		start[22] = 0.0041651;
		start[23] = 0.0042842;
		start[24] = 0.0012476;
		start[25] = 0.003405;
		start[26] = 0.0036335;
		start[27] = 0.0060387;
		start[28] = 0.0067393;
		start[29] = 0.002151;
		start[30] = 0.001507;
		start[31] = 0.0012317;
		start[32] = 0.0025743;
		start[33] = 0.00034813;
		start[34] = 0.0085018;
		start[35] = 0.0072752;
		start[36] = 0.0041293;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0014606;
		start[6] = 0.0054768;
		start[7] = 0.0080743;
		start[8] = 0.0036331;
		start[9] = 0.004733;
		start[10] = 0.0044875;
		start[11] = 0.0071228;
		start[12] = 0.0080815;
		start[13] = 0.0022137;
		start[14] = 0.0029886;
		start[15] = 0.0098031;
		start[16] = 0.0070416;
		start[17] = 0.0040064;
		start[18] = 0.0037693;
		start[19] = 0.00332;
		start[20] = 0.007333;
		start[21] = 0.0054246;
		start[22] = 0.0055284;
		start[23] = 0.001378;
		start[24] = 0.0068438;
		start[25] = 0.0028787;
		start[26] = 0.0092004;
		start[27] = 0.0012372;
		start[28] = 0.0075687;
		start[29] = 0.0044367;
		start[30] = 0.0088817;
		start[31] = 0.008493;
		start[32] = 0.0064957;
		start[33] = 0.009732;
		start[34] = 0.0086397;
		start[35] = 0.004733;
		start[36] = 0.0090671;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0032527;
		start[6] = 0.0063684;
		start[7] = 0.0093571;
		start[8] = 0.0040407;
		start[9] = 0.0088574;
		start[10] = 0.0061703;
		start[11] = 0.00090247;
		start[12] = 0.0071962;
		start[13] = 0.0012666;
		start[14] = 0.008532;
		start[15] = 0.00050885;
		start[16] = 0.0036184;
		start[17] = 0.0087328;
		start[18] = 0.0051526;
		start[19] = 0.0058703;
		start[20] = 0.0070077;
		start[21] = 0.0086298;
		start[22] = 0.0020979;
		start[23] = 0.0038527;
		start[24] = 0.0080069;
		start[25] = 0.0049623;
		start[26] = 0.00099189;
		start[27] = 0.0039545;
		start[28] = 0.0089774;
		start[29] = 0.0015247;
		start[30] = 0.0076356;
		start[31] = 0.0097788;
		start[32] = 0.0015638;
		start[33] = 0.0090533;
		start[34] = 0.0035462;
		start[35] = 0.0069133;
		start[36] = 0.00072577;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0035731;
		start[6] = 0.0085859;
		start[7] = 0.0044968;
		start[8] = 0.0010495;
		start[9] = 0.0051618;
		start[10] = 0.0042129;
		start[11] = 0.0078177;
		start[12] = 0.0091531;
		start[13] = 0.0067743;
		start[14] = 0.0059166;
		start[15] = 0.0050176;
		start[16] = 0.008614;
		start[17] = 0.0036759;
		start[18] = 0.0058825;
		start[19] = 0.00059062;
		start[20] = 0.0073791;
		start[21] = 0.0039538;
		start[22] = 0.0021676;
		start[23] = 0.0016259;
		start[24] = 0.0059619;
		start[25] = 0.0013764;
		start[26] = 9.8551e-05;
		start[27] = 0.0082504;
		start[28] = 0.0018694;
		start[29] = 0.00079398;
		start[30] = 0.00090966;
		start[31] = 0.0075451;
		start[32] = 0.0092477;
		start[33] = 0.0010855;
		start[34] = 0.0062874;
		start[35] = 0.0082644;
		start[36] = 0.00088587;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.00019594;
		start[6] = 0.0089935;
		start[7] = 0.004829;
		start[8] = 0.008143;
		start[9] = 0.0092743;
		start[10] = 0.001383;
		start[11] = 0.0014249;
		start[12] = 0.0087372;
		start[13] = 0.0030971;
		start[14] = 0.0017109;
		start[15] = 0.0084111;
		start[16] = 0.0046708;
		start[17] = 0.0053539;
		start[18] = 0.0098625;
		start[19] = 0.0011149;
		start[20] = 0.006371;
		start[21] = 0.0013115;
		start[22] = 0.0090255;
		start[23] = 0.0091928;
		start[24] = 0.0055523;
		start[25] = 0.003268;
		start[26] = 0.0091307;
		start[27] = 0.0043397;
		start[28] = 0.0078334;
		start[29] = 0.0023202;
		start[30] = 0.0024734;
		start[31] = 0.00061166;
		start[32] = 0.0053463;
		start[33] = 0.0048716;
		start[34] = 0.0079998;
		start[35] = 0.0067719;
		start[36] = 0.0069762;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0035583;
		start[6] = 0.0055557;
		start[7] = 0.0053101;
		start[8] = 0.0038421;
		start[9] = 0.0075125;
		start[10] = 0.0067983;
		start[11] = 0.005911;
		start[12] = 0.00071723;
		start[13] = 0.0015022;
		start[14] = 0.0032138;
		start[15] = 0.0087315;
		start[16] = 0.0078986;
		start[17] = 0.0067342;
		start[18] = 0.004002;
		start[19] = 0.0095375;
		start[20] = 0.0085241;
		start[21] = 0.0038902;
		start[22] = 0.0068048;
		start[23] = 0.0046728;
		start[24] = 0.0015084;
		start[25] = 0.0024041;
		start[26] = 0.0021821;
		start[27] = 0.0058201;
		start[28] = 0.003232;
		start[29] = 0.00026502;
		start[30] = 0.0060309;
		start[31] = 0.0054633;
		start[32] = 0.0060561;
		start[33] = 0.0048694;
		start[34] = 0.0099679;
		start[35] = 0.0031009;
		start[36] = 0.0027151;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0020515;
		start[6] = 0.0086058;
		start[7] = 0.0012839;
		start[8] = 0.0081759;
		start[9] = 0.0085958;
		start[10] = 0.0049739;
		start[11] = 0.0087241;
		start[12] = 0.0071058;
		start[13] = 0.0062965;
		start[14] = 0.006315;
		start[15] = 0.0092166;
		start[16] = 0.0098346;
		start[17] = 0.0066953;
		start[18] = 0.0014731;
		start[19] = 0.0003341;
		start[20] = 0.002142;
		start[21] = 0.0033471;
		start[22] = 0.0055837;
		start[23] = 0.0039803;
		start[24] = 0.00048662;
		start[25] = 0.0050074;
		start[26] = 0.0044456;
		start[27] = 0.0030801;
		start[28] = 0.0085967;
		start[29] = 0.0053969;
		start[30] = 0.0035433;
		start[31] = 0.00085792;
		start[32] = 0.0040663;
		start[33] = 0.0046435;
		start[34] = 0.0022237;
		start[35] = 0.00069526;
		start[36] = 0.0072301;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0021338;
		start[6] = 0.0029701;
		start[7] = 0.00576;
		start[8] = 0.0076147;
		start[9] = 0.0082126;
		start[10] = 0.00486;
		start[11] = 0.0046992;
		start[12] = 0.0050051;
		start[13] = 0.0075793;
		start[14] = 0.0029339;
		start[15] = 0.0081408;
		start[16] = 0.007029;
		start[17] = 5.1875e-05;
		start[18] = 0.001577;
		start[19] = 0.0065903;
		start[20] = 0.0085816;
		start[21] = 0.003935;
		start[22] = 0.0069201;
		start[23] = 0.0098108;
		start[24] = 0.0091856;
		start[25] = 0.0051541;
		start[26] = 0.0063368;
		start[27] = 0.0038017;
		start[28] = 0.00090777;
		start[29] = 0.0023153;
		start[30] = 0.00084097;
		start[31] = 0.0098784;
		start[32] = 0.0046189;
		start[33] = 0.0076591;
		start[34] = 0.0074729;
		start[35] = 0.0047003;
		start[36] = 0.0044824;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0080379;
		start[6] = 0.0059951;
		start[7] = 0.0042688;
		start[8] = 0.0031219;
		start[9] = 0.0030969;
		start[10] = 0.0071896;
		start[11] = 0.0068724;
		start[12] = 0.005533;
		start[13] = 0.0087911;
		start[14] = 0.00064639;
		start[15] = 0.008913;
		start[16] = 0.0062535;
		start[17] = 0.00060434;
		start[18] = 0.0098169;
		start[19] = 0.0089511;
		start[20] = 0.0030307;
		start[21] = 0.005432;
		start[22] = 0.0034812;
		start[23] = 0.0036516;
		start[24] = 0.0051312;
		start[25] = 0.0080168;
		start[26] = 0.0047663;
		start[27] = 0.0099054;
		start[28] = 0.0037312;
		start[29] = 0.0084706;
		start[30] = 0.00034034;
		start[31] = 0.0077662;
		start[32] = 0.0033251;
		start[33] = 0.0026027;
		start[34] = 0.0064937;
		start[35] = 0.0096625;
		start[36] = 0.0058418;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0086683;
		start[6] = 0.005219;
		start[7] = 0.004122;
		start[8] = 0.0053998;
		start[9] = 0.0028684;
		start[10] = 0.0068018;
		start[11] = 0.0040696;
		start[12] = 0.0013473;
		start[13] = 0.0058194;
		start[14] = 0.0012376;
		start[15] = 0.0094192;
		start[16] = 0.00323;
		start[17] = 0.0072101;
		start[18] = 0.0083036;
		start[19] = 0.00062593;
		start[20] = 0.0023721;
		start[21] = 0.0060678;
		start[22] = 0.008294;
		start[23] = 0.0068817;
		start[24] = 0.00047465;
		start[25] = 0.0068414;
		start[26] = 0.0089808;
		start[27] = 0.00057801;
		start[28] = 0.00097478;
		start[29] = 0.0039799;
		start[30] = 0.0089431;
		start[31] = 0.006706;
		start[32] = 0.0082444;
		start[33] = 0.0086854;
		start[34] = 0.006304;
		start[35] = 0.0057247;
		start[36] = 0.0045801;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0044241;
		start[6] = 0.00085605;
		start[7] = 0.00029185;
		start[8] = 0.008072;
		start[9] = 0.0067451;
		start[10] = 0.0027876;
		start[11] = 0.0019292;
		start[12] = 0.0039383;
		start[13] = 0.0063239;
		start[14] = 0.0022615;
		start[15] = 0.00039838;
		start[16] = 0.00089118;
		start[17] = 0.0032342;
		start[18] = 0.0060697;
		start[19] = 0.0042809;
		start[20] = 0.0089861;
		start[21] = 0.0052472;
		start[22] = 0.0083048;
		start[23] = 0.0051751;
		start[24] = 0.0039629;
		start[25] = 0.0020639;
		start[26] = 0.0072147;
		start[27] = 0.0036531;
		start[28] = 0.0055446;
		start[29] = 0.006394;
		start[30] = 0.0075374;
		start[31] = 0.0045107;
		start[32] = 0.00020777;
		start[33] = 0.0019656;
		start[34] = 0.0025958;
		start[35] = 0.0051632;
		start[36] = 0.0024866;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0063715;
		start[6] = 0.0069333;
		start[7] = 0.0076064;
		start[8] = 0.0026106;
		start[9] = 0.0019535;
		start[10] = 0.0084017;
		start[11] = 0.0021277;
		start[12] = 0.0072041;
		start[13] = 0.0059771;
		start[14] = 0.0015019;
		start[15] = 0.0057301;
		start[16] = 0.0040168;
		start[17] = 0.0033943;
		start[18] = 0.0054935;
		start[19] = 0.008559;
		start[20] = 0.0045375;
		start[21] = 0.0035462;
		start[22] = 0.0041443;
		start[23] = 0.0032171;
		start[24] = 0.0090803;
		start[25] = 0.0094982;
		start[26] = 0.0070513;
		start[27] = 0.00092712;
		start[28] = 0.0080913;
		start[29] = 0.0027091;
		start[30] = 0.0068682;
		start[31] = 0.0091336;
		start[32] = 0.00093724;
		start[33] = 0.0029696;
		start[34] = 0.0077012;
		start[35] = 0.0051897;
		start[36] = 0.006171;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0011918;
		start[6] = 0.0056055;
		start[7] = 0.0095647;
		start[8] = 0.0072743;
		start[9] = 0.0092332;
		start[10] = 0.00053315;
		start[11] = 0.0072533;
		start[12] = 0.00082218;
		start[13] = 0.0072374;
		start[14] = 0.00046013;
		start[15] = 0.0058546;
		start[16] = 0.0030119;
		start[17] = 0.0041461;
		start[18] = 0.009586;
		start[19] = 0.0019847;
		start[20] = 0.0049399;
		start[21] = 0.0008944;
		start[22] = 0.0084081;
		start[23] = 0.0064445;
		start[24] = 7.5775e-05;
		start[25] = 0.009581;
		start[26] = 0.0022399;
		start[27] = 0.0022978;
		start[28] = 0.0077808;
		start[29] = 0.009923;
		start[30] = 0.0054685;
		start[31] = 0.0010257;
		start[32] = 0.00056159;
		start[33] = 0.0022646;
		start[34] = 0.0030954;
		start[35] = 0.0081221;
		start[36] = 0.008559;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0014431;
		start[6] = 0.0021304;
		start[7] = 0.00026683;
		start[8] = 0.0033692;
		start[9] = 0.0067633;
		start[10] = 0.0095531;
		start[11] = 0.00050019;
		start[12] = 0.0071166;
		start[13] = 0.0023165;
		start[14] = 0.0016001;
		start[15] = 0.0074378;
		start[16] = 0.0097101;
		start[17] = 0.0037025;
		start[18] = 0.0065521;
		start[19] = 0.0068156;
		start[20] = 0.0012667;
		start[21] = 0.0096753;
		start[22] = 0.0024246;
		start[23] = 0.0053276;
		start[24] = 0.0083144;
		start[25] = 0.006823;
		start[26] = 0.0051714;
		start[27] = 0.0040977;
		start[28] = 0.001451;
		start[29] = 0.0024545;
		start[30] = 0.0069887;
		start[31] = 0.0041834;
		start[32] = 0.0015232;
		start[33] = 0.0081338;
		start[34] = 0.0009607;
		start[35] = 0.0087328;
		start[36] = 0.0051547;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0016371;
		start[6] = 0.0074584;
		start[7] = 0.0043024;
		start[8] = 0.0015769;
		start[9] = 0.00085008;
		start[10] = 0.0086395;
		start[11] = 0.0077714;
		start[12] = 0.0026702;
		start[13] = 0.0028465;
		start[14] = 0.0060699;
		start[15] = 0.00047323;
		start[16] = 0.0083555;
		start[17] = 0.0096881;
		start[18] = 0.007501;
		start[19] = 0.0092423;
		start[20] = 0.00107;
		start[21] = 0.0090917;
		start[22] = 0.0081958;
		start[23] = 0.0064361;
		start[24] = 0.0099884;
		start[25] = 0.0021479;
		start[26] = 0.0099074;
		start[27] = 0.00022532;
		start[28] = 0.0039522;
		start[29] = 0.0056303;
		start[30] = 0.0056388;
		start[31] = 0.0046944;
		start[32] = 0.0044356;
		start[33] = 0.0020157;
		start[34] = 0.0019618;
		start[35] = 0.0085223;
		start[36] = 0.0069114;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0037886;
		start[6] = 0.0019025;
		start[7] = 0.0025562;
		start[8] = 0.0029693;
		start[9] = 0.0089581;
		start[10] = 0.0099096;
		start[11] = 0.00083994;
		start[12] = 0.0034704;
		start[13] = 0.00088489;
		start[14] = 0.0063064;
		start[15] = 0.0025184;
		start[16] = 0.004259;
		start[17] = 0.0050912;
		start[18] = 0.0027846;
		start[19] = 0.0053828;
		start[20] = 0.0080252;
		start[21] = 0.0012227;
		start[22] = 0.0075047;
		start[23] = 0.0047991;
		start[24] = 0.0099887;
		start[25] = 0.0046701;
		start[26] = 0.0054111;
		start[27] = 0.00088269;
		start[28] = 0.009699;
		start[29] = 0.0062237;
		start[30] = 0.0044977;
		start[31] = 0.0091004;
		start[32] = 0.0090643;
		start[33] = 0.0012649;
		start[34] = 0.0025712;
		start[35] = 0.0060338;
		start[36] = 0.0018814;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0026572;
		start[6] = 0.0017381;
		start[7] = 0.0050851;
		start[8] = 0.0062658;
		start[9] = 0.0041456;
		start[10] = 0.00093889;
		start[11] = 0.0034982;
		start[12] = 0.0052913;
		start[13] = 0.0099241;
		start[14] = 0.0010561;
		start[15] = 0.0063054;
		start[16] = 0.0060403;
		start[17] = 0.007746;
		start[18] = 0.0046881;
		start[19] = 0.0014612;
		start[20] = 0.0021722;
		start[21] = 0.0087144;
		start[22] = 0.0094232;
		start[23] = 0.0010712;
		start[24] = 0.0050318;
		start[25] = 0.0030481;
		start[26] = 0.0077599;
		start[27] = 0.0072762;
		start[28] = 0.0026804;
		start[29] = 0.007685;
		start[30] = 0.0084139;
		start[31] = 0.0040273;
		start[32] = 0.0094038;
		start[33] = 0.0001557;
		start[34] = 0.0061408;
		start[35] = 0.0015876;
		start[36] = 0.0038119;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0042477;
		start[6] = 0.0068117;
		start[7] = 0.0038625;
		start[8] = 0.0025398;
		start[9] = 0.007689;
		start[10] = 0.0018214;
		start[11] = 0.0099852;
		start[12] = 0.0032124;
		start[13] = 0.0012066;
		start[14] = 0.0077714;
		start[15] = 0.0096225;
		start[16] = 0.0039239;
		start[17] = 0.0067561;
		start[18] = 0.0029686;
		start[19] = 0.0022925;
		start[20] = 0.0084068;
		start[21] = 0.0038984;
		start[22] = 0.0062238;
		start[23] = 0.0081504;
		start[24] = 0.0095702;
		start[25] = 0.0049567;
		start[26] = 0.0084377;
		start[27] = 0.0054422;
		start[28] = 0.006019;
		start[29] = 0.0044362;
		start[30] = 0.0068193;
		start[31] = 0.0067283;
		start[32] = 0.009189;
		start[33] = 0.00613;
		start[34] = 0.0023276;
		start[35] = 0.001434;
		start[36] = 0.0017316;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.00753;
		start[6] = 0.00075456;
		start[7] = 0.00073975;
		start[8] = 0.0025497;
		start[9] = 0.0015413;
		start[10] = 0.0019045;
		start[11] = 0.0064705;
		start[12] = 0.0078824;
		start[13] = 8.2414e-05;
		start[14] = 0.0098365;
		start[15] = 7.8539e-05;
		start[16] = 0.0081152;
		start[17] = 0.0010875;
		start[18] = 0.0091995;
		start[19] = 0.00029213;
		start[20] = 0.0060731;
		start[21] = 0.0072158;
		start[22] = 0.00078076;
		start[23] = 0.0072439;
		start[24] = 0.0091647;
		start[25] = 0.0069782;
		start[26] = 0.0088714;
		start[27] = 0.0060233;
		start[28] = 0.0097687;
		start[29] = 0.0080855;
		start[30] = 0.0096774;
		start[31] = 0.0044078;
		start[32] = 0.0070002;
		start[33] = 0.0060918;
		start[34] = 0.0039323;
		start[35] = 0.0013299;
		start[36] = 0.0064927;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0055788;
		start[6] = 0.0029592;
		start[7] = 0.0004913;
		start[8] = 0.007078;
		start[9] = 0.0012699;
		start[10] = 0.0089876;
		start[11] = 0.00055555;
		start[12] = 0.0026402;
		start[13] = 0.00077889;
		start[14] = 0.0093176;
		start[15] = 7.3303e-05;
		start[16] = 0.0012714;
		start[17] = 0.0099914;
		start[18] = 0.0032291;
		start[19] = 0.00021409;
		start[20] = 0.0018299;
		start[21] = 0.0022422;
		start[22] = 0.0095577;
		start[23] = 0.0013193;
		start[24] = 0.0034974;
		start[25] = 0.0092067;
		start[26] = 0.003957;
		start[27] = 0.008779;
		start[28] = 0.0059999;
		start[29] = 0.005238;
		start[30] = 0.00092441;
		start[31] = 0.0095632;
		start[32] = 0.0053769;
		start[33] = 0.0081103;
		start[34] = 0.0054209;
		start[35] = 0.0069966;
		start[36] = 0.0098627;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.00090131;
		start[6] = 0.0094938;
		start[7] = 0.00048659;
		start[8] = 0.0094965;
		start[9] = 0.0019311;
		start[10] = 0.0020163;
		start[11] = 0.0033818;
		start[12] = 0.00023182;
		start[13] = 0.0036442;
		start[14] = 0.0048846;
		start[15] = 0.0017657;
		start[16] = 0.0083093;
		start[17] = 0.00024942;
		start[18] = 0.0066733;
		start[19] = 0.0072774;
		start[20] = 0.00057534;
		start[21] = 0.0060715;
		start[22] = 8.8355e-05;
		start[23] = 0.0065108;
		start[24] = 0.0044143;
		start[25] = 0.0061048;
		start[26] = 0.0015296;
		start[27] = 0.0014585;
		start[28] = 0.0097587;
		start[29] = 0.0072981;
		start[30] = 0.00020607;
		start[31] = 0.0032221;
		start[32] = 0.0036983;
		start[33] = 0.0020303;
		start[34] = 0.0049167;
		start[35] = 0.0015089;
		start[36] = 0.0084443;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0052984;
		start[6] = 0.0059715;
		start[7] = 0.0031358;
		start[8] = 0.0034264;
		start[9] = 0.0045908;
		start[10] = 0.00066169;
		start[11] = 0.0029478;
		start[12] = 0.0068927;
		start[13] = 0.0014616;
		start[14] = 0.009786;
		start[15] = 0.0099019;
		start[16] = 0.0014638;
		start[17] = 0.0042197;
		start[18] = 0.0069134;
		start[19] = 0.0060228;
		start[20] = 0.0090566;
		start[21] = 0.0016427;
		start[22] = 0.0034583;
		start[23] = 0.0047493;
		start[24] = 0.0034874;
		start[25] = 0.0057706;
		start[26] = 0.0056085;
		start[27] = 0.0096141;
		start[28] = 0.0048425;
		start[29] = 0.0047472;
		start[30] = 0.0055888;
		start[31] = 0.0069939;
		start[32] = 0.0090007;
		start[33] = 0.0028225;
		start[34] = 0.0070297;
		start[35] = 0.0062267;
		start[36] = 0.0028673;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0090854;
		start[6] = 0.0078493;
		start[7] = 0.0093651;
		start[8] = 0.0089175;
		start[9] = 0.0033592;
		start[10] = 0.006191;
		start[11] = 0.0046553;
		start[12] = 0.0012134;
		start[13] = 0.0093618;
		start[14] = 0.0052867;
		start[15] = 0.0091212;
		start[16] = 0.0023837;
		start[17] = 0.0032302;
		start[18] = 0.0064209;
		start[19] = 0.0039003;
		start[20] = 0.00052766;
		start[21] = 0.0078891;
		start[22] = 0.0066778;
		start[23] = 0.0061717;
		start[24] = 0.00081431;
		start[25] = 0.0026684;
		start[26] = 0.0060149;
		start[27] = 0.009541;
		start[28] = 0.0023493;
		start[29] = 0.0017779;
		start[30] = 0.0039876;
		start[31] = 0.0042541;
		start[32] = 0.0060145;
		start[33] = 0.00091545;
		start[34] = 0.0095233;
		start[35] = 0.0018756;
		start[36] = 0.0097281;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0036072;
		start[6] = 0.0097725;
		start[7] = 0.0014085;
		start[8] = 0.0025286;
		start[9] = 0.0018347;
		start[10] = 0.0041153;
		start[11] = 0.0048847;
		start[12] = 0.0079363;
		start[13] = 0.0057515;
		start[14] = 0.0047571;
		start[15] = 0.0053806;
		start[16] = 0.00040174;
		start[17] = 0.0046155;
		start[18] = 0.0071746;
		start[19] = 0.0059737;
		start[20] = 0.0097104;
		start[21] = 0.0066256;
		start[22] = 0.0015164;
		start[23] = 0.0012969;
		start[24] = 0.0091023;
		start[25] = 0.0091982;
		start[26] = 0.0013869;
		start[27] = 0.00049324;
		start[28] = 0.0025731;
		start[29] = 0.009412;
		start[30] = 0.0039778;
		start[31] = 0.0038396;
		start[32] = 0.0076309;
		start[33] = 0.0015931;
		start[34] = 0.0084838;
		start[35] = 0.0033497;
		start[36] = 0.0063337;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.00031231;
		start[6] = 0.0062998;
		start[7] = 0.0077826;
		start[8] = 0.0044011;
		start[9] = 0.0010866;
		start[10] = 0.00083437;
		start[11] = 0.0062482;
		start[12] = 0.0031411;
		start[13] = 0.0084946;
		start[14] = 0.00080589;
		start[15] = 0.0075961;
		start[16] = 0.0069357;
		start[17] = 0.0070386;
		start[18] = 0.001621;
		start[19] = 0.0032769;
		start[20] = 0.0075211;
		start[21] = 0.0011566;
		start[22] = 0.008675;
		start[23] = 0.0055257;
		start[24] = 0.008715;
		start[25] = 0.0097511;
		start[26] = 0.0061863;
		start[27] = 0.0027538;
		start[28] = 0.00012245;
		start[29] = 0.0095538;
		start[30] = 0.0069502;
		start[31] = 0.0015139;
		start[32] = 0.0063449;
		start[33] = 0.0020436;
		start[34] = 0.0093606;
		start[35] = 0.0022744;
		start[36] = 0.001511;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0098646;
		start[6] = 0.0096215;
		start[7] = 0.0086044;
		start[8] = 0.0043224;
		start[9] = 0.0047935;
		start[10] = 0.0081483;
		start[11] = 0.0042026;
		start[12] = 6.2016e-05;
		start[13] = 0.00084647;
		start[14] = 0.0045323;
		start[15] = 0.0096269;
		start[16] = 0.0029377;
		start[17] = 0.0033296;
		start[18] = 0.0072063;
		start[19] = 0.0014376;
		start[20] = 0.0075631;
		start[21] = 0.008251;
		start[22] = 0.0067324;
		start[23] = 0.0021167;
		start[24] = 0.0099693;
		start[25] = 0.0087192;
		start[26] = 0.008066;
		start[27] = 0.0049758;
		start[28] = 0.0052867;
		start[29] = 0.0022591;
		start[30] = 0.0052281;
		start[31] = 0.0022576;
		start[32] = 0.0032264;
		start[33] = 0.002159;
		start[34] = 0.0047814;
		start[35] = 0.004984;
		start[36] = 0.0083352;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0034603;
		start[6] = 0.00043692;
		start[7] = 0.008043;
		start[8] = 0.00681;
		start[9] = 0.0082389;
		start[10] = 0.0013254;
		start[11] = 0.00055362;
		start[12] = 0.0083952;
		start[13] = 0.007892;
		start[14] = 0.0098912;
		start[15] = 0.00028383;
		start[16] = 0.0075597;
		start[17] = 0.0016501;
		start[18] = 0.0030699;
		start[19] = 0.0036394;
		start[20] = 0.0042626;
		start[21] = 0.0091193;
		start[22] = 0.0076302;
		start[23] = 0.0066157;
		start[24] = 0.0084534;
		start[25] = 0.00062101;
		start[26] = 6.6998e-05;
		start[27] = 0.00067299;
		start[28] = 0.00079722;
		start[29] = 0.0089279;
		start[30] = 0.0029939;
		start[31] = 0.0011692;
		start[32] = 0.0014921;
		start[33] = 0.007597;
		start[34] = 0.0040708;
		start[35] = 0.0062779;
		start[36] = 0.00076095;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0070817;
		start[6] = 0.0089709;
		start[7] = 0.0025327;
		start[8] = 0.0090036;
		start[9] = 0.0016124;
		start[10] = 0.0089833;
		start[11] = 0.0068973;
		start[12] = 0.0096866;
		start[13] = 0.0040823;
		start[14] = 0.0092315;
		start[15] = 0.0032763;
		start[16] = 0.00053934;
		start[17] = 0.0063303;
		start[18] = 0.0019626;
		start[19] = 0.00025951;
		start[20] = 0.0058064;
		start[21] = 0.0050993;
		start[22] = 0.0063641;
		start[23] = 0.0081561;
		start[24] = 0.0079172;
		start[25] = 0.0041755;
		start[26] = 0.0027914;
		start[27] = 0.0065765;
		start[28] = 0.0052213;
		start[29] = 0.00040861;
		start[30] = 0.0067724;
		start[31] = 0.0027285;
		start[32] = 0.0036503;
		start[33] = 0.0080732;
		start[34] = 0.0095036;
		start[35] = 0.003875;
		start[36] = 0.0097576;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.00099489;
		start[6] = 0.0092264;
		start[7] = 3.7844e-05;
		start[8] = 0.0081843;
		start[9] = 0.0087372;
		start[10] = 0.0077863;
		start[11] = 0.0090576;
		start[12] = 0.0035249;
		start[13] = 0.0086923;
		start[14] = 0.0061162;
		start[15] = 0.0099955;
		start[16] = 0.002775;
		start[17] = 0.002944;
		start[18] = 0.0086689;
		start[19] = 0.0033202;
		start[20] = 0.0020927;
		start[21] = 0.0046238;
		start[22] = 0.0059131;
		start[23] = 0.00010521;
		start[24] = 0.0009979;
		start[25] = 0.0072208;
		start[26] = 0.00050392;
		start[27] = 0.0032585;
		start[28] = 0.0029603;
		start[29] = 0.0092133;
		start[30] = 0.0070126;
		start[31] = 0.0010058;
		start[32] = 0.0025701;
		start[33] = 0.0021586;
		start[34] = 0.0032619;
		start[35] = 0.0035218;
		start[36] = 0.0084491;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0010859;
		start[6] = 0.0013918;
		start[7] = 0.0020593;
		start[8] = 0.00045981;
		start[9] = 0.0057316;
		start[10] = 0.0084889;
		start[11] = 0.0031797;
		start[12] = 0.00018219;
		start[13] = 0.0042006;
		start[14] = 0.0068705;
		start[15] = 0.0014698;
		start[16] = 0.0024418;
		start[17] = 0.0020135;
		start[18] = 0.0026547;
		start[19] = 0.0063243;
		start[20] = 0.0030069;
		start[21] = 0.0038089;
		start[22] = 0.0060024;
		start[23] = 0.0022674;
		start[24] = 0.0055323;
		start[25] = 0.0032397;
		start[26] = 0.0071579;
		start[27] = 0.0017472;
		start[28] = 0.0090214;
		start[29] = 0.00060271;
		start[30] = 0.0080082;
		start[31] = 9.7531e-05;
		start[32] = 0.00071635;
		start[33] = 0.0035375;
		start[34] = 0.0040497;
		start[35] = 0.0043717;
		start[36] = 0.001873;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.00053631;
		start[6] = 0.0092998;
		start[7] = 0.0060235;
		start[8] = 0.0095067;
		start[9] = 0.0054233;
		start[10] = 0.0096703;
		start[11] = 0.0016185;
		start[12] = 4.0816e-06;
		start[13] = 0.0086216;
		start[14] = 0.0031956;
		start[15] = 0.0019516;
		start[16] = 0.0099936;
		start[17] = 0.0021041;
		start[18] = 0.0027258;
		start[19] = 0.0094842;
		start[20] = 0.00015492;
		start[21] = 0.0088716;
		start[22] = 0.0065409;
		start[23] = 0.0095464;
		start[24] = 0.0038276;
		start[25] = 0.0060017;
		start[26] = 0.0025295;
		start[27] = 0.0060051;
		start[28] = 0.0020313;
		start[29] = 0.0068368;
		start[30] = 0.0015172;
		start[31] = 0.0033031;
		start[32] = 0.007739;
		start[33] = 0.0034253;
		start[34] = 0.00063227;
		start[35] = 0.007293;
		start[36] = 0.0059788;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.00042816;
		start[6] = 0.0079276;
		start[7] = 0.0079749;
		start[8] = 0.0092037;
		start[9] = 0.0056224;
		start[10] = 0.0013655;
		start[11] = 0.0033618;
		start[12] = 0.0057445;
		start[13] = 0.0021651;
		start[14] = 0.0075194;
		start[15] = 0.0068288;
		start[16] = 0.0052111;
		start[17] = 0.0063882;
		start[18] = 0.0094724;
		start[19] = 0.00025881;
		start[20] = 0.0035896;
		start[21] = 0.0045961;
		start[22] = 0.0025855;
		start[23] = 0.0034381;
		start[24] = 0.0089973;
		start[25] = 0.0085779;
		start[26] = 0.0057952;
		start[27] = 0.00033174;
		start[28] = 0.0027268;
		start[29] = 0.0075397;
		start[30] = 0.0077881;
		start[31] = 0.0068403;
		start[32] = 0.0084652;
		start[33] = 0.0064589;
		start[34] = 0.0075175;
		start[35] = 0.0068622;
		start[36] = 0.00019813;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0023701;
		start[6] = 0.0032857;
		start[7] = 0.0035104;
		start[8] = 0.00025446;
		start[9] = 0.0038844;
		start[10] = 0.0019063;
		start[11] = 0.0042247;
		start[12] = 0.0030964;
		start[13] = 0.0063519;
		start[14] = 0.0017131;
		start[15] = 0.001051;
		start[16] = 0.0030328;
		start[17] = 0.0031611;
		start[18] = 0.00035754;
		start[19] = 0.0026077;
		start[20] = 0.0033311;
		start[21] = 0.0096716;
		start[22] = 0.0042957;
		start[23] = 0.0072869;
		start[24] = 0.0019152;
		start[25] = 0.0023602;
		start[26] = 0.0058651;
		start[27] = 0.0063901;
		start[28] = 0.0042793;
		start[29] = 0.0064148;
		start[30] = 0.0088887;
		start[31] = 0.0065491;
		start[32] = 0.0038041;
		start[33] = 0.0030403;
		start[34] = 0.0029667;
		start[35] = 0.0088643;
		start[36] = 0.0015036;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0090495;
		start[6] = 0.0051569;
		start[7] = 0.0033692;
		start[8] = 0.00049459;
		start[9] = 0.0083967;
		start[10] = 0.0091936;
		start[11] = 0.0094077;
		start[12] = 0.0030236;
		start[13] = 0.0036295;
		start[14] = 0.0030788;
		start[15] = 0.0039576;
		start[16] = 0.0015422;
		start[17] = 0.0016345;
		start[18] = 0.0085295;
		start[19] = 0.0076975;
		start[20] = 0.0049139;
		start[21] = 0.0099916;
		start[22] = 0.0040759;
		start[23] = 0.0073495;
		start[24] = 0.0089653;
		start[25] = 0.0049616;
		start[26] = 0.0057579;
		start[27] = 0.0073635;
		start[28] = 0.0083819;
		start[29] = 0.0064685;
		start[30] = 0.0044859;
		start[31] = 0.0029289;
		start[32] = 0.00089768;
		start[33] = 0.0071948;
		start[34] = 0.0088722;
		start[35] = 0.0064199;
		start[36] = 0.0052544;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0018813;
		start[6] = 0.0025537;
		start[7] = 0.0054969;
		start[8] = 0.0032701;
		start[9] = 0.0016181;
		start[10] = 0.0091179;
		start[11] = 0.008365;
		start[12] = 0.0067301;
		start[13] = 0.0046021;
		start[14] = 0.00053569;
		start[15] = 0.00046812;
		start[16] = 0.0025934;
		start[17] = 0.0015279;
		start[18] = 0.0064587;
		start[19] = 0.0019103;
		start[20] = 0.0077919;
		start[21] = 0.002254;
		start[22] = 0.0045859;
		start[23] = 0.0026044;
		start[24] = 0.0076602;
		start[25] = 0.0083491;
		start[26] = 0.0030208;
		start[27] = 0.0040561;
		start[28] = 0.0093216;
		start[29] = 0.0020137;
		start[30] = 0.0076857;
		start[31] = 0.0062928;
		start[32] = 0.0024794;
		start[33] = 0.0038777;
		start[34] = 0.0025617;
		start[35] = 0.0089666;
		start[36] = 0.0039785;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0087088;
		start[6] = 0.0010584;
		start[7] = 0.0066108;
		start[8] = 0.0048447;
		start[9] = 0.0010724;
		start[10] = 0.0066086;
		start[11] = 0.00068623;
		start[12] = 0.007186;
		start[13] = 0.0054924;
		start[14] = 0.0098425;
		start[15] = 0.00059639;
		start[16] = 0.0046696;
		start[17] = 0.00043424;
		start[18] = 0.009667;
		start[19] = 0.0043413;
		start[20] = 0.0010643;
		start[21] = 0.0058065;
		start[22] = 0.00056258;
		start[23] = 0.0089072;
		start[24] = 0.0027466;
		start[25] = 0.0047294;
		start[26] = 0.007163;
		start[27] = 0.0054941;
		start[28] = 0.0020306;
		start[29] = 0.0049183;
		start[30] = 0.0066422;
		start[31] = 0.0011688;
		start[32] = 0.0022061;
		start[33] = 0.0043881;
		start[34] = 0.0069068;
		start[35] = 0.0045346;
		start[36] = 0.0093252;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0077293;
		start[6] = 0.00010208;
		start[7] = 0.0092649;
		start[8] = 0.0010304;
		start[9] = 0.0040915;
		start[10] = 0.00067406;
		start[11] = 0.0072659;
		start[12] = 0.0094462;
		start[13] = 0.0067122;
		start[14] = 0.0046286;
		start[15] = 0.0015463;
		start[16] = 0.00077512;
		start[17] = 0.0045743;
		start[18] = 0.003999;
		start[19] = 0.008942;
		start[20] = 0.005407;
		start[21] = 0.0025707;
		start[22] = 0.0049972;
		start[23] = 0.0022949;
		start[24] = 0.001669;
		start[25] = 0.0031751;
		start[26] = 0.0060347;
		start[27] = 0.0047179;
		start[28] = 0.0077589;
		start[29] = 0.0082113;
		start[30] = 0.0028115;
		start[31] = 0.0065749;
		start[32] = 0.0012204;
		start[33] = 0.0075727;
		start[34] = 0.00022025;
		start[35] = 0.0046185;
		start[36] = 0.002485;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0072215;
		start[6] = 0.0085129;
		start[7] = 0.00063311;
		start[8] = 0.0066771;
		start[9] = 0.0067621;
		start[10] = 0.00037106;
		start[11] = 0.0045635;
		start[12] = 0.0012132;
		start[13] = 0.0044967;
		start[14] = 0.0090253;
		start[15] = 0.00016662;
		start[16] = 0.009116;
		start[17] = 0.0030863;
		start[18] = 0.0085096;
		start[19] = 0.00067329;
		start[20] = 0.002443;
		start[21] = 0.0034614;
		start[22] = 0.0044586;
		start[23] = 0.0012633;
		start[24] = 0.0052317;
		start[25] = 0.0084423;
		start[26] = 0.0067142;
		start[27] = 0.0092829;
		start[28] = 0.0044663;
		start[29] = 0.0018843;
		start[30] = 0.0040894;
		start[31] = 0.0010971;
		start[32] = 0.0013712;
		start[33] = 0.0011319;
		start[34] = 0.0079616;
		start[35] = 0.0092199;
		start[36] = 0.0043997;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0035343;
		start[6] = 0.0075145;
		start[7] = 0.0048142;
		start[8] = 0.0056686;
		start[9] = 0.0018674;
		start[10] = 0.0040604;
		start[11] = 0.0037125;
		start[12] = 0.00098761;
		start[13] = 0.0092909;
		start[14] = 0.0075282;
		start[15] = 0.00090184;
		start[16] = 0.0034514;
		start[17] = 0.0026427;
		start[18] = 0.004797;
		start[19] = 0.0049369;
		start[20] = 0.0092076;
		start[21] = 0.0016469;
		start[22] = 0.0083892;
		start[23] = 0.0076671;
		start[24] = 0.0053831;
		start[25] = 0.0014409;
		start[26] = 0.0060129;
		start[27] = 0.0090076;
		start[28] = 0.0023221;
		start[29] = 0.0060294;
		start[30] = 0.0069261;
		start[31] = 0.00032293;
		start[32] = 0.0032316;
		start[33] = 0.0059588;
		start[34] = 0.0055652;
		start[35] = 0.002596;
		start[36] = 0.0024414;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.003312;
		start[6] = 0.0088346;
		start[7] = 0.0046079;
		start[8] = 0.00048124;
		start[9] = 0.0062647;
		start[10] = 0.0093575;
		start[11] = 0.0063252;
		start[12] = 0.0071382;
		start[13] = 0.00035494;
		start[14] = 0.0045813;
		start[15] = 0.0028918;
		start[16] = 0.0096526;
		start[17] = 0.0041832;
		start[18] = 0.00046244;
		start[19] = 0.0061954;
		start[20] = 0.0046311;
		start[21] = 0.0022643;
		start[22] = 0.0095461;
		start[23] = 0.006051;
		start[24] = 0.0082542;
		start[25] = 0.0015035;
		start[26] = 0.0059448;
		start[27] = 0.0075607;
		start[28] = 0.0012325;
		start[29] = 0.0098802;
		start[30] = 0.00068551;
		start[31] = 0.0019899;
		start[32] = 0.0022865;
		start[33] = 0.0031726;
		start[34] = 0.005092;
		start[35] = 0.0038584;
		start[36] = 0.0095275;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0033212;
		start[6] = 0.0066466;
		start[7] = 0.0080612;
		start[8] = 0.00077692;
		start[9] = 0.0007893;
		start[10] = 0.00057771;
		start[11] = 0.0037101;
		start[12] = 0.0062797;
		start[13] = 0.0083323;
		start[14] = 0.0038358;
		start[15] = 0.005107;
		start[16] = 0.0027458;
		start[17] = 0.0039386;
		start[18] = 0.0079424;
		start[19] = 0.0010027;
		start[20] = 0.0015482;
		start[21] = 0.0005304;
		start[22] = 0.0044164;
		start[23] = 0.009854;
		start[24] = 0.0009859;
		start[25] = 0.0081739;
		start[26] = 0.006284;
		start[27] = 0.0048992;
		start[28] = 0.0046292;
		start[29] = 0.0053864;
		start[30] = 0.0036422;
		start[31] = 0.0068301;
		start[32] = 0.0093434;
		start[33] = 0.0049056;
		start[34] = 0.0057822;
		start[35] = 0.0029388;
		start[36] = 0.0064995;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0089418;
		start[6] = 0.0079037;
		start[7] = 0.00011377;
		start[8] = 0.0063498;
		start[9] = 0.00061386;
		start[10] = 0.0041162;
		start[11] = 0.0093434;
		start[12] = 0.00035486;
		start[13] = 0.0077928;
		start[14] = 0.0069568;
		start[15] = 2.2421e-06;
		start[16] = 0.0048745;
		start[17] = 0.0024526;
		start[18] = 0.0060036;
		start[19] = 0.0063685;
		start[20] = 0.0068868;
		start[21] = 0.0036387;
		start[22] = 0.0064624;
		start[23] = 0.0058449;
		start[24] = 7.3588e-05;
		start[25] = 0.0010437;
		start[26] = 0.0084426;
		start[27] = 0.0015072;
		start[28] = 0.0011366;
		start[29] = 0.0051962;
		start[30] = 0.004165;
		start[31] = 0.0068809;
		start[32] = 0.0071166;
		start[33] = 0.0096336;
		start[34] = 0.0040699;
		start[35] = 0.0052985;
		start[36] = 0.0070769;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0088782;
		start[6] = 0.0099793;
		start[7] = 0.0042682;
		start[8] = 0.0033698;
		start[9] = 0.0017909;
		start[10] = 0.003778;
		start[11] = 0.007145;
		start[12] = 0.0062131;
		start[13] = 0.002998;
		start[14] = 0.0068401;
		start[15] = 0.0025973;
		start[16] = 0.0004375;
		start[17] = 0.0022071;
		start[18] = 0.0061365;
		start[19] = 0.0025873;
		start[20] = 0.0033796;
		start[21] = 0.0046214;
		start[22] = 0.0031983;
		start[23] = 0.0036748;
		start[24] = 0.004151;
		start[25] = 0.009624;
		start[26] = 0.0059364;
		start[27] = 0.0044825;
		start[28] = 0.0027344;
		start[29] = 0.0075644;
		start[30] = 0.0040029;
		start[31] = 0.0036284;
		start[32] = 0.0041484;
		start[33] = 0.0054197;
		start[34] = 0.0073439;
		start[35] = 0.0097907;
		start[36] = 0.0074063;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0097682;
		start[6] = 0.0014179;
		start[7] = 0.0021564;
		start[8] = 0.0076878;
		start[9] = 0.0093862;
		start[10] = 0.0083095;
		start[11] = 0.0023557;
		start[12] = 0.0055049;
		start[13] = 0.0062074;
		start[14] = 0.0096161;
		start[15] = 0.0095593;
		start[16] = 0.002026;
		start[17] = 0.0029254;
		start[18] = 0.0057356;
		start[19] = 0.0050459;
		start[20] = 0.0004065;
		start[21] = 0.0022113;
		start[22] = 0.0029044;
		start[23] = 0.00038981;
		start[24] = 0.0063974;
		start[25] = 0.0035514;
		start[26] = 0.0055947;
		start[27] = 0.0021149;
		start[28] = 0.0096156;
		start[29] = 0.0056501;
		start[30] = 0.0063957;
		start[31] = 7.1569e-05;
		start[32] = 0.0049686;
		start[33] = 0.0031851;
		start[34] = 0.0067135;
		start[35] = 0.0012391;
		start[36] = 0.00013469;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0053199;
		start[6] = 0.0043034;
		start[7] = 0.0068194;
		start[8] = 0.0098071;
		start[9] = 0.0033722;
		start[10] = 0.0038312;
		start[11] = 0.0074665;
		start[12] = 0.0071882;
		start[13] = 0.0098411;
		start[14] = 0.004363;
		start[15] = 0.0055447;
		start[16] = 0.0090235;
		start[17] = 0.0011644;
		start[18] = 0.00077087;
		start[19] = 0.0084263;
		start[20] = 0.0032296;
		start[21] = 0.0046612;
		start[22] = 0.0071269;
		start[23] = 0.0045279;
		start[24] = 0.0031284;
		start[25] = 0.00021725;
		start[26] = 5.5844e-05;
		start[27] = 0.00066954;
		start[28] = 0.00045807;
		start[29] = 0.0050122;
		start[30] = 0.0024403;
		start[31] = 0.0036247;
		start[32] = 0.0086038;
		start[33] = 0.0092449;
		start[34] = 0.0094315;
		start[35] = 0.00011531;
		start[36] = 0.0092965;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0027627;
		start[6] = 0.00094063;
		start[7] = 0.0081675;
		start[8] = 0.0063737;
		start[9] = 0.0057125;
		start[10] = 7.86e-06;
		start[11] = 0.0029857;
		start[12] = 0.0087956;
		start[13] = 0.0043942;
		start[14] = 0.0022929;
		start[15] = 0.0073157;
		start[16] = 0.0016421;
		start[17] = 0.0056565;
		start[18] = 0.0065537;
		start[19] = 0.00040961;
		start[20] = 0.0016585;
		start[21] = 0.0014743;
		start[22] = 0.0071337;
		start[23] = 0.0086199;
		start[24] = 0.0047866;
		start[25] = 0.00488;
		start[26] = 0.005168;
		start[27] = 0.0035501;
		start[28] = 0.0019682;
		start[29] = 0.0032034;
		start[30] = 0.0078471;
		start[31] = 0.0006129;
		start[32] = 0.0060078;
		start[33] = 0.0075494;
		start[34] = 0.0077045;
		start[35] = 0.0023771;
		start[36] = 0.0069493;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0032766;
		start[6] = 0.0092708;
		start[7] = 0.0050735;
		start[8] = 0.0024588;
		start[9] = 0.0010202;
		start[10] = 0.0077671;
		start[11] = 0.0089714;
		start[12] = 0.005213;
		start[13] = 0.0044361;
		start[14] = 0.0069845;
		start[15] = 0.0095804;
		start[16] = 0.0061892;
		start[17] = 0.0004073;
		start[18] = 0.0077931;
		start[19] = 0.0059737;
		start[20] = 0.0046884;
		start[21] = 0.0023684;
		start[22] = 0.004779;
		start[23] = 0.0075272;
		start[24] = 0.0034571;
		start[25] = 0.0090517;
		start[26] = 0.0065353;
		start[27] = 0.0052617;
		start[28] = 0.0058255;
		start[29] = 0.0059422;
		start[30] = 0.0064067;
		start[31] = 0.0012923;
		start[32] = 0.002816;
		start[33] = 0.0085072;
		start[34] = 0.0018838;
		start[35] = 0.0031673;
		start[36] = 0.0077627;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0053081;
		start[6] = 0.0035872;
		start[7] = 0.0090491;
		start[8] = 0.00063797;
		start[9] = 0.0077197;
		start[10] = 0.0024713;
		start[11] = 0.0086034;
		start[12] = 0.003364;
		start[13] = 0.0091875;
		start[14] = 0.0037057;
		start[15] = 0.0092039;
		start[16] = 0.0037008;
		start[17] = 0.0020963;
		start[18] = 0.0060184;
		start[19] = 0.0022522;
		start[20] = 0.0031297;
		start[21] = 0.0024078;
		start[22] = 0.0014579;
		start[23] = 0.0037564;
		start[24] = 0.0062766;
		start[25] = 0.0077248;
		start[26] = 0.0048074;
		start[27] = 0.00052137;
		start[28] = 0.0088205;
		start[29] = 0.0037313;
		start[30] = 0.0042952;
		start[31] = 0.0020654;
		start[32] = 0.0070975;
		start[33] = 0.0053502;
		start[34] = 0.0077688;
		start[35] = 0.0014451;
		start[36] = 0.0026559;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0051723;
		start[6] = 0.0039231;
		start[7] = 0.0013088;
		start[8] = 0.0010911;
		start[9] = 0.0011804;
		start[10] = 0.0094688;
		start[11] = 0.0032731;
		start[12] = 0.00037608;
		start[13] = 0.0039819;
		start[14] = 0.0098432;
		start[15] = 0.0024638;
		start[16] = 4.9724e-05;
		start[17] = 0.0067004;
		start[18] = 0.0026766;
		start[19] = 0.0056588;
		start[20] = 0.0075909;
		start[21] = 0.0016866;
		start[22] = 0.0094424;
		start[23] = 0.0043496;
		start[24] = 0.00052289;
		start[25] = 0.0040266;
		start[26] = 0.0086305;
		start[27] = 0.00050192;
		start[28] = 0.00049031;
		start[29] = 0.0030858;
		start[30] = 0.0044199;
		start[31] = 0.0041857;
		start[32] = 0.0045499;
		start[33] = 0.001911;
		start[34] = 0.00041239;
		start[35] = 0.0062846;
		start[36] = 0.0079634;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0055984;
		start[6] = 0.0070587;
		start[7] = 0.0022837;
		start[8] = 0.00073966;
		start[9] = 0.003984;
		start[10] = 0.0031562;
		start[11] = 0.0025222;
		start[12] = 0.001315;
		start[13] = 0.00551;
		start[14] = 0.0031652;
		start[15] = 0.0030389;
		start[16] = 0.0065868;
		start[17] = 0.0040844;
		start[18] = 0.0049018;
		start[19] = 0.0010211;
		start[20] = 0.0009496;
		start[21] = 0.002424;
		start[22] = 0.0070699;
		start[23] = 0.008083;
		start[24] = 0.00727;
		start[25] = 0.00042198;
		start[26] = 0.0010663;
		start[27] = 0.0099974;
		start[28] = 0.0083166;
		start[29] = 0.0085803;
		start[30] = 0.0023637;
		start[31] = 0.0099284;
		start[32] = 0.0038599;
		start[33] = 0.0065428;
		start[34] = 0.0086111;
		start[35] = 0.0041207;
		start[36] = 0.005633;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0051172;
		start[6] = 0.0026273;
		start[7] = 0.0082646;
		start[8] = 0.0044937;
		start[9] = 0.0038839;
		start[10] = 0.0078821;
		start[11] = 0.0035842;
		start[12] = 0.00065412;
		start[13] = 0.0024155;
		start[14] = 0.0055962;
		start[15] = 0.00098697;
		start[16] = 0.0056323;
		start[17] = 0.0013343;
		start[18] = 0.0074884;
		start[19] = 0.0061536;
		start[20] = 0.0074287;
		start[21] = 0.0016528;
		start[22] = 0.0018505;
		start[23] = 0.0093348;
		start[24] = 0.0026669;
		start[25] = 0.0087084;
		start[26] = 0.0062044;
		start[27] = 0.00070917;
		start[28] = 0.0084294;
		start[29] = 0.0073135;
		start[30] = 0.0057567;
		start[31] = 0.00012257;
		start[32] = 0.0080191;
		start[33] = 0.0095802;
		start[34] = 0.0077682;
		start[35] = 0.0033516;
		start[36] = 0.0073877;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.009921;
		start[6] = 0.0019489;
		start[7] = 0.0018632;
		start[8] = 0.0045069;
		start[9] = 0.00029169;
		start[10] = 0.0032286;
		start[11] = 0.0032698;
		start[12] = 0.0057154;
		start[13] = 0.0040773;
		start[14] = 0.0066696;
		start[15] = 0.0042883;
		start[16] = 0.0094443;
		start[17] = 0.0048679;
		start[18] = 0.0074365;
		start[19] = 0.0016098;
		start[20] = 0.0029976;
		start[21] = 0.0073021;
		start[22] = 0.0074813;
		start[23] = 0.0096307;
		start[24] = 0.00061848;
		start[25] = 0.0083107;
		start[26] = 0.0060942;
		start[27] = 0.003746;
		start[28] = 0.0082805;
		start[29] = 0.0028381;
		start[30] = 0.00092779;
		start[31] = 0.0058757;
		start[32] = 0.0075056;
		start[33] = 0.0024936;
		start[34] = 0.0039255;
		start[35] = 0.0056729;
		start[36] = 0.0097311;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0092711;
		start[6] = 0.0079667;
		start[7] = 0.0020299;
		start[8] = 0.0054678;
		start[9] = 0.002274;
		start[10] = 0.0068358;
		start[11] = 0.0086233;
		start[12] = 0.0050394;
		start[13] = 0.0047983;
		start[14] = 0.0010542;
		start[15] = 0.0083442;
		start[16] = 0.0067692;
		start[17] = 0.00075345;
		start[18] = 0.0020952;
		start[19] = 0.0072983;
		start[20] = 0.0017145;
		start[21] = 0.0029979;
		start[22] = 0.0087897;
		start[23] = 0.0078554;
		start[24] = 0.0049141;
		start[25] = 0.00072386;
		start[26] = 0.0047455;
		start[27] = 0.006119;
		start[28] = 0.004932;
		start[29] = 0.0082624;
		start[30] = 0.00049302;
		start[31] = 0.0030274;
		start[32] = 8.1709e-05;
		start[33] = 0.0082753;
		start[34] = 0.0091984;
		start[35] = 0.0094698;
		start[36] = 0.003149;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0078555;
		start[6] = 0.0064251;
		start[7] = 0.0064968;
		start[8] = 0.0013557;
		start[9] = 0.0090202;
		start[10] = 0.0082018;
		start[11] = 0.0023808;
		start[12] = 0.001464;
		start[13] = 0.007978;
		start[14] = 0.0090502;
		start[15] = 0.0067397;
		start[16] = 0.0034849;
		start[17] = 0.0087064;
		start[18] = 0.0057013;
		start[19] = 0.002155;
		start[20] = 0.0024741;
		start[21] = 0.00038956;
		start[22] = 0.003664;
		start[23] = 0.0065788;
		start[24] = 0.0073277;
		start[25] = 0.00080876;
		start[26] = 0.0016708;
		start[27] = 0.0085932;
		start[28] = 0.0089952;
		start[29] = 0.0089259;
		start[30] = 0.002709;
		start[31] = 0.006131;
		start[32] = 0.0081583;
		start[33] = 0.0075094;
		start[34] = 0.0021684;
		start[35] = 0.0048124;
		start[36] = 0.0093337;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0060772;
		start[6] = 0.0035517;
		start[7] = 0.0097032;
		start[8] = 0.0030154;
		start[9] = 0.0022651;
		start[10] = 0.0046771;
		start[11] = 0.0069925;
		start[12] = 0.00089868;
		start[13] = 0.00031368;
		start[14] = 0.0051902;
		start[15] = 0.0088205;
		start[16] = 0.0010548;
		start[17] = 0.0076035;
		start[18] = 0.0084594;
		start[19] = 0.00089755;
		start[20] = 0.0032086;
		start[21] = 0.0016725;
		start[22] = 0.0084744;
		start[23] = 0.0048467;
		start[24] = 0.0070007;
		start[25] = 0.0050596;
		start[26] = 0.0029379;
		start[27] = 0.0062833;
		start[28] = 0.0063545;
		start[29] = 0.0076846;
		start[30] = 0.0063973;
		start[31] = 0.0069524;
		start[32] = 0.0016312;
		start[33] = 0.0096378;
		start[34] = 0.0066178;
		start[35] = 0.0023552;
		start[36] = 0.0021352;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0097979;
		start[6] = 0.0073225;
		start[7] = 0.0047176;
		start[8] = 0.0065316;
		start[9] = 0.0022337;
		start[10] = 0.0095916;
		start[11] = 0.0025686;
		start[12] = 0.0099423;
		start[13] = 0.0084326;
		start[14] = 0.0058466;
		start[15] = 0.007761;
		start[16] = 0.0027955;
		start[17] = 0.0024777;
		start[18] = 0.0007603;
		start[19] = 0.0063224;
		start[20] = 0.0065559;
		start[21] = 0.0028213;
		start[22] = 0.0039572;
		start[23] = 0.0060657;
		start[24] = 0.0008842;
		start[25] = 0.0037212;
		start[26] = 0.0035609;
		start[27] = 0.0040189;
		start[28] = 0.004683;
		start[29] = 0.0014046;
		start[30] = 0.0036086;
		start[31] = 0.0012111;
		start[32] = 0.0042651;
		start[33] = 0.0016673;
		start[34] = 0.0032099;
		start[35] = 0.0025526;
		start[36] = 0.0025844;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.005255;
		start[6] = 0.0073489;
		start[7] = 0.0010363;
		start[8] = 0.0074556;
		start[9] = 0.0059262;
		start[10] = 0.0025612;
		start[11] = 0.0007134;
		start[12] = 0.0061779;
		start[13] = 0.00072914;
		start[14] = 0.0012911;
		start[15] = 0.0080113;
		start[16] = 0.0048391;
		start[17] = 0.00055029;
		start[18] = 0.0066678;
		start[19] = 0.00053864;
		start[20] = 0.0079811;
		start[21] = 0.0062152;
		start[22] = 0.0042149;
		start[23] = 0.0014316;
		start[24] = 0.0061946;
		start[25] = 0.008319;
		start[26] = 0.0010976;
		start[27] = 0.0075063;
		start[28] = 0.0021451;
		start[29] = 0.0069104;
		start[30] = 0.0064641;
		start[31] = 0.0020001;
		start[32] = 0.0054015;
		start[33] = 0.0046333;
		start[34] = 0.0061338;
		start[35] = 0.0018951;
		start[36] = 0.0077863;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (37);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0031276;
		start[6] = 0.00075959;
		start[7] = 0.00611;
		start[8] = 0.0025846;
		start[9] = 0.007119;
		start[10] = 0.0093861;
		start[11] = 0.0072469;
		start[12] = 0.0054211;
		start[13] = 0.0089896;
		start[14] = 0.0074133;
		start[15] = 0.0048669;
		start[16] = 0.0057814;
		start[17] = 0.0089681;
		start[18] = 0.0004471;
		start[19] = 0.0012764;
		start[20] = 0.0010881;
		start[21] = 0.0047501;
		start[22] = 0.0084648;
		start[23] = 0.0073071;
		start[24] = 0.0062383;
		start[25] = 0.0099968;
		start[26] = 0.0086616;
		start[27] = 0.00061829;
		start[28] = 0.0017711;
		start[29] = 0.00753;
		start[30] = 6.0127e-05;
		start[31] = 0.0074846;
		start[32] = 0.007095;
		start[33] = 0.0031046;
		start[34] = 0.0097241;
		start[35] = 0.0059659;
		start[36] = 0.0044862;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}


  return 0;
}
