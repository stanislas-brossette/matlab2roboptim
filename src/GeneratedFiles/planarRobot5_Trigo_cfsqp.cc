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
    (32, 1, "CostFunction_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

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
    (32, 1, "LiftConstraint_1_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = pow(w_01_01,2) + pow(w_01_02,2) - 1.0;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 2.0*w_01_01; 
			 grad[1] = 2.0*w_01_02; 
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
    (32, 1, "LiftConstraint_2_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = pow(w_01_03,2) + pow(w_01_06,2) - 1.0;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 2.0*w_01_03; 
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
    (32, 1, "LiftConstraint_3_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = pow(w_01_04,2) + pow(w_01_05,2) - 1.0;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 2.0*w_01_04; 
			 grad[4] = 2.0*w_01_05; 
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
    (32, 1, "LiftConstraint_4_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = pow(w_01_07,2) + pow(w_01_08,2) - 1.0;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

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
    (32, 1, "LiftConstraint_5_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = pow(w_01_09,2) + pow(w_01_10,2) - 1.0;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

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
			 grad[8] = 2.0*w_01_09; 
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
    (32, 1, "LiftConstraint_6_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_03*w_01_09 - 1.0*w_02_01;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = w_01_09; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = w_01_03; 
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
    (32, 1, "LiftConstraint_7_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_06*w_01_09 - 1.0*w_02_02;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_01_09; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = w_01_06; 
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
    (32, 1, "LiftConstraint_8_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_03*w_01_10 - 1.0*w_02_03;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = w_01_10; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = w_01_03; 
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
    (32, 1, "LiftConstraint_9_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_06*w_01_10 - 1.0*w_02_04;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_01_10; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = w_01_06; 
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
    (32, 1, "LiftConstraint_10_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_07*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_01;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

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
			 grad[10] = w_01_07; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = -1.0*w_01_07; 
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
    (32, 1, "LiftConstraint_11_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = - 1.0*w_03_02 - 1.0*w_01_08*(w_02_02 + w_02_03);
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

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
			 grad[7] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = -1.0*w_01_08; 
			 grad[12] = -1.0*w_01_08; 
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
    (32, 1, "LiftConstraint_12_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_07*(w_02_02 + w_02_03) - 1.0*w_03_03;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

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
			 grad[11] = w_01_07; 
			 grad[12] = w_01_07; 
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
    (32, 1, "LiftConstraint_13_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_08*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_04;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

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
			 grad[7] = w_02_01 - 1.0*w_02_04; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = w_01_08; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = -1.0*w_01_08; 
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
    (32, 1, "LiftConstraint_14_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = - 1.0*w_03_05 - 1.0*w_01_07*(w_02_02 + w_02_03);
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

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
			 grad[11] = -1.0*w_01_07; 
			 grad[12] = -1.0*w_01_07; 
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
    (32, 1, "LiftConstraint_15_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_08*(w_02_02 + w_02_03) - 1.0*w_03_06;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

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
			 grad[7] = w_02_02 + w_02_03; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = w_01_08; 
			 grad[12] = w_01_08; 
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
    (32, 1, "LiftConstraint_16_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_04*(w_03_01 + w_03_02) - 1.0*w_04_01;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_03_01 + w_03_02; 
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
			 grad[14] = w_01_04; 
			 grad[15] = w_01_04; 
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
    (32, 1, "LiftConstraint_17_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = - 1.0*w_04_02 - 1.0*w_01_05*(w_03_04 - 1.0*w_03_05);
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = w_03_05 - 1.0*w_03_04; 
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
			 grad[17] = -1.0*w_01_05; 
			 grad[18] = w_01_05; 
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
    (32, 1, "LiftConstraint_18_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_04*(w_03_03 + w_03_04) - 1.0*w_04_03;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_03_03 + w_03_04; 
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
			 grad[16] = w_01_04; 
			 grad[17] = w_01_04; 
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
    (32, 1, "LiftConstraint_19_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_05*(w_03_01 - 1.0*w_03_06) - 1.0*w_04_04;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = w_03_01 - 1.0*w_03_06; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = w_01_05; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = -1.0*w_01_05; 
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
    (32, 1, "LiftConstraint_20_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = - 1.0*w_04_05 - 1.0*w_01_04*(w_03_04 - 1.0*w_03_05);
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_03_05 - 1.0*w_03_04; 
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
			 grad[17] = -1.0*w_01_04; 
			 grad[18] = w_01_04; 
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
    (32, 1, "LiftConstraint_21_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_04*(w_03_01 - 1.0*w_03_06) - 1.0*w_04_06;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_03_01 - 1.0*w_03_06; 
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
			 grad[14] = w_01_04; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = -1.0*w_01_04; 
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
    (32, 1, "LiftConstraint_22_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_05*(w_03_01 + w_03_02) - 1.0*w_04_07;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = w_03_01 + w_03_02; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = w_01_05; 
			 grad[15] = w_01_05; 
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
    (32, 1, "LiftConstraint_23_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_05*(w_03_03 + w_03_04) - 1.0*w_04_08;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = w_03_03 + w_03_04; 
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
			 grad[16] = w_01_05; 
			 grad[17] = w_01_05; 
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
    (32, 1, "LiftConstraint_24_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_01*(w_04_01 + w_04_02) - 1.0*w_05_01;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = w_04_01 + w_04_02; 
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
			 grad[20] = w_01_01; 
			 grad[21] = w_01_01; 
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
    (32, 1, "LiftConstraint_25_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_02*(w_04_05 - 1.0*w_04_07) - 1.0*w_05_02;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = w_04_05 - 1.0*w_04_07; 
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
			 grad[24] = w_01_02; 
			 grad[25] = 0.0; 
			 grad[26] = -1.0*w_01_02; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = -1.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
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
    (32, 1, "LiftConstraint_26_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_01*(w_04_03 + w_04_04) - 1.0*w_05_03;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = w_04_03 + w_04_04; 
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
			 grad[22] = w_01_01; 
			 grad[23] = w_01_01; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = -1.0; 
			 grad[31] = 0.0; 
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
    (32, 1, "LiftConstraint_27_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_02*(w_04_06 - 1.0*w_04_08) - 1.0*w_05_04;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = w_04_06 - 1.0*w_04_08; 
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
			 grad[25] = w_01_02; 
			 grad[26] = 0.0; 
			 grad[27] = -1.0*w_01_02; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 0.0; 
			 grad[31] = -1.0; 
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
    (32, 1, "EEConstraint_1_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_03 - 1.0*EE_1_1 + w_02_01 - 1.0*w_02_04 + w_03_01 + w_03_02 + w_04_01 + w_04_02 + w_05_01 + w_05_02;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 1.0; 
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
			 grad[13] = -1.0; 
			 grad[14] = 1.0; 
			 grad[15] = 1.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 1.0; 
			 grad[21] = 1.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 1.0; 
			 grad[29] = 1.0; 
			 grad[30] = 0.0; 
			 grad[31] = 0.0; 
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
    (32, 1, "EEConstraint_2_planarRobot5"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];
  
	result[0] = w_01_06 - 1.0*EE_1_2 + w_02_02 + w_02_03 + w_03_03 + w_03_04 + w_04_03 + w_04_04 + w_05_03 + w_05_04;
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
	const double& w_02_01 = x[10];
	const double& w_02_02 = x[11];
	const double& w_02_03 = x[12];
	const double& w_02_04 = x[13];
	const double& w_03_01 = x[14];
	const double& w_03_02 = x[15];
	const double& w_03_03 = x[16];
	const double& w_03_04 = x[17];
	const double& w_03_05 = x[18];
	const double& w_03_06 = x[19];
	const double& w_04_01 = x[20];
	const double& w_04_02 = x[21];
	const double& w_04_03 = x[22];
	const double& w_04_04 = x[23];
	const double& w_04_05 = x[24];
	const double& w_04_06 = x[25];
	const double& w_04_07 = x[26];
	const double& w_04_08 = x[27];
	const double& w_05_01 = x[28];
	const double& w_05_02 = x[29];
	const double& w_05_03 = x[30];
	const double& w_05_04 = x[31];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 1.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 1.0; 
			 grad[12] = 1.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 1.0; 
			 grad[17] = 1.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 1.0; 
			 grad[23] = 1.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
			 grad[26] = 0.0; 
			 grad[27] = 0.0; 
			 grad[28] = 0.0; 
			 grad[29] = 0.0; 
			 grad[30] = 1.0; 
			 grad[31] = 1.0; 
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
	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_28 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_29 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);

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
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_28), bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_29), bounds, scales); 
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
		std::vector<double> start (32);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0080586;
		start[6] = 0.004296;
		start[7] = 0.0021855;
		start[8] = 0.0035518;
		start[9] = 0.0059562;
		start[10] = 0.0063122;
		start[11] = 0.0022823;
		start[12] = 0.00018411;
		start[13] = 0.0067103;
		start[14] = 0.0056873;
		start[15] = 0.0008195;
		start[16] = 0.0095586;
		start[17] = 0.0082129;
		start[18] = 0.00056694;
		start[19] = 0.0095893;
		start[20] = 0.0069057;
		start[21] = 0.0023526;
		start[22] = 0.0051207;
		start[23] = 0.0095088;
		start[24] = 0.0097057;
		start[25] = 0.0077206;
		start[26] = 0.00049846;
		start[27] = 0.0052096;
		start[28] = 0.0057678;
		start[29] = 0.0073815;
		start[30] = 0.0082906;
		start[31] = 0.0065192;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0039717;
		start[6] = 0.0037664;
		start[7] = 0.0005656;
		start[8] = 0.0040088;
		start[9] = 0.0020697;
		start[10] = 0.0080452;
		start[11] = 0.0017793;
		start[12] = 0.0038137;
		start[13] = 0.0026395;
		start[14] = 0.00019667;
		start[15] = 0.0058335;
		start[16] = 0.0023241;
		start[17] = 0.006612;
		start[18] = 0.00037396;
		start[19] = 0.001621;
		start[20] = 0.0053195;
		start[21] = 0.0061019;
		start[22] = 0.0054345;
		start[23] = 0.0007005;
		start[24] = 0.00097116;
		start[25] = 0.007933;
		start[26] = 0.0029464;
		start[27] = 0.0019449;
		start[28] = 0.0018517;
		start[29] = 0.0018397;
		start[30] = 0.0078407;
		start[31] = 0.0052298;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.007687;
		start[6] = 0.004236;
		start[7] = 0.0080391;
		start[8] = 0.0093669;
		start[9] = 0.0048798;
		start[10] = 0.0036448;
		start[11] = 0.004199;
		start[12] = 0.0038949;
		start[13] = 0.0066304;
		start[14] = 0.0096814;
		start[15] = 0.007508;
		start[16] = 0.005932;
		start[17] = 0.0097818;
		start[18] = 0.0055449;
		start[19] = 0.0075556;
		start[20] = 0.0026954;
		start[21] = 0.0085036;
		start[22] = 0.0060985;
		start[23] = 0.0049812;
		start[24] = 0.0043944;
		start[25] = 0.0018865;
		start[26] = 0.0087261;
		start[27] = 0.00092992;
		start[28] = 0.0076107;
		start[29] = 4.7266e-05;
		start[30] = 0.00077202;
		start[31] = 0.0012877;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0093805;
		start[6] = 0.0076939;
		start[7] = 0.0054671;
		start[8] = 0.007986;
		start[9] = 0.0056451;
		start[10] = 0.0090208;
		start[11] = 0.0084357;
		start[12] = 0.0083372;
		start[13] = 0.0019878;
		start[14] = 0.0064676;
		start[15] = 0.00073502;
		start[16] = 0.0089593;
		start[17] = 0.0069707;
		start[18] = 0.0020122;
		start[19] = 0.0063024;
		start[20] = 0.00018942;
		start[21] = 0.0080855;
		start[22] = 0.00016706;
		start[23] = 0.0084612;
		start[24] = 0.0016023;
		start[25] = 0.0071257;
		start[26] = 0.0037187;
		start[27] = 0.0056941;
		start[28] = 0.0095518;
		start[29] = 0.0062462;
		start[30] = 0.00062761;
		start[31] = 0.002058;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.00071982;
		start[6] = 0.00018303;
		start[7] = 0.0053892;
		start[8] = 0.0090127;
		start[9] = 0.0082482;
		start[10] = 0.0043685;
		start[11] = 0.00096482;
		start[12] = 0.0040681;
		start[13] = 0.003842;
		start[14] = 0.0064394;
		start[15] = 0.0056329;
		start[16] = 0.0019443;
		start[17] = 0.0061728;
		start[18] = 0.0032309;
		start[19] = 0.0087331;
		start[20] = 0.0075705;
		start[21] = 0.0075206;
		start[22] = 0.0054062;
		start[23] = 0.0053911;
		start[24] = 0.00078321;
		start[25] = 0.0034527;
		start[26] = 0.0069364;
		start[27] = 0.0027469;
		start[28] = 0.0014338;
		start[29] = 0.006161;
		start[30] = 0.0054772;
		start[31] = 0.00098374;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0068929;
		start[6] = 0.0031116;
		start[7] = 0.0091485;
		start[8] = 0.0077951;
		start[9] = 0.0053781;
		start[10] = 0.0040224;
		start[11] = 0.008054;
		start[12] = 0.001688;
		start[13] = 0.0074251;
		start[14] = 0.0028558;
		start[15] = 0.0024407;
		start[16] = 0.0026248;
		start[17] = 0.0036623;
		start[18] = 0.0089909;
		start[19] = 0.0084897;
		start[20] = 0.0051174;
		start[21] = 0.0010667;
		start[22] = 0.0059755;
		start[23] = 0.0056105;
		start[24] = 8.5658e-05;
		start[25] = 0.0051791;
		start[26] = 0.002654;
		start[27] = 0.0046804;
		start[28] = 0.0021932;
		start[29] = 0.0089713;
		start[30] = 0.0019923;
		start[31] = 0.0066039;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.002181;
		start[6] = 0.0065999;
		start[7] = 0.0027531;
		start[8] = 0.0042999;
		start[9] = 0.0098595;
		start[10] = 0.00098248;
		start[11] = 0.0077017;
		start[12] = 0.0097009;
		start[13] = 0.0011697;
		start[14] = 0.006925;
		start[15] = 0.0062884;
		start[16] = 0.0033122;
		start[17] = 0.006371;
		start[18] = 0.0062892;
		start[19] = 0.0023542;
		start[20] = 0.0067265;
		start[21] = 0.0027729;
		start[22] = 0.0092807;
		start[23] = 0.0070954;
		start[24] = 0.0047519;
		start[25] = 0.0041115;
		start[26] = 0.0076472;
		start[27] = 0.0030278;
		start[28] = 0.0094484;
		start[29] = 0.0049129;
		start[30] = 0.0067966;
		start[31] = 0.0032318;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0038505;
		start[6] = 0.0052481;
		start[7] = 0.0015292;
		start[8] = 0.0051763;
		start[9] = 0.0043796;
		start[10] = 0.0074673;
		start[11] = 0.0095986;
		start[12] = 0.0044182;
		start[13] = 0.009694;
		start[14] = 0.0050014;
		start[15] = 0.002127;
		start[16] = 0.0050905;
		start[17] = 0.00047927;
		start[18] = 0.007434;
		start[19] = 5.7813e-05;
		start[20] = 0.005641;
		start[21] = 0.0024833;
		start[22] = 0.0043993;
		start[23] = 0.0081282;
		start[24] = 0.0083904;
		start[25] = 0.0097059;
		start[26] = 0.0054706;
		start[27] = 0.0026975;
		start[28] = 0.0001284;
		start[29] = 0.0050077;
		start[30] = 0.002467;
		start[31] = 0.00435;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.00012028;
		start[6] = 0.0060851;
		start[7] = 0.0027378;
		start[8] = 0.0090868;
		start[9] = 0.0085794;
		start[10] = 0.0042851;
		start[11] = 0.0021821;
		start[12] = 0.0053866;
		start[13] = 0.0068492;
		start[14] = 0.0072305;
		start[15] = 0.0037155;
		start[16] = 0.0066924;
		start[17] = 0.0019484;
		start[18] = 0.0026358;
		start[19] = 0.0061562;
		start[20] = 0.0066645;
		start[21] = 0.0054248;
		start[22] = 0.0039203;
		start[23] = 0.0091455;
		start[24] = 0.0027326;
		start[25] = 0.00084796;
		start[26] = 0.0016099;
		start[27] = 0.0003302;
		start[28] = 0.0076239;
		start[29] = 0.0065819;
		start[30] = 0.00062167;
		start[31] = 0.0011727;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0078949;
		start[6] = 0.0042337;
		start[7] = 0.0036769;
		start[8] = 0.0077112;
		start[9] = 0.008005;
		start[10] = 0.004034;
		start[11] = 0.0038431;
		start[12] = 0.002382;
		start[13] = 0.0033014;
		start[14] = 0.0051114;
		start[15] = 0.0093435;
		start[16] = 0.0035073;
		start[17] = 0.0095651;
		start[18] = 0.0036387;
		start[19] = 0.0050477;
		start[20] = 0.0023408;
		start[21] = 0.0075584;
		start[22] = 0.0079294;
		start[23] = 0.00077796;
		start[24] = 0.0092877;
		start[25] = 0.0079578;
		start[26] = 0.0058165;
		start[27] = 0.00069615;
		start[28] = 0.0064694;
		start[29] = 0.0017663;
		start[30] = 0.0017544;
		start[31] = 0.0044088;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0020667;
		start[6] = 0.0081991;
		start[7] = 0.0077501;
		start[8] = 0.0046176;
		start[9] = 0.0091884;
		start[10] = 0.007617;
		start[11] = 0.0054691;
		start[12] = 0.0021281;
		start[13] = 0.005574;
		start[14] = 0.0011734;
		start[15] = 0.0095226;
		start[16] = 0.0076202;
		start[17] = 0.0042805;
		start[18] = 0.005153;
		start[19] = 0.0027379;
		start[20] = 3.6073e-05;
		start[21] = 0.008197;
		start[22] = 0.0080133;
		start[23] = 0.0072262;
		start[24] = 0.00033266;
		start[25] = 0.0064509;
		start[26] = 0.0056482;
		start[27] = 4.4268e-05;
		start[28] = 0.0099188;
		start[29] = 0.0093366;
		start[30] = 0.007516;
		start[31] = 0.0071228;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0051459;
		start[6] = 0.00034859;
		start[7] = 0.0075732;
		start[8] = 0.0023556;
		start[9] = 0.0048353;
		start[10] = 0.0025857;
		start[11] = 0.00013188;
		start[12] = 0.0065305;
		start[13] = 0.003977;
		start[14] = 0.0042169;
		start[15] = 0.0020454;
		start[16] = 0.0079355;
		start[17] = 0.0070941;
		start[18] = 0.00075897;
		start[19] = 0.0087077;
		start[20] = 0.0041715;
		start[21] = 0.00033345;
		start[22] = 0.0038867;
		start[23] = 0.009068;
		start[24] = 0.0051955;
		start[25] = 0.0060744;
		start[26] = 0.0070994;
		start[27] = 0.0039126;
		start[28] = 0.0029274;
		start[29] = 0.0098899;
		start[30] = 0.0089499;
		start[31] = 0.001166;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0089229;
		start[6] = 0.0063401;
		start[7] = 0.0094788;
		start[8] = 0.004323;
		start[9] = 0.0043226;
		start[10] = 0.0088905;
		start[11] = 0.0088453;
		start[12] = 0.0049614;
		start[13] = 0.0031435;
		start[14] = 0.0034004;
		start[15] = 0.004944;
		start[16] = 0.0014867;
		start[17] = 0.00037345;
		start[18] = 0.0089004;
		start[19] = 0.0008718;
		start[20] = 0.0018723;
		start[21] = 0.006674;
		start[22] = 0.0031035;
		start[23] = 0.0085973;
		start[24] = 0.0072677;
		start[25] = 0.00055186;
		start[26] = 0.0073752;
		start[27] = 0.0069433;
		start[28] = 0.0090436;
		start[29] = 0.0092018;
		start[30] = 0.0031662;
		start[31] = 0.0046651;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0013758;
		start[6] = 0.00094096;
		start[7] = 0.0053915;
		start[8] = 0.0013703;
		start[9] = 0.0033775;
		start[10] = 0.0035159;
		start[11] = 0.0050206;
		start[12] = 0.0067453;
		start[13] = 0.00018171;
		start[14] = 0.0073979;
		start[15] = 0.0072158;
		start[16] = 0.0024706;
		start[17] = 0.0094687;
		start[18] = 0.0011614;
		start[19] = 0.0013467;
		start[20] = 0.001498;
		start[21] = 0.0093318;
		start[22] = 0.0015832;
		start[23] = 0.0011795;
		start[24] = 0.0036541;
		start[25] = 0.0083188;
		start[26] = 0.0095518;
		start[27] = 0.0083436;
		start[28] = 0.0008392;
		start[29] = 0.0061057;
		start[30] = 0.0020216;
		start[31] = 0.0058846;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.00925;
		start[6] = 0.0040362;
		start[7] = 0.0093807;
		start[8] = 0.0073981;
		start[9] = 0.00066067;
		start[10] = 0.0014371;
		start[11] = 0.0086647;
		start[12] = 0.0032243;
		start[13] = 0.0076288;
		start[14] = 0.0088059;
		start[15] = 0.0050524;
		start[16] = 0.0047077;
		start[17] = 0.0068488;
		start[18] = 0.002949;
		start[19] = 0.0065493;
		start[20] = 0.0090639;
		start[21] = 0.0080288;
		start[22] = 0.0071814;
		start[23] = 0.0069398;
		start[24] = 0.0059993;
		start[25] = 0.0064443;
		start[26] = 0.00378;
		start[27] = 0.0061544;
		start[28] = 0.00073736;
		start[29] = 0.0020928;
		start[30] = 0.0031298;
		start[31] = 0.0087898;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0020307;
		start[6] = 0.0025605;
		start[7] = 0.0083179;
		start[8] = 0.0044093;
		start[9] = 0.0094927;
		start[10] = 0.0059021;
		start[11] = 0.0054485;
		start[12] = 0.006715;
		start[13] = 0.003399;
		start[14] = 0.0098376;
		start[15] = 0.0040656;
		start[16] = 0.009566;
		start[17] = 0.0079427;
		start[18] = 0.0094842;
		start[19] = 0.0049991;
		start[20] = 0.0011086;
		start[21] = 0.0056056;
		start[22] = 0.0064013;
		start[23] = 0.0065409;
		start[24] = 0.0079855;
		start[25] = 0.0081946;
		start[26] = 0.0017677;
		start[27] = 0.0056994;
		start[28] = 0.0010572;
		start[29] = 0.008092;
		start[30] = 0.0031842;
		start[31] = 0.0075139;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.009301;
		start[6] = 0.0098175;
		start[7] = 0.00056649;
		start[8] = 0.0084357;
		start[9] = 0.0068042;
		start[10] = 0.005058;
		start[11] = 0.0029538;
		start[12] = 0.0029886;
		start[13] = 0.0092745;
		start[14] = 0.009732;
		start[15] = 0.0097968;
		start[16] = 0.0064969;
		start[17] = 0.0015444;
		start[18] = 0.0019311;
		start[19] = 0.0093832;
		start[20] = 0.0025081;
		start[21] = 0.0022129;
		start[22] = 0.0081373;
		start[23] = 0.0044141;
		start[24] = 0.0076334;
		start[25] = 0.0015027;
		start[26] = 0.00091905;
		start[27] = 0.0017645;
		start[28] = 0.0020947;
		start[29] = 0.0053064;
		start[30] = 0.0025637;
		start[31] = 0.0097742;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.00041178;
		start[6] = 0.0078593;
		start[7] = 0.0071647;
		start[8] = 0.0073929;
		start[9] = 0.0041622;
		start[10] = 0.0051705;
		start[11] = 0.004322;
		start[12] = 0.0017922;
		start[13] = 0.0055868;
		start[14] = 0.0029214;
		start[15] = 0.0025781;
		start[16] = 0.0060075;
		start[17] = 0.0069069;
		start[18] = 0.0047731;
		start[19] = 0.0011527;
		start[20] = 0.0064006;
		start[21] = 0.0047155;
		start[22] = 0.0099997;
		start[23] = 0.003444;
		start[24] = 0.0023;
		start[25] = 0.00071139;
		start[26] = 0.00029991;
		start[27] = 0.0016338;
		start[28] = 0.0022215;
		start[29] = 0.0037413;
		start[30] = 0.0089036;
		start[31] = 0.0015776;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0057282;
		start[6] = 0.0061584;
		start[7] = 0.0039948;
		start[8] = 0.0014383;
		start[9] = 0.0086461;
		start[10] = 0.007312;
		start[11] = 0.0071307;
		start[12] = 0.0025253;
		start[13] = 0.0094965;
		start[14] = 0.0068555;
		start[15] = 0.0021023;
		start[16] = 0.0019284;
		start[17] = 0.0065144;
		start[18] = 0.0012878;
		start[19] = 0.0010768;
		start[20] = 0.0090461;
		start[21] = 0.0026489;
		start[22] = 0.00076953;
		start[23] = 0.0030371;
		start[24] = 0.0066705;
		start[25] = 0.0063086;
		start[26] = 0.00080142;
		start[27] = 0.0036736;
		start[28] = 0.0092115;
		start[29] = 0.0081592;
		start[30] = 0.0025894;
		start[31] = 0.0039456;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0067856;
		start[6] = 5.0096e-05;
		start[7] = 0.0044198;
		start[8] = 0.00094896;
		start[9] = 0.0052187;
		start[10] = 0.0039315;
		start[11] = 0.009876;
		start[12] = 0.0065763;
		start[13] = 0.0057736;
		start[14] = 0.0076298;
		start[15] = 0.0079881;
		start[16] = 0.0090895;
		start[17] = 0.0030902;
		start[18] = 0.0016693;
		start[19] = 0.0085535;
		start[20] = 0.0027794;
		start[21] = 0.0019538;
		start[22] = 0.0041561;
		start[23] = 0.001003;
		start[24] = 0.0001023;
		start[25] = 0.0004234;
		start[26] = 0.0057392;
		start[27] = 0.0026053;
		start[28] = 0.0051693;
		start[29] = 0.0060963;
		start[30] = 0.0017748;
		start[31] = 0.00068565;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0094999;
		start[6] = 0.0067966;
		start[7] = 0.0072996;
		start[8] = 0.0056852;
		start[9] = 0.0049493;
		start[10] = 0.0038649;
		start[11] = 0.0007433;
		start[12] = 0.0036605;
		start[13] = 0.0076458;
		start[14] = 0.0018444;
		start[15] = 0.0051904;
		start[16] = 0.0081531;
		start[17] = 0.0047839;
		start[18] = 0.0035287;
		start[19] = 0.0079127;
		start[20] = 0.0010143;
		start[21] = 0.0084063;
		start[22] = 0.0062358;
		start[23] = 0.0035313;
		start[24] = 0.00087876;
		start[25] = 0.009903;
		start[26] = 0.00084144;
		start[27] = 0.0080231;
		start[28] = 0.0015682;
		start[29] = 0.0095986;
		start[30] = 0.0080177;
		start[31] = 0.0012083;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0087561;
		start[6] = 0.0086726;
		start[7] = 0.0045607;
		start[8] = 0.00043219;
		start[9] = 0.0014986;
		start[10] = 0.0099018;
		start[11] = 0.0049187;
		start[12] = 0.0093048;
		start[13] = 0.00054523;
		start[14] = 0.0030926;
		start[15] = 0.0087778;
		start[16] = 0.00036784;
		start[17] = 0.0096098;
		start[18] = 0.0051011;
		start[19] = 0.0091284;
		start[20] = 0.0057376;
		start[21] = 0.0072252;
		start[22] = 0.0090361;
		start[23] = 0.0078564;
		start[24] = 0.0058546;
		start[25] = 0.0079098;
		start[26] = 0.0074829;
		start[27] = 0.0031424;
		start[28] = 0.0050216;
		start[29] = 0.0074169;
		start[30] = 0.005344;
		start[31] = 0.0069805;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.006769;
		start[6] = 0.0076624;
		start[7] = 0.0063468;
		start[8] = 0.00539;
		start[9] = 0.00612;
		start[10] = 0.008003;
		start[11] = 0.0061466;
		start[12] = 0.00059019;
		start[13] = 0.0034256;
		start[14] = 0.0052864;
		start[15] = 0.0033515;
		start[16] = 0.005752;
		start[17] = 0.0023306;
		start[18] = 0.0015523;
		start[19] = 0.0076059;
		start[20] = 0.001468;
		start[21] = 0.0045276;
		start[22] = 0.00294;
		start[23] = 9.4994e-05;
		start[24] = 0.0093808;
		start[25] = 0.00095401;
		start[26] = 0.0046136;
		start[27] = 0.0069959;
		start[28] = 0.0040021;
		start[29] = 0.0088903;
		start[30] = 0.0061209;
		start[31] = 0.0097638;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0056379;
		start[6] = 0.0017771;
		start[7] = 0.0039217;
		start[8] = 0.0024818;
		start[9] = 0.0013532;
		start[10] = 0.0082733;
		start[11] = 0.001277;
		start[12] = 0.0010366;
		start[13] = 0.0022543;
		start[14] = 0.0054744;
		start[15] = 0.0087644;
		start[16] = 0.0033201;
		start[17] = 0.0037058;
		start[18] = 0.007861;
		start[19] = 0.005001;
		start[20] = 0.00090406;
		start[21] = 0.0033693;
		start[22] = 0.00062946;
		start[23] = 0.0034989;
		start[24] = 0.0070482;
		start[25] = 0.0064742;
		start[26] = 0.0014338;
		start[27] = 0.00022702;
		start[28] = 0.0018866;
		start[29] = 0.0050204;
		start[30] = 0.0020669;
		start[31] = 0.0083597;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0067296;
		start[6] = 0.00016772;
		start[7] = 0.0048234;
		start[8] = 0.0011847;
		start[9] = 0.0016788;
		start[10] = 0.0016153;
		start[11] = 0.0035306;
		start[12] = 0.0044989;
		start[13] = 0.008423;
		start[14] = 0.009126;
		start[15] = 0.0076269;
		start[16] = 0.00022688;
		start[17] = 0.0092661;
		start[18] = 0.0057238;
		start[19] = 0.0089572;
		start[20] = 0.0088464;
		start[21] = 0.0076781;
		start[22] = 0.0017761;
		start[23] = 0.0045944;
		start[24] = 0.0062261;
		start[25] = 0.0047082;
		start[26] = 0.004933;
		start[27] = 0.0068789;
		start[28] = 0.00034159;
		start[29] = 0.000912;
		start[30] = 2.4871e-05;
		start[31] = 0.0082276;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0061341;
		start[6] = 0.004308;
		start[7] = 0.0093736;
		start[8] = 0.00026187;
		start[9] = 0.001817;
		start[10] = 0.0042782;
		start[11] = 0.0022314;
		start[12] = 0.0068919;
		start[13] = 0.003081;
		start[14] = 0.0045007;
		start[15] = 0.0014295;
		start[16] = 0.0081191;
		start[17] = 0.009406;
		start[18] = 0.00013719;
		start[19] = 0.0044337;
		start[20] = 0.0055477;
		start[21] = 0.0032225;
		start[22] = 0.0087126;
		start[23] = 0.0014586;
		start[24] = 0.0094419;
		start[25] = 0.0058851;
		start[26] = 0.0065405;
		start[27] = 0.0016589;
		start[28] = 0.0065085;
		start[29] = 0.0096901;
		start[30] = 0.0084688;
		start[31] = 0.0053182;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0053032;
		start[6] = 0.0046948;
		start[7] = 0.0075308;
		start[8] = 0.0092593;
		start[9] = 0.0089888;
		start[10] = 0.0035105;
		start[11] = 0.0085281;
		start[12] = 0.0074729;
		start[13] = 0.002267;
		start[14] = 0.0084416;
		start[15] = 0.0022782;
		start[16] = 0.007538;
		start[17] = 0.0040322;
		start[18] = 0.0034228;
		start[19] = 0.0014806;
		start[20] = 0.0025333;
		start[21] = 0.0081077;
		start[22] = 0.0027261;
		start[23] = 0.0016773;
		start[24] = 0.009807;
		start[25] = 0.0067925;
		start[26] = 0.00061379;
		start[27] = 0.0047541;
		start[28] = 0.00069974;
		start[29] = 0.0038665;
		start[30] = 0.0062472;
		start[31] = 0.0089941;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0078073;
		start[6] = 0.0058012;
		start[7] = 0.006861;
		start[8] = 0.0099182;
		start[9] = 0.001517;
		start[10] = 0.0032372;
		start[11] = 0.0011834;
		start[12] = 0.00093823;
		start[13] = 0.0057884;
		start[14] = 0.0047138;
		start[15] = 0.0089384;
		start[16] = 0.0015615;
		start[17] = 0.00077262;
		start[18] = 0.0032992;
		start[19] = 0.0020831;
		start[20] = 0.0021172;
		start[21] = 0.00040034;
		start[22] = 0.0098287;
		start[23] = 0.0021506;
		start[24] = 0.0039912;
		start[25] = 0.00061617;
		start[26] = 0.0044685;
		start[27] = 0.007401;
		start[28] = 0.0014491;
		start[29] = 0.0051172;
		start[30] = 0.0081027;
		start[31] = 0.0026037;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0017608;
		start[6] = 0.0030093;
		start[7] = 0.0010047;
		start[8] = 0.003474;
		start[9] = 0.005416;
		start[10] = 0.00088905;
		start[11] = 0.0063944;
		start[12] = 0.005142;
		start[13] = 0.00010454;
		start[14] = 0.0071779;
		start[15] = 0.0069378;
		start[16] = 0.0057643;
		start[17] = 0.0080812;
		start[18] = 0.00037847;
		start[19] = 0.0032045;
		start[20] = 0.0041479;
		start[21] = 0.0086038;
		start[22] = 0.0064308;
		start[23] = 0.0039502;
		start[24] = 0.0037982;
		start[25] = 0.0067268;
		start[26] = 0.00075105;
		start[27] = 0.0062034;
		start[28] = 0.0060201;
		start[29] = 0.006814;
		start[30] = 0.009761;
		start[31] = 0.0054458;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0032638;
		start[6] = 0.0064329;
		start[7] = 0.0070088;
		start[8] = 0.0038392;
		start[9] = 0.0090025;
		start[10] = 0.00086742;
		start[11] = 0.00037862;
		start[12] = 0.0025961;
		start[13] = 0.0032413;
		start[14] = 0.0073628;
		start[15] = 0.0089598;
		start[16] = 0.0074226;
		start[17] = 0.0052559;
		start[18] = 0.0096103;
		start[19] = 0.0024776;
		start[20] = 0.00565;
		start[21] = 0.0059816;
		start[22] = 0.0068201;
		start[23] = 0.0060149;
		start[24] = 0.003029;
		start[25] = 0.0032333;
		start[26] = 0.0015765;
		start[27] = 0.005971;
		start[28] = 0.0057739;
		start[29] = 0.008167;
		start[30] = 0.0021702;
		start[31] = 0.0017246;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0066362;
		start[6] = 0.0031163;
		start[7] = 0.0071944;
		start[8] = 0.0013773;
		start[9] = 0.0049859;
		start[10] = 0.003608;
		start[11] = 0.0015945;
		start[12] = 0.0034443;
		start[13] = 0.00046096;
		start[14] = 0.0031792;
		start[15] = 0.0023484;
		start[16] = 0.0038364;
		start[17] = 0.0027685;
		start[18] = 0.0047658;
		start[19] = 0.0094496;
		start[20] = 0.0033655;
		start[21] = 0.0040411;
		start[22] = 0.0038343;
		start[23] = 0.0033883;
		start[24] = 0.0058272;
		start[25] = 0.0049824;
		start[26] = 0.0053453;
		start[27] = 0.0018841;
		start[28] = 0.0092438;
		start[29] = 0.0076151;
		start[30] = 0.0088323;
		start[31] = 0.0046429;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0058445;
		start[6] = 0.0046968;
		start[7] = 0.0073877;
		start[8] = 0.0028207;
		start[9] = 0.0013416;
		start[10] = 0.0088011;
		start[11] = 0.0023737;
		start[12] = 0.0082685;
		start[13] = 0.0043161;
		start[14] = 0.0041239;
		start[15] = 0.0088758;
		start[16] = 0.0030999;
		start[17] = 0.0075306;
		start[18] = 0.0082595;
		start[19] = 0.0050995;
		start[20] = 0.0031856;
		start[21] = 0.0081234;
		start[22] = 0.0086912;
		start[23] = 0.0028582;
		start[24] = 0.00094796;
		start[25] = 0.0044445;
		start[26] = 0.0072273;
		start[27] = 0.0038808;
		start[28] = 0.0079107;
		start[29] = 0.0064129;
		start[30] = 0.00026865;
		start[31] = 0.00084276;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0056948;
		start[6] = 0.0092645;
		start[7] = 0.0014471;
		start[8] = 0.0061783;
		start[9] = 0.0036488;
		start[10] = 0.0081682;
		start[11] = 0.0056845;
		start[12] = 0.0098635;
		start[13] = 0.0065309;
		start[14] = 0.001992;
		start[15] = 0.00085177;
		start[16] = 0.0012641;
		start[17] = 0.0098702;
		start[18] = 0.009626;
		start[19] = 0.0087328;
		start[20] = 0.0076773;
		start[21] = 0.0056906;
		start[22] = 0.0005509;
		start[23] = 0.008012;
		start[24] = 0.0047492;
		start[25] = 0.0044083;
		start[26] = 0.0092379;
		start[27] = 0.0025784;
		start[28] = 0.0015387;
		start[29] = 0.00027098;
		start[30] = 0.0023544;
		start[31] = 0.0080508;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.00056782;
		start[6] = 0.0088465;
		start[7] = 0.003735;
		start[8] = 0.0019964;
		start[9] = 0.0030638;
		start[10] = 0.0010197;
		start[11] = 0.0021542;
		start[12] = 0.0033125;
		start[13] = 0.0063963;
		start[14] = 0.0060947;
		start[15] = 0.0031567;
		start[16] = 0.0007751;
		start[17] = 0.0050604;
		start[18] = 0.0054923;
		start[19] = 0.0073974;
		start[20] = 0.0093575;
		start[21] = 6.2023e-05;
		start[22] = 0.0016153;
		start[23] = 0.0091687;
		start[24] = 0.00083596;
		start[25] = 0.0035395;
		start[26] = 0.0027574;
		start[27] = 0.0019522;
		start[28] = 0.0035776;
		start[29] = 0.0028594;
		start[30] = 0.0008966;
		start[31] = 0.0012441;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0047041;
		start[6] = 0.0052317;
		start[7] = 0.0034324;
		start[8] = 0.0058295;
		start[9] = 0.0034146;
		start[10] = 0.0083176;
		start[11] = 0.0086145;
		start[12] = 0.0026119;
		start[13] = 0.00036062;
		start[14] = 0.0036814;
		start[15] = 0.0047107;
		start[16] = 0.0033187;
		start[17] = 0.0069886;
		start[18] = 0.0030014;
		start[19] = 0.0024983;
		start[20] = 0.0075144;
		start[21] = 0.0042573;
		start[22] = 0.0073344;
		start[23] = 0.0046513;
		start[24] = 0.0010084;
		start[25] = 0.0057663;
		start[26] = 0.0045141;
		start[27] = 0.0067453;
		start[28] = 0.0028605;
		start[29] = 0.0072779;
		start[30] = 0.0087844;
		start[31] = 0.00033563;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0074495;
		start[6] = 0.0069194;
		start[7] = 0.0087525;
		start[8] = 0.0024118;
		start[9] = 0.0082628;
		start[10] = 0.007994;
		start[11] = 0.0080194;
		start[12] = 0.00094918;
		start[13] = 0.0063298;
		start[14] = 0.0088435;
		start[15] = 0.0029445;
		start[16] = 0.0025871;
		start[17] = 0.0055875;
		start[18] = 0.0083015;
		start[19] = 0.0074038;
		start[20] = 0.0018398;
		start[21] = 0.0085121;
		start[22] = 0.0069344;
		start[23] = 0.00069506;
		start[24] = 0.0093686;
		start[25] = 0.0055277;
		start[26] = 0.005322;
		start[27] = 0.0032519;
		start[28] = 0.0073243;
		start[29] = 0.0030217;
		start[30] = 0.0021114;
		start[31] = 0.0015106;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.00099429;
		start[6] = 0.0018082;
		start[7] = 0.0083875;
		start[8] = 0.003381;
		start[9] = 0.0075136;
		start[10] = 0.0090331;
		start[11] = 0.0051578;
		start[12] = 0.0042883;
		start[13] = 0.0022883;
		start[14] = 0.0037714;
		start[15] = 0.0099546;
		start[16] = 0.0028765;
		start[17] = 0.0013152;
		start[18] = 0.0051166;
		start[19] = 0.0073366;
		start[20] = 0.0031634;
		start[21] = 0.00063347;
		start[22] = 0.0086697;
		start[23] = 0.0065882;
		start[24] = 0.0014544;
		start[25] = 0.0064965;
		start[26] = 0.0019376;
		start[27] = 0.0090021;
		start[28] = 0.0098028;
		start[29] = 0.0053955;
		start[30] = 0.0026228;
		start[31] = 0.0044997;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 4.9148e-05;
		start[6] = 0.0038576;
		start[7] = 0.0055248;
		start[8] = 0.0089167;
		start[9] = 0.0026148;
		start[10] = 0.0071915;
		start[11] = 0.00052807;
		start[12] = 0.0077419;
		start[13] = 0.007268;
		start[14] = 0.0039155;
		start[15] = 0.0057312;
		start[16] = 0.0037968;
		start[17] = 0.0036843;
		start[18] = 0.0072673;
		start[19] = 0.0085505;
		start[20] = 0.0066412;
		start[21] = 0.0064515;
		start[22] = 0.0058576;
		start[23] = 0.003271;
		start[24] = 0.0023337;
		start[25] = 0.0058044;
		start[26] = 0.0076999;
		start[27] = 0.0020766;
		start[28] = 0.0074429;
		start[29] = 0.004697;
		start[30] = 0.0098537;
		start[31] = 0.0038428;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0098492;
		start[6] = 0.0051881;
		start[7] = 0.0072093;
		start[8] = 0.0025876;
		start[9] = 0.0096436;
		start[10] = 0.0092743;
		start[11] = 0.0016083;
		start[12] = 0.00073967;
		start[13] = 0.0020406;
		start[14] = 0.0054305;
		start[15] = 0.0046877;
		start[16] = 0.0028446;
		start[17] = 0.0090707;
		start[18] = 0.0034547;
		start[19] = 0.005405;
		start[20] = 0.00074397;
		start[21] = 0.0044372;
		start[22] = 0.00691;
		start[23] = 0.00065852;
		start[24] = 0.0056943;
		start[25] = 0.0045014;
		start[26] = 0.00017786;
		start[27] = 0.0076706;
		start[28] = 0.0055466;
		start[29] = 0.00018961;
		start[30] = 0.0034103;
		start[31] = 0.0061755;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0014399;
		start[6] = 0.0001434;
		start[7] = 0.0058073;
		start[8] = 0.0062072;
		start[9] = 0.0059968;
		start[10] = 0.0033546;
		start[11] = 0.0081557;
		start[12] = 0.0039101;
		start[13] = 0.0088134;
		start[14] = 4.1958e-05;
		start[15] = 0.0081762;
		start[16] = 0.0034556;
		start[17] = 0.0030732;
		start[18] = 0.0029548;
		start[19] = 0.002491;
		start[20] = 0.0082249;
		start[21] = 0.0025146;
		start[22] = 0.0040469;
		start[23] = 0.0065319;
		start[24] = 0.0090102;
		start[25] = 0.0097963;
		start[26] = 0.0054017;
		start[27] = 0.00040553;
		start[28] = 0.0092429;
		start[29] = 0.0032152;
		start[30] = 0.00049999;
		start[31] = 0.0024019;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.00073442;
		start[6] = 0.0060269;
		start[7] = 0.0048888;
		start[8] = 0.0088412;
		start[9] = 0.002093;
		start[10] = 0.0067413;
		start[11] = 0.0096117;
		start[12] = 0.0084308;
		start[13] = 0.0059319;
		start[14] = 0.0060067;
		start[15] = 0.0049229;
		start[16] = 0.0066677;
		start[17] = 0.0038476;
		start[18] = 0.00257;
		start[19] = 0.005167;
		start[20] = 0.0047987;
		start[21] = 0.0096367;
		start[22] = 0.00087171;
		start[23] = 0.0020515;
		start[24] = 0.0018631;
		start[25] = 0.0082597;
		start[26] = 0.003047;
		start[27] = 0.0055683;
		start[28] = 0.0089052;
		start[29] = 0.0087687;
		start[30] = 0.0041101;
		start[31] = 0.00065591;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0084735;
		start[6] = 0.0023607;
		start[7] = 0.00056583;
		start[8] = 0.0084099;
		start[9] = 0.000966;
		start[10] = 0.0018311;
		start[11] = 0.00064021;
		start[12] = 0.0056277;
		start[13] = 0.0087727;
		start[14] = 0.003052;
		start[15] = 0.006523;
		start[16] = 0.0022349;
		start[17] = 0.00035036;
		start[18] = 0.0064435;
		start[19] = 0.0053947;
		start[20] = 0.0099608;
		start[21] = 0.0041923;
		start[22] = 0.0095099;
		start[23] = 0.0099267;
		start[24] = 0.0019915;
		start[25] = 0.0087026;
		start[26] = 0.0069817;
		start[27] = 0.0022554;
		start[28] = 0.0060009;
		start[29] = 0.00019226;
		start[30] = 0.0071325;
		start[31] = 0.0052343;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0073118;
		start[6] = 0.0053144;
		start[7] = 0.0086621;
		start[8] = 0.0030664;
		start[9] = 5.6313e-05;
		start[10] = 0.0018874;
		start[11] = 0.0091115;
		start[12] = 0.0089528;
		start[13] = 0.0084033;
		start[14] = 0.0025783;
		start[15] = 0.002087;
		start[16] = 0.0034818;
		start[17] = 0.004314;
		start[18] = 0.0059749;
		start[19] = 0.0037737;
		start[20] = 0.0096527;
		start[21] = 0.0032471;
		start[22] = 0.0057796;
		start[23] = 0.0015002;
		start[24] = 0.0063862;
		start[25] = 0.0061082;
		start[26] = 0.0033996;
		start[27] = 0.00025851;
		start[28] = 0.0073091;
		start[29] = 0.0036611;
		start[30] = 0.0043296;
		start[31] = 0.0012357;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0091528;
		start[6] = 0.0032207;
		start[7] = 0.0001819;
		start[8] = 0.0092551;
		start[9] = 0.0065056;
		start[10] = 0.0043348;
		start[11] = 0.0074703;
		start[12] = 0.00061556;
		start[13] = 0.0021683;
		start[14] = 0.00755;
		start[15] = 0.0022734;
		start[16] = 0.0084885;
		start[17] = 0.0017905;
		start[18] = 0.0029057;
		start[19] = 0.0099225;
		start[20] = 0.0040005;
		start[21] = 0.0093885;
		start[22] = 0.0098831;
		start[23] = 0.0029009;
		start[24] = 0.0090054;
		start[25] = 0.0083317;
		start[26] = 0.006657;
		start[27] = 0.0065547;
		start[28] = 0.0024579;
		start[29] = 0.0011163;
		start[30] = 0.0095321;
		start[31] = 0.0079895;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0025913;
		start[6] = 0.005006;
		start[7] = 0.0005225;
		start[8] = 0.0041382;
		start[9] = 0.0027784;
		start[10] = 0.0027347;
		start[11] = 0.0092421;
		start[12] = 0.0026079;
		start[13] = 0.0075038;
		start[14] = 0.0062984;
		start[15] = 0.0033553;
		start[16] = 0.0018149;
		start[17] = 0.0042957;
		start[18] = 0.0097562;
		start[19] = 0.00674;
		start[20] = 0.0074289;
		start[21] = 0.0014616;
		start[22] = 0.0036619;
		start[23] = 0.00015826;
		start[24] = 0.0024384;
		start[25] = 0.0084711;
		start[26] = 0.0090387;
		start[27] = 0.0032855;
		start[28] = 0.00088825;
		start[29] = 0.0061091;
		start[30] = 0.0021199;
		start[31] = 0.00463;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0081128;
		start[6] = 0.00023362;
		start[7] = 0.0099023;
		start[8] = 0.0042927;
		start[9] = 0.0046086;
		start[10] = 0.0066653;
		start[11] = 0.0031155;
		start[12] = 0.0039708;
		start[13] = 0.0079195;
		start[14] = 0.0070286;
		start[15] = 0.0074614;
		start[16] = 0.0057422;
		start[17] = 0.0062087;
		start[18] = 0.0019955;
		start[19] = 0.0090024;
		start[20] = 0.00097986;
		start[21] = 0.0029539;
		start[22] = 0.0089798;
		start[23] = 0.0077736;
		start[24] = 0.0017579;
		start[25] = 0.0092438;
		start[26] = 0.0084143;
		start[27] = 0.0019322;
		start[28] = 0.0046482;
		start[29] = 0.0029288;
		start[30] = 0.0068052;
		start[31] = 0.0023276;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0068253;
		start[6] = 0.0072793;
		start[7] = 0.00091659;
		start[8] = 0.0057462;
		start[9] = 0.0022203;
		start[10] = 0.00088583;
		start[11] = 0.0019129;
		start[12] = 0.0099521;
		start[13] = 3.3561e-05;
		start[14] = 0.0031671;
		start[15] = 0.006999;
		start[16] = 0.002553;
		start[17] = 0.0031348;
		start[18] = 0.00294;
		start[19] = 0.005776;
		start[20] = 0.0042613;
		start[21] = 0.0052872;
		start[22] = 0.0091948;
		start[23] = 0.00037993;
		start[24] = 0.0042878;
		start[25] = 0.0011059;
		start[26] = 0.0022654;
		start[27] = 0.001646;
		start[28] = 0.0046272;
		start[29] = 0.0034598;
		start[30] = 0.0076546;
		start[31] = 0.0079864;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0063631;
		start[6] = 0.0025556;
		start[7] = 1.5326e-05;
		start[8] = 0.0084951;
		start[9] = 0.0067478;
		start[10] = 0.00020529;
		start[11] = 0.0033466;
		start[12] = 0.0076592;
		start[13] = 0.0051599;
		start[14] = 0.00019436;
		start[15] = 0.0031997;
		start[16] = 0.0045867;
		start[17] = 0.0055453;
		start[18] = 0.0050802;
		start[19] = 0.0066724;
		start[20] = 0.0056163;
		start[21] = 0.0071288;
		start[22] = 0.0097075;
		start[23] = 0.0091603;
		start[24] = 0.0038337;
		start[25] = 0.0028985;
		start[26] = 0.0027543;
		start[27] = 0.0077864;
		start[28] = 0.0038784;
		start[29] = 0.0085199;
		start[30] = 0.0085177;
		start[31] = 0.0023325;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0028955;
		start[6] = 0.0015527;
		start[7] = 0.0098266;
		start[8] = 0.0083134;
		start[9] = 0.0059993;
		start[10] = 0.0041127;
		start[11] = 0.0017364;
		start[12] = 0.00072639;
		start[13] = 0.0096876;
		start[14] = 0.0028549;
		start[15] = 0.0082429;
		start[16] = 0.0086044;
		start[17] = 0.005335;
		start[18] = 0.0065271;
		start[19] = 0.0096201;
		start[20] = 0.003584;
		start[21] = 0.0081523;
		start[22] = 0.006785;
		start[23] = 0.0024181;
		start[24] = 0.00010109;
		start[25] = 0.0011704;
		start[26] = 0.0052854;
		start[27] = 0.0044584;
		start[28] = 0.002361;
		start[29] = 0.007878;
		start[30] = 0.0055572;
		start[31] = 0.0099131;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0048752;
		start[6] = 0.0039295;
		start[7] = 0.0079473;
		start[8] = 0.009811;
		start[9] = 0.0028528;
		start[10] = 0.0044777;
		start[11] = 0.008805;
		start[12] = 0.0079557;
		start[13] = 8.1425e-05;
		start[14] = 0.00037274;
		start[15] = 0.0054467;
		start[16] = 0.0052621;
		start[17] = 0.0023334;
		start[18] = 0.003748;
		start[19] = 0.0066085;
		start[20] = 0.0012447;
		start[21] = 0.0049066;
		start[22] = 0.00088523;
		start[23] = 3.893e-05;
		start[24] = 0.0022074;
		start[25] = 0.0064164;
		start[26] = 0.0056939;
		start[27] = 0.0056058;
		start[28] = 0.00031608;
		start[29] = 0.0062609;
		start[30] = 0.0064421;
		start[31] = 0.0054553;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0084608;
		start[6] = 0.0079874;
		start[7] = 0.0087697;
		start[8] = 0.0071124;
		start[9] = 0.0016738;
		start[10] = 0.0091293;
		start[11] = 0.0077001;
		start[12] = 0.0033501;
		start[13] = 0.0089112;
		start[14] = 0.0084213;
		start[15] = 0.002015;
		start[16] = 0.0076634;
		start[17] = 0.0053701;
		start[18] = 0.0038887;
		start[19] = 0.0010827;
		start[20] = 0.0018523;
		start[21] = 0.00046815;
		start[22] = 0.0045612;
		start[23] = 0.0093522;
		start[24] = 0.0053202;
		start[25] = 0.0002624;
		start[26] = 0.0062586;
		start[27] = 0.0065228;
		start[28] = 0.0059629;
		start[29] = 0.0050671;
		start[30] = 0.00068074;
		start[31] = 0.0014448;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.008781;
		start[6] = 0.0049475;
		start[7] = 0.0060324;
		start[8] = 0.0050515;
		start[9] = 0.0032424;
		start[10] = 0.0064065;
		start[11] = 0.00033786;
		start[12] = 0.00060309;
		start[13] = 0.0089444;
		start[14] = 0.0038684;
		start[15] = 0.0013167;
		start[16] = 0.0079343;
		start[17] = 0.0080952;
		start[18] = 0.0023914;
		start[19] = 0.0047427;
		start[20] = 0.0012748;
		start[21] = 0.0001243;
		start[22] = 0.0040891;
		start[23] = 0.0021284;
		start[24] = 0.0078729;
		start[25] = 8.1849e-05;
		start[26] = 0.008459;
		start[27] = 0.0082149;
		start[28] = 0.0070428;
		start[29] = 0.0076513;
		start[30] = 0.007669;
		start[31] = 0.0082063;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0060399;
		start[6] = 0.00083444;
		start[7] = 0.0051348;
		start[8] = 0.0053052;
		start[9] = 0.0054071;
		start[10] = 0.0037549;
		start[11] = 0.0079366;
		start[12] = 0.0034729;
		start[13] = 0.0051684;
		start[14] = 0.0030751;
		start[15] = 0.0099912;
		start[16] = 0.0054476;
		start[17] = 0.005033;
		start[18] = 0.00067621;
		start[19] = 0.0043701;
		start[20] = 0.0017304;
		start[21] = 0.00044621;
		start[22] = 0.0018639;
		start[23] = 0.003961;
		start[24] = 0.0015537;
		start[25] = 0.0067253;
		start[26] = 0.0018061;
		start[27] = 0.0087104;
		start[28] = 0.007679;
		start[29] = 0.0046762;
		start[30] = 0.0042043;
		start[31] = 0.0091312;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.003039;
		start[6] = 0.0046984;
		start[7] = 0.0084395;
		start[8] = 0.0018223;
		start[9] = 0.0037924;
		start[10] = 0.00068696;
		start[11] = 0.0087345;
		start[12] = 0.00094297;
		start[13] = 0.0052104;
		start[14] = 0.0052198;
		start[15] = 0.0053747;
		start[16] = 0.0049911;
		start[17] = 0.0055214;
		start[18] = 0.0019274;
		start[19] = 0.0041221;
		start[20] = 0.0020667;
		start[21] = 0.0017947;
		start[22] = 0.0066509;
		start[23] = 0.006042;
		start[24] = 0.00034411;
		start[25] = 0.0098371;
		start[26] = 0.002427;
		start[27] = 0.0026462;
		start[28] = 0.0078844;
		start[29] = 0.00036023;
		start[30] = 0.00087923;
		start[31] = 0.0078943;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.00062834;
		start[6] = 0.0026792;
		start[7] = 0.0026417;
		start[8] = 0.0020523;
		start[9] = 0.0032391;
		start[10] = 0.00023628;
		start[11] = 0.0021303;
		start[12] = 0.0096556;
		start[13] = 0.0011392;
		start[14] = 0.0023921;
		start[15] = 0.002227;
		start[16] = 0.0065184;
		start[17] = 0.009427;
		start[18] = 0.0087836;
		start[19] = 0.0012582;
		start[20] = 0.00069465;
		start[21] = 0.00039968;
		start[22] = 0.003512;
		start[23] = 0.0074916;
		start[24] = 0.0095506;
		start[25] = 0.004018;
		start[26] = 0.007004;
		start[27] = 0.0051439;
		start[28] = 0.0051023;
		start[29] = 0.00092961;
		start[30] = 0.0060177;
		start[31] = 0.00010641;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0089439;
		start[6] = 0.0005634;
		start[7] = 0.0018799;
		start[8] = 0.0023533;
		start[9] = 0.00064582;
		start[10] = 0.0015306;
		start[11] = 0.0038357;
		start[12] = 0.0018028;
		start[13] = 0.00036538;
		start[14] = 0.0016531;
		start[15] = 0.00079056;
		start[16] = 0.0081812;
		start[17] = 0.0018208;
		start[18] = 0.0084779;
		start[19] = 0.0023538;
		start[20] = 0.0025374;
		start[21] = 0.0084088;
		start[22] = 0.0024538;
		start[23] = 0.0020874;
		start[24] = 0.0077426;
		start[25] = 0.0030087;
		start[26] = 0.0078116;
		start[27] = 0.0089499;
		start[28] = 0.001871;
		start[29] = 0.0083606;
		start[30] = 0.0086839;
		start[31] = 0.0073406;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0041548;
		start[6] = 0.0047073;
		start[7] = 0.00641;
		start[8] = 0.0075566;
		start[9] = 0.0090765;
		start[10] = 0.0027481;
		start[11] = 0.0036532;
		start[12] = 0.0077404;
		start[13] = 0.0063702;
		start[14] = 0.00047895;
		start[15] = 0.0029218;
		start[16] = 0.0054906;
		start[17] = 0.0090377;
		start[18] = 0.0070632;
		start[19] = 0.0098599;
		start[20] = 0.003932;
		start[21] = 0.0056261;
		start[22] = 0.0028973;
		start[23] = 0.0023992;
		start[24] = 0.0071553;
		start[25] = 0.0051707;
		start[26] = 0.00025596;
		start[27] = 0.0090575;
		start[28] = 0.0021998;
		start[29] = 0.0026817;
		start[30] = 0.0080545;
		start[31] = 0.00063757;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0019512;
		start[6] = 0.0043013;
		start[7] = 0.0002598;
		start[8] = 0.006411;
		start[9] = 0.00025469;
		start[10] = 0.0057006;
		start[11] = 0.0032728;
		start[12] = 0.009832;
		start[13] = 0.0068926;
		start[14] = 0.0022566;
		start[15] = 0.0032177;
		start[16] = 0.0035905;
		start[17] = 0.0082885;
		start[18] = 0.0030076;
		start[19] = 0.0059647;
		start[20] = 0.0051735;
		start[21] = 0.0053772;
		start[22] = 0.0036313;
		start[23] = 0.0050851;
		start[24] = 0.0051625;
		start[25] = 0.0069054;
		start[26] = 0.009878;
		start[27] = 0.0073453;
		start[28] = 0.0070797;
		start[29] = 0.0062589;
		start[30] = 0.0044478;
		start[31] = 0.0042783;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.00488;
		start[6] = 0.0044738;
		start[7] = 0.0061942;
		start[8] = 0.0086368;
		start[9] = 0.0066322;
		start[10] = 0.0066411;
		start[11] = 0.0062884;
		start[12] = 0.00085603;
		start[13] = 0.0049457;
		start[14] = 0.0039997;
		start[15] = 0.0026535;
		start[16] = 0.0090517;
		start[17] = 0.0080124;
		start[18] = 0.0013154;
		start[19] = 0.0017976;
		start[20] = 3.0149e-05;
		start[21] = 0.0094703;
		start[22] = 0.0013334;
		start[23] = 0.0057777;
		start[24] = 0.0060455;
		start[25] = 0.0078676;
		start[26] = 0.0086314;
		start[27] = 0.0053738;
		start[28] = 0.0060048;
		start[29] = 0.0013438;
		start[30] = 0.0014549;
		start[31] = 0.0047026;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0072568;
		start[6] = 0.0074333;
		start[7] = 0.0034676;
		start[8] = 0.00058248;
		start[9] = 0.0041666;
		start[10] = 0.0047384;
		start[11] = 0.00056841;
		start[12] = 0.00628;
		start[13] = 0.0037839;
		start[14] = 0.0033053;
		start[15] = 0.0062482;
		start[16] = 0.0033196;
		start[17] = 0.0089997;
		start[18] = 0.0078959;
		start[19] = 0.00024629;
		start[20] = 0.0084749;
		start[21] = 0.0057759;
		start[22] = 0.00093041;
		start[23] = 0.0072457;
		start[24] = 0.0045653;
		start[25] = 0.0049523;
		start[26] = 0.0017871;
		start[27] = 0.0081866;
		start[28] = 0.0079561;
		start[29] = 0.00324;
		start[30] = 0.0029726;
		start[31] = 0.0075659;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0063741;
		start[6] = 0.0039808;
		start[7] = 0.0092485;
		start[8] = 0.0098276;
		start[9] = 0.0067596;
		start[10] = 0.0047358;
		start[11] = 0.0011074;
		start[12] = 0.0097142;
		start[13] = 0.0046476;
		start[14] = 0.0066752;
		start[15] = 0.00040496;
		start[16] = 0.0063411;
		start[17] = 0.0071041;
		start[18] = 0.0049503;
		start[19] = 0.007518;
		start[20] = 0.0035545;
		start[21] = 0.0056707;
		start[22] = 0.0050117;
		start[23] = 0.0077601;
		start[24] = 0.0065595;
		start[25] = 0.0075834;
		start[26] = 0.0079189;
		start[27] = 0.00016353;
		start[28] = 0.0072757;
		start[29] = 0.006277;
		start[30] = 0.00832;
		start[31] = 0.0016392;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.009827;
		start[6] = 0.0080825;
		start[7] = 0.0063028;
		start[8] = 0.0072363;
		start[9] = 0.0024265;
		start[10] = 0.0069451;
		start[11] = 0.0060749;
		start[12] = 0.00012743;
		start[13] = 0.0097652;
		start[14] = 0.0091329;
		start[15] = 0.0047996;
		start[16] = 0.0060952;
		start[17] = 0.0030162;
		start[18] = 0.0076252;
		start[19] = 0.0033226;
		start[20] = 0.0024326;
		start[21] = 0.0038261;
		start[22] = 0.00058972;
		start[23] = 0.0059201;
		start[24] = 0.0027079;
		start[25] = 0.001428;
		start[26] = 0.0065013;
		start[27] = 0.0091942;
		start[28] = 0.0092423;
		start[29] = 0.0032607;
		start[30] = 0.0093116;
		start[31] = 0.0042039;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0058898;
		start[6] = 0.002193;
		start[7] = 0.0098219;
		start[8] = 0.0098018;
		start[9] = 0.0024261;
		start[10] = 0.0076157;
		start[11] = 0.00040263;
		start[12] = 0.0057685;
		start[13] = 0.0070083;
		start[14] = 0.0053018;
		start[15] = 0.0048976;
		start[16] = 0.0023944;
		start[17] = 0.0038015;
		start[18] = 0.0021487;
		start[19] = 0.0057338;
		start[20] = 0.0097694;
		start[21] = 0.0013795;
		start[22] = 0.001087;
		start[23] = 0.005995;
		start[24] = 0.0046635;
		start[25] = 0.0090728;
		start[26] = 0.0068849;
		start[27] = 0.00016186;
		start[28] = 0.0018416;
		start[29] = 0.0079801;
		start[30] = 0.0089082;
		start[31] = 0.0028556;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0079875;
		start[6] = 0.0087619;
		start[7] = 0.0018921;
		start[8] = 0.0040037;
		start[9] = 0.0069888;
		start[10] = 0.0065082;
		start[11] = 0.0013412;
		start[12] = 0.00715;
		start[13] = 0.0090133;
		start[14] = 0.0064675;
		start[15] = 0.0014731;
		start[16] = 0.0032063;
		start[17] = 0.0048321;
		start[18] = 0.0078872;
		start[19] = 0.0095933;
		start[20] = 0.004773;
		start[21] = 0.0050647;
		start[22] = 0.005812;
		start[23] = 0.0075707;
		start[24] = 0.0078268;
		start[25] = 0.0022379;
		start[26] = 0.001575;
		start[27] = 0.0058625;
		start[28] = 0.0051904;
		start[29] = 0.008477;
		start[30] = 0.0055796;
		start[31] = 0.0025648;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0052378;
		start[6] = 0.0025461;
		start[7] = 0.0014056;
		start[8] = 0.00029917;
		start[9] = 0.0069138;
		start[10] = 0.0057254;
		start[11] = 0.0052958;
		start[12] = 0.0019446;
		start[13] = 0.0045317;
		start[14] = 0.0020943;
		start[15] = 0.004773;
		start[16] = 0.0072453;
		start[17] = 0.0003887;
		start[18] = 0.0062843;
		start[19] = 0.0078872;
		start[20] = 0.0096775;
		start[21] = 0.00013024;
		start[22] = 0.0090596;
		start[23] = 0.0092432;
		start[24] = 0.0024927;
		start[25] = 0.0096639;
		start[26] = 0.0051873;
		start[27] = 0.0024084;
		start[28] = 0.0022293;
		start[29] = 0.0045316;
		start[30] = 0.009703;
		start[31] = 0.0012957;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0016116;
		start[6] = 8.7068e-05;
		start[7] = 0.0066305;
		start[8] = 0.0014672;
		start[9] = 0.0020054;
		start[10] = 0.0027314;
		start[11] = 0.0021676;
		start[12] = 0.0058829;
		start[13] = 0.0014194;
		start[14] = 0.0030767;
		start[15] = 0.0049569;
		start[16] = 8.5977e-05;
		start[17] = 0.0054309;
		start[18] = 0.0039124;
		start[19] = 0.0055156;
		start[20] = 0.0065102;
		start[21] = 0.00071062;
		start[22] = 0.0081108;
		start[23] = 0.00071875;
		start[24] = 0.0010569;
		start[25] = 0.0098186;
		start[26] = 0.0058792;
		start[27] = 0.0032568;
		start[28] = 0.0099399;
		start[29] = 0.0042135;
		start[30] = 0.0026258;
		start[31] = 0.0085765;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0059793;
		start[6] = 2.1805e-05;
		start[7] = 0.0085647;
		start[8] = 8.9807e-05;
		start[9] = 0.0080609;
		start[10] = 0.00012781;
		start[11] = 0.005468;
		start[12] = 0.0039713;
		start[13] = 0.0028853;
		start[14] = 0.0051997;
		start[15] = 0.0045126;
		start[16] = 0.0090835;
		start[17] = 0.00047066;
		start[18] = 0.0035525;
		start[19] = 0.0020883;
		start[20] = 0.0018786;
		start[21] = 0.0020562;
		start[22] = 0.0011503;
		start[23] = 0.0034826;
		start[24] = 0.0062277;
		start[25] = 0.0020908;
		start[26] = 0.0044077;
		start[27] = 0.0031107;
		start[28] = 0.0089869;
		start[29] = 0.0086633;
		start[30] = 0.0007589;
		start[31] = 0.0041239;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0090808;
		start[6] = 0.00090897;
		start[7] = 0.0092859;
		start[8] = 0.0087927;
		start[9] = 0.0029982;
		start[10] = 5.8199e-05;
		start[11] = 0.009964;
		start[12] = 0.0058222;
		start[13] = 0.0027531;
		start[14] = 0.005693;
		start[15] = 0.0053469;
		start[16] = 0.0026532;
		start[17] = 0.0051803;
		start[18] = 0.0086604;
		start[19] = 0.0097146;
		start[20] = 0.0086842;
		start[21] = 0.00052712;
		start[22] = 0.0023335;
		start[23] = 0.0098309;
		start[24] = 0.0044412;
		start[25] = 0.0028588;
		start[26] = 0.0057707;
		start[27] = 0.0090272;
		start[28] = 0.0040362;
		start[29] = 0.004559;
		start[30] = 8.9232e-05;
		start[31] = 0.0098236;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0089912;
		start[6] = 0.0045285;
		start[7] = 2.1011e-05;
		start[8] = 0.0058146;
		start[9] = 0.0043531;
		start[10] = 0.0070455;
		start[11] = 0.0065107;
		start[12] = 0.0012492;
		start[13] = 0.0040607;
		start[14] = 0.00011529;
		start[15] = 0.0018552;
		start[16] = 0.0076649;
		start[17] = 0.0014012;
		start[18] = 0.0044714;
		start[19] = 0.0035839;
		start[20] = 0.0031457;
		start[21] = 0.0044758;
		start[22] = 0.0098374;
		start[23] = 0.007088;
		start[24] = 0.0099495;
		start[25] = 0.0013356;
		start[26] = 0.00016566;
		start[27] = 0.0074107;
		start[28] = 0.0018978;
		start[29] = 0.0035515;
		start[30] = 0.0028218;
		start[31] = 0.0041015;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0081688;
		start[6] = 0.0020825;
		start[7] = 0.0013278;
		start[8] = 0.0083721;
		start[9] = 0.0068111;
		start[10] = 0.002445;
		start[11] = 0.0064237;
		start[12] = 0.00062188;
		start[13] = 0.0071359;
		start[14] = 0.0099744;
		start[15] = 0.0071243;
		start[16] = 0.0095997;
		start[17] = 0.0078502;
		start[18] = 0.0024908;
		start[19] = 6.9446e-05;
		start[20] = 0.0054339;
		start[21] = 0.009975;
		start[22] = 0.0062155;
		start[23] = 0.0026527;
		start[24] = 0.0060694;
		start[25] = 0.0093421;
		start[26] = 0.0055769;
		start[27] = 0.0034393;
		start[28] = 0.0043623;
		start[29] = 0.0073858;
		start[30] = 0.0023441;
		start[31] = 0.0017742;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.0015724;
		start[6] = 0.0072791;
		start[7] = 0.0067601;
		start[8] = 0.0004807;
		start[9] = 0.0069747;
		start[10] = 0.00019062;
		start[11] = 0.0081125;
		start[12] = 0.0057831;
		start[13] = 0.0088309;
		start[14] = 0.001561;
		start[15] = 0.005443;
		start[16] = 0.0038733;
		start[17] = 0.0038949;
		start[18] = 0.0021628;
		start[19] = 0.004939;
		start[20] = 0.0071642;
		start[21] = 0.004364;
		start[22] = 0.0090941;
		start[23] = 0.0045309;
		start[24] = 0.0042872;
		start[25] = 0.0049732;
		start[26] = 0.0047298;
		start[27] = 0.0064925;
		start[28] = 0.0047369;
		start[29] = 0.0050021;
		start[30] = 0.006174;
		start[31] = 0.0073527;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0047472;
		start[6] = 0.0079107;
		start[7] = 0.0052202;
		start[8] = 0.0099221;
		start[9] = 0.0065341;
		start[10] = 0.0053547;
		start[11] = 0.0077494;
		start[12] = 0.0040343;
		start[13] = 0.00027977;
		start[14] = 0.0085918;
		start[15] = 0.0087213;
		start[16] = 0.0093045;
		start[17] = 0.0060253;
		start[18] = 0.0020101;
		start[19] = 0.0026167;
		start[20] = 0.0010548;
		start[21] = 0.00093313;
		start[22] = 0.0071657;
		start[23] = 0.0022701;
		start[24] = 0.0079189;
		start[25] = 0.0031605;
		start[26] = 0.0082839;
		start[27] = 0.0087078;
		start[28] = 0.0064894;
		start[29] = 0.0089278;
		start[30] = 0.006113;
		start[31] = 0.00062391;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0072636;
		start[6] = 0.0079495;
		start[7] = 0.0026002;
		start[8] = 0.0041319;
		start[9] = 0.0093986;
		start[10] = 0.0035791;
		start[11] = 0.00063738;
		start[12] = 0.0077271;
		start[13] = 0.0018582;
		start[14] = 0.0091153;
		start[15] = 0.0052056;
		start[16] = 0.0072085;
		start[17] = 0.0091286;
		start[18] = 0.0092637;
		start[19] = 0.0037537;
		start[20] = 0.0060423;
		start[21] = 0.0060243;
		start[22] = 0.0029903;
		start[23] = 0.0038037;
		start[24] = 0.0028299;
		start[25] = 0.0046529;
		start[26] = 0.0029016;
		start[27] = 0.0082906;
		start[28] = 0.00069519;
		start[29] = 0.0038197;
		start[30] = 0.0040612;
		start[31] = 0.004779;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0021601;
		start[6] = 0.0081839;
		start[7] = 0.0093461;
		start[8] = 0.0087087;
		start[9] = 0.0047803;
		start[10] = 0.0097511;
		start[11] = 0.0018533;
		start[12] = 0.0060654;
		start[13] = 0.0045754;
		start[14] = 0.0019558;
		start[15] = 0.0030479;
		start[16] = 0.0031785;
		start[17] = 0.0060523;
		start[18] = 0.0046535;
		start[19] = 0.0037095;
		start[20] = 0.0093866;
		start[21] = 0.0091245;
		start[22] = 0.0096682;
		start[23] = 0.0057514;
		start[24] = 0.0042859;
		start[25] = 5.5034e-05;
		start[26] = 0.00025253;
		start[27] = 0.0059674;
		start[28] = 0.0013187;
		start[29] = 0.0093719;
		start[30] = 0.0031048;
		start[31] = 0.0083126;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.00033373;
		start[6] = 0.0085976;
		start[7] = 0.0090511;
		start[8] = 0.0018913;
		start[9] = 0.0051477;
		start[10] = 0.0026958;
		start[11] = 0.0014494;
		start[12] = 0.0029888;
		start[13] = 0.0085772;
		start[14] = 0.0085766;
		start[15] = 0.001392;
		start[16] = 0.0029362;
		start[17] = 0.0086973;
		start[18] = 0.0053766;
		start[19] = 0.0019649;
		start[20] = 0.0084366;
		start[21] = 0.0032162;
		start[22] = 0.0088508;
		start[23] = 0.0054274;
		start[24] = 0.0047276;
		start[25] = 0.00084154;
		start[26] = 0.0010476;
		start[27] = 0.0084272;
		start[28] = 0.0088706;
		start[29] = 0.0048621;
		start[30] = 0.0048422;
		start[31] = 0.00057259;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.00060809;
		start[6] = 0.0019306;
		start[7] = 0.0035615;
		start[8] = 0.0093373;
		start[9] = 0.0057827;
		start[10] = 0.00055725;
		start[11] = 0.0025291;
		start[12] = 0.0075751;
		start[13] = 0.0024584;
		start[14] = 0.0098662;
		start[15] = 0.0035474;
		start[16] = 0.0091875;
		start[17] = 0.0095985;
		start[18] = 0.009517;
		start[19] = 0.0060374;
		start[20] = 0.0024279;
		start[21] = 0.0018389;
		start[22] = 0.0015923;
		start[23] = 0.0074542;
		start[24] = 0.0041104;
		start[25] = 0.0022307;
		start[26] = 0.0069259;
		start[27] = 0.0044862;
		start[28] = 0.0064418;
		start[29] = 0.0081741;
		start[30] = 0.0089664;
		start[31] = 0.0088614;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0032405;
		start[6] = 0.0020292;
		start[7] = 0.00054262;
		start[8] = 0.0085538;
		start[9] = 0.0082124;
		start[10] = 0.001132;
		start[11] = 0.0096388;
		start[12] = 0.0080198;
		start[13] = 0.000789;
		start[14] = 0.0070173;
		start[15] = 0.0080012;
		start[16] = 0.0031308;
		start[17] = 0.0062602;
		start[18] = 0.0011629;
		start[19] = 0.00082399;
		start[20] = 0.0020985;
		start[21] = 0.0015386;
		start[22] = 0.0087839;
		start[23] = 0.0039451;
		start[24] = 0.006814;
		start[25] = 0.006556;
		start[26] = 0.002805;
		start[27] = 0.0082768;
		start[28] = 0.0087396;
		start[29] = 0.0028907;
		start[30] = 0.004024;
		start[31] = 0.0081366;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.00049936;
		start[6] = 0.00096622;
		start[7] = 0.0031568;
		start[8] = 0.0090334;
		start[9] = 0.0057609;
		start[10] = 0.0017075;
		start[11] = 0.00080273;
		start[12] = 0.0054689;
		start[13] = 0.008335;
		start[14] = 0.0014248;
		start[15] = 0.0090291;
		start[16] = 0.0039167;
		start[17] = 0.0011024;
		start[18] = 0.003031;
		start[19] = 0.0093017;
		start[20] = 0.001012;
		start[21] = 0.007434;
		start[22] = 0.0042686;
		start[23] = 0.0029017;
		start[24] = 0.0026874;
		start[25] = 0.0099172;
		start[26] = 0.0031492;
		start[27] = 0.0025185;
		start[28] = 0.004448;
		start[29] = 0.0089002;
		start[30] = 0.0007682;
		start[31] = 0.0031567;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0091923;
		start[6] = 0.0027661;
		start[7] = 0.0062375;
		start[8] = 0.0049177;
		start[9] = 0.0050643;
		start[10] = 0.0032929;
		start[11] = 0.00015467;
		start[12] = 0.0061617;
		start[13] = 0.0098301;
		start[14] = 0.0060279;
		start[15] = 0.0051115;
		start[16] = 0.0049396;
		start[17] = 0.0020996;
		start[18] = 0.0063855;
		start[19] = 0.0073772;
		start[20] = 0.0025153;
		start[21] = 0.0065475;
		start[22] = 0.0059523;
		start[23] = 0.0053905;
		start[24] = 0.00086643;
		start[25] = 0.0098903;
		start[26] = 0.00078173;
		start[27] = 0.00088545;
		start[28] = 0.00015519;
		start[29] = 0.0095425;
		start[30] = 0.005481;
		start[31] = 0.0060595;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.00012758;
		start[6] = 0.0054913;
		start[7] = 0.0050587;
		start[8] = 0.0080306;
		start[9] = 0.0050913;
		start[10] = 0.007579;
		start[11] = 0.0097898;
		start[12] = 0.0091333;
		start[13] = 0.0032493;
		start[14] = 0.0054951;
		start[15] = 0.008508;
		start[16] = 0.0072376;
		start[17] = 0.0073299;
		start[18] = 0.0094141;
		start[19] = 0.0067324;
		start[20] = 0.0048838;
		start[21] = 0.0034315;
		start[22] = 0.0086833;
		start[23] = 0.0034195;
		start[24] = 0.0035053;
		start[25] = 0.0022387;
		start[26] = 0.00067887;
		start[27] = 0.009375;
		start[28] = 0.0030011;
		start[29] = 0.0092186;
		start[30] = 0.0022154;
		start[31] = 0.004511;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.002299;
		start[6] = 0.0047578;
		start[7] = 0.002714;
		start[8] = 0.0035374;
		start[9] = 0.0036365;
		start[10] = 0.0065707;
		start[11] = 0.0027965;
		start[12] = 0.0005998;
		start[13] = 0.0036899;
		start[14] = 0.0026249;
		start[15] = 0.0042057;
		start[16] = 0.0019997;
		start[17] = 0.0078183;
		start[18] = 0.001331;
		start[19] = 0.003422;
		start[20] = 0.0031392;
		start[21] = 0.008789;
		start[22] = 0.0054841;
		start[23] = 0.00056674;
		start[24] = 0.0081273;
		start[25] = 0.00659;
		start[26] = 0.0011719;
		start[27] = 0.0027945;
		start[28] = 0.0053071;
		start[29] = 0.0073227;
		start[30] = 0.0082636;
		start[31] = 0.0022942;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0092258;
		start[6] = 0.0036745;
		start[7] = 0.0003805;
		start[8] = 0.0064889;
		start[9] = 0.0081779;
		start[10] = 0.0042105;
		start[11] = 0.00805;
		start[12] = 0.0020687;
		start[13] = 0.0031117;
		start[14] = 0.0087261;
		start[15] = 0.0058674;
		start[16] = 0.0075181;
		start[17] = 0.0075501;
		start[18] = 0.00028024;
		start[19] = 0.0064225;
		start[20] = 0.0069289;
		start[21] = 0.0023218;
		start[22] = 0.0029048;
		start[23] = 0.0020561;
		start[24] = 0.0084316;
		start[25] = 0.00095302;
		start[26] = 0.0019071;
		start[27] = 0.0076308;
		start[28] = 0.0021486;
		start[29] = 0.00046862;
		start[30] = 0.0017862;
		start[31] = 0.0043714;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0070157;
		start[6] = 0.0043666;
		start[7] = 0.0033272;
		start[8] = 0.0048596;
		start[9] = 0.00010981;
		start[10] = 0.00053858;
		start[11] = 0.0035729;
		start[12] = 0.0076897;
		start[13] = 0.0061434;
		start[14] = 0.0058393;
		start[15] = 0.0070457;
		start[16] = 0.0051018;
		start[17] = 0.005763;
		start[18] = 0.0096564;
		start[19] = 0.0014744;
		start[20] = 0.0077542;
		start[21] = 0.006896;
		start[22] = 0.0011825;
		start[23] = 0.0075907;
		start[24] = 0.0033351;
		start[25] = 0.0067671;
		start[26] = 0.0039771;
		start[27] = 0.0013352;
		start[28] = 0.0061665;
		start[29] = 0.0079651;
		start[30] = 0.0058862;
		start[31] = 0.0056475;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0018087;
		start[6] = 0.0051153;
		start[7] = 0.0096475;
		start[8] = 0.0054728;
		start[9] = 0.0055436;
		start[10] = 0.0019381;
		start[11] = 0.0026954;
		start[12] = 0.0092951;
		start[13] = 0.0010301;
		start[14] = 0.001836;
		start[15] = 0.0024308;
		start[16] = 0.0074466;
		start[17] = 0.0024393;
		start[18] = 0.0041145;
		start[19] = 0.0026472;
		start[20] = 0.0047024;
		start[21] = 0.0038426;
		start[22] = 0.0020879;
		start[23] = 0.00077387;
		start[24] = 0.0094087;
		start[25] = 0.0052208;
		start[26] = 0.0028024;
		start[27] = 0.0041692;
		start[28] = 0.0026458;
		start[29] = 0.0052463;
		start[30] = 0.004058;
		start[31] = 0.005536;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0027071;
		start[6] = 0.0079231;
		start[7] = 0.00031891;
		start[8] = 0.0068063;
		start[9] = 0.0087259;
		start[10] = 0.0099179;
		start[11] = 0.0079676;
		start[12] = 0.0065067;
		start[13] = 0.005351;
		start[14] = 0.002874;
		start[15] = 0.00098423;
		start[16] = 0.0058989;
		start[17] = 0.0091432;
		start[18] = 0.0037121;
		start[19] = 0.0028415;
		start[20] = 0.0022273;
		start[21] = 0.00053135;
		start[22] = 0.0089823;
		start[23] = 0.0019941;
		start[24] = 0.0071427;
		start[25] = 0.0073136;
		start[26] = 0.0022893;
		start[27] = 0.0068837;
		start[28] = 0.0076208;
		start[29] = 0.0035005;
		start[30] = 0.0024558;
		start[31] = 0.002232;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0024681;
		start[6] = 0.0073086;
		start[7] = 0.0080436;
		start[8] = 0.0077682;
		start[9] = 0.0016241;
		start[10] = 0.0034717;
		start[11] = 0.000307;
		start[12] = 0.00034983;
		start[13] = 0.0048;
		start[14] = 0.00024055;
		start[15] = 0.0087685;
		start[16] = 0.0067836;
		start[17] = 0.0081931;
		start[18] = 0.0019592;
		start[19] = 0.0045701;
		start[20] = 0.0044897;
		start[21] = 0.0022887;
		start[22] = 0.0011086;
		start[23] = 0.0058454;
		start[24] = 0.005788;
		start[25] = 0.0074288;
		start[26] = 0.0077274;
		start[27] = 0.0052761;
		start[28] = 0.0035807;
		start[29] = 0.0033984;
		start[30] = 0.0090185;
		start[31] = 0.0067407;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0032575;
		start[6] = 0.0085942;
		start[7] = 0.0070351;
		start[8] = 0.0091944;
		start[9] = 0.0060782;
		start[10] = 0.0031745;
		start[11] = 0.0050854;
		start[12] = 0.0086266;
		start[13] = 7.817e-05;
		start[14] = 0.005874;
		start[15] = 0.0084879;
		start[16] = 0.0015073;
		start[17] = 0.0033544;
		start[18] = 0.0020792;
		start[19] = 0.0050636;
		start[20] = 0.0080796;
		start[21] = 0.00022712;
		start[22] = 0.0096506;
		start[23] = 0.0034654;
		start[24] = 0.0058691;
		start[25] = 0.0029392;
		start[26] = 0.001279;
		start[27] = 1.7224e-05;
		start[28] = 0.0018166;
		start[29] = 0.0027089;
		start[30] = 0.0012386;
		start[31] = 0.0073819;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0080912;
		start[6] = 0.0053393;
		start[7] = 0.0090109;
		start[8] = 0.00093279;
		start[9] = 0.0009312;
		start[10] = 0.005976;
		start[11] = 0.0077537;
		start[12] = 0.0085555;
		start[13] = 0.008229;
		start[14] = 0.0029465;
		start[15] = 0.0068568;
		start[16] = 0.0012532;
		start[17] = 0.0065232;
		start[18] = 0.0027282;
		start[19] = 0.0095671;
		start[20] = 0.0041529;
		start[21] = 0.0058269;
		start[22] = 0.0070157;
		start[23] = 0.0028771;
		start[24] = 0.0013418;
		start[25] = 0.0086167;
		start[26] = 0.0030741;
		start[27] = 0.0084043;
		start[28] = 0.0054271;
		start[29] = 0.001594;
		start[30] = 0.003959;
		start[31] = 0.00071992;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0014185;
		start[6] = 0.0062501;
		start[7] = 0.0004898;
		start[8] = 0.0010475;
		start[9] = 0.002018;
		start[10] = 0.0055531;
		start[11] = 0.0054839;
		start[12] = 0.0041383;
		start[13] = 0.0038276;
		start[14] = 0.0030459;
		start[15] = 0.009706;
		start[16] = 0.0083268;
		start[17] = 0.0076673;
		start[18] = 0.0017336;
		start[19] = 0.00016868;
		start[20] = 0.0017346;
		start[21] = 0.004374;
		start[22] = 0.0013238;
		start[23] = 0.0079541;
		start[24] = 0.0072488;
		start[25] = 0.0008874;
		start[26] = 0.00072569;
		start[27] = 0.00020095;
		start[28] = 0.004619;
		start[29] = 0.0012596;
		start[30] = 0.0075023;
		start[31] = 0.0064425;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0080914;
		start[6] = 0.0049512;
		start[7] = 0.0098213;
		start[8] = 0.0039272;
		start[9] = 0.0049882;
		start[10] = 0.0036991;
		start[11] = 0.00254;
		start[12] = 0.0031026;
		start[13] = 0.00808;
		start[14] = 0.0032544;
		start[15] = 0.0049166;
		start[16] = 0.0096624;
		start[17] = 7.6233e-05;
		start[18] = 0.0025714;
		start[19] = 0.0090609;
		start[20] = 0.0043283;
		start[21] = 0.0052896;
		start[22] = 0.0024038;
		start[23] = 0.0053544;
		start[24] = 0.0043073;
		start[25] = 0.0066506;
		start[26] = 0.0064288;
		start[27] = 0.0057043;
		start[28] = 0.0071045;
		start[29] = 0.0017559;
		start[30] = 0.00048367;
		start[31] = 0.0099343;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		start[5] = 0.008725;
		start[6] = 0.0074973;
		start[7] = 0.0081582;
		start[8] = 0.0084304;
		start[9] = 0.0088541;
		start[10] = 0.0068442;
		start[11] = 0.0040406;
		start[12] = 0.0096057;
		start[13] = 0.0079709;
		start[14] = 0.00681;
		start[15] = 0.0092688;
		start[16] = 0.0042823;
		start[17] = 0.004934;
		start[18] = 0.0045229;
		start[19] = 0.00011208;
		start[20] = 0.0094198;
		start[21] = 0.0078271;
		start[22] = 0.0096155;
		start[23] = 0.005169;
		start[24] = 0.0085681;
		start[25] = 0.0064331;
		start[26] = 0.0015943;
		start[27] = 0.0056346;
		start[28] = 0.0094551;
		start[29] = 0.0098526;
		start[30] = 0.0040847;
		start[31] = 0.0094996;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		start[5] = 0.0059618;
		start[6] = 0.0034294;
		start[7] = 0.0097314;
		start[8] = 0.0045191;
		start[9] = 0.0028705;
		start[10] = 0.0039339;
		start[11] = 0.0098363;
		start[12] = 0.0012162;
		start[13] = 0.0039759;
		start[14] = 0.0071836;
		start[15] = 0.0071962;
		start[16] = 0.0068282;
		start[17] = 0.0051809;
		start[18] = 0.00077991;
		start[19] = 0.0057171;
		start[20] = 0.0038038;
		start[21] = 0.0078833;
		start[22] = 0.0073479;
		start[23] = 0.00043055;
		start[24] = 0.0057204;
		start[25] = 0.0032397;
		start[26] = 0.0041866;
		start[27] = 0.0019203;
		start[28] = 0.0020031;
		start[29] = 0.0048254;
		start[30] = 0.0023939;
		start[31] = 0.0017014;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		start[5] = 0.0093374;
		start[6] = 0.0068096;
		start[7] = 0.0075849;
		start[8] = 0.00476;
		start[9] = 0.0094372;
		start[10] = 0.0067303;
		start[11] = 0.0047631;
		start[12] = 0.0017816;
		start[13] = 0.0066718;
		start[14] = 0.0041739;
		start[15] = 0.0013033;
		start[16] = 0.0068267;
		start[17] = 0.0056016;
		start[18] = 0.0089664;
		start[19] = 0.0026324;
		start[20] = 0.0058164;
		start[21] = 0.0057305;
		start[22] = 0.0006702;
		start[23] = 0.0067604;
		start[24] = 7.4122e-05;
		start[25] = 0.0057673;
		start[26] = 0.0076273;
		start[27] = 0.0035162;
		start[28] = 0.008264;
		start[29] = 0.0080285;
		start[30] = 0.001047;
		start[31] = 0.00064064;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		start[5] = 0.0020739;
		start[6] = 0.0026322;
		start[7] = 0.0048836;
		start[8] = 0.0076763;
		start[9] = 0.0098983;
		start[10] = 0.0035287;
		start[11] = 0.0038205;
		start[12] = 0.0097119;
		start[13] = 0.004636;
		start[14] = 0.0049703;
		start[15] = 0.001634;
		start[16] = 0.0005885;
		start[17] = 0.0049217;
		start[18] = 0.0016484;
		start[19] = 0.007908;
		start[20] = 0.0025129;
		start[21] = 0.001853;
		start[22] = 0.0030704;
		start[23] = 0.0087535;
		start[24] = 0.00092892;
		start[25] = 0.006367;
		start[26] = 0.0015946;
		start[27] = 0.00087502;
		start[28] = 0.0076186;
		start[29] = 0.0053377;
		start[30] = 0.0070812;
		start[31] = 0.0012086;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		start[5] = 0.0037906;
		start[6] = 0.0058606;
		start[7] = 0.0062129;
		start[8] = 0.0026183;
		start[9] = 0.0075289;
		start[10] = 0.009765;
		start[11] = 0.0035684;
		start[12] = 0.0087118;
		start[13] = 0.00081559;
		start[14] = 0.0087898;
		start[15] = 0.0075616;
		start[16] = 0.0040499;
		start[17] = 0.0061437;
		start[18] = 0.0067518;
		start[19] = 0.0085754;
		start[20] = 0.0074278;
		start[21] = 0.008519;
		start[22] = 0.0048953;
		start[23] = 0.00092524;
		start[24] = 0.0042451;
		start[25] = 0.0025012;
		start[26] = 0.006081;
		start[27] = 0.0085863;
		start[28] = 0.0070588;
		start[29] = 0.0065122;
		start[30] = 0.0078524;
		start[31] = 0.0041561;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		start[5] = 0.0011243;
		start[6] = 0.0080217;
		start[7] = 0.0034425;
		start[8] = 0.0070291;
		start[9] = 0.0064947;
		start[10] = 6.6896e-06;
		start[11] = 0.0026182;
		start[12] = 0.0023314;
		start[13] = 0.0082802;
		start[14] = 0.0051232;
		start[15] = 0.003769;
		start[16] = 0.0064523;
		start[17] = 0.001647;
		start[18] = 0.0057841;
		start[19] = 0.0057122;
		start[20] = 0.0043807;
		start[21] = 0.0026707;
		start[22] = 0.0069172;
		start[23] = 0.0097521;
		start[24] = 0.0090552;
		start[25] = 0.0067323;
		start[26] = 0.005867;
		start[27] = 0.0042023;
		start[28] = 0.0053767;
		start[29] = 0.0037674;
		start[30] = 0.0011842;
		start[31] = 0.0096979;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		start[5] = 0.0075682;
		start[6] = 0.005623;
		start[7] = 0.009194;
		start[8] = 0.0015416;
		start[9] = 0.0034166;
		start[10] = 0.004794;
		start[11] = 0.0027188;
		start[12] = 0.0087301;
		start[13] = 0.0042871;
		start[14] = 0.0075084;
		start[15] = 0.0088959;
		start[16] = 0.0023402;
		start[17] = 0.0080303;
		start[18] = 0.009614;
		start[19] = 0.0016307;
		start[20] = 0.0064883;
		start[21] = 0.0061633;
		start[22] = 0.002184;
		start[23] = 0.0098688;
		start[24] = 0.0069146;
		start[25] = 0.006339;
		start[26] = 0.0042637;
		start[27] = 0.0084042;
		start[28] = 0.0095703;
		start[29] = 0.0095642;
		start[30] = 0.0049296;
		start[31] = 0.0077651;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		start[5] = 0.0035132;
		start[6] = 0.0089226;
		start[7] = 0.0063431;
		start[8] = 0.0058407;
		start[9] = 0.0073366;
		start[10] = 0.0034946;
		start[11] = 0.0084054;
		start[12] = 0.00067788;
		start[13] = 0.0014329;
		start[14] = 0.0039702;
		start[15] = 0.00067879;
		start[16] = 0.0076937;
		start[17] = 0.0040305;
		start[18] = 0.0049281;
		start[19] = 0.0043307;
		start[20] = 0.006805;
		start[21] = 0.0014966;
		start[22] = 0.0061718;
		start[23] = 0.0064686;
		start[24] = 0.0092697;
		start[25] = 0.00065885;
		start[26] = 0.0094818;
		start[27] = 0.00048426;
		start[28] = 0.0017045;
		start[29] = 0.0053148;
		start[30] = 0.0014135;
		start[31] = 0.002556;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		start[5] = 0.0089097;
		start[6] = 0.0071982;
		start[7] = 0.0039868;
		start[8] = 0.00040507;
		start[9] = 0.0097468;
		start[10] = 0.0068453;
		start[11] = 0.004445;
		start[12] = 0.008121;
		start[13] = 0.0091049;
		start[14] = 0.0020357;
		start[15] = 0.0058386;
		start[16] = 0.0095741;
		start[17] = 0.002747;
		start[18] = 0.0035485;
		start[19] = 0.0093963;
		start[20] = 0.0055774;
		start[21] = 0.006306;
		start[22] = 0.0082332;
		start[23] = 0.0089488;
		start[24] = 0.0039867;
		start[25] = 0.0045744;
		start[26] = 0.0047848;
		start[27] = 0.0040014;
		start[28] = 0.0084402;
		start[29] = 0.0034081;
		start[30] = 0.0044532;
		start[31] = 0.0050639;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (32);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		start[5] = 0.0034728;
		start[6] = 0.0098292;
		start[7] = 0.0067787;
		start[8] = 0.001894;
		start[9] = 0.0024542;
		start[10] = 0.0009709;
		start[11] = 0.006439;
		start[12] = 0.009731;
		start[13] = 0.00675;
		start[14] = 0.0054679;
		start[15] = 0.005916;
		start[16] = 0.0070258;
		start[17] = 0.0011607;
		start[18] = 0.00096961;
		start[19] = 0.0062486;
		start[20] = 0.0088577;
		start[21] = 0.0013702;
		start[22] = 0.006548;
		start[23] = 0.0081234;
		start[24] = 0.0055001;
		start[25] = 0.0041369;
		start[26] = 0.0027808;
		start[27] = 0.006737;
		start[28] = 0.006074;
		start[29] = 0.006284;
		start[30] = 0.0014871;
		start[31] = 0.00054996;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}


  return 0;
}
