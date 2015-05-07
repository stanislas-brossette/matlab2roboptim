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
    (22, 1, "CostFunction_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

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
    (22, 1, "LiftConstraint_1_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = pow(w_01_01,2) + pow(w_01_04,2) - 1.0;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 2.0*w_01_01; 
			 grad[1] = 0.0; 
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
    (22, 1, "LiftConstraint_2_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = pow(w_01_02,2) + pow(w_01_03,2) - 1.0;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 2.0*w_01_02; 
			 grad[2] = 2.0*w_01_03; 
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
    (22, 1, "LiftConstraint_3_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = pow(w_01_05,2) + pow(w_01_06,2) - 1.0;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 2.0*w_01_05; 
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
    (22, 1, "LiftConstraint_4_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

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
    (22, 1, "LiftConstraint_5_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_01*w_01_07 - 1.0*w_02_01;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = w_01_07; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = w_01_01; 
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
    (22, 1, "LiftConstraint_6_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_04*w_01_07 - 1.0*w_02_02;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_01_07; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = w_01_04; 
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
    (22, 1, "LiftConstraint_7_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_01*w_01_08 - 1.0*w_02_03;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = w_01_08; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = w_01_01; 
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
    (22, 1, "LiftConstraint_8_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_04*w_01_08 - 1.0*w_02_04;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_01_08; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = w_01_04; 
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
    (22, 1, "LiftConstraint_9_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_05*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_01;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

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
			 grad[8] = w_01_05; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = -1.0*w_01_05; 
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
    (22, 1, "LiftConstraint_10_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = - 1.0*w_03_02 - 1.0*w_01_06*(w_02_02 + w_02_03);
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = -1.0*w_01_06; 
			 grad[10] = -1.0*w_01_06; 
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
    (22, 1, "LiftConstraint_11_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_05*(w_02_02 + w_02_03) - 1.0*w_03_03;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

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
			 grad[9] = w_01_05; 
			 grad[10] = w_01_05; 
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
    (22, 1, "LiftConstraint_12_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_06*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_04;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_02_01 - 1.0*w_02_04; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = w_01_06; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = -1.0*w_01_06; 
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
    (22, 1, "LiftConstraint_13_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = - 1.0*w_03_05 - 1.0*w_01_05*(w_02_02 + w_02_03);
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

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
			 grad[9] = -1.0*w_01_05; 
			 grad[10] = -1.0*w_01_05; 
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
    (22, 1, "LiftConstraint_14_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_06*(w_02_02 + w_02_03) - 1.0*w_03_06;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_02_02 + w_02_03; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = w_01_06; 
			 grad[10] = w_01_06; 
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
    (22, 1, "LiftConstraint_15_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_02*(w_03_01 + w_03_02) - 1.0*w_04_01;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = w_03_01 + w_03_02; 
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
			 grad[12] = w_01_02; 
			 grad[13] = w_01_02; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = -1.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
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
    (22, 1, "LiftConstraint_16_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = - 1.0*w_04_02 - 1.0*w_01_03*(w_03_04 - 1.0*w_03_05);
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = w_03_05 - 1.0*w_03_04; 
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
			 grad[15] = -1.0*w_01_03; 
			 grad[16] = w_01_03; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = -1.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
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
    (22, 1, "LiftConstraint_17_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_02*(w_03_03 + w_03_04) - 1.0*w_04_03;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = w_03_03 + w_03_04; 
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
			 grad[14] = w_01_02; 
			 grad[15] = w_01_02; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = -1.0; 
			 grad[21] = 0.0; 
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
    (22, 1, "LiftConstraint_18_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_03*(w_03_01 - 1.0*w_03_06) - 1.0*w_04_04;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = w_03_01 - 1.0*w_03_06; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = w_01_03; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = -1.0*w_01_03; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = -1.0; 
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
    (22, 1, "EEConstraint_1_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_01 - 1.0*EE_1_1 + w_02_01 - 1.0*w_02_04 + w_03_01 + w_03_02 + w_04_01 + w_04_02;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 1.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 1.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = -1.0; 
			 grad[12] = 1.0; 
			 grad[13] = 1.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 1.0; 
			 grad[19] = 1.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
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
    (22, 1, "EEConstraint_2_planarRobot4"),
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];
  
	result[0] = w_01_04 - 1.0*EE_1_2 + w_02_02 + w_02_03 + w_03_03 + w_03_04 + w_04_03 + w_04_04;
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
	const double& w_02_01 = x[8];
	const double& w_02_02 = x[9];
	const double& w_02_03 = x[10];
	const double& w_02_04 = x[11];
	const double& w_03_01 = x[12];
	const double& w_03_02 = x[13];
	const double& w_03_03 = x[14];
	const double& w_03_04 = x[15];
	const double& w_03_05 = x[16];
	const double& w_03_06 = x[17];
	const double& w_04_01 = x[18];
	const double& w_04_02 = x[19];
	const double& w_04_03 = x[20];
	const double& w_04_04 = x[21];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 1.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 1.0; 
			 grad[10] = 1.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 1.0; 
			 grad[15] = 1.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 1.0; 
			 grad[21] = 1.0; 
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
	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_19 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_20 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);

  //Create problem
  solver_t::problem_t pb (*cost);

  // Set bounds for all optimization parameters
  pb.argumentBounds ()[0] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[1] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[2] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[3] = roboptim::Function::makeInterval (-pi, pi);

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
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_19), bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_20), bounds, scales); 
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
		std::vector<double> start (22);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.006849;
		start[5] = 0.0090003;
		start[6] = 0.0032034;
		start[7] = 0.0030078;
		start[8] = 0.00094173;
		start[9] = 0.0020356;
		start[10] = 0.0097015;
		start[11] = 0.0064338;
		start[12] = 0.004096;
		start[13] = 0.002264;
		start[14] = 0.0061952;
		start[15] = 0.0085808;
		start[16] = 0.0089806;
		start[17] = 0.0060175;
		start[18] = 0.0066359;
		start[19] = 0.004452;
		start[20] = 0.0014531;
		start[21] = 0.0057937;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0099143;
		start[5] = 0.0036285;
		start[6] = 0.0052405;
		start[7] = 0.0035147;
		start[8] = 0.00013548;
		start[9] = 0.0043279;
		start[10] = 0.0024782;
		start[11] = 0.0066058;
		start[12] = 0.0039891;
		start[13] = 0.009133;
		start[14] = 0.0041227;
		start[15] = 0.0041934;
		start[16] = 0.0014705;
		start[17] = 0.0049947;
		start[18] = 0.0038568;
		start[19] = 0.0030216;
		start[20] = 0.007776;
		start[21] = 0.0012466;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0053865;
		start[5] = 0.0085177;
		start[6] = 0.0037065;
		start[7] = 0.0035238;
		start[8] = 0.0096518;
		start[9] = 0.0031056;
		start[10] = 0.0022398;
		start[11] = 0.00939;
		start[12] = 0.0066238;
		start[13] = 0.0040584;
		start[14] = 0.0040482;
		start[15] = 0.0048957;
		start[16] = 0.0033203;
		start[17] = 0.0096055;
		start[18] = 0.0087692;
		start[19] = 0.0050417;
		start[20] = 0.001072;
		start[21] = 0.0056837;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0038112;
		start[5] = 0.0060029;
		start[6] = 0.00051418;
		start[7] = 0.0069764;
		start[8] = 0.0077521;
		start[9] = 0.0081776;
		start[10] = 0.0088802;
		start[11] = 0.0068105;
		start[12] = 0.0090189;
		start[13] = 0.0086364;
		start[14] = 0.0064922;
		start[15] = 0.0040241;
		start[16] = 0.005339;
		start[17] = 0.00035308;
		start[18] = 0.0012168;
		start[19] = 0.0056717;
		start[20] = 0.0098984;
		start[21] = 0.0035881;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0092061;
		start[5] = 0.0046681;
		start[6] = 0.004516;
		start[7] = 0.0095785;
		start[8] = 0.006275;
		start[9] = 0.0085482;
		start[10] = 0.0033438;
		start[11] = 0.0054597;
		start[12] = 0.006239;
		start[13] = 0.008159;
		start[14] = 0.0042691;
		start[15] = 0.00045535;
		start[16] = 0.0079294;
		start[17] = 0.007838;
		start[18] = 0.0067914;
		start[19] = 0.0095066;
		start[20] = 0.0039417;
		start[21] = 0.0012755;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0065306;
		start[5] = 0.008912;
		start[6] = 0.0016111;
		start[7] = 0.0053185;
		start[8] = 0.0046459;
		start[9] = 0.0027749;
		start[10] = 0.0080447;
		start[11] = 0.00424;
		start[12] = 0.00069693;
		start[13] = 0.0014565;
		start[14] = 0.0052409;
		start[15] = 0.0028592;
		start[16] = 0.0045026;
		start[17] = 0.0079262;
		start[18] = 0.0082556;
		start[19] = 0.005773;
		start[20] = 0.0078205;
		start[21] = 0.003556;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.00032739;
		start[5] = 0.0085673;
		start[6] = 0.0033966;
		start[7] = 0.0039575;
		start[8] = 0.0077174;
		start[9] = 0.009448;
		start[10] = 0.0012577;
		start[11] = 0.0065991;
		start[12] = 0.00076321;
		start[13] = 0.0043089;
		start[14] = 0.0094235;
		start[15] = 0.0060357;
		start[16] = 0.00062337;
		start[17] = 0.0026074;
		start[18] = 0.0070492;
		start[19] = 0.0094898;
		start[20] = 0.0032532;
		start[21] = 0.0041422;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0068034;
		start[5] = 7.2742e-05;
		start[6] = 0.0043439;
		start[7] = 0.0060624;
		start[8] = 0.0088905;
		start[9] = 0.0058887;
		start[10] = 0.0075088;
		start[11] = 0.0041193;
		start[12] = 0.0086876;
		start[13] = 0.0065091;
		start[14] = 0.0058255;
		start[15] = 0.00039382;
		start[16] = 0.006271;
		start[17] = 0.0040019;
		start[18] = 0.0021419;
		start[19] = 0.0046319;
		start[20] = 0.0068652;
		start[21] = 0.0026989;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0069455;
		start[5] = 0.009455;
		start[6] = 0.0045104;
		start[7] = 0.0027213;
		start[8] = 0.0078901;
		start[9] = 0.0041951;
		start[10] = 0.0061115;
		start[11] = 0.0017998;
		start[12] = 0.0024685;
		start[13] = 0.0088728;
		start[14] = 0.0013564;
		start[15] = 0.002845;
		start[16] = 0.0023394;
		start[17] = 0.002662;
		start[18] = 0.00018369;
		start[19] = 0.0010568;
		start[20] = 0.006444;
		start[21] = 0.0069652;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0099171;
		start[5] = 0.0086566;
		start[6] = 0.00038371;
		start[7] = 0.0070058;
		start[8] = 0.00099898;
		start[9] = 0.0080887;
		start[10] = 0.00070443;
		start[11] = 0.0064644;
		start[12] = 0.0065109;
		start[13] = 0.0070316;
		start[14] = 0.0047366;
		start[15] = 0.0011751;
		start[16] = 0.007948;
		start[17] = 0.008128;
		start[18] = 0.0063356;
		start[19] = 0.0032299;
		start[20] = 0.0050403;
		start[21] = 0.0013787;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0040173;
		start[5] = 0.0060156;
		start[6] = 0.008696;
		start[7] = 0.00063054;
		start[8] = 0.006875;
		start[9] = 0.007326;
		start[10] = 0.0011258;
		start[11] = 0.0092563;
		start[12] = 0.00079306;
		start[13] = 0.00091116;
		start[14] = 0.0079598;
		start[15] = 0.0011612;
		start[16] = 5.0879e-05;
		start[17] = 0.00016963;
		start[18] = 0.0056912;
		start[19] = 0.0096013;
		start[20] = 0.0012373;
		start[21] = 0.0009423;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0065417;
		start[5] = 0.0021696;
		start[6] = 0.0024506;
		start[7] = 0.0018108;
		start[8] = 0.0090114;
		start[9] = 0.0043382;
		start[10] = 0.0045656;
		start[11] = 0.0018877;
		start[12] = 0.0093052;
		start[13] = 0.0079507;
		start[14] = 0.0017957;
		start[15] = 0.0021607;
		start[16] = 0.0079253;
		start[17] = 0.0050439;
		start[18] = 0.0031699;
		start[19] = 0.00074101;
		start[20] = 0.0074062;
		start[21] = 0.0095959;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.00028709;
		start[5] = 0.0020244;
		start[6] = 0.0016449;
		start[7] = 0.0020533;
		start[8] = 0.0031304;
		start[9] = 0.0086012;
		start[10] = 0.0050885;
		start[11] = 0.0059023;
		start[12] = 0.0057811;
		start[13] = 0.0046547;
		start[14] = 0.00070027;
		start[15] = 0.001985;
		start[16] = 0.0040013;
		start[17] = 0.005607;
		start[18] = 0.0043087;
		start[19] = 0.0046522;
		start[20] = 0.0091913;
		start[21] = 0.0045998;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0021834;
		start[5] = 0.0027038;
		start[6] = 9.4289e-05;
		start[7] = 0.0082137;
		start[8] = 0.0092995;
		start[9] = 0.0049714;
		start[10] = 0.0065827;
		start[11] = 0.0030221;
		start[12] = 0.00147;
		start[13] = 0.0023324;
		start[14] = 0.0055395;
		start[15] = 0.0073802;
		start[16] = 0.0017071;
		start[17] = 0.0021733;
		start[18] = 0.0044203;
		start[19] = 0.0062808;
		start[20] = 0.0023374;
		start[21] = 0.00084456;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0035285;
		start[5] = 0.0013314;
		start[6] = 0.004638;
		start[7] = 0.00094997;
		start[8] = 0.0071426;
		start[9] = 0.0036743;
		start[10] = 0.0076305;
		start[11] = 0.0047552;
		start[12] = 0.0056774;
		start[13] = 0.0028074;
		start[14] = 0.0015059;
		start[15] = 0.002209;
		start[16] = 0.0066855;
		start[17] = 0.0025542;
		start[18] = 0.0010223;
		start[19] = 0.0067149;
		start[20] = 0.0018957;
		start[21] = 0.003647;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0054355;
		start[5] = 0.0010017;
		start[6] = 0.0054495;
		start[7] = 0.0051484;
		start[8] = 0.006153;
		start[9] = 0.0015903;
		start[10] = 0.0027192;
		start[11] = 0.0079391;
		start[12] = 0.004361;
		start[13] = 0.0013376;
		start[14] = 0.0027285;
		start[15] = 0.0065701;
		start[16] = 0.0050237;
		start[17] = 0.0038758;
		start[18] = 0.0037226;
		start[19] = 0.0087016;
		start[20] = 0.0064449;
		start[21] = 0.0045446;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.00013881;
		start[5] = 0.0024959;
		start[6] = 0.0065502;
		start[7] = 0.0050255;
		start[8] = 0.0041365;
		start[9] = 0.0076493;
		start[10] = 0.0025108;
		start[11] = 0.0025109;
		start[12] = 0.0049472;
		start[13] = 0.0097946;
		start[14] = 0.0072843;
		start[15] = 0.0013798;
		start[16] = 0.0061993;
		start[17] = 0.0074105;
		start[18] = 0.0034573;
		start[19] = 0.0030925;
		start[20] = 0.0022496;
		start[21] = 0.0017566;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.00032526;
		start[5] = 0.0012834;
		start[6] = 0.0080567;
		start[7] = 0.0014699;
		start[8] = 0.0097432;
		start[9] = 0.0019207;
		start[10] = 0.005063;
		start[11] = 0.0022144;
		start[12] = 0.0049057;
		start[13] = 0.0042919;
		start[14] = 6.0082e-05;
		start[15] = 0.0019593;
		start[16] = 0.0075873;
		start[17] = 0.0031921;
		start[18] = 0.0035228;
		start[19] = 0.0067201;
		start[20] = 0.006003;
		start[21] = 0.004124;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0079984;
		start[5] = 0.0068934;
		start[6] = 0.0096609;
		start[7] = 0.008888;
		start[8] = 0.0061187;
		start[9] = 0.0096491;
		start[10] = 0.0046847;
		start[11] = 0.0059231;
		start[12] = 0.0038213;
		start[13] = 0.0080225;
		start[14] = 0.0041326;
		start[15] = 0.0026835;
		start[16] = 0.0025829;
		start[17] = 0.0077483;
		start[18] = 0.0070712;
		start[19] = 0.0043249;
		start[20] = 0.0036519;
		start[21] = 0.0087091;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.007469;
		start[5] = 0.0029357;
		start[6] = 0.0026743;
		start[7] = 0.0047466;
		start[8] = 0.0018758;
		start[9] = 0.0046447;
		start[10] = 0.0093209;
		start[11] = 0.005228;
		start[12] = 0.0058956;
		start[13] = 0.0086948;
		start[14] = 0.0043815;
		start[15] = 0.0032297;
		start[16] = 0.0075518;
		start[17] = 0.008397;
		start[18] = 0.005781;
		start[19] = 0.0066656;
		start[20] = 0.0045547;
		start[21] = 0.0012122;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0047998;
		start[5] = 0.0055105;
		start[6] = 0.0051977;
		start[7] = 0.0055288;
		start[8] = 0.0030062;
		start[9] = 0.0038202;
		start[10] = 0.0038658;
		start[11] = 0.0029304;
		start[12] = 8.4945e-05;
		start[13] = 0.002316;
		start[14] = 0.0018468;
		start[15] = 0.007573;
		start[16] = 0.0079476;
		start[17] = 0.0031146;
		start[18] = 0.0038873;
		start[19] = 0.002339;
		start[20] = 0.0045526;
		start[21] = 0.0093903;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0081846;
		start[5] = 0.0040871;
		start[6] = 0.0071609;
		start[7] = 0.0003169;
		start[8] = 0.0083384;
		start[9] = 0.0092884;
		start[10] = 0.00037862;
		start[11] = 0.0044465;
		start[12] = 0.0081025;
		start[13] = 0.000725;
		start[14] = 0.0033447;
		start[15] = 0.0020768;
		start[16] = 0.0088258;
		start[17] = 0.00758;
		start[18] = 0.0028323;
		start[19] = 0.0047934;
		start[20] = 0.0074816;
		start[21] = 0.0032701;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0029661;
		start[5] = 0.0085184;
		start[6] = 0.0056319;
		start[7] = 0.0037657;
		start[8] = 0.00092562;
		start[9] = 0.0047312;
		start[10] = 0.0067212;
		start[11] = 0.004056;
		start[12] = 0.0064983;
		start[13] = 0.0096857;
		start[14] = 0.0017065;
		start[15] = 0.0012645;
		start[16] = 0.0091505;
		start[17] = 0.0072302;
		start[18] = 0.0066493;
		start[19] = 0.0083099;
		start[20] = 0.0061178;
		start[21] = 0.0028582;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.00049035;
		start[5] = 0.0086303;
		start[6] = 0.00060943;
		start[7] = 0.0048748;
		start[8] = 0.0063397;
		start[9] = 0.0071836;
		start[10] = 0.0035221;
		start[11] = 0.0071324;
		start[12] = 0.0053034;
		start[13] = 0.0018867;
		start[14] = 0.0012019;
		start[15] = 0.0064504;
		start[16] = 0.0013351;
		start[17] = 0.0078247;
		start[18] = 0.0018749;
		start[19] = 0.0055052;
		start[20] = 0.0062414;
		start[21] = 0.0069542;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0086153;
		start[5] = 0.0055069;
		start[6] = 0.0092111;
		start[7] = 0.00066643;
		start[8] = 0.0096329;
		start[9] = 0.0086486;
		start[10] = 0.00048364;
		start[11] = 0.0065933;
		start[12] = 0.0090341;
		start[13] = 0.0020248;
		start[14] = 0.0012902;
		start[15] = 0.0062396;
		start[16] = 0.00024083;
		start[17] = 0.0033564;
		start[18] = 0.0091306;
		start[19] = 0.00087178;
		start[20] = 0.0055248;
		start[21] = 0.0085896;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0041166;
		start[5] = 0.0041006;
		start[6] = 0.0065822;
		start[7] = 0.0032451;
		start[8] = 0.0015127;
		start[9] = 0.004431;
		start[10] = 0.0097646;
		start[11] = 0.0083901;
		start[12] = 0.0026948;
		start[13] = 0.002712;
		start[14] = 0.0053097;
		start[15] = 0.0069326;
		start[16] = 0.0041995;
		start[17] = 0.0060473;
		start[18] = 0.0060199;
		start[19] = 0.008468;
		start[20] = 0.002715;
		start[21] = 0.0038262;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0085788;
		start[5] = 0.0020733;
		start[6] = 0.001375;
		start[7] = 0.0086662;
		start[8] = 0.0077495;
		start[9] = 0.0076332;
		start[10] = 0.0059477;
		start[11] = 0.0031631;
		start[12] = 0.0063504;
		start[13] = 0.0074861;
		start[14] = 0.0088013;
		start[15] = 0.0038343;
		start[16] = 0.00099847;
		start[17] = 0.0074341;
		start[18] = 0.0017081;
		start[19] = 0.0076634;
		start[20] = 0.0068015;
		start[21] = 0.0021857;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0053502;
		start[5] = 0.00613;
		start[6] = 0.0086239;
		start[7] = 0.00027797;
		start[8] = 0.0056577;
		start[9] = 0.0009183;
		start[10] = 0.0034431;
		start[11] = 0.0076641;
		start[12] = 0.0055693;
		start[13] = 0.0014686;
		start[14] = 0.0088356;
		start[15] = 0.00068198;
		start[16] = 0.0033134;
		start[17] = 0.00026604;
		start[18] = 0.0028064;
		start[19] = 0.0020662;
		start[20] = 0.007711;
		start[21] = 0.00093583;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0021565;
		start[5] = 0.00087544;
		start[6] = 0.0078471;
		start[7] = 0.0059105;
		start[8] = 0.0032074;
		start[9] = 0.0062691;
		start[10] = 0.0090335;
		start[11] = 0.0025889;
		start[12] = 0.0039228;
		start[13] = 0.001967;
		start[14] = 0.008196;
		start[15] = 0.00056821;
		start[16] = 0.0045387;
		start[17] = 0.0024761;
		start[18] = 0.0037158;
		start[19] = 0.0015808;
		start[20] = 0.0058892;
		start[21] = 0.0055594;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0014386;
		start[5] = 0.0097944;
		start[6] = 0.0073711;
		start[7] = 0.0010697;
		start[8] = 0.00091773;
		start[9] = 0.0052736;
		start[10] = 0.0032304;
		start[11] = 0.0047981;
		start[12] = 0.0010522;
		start[13] = 0.0097611;
		start[14] = 0.0076766;
		start[15] = 0.0088727;
		start[16] = 0.00672;
		start[17] = 0.0082183;
		start[18] = 0.0021902;
		start[19] = 0.0080896;
		start[20] = 0.0025797;
		start[21] = 0.0074818;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0039435;
		start[5] = 0.00011162;
		start[6] = 0.0016295;
		start[7] = 0.0017103;
		start[8] = 0.0046585;
		start[9] = 0.0037905;
		start[10] = 0.0020888;
		start[11] = 0.0013743;
		start[12] = 0.0068886;
		start[13] = 0.0066301;
		start[14] = 0.0022617;
		start[15] = 0.0034877;
		start[16] = 0.0074151;
		start[17] = 0.0010176;
		start[18] = 0.0081533;
		start[19] = 0.0038913;
		start[20] = 0.0030367;
		start[21] = 0.0085323;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0057083;
		start[5] = 0.0035056;
		start[6] = 0.0052058;
		start[7] = 0.0031595;
		start[8] = 0.0049166;
		start[9] = 0.004441;
		start[10] = 0.0079141;
		start[11] = 0.0054685;
		start[12] = 0.0041456;
		start[13] = 0.0022984;
		start[14] = 0.0021381;
		start[15] = 0.0080604;
		start[16] = 0.0094214;
		start[17] = 0.0025004;
		start[18] = 0.0084099;
		start[19] = 0.00015769;
		start[20] = 0.0013795;
		start[21] = 0.0084646;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0031369;
		start[5] = 0.0055065;
		start[6] = 0.0080424;
		start[7] = 0.00018622;
		start[8] = 0.0031512;
		start[9] = 0.0043834;
		start[10] = 5.362e-05;
		start[11] = 0.0045093;
		start[12] = 0.005569;
		start[13] = 0.0024423;
		start[14] = 0.0058182;
		start[15] = 0.0062543;
		start[16] = 0.0037042;
		start[17] = 0.00094722;
		start[18] = 0.0074224;
		start[19] = 0.0070172;
		start[20] = 0.003609;
		start[21] = 0.0076297;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0071078;
		start[5] = 0.0059895;
		start[6] = 0.0031914;
		start[7] = 0.005616;
		start[8] = 0.0021587;
		start[9] = 0.0015755;
		start[10] = 0.0052753;
		start[11] = 0.0030927;
		start[12] = 0.006075;
		start[13] = 0.0048127;
		start[14] = 0.0056584;
		start[15] = 0.0014126;
		start[16] = 0.004695;
		start[17] = 0.004071;
		start[18] = 0.0044653;
		start[19] = 0.0069394;
		start[20] = 0.0024585;
		start[21] = 0.0023289;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0012662;
		start[5] = 0.0055106;
		start[6] = 0.0054778;
		start[7] = 0.00097336;
		start[8] = 0.0032063;
		start[9] = 0.0017068;
		start[10] = 0.008535;
		start[11] = 0.0035447;
		start[12] = 0.0053066;
		start[13] = 0.0034034;
		start[14] = 0.0089334;
		start[15] = 0.0011611;
		start[16] = 0.0083836;
		start[17] = 0.0027602;
		start[18] = 0.0024296;
		start[19] = 0.003482;
		start[20] = 0.0016231;
		start[21] = 0.0081115;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.00068584;
		start[5] = 0.0073523;
		start[6] = 0.0075939;
		start[7] = 0.00062837;
		start[8] = 0.0079694;
		start[9] = 0.00095009;
		start[10] = 0.00029975;
		start[11] = 0.0090161;
		start[12] = 0.0073867;
		start[13] = 0.0009622;
		start[14] = 0.0058628;
		start[15] = 0.0037512;
		start[16] = 0.0077522;
		start[17] = 0.0082934;
		start[18] = 0.008531;
		start[19] = 0.0046539;
		start[20] = 0.0043622;
		start[21] = 0.0062449;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0091145;
		start[5] = 0.0077098;
		start[6] = 0.008717;
		start[7] = 0.0016404;
		start[8] = 0.0089044;
		start[9] = 0.0041037;
		start[10] = 0.0064123;
		start[11] = 0.0011776;
		start[12] = 0.0084734;
		start[13] = 0.0091165;
		start[14] = 0.0015655;
		start[15] = 0.001542;
		start[16] = 0.007274;
		start[17] = 0.005421;
		start[18] = 0.0012815;
		start[19] = 0.0083399;
		start[20] = 0.0053062;
		start[21] = 0.0080205;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0008274;
		start[5] = 0.0022464;
		start[6] = 0.008524;
		start[7] = 0.0062526;
		start[8] = 0.007088;
		start[9] = 0.0011456;
		start[10] = 0.0092393;
		start[11] = 0.0078814;
		start[12] = 0.0066264;
		start[13] = 0.00086889;
		start[14] = 0.0081962;
		start[15] = 0.009955;
		start[16] = 0.0016289;
		start[17] = 0.009133;
		start[18] = 0.0060731;
		start[19] = 0.0022275;
		start[20] = 0.0069158;
		start[21] = 0.0070301;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0094956;
		start[5] = 0.00042249;
		start[6] = 0.0011516;
		start[7] = 0.0064102;
		start[8] = 0.0077752;
		start[9] = 0.00048071;
		start[10] = 0.006841;
		start[11] = 0.0042518;
		start[12] = 0.0076757;
		start[13] = 0.0036151;
		start[14] = 0.0019863;
		start[15] = 0.009042;
		start[16] = 0.0010582;
		start[17] = 0.0057434;
		start[18] = 0.0024235;
		start[19] = 0.0072123;
		start[20] = 0.0062347;
		start[21] = 0.0045508;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0078329;
		start[5] = 0.0033768;
		start[6] = 0.0015218;
		start[7] = 0.0062394;
		start[8] = 0.0027658;
		start[9] = 0.00015973;
		start[10] = 0.0097303;
		start[11] = 0.0011428;
		start[12] = 0.0022873;
		start[13] = 0.0011146;
		start[14] = 0.0097349;
		start[15] = 0.0015378;
		start[16] = 0.006949;
		start[17] = 0.0012285;
		start[18] = 0.0071803;
		start[19] = 0.0090796;
		start[20] = 0.0073862;
		start[21] = 0.0023214;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0021634;
		start[5] = 0.0071654;
		start[6] = 0.0078757;
		start[7] = 0.0040685;
		start[8] = 0.0080957;
		start[9] = 0.00061316;
		start[10] = 0.0018243;
		start[11] = 0.0027515;
		start[12] = 0.0095378;
		start[13] = 0.0015947;
		start[14] = 0.0035901;
		start[15] = 0.0033144;
		start[16] = 0.0082191;
		start[17] = 0.0015473;
		start[18] = 0.0053013;
		start[19] = 0.0038319;
		start[20] = 0.0094103;
		start[21] = 0.0095157;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0099432;
		start[5] = 0.0037714;
		start[6] = 0.0038103;
		start[7] = 0.0066939;
		start[8] = 0.0012591;
		start[9] = 0.0041663;
		start[10] = 0.0012634;
		start[11] = 0.005086;
		start[12] = 0.0090563;
		start[13] = 0.0070042;
		start[14] = 0.0053224;
		start[15] = 0.0061997;
		start[16] = 0.0011746;
		start[17] = 0.0013514;
		start[18] = 0.0041602;
		start[19] = 0.0071104;
		start[20] = 0.0071057;
		start[21] = 0.0060941;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0075956;
		start[5] = 0.0058446;
		start[6] = 0.0039183;
		start[7] = 0.0099392;
		start[8] = 0.0060634;
		start[9] = 0.0092701;
		start[10] = 0.0087634;
		start[11] = 0.00088238;
		start[12] = 0.0028283;
		start[13] = 0.00054229;
		start[14] = 0.0015024;
		start[15] = 0.007499;
		start[16] = 0.0039992;
		start[17] = 0.0069951;
		start[18] = 0.0008779;
		start[19] = 0.0066309;
		start[20] = 0.0038803;
		start[21] = 0.0043464;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.007946;
		start[5] = 0.003006;
		start[6] = 0.0038521;
		start[7] = 0.0087894;
		start[8] = 0.0090417;
		start[9] = 0.0078536;
		start[10] = 0.0030958;
		start[11] = 0.00016397;
		start[12] = 0.0016523;
		start[13] = 0.0025623;
		start[14] = 0.0047441;
		start[15] = 0.0038214;
		start[16] = 0.00034929;
		start[17] = 0.0099492;
		start[18] = 0.0010897;
		start[19] = 0.0082878;
		start[20] = 0.0093994;
		start[21] = 0.0083452;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.001638;
		start[5] = 0.0018816;
		start[6] = 0.0035793;
		start[7] = 0.0067417;
		start[8] = 0.0093;
		start[9] = 0.0015462;
		start[10] = 0.0093393;
		start[11] = 0.0081377;
		start[12] = 0.0017124;
		start[13] = 0.0057556;
		start[14] = 0.00067773;
		start[15] = 0.009729;
		start[16] = 0.0023326;
		start[17] = 0.0088757;
		start[18] = 0.0065055;
		start[19] = 0.0039984;
		start[20] = 0.0041535;
		start[21] = 0.0015219;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.00024986;
		start[5] = 0.00082195;
		start[6] = 0.0015621;
		start[7] = 0.0097629;
		start[8] = 0.0016761;
		start[9] = 0.007278;
		start[10] = 0.0022062;
		start[11] = 0.0025629;
		start[12] = 0.004978;
		start[13] = 3.2342e-05;
		start[14] = 0.00043035;
		start[15] = 0.0058452;
		start[16] = 0.0087939;
		start[17] = 0.0020619;
		start[18] = 0.009654;
		start[19] = 0.0051903;
		start[20] = 0.0065622;
		start[21] = 0.0019757;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.00091936;
		start[5] = 0.0045033;
		start[6] = 0.0029924;
		start[7] = 0.0054078;
		start[8] = 0.0062113;
		start[9] = 0.0029427;
		start[10] = 0.0033009;
		start[11] = 4.9394e-07;
		start[12] = 0.00086617;
		start[13] = 0.009553;
		start[14] = 0.0018661;
		start[15] = 0.0069683;
		start[16] = 0.0067238;
		start[17] = 0.0038462;
		start[18] = 0.0076956;
		start[19] = 0.0025819;
		start[20] = 0.0021068;
		start[21] = 0.0078801;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0085869;
		start[5] = 0.0029987;
		start[6] = 0.0082455;
		start[7] = 0.0050087;
		start[8] = 0.0033443;
		start[9] = 0.0014656;
		start[10] = 0.0026277;
		start[11] = 0.0064976;
		start[12] = 0.00078808;
		start[13] = 0.0081102;
		start[14] = 0.0058545;
		start[15] = 0.0074918;
		start[16] = 0.0065059;
		start[17] = 0.0097156;
		start[18] = 0.0097776;
		start[19] = 0.0070119;
		start[20] = 0.0012161;
		start[21] = 0.0081523;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0073309;
		start[5] = 0.0093187;
		start[6] = 0.00097548;
		start[7] = 0.0076344;
		start[8] = 0.0038226;
		start[9] = 0.00491;
		start[10] = 0.0056171;
		start[11] = 0.0011424;
		start[12] = 0.00090775;
		start[13] = 0.0054769;
		start[14] = 0.00095003;
		start[15] = 0.001476;
		start[16] = 0.0089559;
		start[17] = 0.0026138;
		start[18] = 0.0085191;
		start[19] = 0.0003794;
		start[20] = 0.0053293;
		start[21] = 0.0072597;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 6.5069e-05;
		start[5] = 0.0078243;
		start[6] = 0.0041417;
		start[7] = 0.0018979;
		start[8] = 0.0066888;
		start[9] = 0.0093722;
		start[10] = 0.002995;
		start[11] = 0.0091107;
		start[12] = 0.003697;
		start[13] = 0.0053263;
		start[14] = 0.00615;
		start[15] = 0.0042758;
		start[16] = 0.0012928;
		start[17] = 0.0065303;
		start[18] = 0.0076981;
		start[19] = 0.0052419;
		start[20] = 0.006179;
		start[21] = 0.0031871;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0059351;
		start[5] = 0.0087281;
		start[6] = 0.0091737;
		start[7] = 0.002941;
		start[8] = 0.0058865;
		start[9] = 0.0041396;
		start[10] = 0.0056842;
		start[11] = 0.0076243;
		start[12] = 0.0059641;
		start[13] = 0.004098;
		start[14] = 0.0091257;
		start[15] = 0.0016318;
		start[16] = 0.0014919;
		start[17] = 0.006057;
		start[18] = 0.0082067;
		start[19] = 0.0038264;
		start[20] = 0.0048934;
		start[21] = 0.0026069;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0021152;
		start[5] = 0.0058359;
		start[6] = 0.0080907;
		start[7] = 0.0043308;
		start[8] = 0.0050117;
		start[9] = 0.0067524;
		start[10] = 0.0076014;
		start[11] = 0.0051735;
		start[12] = 0.0064234;
		start[13] = 0.001124;
		start[14] = 0.0020952;
		start[15] = 0.0048393;
		start[16] = 0.0062859;
		start[17] = 0.0086786;
		start[18] = 0.0017192;
		start[19] = 0.0070107;
		start[20] = 0.0055072;
		start[21] = 0.0093353;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0059004;
		start[5] = 0.0094002;
		start[6] = 0.0087763;
		start[7] = 0.0037144;
		start[8] = 0.0044491;
		start[9] = 0.0022509;
		start[10] = 0.0049305;
		start[11] = 0.0082556;
		start[12] = 0.0016358;
		start[13] = 0.0091154;
		start[14] = 0.008543;
		start[15] = 0.00084178;
		start[16] = 0.0093973;
		start[17] = 0.009574;
		start[18] = 0.0094643;
		start[19] = 0.003246;
		start[20] = 0.002001;
		start[21] = 0.0034435;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0078136;
		start[5] = 0.0086536;
		start[6] = 0.0095861;
		start[7] = 0.0023822;
		start[8] = 0.0082075;
		start[9] = 0.0075746;
		start[10] = 0.0034772;
		start[11] = 0.0048036;
		start[12] = 0.0096448;
		start[13] = 0.00082551;
		start[14] = 0.0067149;
		start[15] = 0.0039574;
		start[16] = 0.0039269;
		start[17] = 0.0098766;
		start[18] = 0.0051845;
		start[19] = 0.0065758;
		start[20] = 0.0012239;
		start[21] = 0.0062662;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0010718;
		start[5] = 0.00037169;
		start[6] = 0.0055369;
		start[7] = 0.0016643;
		start[8] = 0.0078893;
		start[9] = 0.0013587;
		start[10] = 0.0078231;
		start[11] = 0.00419;
		start[12] = 0.0031582;
		start[13] = 0.0039194;
		start[14] = 0.0090029;
		start[15] = 0.006639;
		start[16] = 0.0081055;
		start[17] = 0.0086564;
		start[18] = 0.0090143;
		start[19] = 0.0094149;
		start[20] = 0.0065072;
		start[21] = 0.0038354;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0093833;
		start[5] = 0.0022169;
		start[6] = 0.0076328;
		start[7] = 0.0090042;
		start[8] = 0.0048911;
		start[9] = 0.0035921;
		start[10] = 0.0022137;
		start[11] = 0.0022703;
		start[12] = 0.0025044;
		start[13] = 0.005172;
		start[14] = 0.0017856;
		start[15] = 0.0065181;
		start[16] = 0.0015214;
		start[17] = 0.0041834;
		start[18] = 0.0076677;
		start[19] = 0.0026507;
		start[20] = 0.0024785;
		start[21] = 0.0077029;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0051564;
		start[5] = 0.0094674;
		start[6] = 0.0048012;
		start[7] = 0.0033467;
		start[8] = 0.0074461;
		start[9] = 0.0061386;
		start[10] = 0.0062009;
		start[11] = 0.0061089;
		start[12] = 0.0069951;
		start[13] = 0.0061408;
		start[14] = 0.0074309;
		start[15] = 0.006592;
		start[16] = 0.0096248;
		start[17] = 0.005057;
		start[18] = 0.0016087;
		start[19] = 0.0041533;
		start[20] = 0.0041111;
		start[21] = 0.0013222;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0030371;
		start[5] = 0.006748;
		start[6] = 0.0010034;
		start[7] = 0.0090818;
		start[8] = 0.00879;
		start[9] = 0.0011443;
		start[10] = 0.0068739;
		start[11] = 0.0045289;
		start[12] = 0.0084374;
		start[13] = 0.0019293;
		start[14] = 0.0066908;
		start[15] = 0.0053749;
		start[16] = 0.0048968;
		start[17] = 0.0030292;
		start[18] = 0.0094652;
		start[19] = 0.00075353;
		start[20] = 0.0033814;
		start[21] = 0.0051721;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.00999;
		start[5] = 0.0016122;
		start[6] = 0.0017126;
		start[7] = 0.0015869;
		start[8] = 0.0027729;
		start[9] = 0.0073073;
		start[10] = 0.0066489;
		start[11] = 0.0016025;
		start[12] = 0.00028599;
		start[13] = 0.0092746;
		start[14] = 0.0074447;
		start[15] = 0.0060296;
		start[16] = 0.0035963;
		start[17] = 0.0062323;
		start[18] = 0.0040386;
		start[19] = 0.0064857;
		start[20] = 0.0056098;
		start[21] = 0.0019537;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.00082516;
		start[5] = 0.007277;
		start[6] = 0.0052145;
		start[7] = 0.0051672;
		start[8] = 0.00025599;
		start[9] = 0.0046468;
		start[10] = 0.0049825;
		start[11] = 0.0024777;
		start[12] = 0.0083863;
		start[13] = 0.0093209;
		start[14] = 0.0098669;
		start[15] = 0.0065821;
		start[16] = 0.0024493;
		start[17] = 0.0049023;
		start[18] = 0.0063509;
		start[19] = 0.00057087;
		start[20] = 0.00086333;
		start[21] = 0.0019232;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0023992;
		start[5] = 0.0077064;
		start[6] = 9.9432e-06;
		start[7] = 0.00049765;
		start[8] = 0.0081599;
		start[9] = 0.0013524;
		start[10] = 0.0032004;
		start[11] = 0.00205;
		start[12] = 0.0017765;
		start[13] = 0.0018318;
		start[14] = 0.0095771;
		start[15] = 0.0056023;
		start[16] = 0.0077368;
		start[17] = 0.0084574;
		start[18] = 0.0047769;
		start[19] = 0.00024397;
		start[20] = 0.0032485;
		start[21] = 0.002098;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0086665;
		start[5] = 0.0036953;
		start[6] = 0.0032355;
		start[7] = 0.0054837;
		start[8] = 0.0021032;
		start[9] = 0.0010339;
		start[10] = 0.0027244;
		start[11] = 0.0038858;
		start[12] = 0.0070522;
		start[13] = 0.0042955;
		start[14] = 0.0059388;
		start[15] = 0.0031194;
		start[16] = 0.0072771;
		start[17] = 0.0034432;
		start[18] = 0.0097365;
		start[19] = 0.0041291;
		start[20] = 0.0047959;
		start[21] = 0.0064207;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0077164;
		start[5] = 0.0025965;
		start[6] = 0.0018236;
		start[7] = 0.0097804;
		start[8] = 0.0079959;
		start[9] = 0.0032608;
		start[10] = 0.0033302;
		start[11] = 0.0020635;
		start[12] = 0.0068097;
		start[13] = 0.0019659;
		start[14] = 0.0027729;
		start[15] = 0.0063594;
		start[16] = 0.0010198;
		start[17] = 0.00080684;
		start[18] = 0.0074271;
		start[19] = 0.0033494;
		start[20] = 0.0061625;
		start[21] = 0.0047518;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0074941;
		start[5] = 0.0062671;
		start[6] = 0.0030056;
		start[7] = 0.0092999;
		start[8] = 0.0048986;
		start[9] = 0.0011067;
		start[10] = 0.0081642;
		start[11] = 0.0090526;
		start[12] = 0.0063235;
		start[13] = 0.0034009;
		start[14] = 0.0054793;
		start[15] = 0.00081994;
		start[16] = 0.0034869;
		start[17] = 0.0092481;
		start[18] = 0.0084598;
		start[19] = 0.0019536;
		start[20] = 0.0081037;
		start[21] = 0.0027438;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0064131;
		start[5] = 0.0055518;
		start[6] = 0.0056609;
		start[7] = 0.0085768;
		start[8] = 0.0010085;
		start[9] = 0.0087581;
		start[10] = 0.0072951;
		start[11] = 0.0044296;
		start[12] = 0.0020702;
		start[13] = 0.0015538;
		start[14] = 0.0072922;
		start[15] = 0.0029843;
		start[16] = 0.0056497;
		start[17] = 0.0055541;
		start[18] = 0.0071268;
		start[19] = 0.0098687;
		start[20] = 0.0048461;
		start[21] = 0.0074633;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0053308;
		start[5] = 0.0099112;
		start[6] = 0.0012273;
		start[7] = 0.0095161;
		start[8] = 0.0013236;
		start[9] = 0.0012095;
		start[10] = 0.009137;
		start[11] = 0.0094839;
		start[12] = 0.006644;
		start[13] = 0.0056072;
		start[14] = 0.0015723;
		start[15] = 0.0069664;
		start[16] = 0.0041224;
		start[17] = 0.0058416;
		start[18] = 0.0098853;
		start[19] = 0.00017587;
		start[20] = 0.006218;
		start[21] = 0.001406;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0080364;
		start[5] = 0.000815;
		start[6] = 0.0032986;
		start[7] = 0.0025692;
		start[8] = 0.00073971;
		start[9] = 0.0027237;
		start[10] = 0.0050995;
		start[11] = 0.0039825;
		start[12] = 4.5609e-05;
		start[13] = 0.0012884;
		start[14] = 0.0091843;
		start[15] = 0.0027478;
		start[16] = 0.0075194;
		start[17] = 0.009785;
		start[18] = 0.0057335;
		start[19] = 0.0047944;
		start[20] = 0.0011882;
		start[21] = 0.0055047;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0069986;
		start[5] = 0.0070742;
		start[6] = 0.0024196;
		start[7] = 0.0049127;
		start[8] = 0.0055333;
		start[9] = 0.0044069;
		start[10] = 0.006683;
		start[11] = 0.009341;
		start[12] = 0.0065224;
		start[13] = 0.001672;
		start[14] = 0.0024005;
		start[15] = 0.0038533;
		start[16] = 0.0032067;
		start[17] = 0.0057773;
		start[18] = 0.009872;
		start[19] = 0.0026845;
		start[20] = 0.0057807;
		start[21] = 0.0029213;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0091834;
		start[5] = 0.0011439;
		start[6] = 0.0047436;
		start[7] = 0.0079915;
		start[8] = 0.0082583;
		start[9] = 0.003739;
		start[10] = 0.002916;
		start[11] = 0.0037456;
		start[12] = 0.0098673;
		start[13] = 0.004333;
		start[14] = 0.0081543;
		start[15] = 0.0006426;
		start[16] = 0.0042776;
		start[17] = 0.00019945;
		start[18] = 0.0038152;
		start[19] = 0.0041397;
		start[20] = 0.0055982;
		start[21] = 0.0088342;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0048655;
		start[5] = 0.0076406;
		start[6] = 0.0067084;
		start[7] = 0.0080869;
		start[8] = 0.0099238;
		start[9] = 0.0082961;
		start[10] = 0.0020735;
		start[11] = 0.0087521;
		start[12] = 0.0013254;
		start[13] = 0.0038705;
		start[14] = 0.0013553;
		start[15] = 0.0097152;
		start[16] = 0.00076893;
		start[17] = 0.0077269;
		start[18] = 0.0077255;
		start[19] = 0.0031508;
		start[20] = 0.0040887;
		start[21] = 0.0055258;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0043497;
		start[5] = 0.0011792;
		start[6] = 0.007647;
		start[7] = 0.0066183;
		start[8] = 0.0085949;
		start[9] = 0.0093447;
		start[10] = 0.0046983;
		start[11] = 0.0082389;
		start[12] = 0.00089338;
		start[13] = 0.0029963;
		start[14] = 0.00079201;
		start[15] = 0.0034916;
		start[16] = 0.0080923;
		start[17] = 0.0070024;
		start[18] = 0.0097551;
		start[19] = 0.0092566;
		start[20] = 0.00093932;
		start[21] = 0.0013519;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.00058124;
		start[5] = 0.0061336;
		start[6] = 0.0083025;
		start[7] = 0.0059438;
		start[8] = 0.0035959;
		start[9] = 0.0097224;
		start[10] = 0.0026303;
		start[11] = 0.0097267;
		start[12] = 0.0055061;
		start[13] = 0.0044533;
		start[14] = 0.0089844;
		start[15] = 0.0050237;
		start[16] = 0.0091892;
		start[17] = 0.0093271;
		start[18] = 0.0011495;
		start[19] = 0.007378;
		start[20] = 0.0066182;
		start[21] = 0.0076581;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0013063;
		start[5] = 0.0097946;
		start[6] = 0.0067177;
		start[7] = 0.0087702;
		start[8] = 0.00054118;
		start[9] = 0.0049137;
		start[10] = 0.0048797;
		start[11] = 0.005818;
		start[12] = 0.0091163;
		start[13] = 0.0075365;
		start[14] = 0.0088592;
		start[15] = 0.0017459;
		start[16] = 0.0007395;
		start[17] = 0.0026928;
		start[18] = 0.002127;
		start[19] = 0.0032867;
		start[20] = 0.0066082;
		start[21] = 0.0048016;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0021162;
		start[5] = 0.0083113;
		start[6] = 0.0022091;
		start[7] = 0.0072329;
		start[8] = 0.0066151;
		start[9] = 0.0022599;
		start[10] = 0.0074184;
		start[11] = 0.0089222;
		start[12] = 0.0072155;
		start[13] = 0.0041103;
		start[14] = 0.0071555;
		start[15] = 0.0094158;
		start[16] = 0.0093541;
		start[17] = 0.0056664;
		start[18] = 0.00035342;
		start[19] = 0.00060221;
		start[20] = 0.0018849;
		start[21] = 0.0037614;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0056867;
		start[5] = 0.0066903;
		start[6] = 0.0090499;
		start[7] = 0.0077943;
		start[8] = 0.0088611;
		start[9] = 0.0065405;
		start[10] = 0.0012631;
		start[11] = 1.6063e-05;
		start[12] = 0.0017394;
		start[13] = 0.0063386;
		start[14] = 0.0081804;
		start[15] = 0.004491;
		start[16] = 0.0080893;
		start[17] = 0.00028704;
		start[18] = 0.00035029;
		start[19] = 0.0038349;
		start[20] = 0.0030674;
		start[21] = 0.0031973;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0023053;
		start[5] = 0.0088607;
		start[6] = 0.0048729;
		start[7] = 0.00041079;
		start[8] = 0.0071193;
		start[9] = 0.0064361;
		start[10] = 0.0095407;
		start[11] = 0.0066181;
		start[12] = 0.0028161;
		start[13] = 0.0077893;
		start[14] = 0.00073929;
		start[15] = 0.0028966;
		start[16] = 0.0033132;
		start[17] = 0.0020354;
		start[18] = 0.0058621;
		start[19] = 0.0056102;
		start[20] = 4.8181e-05;
		start[21] = 0.0069091;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0029691;
		start[5] = 0.0049955;
		start[6] = 0.0040333;
		start[7] = 0.0062638;
		start[8] = 0.0065266;
		start[9] = 0.00064413;
		start[10] = 0.0062764;
		start[11] = 0.0048934;
		start[12] = 0.00011619;
		start[13] = 5.715e-05;
		start[14] = 0.0048706;
		start[15] = 0.0080029;
		start[16] = 0.0070636;
		start[17] = 0.0023399;
		start[18] = 0.0040517;
		start[19] = 0.0070418;
		start[20] = 0.0029517;
		start[21] = 0.00882;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.00063538;
		start[5] = 0.0062368;
		start[6] = 0.0059666;
		start[7] = 0.0079045;
		start[8] = 0.0059951;
		start[9] = 0.0084734;
		start[10] = 0.0015852;
		start[11] = 0.0095256;
		start[12] = 0.0066394;
		start[13] = 0.008771;
		start[14] = 0.0011757;
		start[15] = 0.0075753;
		start[16] = 0.004935;
		start[17] = 0.0022891;
		start[18] = 0.0050999;
		start[19] = 0.0014532;
		start[20] = 0.00084089;
		start[21] = 0.0050128;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0097009;
		start[5] = 0.0013223;
		start[6] = 0.0096815;
		start[7] = 0.0079916;
		start[8] = 0.009785;
		start[9] = 0.0091625;
		start[10] = 0.0052912;
		start[11] = 0.0097167;
		start[12] = 0.002643;
		start[13] = 0.0050832;
		start[14] = 0.0048643;
		start[15] = 0.0049672;
		start[16] = 0.0032243;
		start[17] = 0.0069464;
		start[18] = 0.0027679;
		start[19] = 0.0046421;
		start[20] = 0.0069057;
		start[21] = 0.0020116;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0053863;
		start[5] = 0.0038469;
		start[6] = 0.0079603;
		start[7] = 0.0016187;
		start[8] = 0.0066596;
		start[9] = 0.0031694;
		start[10] = 0.0033583;
		start[11] = 0.0049621;
		start[12] = 0.0061336;
		start[13] = 0.0029539;
		start[14] = 0.00047195;
		start[15] = 0.0080078;
		start[16] = 0.0019967;
		start[17] = 0.0043797;
		start[18] = 0.0038385;
		start[19] = 0.0053992;
		start[20] = 0.0039471;
		start[21] = 0.0043135;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0016017;
		start[5] = 0.0022012;
		start[6] = 0.00027149;
		start[7] = 0.0053248;
		start[8] = 0.0071641;
		start[9] = 3.1445e-05;
		start[10] = 0.0037632;
		start[11] = 0.002238;
		start[12] = 0.0092749;
		start[13] = 0.0036458;
		start[14] = 0.0088256;
		start[15] = 0.0010339;
		start[16] = 0.00023476;
		start[17] = 0.00027227;
		start[18] = 0.0016005;
		start[19] = 0.0064378;
		start[20] = 0.0078648;
		start[21] = 0.0035853;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0045243;
		start[5] = 0.0038499;
		start[6] = 0.0089711;
		start[7] = 0.0094188;
		start[8] = 0.0022106;
		start[9] = 0.0078033;
		start[10] = 0.0045317;
		start[11] = 0.0052162;
		start[12] = 0.0094893;
		start[13] = 0.0017015;
		start[14] = 0.0068772;
		start[15] = 0.0035997;
		start[16] = 0.00078392;
		start[17] = 0.002963;
		start[18] = 0.0075284;
		start[19] = 0.0078481;
		start[20] = 0.0071323;
		start[21] = 0.00098723;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0042306;
		start[5] = 0.0091445;
		start[6] = 0.0042348;
		start[7] = 0.0012029;
		start[8] = 0.0027334;
		start[9] = 0.0066421;
		start[10] = 0.0083495;
		start[11] = 0.0036688;
		start[12] = 0.0094214;
		start[13] = 0.0015646;
		start[14] = 0.0081615;
		start[15] = 0.006995;
		start[16] = 0.0064395;
		start[17] = 0.0057096;
		start[18] = 1.2502e-05;
		start[19] = 0.0084903;
		start[20] = 0.007534;
		start[21] = 0.0010846;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0089081;
		start[5] = 0.0035708;
		start[6] = 0.0015059;
		start[7] = 0.0079038;
		start[8] = 0.005336;
		start[9] = 0.003069;
		start[10] = 0.0041452;
		start[11] = 0.0073228;
		start[12] = 0.0037891;
		start[13] = 0.0033194;
		start[14] = 0.0016485;
		start[15] = 0.0037629;
		start[16] = 0.0058935;
		start[17] = 0.0034362;
		start[18] = 0.0027256;
		start[19] = 0.0025068;
		start[20] = 0.008567;
		start[21] = 0.0053273;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0065441;
		start[5] = 0.0026387;
		start[6] = 0.0089555;
		start[7] = 0.0096737;
		start[8] = 0.0043751;
		start[9] = 0.008773;
		start[10] = 0.0085587;
		start[11] = 0.0016514;
		start[12] = 0.0001849;
		start[13] = 0.0040117;
		start[14] = 0.0034297;
		start[15] = 0.0075124;
		start[16] = 0.0012796;
		start[17] = 0.0096033;
		start[18] = 0.0020333;
		start[19] = 0.0051812;
		start[20] = 0.0069844;
		start[21] = 0.0077235;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0066415;
		start[5] = 0.0029305;
		start[6] = 0.0096414;
		start[7] = 0.0036294;
		start[8] = 0.0018118;
		start[9] = 0.0059262;
		start[10] = 0.0048817;
		start[11] = 0.0024621;
		start[12] = 0.0094274;
		start[13] = 0.0048583;
		start[14] = 0.0091283;
		start[15] = 0.0013218;
		start[16] = 0.0030889;
		start[17] = 0.0041605;
		start[18] = 0.0040538;
		start[19] = 0.0096596;
		start[20] = 0.0018571;
		start[21] = 0.007566;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0094742;
		start[5] = 0.005687;
		start[6] = 0.00079199;
		start[7] = 0.00061555;
		start[8] = 0.0062988;
		start[9] = 0.0020015;
		start[10] = 0.0075203;
		start[11] = 0.0032832;
		start[12] = 0.0045971;
		start[13] = 0.0045962;
		start[14] = 0.0033437;
		start[15] = 0.0069324;
		start[16] = 0.0062715;
		start[17] = 0.0011643;
		start[18] = 0.00057014;
		start[19] = 0.0043162;
		start[20] = 0.009322;
		start[21] = 0.0010678;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.00021558;
		start[5] = 0.0068162;
		start[6] = 0.0073453;
		start[7] = 0.002426;
		start[8] = 0.0088168;
		start[9] = 0.0059371;
		start[10] = 0.0058845;
		start[11] = 0.0041426;
		start[12] = 0.00078354;
		start[13] = 0.0072741;
		start[14] = 0.0085416;
		start[15] = 0.0034374;
		start[16] = 0.0087374;
		start[17] = 0.0082217;
		start[18] = 0.0019201;
		start[19] = 0.006611;
		start[20] = 0.00084992;
		start[21] = 0.0064246;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0056242;
		start[5] = 0.0091114;
		start[6] = 0.008267;
		start[7] = 0.0035749;
		start[8] = 0.007644;
		start[9] = 0.0073421;
		start[10] = 0.00080309;
		start[11] = 0.0050857;
		start[12] = 0.0030481;
		start[13] = 0.0059768;
		start[14] = 0.0050549;
		start[15] = 0.0016902;
		start[16] = 0.0080958;
		start[17] = 0.0065241;
		start[18] = 0.0053655;
		start[19] = 0.0089041;
		start[20] = 0.00054617;
		start[21] = 0.00291;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0057245;
		start[5] = 0.0050136;
		start[6] = 0.0072893;
		start[7] = 0.0047203;
		start[8] = 0.0092833;
		start[9] = 0.0095582;
		start[10] = 0.006705;
		start[11] = 0.0051282;
		start[12] = 0.0080184;
		start[13] = 0.0022282;
		start[14] = 0.0088466;
		start[15] = 0.0071821;
		start[16] = 0.0082571;
		start[17] = 0.0011112;
		start[18] = 0.0056278;
		start[19] = 0.0087427;
		start[20] = 0.0081719;
		start[21] = 0.00069111;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0054675;
		start[5] = 0.0098721;
		start[6] = 0.0078902;
		start[7] = 0.0023638;
		start[8] = 0.0039011;
		start[9] = 0.0012788;
		start[10] = 0.0010472;
		start[11] = 0.0033166;
		start[12] = 0.0074522;
		start[13] = 0.0041225;
		start[14] = 0.0085712;
		start[15] = 0.0059087;
		start[16] = 0.0067818;
		start[17] = 0.005351;
		start[18] = 0.00092309;
		start[19] = 0.0039108;
		start[20] = 0.004472;
		start[21] = 0.00058708;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0071692;
		start[5] = 0.002985;
		start[6] = 0.0099593;
		start[7] = 0.00084576;
		start[8] = 0.0001521;
		start[9] = 0.0019968;
		start[10] = 0.0080717;
		start[11] = 0.0047019;
		start[12] = 0.0031251;
		start[13] = 0.0028584;
		start[14] = 0.0098026;
		start[15] = 0.007583;
		start[16] = 0.0013044;
		start[17] = 0.0062721;
		start[18] = 0.0045744;
		start[19] = 0.006763;
		start[20] = 0.0070719;
		start[21] = 0.0017362;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0050463;
		start[5] = 0.0053188;
		start[6] = 0.0053616;
		start[7] = 0.0041575;
		start[8] = 0.0055595;
		start[9] = 0.0067562;
		start[10] = 0.0051154;
		start[11] = 0.0056492;
		start[12] = 0.00094214;
		start[13] = 0.0047012;
		start[14] = 0.0016802;
		start[15] = 0.0072105;
		start[16] = 0.0098215;
		start[17] = 0.0091892;
		start[18] = 0.0040634;
		start[19] = 0.0074648;
		start[20] = 0.0043896;
		start[21] = 0.0084842;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0044541;
		start[5] = 0.0071619;
		start[6] = 0.0065688;
		start[7] = 0.000673;
		start[8] = 0.0061357;
		start[9] = 0.0023616;
		start[10] = 0.0053281;
		start[11] = 0.0018269;
		start[12] = 0.0098402;
		start[13] = 0.0069214;
		start[14] = 0.0029045;
		start[15] = 0.006705;
		start[16] = 0.0057405;
		start[17] = 0.008176;
		start[18] = 0.0025045;
		start[19] = 0.002337;
		start[20] = 0.0026953;
		start[21] = 0.0021431;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0097169;
		start[5] = 0.0094398;
		start[6] = 0.0089218;
		start[7] = 0.0003388;
		start[8] = 0.0030009;
		start[9] = 0.0034285;
		start[10] = 0.0021415;
		start[11] = 0.0098414;
		start[12] = 0.0030627;
		start[13] = 0.0070349;
		start[14] = 0.004523;
		start[15] = 0.0094097;
		start[16] = 0.0037912;
		start[17] = 0.0088998;
		start[18] = 0.0024427;
		start[19] = 0.00096271;
		start[20] = 0.002135;
		start[21] = 0.004628;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0081041;
		start[5] = 0.0064079;
		start[6] = 0.0055855;
		start[7] = 0.0067971;
		start[8] = 0.0032652;
		start[9] = 0.00454;
		start[10] = 0.0066541;
		start[11] = 0.0032392;
		start[12] = 0.0036893;
		start[13] = 0.0069426;
		start[14] = 0.0038965;
		start[15] = 0.0042766;
		start[16] = 0.0022187;
		start[17] = 0.0075249;
		start[18] = 0.0077491;
		start[19] = 0.0043578;
		start[20] = 0.00071449;
		start[21] = 0.0066448;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0043125;
		start[5] = 0.0092829;
		start[6] = 0.0074644;
		start[7] = 0.0035986;
		start[8] = 0.0037646;
		start[9] = 0.0015339;
		start[10] = 0.0066415;
		start[11] = 0.0017885;
		start[12] = 0.008826;
		start[13] = 0.00012159;
		start[14] = 0.00057589;
		start[15] = 0.0076975;
		start[16] = 0.00046277;
		start[17] = 0.0087632;
		start[18] = 0.007688;
		start[19] = 0.00097644;
		start[20] = 0.0044556;
		start[21] = 0.0058065;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0060689;
		start[5] = 0.0020661;
		start[6] = 0.0015354;
		start[7] = 0.0050591;
		start[8] = 0.0048378;
		start[9] = 0.0012295;
		start[10] = 0.0029991;
		start[11] = 0.0013611;
		start[12] = 0.00052497;
		start[13] = 0.0038432;
		start[14] = 0.007396;
		start[15] = 0.0065588;
		start[16] = 0.0095819;
		start[17] = 0.0050526;
		start[18] = 0.006359;
		start[19] = 0.0034602;
		start[20] = 0.0024611;
		start[21] = 0.0095122;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.001654;
		start[5] = 0.0052749;
		start[6] = 0.0099696;
		start[7] = 0.0090932;
		start[8] = 0.0065182;
		start[9] = 0.0013145;
		start[10] = 0.0072438;
		start[11] = 0.0063332;
		start[12] = 0.0079752;
		start[13] = 0.0074818;
		start[14] = 0.00034212;
		start[15] = 0.0050279;
		start[16] = 0.0037588;
		start[17] = 0.0089323;
		start[18] = 0.0018286;
		start[19] = 0.0011582;
		start[20] = 0.0040235;
		start[21] = 0.0068124;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (22);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0075039;
		start[5] = 0.0071771;
		start[6] = 0.0073291;
		start[7] = 0.003155;
		start[8] = 0.0067051;
		start[9] = 0.0011407;
		start[10] = 0.0094921;
		start[11] = 0.0047818;
		start[12] = 0.0059536;
		start[13] = 0.0060048;
		start[14] = 0.0039626;
		start[15] = 0.00084978;
		start[16] = 0.0026151;
		start[17] = 0.0026506;
		start[18] = 0.0050389;
		start[19] = 0.0021293;
		start[20] = 0.0035402;
		start[21] = 0.0019479;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}


  return 0;
}
