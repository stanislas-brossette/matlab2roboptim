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
    (6, 1, "CostFunction_upperBody2D"),
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

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0; 
			 grad[1] = 0; 
			 grad[2] = 0; 
			 grad[3] = 0; 
			 grad[4] = 0; 
			 grad[5] = 0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class EEConstraint_1 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
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
  : roboptim::GenericDifferentiableFunction<T>
    (6, 1, "EEConstraint_1_upperBody2D"),
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
  
	result[0] = 0.25*sin(q_01)*sin(q_02) - 0.5*sin(q_01) - 0.25*cos(q_01)*cos(q_02) - 0.5*cos(q_01)*sin(q_02) - 0.5*cos(q_02)*sin(q_01) - 1.0*EE_1_1 - 0.3*sin(q_04)*(cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 0.3*cos(q_04)*(sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01));
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

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.25*cos(q_01)*sin(q_02) - 0.5*cos(q_01)*cos(q_02) - 0.5*cos(q_01) + 0.25*cos(q_02)*sin(q_01) + 0.5*sin(q_01)*sin(q_02) - 0.3*cos(q_04)*(cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + 0.3*sin(q_04)*(sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + 0.25*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)); 
			 grad[1] = 0.25*cos(q_01)*sin(q_02) - 0.5*cos(q_01)*cos(q_02) + 0.25*cos(q_02)*sin(q_01) + 0.5*sin(q_01)*sin(q_02) - 0.3*cos(q_04)*(cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + 0.3*sin(q_04)*(sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + 0.25*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)); 
			 grad[2] = 0.3*sin(q_04)*(sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 0.3*cos(q_04)*(cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + 0.25*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)); 
			 grad[3] = 0.3*sin(q_04)*(sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.3*cos(q_04)*(cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[4] = 0; 
			 grad[5] = 0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class EEConstraint_2 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
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
  : roboptim::GenericDifferentiableFunction<T>
    (6, 1, "EEConstraint_2_upperBody2D"),
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
  
	result[0] = 0.5*cos(q_01) - 1.0*EE_1_2 + 0.5*cos(q_01)*cos(q_02) - 0.25*cos(q_01)*sin(q_02) - 0.25*cos(q_02)*sin(q_01) - 0.5*sin(q_01)*sin(q_02) + 0.3*cos(q_04)*(cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + 0.25*cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 0.3*sin(q_04)*(sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01));
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

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.25*sin(q_01)*sin(q_02) - 0.25*cos(q_01)*cos(q_02) - 0.5*cos(q_01)*sin(q_02) - 0.5*cos(q_02)*sin(q_01) - 0.5*sin(q_01) - 0.3*sin(q_04)*(cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 0.3*cos(q_04)*(sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)); 
			 grad[1] = 0.25*sin(q_01)*sin(q_02) - 0.5*cos(q_01)*sin(q_02) - 0.5*cos(q_02)*sin(q_01) - 0.25*cos(q_01)*cos(q_02) - 0.3*sin(q_04)*(cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 0.3*cos(q_04)*(sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)); 
			 grad[2] = - 0.3*sin(q_04)*(cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 0.3*cos(q_04)*(sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)); 
			 grad[3] = - 0.3*sin(q_04)*(cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.3*cos(q_04)*(sin(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03 + 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[4] = 0; 
			 grad[5] = 0; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class EEConstraint_3 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
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
  : roboptim::GenericDifferentiableFunction<T>
    (6, 1, "EEConstraint_3_upperBody2D"),
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
  
	result[0] = 0.25*cos(q_01)*cos(q_02) - 0.5*sin(q_01) - 1.0*EE_2_1 - 0.5*cos(q_01)*sin(q_02) - 0.5*cos(q_02)*sin(q_01) - 0.25*sin(q_01)*sin(q_02) - 0.25*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 0.3*sin(q_06)*(cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 0.3*cos(q_06)*(sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)));
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

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.5*sin(q_01)*sin(q_02) - 0.5*cos(q_01)*cos(q_02) - 0.25*cos(q_01)*sin(q_02) - 0.25*cos(q_02)*sin(q_01) - 0.5*cos(q_01) - 0.25*cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 0.3*cos(q_06)*(cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + 0.25*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + 0.3*sin(q_06)*(sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[1] = 0.5*sin(q_01)*sin(q_02) - 0.25*cos(q_01)*sin(q_02) - 0.25*cos(q_02)*sin(q_01) - 0.5*cos(q_01)*cos(q_02) - 0.25*cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 0.3*cos(q_06)*(cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + 0.25*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + 0.3*sin(q_06)*(sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[2] = 0; 
			 grad[3] = 0; 
			 grad[4] = 0.25*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 0.3*cos(q_06)*(cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + 0.3*sin(q_06)*(sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[5] = 0.3*sin(q_06)*(sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.3*cos(q_06)*(cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
template <typename T>
class EEConstraint_4 : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
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
  : roboptim::GenericDifferentiableFunction<T>
    (6, 1, "EEConstraint_4_upperBody2D"),
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
  
	result[0] = 0.5*cos(q_01) - 1.0*EE_2_2 + 0.5*cos(q_01)*cos(q_02) + 0.25*cos(q_01)*sin(q_02) + 0.25*cos(q_02)*sin(q_01) - 0.5*sin(q_01)*sin(q_02) + 0.25*cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + 0.3*cos(q_06)*(cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 0.3*sin(q_06)*(sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)));
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

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.25*cos(q_01)*cos(q_02) - 0.5*sin(q_01) - 0.5*cos(q_01)*sin(q_02) - 0.5*cos(q_02)*sin(q_01) - 0.25*sin(q_01)*sin(q_02) - 0.25*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 0.3*sin(q_06)*(cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 0.3*cos(q_06)*(sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[1] = 0.25*cos(q_01)*cos(q_02) - 0.5*cos(q_01)*sin(q_02) - 0.5*cos(q_02)*sin(q_01) - 0.25*sin(q_01)*sin(q_02) - 0.25*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 0.3*sin(q_06)*(cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 0.3*cos(q_06)*(sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[2] = 0; 
			 grad[3] = 0; 
			 grad[4] = - 0.25*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 0.3*sin(q_06)*(cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.25*cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 0.3*cos(q_06)*(sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[5] = - 0.3*sin(q_06)*(cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 0.3*cos(q_06)*(sin(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_05 - 1.5707963267948966192313216916398)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
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

	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_1 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_2 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<EEConstraint_3<roboptim::EigenMatrixDense> > cstrFunc_3 = boost::make_shared<EEConstraint_3<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);
	boost::shared_ptr<EEConstraint_4<roboptim::EigenMatrixDense> > cstrFunc_4 = boost::make_shared<EEConstraint_4<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);

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
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_1), bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_2), bounds, scales); 
	}
	{
		EEConstraint_3<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_3), bounds, scales); 
	}
	{
		EEConstraint_4<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense> > (cstrFunc_4), bounds, scales); 
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
		std::vector<double> start (6);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		std::vector<double> endEff (4);
		endEff[0] = 0.28647;
		endEff[1] = -0.17173;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		std::vector<double> endEff (4);
		endEff[0] = 0.53075;
		endEff[1] = 0.7776;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		std::vector<double> endEff (4);
		endEff[0] = -0.54792;
		endEff[1] = 0.30006;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		std::vector<double> endEff (4);
		endEff[0] = 0.6638;
		endEff[1] = 0.80531;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		std::vector<double> endEff (4);
		endEff[0] = -0.20647;
		endEff[1] = 0.10678;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		std::vector<double> endEff (4);
		endEff[0] = -0.34179;
		endEff[1] = 0.52947;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		std::vector<double> endEff (4);
		endEff[0] = 0.65134;
		endEff[1] = 0.38062;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		std::vector<double> endEff (4);
		endEff[0] = -0.19848;
		endEff[1] = -0.42844;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		std::vector<double> endEff (4);
		endEff[0] = 0.021611;
		endEff[1] = -0.093342;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.091544;
		start[1] = -0.099695;
		start[2] = 0.0020518;
		start[3] = -0.0048826;
		start[4] = -0.0391;
		start[5] = -0.033367;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.02063;
		start[1] = 0.022069;
		start[2] = 0.02331;
		start[3] = -0.040672;
		start[4] = 0.022989;
		start[5] = -0.074244;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.094283;
		start[1] = 0.096459;
		start[2] = 0.034057;
		start[3] = -0.035792;
		start[4] = -0.093565;
		start[5] = -0.059461;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = -0.016727;
		start[1] = -0.043958;
		start[2] = -0.091748;
		start[3] = 0.046817;
		start[4] = 0.080009;
		start[5] = 0.064654;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.025469;
		start[1] = -0.073271;
		start[2] = 0.046743;
		start[3] = -0.014786;
		start[4] = 0.078479;
		start[5] = 0.032396;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.087854;
		start[1] = -0.030866;
		start[2] = -0.014277;
		start[3] = 0.0026397;
		start[4] = 0.056562;
		start[5] = -0.010294;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.095432;
		start[1] = -0.086115;
		start[2] = 0.086116;
		start[3] = -0.053985;
		start[4] = -0.096175;
		start[5] = -0.049586;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.037482;
		start[1] = -0.047368;
		start[2] = 0.030357;
		start[3] = -0.069896;
		start[4] = -0.050139;
		start[5] = 0.016097;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.090523;
		start[1] = 0.058064;
		start[2] = -0.06892;
		start[3] = -0.077258;
		start[4] = -0.080933;
		start[5] = 0.025848;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (6);
		start[0] = 0.033482;
		start[1] = -0.034502;
		start[2] = 0.083832;
		start[3] = 0.015981;
		start[4] = -0.067822;
		start[5] = 0.083532;
		std::vector<double> endEff (4);
		endEff[0] = 1.026;
		endEff[1] = -0.19282;
		solveFor( start, endEff);
	}


  return 0;
}
