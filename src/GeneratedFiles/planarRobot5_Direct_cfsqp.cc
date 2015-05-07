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
    (5, 1, "CostFunction_planarRobot5"),
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

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0; 
			 grad[1] = 0; 
			 grad[2] = 0; 
			 grad[3] = 0; 
			 grad[4] = 0; 
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
  : roboptim::GenericDifferentiableFunction<T>
    (5, 1, "EEConstraint_1_planarRobot5"),
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
  
	result[0] = cos(q_01) - 1.0*EE_1_1 + cos(q_05)*(cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) + cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02) - 1.0*sin(q_05)*(sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) + cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)));
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

  switch (id)
    {
      
		case 0: 
			 grad[0] = - 1.0*sin(q_01) - 1.0*sin(q_05)*(cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) - 1.0*cos(q_01)*sin(q_02) - 1.0*cos(q_02)*sin(q_01) - 1.0*cos(q_05)*(sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) - 1.0*sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[1] = - 1.0*sin(q_05)*(cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) - 1.0*cos(q_01)*sin(q_02) - 1.0*cos(q_02)*sin(q_01) - 1.0*cos(q_05)*(sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) - 1.0*sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[2] = - 1.0*sin(q_05)*(cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) - 1.0*cos(q_05)*(sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) - 1.0*sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[3] = - 1.0*sin(q_05)*(cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) - 1.0*cos(q_05)*(sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) - 1.0*sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[4] = - 1.0*sin(q_05)*(cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) - 1.0*cos(q_05)*(sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))); 
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
  : roboptim::GenericDifferentiableFunction<T>
    (5, 1, "EEConstraint_2_planarRobot5"),
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
  
	result[0] = sin(q_01) - 1.0*EE_1_2 + sin(q_05)*(cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) + cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01) + cos(q_05)*(sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) + sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)));
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

  switch (id)
    {
      
		case 0: 
			 grad[0] = cos(q_01) + cos(q_05)*(cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) + cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02) - 1.0*sin(q_05)*(sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) + cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[1] = cos(q_05)*(cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) + cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02) - 1.0*sin(q_05)*(sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) + cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[2] = cos(q_05)*(cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) - 1.0*sin(q_05)*(sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) + cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[3] = cos(q_05)*(cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) - 1.0*sin(q_05)*(sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) + cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[4] = cos(q_05)*(cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))) - 1.0*sin(q_05)*(sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))); 
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

	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_1 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_2 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);

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
		std::vector<double> start (5);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		std::vector<double> endEff (2);
		endEff[0] = -0.09403;
		endEff[1] = -1.8879;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		std::vector<double> endEff (2);
		endEff[0] = -4.0168;
		endEff[1] = -0.13946;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		std::vector<double> endEff (2);
		endEff[0] = -2.6152;
		endEff[1] = 3.7334;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		std::vector<double> endEff (2);
		endEff[0] = -2.0873;
		endEff[1] = -2.2549;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		std::vector<double> endEff (2);
		endEff[0] = -0.40931;
		endEff[1] = 2.9032;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		std::vector<double> endEff (2);
		endEff[0] = -0.95379;
		endEff[1] = -1.7733;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		std::vector<double> endEff (2);
		endEff[0] = 2.5891;
		endEff[1] = 0.17628;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		std::vector<double> endEff (2);
		endEff[0] = 2.2239;
		endEff[1] = -1.3385;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		std::vector<double> endEff (2);
		endEff[0] = 0.33335;
		endEff[1] = 1.7269;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.033928;
		start[1] = -0.076058;
		start[2] = 0.086063;
		start[3] = 0.073272;
		start[4] = -0.010986;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.079662;
		start[1] = 0.097787;
		start[2] = 0.0012307;
		start[3] = 0.0088776;
		start[4] = -0.070664;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.029174;
		start[1] = 0.03849;
		start[2] = 0.08846;
		start[3] = 0.075201;
		start[4] = -0.088315;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.074133;
		start[1] = 0.015749;
		start[2] = 0.092538;
		start[3] = -0.055621;
		start[4] = 0.035724;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.026144;
		start[1] = -0.041687;
		start[2] = -0.077881;
		start[3] = 0.013521;
		start[4] = -0.092668;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = -0.0093254;
		start[1] = -0.037215;
		start[2] = -0.0020639;
		start[3] = 0.087869;
		start[4] = 0.035753;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.03201;
		start[1] = -0.013917;
		start[2] = 0.055645;
		start[3] = -0.0031786;
		start[4] = -0.079085;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.055958;
		start[1] = 0.010662;
		start[2] = 0.0026134;
		start[3] = -0.061943;
		start[4] = -0.09929;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.016664;
		start[1] = 0.0892;
		start[2] = -0.078576;
		start[3] = -0.081146;
		start[4] = 0.075104;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (5);
		start[0] = 0.057803;
		start[1] = -0.0078275;
		start[2] = -0.0015922;
		start[3] = -0.010119;
		start[4] = 0.096248;
		std::vector<double> endEff (2);
		endEff[0] = 1.1085;
		endEff[1] = 3.3765;
		solveFor( start, endEff);
	}


  return 0;
}
