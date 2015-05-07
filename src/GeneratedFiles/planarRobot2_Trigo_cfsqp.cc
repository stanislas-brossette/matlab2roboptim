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
    (8, 1, "CostFunction_planarRobot2"),
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];
  
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];

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
    (8, 1, "LiftConstraint_1_planarRobot2"),
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];
  
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];

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
    (8, 1, "LiftConstraint_2_planarRobot2"),
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];
  
	result[0] = pow(w_01_03,2) + pow(w_01_04,2) - 1.0;
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 2.0*w_01_03; 
			 grad[3] = 2.0*w_01_04; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
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
    (8, 1, "LiftConstraint_3_planarRobot2"),
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];
  
	result[0] = w_01_01*w_01_03 - 1.0*w_02_01;
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];

  switch (id)
    {
      
		case 0: 
			 grad[0] = w_01_03; 
			 grad[1] = 0.0; 
			 grad[2] = w_01_01; 
			 grad[3] = 0.0; 
			 grad[4] = -1.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
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
    (8, 1, "LiftConstraint_4_planarRobot2"),
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];
  
	result[0] = w_01_02*w_01_03 - 1.0*w_02_02;
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = w_01_03; 
			 grad[2] = w_01_02; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = -1.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
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
    (8, 1, "LiftConstraint_5_planarRobot2"),
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];
  
	result[0] = w_01_01*w_01_04 - 1.0*w_02_03;
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];

  switch (id)
    {
      
		case 0: 
			 grad[0] = w_01_04; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_01_01; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = -1.0; 
			 grad[7] = 0.0; 
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
    (8, 1, "LiftConstraint_6_planarRobot2"),
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];
  
	result[0] = w_01_02*w_01_04 - 1.0*w_02_04;
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = w_01_04; 
			 grad[2] = 0.0; 
			 grad[3] = w_01_02; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = -1.0; 
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
    (8, 1, "EEConstraint_1_planarRobot2"),
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];
  
	result[0] = w_01_01 - 1.0*EE_1_1 + w_02_01 - 1.0*w_02_04;
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 1.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 1.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = -1.0; 
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
    (8, 1, "EEConstraint_2_planarRobot2"),
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];
  
	result[0] = w_01_02 - 1.0*EE_1_2 + w_02_02 + w_02_03;
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
	const double& w_02_01 = x[4];
	const double& w_02_02 = x[5];
	const double& w_02_03 = x[6];
	const double& w_02_04 = x[7];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 1.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 1.0; 
			 grad[6] = 1.0; 
			 grad[7] = 0.0; 
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
	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_7 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_8 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);

  //Create problem
  solver_t::problem_t pb (*cost);

  // Set bounds for all optimization parameters
  pb.argumentBounds ()[0] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[1] = roboptim::Function::makeInterval (-pi, pi);

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
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_7), bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_8), bounds, scales); 
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
		std::vector<double> start (8);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0084453;
		start[3] = 0.0082414;
		start[4] = 0.0075267;
		start[5] = 0.0091681;
		start[6] = 0.004514;
		start[7] = 0.0070592;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0067636;
		start[3] = 0.0040706;
		start[4] = 0.0090959;
		start[5] = 0.0086144;
		start[6] = 0.0023482;
		start[7] = 0.0056461;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0050037;
		start[3] = 0.0094757;
		start[4] = 0.00016323;
		start[5] = 0.0030962;
		start[6] = 0.0096642;
		start[7] = 0.0084838;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.00099305;
		start[3] = 0.0004152;
		start[4] = 0.0090912;
		start[5] = 0.0055382;
		start[6] = 0.0084235;
		start[7] = 0.003135;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0052003;
		start[3] = 0.0019535;
		start[4] = 0.00013121;
		start[5] = 0.0094655;
		start[6] = 0.0053397;
		start[7] = 0.0010044;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.004514;
		start[3] = 0.0074855;
		start[4] = 0.0053209;
		start[5] = 0.0092846;
		start[6] = 0.0039492;
		start[7] = 0.0016016;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0084649;
		start[3] = 0.0016375;
		start[4] = 0.0085406;
		start[5] = 0.0084316;
		start[6] = 0.0048447;
		start[7] = 0.0077395;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0084543;
		start[3] = 0.0039974;
		start[4] = 0.0055993;
		start[5] = 0.0051727;
		start[6] = 0.0029223;
		start[7] = 0.0092161;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0040301;
		start[3] = 0.0090937;
		start[4] = 0.0048359;
		start[5] = 0.0040913;
		start[6] = 0.0069906;
		start[7] = 0.0084015;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0019082;
		start[3] = 0.0086644;
		start[4] = 0.0061676;
		start[5] = 0.0044225;
		start[6] = 0.0065396;
		start[7] = 0.0011844;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0071203;
		start[3] = 0.0010294;
		start[4] = 0.0056721;
		start[5] = 0.0073733;
		start[6] = 0.0091897;
		start[7] = 0.0087961;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0037025;
		start[3] = 0.0070904;
		start[4] = 0.0038078;
		start[5] = 0.0044511;
		start[6] = 0.0023199;
		start[7] = 0.005839;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.00014234;
		start[3] = 0.0040855;
		start[4] = 0.0038337;
		start[5] = 0.0014365;
		start[6] = 0.0073985;
		start[7] = 0.0032479;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0038294;
		start[3] = 0.0098905;
		start[4] = 0.00623;
		start[5] = 0.0040398;
		start[6] = 0.0023231;
		start[7] = 0.0096562;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0014481;
		start[3] = 0.0040102;
		start[4] = 0.0053205;
		start[5] = 0.0010095;
		start[6] = 0.0058078;
		start[7] = 0.0010449;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0059587;
		start[3] = 0.0047811;
		start[4] = 0.0081629;
		start[5] = 0.005737;
		start[6] = 0.0021062;
		start[7] = 0.0043105;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0043279;
		start[3] = 0.0087123;
		start[4] = 0.00747;
		start[5] = 0.0039664;
		start[6] = 0.0016646;
		start[7] = 0.0054183;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.00020255;
		start[3] = 0.0012034;
		start[4] = 0.0063411;
		start[5] = 0.0048479;
		start[6] = 0.0099483;
		start[7] = 0.0065897;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0097233;
		start[3] = 0.008807;
		start[4] = 0.0070458;
		start[5] = 0.00083358;
		start[6] = 0.0044178;
		start[7] = 5.2199e-06;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0067862;
		start[3] = 0.0077484;
		start[4] = 0.0091188;
		start[5] = 0.0035425;
		start[6] = 0.0054383;
		start[7] = 0.0065316;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0017754;
		start[3] = 0.0098479;
		start[4] = 2.7325e-05;
		start[5] = 0.0019577;
		start[6] = 0.0046825;
		start[7] = 0.0055689;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.002541;
		start[3] = 0.0099496;
		start[4] = 0.0044664;
		start[5] = 0.00055288;
		start[6] = 0.0069643;
		start[7] = 0.0023766;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0029969;
		start[3] = 0.0025605;
		start[4] = 0.00087146;
		start[5] = 0.0056742;
		start[6] = 0.0026614;
		start[7] = 0.00032025;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0064009;
		start[3] = 0.006009;
		start[4] = 0.0019107;
		start[5] = 0.0068791;
		start[6] = 0.0031048;
		start[7] = 0.004265;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0038152;
		start[3] = 0.0062468;
		start[4] = 0.0078455;
		start[5] = 0.0059463;
		start[6] = 0.0065158;
		start[7] = 0.0058943;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0015915;
		start[3] = 0.00042133;
		start[4] = 0.0013702;
		start[5] = 0.0028267;
		start[6] = 0.0075692;
		start[7] = 0.0041502;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0014565;
		start[3] = 0.0073402;
		start[4] = 0.006695;
		start[5] = 0.0063547;
		start[6] = 0.0054285;
		start[7] = 0.0025207;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.000739;
		start[3] = 0.0038718;
		start[4] = 0.0095467;
		start[5] = 0.00024173;
		start[6] = 0.0059134;
		start[7] = 0.0088438;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.005819;
		start[3] = 0.0071632;
		start[4] = 0.0071658;
		start[5] = 0.0037915;
		start[6] = 0.0015427;
		start[7] = 0.001575;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0046256;
		start[3] = 0.00035016;
		start[4] = 0.0072649;
		start[5] = 0.0075187;
		start[6] = 0.0045059;
		start[7] = 0.0028686;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0014233;
		start[3] = 0.004937;
		start[4] = 0.0036511;
		start[5] = 0.0078088;
		start[6] = 0.0013942;
		start[7] = 0.0074641;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.006206;
		start[3] = 0.0090776;
		start[4] = 0.0071788;
		start[5] = 0.0011755;
		start[6] = 0.0087068;
		start[7] = 0.00065958;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 1.3468e-05;
		start[3] = 0.0030667;
		start[4] = 0.0039632;
		start[5] = 0.0028592;
		start[6] = 0.0064905;
		start[7] = 0.0026023;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0098604;
		start[3] = 0.0067291;
		start[4] = 0.0066725;
		start[5] = 0.0078936;
		start[6] = 0.00069405;
		start[7] = 0.0024325;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0050987;
		start[3] = 0.0086573;
		start[4] = 0.0099739;
		start[5] = 0.0060983;
		start[6] = 0.0002318;
		start[7] = 0.0090563;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0078738;
		start[3] = 0.0050855;
		start[4] = 0.0075462;
		start[5] = 0.0061101;
		start[6] = 0.0042215;
		start[7] = 0.0024658;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0020631;
		start[3] = 0.0032351;
		start[4] = 0.0035683;
		start[5] = 0.00408;
		start[6] = 0.005144;
		start[7] = 0.0090879;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0011694;
		start[3] = 0.0018004;
		start[4] = 0.0046911;
		start[5] = 0.0031214;
		start[6] = 0.0093196;
		start[7] = 0.0087776;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0098941;
		start[3] = 0.0019438;
		start[4] = 0.0081941;
		start[5] = 0.0061421;
		start[6] = 0.0023523;
		start[7] = 0.0079211;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0043343;
		start[3] = 0.0012968;
		start[4] = 0.0057025;
		start[5] = 0.0012784;
		start[6] = 0.0021793;
		start[7] = 0.0046776;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0048974;
		start[3] = 0.0019345;
		start[4] = 0.0012013;
		start[5] = 0.0067558;
		start[6] = 0.0099016;
		start[7] = 0.0065719;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0022641;
		start[3] = 0.0082799;
		start[4] = 0.0037267;
		start[5] = 0.0069747;
		start[6] = 0.0064495;
		start[7] = 0.00025331;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0029773;
		start[3] = 0.0090562;
		start[4] = 0.0065823;
		start[5] = 0.0030199;
		start[6] = 0.0058084;
		start[7] = 0.0037065;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0016907;
		start[3] = 0.0004142;
		start[4] = 0.0098237;
		start[5] = 0.0058489;
		start[6] = 0.0061813;
		start[7] = 0.0028002;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0090224;
		start[3] = 0.0043998;
		start[4] = 0.0024269;
		start[5] = 0.00043488;
		start[6] = 0.001219;
		start[7] = 0.0049738;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0070091;
		start[3] = 0.0032308;
		start[4] = 0.00012864;
		start[5] = 0.0065078;
		start[6] = 0.0018991;
		start[7] = 0.0052025;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.009264;
		start[3] = 0.0046753;
		start[4] = 0.0086789;
		start[5] = 0.0042194;
		start[6] = 0.0088762;
		start[7] = 0.0037149;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0013245;
		start[3] = 0.0042937;
		start[4] = 0.0039;
		start[5] = 0.0098253;
		start[6] = 0.0097958;
		start[7] = 0.0030746;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0093188;
		start[3] = 0.0079949;
		start[4] = 0.0093741;
		start[5] = 0.0061928;
		start[6] = 0.0036781;
		start[7] = 0.004093;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0071957;
		start[3] = 0.0063328;
		start[4] = 0.0057363;
		start[5] = 0.0081443;
		start[6] = 0.0014637;
		start[7] = 0.0026638;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0037448;
		start[3] = 0.0010306;
		start[4] = 0.0073937;
		start[5] = 0.0032602;
		start[6] = 0.0059541;
		start[7] = 0.0089189;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.002879;
		start[3] = 0.0074153;
		start[4] = 0.0047657;
		start[5] = 0.0041475;
		start[6] = 0.0082769;
		start[7] = 0.0052445;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0047624;
		start[3] = 0.0086079;
		start[4] = 0.0013066;
		start[5] = 0.0012457;
		start[6] = 0.0013217;
		start[7] = 0.0065271;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0097823;
		start[3] = 0.0058863;
		start[4] = 0.0041723;
		start[5] = 8.9615e-05;
		start[6] = 0.0089584;
		start[7] = 0.0097529;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0099142;
		start[3] = 0.0092228;
		start[4] = 0.0018216;
		start[5] = 0.0056208;
		start[6] = 0.0049994;
		start[7] = 0.0086441;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.003686;
		start[3] = 0.0090352;
		start[4] = 0.006153;
		start[5] = 0.007721;
		start[6] = 0.0073157;
		start[7] = 0.0097356;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0079141;
		start[3] = 0.0046016;
		start[4] = 0.0076492;
		start[5] = 0.009721;
		start[6] = 0.0025603;
		start[7] = 0.0091574;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0033459;
		start[3] = 0.0018145;
		start[4] = 0.0043434;
		start[5] = 0.0076057;
		start[6] = 0.0091636;
		start[7] = 0.0050179;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0037472;
		start[3] = 0.0077158;
		start[4] = 0.0018976;
		start[5] = 0.0047699;
		start[6] = 0.0058116;
		start[7] = 0.0085736;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0067146;
		start[3] = 0.0019345;
		start[4] = 0.0093351;
		start[5] = 0.0059893;
		start[6] = 0.006539;
		start[7] = 0.0037543;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0032807;
		start[3] = 0.00032114;
		start[4] = 0.0043845;
		start[5] = 0.0082111;
		start[6] = 0.006523;
		start[7] = 0.0099447;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0059987;
		start[3] = 0.0051867;
		start[4] = 0.00072206;
		start[5] = 0.0066295;
		start[6] = 0.0041011;
		start[7] = 0.0025984;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0076363;
		start[3] = 0.0088071;
		start[4] = 0.0054836;
		start[5] = 0.0045146;
		start[6] = 0.0093526;
		start[7] = 0.00083413;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0036804;
		start[3] = 0.00033132;
		start[4] = 0.00046005;
		start[5] = 0.0064059;
		start[6] = 0.0051019;
		start[7] = 0.005618;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0028145;
		start[3] = 0.0067873;
		start[4] = 0.0062686;
		start[5] = 0.0070895;
		start[6] = 0.0061537;
		start[7] = 0.0044957;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0072749;
		start[3] = 0.0098481;
		start[4] = 0.0053448;
		start[5] = 0.0077988;
		start[6] = 0.0099153;
		start[7] = 0.0063659;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0027327;
		start[3] = 0.0060308;
		start[4] = 0.0063604;
		start[5] = 0.0091109;
		start[6] = 0.0061564;
		start[7] = 0.0094366;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0025573;
		start[3] = 0.0019821;
		start[4] = 0.0044387;
		start[5] = 0.0087717;
		start[6] = 0.0034839;
		start[7] = 0.0010752;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0076474;
		start[3] = 0.0089628;
		start[4] = 0.0018864;
		start[5] = 0.0067352;
		start[6] = 0.0051589;
		start[7] = 0.0086196;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0029815;
		start[3] = 0.003033;
		start[4] = 0.002211;
		start[5] = 0.0013983;
		start[6] = 0.0095901;
		start[7] = 0.0068068;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0080479;
		start[3] = 0.0053621;
		start[4] = 0.0082146;
		start[5] = 0.0026243;
		start[6] = 0.0069571;
		start[7] = 0.0085103;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.00087153;
		start[3] = 0.0096579;
		start[4] = 0.0070092;
		start[5] = 0.00075536;
		start[6] = 0.0094284;
		start[7] = 0.0096246;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0038875;
		start[3] = 0.0057694;
		start[4] = 0.0081716;
		start[5] = 0.00076544;
		start[6] = 0.0086531;
		start[7] = 0.0018137;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0083467;
		start[3] = 0.0084104;
		start[4] = 0.0050862;
		start[5] = 0.0052002;
		start[6] = 0.003669;
		start[7] = 0.009236;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0070023;
		start[3] = 0.0099073;
		start[4] = 0.0096897;
		start[5] = 0.00092728;
		start[6] = 0.003706;
		start[7] = 0.0053016;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.00080593;
		start[3] = 0.0070431;
		start[4] = 0.0034697;
		start[5] = 0.0011229;
		start[6] = 0.009419;
		start[7] = 0.0020735;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 9.8048e-05;
		start[3] = 0.0086856;
		start[4] = 0.0067863;
		start[5] = 0.001717;
		start[6] = 0.0017666;
		start[7] = 0.0089586;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.00080977;
		start[3] = 0.00071843;
		start[4] = 0.00023435;
		start[5] = 0.0057251;
		start[6] = 0.0032539;
		start[7] = 0.0052491;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0040191;
		start[3] = 0.0062923;
		start[4] = 0.0063162;
		start[5] = 0.0034268;
		start[6] = 0.0095666;
		start[7] = 0.0033334;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0067104;
		start[3] = 0.00026174;
		start[4] = 0.0086177;
		start[5] = 0.0033517;
		start[6] = 0.0092007;
		start[7] = 0.0024608;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.00077658;
		start[3] = 0.0028344;
		start[4] = 0.0030294;
		start[5] = 0.0062216;
		start[6] = 0.0058178;
		start[7] = 0.0069232;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0014356;
		start[3] = 0.00348;
		start[4] = 0.0062339;
		start[5] = 0.0085898;
		start[6] = 0.0047157;
		start[7] = 0.0069072;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.00093759;
		start[3] = 0.00057598;
		start[4] = 0.008252;
		start[5] = 0.0009079;
		start[6] = 0.0020638;
		start[7] = 0.0079324;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0068173;
		start[3] = 0.0046999;
		start[4] = 0.0089777;
		start[5] = 0.0040853;
		start[6] = 0.0065855;
		start[7] = 0.0095842;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0030182;
		start[3] = 0.0072486;
		start[4] = 0.007519;
		start[5] = 0.0018563;
		start[6] = 0.0056838;
		start[7] = 0.00061113;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0068328;
		start[3] = 0.0066873;
		start[4] = 0.001342;
		start[5] = 0.008931;
		start[6] = 0.0016966;
		start[7] = 0.0054411;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.00034614;
		start[3] = 0.0081456;
		start[4] = 0.005951;
		start[5] = 0.004058;
		start[6] = 0.0064404;
		start[7] = 0.0036634;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0099997;
		start[3] = 0.0070605;
		start[4] = 0.001968;
		start[5] = 0.0082957;
		start[6] = 0.0030249;
		start[7] = 0.0052302;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0062486;
		start[3] = 4.7512e-05;
		start[4] = 0.0077537;
		start[5] = 0.002124;
		start[6] = 0.0020398;
		start[7] = 0.007948;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.002699;
		start[3] = 0.0083079;
		start[4] = 0.0099726;
		start[5] = 0.0039538;
		start[6] = 0.0089964;
		start[7] = 0.0053084;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.00082251;
		start[3] = 0.0041855;
		start[4] = 0.00018904;
		start[5] = 0.0090152;
		start[6] = 0.0037496;
		start[7] = 0.00011844;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0050115;
		start[3] = 0.0044338;
		start[4] = 0.0085244;
		start[5] = 0.0050086;
		start[6] = 0.001122;
		start[7] = 0.007914;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0093483;
		start[3] = 0.0085589;
		start[4] = 0.0015501;
		start[5] = 0.00081301;
		start[6] = 0.0062826;
		start[7] = 0.00096957;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.00033433;
		start[3] = 0.0032361;
		start[4] = 0.0064505;
		start[5] = 0.0058428;
		start[6] = 0.0067218;
		start[7] = 0.0079269;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0038019;
		start[3] = 0.00079293;
		start[4] = 0.0043419;
		start[5] = 0.0078547;
		start[6] = 0.0030488;
		start[7] = 0.0073403;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.001002;
		start[3] = 0.0017277;
		start[4] = 0.0092512;
		start[5] = 0.0078842;
		start[6] = 0.0077883;
		start[7] = 0.0087731;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0090972;
		start[3] = 0.0093039;
		start[4] = 0.0040401;
		start[5] = 0.0028915;
		start[6] = 0.0076871;
		start[7] = 0.0094388;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.002794;
		start[3] = 0.0037113;
		start[4] = 0.0027321;
		start[5] = 0.0047354;
		start[6] = 0.0096054;
		start[7] = 0.0010176;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0054064;
		start[3] = 0.0030753;
		start[4] = 0.0015165;
		start[5] = 0.0042262;
		start[6] = 0.0032421;
		start[7] = 0.0055641;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (8);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0018444;
		start[3] = 0.002149;
		start[4] = 0.0056526;
		start[5] = 0.0090865;
		start[6] = 0.006999;
		start[7] = 0.0088546;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}


  return 0;
}
