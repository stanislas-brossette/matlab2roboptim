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
    (10, 1, "CostFunction_planarRobot2"),
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
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
  
	result[0] = 0.0;
}

template <typename T>
void
CostFunction<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];

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
    (10, 1, "LiftConstraint_1_planarRobot2"),
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
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
  
	result[0] = cos(q_01) - 1.0*w_01_01;
}

template <typename T>
void
LiftConstraint_1<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];

  switch (id)
    {
      
		case 0: 
			 grad[0] = -1.0*sin(q_01); 
			 grad[1] = 0.0; 
			 grad[2] = -1.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
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
    (10, 1, "LiftConstraint_2_planarRobot2"),
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
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
  
	result[0] = sin(q_01) - 1.0*w_01_02;
}

template <typename T>
void
LiftConstraint_2<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];

  switch (id)
    {
      
		case 0: 
			 grad[0] = cos(q_01); 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = -1.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
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
    (10, 1, "LiftConstraint_3_planarRobot2"),
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
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
  
	result[0] = cos(q_02) - 1.0*w_01_03;
}

template <typename T>
void
LiftConstraint_3<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = -1.0*sin(q_02); 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = -1.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
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
    (10, 1, "LiftConstraint_4_planarRobot2"),
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
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
  
	result[0] = sin(q_02) - 1.0*w_01_04;
}

template <typename T>
void
LiftConstraint_4<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = cos(q_02); 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = -1.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
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
    (10, 1, "LiftConstraint_5_planarRobot2"),
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
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
  
	result[0] = w_01_01*w_01_03 - 1.0*w_02_01;
}

template <typename T>
void
LiftConstraint_5<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = w_01_03; 
			 grad[3] = 0.0; 
			 grad[4] = w_01_01; 
			 grad[5] = 0.0; 
			 grad[6] = -1.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
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
    (10, 1, "LiftConstraint_6_planarRobot2"),
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
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
  
	result[0] = w_01_02*w_01_03 - 1.0*w_02_02;
}

template <typename T>
void
LiftConstraint_6<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_01_03; 
			 grad[4] = w_01_02; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = -1.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
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
    (10, 1, "LiftConstraint_7_planarRobot2"),
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
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
  
	result[0] = w_01_01*w_01_04 - 1.0*w_02_03;
}

template <typename T>
void
LiftConstraint_7<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = w_01_04; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_01_01; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = -1.0; 
			 grad[9] = 0.0; 
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
    (10, 1, "LiftConstraint_8_planarRobot2"),
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
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
  
	result[0] = w_01_02*w_01_04 - 1.0*w_02_04;
}

template <typename T>
void
LiftConstraint_8<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_01_04; 
			 grad[4] = 0.0; 
			 grad[5] = w_01_02; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = -1.0; 
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
    (10, 1, "EEConstraint_1_planarRobot2"),
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
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
  
	result[0] = w_01_01 - 1.0*EE_1_1 + w_02_01 - 1.0*w_02_04;
}

template <typename T>
void
EEConstraint_1<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 1.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 1.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = -1.0; 
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
    (10, 1, "EEConstraint_2_planarRobot2"),
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
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
  
	result[0] = w_01_02 - 1.0*EE_1_2 + w_02_02 + w_02_03;
}

template <typename T>
void
EEConstraint_2<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& w_01_01 = x[2];
	const double& w_01_02 = x[3];
	const double& w_01_03 = x[4];
	const double& w_01_04 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];

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
			 grad[7] = 1.0; 
			 grad[8] = 1.0; 
			 grad[9] = 0.0; 
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
	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_9 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_10 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);

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
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_9), bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_10), bounds, scales); 
	}

  pb.startingPoint () = start;
  roboptim::SolverFactory<solver_t> factory ("cfsqp", pb);
  solver_t& solver = factory ();

  {
    boost::timer::auto_cpu_timer t;
    for( int i = 0; i<100; i++)
    {
      solver.solve();
    }
  }

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
		std::vector<double> start (10);
		start[0] = 0.73597;
		start[1] = 0.79468;
		start[2] = 0.74118;
		start[3] = 0.6713;
		start[4] = 0.70051;
		start[5] = 0.71364;
		start[6] = 0.51921;
		start[7] = 0.47026;
		start[8] = 0.52894;
		start[9] = 0.47907;
		std::vector<double> endEff (2);
		endEff[0] = -0.24915;
		endEff[1] = -0.18122;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.54491;
		start[1] = 0.68622;
		start[2] = 0.85518;
		start[3] = 0.51834;
		start[4] = 0.77364;
		start[5] = 0.63362;
		start[6] = 0.6616;
		start[7] = 0.40101;
		start[8] = 0.54186;
		start[9] = 0.32843;
		std::vector<double> endEff (2);
		endEff[0] = -0.24915;
		endEff[1] = -0.18122;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.89363;
		start[1] = 0.054792;
		start[2] = 0.62659;
		start[3] = 0.77935;
		start[4] = 0.9985;
		start[5] = 0.054764;
		start[6] = 0.62564;
		start[7] = 0.77818;
		start[8] = 0.034315;
		start[9] = 0.042681;
		std::vector<double> endEff (2);
		endEff[0] = -0.24915;
		endEff[1] = -0.18122;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.30366;
		start[1] = 0.046192;
		start[2] = 0.95425;
		start[3] = 0.29902;
		start[4] = 0.99893;
		start[5] = 0.046175;
		start[6] = 0.95323;
		start[7] = 0.2987;
		start[8] = 0.044063;
		start[9] = 0.013807;
		std::vector<double> endEff (2);
		endEff[0] = -0.24915;
		endEff[1] = -0.18122;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.19548;
		start[1] = 0.72017;
		start[2] = 0.98096;
		start[3] = 0.19423;
		start[4] = 0.7517;
		start[5] = 0.65951;
		start[6] = 0.73738;
		start[7] = 0.14601;
		start[8] = 0.64695;
		start[9] = 0.1281;
		std::vector<double> endEff (2);
		endEff[0] = -0.24915;
		endEff[1] = -0.18122;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.72175;
		start[1] = 0.8778;
		start[2] = 0.75065;
		start[3] = 0.6607;
		start[4] = 0.63885;
		start[5] = 0.76933;
		start[6] = 0.47955;
		start[7] = 0.42209;
		start[8] = 0.5775;
		start[9] = 0.5083;
		std::vector<double> endEff (2);
		endEff[0] = -0.24915;
		endEff[1] = -0.18122;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.58243;
		start[1] = 0.070684;
		start[2] = 0.83513;
		start[3] = 0.55006;
		start[4] = 0.9975;
		start[5] = 0.070625;
		start[6] = 0.83304;
		start[7] = 0.54868;
		start[8] = 0.058981;
		start[9] = 0.038848;
		std::vector<double> endEff (2);
		endEff[0] = -0.24915;
		endEff[1] = -0.18122;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.92274;
		start[1] = 0.80037;
		start[2] = 0.60363;
		start[3] = 0.79726;
		start[4] = 0.69644;
		start[5] = 0.71762;
		start[6] = 0.42039;
		start[7] = 0.55524;
		start[8] = 0.43318;
		start[9] = 0.57213;
		std::vector<double> endEff (2);
		endEff[0] = -0.24915;
		endEff[1] = -0.18122;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.28595;
		start[1] = 0.54366;
		start[2] = 0.9594;
		start[3] = 0.28207;
		start[4] = 0.85582;
		start[5] = 0.51727;
		start[6] = 0.82107;
		start[7] = 0.2414;
		start[8] = 0.49627;
		start[9] = 0.14591;
		std::vector<double> endEff (2);
		endEff[0] = -0.24915;
		endEff[1] = -0.18122;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.98478;
		start[1] = 0.71568;
		start[2] = 0.55305;
		start[3] = 0.83315;
		start[4] = 0.75465;
		start[5] = 0.65613;
		start[6] = 0.41736;
		start[7] = 0.62873;
		start[8] = 0.36287;
		start[9] = 0.54665;
		std::vector<double> endEff (2);
		endEff[0] = -0.24915;
		endEff[1] = -0.18122;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.73597;
		start[1] = 0.79468;
		start[2] = 0.74118;
		start[3] = 0.6713;
		start[4] = 0.70051;
		start[5] = 0.71364;
		start[6] = 0.51921;
		start[7] = 0.47026;
		start[8] = 0.52894;
		start[9] = 0.47907;
		std::vector<double> endEff (2);
		endEff[0] = 0.90489;
		endEff[1] = 0.30243;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.54491;
		start[1] = 0.68622;
		start[2] = 0.85518;
		start[3] = 0.51834;
		start[4] = 0.77364;
		start[5] = 0.63362;
		start[6] = 0.6616;
		start[7] = 0.40101;
		start[8] = 0.54186;
		start[9] = 0.32843;
		std::vector<double> endEff (2);
		endEff[0] = 0.90489;
		endEff[1] = 0.30243;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.89363;
		start[1] = 0.054792;
		start[2] = 0.62659;
		start[3] = 0.77935;
		start[4] = 0.9985;
		start[5] = 0.054764;
		start[6] = 0.62564;
		start[7] = 0.77818;
		start[8] = 0.034315;
		start[9] = 0.042681;
		std::vector<double> endEff (2);
		endEff[0] = 0.90489;
		endEff[1] = 0.30243;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.30366;
		start[1] = 0.046192;
		start[2] = 0.95425;
		start[3] = 0.29902;
		start[4] = 0.99893;
		start[5] = 0.046175;
		start[6] = 0.95323;
		start[7] = 0.2987;
		start[8] = 0.044063;
		start[9] = 0.013807;
		std::vector<double> endEff (2);
		endEff[0] = 0.90489;
		endEff[1] = 0.30243;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.19548;
		start[1] = 0.72017;
		start[2] = 0.98096;
		start[3] = 0.19423;
		start[4] = 0.7517;
		start[5] = 0.65951;
		start[6] = 0.73738;
		start[7] = 0.14601;
		start[8] = 0.64695;
		start[9] = 0.1281;
		std::vector<double> endEff (2);
		endEff[0] = 0.90489;
		endEff[1] = 0.30243;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.72175;
		start[1] = 0.8778;
		start[2] = 0.75065;
		start[3] = 0.6607;
		start[4] = 0.63885;
		start[5] = 0.76933;
		start[6] = 0.47955;
		start[7] = 0.42209;
		start[8] = 0.5775;
		start[9] = 0.5083;
		std::vector<double> endEff (2);
		endEff[0] = 0.90489;
		endEff[1] = 0.30243;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.58243;
		start[1] = 0.070684;
		start[2] = 0.83513;
		start[3] = 0.55006;
		start[4] = 0.9975;
		start[5] = 0.070625;
		start[6] = 0.83304;
		start[7] = 0.54868;
		start[8] = 0.058981;
		start[9] = 0.038848;
		std::vector<double> endEff (2);
		endEff[0] = 0.90489;
		endEff[1] = 0.30243;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.92274;
		start[1] = 0.80037;
		start[2] = 0.60363;
		start[3] = 0.79726;
		start[4] = 0.69644;
		start[5] = 0.71762;
		start[6] = 0.42039;
		start[7] = 0.55524;
		start[8] = 0.43318;
		start[9] = 0.57213;
		std::vector<double> endEff (2);
		endEff[0] = 0.90489;
		endEff[1] = 0.30243;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.28595;
		start[1] = 0.54366;
		start[2] = 0.9594;
		start[3] = 0.28207;
		start[4] = 0.85582;
		start[5] = 0.51727;
		start[6] = 0.82107;
		start[7] = 0.2414;
		start[8] = 0.49627;
		start[9] = 0.14591;
		std::vector<double> endEff (2);
		endEff[0] = 0.90489;
		endEff[1] = 0.30243;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.98478;
		start[1] = 0.71568;
		start[2] = 0.55305;
		start[3] = 0.83315;
		start[4] = 0.75465;
		start[5] = 0.65613;
		start[6] = 0.41736;
		start[7] = 0.62873;
		start[8] = 0.36287;
		start[9] = 0.54665;
		std::vector<double> endEff (2);
		endEff[0] = 0.90489;
		endEff[1] = 0.30243;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.73597;
		start[1] = 0.79468;
		start[2] = 0.74118;
		start[3] = 0.6713;
		start[4] = 0.70051;
		start[5] = 0.71364;
		start[6] = 0.51921;
		start[7] = 0.47026;
		start[8] = 0.52894;
		start[9] = 0.47907;
		std::vector<double> endEff (2);
		endEff[0] = 0.51819;
		endEff[1] = -0.43009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.54491;
		start[1] = 0.68622;
		start[2] = 0.85518;
		start[3] = 0.51834;
		start[4] = 0.77364;
		start[5] = 0.63362;
		start[6] = 0.6616;
		start[7] = 0.40101;
		start[8] = 0.54186;
		start[9] = 0.32843;
		std::vector<double> endEff (2);
		endEff[0] = 0.51819;
		endEff[1] = -0.43009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.89363;
		start[1] = 0.054792;
		start[2] = 0.62659;
		start[3] = 0.77935;
		start[4] = 0.9985;
		start[5] = 0.054764;
		start[6] = 0.62564;
		start[7] = 0.77818;
		start[8] = 0.034315;
		start[9] = 0.042681;
		std::vector<double> endEff (2);
		endEff[0] = 0.51819;
		endEff[1] = -0.43009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.30366;
		start[1] = 0.046192;
		start[2] = 0.95425;
		start[3] = 0.29902;
		start[4] = 0.99893;
		start[5] = 0.046175;
		start[6] = 0.95323;
		start[7] = 0.2987;
		start[8] = 0.044063;
		start[9] = 0.013807;
		std::vector<double> endEff (2);
		endEff[0] = 0.51819;
		endEff[1] = -0.43009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.19548;
		start[1] = 0.72017;
		start[2] = 0.98096;
		start[3] = 0.19423;
		start[4] = 0.7517;
		start[5] = 0.65951;
		start[6] = 0.73738;
		start[7] = 0.14601;
		start[8] = 0.64695;
		start[9] = 0.1281;
		std::vector<double> endEff (2);
		endEff[0] = 0.51819;
		endEff[1] = -0.43009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.72175;
		start[1] = 0.8778;
		start[2] = 0.75065;
		start[3] = 0.6607;
		start[4] = 0.63885;
		start[5] = 0.76933;
		start[6] = 0.47955;
		start[7] = 0.42209;
		start[8] = 0.5775;
		start[9] = 0.5083;
		std::vector<double> endEff (2);
		endEff[0] = 0.51819;
		endEff[1] = -0.43009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.58243;
		start[1] = 0.070684;
		start[2] = 0.83513;
		start[3] = 0.55006;
		start[4] = 0.9975;
		start[5] = 0.070625;
		start[6] = 0.83304;
		start[7] = 0.54868;
		start[8] = 0.058981;
		start[9] = 0.038848;
		std::vector<double> endEff (2);
		endEff[0] = 0.51819;
		endEff[1] = -0.43009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.92274;
		start[1] = 0.80037;
		start[2] = 0.60363;
		start[3] = 0.79726;
		start[4] = 0.69644;
		start[5] = 0.71762;
		start[6] = 0.42039;
		start[7] = 0.55524;
		start[8] = 0.43318;
		start[9] = 0.57213;
		std::vector<double> endEff (2);
		endEff[0] = 0.51819;
		endEff[1] = -0.43009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.28595;
		start[1] = 0.54366;
		start[2] = 0.9594;
		start[3] = 0.28207;
		start[4] = 0.85582;
		start[5] = 0.51727;
		start[6] = 0.82107;
		start[7] = 0.2414;
		start[8] = 0.49627;
		start[9] = 0.14591;
		std::vector<double> endEff (2);
		endEff[0] = 0.51819;
		endEff[1] = -0.43009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.98478;
		start[1] = 0.71568;
		start[2] = 0.55305;
		start[3] = 0.83315;
		start[4] = 0.75465;
		start[5] = 0.65613;
		start[6] = 0.41736;
		start[7] = 0.62873;
		start[8] = 0.36287;
		start[9] = 0.54665;
		std::vector<double> endEff (2);
		endEff[0] = 0.51819;
		endEff[1] = -0.43009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.73597;
		start[1] = 0.79468;
		start[2] = 0.74118;
		start[3] = 0.6713;
		start[4] = 0.70051;
		start[5] = 0.71364;
		start[6] = 0.51921;
		start[7] = 0.47026;
		start[8] = 0.52894;
		start[9] = 0.47907;
		std::vector<double> endEff (2);
		endEff[0] = 0.43698;
		endEff[1] = -0.4621;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.54491;
		start[1] = 0.68622;
		start[2] = 0.85518;
		start[3] = 0.51834;
		start[4] = 0.77364;
		start[5] = 0.63362;
		start[6] = 0.6616;
		start[7] = 0.40101;
		start[8] = 0.54186;
		start[9] = 0.32843;
		std::vector<double> endEff (2);
		endEff[0] = 0.43698;
		endEff[1] = -0.4621;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.89363;
		start[1] = 0.054792;
		start[2] = 0.62659;
		start[3] = 0.77935;
		start[4] = 0.9985;
		start[5] = 0.054764;
		start[6] = 0.62564;
		start[7] = 0.77818;
		start[8] = 0.034315;
		start[9] = 0.042681;
		std::vector<double> endEff (2);
		endEff[0] = 0.43698;
		endEff[1] = -0.4621;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.30366;
		start[1] = 0.046192;
		start[2] = 0.95425;
		start[3] = 0.29902;
		start[4] = 0.99893;
		start[5] = 0.046175;
		start[6] = 0.95323;
		start[7] = 0.2987;
		start[8] = 0.044063;
		start[9] = 0.013807;
		std::vector<double> endEff (2);
		endEff[0] = 0.43698;
		endEff[1] = -0.4621;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.19548;
		start[1] = 0.72017;
		start[2] = 0.98096;
		start[3] = 0.19423;
		start[4] = 0.7517;
		start[5] = 0.65951;
		start[6] = 0.73738;
		start[7] = 0.14601;
		start[8] = 0.64695;
		start[9] = 0.1281;
		std::vector<double> endEff (2);
		endEff[0] = 0.43698;
		endEff[1] = -0.4621;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.72175;
		start[1] = 0.8778;
		start[2] = 0.75065;
		start[3] = 0.6607;
		start[4] = 0.63885;
		start[5] = 0.76933;
		start[6] = 0.47955;
		start[7] = 0.42209;
		start[8] = 0.5775;
		start[9] = 0.5083;
		std::vector<double> endEff (2);
		endEff[0] = 0.43698;
		endEff[1] = -0.4621;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.58243;
		start[1] = 0.070684;
		start[2] = 0.83513;
		start[3] = 0.55006;
		start[4] = 0.9975;
		start[5] = 0.070625;
		start[6] = 0.83304;
		start[7] = 0.54868;
		start[8] = 0.058981;
		start[9] = 0.038848;
		std::vector<double> endEff (2);
		endEff[0] = 0.43698;
		endEff[1] = -0.4621;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.92274;
		start[1] = 0.80037;
		start[2] = 0.60363;
		start[3] = 0.79726;
		start[4] = 0.69644;
		start[5] = 0.71762;
		start[6] = 0.42039;
		start[7] = 0.55524;
		start[8] = 0.43318;
		start[9] = 0.57213;
		std::vector<double> endEff (2);
		endEff[0] = 0.43698;
		endEff[1] = -0.4621;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.28595;
		start[1] = 0.54366;
		start[2] = 0.9594;
		start[3] = 0.28207;
		start[4] = 0.85582;
		start[5] = 0.51727;
		start[6] = 0.82107;
		start[7] = 0.2414;
		start[8] = 0.49627;
		start[9] = 0.14591;
		std::vector<double> endEff (2);
		endEff[0] = 0.43698;
		endEff[1] = -0.4621;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.98478;
		start[1] = 0.71568;
		start[2] = 0.55305;
		start[3] = 0.83315;
		start[4] = 0.75465;
		start[5] = 0.65613;
		start[6] = 0.41736;
		start[7] = 0.62873;
		start[8] = 0.36287;
		start[9] = 0.54665;
		std::vector<double> endEff (2);
		endEff[0] = 0.43698;
		endEff[1] = -0.4621;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.73597;
		start[1] = 0.79468;
		start[2] = 0.74118;
		start[3] = 0.6713;
		start[4] = 0.70051;
		start[5] = 0.71364;
		start[6] = 0.51921;
		start[7] = 0.47026;
		start[8] = 0.52894;
		start[9] = 0.47907;
		std::vector<double> endEff (2);
		endEff[0] = 0.26769;
		endEff[1] = -0.61352;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.54491;
		start[1] = 0.68622;
		start[2] = 0.85518;
		start[3] = 0.51834;
		start[4] = 0.77364;
		start[5] = 0.63362;
		start[6] = 0.6616;
		start[7] = 0.40101;
		start[8] = 0.54186;
		start[9] = 0.32843;
		std::vector<double> endEff (2);
		endEff[0] = 0.26769;
		endEff[1] = -0.61352;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.89363;
		start[1] = 0.054792;
		start[2] = 0.62659;
		start[3] = 0.77935;
		start[4] = 0.9985;
		start[5] = 0.054764;
		start[6] = 0.62564;
		start[7] = 0.77818;
		start[8] = 0.034315;
		start[9] = 0.042681;
		std::vector<double> endEff (2);
		endEff[0] = 0.26769;
		endEff[1] = -0.61352;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.30366;
		start[1] = 0.046192;
		start[2] = 0.95425;
		start[3] = 0.29902;
		start[4] = 0.99893;
		start[5] = 0.046175;
		start[6] = 0.95323;
		start[7] = 0.2987;
		start[8] = 0.044063;
		start[9] = 0.013807;
		std::vector<double> endEff (2);
		endEff[0] = 0.26769;
		endEff[1] = -0.61352;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.19548;
		start[1] = 0.72017;
		start[2] = 0.98096;
		start[3] = 0.19423;
		start[4] = 0.7517;
		start[5] = 0.65951;
		start[6] = 0.73738;
		start[7] = 0.14601;
		start[8] = 0.64695;
		start[9] = 0.1281;
		std::vector<double> endEff (2);
		endEff[0] = 0.26769;
		endEff[1] = -0.61352;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.72175;
		start[1] = 0.8778;
		start[2] = 0.75065;
		start[3] = 0.6607;
		start[4] = 0.63885;
		start[5] = 0.76933;
		start[6] = 0.47955;
		start[7] = 0.42209;
		start[8] = 0.5775;
		start[9] = 0.5083;
		std::vector<double> endEff (2);
		endEff[0] = 0.26769;
		endEff[1] = -0.61352;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.58243;
		start[1] = 0.070684;
		start[2] = 0.83513;
		start[3] = 0.55006;
		start[4] = 0.9975;
		start[5] = 0.070625;
		start[6] = 0.83304;
		start[7] = 0.54868;
		start[8] = 0.058981;
		start[9] = 0.038848;
		std::vector<double> endEff (2);
		endEff[0] = 0.26769;
		endEff[1] = -0.61352;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.92274;
		start[1] = 0.80037;
		start[2] = 0.60363;
		start[3] = 0.79726;
		start[4] = 0.69644;
		start[5] = 0.71762;
		start[6] = 0.42039;
		start[7] = 0.55524;
		start[8] = 0.43318;
		start[9] = 0.57213;
		std::vector<double> endEff (2);
		endEff[0] = 0.26769;
		endEff[1] = -0.61352;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.28595;
		start[1] = 0.54366;
		start[2] = 0.9594;
		start[3] = 0.28207;
		start[4] = 0.85582;
		start[5] = 0.51727;
		start[6] = 0.82107;
		start[7] = 0.2414;
		start[8] = 0.49627;
		start[9] = 0.14591;
		std::vector<double> endEff (2);
		endEff[0] = 0.26769;
		endEff[1] = -0.61352;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.98478;
		start[1] = 0.71568;
		start[2] = 0.55305;
		start[3] = 0.83315;
		start[4] = 0.75465;
		start[5] = 0.65613;
		start[6] = 0.41736;
		start[7] = 0.62873;
		start[8] = 0.36287;
		start[9] = 0.54665;
		std::vector<double> endEff (2);
		endEff[0] = 0.26769;
		endEff[1] = -0.61352;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.73597;
		start[1] = 0.79468;
		start[2] = 0.74118;
		start[3] = 0.6713;
		start[4] = 0.70051;
		start[5] = 0.71364;
		start[6] = 0.51921;
		start[7] = 0.47026;
		start[8] = 0.52894;
		start[9] = 0.47907;
		std::vector<double> endEff (2);
		endEff[0] = 0.020811;
		endEff[1] = -0.7541;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.54491;
		start[1] = 0.68622;
		start[2] = 0.85518;
		start[3] = 0.51834;
		start[4] = 0.77364;
		start[5] = 0.63362;
		start[6] = 0.6616;
		start[7] = 0.40101;
		start[8] = 0.54186;
		start[9] = 0.32843;
		std::vector<double> endEff (2);
		endEff[0] = 0.020811;
		endEff[1] = -0.7541;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.89363;
		start[1] = 0.054792;
		start[2] = 0.62659;
		start[3] = 0.77935;
		start[4] = 0.9985;
		start[5] = 0.054764;
		start[6] = 0.62564;
		start[7] = 0.77818;
		start[8] = 0.034315;
		start[9] = 0.042681;
		std::vector<double> endEff (2);
		endEff[0] = 0.020811;
		endEff[1] = -0.7541;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.30366;
		start[1] = 0.046192;
		start[2] = 0.95425;
		start[3] = 0.29902;
		start[4] = 0.99893;
		start[5] = 0.046175;
		start[6] = 0.95323;
		start[7] = 0.2987;
		start[8] = 0.044063;
		start[9] = 0.013807;
		std::vector<double> endEff (2);
		endEff[0] = 0.020811;
		endEff[1] = -0.7541;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.19548;
		start[1] = 0.72017;
		start[2] = 0.98096;
		start[3] = 0.19423;
		start[4] = 0.7517;
		start[5] = 0.65951;
		start[6] = 0.73738;
		start[7] = 0.14601;
		start[8] = 0.64695;
		start[9] = 0.1281;
		std::vector<double> endEff (2);
		endEff[0] = 0.020811;
		endEff[1] = -0.7541;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.72175;
		start[1] = 0.8778;
		start[2] = 0.75065;
		start[3] = 0.6607;
		start[4] = 0.63885;
		start[5] = 0.76933;
		start[6] = 0.47955;
		start[7] = 0.42209;
		start[8] = 0.5775;
		start[9] = 0.5083;
		std::vector<double> endEff (2);
		endEff[0] = 0.020811;
		endEff[1] = -0.7541;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.58243;
		start[1] = 0.070684;
		start[2] = 0.83513;
		start[3] = 0.55006;
		start[4] = 0.9975;
		start[5] = 0.070625;
		start[6] = 0.83304;
		start[7] = 0.54868;
		start[8] = 0.058981;
		start[9] = 0.038848;
		std::vector<double> endEff (2);
		endEff[0] = 0.020811;
		endEff[1] = -0.7541;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.92274;
		start[1] = 0.80037;
		start[2] = 0.60363;
		start[3] = 0.79726;
		start[4] = 0.69644;
		start[5] = 0.71762;
		start[6] = 0.42039;
		start[7] = 0.55524;
		start[8] = 0.43318;
		start[9] = 0.57213;
		std::vector<double> endEff (2);
		endEff[0] = 0.020811;
		endEff[1] = -0.7541;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.28595;
		start[1] = 0.54366;
		start[2] = 0.9594;
		start[3] = 0.28207;
		start[4] = 0.85582;
		start[5] = 0.51727;
		start[6] = 0.82107;
		start[7] = 0.2414;
		start[8] = 0.49627;
		start[9] = 0.14591;
		std::vector<double> endEff (2);
		endEff[0] = 0.020811;
		endEff[1] = -0.7541;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.98478;
		start[1] = 0.71568;
		start[2] = 0.55305;
		start[3] = 0.83315;
		start[4] = 0.75465;
		start[5] = 0.65613;
		start[6] = 0.41736;
		start[7] = 0.62873;
		start[8] = 0.36287;
		start[9] = 0.54665;
		std::vector<double> endEff (2);
		endEff[0] = 0.020811;
		endEff[1] = -0.7541;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.73597;
		start[1] = 0.79468;
		start[2] = 0.74118;
		start[3] = 0.6713;
		start[4] = 0.70051;
		start[5] = 0.71364;
		start[6] = 0.51921;
		start[7] = 0.47026;
		start[8] = 0.52894;
		start[9] = 0.47907;
		std::vector<double> endEff (2);
		endEff[0] = -0.17288;
		endEff[1] = -0.35022;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.54491;
		start[1] = 0.68622;
		start[2] = 0.85518;
		start[3] = 0.51834;
		start[4] = 0.77364;
		start[5] = 0.63362;
		start[6] = 0.6616;
		start[7] = 0.40101;
		start[8] = 0.54186;
		start[9] = 0.32843;
		std::vector<double> endEff (2);
		endEff[0] = -0.17288;
		endEff[1] = -0.35022;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.89363;
		start[1] = 0.054792;
		start[2] = 0.62659;
		start[3] = 0.77935;
		start[4] = 0.9985;
		start[5] = 0.054764;
		start[6] = 0.62564;
		start[7] = 0.77818;
		start[8] = 0.034315;
		start[9] = 0.042681;
		std::vector<double> endEff (2);
		endEff[0] = -0.17288;
		endEff[1] = -0.35022;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.30366;
		start[1] = 0.046192;
		start[2] = 0.95425;
		start[3] = 0.29902;
		start[4] = 0.99893;
		start[5] = 0.046175;
		start[6] = 0.95323;
		start[7] = 0.2987;
		start[8] = 0.044063;
		start[9] = 0.013807;
		std::vector<double> endEff (2);
		endEff[0] = -0.17288;
		endEff[1] = -0.35022;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.19548;
		start[1] = 0.72017;
		start[2] = 0.98096;
		start[3] = 0.19423;
		start[4] = 0.7517;
		start[5] = 0.65951;
		start[6] = 0.73738;
		start[7] = 0.14601;
		start[8] = 0.64695;
		start[9] = 0.1281;
		std::vector<double> endEff (2);
		endEff[0] = -0.17288;
		endEff[1] = -0.35022;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.72175;
		start[1] = 0.8778;
		start[2] = 0.75065;
		start[3] = 0.6607;
		start[4] = 0.63885;
		start[5] = 0.76933;
		start[6] = 0.47955;
		start[7] = 0.42209;
		start[8] = 0.5775;
		start[9] = 0.5083;
		std::vector<double> endEff (2);
		endEff[0] = -0.17288;
		endEff[1] = -0.35022;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.58243;
		start[1] = 0.070684;
		start[2] = 0.83513;
		start[3] = 0.55006;
		start[4] = 0.9975;
		start[5] = 0.070625;
		start[6] = 0.83304;
		start[7] = 0.54868;
		start[8] = 0.058981;
		start[9] = 0.038848;
		std::vector<double> endEff (2);
		endEff[0] = -0.17288;
		endEff[1] = -0.35022;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.92274;
		start[1] = 0.80037;
		start[2] = 0.60363;
		start[3] = 0.79726;
		start[4] = 0.69644;
		start[5] = 0.71762;
		start[6] = 0.42039;
		start[7] = 0.55524;
		start[8] = 0.43318;
		start[9] = 0.57213;
		std::vector<double> endEff (2);
		endEff[0] = -0.17288;
		endEff[1] = -0.35022;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.28595;
		start[1] = 0.54366;
		start[2] = 0.9594;
		start[3] = 0.28207;
		start[4] = 0.85582;
		start[5] = 0.51727;
		start[6] = 0.82107;
		start[7] = 0.2414;
		start[8] = 0.49627;
		start[9] = 0.14591;
		std::vector<double> endEff (2);
		endEff[0] = -0.17288;
		endEff[1] = -0.35022;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.98478;
		start[1] = 0.71568;
		start[2] = 0.55305;
		start[3] = 0.83315;
		start[4] = 0.75465;
		start[5] = 0.65613;
		start[6] = 0.41736;
		start[7] = 0.62873;
		start[8] = 0.36287;
		start[9] = 0.54665;
		std::vector<double> endEff (2);
		endEff[0] = -0.17288;
		endEff[1] = -0.35022;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.73597;
		start[1] = 0.79468;
		start[2] = 0.74118;
		start[3] = 0.6713;
		start[4] = 0.70051;
		start[5] = 0.71364;
		start[6] = 0.51921;
		start[7] = 0.47026;
		start[8] = 0.52894;
		start[9] = 0.47907;
		std::vector<double> endEff (2);
		endEff[0] = 0.53752;
		endEff[1] = 0.19302;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.54491;
		start[1] = 0.68622;
		start[2] = 0.85518;
		start[3] = 0.51834;
		start[4] = 0.77364;
		start[5] = 0.63362;
		start[6] = 0.6616;
		start[7] = 0.40101;
		start[8] = 0.54186;
		start[9] = 0.32843;
		std::vector<double> endEff (2);
		endEff[0] = 0.53752;
		endEff[1] = 0.19302;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.89363;
		start[1] = 0.054792;
		start[2] = 0.62659;
		start[3] = 0.77935;
		start[4] = 0.9985;
		start[5] = 0.054764;
		start[6] = 0.62564;
		start[7] = 0.77818;
		start[8] = 0.034315;
		start[9] = 0.042681;
		std::vector<double> endEff (2);
		endEff[0] = 0.53752;
		endEff[1] = 0.19302;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.30366;
		start[1] = 0.046192;
		start[2] = 0.95425;
		start[3] = 0.29902;
		start[4] = 0.99893;
		start[5] = 0.046175;
		start[6] = 0.95323;
		start[7] = 0.2987;
		start[8] = 0.044063;
		start[9] = 0.013807;
		std::vector<double> endEff (2);
		endEff[0] = 0.53752;
		endEff[1] = 0.19302;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.19548;
		start[1] = 0.72017;
		start[2] = 0.98096;
		start[3] = 0.19423;
		start[4] = 0.7517;
		start[5] = 0.65951;
		start[6] = 0.73738;
		start[7] = 0.14601;
		start[8] = 0.64695;
		start[9] = 0.1281;
		std::vector<double> endEff (2);
		endEff[0] = 0.53752;
		endEff[1] = 0.19302;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.72175;
		start[1] = 0.8778;
		start[2] = 0.75065;
		start[3] = 0.6607;
		start[4] = 0.63885;
		start[5] = 0.76933;
		start[6] = 0.47955;
		start[7] = 0.42209;
		start[8] = 0.5775;
		start[9] = 0.5083;
		std::vector<double> endEff (2);
		endEff[0] = 0.53752;
		endEff[1] = 0.19302;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.58243;
		start[1] = 0.070684;
		start[2] = 0.83513;
		start[3] = 0.55006;
		start[4] = 0.9975;
		start[5] = 0.070625;
		start[6] = 0.83304;
		start[7] = 0.54868;
		start[8] = 0.058981;
		start[9] = 0.038848;
		std::vector<double> endEff (2);
		endEff[0] = 0.53752;
		endEff[1] = 0.19302;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.92274;
		start[1] = 0.80037;
		start[2] = 0.60363;
		start[3] = 0.79726;
		start[4] = 0.69644;
		start[5] = 0.71762;
		start[6] = 0.42039;
		start[7] = 0.55524;
		start[8] = 0.43318;
		start[9] = 0.57213;
		std::vector<double> endEff (2);
		endEff[0] = 0.53752;
		endEff[1] = 0.19302;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.28595;
		start[1] = 0.54366;
		start[2] = 0.9594;
		start[3] = 0.28207;
		start[4] = 0.85582;
		start[5] = 0.51727;
		start[6] = 0.82107;
		start[7] = 0.2414;
		start[8] = 0.49627;
		start[9] = 0.14591;
		std::vector<double> endEff (2);
		endEff[0] = 0.53752;
		endEff[1] = 0.19302;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.98478;
		start[1] = 0.71568;
		start[2] = 0.55305;
		start[3] = 0.83315;
		start[4] = 0.75465;
		start[5] = 0.65613;
		start[6] = 0.41736;
		start[7] = 0.62873;
		start[8] = 0.36287;
		start[9] = 0.54665;
		std::vector<double> endEff (2);
		endEff[0] = 0.53752;
		endEff[1] = 0.19302;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.73597;
		start[1] = 0.79468;
		start[2] = 0.74118;
		start[3] = 0.6713;
		start[4] = 0.70051;
		start[5] = 0.71364;
		start[6] = 0.51921;
		start[7] = 0.47026;
		start[8] = 0.52894;
		start[9] = 0.47907;
		std::vector<double> endEff (2);
		endEff[0] = 0.46312;
		endEff[1] = 0.48431;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.54491;
		start[1] = 0.68622;
		start[2] = 0.85518;
		start[3] = 0.51834;
		start[4] = 0.77364;
		start[5] = 0.63362;
		start[6] = 0.6616;
		start[7] = 0.40101;
		start[8] = 0.54186;
		start[9] = 0.32843;
		std::vector<double> endEff (2);
		endEff[0] = 0.46312;
		endEff[1] = 0.48431;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.89363;
		start[1] = 0.054792;
		start[2] = 0.62659;
		start[3] = 0.77935;
		start[4] = 0.9985;
		start[5] = 0.054764;
		start[6] = 0.62564;
		start[7] = 0.77818;
		start[8] = 0.034315;
		start[9] = 0.042681;
		std::vector<double> endEff (2);
		endEff[0] = 0.46312;
		endEff[1] = 0.48431;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.30366;
		start[1] = 0.046192;
		start[2] = 0.95425;
		start[3] = 0.29902;
		start[4] = 0.99893;
		start[5] = 0.046175;
		start[6] = 0.95323;
		start[7] = 0.2987;
		start[8] = 0.044063;
		start[9] = 0.013807;
		std::vector<double> endEff (2);
		endEff[0] = 0.46312;
		endEff[1] = 0.48431;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.19548;
		start[1] = 0.72017;
		start[2] = 0.98096;
		start[3] = 0.19423;
		start[4] = 0.7517;
		start[5] = 0.65951;
		start[6] = 0.73738;
		start[7] = 0.14601;
		start[8] = 0.64695;
		start[9] = 0.1281;
		std::vector<double> endEff (2);
		endEff[0] = 0.46312;
		endEff[1] = 0.48431;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.72175;
		start[1] = 0.8778;
		start[2] = 0.75065;
		start[3] = 0.6607;
		start[4] = 0.63885;
		start[5] = 0.76933;
		start[6] = 0.47955;
		start[7] = 0.42209;
		start[8] = 0.5775;
		start[9] = 0.5083;
		std::vector<double> endEff (2);
		endEff[0] = 0.46312;
		endEff[1] = 0.48431;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.58243;
		start[1] = 0.070684;
		start[2] = 0.83513;
		start[3] = 0.55006;
		start[4] = 0.9975;
		start[5] = 0.070625;
		start[6] = 0.83304;
		start[7] = 0.54868;
		start[8] = 0.058981;
		start[9] = 0.038848;
		std::vector<double> endEff (2);
		endEff[0] = 0.46312;
		endEff[1] = 0.48431;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.92274;
		start[1] = 0.80037;
		start[2] = 0.60363;
		start[3] = 0.79726;
		start[4] = 0.69644;
		start[5] = 0.71762;
		start[6] = 0.42039;
		start[7] = 0.55524;
		start[8] = 0.43318;
		start[9] = 0.57213;
		std::vector<double> endEff (2);
		endEff[0] = 0.46312;
		endEff[1] = 0.48431;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.28595;
		start[1] = 0.54366;
		start[2] = 0.9594;
		start[3] = 0.28207;
		start[4] = 0.85582;
		start[5] = 0.51727;
		start[6] = 0.82107;
		start[7] = 0.2414;
		start[8] = 0.49627;
		start[9] = 0.14591;
		std::vector<double> endEff (2);
		endEff[0] = 0.46312;
		endEff[1] = 0.48431;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.98478;
		start[1] = 0.71568;
		start[2] = 0.55305;
		start[3] = 0.83315;
		start[4] = 0.75465;
		start[5] = 0.65613;
		start[6] = 0.41736;
		start[7] = 0.62873;
		start[8] = 0.36287;
		start[9] = 0.54665;
		std::vector<double> endEff (2);
		endEff[0] = 0.46312;
		endEff[1] = 0.48431;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.73597;
		start[1] = 0.79468;
		start[2] = 0.74118;
		start[3] = 0.6713;
		start[4] = 0.70051;
		start[5] = 0.71364;
		start[6] = 0.51921;
		start[7] = 0.47026;
		start[8] = 0.52894;
		start[9] = 0.47907;
		std::vector<double> endEff (2);
		endEff[0] = 0.20196;
		endEff[1] = 0.9052;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.54491;
		start[1] = 0.68622;
		start[2] = 0.85518;
		start[3] = 0.51834;
		start[4] = 0.77364;
		start[5] = 0.63362;
		start[6] = 0.6616;
		start[7] = 0.40101;
		start[8] = 0.54186;
		start[9] = 0.32843;
		std::vector<double> endEff (2);
		endEff[0] = 0.20196;
		endEff[1] = 0.9052;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.89363;
		start[1] = 0.054792;
		start[2] = 0.62659;
		start[3] = 0.77935;
		start[4] = 0.9985;
		start[5] = 0.054764;
		start[6] = 0.62564;
		start[7] = 0.77818;
		start[8] = 0.034315;
		start[9] = 0.042681;
		std::vector<double> endEff (2);
		endEff[0] = 0.20196;
		endEff[1] = 0.9052;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.30366;
		start[1] = 0.046192;
		start[2] = 0.95425;
		start[3] = 0.29902;
		start[4] = 0.99893;
		start[5] = 0.046175;
		start[6] = 0.95323;
		start[7] = 0.2987;
		start[8] = 0.044063;
		start[9] = 0.013807;
		std::vector<double> endEff (2);
		endEff[0] = 0.20196;
		endEff[1] = 0.9052;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.19548;
		start[1] = 0.72017;
		start[2] = 0.98096;
		start[3] = 0.19423;
		start[4] = 0.7517;
		start[5] = 0.65951;
		start[6] = 0.73738;
		start[7] = 0.14601;
		start[8] = 0.64695;
		start[9] = 0.1281;
		std::vector<double> endEff (2);
		endEff[0] = 0.20196;
		endEff[1] = 0.9052;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.72175;
		start[1] = 0.8778;
		start[2] = 0.75065;
		start[3] = 0.6607;
		start[4] = 0.63885;
		start[5] = 0.76933;
		start[6] = 0.47955;
		start[7] = 0.42209;
		start[8] = 0.5775;
		start[9] = 0.5083;
		std::vector<double> endEff (2);
		endEff[0] = 0.20196;
		endEff[1] = 0.9052;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.58243;
		start[1] = 0.070684;
		start[2] = 0.83513;
		start[3] = 0.55006;
		start[4] = 0.9975;
		start[5] = 0.070625;
		start[6] = 0.83304;
		start[7] = 0.54868;
		start[8] = 0.058981;
		start[9] = 0.038848;
		std::vector<double> endEff (2);
		endEff[0] = 0.20196;
		endEff[1] = 0.9052;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.92274;
		start[1] = 0.80037;
		start[2] = 0.60363;
		start[3] = 0.79726;
		start[4] = 0.69644;
		start[5] = 0.71762;
		start[6] = 0.42039;
		start[7] = 0.55524;
		start[8] = 0.43318;
		start[9] = 0.57213;
		std::vector<double> endEff (2);
		endEff[0] = 0.20196;
		endEff[1] = 0.9052;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.28595;
		start[1] = 0.54366;
		start[2] = 0.9594;
		start[3] = 0.28207;
		start[4] = 0.85582;
		start[5] = 0.51727;
		start[6] = 0.82107;
		start[7] = 0.2414;
		start[8] = 0.49627;
		start[9] = 0.14591;
		std::vector<double> endEff (2);
		endEff[0] = 0.20196;
		endEff[1] = 0.9052;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.98478;
		start[1] = 0.71568;
		start[2] = 0.55305;
		start[3] = 0.83315;
		start[4] = 0.75465;
		start[5] = 0.65613;
		start[6] = 0.41736;
		start[7] = 0.62873;
		start[8] = 0.36287;
		start[9] = 0.54665;
		std::vector<double> endEff (2);
		endEff[0] = 0.20196;
		endEff[1] = 0.9052;
		solveFor( start, endEff);
	}


  return 0;
}
