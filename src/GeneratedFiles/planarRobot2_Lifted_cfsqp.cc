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
		std::vector<double> start (10);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0034479;
		start[3] = 7.7963e-05;
		start[4] = 0.0068794;
		start[5] = 0.0035003;
		start[6] = 0.0098207;
		start[7] = 0.0081655;
		start[8] = 0.0043657;
		start[9] = 0.0057265;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0097133;
		start[3] = 0.0093506;
		start[4] = 0.0053808;
		start[5] = 0.0091781;
		start[6] = 0.007612;
		start[7] = 0.0097868;
		start[8] = 0.0041919;
		start[9] = 0.0018472;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.008873;
		start[3] = 0.0097362;
		start[4] = 0.008539;
		start[5] = 0.00048129;
		start[6] = 0.0020244;
		start[7] = 0.0083052;
		start[8] = 0.0032337;
		start[9] = 0.0088826;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0013205;
		start[3] = 0.0095385;
		start[4] = 0.0023106;
		start[5] = 0.00053978;
		start[6] = 0.0059378;
		start[7] = 0.0090565;
		start[8] = 0.0082883;
		start[9] = 0.0092461;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0041095;
		start[3] = 0.0056241;
		start[4] = 0.0066173;
		start[5] = 0.0015821;
		start[6] = 0.0086255;
		start[7] = 0.0040716;
		start[8] = 0.0080194;
		start[9] = 0.0018319;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0098772;
		start[3] = 0.0085995;
		start[4] = 0.0082384;
		start[5] = 0.0040561;
		start[6] = 0.0006863;
		start[7] = 0.0061072;
		start[8] = 0.0099098;
		start[9] = 0.0043664;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0015858;
		start[3] = 0.0088683;
		start[4] = 0.001637;
		start[5] = 1.7359e-05;
		start[6] = 0.0086436;
		start[7] = 0.0097404;
		start[8] = 0.0072157;
		start[9] = 0.0072066;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0054994;
		start[3] = 0.0060202;
		start[4] = 0.0010606;
		start[5] = 0.0068864;
		start[6] = 0.0082555;
		start[7] = 0.0078801;
		start[8] = 0.0010027;
		start[9] = 0.009155;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0036198;
		start[3] = 0.0032467;
		start[4] = 0.0095665;
		start[5] = 0.00090819;
		start[6] = 0.0075382;
		start[7] = 0.0037571;
		start[8] = 0.00088711;
		start[9] = 0.0097948;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0067718;
		start[3] = 0.0082884;
		start[4] = 0.0096263;
		start[5] = 0.0063122;
		start[6] = 0.00028491;
		start[7] = 0.0049574;
		start[8] = 0.0018373;
		start[9] = 0.0061437;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0037841;
		start[3] = 0.0063301;
		start[4] = 0.0084714;
		start[5] = 0.0075136;
		start[6] = 0.0037548;
		start[7] = 0.0068803;
		start[8] = 0.0005267;
		start[9] = 0.0080506;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0072517;
		start[3] = 0.00035639;
		start[4] = 0.0050173;
		start[5] = 0.009281;
		start[6] = 0.00248;
		start[7] = 0.0092113;
		start[8] = 0.0015194;
		start[9] = 0.0022985;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.008359;
		start[3] = 0.0078243;
		start[4] = 0.0069991;
		start[5] = 0.0084789;
		start[6] = 0.0012964;
		start[7] = 0.0016147;
		start[8] = 0.0020168;
		start[9] = 0.0053215;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0087748;
		start[3] = 0.0058785;
		start[4] = 0.0032585;
		start[5] = 0.0068572;
		start[6] = 0.00075487;
		start[7] = 0.0028718;
		start[8] = 0.0059274;
		start[9] = 0.0017319;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0021098;
		start[3] = 0.0089149;
		start[4] = 0.0087796;
		start[5] = 0.0018556;
		start[6] = 0.0030175;
		start[7] = 0.003493;
		start[8] = 0.00177;
		start[9] = 0.001799;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0078145;
		start[3] = 0.0067622;
		start[4] = 0.0053955;
		start[5] = 0.0062131;
		start[6] = 0.0026226;
		start[7] = 0.0031218;
		start[8] = 0.0085371;
		start[9] = 0.0005909;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0033636;
		start[3] = 0.0094249;
		start[4] = 0.0013024;
		start[5] = 0.0075643;
		start[6] = 0.00066598;
		start[7] = 0.0094801;
		start[8] = 0.0065367;
		start[9] = 0.0010325;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0040027;
		start[3] = 0.0057021;
		start[4] = 0.0063199;
		start[5] = 0.0026977;
		start[6] = 0.0074009;
		start[7] = 0.0037375;
		start[8] = 0.0077298;
		start[9] = 0.0074274;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0094398;
		start[3] = 0.00097803;
		start[4] = 0.0031935;
		start[5] = 6.1099e-06;
		start[6] = 0.0049793;
		start[7] = 0.0096837;
		start[8] = 0.00024041;
		start[9] = 0.0049993;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0041624;
		start[3] = 0.0082718;
		start[4] = 0.0071872;
		start[5] = 0.00041973;
		start[6] = 0.0076457;
		start[7] = 0.0018303;
		start[8] = 0.0059606;
		start[9] = 0.0059953;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0084732;
		start[3] = 0.0085938;
		start[4] = 0.0015314;
		start[5] = 0.0073549;
		start[6] = 0.0086362;
		start[7] = 0.009494;
		start[8] = 0.0038324;
		start[9] = 0.00093471;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0019896;
		start[3] = 0.0036088;
		start[4] = 0.0092414;
		start[5] = 0.0043907;
		start[6] = 0.0037712;
		start[7] = 0.004748;
		start[8] = 0.00084639;
		start[9] = 0.004779;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0077129;
		start[3] = 0.0014981;
		start[4] = 0.0030356;
		start[5] = 0.0098489;
		start[6] = 0.0039616;
		start[7] = 0.006637;
		start[8] = 0.0077188;
		start[9] = 0.0061381;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0017795;
		start[3] = 0.0001951;
		start[4] = 0.0019775;
		start[5] = 0.0054936;
		start[6] = 0.0031048;
		start[7] = 0.0020181;
		start[8] = 0.0049945;
		start[9] = 0.0060916;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0080778;
		start[3] = 0.0061542;
		start[4] = 0.0040562;
		start[5] = 0.0047112;
		start[6] = 7.3065e-05;
		start[7] = 0.0043661;
		start[8] = 0.0016036;
		start[9] = 0.004692;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0081644;
		start[3] = 0.0054709;
		start[4] = 0.00041359;
		start[5] = 0.0027979;
		start[6] = 0.00052741;
		start[7] = 0.0035158;
		start[8] = 0.0014036;
		start[9] = 0.0035983;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0020215;
		start[3] = 0.0029126;
		start[4] = 0.007497;
		start[5] = 0.0078999;
		start[6] = 0.0099415;
		start[7] = 0.008769;
		start[8] = 0.0020671;
		start[9] = 0.0021029;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0094263;
		start[3] = 0.006669;
		start[4] = 0.00089812;
		start[5] = 0.0048903;
		start[6] = 0.0015446;
		start[7] = 0.0087553;
		start[8] = 0.0088482;
		start[9] = 0.0065927;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.00027001;
		start[3] = 0.0095805;
		start[4] = 0.0064596;
		start[5] = 0.00090006;
		start[6] = 0.00282;
		start[7] = 0.0076695;
		start[8] = 0.001959;
		start[9] = 0.00024929;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0013174;
		start[3] = 0.0034901;
		start[4] = 0.0068953;
		start[5] = 0.00095606;
		start[6] = 0.0049922;
		start[7] = 0.0013197;
		start[8] = 0.0062685;
		start[9] = 0.0054386;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0060379;
		start[3] = 0.0080454;
		start[4] = 0.0052239;
		start[5] = 0.0010105;
		start[6] = 0.0067841;
		start[7] = 0.0081854;
		start[8] = 0.0037602;
		start[9] = 7.2029e-07;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.00016285;
		start[3] = 0.0025953;
		start[4] = 0.0094277;
		start[5] = 0.0040085;
		start[6] = 0.0014994;
		start[7] = 0.0030496;
		start[8] = 0.0016569;
		start[9] = 0.0037415;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0066423;
		start[3] = 0.0073166;
		start[4] = 0.0013497;
		start[5] = 0.0024843;
		start[6] = 0.0097638;
		start[7] = 0.0070385;
		start[8] = 0.0042759;
		start[9] = 0.0014381;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0045854;
		start[3] = 0.0060025;
		start[4] = 0.0075414;
		start[5] = 0.0018624;
		start[6] = 0.0082297;
		start[7] = 0.0022885;
		start[8] = 0.0090592;
		start[9] = 0.0095353;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0050152;
		start[3] = 0.0033212;
		start[4] = 0.0019452;
		start[5] = 0.00079115;
		start[6] = 0.0016364;
		start[7] = 0.0063286;
		start[8] = 0.0075838;
		start[9] = 0.0048607;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0026741;
		start[3] = 0.0062152;
		start[4] = 0.0083998;
		start[5] = 0.0019822;
		start[6] = 0.0078698;
		start[7] = 0.0053839;
		start[8] = 0.0050903;
		start[9] = 0.0055697;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0031903;
		start[3] = 0.004113;
		start[4] = 0.0090445;
		start[5] = 0.0086634;
		start[6] = 0.0092253;
		start[7] = 0.0056374;
		start[8] = 0.0018632;
		start[9] = 0.0067659;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.003573;
		start[3] = 0.0018018;
		start[4] = 0.0041346;
		start[5] = 0.0080096;
		start[6] = 0.0025128;
		start[7] = 0.0066499;
		start[8] = 0.0017696;
		start[9] = 0.0043209;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0009582;
		start[3] = 0.0030095;
		start[4] = 0.007197;
		start[5] = 0.0038354;
		start[6] = 0.004384;
		start[7] = 9.0919e-05;
		start[8] = 0.0017253;
		start[9] = 0.00083199;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0065513;
		start[3] = 0.0043491;
		start[4] = 0.0064248;
		start[5] = 0.0051791;
		start[6] = 0.0013162;
		start[7] = 0.0042929;
		start[8] = 0.0099345;
		start[9] = 0.0038066;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0083894;
		start[3] = 0.0004869;
		start[4] = 0.001798;
		start[5] = 0.0070321;
		start[6] = 0.0029785;
		start[7] = 0.0042325;
		start[8] = 0.0053377;
		start[9] = 0.0077163;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0023802;
		start[3] = 0.0062561;
		start[4] = 0.00718;
		start[5] = 0.0021887;
		start[6] = 0.0048928;
		start[7] = 0.0088591;
		start[8] = 0.0099957;
		start[9] = 0.0078656;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0016283;
		start[3] = 0.0062692;
		start[4] = 0.0032639;
		start[5] = 0.0059954;
		start[6] = 0.0074861;
		start[7] = 0.0086253;
		start[8] = 0.00061527;
		start[9] = 0.0060731;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0041507;
		start[3] = 0.0041407;
		start[4] = 0.0037303;
		start[5] = 0.0024261;
		start[6] = 0.0050762;
		start[7] = 0.0097806;
		start[8] = 0.0045789;
		start[9] = 0.0097631;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.00043576;
		start[3] = 0.0055215;
		start[4] = 0.0087538;
		start[5] = 0.0029948;
		start[6] = 0.00062944;
		start[7] = 0.0059399;
		start[8] = 0.0019072;
		start[9] = 0.0029575;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.002354;
		start[3] = 0.006308;
		start[4] = 0.00037771;
		start[5] = 0.0044909;
		start[6] = 0.0021375;
		start[7] = 0.0038779;
		start[8] = 0.00033661;
		start[9] = 0.0094454;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0045936;
		start[3] = 0.0079051;
		start[4] = 0.0010692;
		start[5] = 0.0076048;
		start[6] = 0.0097274;
		start[7] = 0.00092125;
		start[8] = 0.0037569;
		start[9] = 0.005158;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 3.9578e-05;
		start[3] = 0.0094738;
		start[4] = 0.0045152;
		start[5] = 0.0098314;
		start[6] = 0.0023313;
		start[7] = 0.0036739;
		start[8] = 0.0026284;
		start[9] = 0.00013248;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.00070377;
		start[3] = 0.0037827;
		start[4] = 0.0055988;
		start[5] = 0.0023605;
		start[6] = 0.0059087;
		start[7] = 0.0089351;
		start[8] = 0.0065097;
		start[9] = 0.0050389;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0064439;
		start[3] = 0.0075926;
		start[4] = 0.0032892;
		start[5] = 0.0054818;
		start[6] = 0.0076166;
		start[7] = 0.0015148;
		start[8] = 0.0091553;
		start[9] = 0.007363;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0090863;
		start[3] = 0.0060888;
		start[4] = 0.0062559;
		start[5] = 0.0035769;
		start[6] = 0.0088555;
		start[7] = 0.0026391;
		start[8] = 0.0010462;
		start[9] = 0.0032481;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0052529;
		start[3] = 0.0018099;
		start[4] = 0.00541;
		start[5] = 0.002931;
		start[6] = 0.0096499;
		start[7] = 0.0035107;
		start[8] = 0.004333;
		start[9] = 0.0084506;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0034033;
		start[3] = 0.0065162;
		start[4] = 0.005274;
		start[5] = 0.00067469;
		start[6] = 0.0030256;
		start[7] = 0.00494;
		start[8] = 0.0066842;
		start[9] = 0.0099631;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0063541;
		start[3] = 0.0077109;
		start[4] = 0.0025855;
		start[5] = 0.0019888;
		start[6] = 0.0023863;
		start[7] = 0.0051919;
		start[8] = 0.0061869;
		start[9] = 0.00099882;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0040624;
		start[3] = 0.0097119;
		start[4] = 0.0065055;
		start[5] = 0.0078291;
		start[6] = 0.00079982;
		start[7] = 0.0048056;
		start[8] = 0.0006652;
		start[9] = 0.0043464;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0054613;
		start[3] = 0.0085728;
		start[4] = 0.0039221;
		start[5] = 0.0082937;
		start[6] = 0.0025858;
		start[7] = 0.00050662;
		start[8] = 0.0091398;
		start[9] = 0.007087;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0068363;
		start[3] = 0.0076026;
		start[4] = 0.00030832;
		start[5] = 0.0062013;
		start[6] = 0.0051674;
		start[7] = 0.0057099;
		start[8] = 0.0057993;
		start[9] = 0.0020997;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0082702;
		start[3] = 0.0031836;
		start[4] = 0.0044252;
		start[5] = 0.001839;
		start[6] = 0.0026432;
		start[7] = 0.00073339;
		start[8] = 0.0074952;
		start[9] = 0.0039093;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0062329;
		start[3] = 0.0014614;
		start[4] = 0.0037218;
		start[5] = 0.001476;
		start[6] = 0.0035593;
		start[7] = 0.0048208;
		start[8] = 0.0056636;
		start[9] = 0.0056984;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0080129;
		start[3] = 0.0081724;
		start[4] = 0.0050606;
		start[5] = 0.005031;
		start[6] = 0.0070612;
		start[7] = 0.0053643;
		start[8] = 0.009066;
		start[9] = 0.0094907;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0059868;
		start[3] = 0.0072644;
		start[4] = 0.0042728;
		start[5] = 8.6971e-05;
		start[6] = 0.0053096;
		start[7] = 0.0021477;
		start[8] = 0.001833;
		start[9] = 0.0095355;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0074444;
		start[3] = 0.002454;
		start[4] = 0.001061;
		start[5] = 0.0099922;
		start[6] = 0.0066376;
		start[7] = 0.0046956;
		start[8] = 0.00067412;
		start[9] = 0.0050931;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0055288;
		start[3] = 0.003129;
		start[4] = 0.0036236;
		start[5] = 0.0074924;
		start[6] = 0.0080169;
		start[7] = 0.006007;
		start[8] = 0.0029602;
		start[9] = 0.0067737;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0040021;
		start[3] = 0.0054654;
		start[4] = 0.0064585;
		start[5] = 0.0068893;
		start[6] = 0.0065571;
		start[7] = 0.0020932;
		start[8] = 0.0095455;
		start[9] = 0.0047836;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0046153;
		start[3] = 0.0048116;
		start[4] = 0.0014979;
		start[5] = 0.0011623;
		start[6] = 0.0068357;
		start[7] = 0.0085552;
		start[8] = 0.0014465;
		start[9] = 0.0044321;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.00017446;
		start[3] = 0.00091545;
		start[4] = 0.0068016;
		start[5] = 0.009426;
		start[6] = 0.0046385;
		start[7] = 0.0095874;
		start[8] = 0.0049868;
		start[9] = 0.0078357;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0066822;
		start[3] = 0.00375;
		start[4] = 0.00067799;
		start[5] = 0.0064704;
		start[6] = 0.0044339;
		start[7] = 0.0015011;
		start[8] = 0.00072437;
		start[9] = 0.0076826;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0017779;
		start[3] = 0.0021528;
		start[4] = 0.0057317;
		start[5] = 0.0063646;
		start[6] = 0.0021965;
		start[7] = 0.0044657;
		start[8] = 0.0070918;
		start[9] = 0.0001823;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0055747;
		start[3] = 0.0054833;
		start[4] = 9.7311e-05;
		start[5] = 0.0045227;
		start[6] = 0.0033611;
		start[7] = 0.0034743;
		start[8] = 0.0043625;
		start[9] = 0.0038562;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0062755;
		start[3] = 0.0070689;
		start[4] = 0.0023985;
		start[5] = 0.0069335;
		start[6] = 0.0019507;
		start[7] = 0.0043782;
		start[8] = 0.0065431;
		start[9] = 0.0019529;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0077454;
		start[3] = 0.00064769;
		start[4] = 0.00030503;
		start[5] = 0.0099817;
		start[6] = 0.0069379;
		start[7] = 0.0094837;
		start[8] = 0.0088791;
		start[9] = 0.0096237;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0010066;
		start[3] = 0.009006;
		start[4] = 0.0052996;
		start[5] = 0.0081921;
		start[6] = 0.007507;
		start[7] = 0.0088818;
		start[8] = 0.0081342;
		start[9] = 0.0022276;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0015171;
		start[3] = 0.0076314;
		start[4] = 0.0096761;
		start[5] = 0.009052;
		start[6] = 0.0019467;
		start[7] = 0.0015159;
		start[8] = 0.0046843;
		start[9] = 0.0090966;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0019016;
		start[3] = 0.0066882;
		start[4] = 0.0044988;
		start[5] = 0.0080106;
		start[6] = 0.0061049;
		start[7] = 0.0042373;
		start[8] = 0.0046263;
		start[9] = 0.009377;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0091264;
		start[3] = 0.006056;
		start[4] = 0.0087941;
		start[5] = 0.0032308;
		start[6] = 0.0077201;
		start[7] = 0.0035199;
		start[8] = 0.003118;
		start[9] = 0.0080926;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0052489;
		start[3] = 0.0036362;
		start[4] = 0.006391;
		start[5] = 0.0019402;
		start[6] = 0.0073177;
		start[7] = 0.0026287;
		start[8] = 0.0023812;
		start[9] = 0.0052236;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0080947;
		start[3] = 0.0042309;
		start[4] = 0.0062713;
		start[5] = 0.0018015;
		start[6] = 0.0065749;
		start[7] = 0.0029684;
		start[8] = 0.0098925;
		start[9] = 0.0045021;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0035138;
		start[3] = 0.0058496;
		start[4] = 0.008836;
		start[5] = 0.0081236;
		start[6] = 0.0068277;
		start[7] = 0.0099416;
		start[8] = 0.0022054;
		start[9] = 0.0091907;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0089507;
		start[3] = 0.0089094;
		start[4] = 0.0028268;
		start[5] = 0.0031277;
		start[6] = 0.0051613;
		start[7] = 0.0092591;
		start[8] = 0.0053416;
		start[9] = 0.0067958;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0011246;
		start[3] = 0.0060659;
		start[4] = 0.0027968;
		start[5] = 0.0011533;
		start[6] = 0.0087323;
		start[7] = 0.0066919;
		start[8] = 0.0014415;
		start[9] = 0.0078021;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0038913;
		start[3] = 0.0031491;
		start[4] = 0.0086111;
		start[5] = 0.0091519;
		start[6] = 0.0027234;
		start[7] = 0.0085445;
		start[8] = 0.00066246;
		start[9] = 0.0063425;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0087876;
		start[3] = 0.0011676;
		start[4] = 0.0092157;
		start[5] = 0.0060486;
		start[6] = 0.00045065;
		start[7] = 0.00034714;
		start[8] = 0.0012689;
		start[9] = 0.0086536;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0052408;
		start[3] = 0.0018816;
		start[4] = 0.00083342;
		start[5] = 0.0027664;
		start[6] = 0.0010064;
		start[7] = 0.0089126;
		start[8] = 0.0086517;
		start[9] = 0.0028834;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0012183;
		start[3] = 0.0073235;
		start[4] = 0.0024151;
		start[5] = 0.0006922;
		start[6] = 0.0035447;
		start[7] = 0.0042306;
		start[8] = 0.0049641;
		start[9] = 0.0072451;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.009219;
		start[3] = 0.0069134;
		start[4] = 0.007963;
		start[5] = 0.0083725;
		start[6] = 0.0004705;
		start[7] = 0.0085647;
		start[8] = 0.0040304;
		start[9] = 0.0056316;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.00012703;
		start[3] = 0.0078537;
		start[4] = 0.0048554;
		start[5] = 0.0033184;
		start[6] = 0.0052425;
		start[7] = 0.0079633;
		start[8] = 0.0066855;
		start[9] = 0.0081457;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0093964;
		start[3] = 0.0060425;
		start[4] = 0.0044033;
		start[5] = 0.0073989;
		start[6] = 0.0025425;
		start[7] = 0.0040019;
		start[8] = 0.0078323;
		start[9] = 0.007146;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0066657;
		start[3] = 0.0036715;
		start[4] = 0.0017929;
		start[5] = 0.0044269;
		start[6] = 0.008136;
		start[7] = 0.0015075;
		start[8] = 0.00020693;
		start[9] = 0.0080864;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0035262;
		start[3] = 0.0022114;
		start[4] = 0.0081113;
		start[5] = 0.001044;
		start[6] = 0.0025323;
		start[7] = 0.0080525;
		start[8] = 0.0030474;
		start[9] = 0.0070205;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.0067895;
		start[3] = 0.0028135;
		start[4] = 0.0019493;
		start[5] = 0.0026952;
		start[6] = 0.0015935;
		start[7] = 0.0026076;
		start[8] = 0.0011555;
		start[9] = 0.0054792;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.045744;
		start[1] = -0.065836;
		start[2] = 0.0066006;
		start[3] = 0.0044136;
		start[4] = 0.0048915;
		start[5] = 0.009523;
		start[6] = 0.0019777;
		start[7] = 0.0083714;
		start[8] = 0.0029899;
		start[9] = 0.0064307;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		start[2] = 0.0048065;
		start[3] = 0.00061264;
		start[4] = 0.0048325;
		start[5] = 0.0063812;
		start[6] = 0.0038515;
		start[7] = 0.0026199;
		start[8] = 0.006078;
		start[9] = 0.0041692;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.019342;
		start[1] = 0.07281;
		start[2] = 0.0036424;
		start[3] = 0.00050079;
		start[4] = 0.0050481;
		start[5] = 0.0062037;
		start[6] = 0.009467;
		start[7] = 0.0074648;
		start[8] = 0.0088528;
		start[9] = 0.0045039;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.050277;
		start[1] = 0.034234;
		start[2] = 0.0057733;
		start[3] = 0.0062246;
		start[4] = 0.002394;
		start[5] = 0.0013017;
		start[6] = 0.0025351;
		start[7] = 0.0023231;
		start[8] = 0.0025018;
		start[9] = 0.0020507;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.089024;
		start[1] = -0.044114;
		start[2] = 0.0078909;
		start[3] = 0.00465;
		start[4] = 0.00079811;
		start[5] = 0.00056451;
		start[6] = 0.0064844;
		start[7] = 0.0011891;
		start[8] = 0.0049841;
		start[9] = 0.0061881;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.058285;
		start[1] = -0.095648;
		start[2] = 0.0022698;
		start[3] = 0.0096225;
		start[4] = 0.0075;
		start[5] = 0.0038837;
		start[6] = 0.0029398;
		start[7] = 0.0089823;
		start[8] = 0.00386;
		start[9] = 0.00012075;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.080163;
		start[1] = -0.039822;
		start[2] = 0.0013363;
		start[3] = 0.0063246;
		start[4] = 0.0055159;
		start[5] = 0.0025278;
		start[6] = 0.0039692;
		start[7] = 0.001744;
		start[8] = 0.0076255;
		start[9] = 0.0079304;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.072837;
		start[1] = 0.036878;
		start[2] = 0.0047836;
		start[3] = 0.00085622;
		start[4] = 0.0061817;
		start[5] = 0.0097154;
		start[6] = 0.005011;
		start[7] = 0.0047581;
		start[8] = 0.0099924;
		start[9] = 0.0071708;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = 0.017422;
		start[1] = 0.094855;
		start[2] = 0.0046979;
		start[3] = 0.00019011;
		start[4] = 0.0051528;
		start[5] = 0.0069729;
		start[6] = 0.00074735;
		start[7] = 0.00081946;
		start[8] = 0.0062356;
		start[9] = 0.0041306;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (10);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		start[2] = 0.004187;
		start[3] = 0.0054015;
		start[4] = 0.00074818;
		start[5] = 0.005704;
		start[6] = 0.0083091;
		start[7] = 0.0029629;
		start[8] = 0.0064406;
		start[9] = 0.0013784;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}


  return 0;
}
