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
    (14, 1, "CostFunction_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

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
    (14, 1, "LiftConstraint_1_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

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
    (14, 1, "LiftConstraint_2_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
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
	const double& w_01_05 = x[4];
	const double& w_01_06 = x[5];
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

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
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
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
    (14, 1, "LiftConstraint_3_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

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
    (14, 1, "LiftConstraint_4_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
	result[0] = w_01_01*w_01_05 - 1.0*w_02_01;
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

  switch (id)
    {
      
		case 0: 
			 grad[0] = w_01_05; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = w_01_01; 
			 grad[5] = 0.0; 
			 grad[6] = -1.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
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
    (14, 1, "LiftConstraint_5_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
	result[0] = w_01_02*w_01_05 - 1.0*w_02_02;
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = w_01_05; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = w_01_02; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = -1.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
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
    (14, 1, "LiftConstraint_6_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
	result[0] = w_01_01*w_01_06 - 1.0*w_02_03;
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

  switch (id)
    {
      
		case 0: 
			 grad[0] = w_01_06; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_01_01; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = -1.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
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
    (14, 1, "LiftConstraint_7_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
	result[0] = w_01_02*w_01_06 - 1.0*w_02_04;
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = w_01_06; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_01_02; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = -1.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
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
    (14, 1, "LiftConstraint_8_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
	result[0] = w_01_03*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_01;
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = w_02_01 - 1.0*w_02_04; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = w_01_03; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = -1.0*w_01_03; 
			 grad[10] = -1.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
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
    (14, 1, "LiftConstraint_9_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
	result[0] = - 1.0*w_03_02 - 1.0*w_01_04*(w_02_02 + w_02_03);
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = -1.0*w_01_04; 
			 grad[8] = -1.0*w_01_04; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = -1.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
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
    (14, 1, "LiftConstraint_10_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
	result[0] = w_01_03*(w_02_02 + w_02_03) - 1.0*w_03_03;
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = w_02_02 + w_02_03; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = w_01_03; 
			 grad[8] = w_01_03; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = -1.0; 
			 grad[13] = 0.0; 
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
    (14, 1, "LiftConstraint_11_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
	result[0] = w_01_04*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_04;
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_02_01 - 1.0*w_02_04; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = w_01_04; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = -1.0*w_01_04; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = -1.0; 
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
    (14, 1, "EEConstraint_1_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
	result[0] = w_01_01 - 1.0*EE_1_1 + w_02_01 - 1.0*w_02_04 + w_03_01 + w_03_02;
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 1.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 1.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = -1.0; 
			 grad[10] = 1.0; 
			 grad[11] = 1.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
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
    (14, 1, "EEConstraint_2_planarRobot3"),
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];
  
	result[0] = w_01_02 - 1.0*EE_1_2 + w_02_02 + w_02_03 + w_03_03 + w_03_04;
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
	const double& w_02_01 = x[6];
	const double& w_02_02 = x[7];
	const double& w_02_03 = x[8];
	const double& w_02_04 = x[9];
	const double& w_03_01 = x[10];
	const double& w_03_02 = x[11];
	const double& w_03_03 = x[12];
	const double& w_03_04 = x[13];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 1.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 1.0; 
			 grad[8] = 1.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 1.0; 
			 grad[13] = 1.0; 
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
	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_12 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_13 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);

  //Create problem
  solver_t::problem_t pb (*cost);

  // Set bounds for all optimization parameters
  pb.argumentBounds ()[0] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[1] = roboptim::Function::makeInterval (-pi, pi);
	pb.argumentBounds ()[2] = roboptim::Function::makeInterval (-pi, pi);

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
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_12), bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_13), bounds, scales); 
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
		std::vector<double> start (14);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		start[3] = 0.0060882;
		start[4] = 0.0084598;
		start[5] = 0.00036051;
		start[6] = 0.0043598;
		start[7] = 0.0089317;
		start[8] = 0.0030286;
		start[9] = 0.003375;
		start[10] = 0.0049926;
		start[11] = 0.0030297;
		start[12] = 0.001894;
		start[13] = 0.0063885;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		start[3] = 0.0064651;
		start[4] = 0.0003013;
		start[5] = 0.0058771;
		start[6] = 0.0065882;
		start[7] = 0.00070722;
		start[8] = 0.0088229;
		start[9] = 0.0070893;
		start[10] = 0.0050968;
		start[11] = 0.0081811;
		start[12] = 0.0005682;
		start[13] = 0.0026468;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		start[3] = 0.002395;
		start[4] = 0.007911;
		start[5] = 0.00194;
		start[6] = 0.0085187;
		start[7] = 0.0099912;
		start[8] = 0.0056117;
		start[9] = 0.003723;
		start[10] = 0.0099422;
		start[11] = 0.0063892;
		start[12] = 0.0045015;
		start[13] = 0.0074428;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		start[3] = 0.0024791;
		start[4] = 0.0081249;
		start[5] = 0.0046024;
		start[6] = 0.0073894;
		start[7] = 0.0054853;
		start[8] = 0.0056742;
		start[9] = 0.0088816;
		start[10] = 0.0021669;
		start[11] = 0.0061246;
		start[12] = 0.0021883;
		start[13] = 0.0076485;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		start[3] = 0.0073064;
		start[4] = 0.0067682;
		start[5] = 0.0046666;
		start[6] = 0.0025385;
		start[7] = 0.002064;
		start[8] = 0.0055959;
		start[9] = 0.0077244;
		start[10] = 0.0056901;
		start[11] = 0.0050714;
		start[12] = 0.0093538;
		start[13] = 0.0043613;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		start[3] = 0.0075671;
		start[4] = 0.0041382;
		start[5] = 0.0077718;
		start[6] = 0.0040733;
		start[7] = 0.0035105;
		start[8] = 0.0088889;
		start[9] = 0.00050381;
		start[10] = 0.0048454;
		start[11] = 0.0080997;
		start[12] = 0.0044467;
		start[13] = 0.0054404;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		start[3] = 0.0046743;
		start[4] = 0.001244;
		start[5] = 0.0025009;
		start[6] = 0.0044573;
		start[7] = 0.0018848;
		start[8] = 0.0064648;
		start[9] = 0.0023293;
		start[10] = 0.0013831;
		start[11] = 0.00051731;
		start[12] = 0.0077231;
		start[13] = 0.0070702;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		start[3] = 0.0061785;
		start[4] = 0.0054393;
		start[5] = 0.00032246;
		start[6] = 0.0057146;
		start[7] = 0.009596;
		start[8] = 0.0082974;
		start[9] = 0.0039205;
		start[10] = 0.0081143;
		start[11] = 0.0091092;
		start[12] = 0.0081055;
		start[13] = 0.0032224;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		start[3] = 0.0065747;
		start[4] = 0.0010995;
		start[5] = 0.0082726;
		start[6] = 0.0057181;
		start[7] = 0.0027316;
		start[8] = 0.00095016;
		start[9] = 0.00225;
		start[10] = 0.0013791;
		start[11] = 0.001879;
		start[12] = 0.0015287;
		start[13] = 0.002779;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		start[3] = 0.0067127;
		start[4] = 0.0010681;
		start[5] = 0.0049379;
		start[6] = 0.00062788;
		start[7] = 0.0053419;
		start[8] = 0.0054942;
		start[9] = 0.008357;
		start[10] = 0.00457;
		start[11] = 0.0065481;
		start[12] = 0.0097503;
		start[13] = 0.0072196;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		start[3] = 0.0064666;
		start[4] = 0.00043526;
		start[5] = 0.0059717;
		start[6] = 0.0049174;
		start[7] = 0.0018224;
		start[8] = 0.008938;
		start[9] = 0.0015312;
		start[10] = 0.0045922;
		start[11] = 0.0095091;
		start[12] = 0.0059286;
		start[13] = 0.002523;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		start[3] = 0.0077025;
		start[4] = 0.0091288;
		start[5] = 0.0040206;
		start[6] = 0.0093172;
		start[7] = 0.0067251;
		start[8] = 0.0093003;
		start[9] = 0.0072987;
		start[10] = 0.0059181;
		start[11] = 0.0011065;
		start[12] = 0.0078578;
		start[13] = 0.0061331;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		start[3] = 0.003526;
		start[4] = 0.0062332;
		start[5] = 0.0047449;
		start[6] = 0.0062594;
		start[7] = 0.0064124;
		start[8] = 0.0088768;
		start[9] = 0.00074238;
		start[10] = 0.0064838;
		start[11] = 0.0061906;
		start[12] = 0.0043038;
		start[13] = 0.0020701;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		start[3] = 0.0064001;
		start[4] = 0.00055948;
		start[5] = 0.0071825;
		start[6] = 0.0069049;
		start[7] = 0.00061688;
		start[8] = 0.0062505;
		start[9] = 0.007828;
		start[10] = 0.0045036;
		start[11] = 0.0015813;
		start[12] = 0.0092403;
		start[13] = 0.0095171;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		start[3] = 0.0038619;
		start[4] = 0.008705;
		start[5] = 0.0073889;
		start[6] = 0.009611;
		start[7] = 0.0083457;
		start[8] = 0.0040109;
		start[9] = 0.0050524;
		start[10] = 0.0064638;
		start[11] = 0.0077397;
		start[12] = 0.009138;
		start[13] = 0.0070311;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		start[3] = 0.0062357;
		start[4] = 0.0096811;
		start[5] = 0.00078827;
		start[6] = 0.0019524;
		start[7] = 0.0075259;
		start[8] = 0.0053057;
		start[9] = 0.009022;
		start[10] = 0.0048456;
		start[11] = 0.0097667;
		start[12] = 0.0022644;
		start[13] = 0.0071573;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		start[3] = 0.0061337;
		start[4] = 0.0051432;
		start[5] = 0.0067043;
		start[6] = 0.0092002;
		start[7] = 0.0016562;
		start[8] = 0.0023295;
		start[9] = 0.0078423;
		start[10] = 0.0027111;
		start[11] = 0.0071964;
		start[12] = 0.0058192;
		start[13] = 0.0089366;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		start[3] = 0.0061306;
		start[4] = 0.0083404;
		start[5] = 0.0076472;
		start[6] = 0.0028989;
		start[7] = 0.0076804;
		start[8] = 0.0020109;
		start[9] = 0.0086711;
		start[10] = 0.0060229;
		start[11] = 0.0043315;
		start[12] = 0.0034452;
		start[13] = 0.00053415;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		start[3] = 0.0031862;
		start[4] = 0.0030332;
		start[5] = 0.0077182;
		start[6] = 0.0096888;
		start[7] = 0.0010057;
		start[8] = 0.0095403;
		start[9] = 0.0087813;
		start[10] = 0.0056165;
		start[11] = 0.0014619;
		start[12] = 0.0072318;
		start[13] = 0.00095764;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		start[3] = 0.0052432;
		start[4] = 0.00018282;
		start[5] = 0.0012233;
		start[6] = 0.00098422;
		start[7] = 0.0014458;
		start[8] = 0.0032232;
		start[9] = 0.0080139;
		start[10] = 0.0097418;
		start[11] = 0.0036737;
		start[12] = 0.0091102;
		start[13] = 0.0064072;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		start[3] = 0.0008611;
		start[4] = 0.0052334;
		start[5] = 0.00128;
		start[6] = 0.00048054;
		start[7] = 0.0022207;
		start[8] = 0.0021477;
		start[9] = 0.0057145;
		start[10] = 0.0043193;
		start[11] = 0.0021911;
		start[12] = 0.0048548;
		start[13] = 0.0028242;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		start[3] = 0.0048005;
		start[4] = 0.0068417;
		start[5] = 0.0019803;
		start[6] = 0.0015929;
		start[7] = 0.0087504;
		start[8] = 0.0064749;
		start[9] = 0.0057748;
		start[10] = 0.0057469;
		start[11] = 0.0037814;
		start[12] = 0.0076394;
		start[13] = 0.00055384;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		start[3] = 0.0038233;
		start[4] = 0.0057724;
		start[5] = 0.0069283;
		start[6] = 0.0056843;
		start[7] = 0.0041135;
		start[8] = 0.0028835;
		start[9] = 0.0071926;
		start[10] = 0.0078922;
		start[11] = 0.0087869;
		start[12] = 0.0066865;
		start[13] = 0.0031485;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		start[3] = 0.0036815;
		start[4] = 0.0045183;
		start[5] = 0.0091499;
		start[6] = 0.0068727;
		start[7] = 0.0078994;
		start[8] = 0.0066623;
		start[9] = 0.00057017;
		start[10] = 0.00064561;
		start[11] = 0.0020608;
		start[12] = 0.0051792;
		start[13] = 0.00088583;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		start[3] = 0.0058837;
		start[4] = 0.0065178;
		start[5] = 0.0036046;
		start[6] = 0.0066668;
		start[7] = 0.0042764;
		start[8] = 0.0084511;
		start[9] = 0.00015299;
		start[10] = 0.007501;
		start[11] = 0.0081998;
		start[12] = 0.0011176;
		start[13] = 0.0096011;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		start[3] = 0.004192;
		start[4] = 0.0053644;
		start[5] = 0.0046699;
		start[6] = 0.0028002;
		start[7] = 0.0074484;
		start[8] = 0.00046278;
		start[9] = 0.005174;
		start[10] = 0.0016462;
		start[11] = 0.0082105;
		start[12] = 0.0076644;
		start[13] = 0.0015346;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		start[3] = 0.0084981;
		start[4] = 0.0029181;
		start[5] = 0.0072302;
		start[6] = 0.0050376;
		start[7] = 0.007422;
		start[8] = 0.0019871;
		start[9] = 0.0081091;
		start[10] = 0.0084823;
		start[11] = 0.0015588;
		start[12] = 0.0069521;
		start[13] = 0.0062361;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		start[3] = 4.4947e-06;
		start[4] = 0.0049545;
		start[5] = 0.0041427;
		start[6] = 0.00054798;
		start[7] = 0.0056852;
		start[8] = 0.0086688;
		start[9] = 0.0095513;
		start[10] = 0.0052412;
		start[11] = 0.0078122;
		start[12] = 0.00037949;
		start[13] = 0.0013778;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		start[3] = 0.00028686;
		start[4] = 0.00813;
		start[5] = 0.0032511;
		start[6] = 0.0090997;
		start[7] = 0.0092466;
		start[8] = 0.0094203;
		start[9] = 0.004534;
		start[10] = 0.0019293;
		start[11] = 0.0013825;
		start[12] = 0.0048058;
		start[13] = 0.00064915;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		start[3] = 0.0086654;
		start[4] = 0.0016303;
		start[5] = 0.0040116;
		start[6] = 0.0070735;
		start[7] = 0.0040658;
		start[8] = 0.0075588;
		start[9] = 0.0085892;
		start[10] = 0.0083855;
		start[11] = 0.0067626;
		start[12] = 0.0052065;
		start[13] = 0.0095225;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		start[3] = 0.0074426;
		start[4] = 0.007224;
		start[5] = 0.0012565;
		start[6] = 0.0058803;
		start[7] = 0.0021668;
		start[8] = 0.00034484;
		start[9] = 0.0099881;
		start[10] = 0.0037764;
		start[11] = 0.0095981;
		start[12] = 0.0042352;
		start[13] = 0.0016476;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		start[3] = 0.0026162;
		start[4] = 0.0031283;
		start[5] = 0.0019459;
		start[6] = 0.0011677;
		start[7] = 0.0094606;
		start[8] = 0.0044011;
		start[9] = 0.0099696;
		start[10] = 0.0041474;
		start[11] = 0.0051269;
		start[12] = 0.0015187;
		start[13] = 0.004538;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		start[3] = 0.0096278;
		start[4] = 0.00024773;
		start[5] = 1.1472e-05;
		start[6] = 0.0053162;
		start[7] = 0.0078763;
		start[8] = 0.002432;
		start[9] = 0.0091671;
		start[10] = 0.0057854;
		start[11] = 0.0084066;
		start[12] = 4.3776e-05;
		start[13] = 0.0065059;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		start[3] = 0.0025961;
		start[4] = 0.0072939;
		start[5] = 0.0033919;
		start[6] = 0.0014894;
		start[7] = 0.0036077;
		start[8] = 0.00012756;
		start[9] = 8.9133e-05;
		start[10] = 0.0038856;
		start[11] = 0.0088697;
		start[12] = 0.0094269;
		start[13] = 0.0033093;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		start[3] = 0.0038398;
		start[4] = 0.0048758;
		start[5] = 0.0061552;
		start[6] = 0.0011346;
		start[7] = 0.003075;
		start[8] = 0.0079517;
		start[9] = 0.0047341;
		start[10] = 0.0025506;
		start[11] = 0.0055538;
		start[12] = 0.0096326;
		start[13] = 0.0016269;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		start[3] = 0.0056945;
		start[4] = 0.0070046;
		start[5] = 0.0069145;
		start[6] = 0.0086925;
		start[7] = 0.00031737;
		start[8] = 0.00060457;
		start[9] = 0.0052576;
		start[10] = 0.00364;
		start[11] = 0.0053464;
		start[12] = 0.0070127;
		start[13] = 0.0033829;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		start[3] = 0.0097658;
		start[4] = 0.0064965;
		start[5] = 0.0094582;
		start[6] = 0.0063971;
		start[7] = 0.0005793;
		start[8] = 0.0042541;
		start[9] = 0.0047281;
		start[10] = 0.0095587;
		start[11] = 0.0057139;
		start[12] = 0.00036739;
		start[13] = 0.0080349;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		start[3] = 0.0033698;
		start[4] = 0.0031059;
		start[5] = 0.0052095;
		start[6] = 0.0058165;
		start[7] = 0.0064099;
		start[8] = 0.0055266;
		start[9] = 0.00023801;
		start[10] = 0.0043717;
		start[11] = 0.0022086;
		start[12] = 0.006931;
		start[13] = 0.0064675;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		start[3] = 0.0041931;
		start[4] = 0.0088877;
		start[5] = 0.0057475;
		start[6] = 0.0024945;
		start[7] = 0.0043872;
		start[8] = 0.0040073;
		start[9] = 0.0049423;
		start[10] = 0.0040806;
		start[11] = 0.0086129;
		start[12] = 0.0090881;
		start[13] = 0.0067614;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		start[3] = 0.0038347;
		start[4] = 0.008717;
		start[5] = 0.0017792;
		start[6] = 0.001055;
		start[7] = 0.0014538;
		start[8] = 0.0032827;
		start[9] = 0.0081616;
		start[10] = 0.0066046;
		start[11] = 0.0023711;
		start[12] = 0.0070481;
		start[13] = 0.0076623;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		start[3] = 0.0053744;
		start[4] = 0.0015695;
		start[5] = 0.002372;
		start[6] = 0.0019299;
		start[7] = 0.0077898;
		start[8] = 0.0097027;
		start[9] = 0.0035863;
		start[10] = 0.0087374;
		start[11] = 0.0015809;
		start[12] = 0.0081507;
		start[13] = 0.0028852;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		start[3] = 0.00051789;
		start[4] = 0.0018838;
		start[5] = 0.0067557;
		start[6] = 0.0067391;
		start[7] = 0.0058212;
		start[8] = 0.001326;
		start[9] = 0.0069078;
		start[10] = 0.00018937;
		start[11] = 0.0011177;
		start[12] = 0.009505;
		start[13] = 0.0025556;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		start[3] = 0.0095336;
		start[4] = 0.0050747;
		start[5] = 0.0043836;
		start[6] = 0.0024652;
		start[7] = 0.0086752;
		start[8] = 0.0038934;
		start[9] = 0.0009077;
		start[10] = 0.0033138;
		start[11] = 0.0077629;
		start[12] = 0.0082783;
		start[13] = 0.00055975;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		start[3] = 0.0030575;
		start[4] = 0.008215;
		start[5] = 0.0052025;
		start[6] = 0.0073594;
		start[7] = 0.0041448;
		start[8] = 0.0023343;
		start[9] = 0.0014508;
		start[10] = 0.0027843;
		start[11] = 0.0028136;
		start[12] = 0.0041835;
		start[13] = 0.00031714;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		start[3] = 0.0042689;
		start[4] = 0.00053454;
		start[5] = 0.0093759;
		start[6] = 0.0094361;
		start[7] = 0.0057152;
		start[8] = 0.0077053;
		start[9] = 0.002909;
		start[10] = 0.00093317;
		start[11] = 0.0087145;
		start[12] = 0.003498;
		start[13] = 0.0066208;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		start[3] = 0.0035926;
		start[4] = 0.00165;
		start[5] = 0.0061287;
		start[6] = 0.00081243;
		start[7] = 0.0094785;
		start[8] = 0.0062541;
		start[9] = 0.0027957;
		start[10] = 0.0049954;
		start[11] = 0.006993;
		start[12] = 0.0079345;
		start[13] = 0.0051044;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		start[3] = 0.00086466;
		start[4] = 0.0067152;
		start[5] = 0.0030458;
		start[6] = 0.0095132;
		start[7] = 0.0054126;
		start[8] = 0.0014053;
		start[9] = 0.0091355;
		start[10] = 0.00057241;
		start[11] = 0.0031296;
		start[12] = 0.0022829;
		start[13] = 0.0086643;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		start[3] = 0.0087122;
		start[4] = 0.0086217;
		start[5] = 0.0023489;
		start[6] = 0.0035908;
		start[7] = 0.0050145;
		start[8] = 0.0054779;
		start[9] = 0.0048008;
		start[10] = 0.0083612;
		start[11] = 0.0019814;
		start[12] = 0.0013633;
		start[13] = 0.00033477;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		start[3] = 0.0078383;
		start[4] = 0.0079388;
		start[5] = 0.0065331;
		start[6] = 0.0069202;
		start[7] = 0.00042147;
		start[8] = 0.0093463;
		start[9] = 0.0059875;
		start[10] = 0.0013942;
		start[11] = 0.0076709;
		start[12] = 0.0061591;
		start[13] = 0.0057739;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		start[3] = 0.0050699;
		start[4] = 0.0083136;
		start[5] = 0.0034223;
		start[6] = 0.0087851;
		start[7] = 0.0031011;
		start[8] = 0.0068282;
		start[9] = 4.9656e-05;
		start[10] = 0.0033704;
		start[11] = 0.0045739;
		start[12] = 0.0028949;
		start[13] = 0.0076902;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		start[3] = 0.0013504;
		start[4] = 0.0085204;
		start[5] = 0.0078837;
		start[6] = 0.0068197;
		start[7] = 0.0038863;
		start[8] = 0.0086721;
		start[9] = 0.0018136;
		start[10] = 0.0021254;
		start[11] = 0.0096241;
		start[12] = 0.0043378;
		start[13] = 0.0059556;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		start[3] = 0.0075455;
		start[4] = 0.006923;
		start[5] = 0.0044393;
		start[6] = 0.0061575;
		start[7] = 0.001672;
		start[8] = 0.0064877;
		start[9] = 0.0024784;
		start[10] = 0.0051567;
		start[11] = 0.0056633;
		start[12] = 0.0072164;
		start[13] = 0.0033124;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		start[3] = 0.0099059;
		start[4] = 0.0031856;
		start[5] = 0.0017149;
		start[6] = 0.0079195;
		start[7] = 0.00051955;
		start[8] = 0.007919;
		start[9] = 0.0086164;
		start[10] = 0.0010643;
		start[11] = 0.0017387;
		start[12] = 0.00014835;
		start[13] = 0.0028564;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		start[3] = 7.368e-05;
		start[4] = 0.0076201;
		start[5] = 0.0087993;
		start[6] = 0.0079541;
		start[7] = 0.0026565;
		start[8] = 0.0062339;
		start[9] = 0.00052677;
		start[10] = 0.0073715;
		start[11] = 0.0033756;
		start[12] = 0.0068184;
		start[13] = 0.0032199;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		start[3] = 0.0021373;
		start[4] = 0.0066265;
		start[5] = 1.1688e-05;
		start[6] = 0.0075171;
		start[7] = 0.0073716;
		start[8] = 0.0055456;
		start[9] = 0.0099608;
		start[10] = 0.0038178;
		start[11] = 0.0085159;
		start[12] = 0.0095635;
		start[13] = 0.0018417;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		start[3] = 0.0027703;
		start[4] = 0.0018152;
		start[5] = 0.0093566;
		start[6] = 0.0067931;
		start[7] = 0.0034385;
		start[8] = 0.0048076;
		start[9] = 0.0049434;
		start[10] = 0.003781;
		start[11] = 0.0043336;
		start[12] = 0.0089766;
		start[13] = 0.0031642;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		start[3] = 0.0070981;
		start[4] = 0.0079831;
		start[5] = 0.0097012;
		start[6] = 0.0053188;
		start[7] = 0.0040105;
		start[8] = 0.0084067;
		start[9] = 0.0016625;
		start[10] = 0.0047426;
		start[11] = 0.0016795;
		start[12] = 0.0021359;
		start[13] = 0.0044525;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		start[3] = 0.0048945;
		start[4] = 0.0097281;
		start[5] = 0.0064634;
		start[6] = 0.0096243;
		start[7] = 0.0077216;
		start[8] = 0.0090493;
		start[9] = 0.00024717;
		start[10] = 0.0093167;
		start[11] = 0.0024041;
		start[12] = 0.0016127;
		start[13] = 0.0096132;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		start[3] = 0.0046377;
		start[4] = 0.00081423;
		start[5] = 0.0086225;
		start[6] = 0.00026332;
		start[7] = 0.0088999;
		start[8] = 0.0080596;
		start[9] = 0.0038648;
		start[10] = 0.006568;
		start[11] = 0.0034195;
		start[12] = 0.0021758;
		start[13] = 0.0084846;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		start[3] = 0.00086127;
		start[4] = 0.0090273;
		start[5] = 0.0066665;
		start[6] = 0.0017644;
		start[7] = 0.0061148;
		start[8] = 0.0099411;
		start[9] = 0.0033345;
		start[10] = 0.0044813;
		start[11] = 0.005033;
		start[12] = 0.00054959;
		start[13] = 0.0055214;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		start[3] = 0.0044726;
		start[4] = 0.00185;
		start[5] = 0.0011542;
		start[6] = 0.0013551;
		start[7] = 0.0094936;
		start[8] = 0.0054791;
		start[9] = 0.00090027;
		start[10] = 0.0082652;
		start[11] = 0.0047604;
		start[12] = 0.0090889;
		start[13] = 0.0019499;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		start[3] = 0.0043513;
		start[4] = 0.0068835;
		start[5] = 0.008828;
		start[6] = 0.0046571;
		start[7] = 0.00081737;
		start[8] = 0.0068066;
		start[9] = 0.00091109;
		start[10] = 0.00056193;
		start[11] = 0.0098776;
		start[12] = 0.0023381;
		start[13] = 0.0081229;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		start[3] = 0.0014171;
		start[4] = 0.0078805;
		start[5] = 0.0053724;
		start[6] = 0.002356;
		start[7] = 0.0040917;
		start[8] = 0.0016343;
		start[9] = 0.0044108;
		start[10] = 0.0042683;
		start[11] = 0.0031411;
		start[12] = 0.0094188;
		start[13] = 0.0024618;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		start[3] = 0.0099813;
		start[4] = 0.00056227;
		start[5] = 0.0086361;
		start[6] = 0.0058616;
		start[7] = 0.0058677;
		start[8] = 0.0039379;
		start[9] = 0.0048546;
		start[10] = 0.0049682;
		start[11] = 0.0080123;
		start[12] = 0.0042897;
		start[13] = 0.0060603;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		start[3] = 0.0063263;
		start[4] = 0.0052569;
		start[5] = 0.0040406;
		start[6] = 0.0068958;
		start[7] = 0.0042523;
		start[8] = 0.002387;
		start[9] = 0.0077444;
		start[10] = 0.0019994;
		start[11] = 0.0033339;
		start[12] = 0.0034607;
		start[13] = 0.0043928;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		start[3] = 0.0084049;
		start[4] = 0.0079813;
		start[5] = 0.0066577;
		start[6] = 0.0025335;
		start[7] = 0.0034926;
		start[8] = 0.0045521;
		start[9] = 0.0070672;
		start[10] = 0.0049906;
		start[11] = 0.0034841;
		start[12] = 0.0080647;
		start[13] = 0.0017312;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		start[3] = 0.0021491;
		start[4] = 0.0084538;
		start[5] = 0.0043323;
		start[6] = 0.0094004;
		start[7] = 0.0078239;
		start[8] = 0.0082808;
		start[9] = 0.0027529;
		start[10] = 0.0038815;
		start[11] = 0.009336;
		start[12] = 0.0044594;
		start[13] = 0.0026317;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		start[3] = 0.007371;
		start[4] = 0.0055551;
		start[5] = 0.0050772;
		start[6] = 0.0034667;
		start[7] = 4.5075e-05;
		start[8] = 0.002827;
		start[9] = 0.0084526;
		start[10] = 0.0031482;
		start[11] = 0.0094132;
		start[12] = 0.0066827;
		start[13] = 0.00010256;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		start[3] = 0.0073661;
		start[4] = 0.0038075;
		start[5] = 0.0099899;
		start[6] = 0.0020268;
		start[7] = 0.0014976;
		start[8] = 0.0080111;
		start[9] = 0.0096538;
		start[10] = 0.0092139;
		start[11] = 0.0076914;
		start[12] = 0.0043309;
		start[13] = 0.00052887;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		start[3] = 0.0096003;
		start[4] = 0.0015581;
		start[5] = 0.0014044;
		start[6] = 0.0071507;
		start[7] = 0.009456;
		start[8] = 0.0074492;
		start[9] = 0.0056113;
		start[10] = 0.0014536;
		start[11] = 0.0063409;
		start[12] = 0.0062757;
		start[13] = 0.0084827;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		start[3] = 0.009028;
		start[4] = 0.001214;
		start[5] = 0.00079647;
		start[6] = 0.0054155;
		start[7] = 0.0084345;
		start[8] = 0.00054924;
		start[9] = 0.0079765;
		start[10] = 0.00011873;
		start[11] = 0.0029538;
		start[12] = 0.0021978;
		start[13] = 0.0098009;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		start[3] = 0.00092657;
		start[4] = 0.0072165;
		start[5] = 0.0080576;
		start[6] = 0.0034443;
		start[7] = 0.0096104;
		start[8] = 0.0078753;
		start[9] = 0.0046323;
		start[10] = 0.0041239;
		start[11] = 0.00010954;
		start[12] = 0.0038194;
		start[13] = 0.00044886;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		start[3] = 0.0084253;
		start[4] = 0.00060565;
		start[5] = 0.0017306;
		start[6] = 0.0098879;
		start[7] = 0.0034202;
		start[8] = 0.0053391;
		start[9] = 0.0094076;
		start[10] = 0.00038341;
		start[11] = 0.0017407;
		start[12] = 0.0016987;
		start[13] = 0.0077716;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		start[3] = 0.00033992;
		start[4] = 0.0049037;
		start[5] = 0.0074696;
		start[6] = 0.007667;
		start[7] = 0.0030645;
		start[8] = 0.0019702;
		start[9] = 0.0050136;
		start[10] = 0.00029427;
		start[11] = 0.0042884;
		start[12] = 0.0058227;
		start[13] = 0.00070604;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		start[3] = 0.0068211;
		start[4] = 0.0087222;
		start[5] = 0.00048825;
		start[6] = 0.00070446;
		start[7] = 0.0037033;
		start[8] = 0.0018223;
		start[9] = 0.00063153;
		start[10] = 0.0032663;
		start[11] = 0.0086057;
		start[12] = 0.005952;
		start[13] = 0.0042041;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		start[3] = 0.0050202;
		start[4] = 0.003502;
		start[5] = 0.0012888;
		start[6] = 0.0053525;
		start[7] = 0.003975;
		start[8] = 0.0059743;
		start[9] = 0.0034317;
		start[10] = 0.0019143;
		start[11] = 5.06e-05;
		start[12] = 0.001331;
		start[13] = 0.0053671;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		start[3] = 0.0015702;
		start[4] = 0.004276;
		start[5] = 0.0092158;
		start[6] = 0.0082695;
		start[7] = 0.0098612;
		start[8] = 0.0085615;
		start[9] = 0.0020112;
		start[10] = 0.0032606;
		start[11] = 0.00843;
		start[12] = 0.00067359;
		start[13] = 0.0079709;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		start[3] = 0.0071124;
		start[4] = 0.0035685;
		start[5] = 0.0040976;
		start[6] = 0.00066035;
		start[7] = 0.0083686;
		start[8] = 0.00297;
		start[9] = 0.008969;
		start[10] = 0.003065;
		start[11] = 0.0040656;
		start[12] = 0.0085675;
		start[13] = 0.0047668;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		start[3] = 0.0050389;
		start[4] = 0.0068181;
		start[5] = 0.0027881;
		start[6] = 0.0082063;
		start[7] = 0.009979;
		start[8] = 0.0075178;
		start[9] = 0.0093264;
		start[10] = 0.0071205;
		start[11] = 0.0064392;
		start[12] = 0.0092597;
		start[13] = 0.0076068;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		start[3] = 0.0015801;
		start[4] = 0.0084703;
		start[5] = 0.0098135;
		start[6] = 0.0048685;
		start[7] = 0.0093263;
		start[8] = 0.0053265;
		start[9] = 0.0069984;
		start[10] = 0.0068441;
		start[11] = 0.0096194;
		start[12] = 0.0071454;
		start[13] = 0.0074202;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		start[3] = 0.0028224;
		start[4] = 0.0087018;
		start[5] = 0.001227;
		start[6] = 0.0039139;
		start[7] = 0.0077251;
		start[8] = 0.0083419;
		start[9] = 0.0050838;
		start[10] = 0.0099826;
		start[11] = 0.0066427;
		start[12] = 0.00095399;
		start[13] = 0.0069487;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		start[3] = 0.0060752;
		start[4] = 0.0037171;
		start[5] = 0.00019256;
		start[6] = 0.0093136;
		start[7] = 0.0022057;
		start[8] = 0.0009033;
		start[9] = 0.0054663;
		start[10] = 0.004331;
		start[11] = 0.0052926;
		start[12] = 0.0089552;
		start[13] = 0.00070089;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		start[3] = 0.00077133;
		start[4] = 0.0019715;
		start[5] = 0.0064048;
		start[6] = 0.0032033;
		start[7] = 0.0010303;
		start[8] = 0.0056046;
		start[9] = 0.0042883;
		start[10] = 0.0071005;
		start[11] = 0.0065752;
		start[12] = 0.0094981;
		start[13] = 0.0055501;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		start[3] = 0.00094728;
		start[4] = 0.0042928;
		start[5] = 0.0031;
		start[6] = 0.0072968;
		start[7] = 0.0019726;
		start[8] = 0.001571;
		start[9] = 0.0022719;
		start[10] = 0.002099;
		start[11] = 0.0039719;
		start[12] = 3.3147e-05;
		start[13] = 0.0064412;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		start[3] = 0.0055486;
		start[4] = 0.0091932;
		start[5] = 0.0053405;
		start[6] = 0.0015554;
		start[7] = 0.0018394;
		start[8] = 0.0099495;
		start[9] = 0.0079177;
		start[10] = 0.0040877;
		start[11] = 0.0022468;
		start[12] = 0.0039005;
		start[13] = 0.0043088;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		start[3] = 0.00073197;
		start[4] = 0.0080772;
		start[5] = 0.00085386;
		start[6] = 0.0089547;
		start[7] = 0.0087945;
		start[8] = 0.0010938;
		start[9] = 0.00072592;
		start[10] = 0.0043766;
		start[11] = 0.0088903;
		start[12] = 0.0057912;
		start[13] = 0.0097206;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		start[3] = 0.005719;
		start[4] = 0.0031557;
		start[5] = 0.0080649;
		start[6] = 0.0047767;
		start[7] = 0.0062638;
		start[8] = 0.0021165;
		start[9] = 0.00042016;
		start[10] = 0.00091584;
		start[11] = 0.0060347;
		start[12] = 0.0063539;
		start[13] = 0.0041015;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		start[3] = 0.0069501;
		start[4] = 0.002153;
		start[5] = 0.0007684;
		start[6] = 0.0023564;
		start[7] = 0.0015914;
		start[8] = 0.0071712;
		start[9] = 0.0099885;
		start[10] = 0.0039358;
		start[11] = 0.00020138;
		start[12] = 0.0062945;
		start[13] = 0.0017706;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		start[3] = 0.0058884;
		start[4] = 0.0053025;
		start[5] = 0.003434;
		start[6] = 0.0071542;
		start[7] = 0.0035059;
		start[8] = 0.0054317;
		start[9] = 0.004687;
		start[10] = 0.00013046;
		start[11] = 0.0013113;
		start[12] = 0.0069698;
		start[13] = 0.0039959;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		start[3] = 0.0022083;
		start[4] = 0.0011192;
		start[5] = 0.0043168;
		start[6] = 0.0029896;
		start[7] = 0.0063924;
		start[8] = 0.00082598;
		start[9] = 0.00082275;
		start[10] = 0.0080251;
		start[11] = 0.0074036;
		start[12] = 0.0038608;
		start[13] = 0.0098605;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		start[3] = 0.00605;
		start[4] = 0.0036181;
		start[5] = 0.0076103;
		start[6] = 0.0071921;
		start[7] = 0.0051811;
		start[8] = 0.0010912;
		start[9] = 0.001085;
		start[10] = 0.0020277;
		start[11] = 0.0070492;
		start[12] = 0.00688;
		start[13] = 0.008477;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		start[3] = 0.0061755;
		start[4] = 0.0078829;
		start[5] = 0.0062733;
		start[6] = 0.0011571;
		start[7] = 0.0023046;
		start[8] = 0.00314;
		start[9] = 0.0031314;
		start[10] = 0.0092054;
		start[11] = 0.00058238;
		start[12] = 0.0069943;
		start[13] = 0.005013;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		start[3] = 0.0051515;
		start[4] = 0.0045085;
		start[5] = 0.0083429;
		start[6] = 0.0047304;
		start[7] = 0.0072882;
		start[8] = 0.0040276;
		start[9] = 0.0011603;
		start[10] = 0.0039025;
		start[11] = 0.0051908;
		start[12] = 0.0070467;
		start[13] = 0.00043829;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		start[3] = 0.0022801;
		start[4] = 0.0082973;
		start[5] = 0.0047303;
		start[6] = 0.0037639;
		start[7] = 0.0042563;
		start[8] = 0.0056148;
		start[9] = 0.0047387;
		start[10] = 0.0019684;
		start[11] = 0.0084261;
		start[12] = 0.0057139;
		start[13] = 0.0037477;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		start[3] = 0.00095739;
		start[4] = 0.0037809;
		start[5] = 0.00076051;
		start[6] = 0.001927;
		start[7] = 0.0094694;
		start[8] = 0.0080771;
		start[9] = 0.0012446;
		start[10] = 0.0096617;
		start[11] = 0.00603;
		start[12] = 0.006766;
		start[13] = 0.0024271;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		start[3] = 0.0093484;
		start[4] = 0.0053314;
		start[5] = 8.7728e-05;
		start[6] = 0.0097046;
		start[7] = 0.0088016;
		start[8] = 0.0064733;
		start[9] = 0.00089231;
		start[10] = 0.0064193;
		start[11] = 0.00073186;
		start[12] = 0.002997;
		start[13] = 0.0062529;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		start[3] = 0.00028786;
		start[4] = 0.0015863;
		start[5] = 0.0072857;
		start[6] = 0.0053227;
		start[7] = 0.0019669;
		start[8] = 0.0091828;
		start[9] = 0.0021527;
		start[10] = 0.0035289;
		start[11] = 0.0015824;
		start[12] = 0.0037007;
		start[13] = 0.0073709;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		start[3] = 0.00029998;
		start[4] = 0.0056087;
		start[5] = 0.0021821;
		start[6] = 0.0091817;
		start[7] = 0.0051806;
		start[8] = 0.001568;
		start[9] = 0.0083033;
		start[10] = 0.0075146;
		start[11] = 0.0034368;
		start[12] = 0.0062139;
		start[13] = 0.0061135;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		start[3] = 0.0060899;
		start[4] = 0.0059603;
		start[5] = 0.00029332;
		start[6] = 0.0038725;
		start[7] = 0.0094387;
		start[8] = 0.0065975;
		start[9] = 0.0093133;
		start[10] = 0.0088592;
		start[11] = 0.0070838;
		start[12] = 0.0044249;
		start[13] = 0.0081363;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (14);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		start[3] = 0.0085526;
		start[4] = 0.0018599;
		start[5] = 0.0035321;
		start[6] = 0.0056611;
		start[7] = 0.0018481;
		start[8] = 0.0046559;
		start[9] = 0.0018327;
		start[10] = 0.0059142;
		start[11] = 0.00781;
		start[12] = 0.0065547;
		start[13] = 0.00060048;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}


  return 0;
}
