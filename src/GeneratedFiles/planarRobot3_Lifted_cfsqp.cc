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
    (17, 1, "CostFunction_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

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
    (17, 1, "LiftConstraint_1_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = cos(q_01) - 1.0*w_01_01;
}

template <typename T>
void
LiftConstraint_1<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

  switch (id)
    {
      
		case 0: 
			 grad[0] = -1.0*sin(q_01); 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = -1.0; 
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
    (17, 1, "LiftConstraint_2_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = sin(q_01) - 1.0*w_01_02;
}

template <typename T>
void
LiftConstraint_2<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

  switch (id)
    {
      
		case 0: 
			 grad[0] = cos(q_01); 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = -1.0; 
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
    (17, 1, "LiftConstraint_3_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = cos(q_03) - 1.0*w_01_03;
}

template <typename T>
void
LiftConstraint_3<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = -1.0*sin(q_03); 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
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
    (17, 1, "LiftConstraint_4_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = sin(q_03) - 1.0*w_01_04;
}

template <typename T>
void
LiftConstraint_4<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = cos(q_03); 
			 grad[3] = 0.0; 
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
    (17, 1, "LiftConstraint_5_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = cos(q_02) - 1.0*w_01_05;
}

template <typename T>
void
LiftConstraint_5<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

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
    (17, 1, "LiftConstraint_6_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = sin(q_02) - 1.0*w_01_06;
}

template <typename T>
void
LiftConstraint_6<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

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
			 grad[8] = -1.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
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
    (17, 1, "LiftConstraint_7_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = w_01_01*w_01_05 - 1.0*w_02_01;
}

template <typename T>
void
LiftConstraint_7<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_01_05; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = w_01_01; 
			 grad[8] = 0.0; 
			 grad[9] = -1.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
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
    (17, 1, "LiftConstraint_8_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = w_01_02*w_01_05 - 1.0*w_02_02;
}

template <typename T>
void
LiftConstraint_8<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = w_01_05; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = w_01_02; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = -1.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
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
    (17, 1, "LiftConstraint_9_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = w_01_01*w_01_06 - 1.0*w_02_03;
}

template <typename T>
void
LiftConstraint_9<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = w_01_06; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = w_01_01; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = -1.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
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
    (17, 1, "LiftConstraint_10_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = w_01_02*w_01_06 - 1.0*w_02_04;
}

template <typename T>
void
LiftConstraint_10<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = w_01_06; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = w_01_02; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = -1.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
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
    (17, 1, "LiftConstraint_11_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = w_01_03*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_01;
}

template <typename T>
void
LiftConstraint_11<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

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
			 grad[8] = 0.0; 
			 grad[9] = w_01_03; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = -1.0*w_01_03; 
			 grad[13] = -1.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
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
    (17, 1, "LiftConstraint_12_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = - 1.0*w_03_02 - 1.0*w_01_04*(w_02_02 + w_02_03);
}

template <typename T>
void
LiftConstraint_12<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

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
			 grad[10] = -1.0*w_01_04; 
			 grad[11] = -1.0*w_01_04; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = -1.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
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
    (17, 1, "LiftConstraint_13_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = w_01_03*(w_02_02 + w_02_03) - 1.0*w_03_03;
}

template <typename T>
void
LiftConstraint_13<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

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
			 grad[9] = 0.0; 
			 grad[10] = w_01_03; 
			 grad[11] = w_01_03; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = -1.0; 
			 grad[16] = 0.0; 
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
    (17, 1, "LiftConstraint_14_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = w_01_04*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_04;
}

template <typename T>
void
LiftConstraint_14<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

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
			 grad[9] = w_01_04; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = -1.0*w_01_04; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = -1.0; 
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
    (17, 1, "EEConstraint_1_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = w_01_01 - 1.0*EE_1_1 + w_02_01 - 1.0*w_02_04 + w_03_01 + w_03_02;
}

template <typename T>
void
EEConstraint_1<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

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
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = -1.0; 
			 grad[13] = 1.0; 
			 grad[14] = 1.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
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
    (17, 1, "EEConstraint_2_planarRobot3"),
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
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];
  
	result[0] = w_01_02 - 1.0*EE_1_2 + w_02_02 + w_02_03 + w_03_03 + w_03_04;
}

template <typename T>
void
EEConstraint_2<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& w_01_01 = x[3];
	const double& w_01_02 = x[4];
	const double& w_01_03 = x[5];
	const double& w_01_04 = x[6];
	const double& w_01_05 = x[7];
	const double& w_01_06 = x[8];
	const double& w_02_01 = x[9];
	const double& w_02_02 = x[10];
	const double& w_02_03 = x[11];
	const double& w_02_04 = x[12];
	const double& w_03_01 = x[13];
	const double& w_03_02 = x[14];
	const double& w_03_03 = x[15];
	const double& w_03_04 = x[16];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 1.0; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 1.0; 
			 grad[11] = 1.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 1.0; 
			 grad[16] = 1.0; 
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
	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_15 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_16 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);

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
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_15), bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_16), bounds, scales); 
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
		std::vector<double> start (17);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		start[3] = 0.0045295;
		start[4] = 0.0053322;
		start[5] = 0.0078338;
		start[6] = 0.0041234;
		start[7] = 0.009098;
		start[8] = 0.0012065;
		start[9] = 0.0087247;
		start[10] = 0.0065139;
		start[11] = 0.004315;
		start[12] = 0.0013716;
		start[13] = 0.00018417;
		start[14] = 0.0036574;
		start[15] = 0.00017309;
		start[16] = 0.0047144;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (17);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		start[3] = 0.0013166;
		start[4] = 0.0084485;
		start[5] = 0.0095741;
		start[6] = 0.0019613;
		start[7] = 0.0073774;
		start[8] = 0.00055588;
		start[9] = 0.0051449;
		start[10] = 0.0011878;
		start[11] = 0.0064265;
		start[12] = 0.0016266;
		start[13] = 0.00061672;
		start[14] = 0.005585;
		start[15] = 0.0014438;
		start[16] = 0.0036828;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (17);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		start[3] = 0.0020119;
		start[4] = 1.8024e-06;
		start[5] = 0.0081;
		start[6] = 0.0090307;
		start[7] = 0.0044065;
		start[8] = 0.0036583;
		start[9] = 0.0036481;
		start[10] = 0.0028787;
		start[11] = 0.0083509;
		start[12] = 0.0097588;
		start[13] = 0.0021429;
		start[14] = 0.0066601;
		start[15] = 0.0044075;
		start[16] = 0.0041279;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (17);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		start[3] = 0.0084655;
		start[4] = 0.0050236;
		start[5] = 0.0022902;
		start[6] = 0.0071204;
		start[7] = 0.0043218;
		start[8] = 0.0041595;
		start[9] = 0.0093127;
		start[10] = 0.0014712;
		start[11] = 0.00019838;
		start[12] = 0.0042402;
		start[13] = 0.009468;
		start[14] = 0.0073077;
		start[15] = 0.0031191;
		start[16] = 0.0015508;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (17);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		start[3] = 0.0098911;
		start[4] = 0.008529;
		start[5] = 0.006736;
		start[6] = 0.0026952;
		start[7] = 0.0090255;
		start[8] = 0.0026571;
		start[9] = 0.0085365;
		start[10] = 0.0041913;
		start[11] = 0.00085687;
		start[12] = 0.0093546;
		start[13] = 0.0082357;
		start[14] = 0.0093245;
		start[15] = 0.0063703;
		start[16] = 0.0098652;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}


  return 0;
}
