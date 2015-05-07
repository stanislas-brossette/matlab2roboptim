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
    (26, 1, "CostFunction_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
    (26, 1, "LiftConstraint_1_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
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
	const double& q_04 = x[3];
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

  switch (id)
    {
      
		case 0: 
			 grad[0] = -1.0*sin(q_01); 
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
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
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
    (26, 1, "LiftConstraint_2_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = cos(q_04) - 1.0*w_01_02;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = -1.0*sin(q_04); 
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
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
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
    (26, 1, "LiftConstraint_3_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = sin(q_04) - 1.0*w_01_03;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = cos(q_04); 
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
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
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
    (26, 1, "LiftConstraint_4_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = sin(q_01) - 1.0*w_01_04;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
    (26, 1, "LiftConstraint_5_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = cos(q_03) - 1.0*w_01_05;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
    (26, 1, "LiftConstraint_6_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = sin(q_03) - 1.0*w_01_06;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
    (26, 1, "LiftConstraint_7_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = cos(q_02) - 1.0*w_01_07;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
    (26, 1, "LiftConstraint_8_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = sin(q_02) - 1.0*w_01_08;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
    (26, 1, "LiftConstraint_9_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_01*w_01_07 - 1.0*w_02_01;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = w_01_07; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = w_01_01; 
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
    (26, 1, "LiftConstraint_10_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_04*w_01_07 - 1.0*w_02_02;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
			 grad[7] = w_01_07; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = w_01_04; 
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
    (26, 1, "LiftConstraint_11_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_01*w_01_08 - 1.0*w_02_03;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = w_01_08; 
			 grad[5] = 0.0; 
			 grad[6] = 0.0; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = w_01_01; 
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
    (26, 1, "LiftConstraint_12_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_04*w_01_08 - 1.0*w_02_04;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
			 grad[7] = w_01_08; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = w_01_04; 
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
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
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
    (26, 1, "LiftConstraint_13_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_05*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_01;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
			 grad[8] = w_02_01 - 1.0*w_02_04; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = w_01_05; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = -1.0*w_01_05; 
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
    (26, 1, "LiftConstraint_14_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = - 1.0*w_03_02 - 1.0*w_01_06*(w_02_02 + w_02_03);
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
			 grad[9] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = -1.0*w_01_06; 
			 grad[14] = -1.0*w_01_06; 
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
    (26, 1, "LiftConstraint_15_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_05*(w_02_02 + w_02_03) - 1.0*w_03_03;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
			 grad[8] = w_02_02 + w_02_03; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_01_05; 
			 grad[14] = w_01_05; 
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
    (26, 1, "LiftConstraint_16_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_06*(w_02_01 - 1.0*w_02_04) - 1.0*w_03_04;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
			 grad[9] = w_02_01 - 1.0*w_02_04; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = w_01_06; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = -1.0*w_01_06; 
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
    (26, 1, "LiftConstraint_17_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = - 1.0*w_03_05 - 1.0*w_01_05*(w_02_02 + w_02_03);
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
			 grad[8] = - 1.0*w_02_02 - 1.0*w_02_03; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = -1.0*w_01_05; 
			 grad[14] = -1.0*w_01_05; 
			 grad[15] = 0.0; 
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
    (26, 1, "LiftConstraint_18_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_06*(w_02_02 + w_02_03) - 1.0*w_03_06;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
			 grad[9] = w_02_02 + w_02_03; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = w_01_06; 
			 grad[14] = w_01_06; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = -1.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
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
    (26, 1, "LiftConstraint_19_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_02*(w_03_01 + w_03_02) - 1.0*w_04_01;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_03_01 + w_03_02; 
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
			 grad[16] = w_01_02; 
			 grad[17] = w_01_02; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = -1.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
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
    (26, 1, "LiftConstraint_20_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = - 1.0*w_04_02 - 1.0*w_01_03*(w_03_04 - 1.0*w_03_05);
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = w_03_05 - 1.0*w_03_04; 
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
			 grad[19] = -1.0*w_01_03; 
			 grad[20] = w_01_03; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = -1.0; 
			 grad[24] = 0.0; 
			 grad[25] = 0.0; 
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
    (26, 1, "LiftConstraint_21_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_02*(w_03_03 + w_03_04) - 1.0*w_04_03;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = w_03_03 + w_03_04; 
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
			 grad[18] = w_01_02; 
			 grad[19] = w_01_02; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = -1.0; 
			 grad[25] = 0.0; 
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
    (26, 1, "LiftConstraint_22_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_03*(w_03_01 - 1.0*w_03_06) - 1.0*w_04_04;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0.0; 
			 grad[1] = 0.0; 
			 grad[2] = 0.0; 
			 grad[3] = 0.0; 
			 grad[4] = 0.0; 
			 grad[5] = 0.0; 
			 grad[6] = w_03_01 - 1.0*w_03_06; 
			 grad[7] = 0.0; 
			 grad[8] = 0.0; 
			 grad[9] = 0.0; 
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 0.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = 0.0; 
			 grad[16] = w_01_03; 
			 grad[17] = 0.0; 
			 grad[18] = 0.0; 
			 grad[19] = 0.0; 
			 grad[20] = 0.0; 
			 grad[21] = -1.0*w_01_03; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 0.0; 
			 grad[25] = -1.0; 
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
    (26, 1, "EEConstraint_1_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_01 - 1.0*EE_1_1 + w_02_01 - 1.0*w_02_04 + w_03_01 + w_03_02 + w_04_01 + w_04_02;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
			 grad[10] = 0.0; 
			 grad[11] = 0.0; 
			 grad[12] = 1.0; 
			 grad[13] = 0.0; 
			 grad[14] = 0.0; 
			 grad[15] = -1.0; 
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
    (26, 1, "EEConstraint_2_planarRobot4"),
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];
  
	result[0] = w_01_04 - 1.0*EE_1_2 + w_02_02 + w_02_03 + w_03_03 + w_03_04 + w_04_03 + w_04_04;
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
	const double& w_01_01 = x[4];
	const double& w_01_02 = x[5];
	const double& w_01_03 = x[6];
	const double& w_01_04 = x[7];
	const double& w_01_05 = x[8];
	const double& w_01_06 = x[9];
	const double& w_01_07 = x[10];
	const double& w_01_08 = x[11];
	const double& w_02_01 = x[12];
	const double& w_02_02 = x[13];
	const double& w_02_03 = x[14];
	const double& w_02_04 = x[15];
	const double& w_03_01 = x[16];
	const double& w_03_02 = x[17];
	const double& w_03_03 = x[18];
	const double& w_03_04 = x[19];
	const double& w_03_05 = x[20];
	const double& w_03_06 = x[21];
	const double& w_04_01 = x[22];
	const double& w_04_02 = x[23];
	const double& w_04_03 = x[24];
	const double& w_04_04 = x[25];

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
			 grad[13] = 1.0; 
			 grad[14] = 1.0; 
			 grad[15] = 0.0; 
			 grad[16] = 0.0; 
			 grad[17] = 0.0; 
			 grad[18] = 1.0; 
			 grad[19] = 1.0; 
			 grad[20] = 0.0; 
			 grad[21] = 0.0; 
			 grad[22] = 0.0; 
			 grad[23] = 0.0; 
			 grad[24] = 1.0; 
			 grad[25] = 1.0; 
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
	boost::shared_ptr<EEConstraint_1<roboptim::EigenMatrixDense> > cstrFunc_23 = boost::make_shared<EEConstraint_1<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);
	boost::shared_ptr<EEConstraint_2<roboptim::EigenMatrixDense> > cstrFunc_24 = boost::make_shared<EEConstraint_2<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);

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
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_23), bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint (boost::static_pointer_cast<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > (cstrFunc_24), bounds, scales); 
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
		std::vector<double> start (26);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0033588;
		start[5] = 0.0073085;
		start[6] = 0.0068116;
		start[7] = 0.0032712;
		start[8] = 0.0036041;
		start[9] = 0.0032993;
		start[10] = 0.0046634;
		start[11] = 0.0087107;
		start[12] = 0.0095363;
		start[13] = 0.0056422;
		start[14] = 0.0094467;
		start[15] = 0.0083264;
		start[16] = 0.0027094;
		start[17] = 0.0033744;
		start[18] = 0.0016748;
		start[19] = 0.0096546;
		start[20] = 0.0041126;
		start[21] = 0.0086594;
		start[22] = 0.0092066;
		start[23] = 0.0078724;
		start[24] = 0.0034499;
		start[25] = 0.00069306;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0076343;
		start[5] = 0.0059497;
		start[6] = 0.0056431;
		start[7] = 0.0068415;
		start[8] = 0.0062601;
		start[9] = 0.0074134;
		start[10] = 0.0072572;
		start[11] = 0.003364;
		start[12] = 0.0025691;
		start[13] = 0.002329;
		start[14] = 0.0093241;
		start[15] = 0.0044632;
		start[16] = 0.007586;
		start[17] = 0.0041139;
		start[18] = 0.0070826;
		start[19] = 0.0066719;
		start[20] = 0.0091529;
		start[21] = 0.0073778;
		start[22] = 0.0031867;
		start[23] = 0.0088813;
		start[24] = 0.0029009;
		start[25] = 0.0091882;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0098708;
		start[5] = 0.0046406;
		start[6] = 0.0085157;
		start[7] = 0.0033442;
		start[8] = 0.00083273;
		start[9] = 0.0099667;
		start[10] = 0.0027124;
		start[11] = 0.0061338;
		start[12] = 0.0092248;
		start[13] = 0.0026062;
		start[14] = 0.0035077;
		start[15] = 0.0076904;
		start[16] = 0.0052118;
		start[17] = 0.0035433;
		start[18] = 0.0035662;
		start[19] = 0.0077075;
		start[20] = 0.0010548;
		start[21] = 0.00079809;
		start[22] = 0.0095827;
		start[23] = 0.0090836;
		start[24] = 0.0037057;
		start[25] = 0.00097615;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.003677;
		start[5] = 0.0069038;
		start[6] = 0.0091611;
		start[7] = 0.0056168;
		start[8] = 0.0059066;
		start[9] = 0.0034953;
		start[10] = 0.00319;
		start[11] = 0.00015559;
		start[12] = 0.0041561;
		start[13] = 0.0092959;
		start[14] = 0.0033385;
		start[15] = 0.0030252;
		start[16] = 0.0013698;
		start[17] = 0.0061769;
		start[18] = 0.0054567;
		start[19] = 0.0092407;
		start[20] = 0.002115;
		start[21] = 0.0083203;
		start[22] = 0.0038396;
		start[23] = 0.0077143;
		start[24] = 0.0021645;
		start[25] = 0.0088362;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0044708;
		start[5] = 0.0089423;
		start[6] = 0.0035928;
		start[7] = 0.0069438;
		start[8] = 0.0058862;
		start[9] = 0.0081845;
		start[10] = 0.0016481;
		start[11] = 0.0016659;
		start[12] = 0.0056434;
		start[13] = 0.0057263;
		start[14] = 0.0087453;
		start[15] = 0.0059796;
		start[16] = 0.0050166;
		start[17] = 0.0030379;
		start[18] = 0.0036278;
		start[19] = 0.0082711;
		start[20] = 0.0052238;
		start[21] = 0.0056258;
		start[22] = 0.0074623;
		start[23] = 0.0065824;
		start[24] = 0.0099063;
		start[25] = 0.0068805;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0059504;
		start[5] = 0.0075717;
		start[6] = 0.0057318;
		start[7] = 0.007981;
		start[8] = 0.009233;
		start[9] = 0.0077124;
		start[10] = 0.0026944;
		start[11] = 0.0094757;
		start[12] = 0.008634;
		start[13] = 0.0058924;
		start[14] = 0.0015143;
		start[15] = 0.0074616;
		start[16] = 0.0058608;
		start[17] = 0.0075616;
		start[18] = 0.0006309;
		start[19] = 0.0041345;
		start[20] = 0.0088994;
		start[21] = 0.0087614;
		start[22] = 0.0093989;
		start[23] = 0.0033554;
		start[24] = 0.00093522;
		start[25] = 0.0049627;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.00094174;
		start[5] = 0.0086971;
		start[6] = 0.0010704;
		start[7] = 0.0063983;
		start[8] = 0.0033662;
		start[9] = 0.0031574;
		start[10] = 0.0085357;
		start[11] = 0.0056905;
		start[12] = 0.0048066;
		start[13] = 0.006363;
		start[14] = 0.0025508;
		start[15] = 0.0010422;
		start[16] = 0.0036038;
		start[17] = 0.0051841;
		start[18] = 0.0015302;
		start[19] = 0.0012477;
		start[20] = 0.0086316;
		start[21] = 0.0026717;
		start[22] = 0.0032981;
		start[23] = 0.0028215;
		start[24] = 0.0071103;
		start[25] = 0.00024598;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 5.4103e-05;
		start[5] = 0.00082124;
		start[6] = 0.0029141;
		start[7] = 0.0085983;
		start[8] = 0.00089129;
		start[9] = 0.0091005;
		start[10] = 0.0014441;
		start[11] = 0.0067471;
		start[12] = 0.002128;
		start[13] = 0.0032407;
		start[14] = 0.0050428;
		start[15] = 3.9145e-05;
		start[16] = 0.00045366;
		start[17] = 0.0088287;
		start[18] = 0.00069666;
		start[19] = 0.0059461;
		start[20] = 0.0034066;
		start[21] = 0.0091406;
		start[22] = 0.0066392;
		start[23] = 0.0027343;
		start[24] = 0.003936;
		start[25] = 0.0015693;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0074875;
		start[5] = 0.00036115;
		start[6] = 0.0097978;
		start[7] = 0.0050557;
		start[8] = 0.0066527;
		start[9] = 0.0054044;
		start[10] = 0.0043914;
		start[11] = 0.0039859;
		start[12] = 0.0022751;
		start[13] = 0.0032466;
		start[14] = 0.0064379;
		start[15] = 0.0072531;
		start[16] = 0.00026173;
		start[17] = 0.0035296;
		start[18] = 0.0037138;
		start[19] = 0.0011828;
		start[20] = 0.0017267;
		start[21] = 0.0043949;
		start[22] = 0.0021454;
		start[23] = 0.0036069;
		start[24] = 0.0068653;
		start[25] = 0.0077107;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0033965;
		start[5] = 0.00018103;
		start[6] = 0.001601;
		start[7] = 0.0028887;
		start[8] = 0.0041927;
		start[9] = 0.0030926;
		start[10] = 0.0045606;
		start[11] = 0.0033392;
		start[12] = 0.0014335;
		start[13] = 0.0027593;
		start[14] = 0.0052098;
		start[15] = 0.0080417;
		start[16] = 0.0098855;
		start[17] = 0.0067915;
		start[18] = 0.0019718;
		start[19] = 0.0013813;
		start[20] = 0.0057221;
		start[21] = 0.0010347;
		start[22] = 0.0082982;
		start[23] = 0.0081857;
		start[24] = 0.0077671;
		start[25] = 0.0076624;
		std::vector<double> endEff (2);
		endEff[0] = -1.6579;
		endEff[1] = 3.2843;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0075445;
		start[5] = 0.002528;
		start[6] = 0.0081189;
		start[7] = 0.0042631;
		start[8] = 0.0086925;
		start[9] = 0.0014462;
		start[10] = 0.0095138;
		start[11] = 0.001529;
		start[12] = 0.00039922;
		start[13] = 0.0038715;
		start[14] = 0.0028897;
		start[15] = 0.003399;
		start[16] = 0.006587;
		start[17] = 0.0037183;
		start[18] = 0.0074972;
		start[19] = 0.0073854;
		start[20] = 0.003325;
		start[21] = 0.0076782;
		start[22] = 0.0026867;
		start[23] = 0.001937;
		start[24] = 0.00039584;
		start[25] = 0.0039316;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0074211;
		start[5] = 0.0072906;
		start[6] = 0.0048557;
		start[7] = 0.0013082;
		start[8] = 0.0068723;
		start[9] = 0.0026479;
		start[10] = 0.0036546;
		start[11] = 0.0096719;
		start[12] = 3.3806e-05;
		start[13] = 0.004214;
		start[14] = 0.0076507;
		start[15] = 0.0033237;
		start[16] = 9.5829e-05;
		start[17] = 0.0088353;
		start[18] = 0.0089745;
		start[19] = 0.0008067;
		start[20] = 0.007478;
		start[21] = 0.00018246;
		start[22] = 0.00080818;
		start[23] = 0.002772;
		start[24] = 0.0067728;
		start[25] = 0.0031998;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.00041466;
		start[5] = 0.0066844;
		start[6] = 0.0057306;
		start[7] = 0.0017833;
		start[8] = 0.0052848;
		start[9] = 0.0094957;
		start[10] = 0.0095138;
		start[11] = 0.00095097;
		start[12] = 0.0077121;
		start[13] = 0.0042663;
		start[14] = 0.0038659;
		start[15] = 0.0091919;
		start[16] = 0.0053066;
		start[17] = 0.0021856;
		start[18] = 0.0028572;
		start[19] = 0.0027992;
		start[20] = 0.00099698;
		start[21] = 0.0037664;
		start[22] = 0.0073059;
		start[23] = 0.0041325;
		start[24] = 0.0024048;
		start[25] = 0.0055547;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.00062097;
		start[5] = 0.0010655;
		start[6] = 0.0037547;
		start[7] = 0.009515;
		start[8] = 0.00087943;
		start[9] = 0.0090508;
		start[10] = 0.0050236;
		start[11] = 0.0025458;
		start[12] = 0.0029807;
		start[13] = 0.0060546;
		start[14] = 0.001439;
		start[15] = 0.0011988;
		start[16] = 0.0011626;
		start[17] = 0.00033438;
		start[18] = 0.00059749;
		start[19] = 0.0077444;
		start[20] = 0.0072531;
		start[21] = 0.0089855;
		start[22] = 0.0019323;
		start[23] = 0.0030992;
		start[24] = 0.002307;
		start[25] = 0.00012793;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0010861;
		start[5] = 0.0064737;
		start[6] = 0.0036009;
		start[7] = 0.0085831;
		start[8] = 0.0048097;
		start[9] = 0.0015642;
		start[10] = 0.003233;
		start[11] = 0.005926;
		start[12] = 0.0089883;
		start[13] = 0.0075101;
		start[14] = 0.0072014;
		start[15] = 0.001337;
		start[16] = 0.007758;
		start[17] = 0.0059543;
		start[18] = 0.0099024;
		start[19] = 0.00015619;
		start[20] = 0.0083321;
		start[21] = 0.0014139;
		start[22] = 0.0094572;
		start[23] = 0.0034282;
		start[24] = 0.0060464;
		start[25] = 0.0006737;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0079522;
		start[5] = 0.0027665;
		start[6] = 0.00033531;
		start[7] = 0.0081884;
		start[8] = 0.0077463;
		start[9] = 0.0013135;
		start[10] = 0.0012218;
		start[11] = 0.0033963;
		start[12] = 0.0038947;
		start[13] = 0.0026706;
		start[14] = 0.0003584;
		start[15] = 0.0038194;
		start[16] = 0.0056;
		start[17] = 0.0029197;
		start[18] = 0.00021607;
		start[19] = 0.0027807;
		start[20] = 0.0062508;
		start[21] = 0.0023829;
		start[22] = 0.001846;
		start[23] = 0.0077633;
		start[24] = 0.0084211;
		start[25] = 0.0065613;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0022251;
		start[5] = 0.0058116;
		start[6] = 0.0095826;
		start[7] = 0.0032373;
		start[8] = 0.001304;
		start[9] = 0.0074023;
		start[10] = 0.0092853;
		start[11] = 0.0036733;
		start[12] = 0.0012142;
		start[13] = 0.0015873;
		start[14] = 0.0086748;
		start[15] = 0.0048669;
		start[16] = 0.0020043;
		start[17] = 0.0025304;
		start[18] = 0.0070317;
		start[19] = 0.0076693;
		start[20] = 0.0048546;
		start[21] = 0.0092389;
		start[22] = 0.0062176;
		start[23] = 0.0022451;
		start[24] = 0.0082118;
		start[25] = 0.0016969;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0092088;
		start[5] = 0.0077575;
		start[6] = 0.009201;
		start[7] = 0.002252;
		start[8] = 0.0043726;
		start[9] = 0.0065453;
		start[10] = 0.0099835;
		start[11] = 0.00051289;
		start[12] = 0.0028715;
		start[13] = 0.0015976;
		start[14] = 0.0092049;
		start[15] = 0.0011105;
		start[16] = 0.00072526;
		start[17] = 0.0064466;
		start[18] = 0.0046524;
		start[19] = 0.004014;
		start[20] = 0.0081031;
		start[21] = 0.00061046;
		start[22] = 0.00066653;
		start[23] = 0.009501;
		start[24] = 0.0057724;
		start[25] = 0.0099223;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0018854;
		start[5] = 0.0095119;
		start[6] = 0.0064556;
		start[7] = 0.0041888;
		start[8] = 0.0011767;
		start[9] = 0.0027298;
		start[10] = 0.00055021;
		start[11] = 0.0002255;
		start[12] = 0.0061;
		start[13] = 0.0093526;
		start[14] = 0.0071958;
		start[15] = 0.0004772;
		start[16] = 0.0086173;
		start[17] = 0.00050298;
		start[18] = 0.00813;
		start[19] = 0.0030791;
		start[20] = 0.0087688;
		start[21] = 0.0052177;
		start[22] = 0.004785;
		start[23] = 0.0045181;
		start[24] = 0.0033957;
		start[25] = 0.003947;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0038964;
		start[5] = 0.0059222;
		start[6] = 0.00093311;
		start[7] = 0.0039272;
		start[8] = 0.0089562;
		start[9] = 0.0050057;
		start[10] = 0.009132;
		start[11] = 0.001604;
		start[12] = 0.0029325;
		start[13] = 0.0083646;
		start[14] = 0.0078641;
		start[15] = 0.0092255;
		start[16] = 0.0021184;
		start[17] = 0.0035617;
		start[18] = 0.0069222;
		start[19] = 0.0093117;
		start[20] = 0.0043268;
		start[21] = 0.0060987;
		start[22] = 0.0068778;
		start[23] = 0.0079483;
		start[24] = 0.0086656;
		start[25] = 0.0044504;
		std::vector<double> endEff (2);
		endEff[0] = -1.458;
		endEff[1] = -1.152;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0071938;
		start[5] = 0.0077544;
		start[6] = 0.0046859;
		start[7] = 0.0062115;
		start[8] = 0.0068731;
		start[9] = 0.0016594;
		start[10] = 0.0027004;
		start[11] = 0.0050418;
		start[12] = 0.00206;
		start[13] = 0.0045248;
		start[14] = 0.0080424;
		start[15] = 0.0067277;
		start[16] = 0.0026457;
		start[17] = 0.000175;
		start[18] = 0.0062446;
		start[19] = 0.0067299;
		start[20] = 0.0040091;
		start[21] = 0.0088597;
		start[22] = 0.0048235;
		start[23] = 0.0048058;
		start[24] = 0.0090905;
		start[25] = 0.003141;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0027839;
		start[5] = 0.0086118;
		start[6] = 0.0084682;
		start[7] = 0.0082245;
		start[8] = 0.0058787;
		start[9] = 0.0098424;
		start[10] = 0.0025059;
		start[11] = 0.0074963;
		start[12] = 0.0030843;
		start[13] = 0.00021156;
		start[14] = 0.0027432;
		start[15] = 0.0099145;
		start[16] = 0.003788;
		start[17] = 0.0099206;
		start[18] = 0.0026179;
		start[19] = 0.0065029;
		start[20] = 0.0071698;
		start[21] = 0.0071724;
		start[22] = 0.0073641;
		start[23] = 0.0011026;
		start[24] = 0.0013528;
		start[25] = 0.0058561;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0051441;
		start[5] = 0.0063514;
		start[6] = 0.0091369;
		start[7] = 0.003583;
		start[8] = 0.0042809;
		start[9] = 0.0079649;
		start[10] = 0.0029662;
		start[11] = 0.0059694;
		start[12] = 0.0083388;
		start[13] = 0.00087297;
		start[14] = 0.0058739;
		start[15] = 0.0079169;
		start[16] = 0.00053574;
		start[17] = 0.0030365;
		start[18] = 0.00054924;
		start[19] = 0.0026826;
		start[20] = 0.0030317;
		start[21] = 0.0047862;
		start[22] = 0.0073094;
		start[23] = 0.0039919;
		start[24] = 0.0052001;
		start[25] = 0.0012468;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0066011;
		start[5] = 0.004247;
		start[6] = 0.008576;
		start[7] = 0.0095862;
		start[8] = 0.0024448;
		start[9] = 0.0075709;
		start[10] = 0.0097079;
		start[11] = 0.0068744;
		start[12] = 0.0059819;
		start[13] = 0.0047319;
		start[14] = 0.0012515;
		start[15] = 0.0047339;
		start[16] = 0.0068473;
		start[17] = 0.0041885;
		start[18] = 0.0035082;
		start[19] = 0.0020421;
		start[20] = 0.0019535;
		start[21] = 0.0020793;
		start[22] = 0.0011877;
		start[23] = 0.0062884;
		start[24] = 0.0096305;
		start[25] = 0.0097397;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0088444;
		start[5] = 0.0074088;
		start[6] = 0.0075898;
		start[7] = 0.0084085;
		start[8] = 0.0026657;
		start[9] = 0.0032321;
		start[10] = 0.0096395;
		start[11] = 0.0033068;
		start[12] = 0.0084407;
		start[13] = 0.0084505;
		start[14] = 0.0044417;
		start[15] = 0.0093085;
		start[16] = 0.001601;
		start[17] = 9.6763e-05;
		start[18] = 0.0083705;
		start[19] = 0.0034348;
		start[20] = 0.001259;
		start[21] = 0.0055915;
		start[22] = 0.0021291;
		start[23] = 0.0065361;
		start[24] = 0.0041545;
		start[25] = 0.0041724;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.007114;
		start[5] = 0.009192;
		start[6] = 0.0064205;
		start[7] = 0.0033028;
		start[8] = 0.0071929;
		start[9] = 0.002165;
		start[10] = 0.0031801;
		start[11] = 0.0016455;
		start[12] = 0.0019241;
		start[13] = 0.0085604;
		start[14] = 0.0017387;
		start[15] = 0.0021393;
		start[16] = 0.0075308;
		start[17] = 0.0085033;
		start[18] = 0.0032248;
		start[19] = 0.0044051;
		start[20] = 0.0079652;
		start[21] = 0.0093339;
		start[22] = 0.0043946;
		start[23] = 0.0015948;
		start[24] = 0.0079946;
		start[25] = 0.0023006;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.00029974;
		start[5] = 0.007396;
		start[6] = 0.0032729;
		start[7] = 0.0009206;
		start[8] = 0.0023221;
		start[9] = 0.00057783;
		start[10] = 0.0079944;
		start[11] = 0.0044801;
		start[12] = 0.00066619;
		start[13] = 0.0097529;
		start[14] = 0.0030504;
		start[15] = 0.0020843;
		start[16] = 0.0061562;
		start[17] = 0.0079374;
		start[18] = 0.0067219;
		start[19] = 0.002152;
		start[20] = 0.0067803;
		start[21] = 0.0010419;
		start[22] = 0.0069424;
		start[23] = 0.0081085;
		start[24] = 0.0096573;
		start[25] = 0.0082915;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0088113;
		start[5] = 0.005704;
		start[6] = 0.0091928;
		start[7] = 0.00097397;
		start[8] = 0.00092042;
		start[9] = 0.0070216;
		start[10] = 0.0012772;
		start[11] = 0.0063361;
		start[12] = 0.0097815;
		start[13] = 0.0031706;
		start[14] = 0.0057031;
		start[15] = 0.0030287;
		start[16] = 0.0033677;
		start[17] = 0.0063053;
		start[18] = 0.0086503;
		start[19] = 0.0022216;
		start[20] = 0.0038053;
		start[21] = 0.002163;
		start[22] = 0.004423;
		start[23] = 0.0049409;
		start[24] = 0.0076365;
		start[25] = 0.0015344;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.008883;
		start[5] = 0.0096199;
		start[6] = 0.0094715;
		start[7] = 0.0039493;
		start[8] = 0.0078071;
		start[9] = 0.0083884;
		start[10] = 0.0076245;
		start[11] = 0.0038921;
		start[12] = 0.0034495;
		start[13] = 0.0097728;
		start[14] = 0.0080362;
		start[15] = 0.00070427;
		start[16] = 0.0066498;
		start[17] = 0.0079455;
		start[18] = 0.0074503;
		start[19] = 0.0050057;
		start[20] = 0.0076005;
		start[21] = 0.0056527;
		start[22] = 0.0077393;
		start[23] = 0.0062555;
		start[24] = 0.008043;
		start[25] = 0.0054553;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.002492;
		start[5] = 0.0089514;
		start[6] = 0.0085509;
		start[7] = 0.0058881;
		start[8] = 0.00052611;
		start[9] = 0.0049634;
		start[10] = 0.0075509;
		start[11] = 0.0089744;
		start[12] = 0.00084769;
		start[13] = 0.0050754;
		start[14] = 0.00016932;
		start[15] = 0.0019853;
		start[16] = 0.0037425;
		start[17] = 0.0072652;
		start[18] = 0.0063986;
		start[19] = 0.0098087;
		start[20] = 0.0038846;
		start[21] = 0.001231;
		start[22] = 0.0087101;
		start[23] = 0.0079221;
		start[24] = 0.0085344;
		start[25] = 0.0082738;
		std::vector<double> endEff (2);
		endEff[0] = 3.3635;
		endEff[1] = -1.8607;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0098382;
		start[5] = 0.0032434;
		start[6] = 0.0024649;
		start[7] = 0.0018936;
		start[8] = 0.00021433;
		start[9] = 0.0026612;
		start[10] = 0.0026597;
		start[11] = 0.0091626;
		start[12] = 0.0048405;
		start[13] = 0.0099857;
		start[14] = 0.0002766;
		start[15] = 0.0088869;
		start[16] = 0.004709;
		start[17] = 0.0015533;
		start[18] = 0.0065081;
		start[19] = 0.00351;
		start[20] = 0.0010113;
		start[21] = 0.0035604;
		start[22] = 0.0066296;
		start[23] = 0.0014876;
		start[24] = 0.0073018;
		start[25] = 0.0049648;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0046973;
		start[5] = 0.0057139;
		start[6] = 0.0064132;
		start[7] = 0.002665;
		start[8] = 0.0049536;
		start[9] = 0.0086129;
		start[10] = 0.0069309;
		start[11] = 0.0051672;
		start[12] = 0.0075923;
		start[13] = 0.0009839;
		start[14] = 0.0046587;
		start[15] = 0.0074286;
		start[16] = 0.0089866;
		start[17] = 0.0010094;
		start[18] = 0.0030285;
		start[19] = 0.0012542;
		start[20] = 0.0049969;
		start[21] = 0.0076162;
		start[22] = 0.0066656;
		start[23] = 0.0062136;
		start[24] = 0.0018957;
		start[25] = 0.0042368;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0095469;
		start[5] = 0.0013507;
		start[6] = 0.0055961;
		start[7] = 0.00047661;
		start[8] = 0.0019554;
		start[9] = 0.0050859;
		start[10] = 0.0042398;
		start[11] = 0.0092911;
		start[12] = 0.0060735;
		start[13] = 0.0038062;
		start[14] = 0.00058758;
		start[15] = 0.0054321;
		start[16] = 0.0052637;
		start[17] = 0.0089655;
		start[18] = 0.00735;
		start[19] = 0.005972;
		start[20] = 0.0085557;
		start[21] = 0.0010132;
		start[22] = 0.005562;
		start[23] = 0.0030265;
		start[24] = 0.009369;
		start[25] = 0.0041965;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.00025413;
		start[5] = 0.0023569;
		start[6] = 0.0074909;
		start[7] = 0.0089499;
		start[8] = 0.0073075;
		start[9] = 0.0028753;
		start[10] = 0.0079327;
		start[11] = 0.0022787;
		start[12] = 0.0054666;
		start[13] = 0.0032957;
		start[14] = 0.0067212;
		start[15] = 0.0069957;
		start[16] = 0.0057419;
		start[17] = 0.006991;
		start[18] = 0.0090227;
		start[19] = 0.0048981;
		start[20] = 0.002645;
		start[21] = 0.0068877;
		start[22] = 0.0083836;
		start[23] = 0.009738;
		start[24] = 0.0087401;
		start[25] = 0.0038857;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.00068619;
		start[5] = 0.0058866;
		start[6] = 0.0066598;
		start[7] = 0.0026879;
		start[8] = 0.0063326;
		start[9] = 0.0077544;
		start[10] = 0.0064591;
		start[11] = 0.0064649;
		start[12] = 0.0028974;
		start[13] = 0.0078848;
		start[14] = 0.0029072;
		start[15] = 0.0096038;
		start[16] = 0.0042103;
		start[17] = 0.0044693;
		start[18] = 0.0035527;
		start[19] = 0.0027627;
		start[20] = 0.009525;
		start[21] = 0.0031533;
		start[22] = 0.0025808;
		start[23] = 0.00048122;
		start[24] = 0.0029508;
		start[25] = 0.0011386;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0074502;
		start[5] = 0.0026367;
		start[6] = 0.008413;
		start[7] = 0.007598;
		start[8] = 0.002553;
		start[9] = 0.0069215;
		start[10] = 0.00095714;
		start[11] = 0.0071656;
		start[12] = 0.0064369;
		start[13] = 0.0088471;
		start[14] = 0.0036697;
		start[15] = 0.0016465;
		start[16] = 0.0017939;
		start[17] = 0.0043974;
		start[18] = 0.0033641;
		start[19] = 0.0032402;
		start[20] = 0.00097393;
		start[21] = 0.0051463;
		start[22] = 0.00044907;
		start[23] = 0.0051466;
		start[24] = 0.00037953;
		start[25] = 0.0038088;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0058412;
		start[5] = 0.0093578;
		start[6] = 0.00016946;
		start[7] = 0.0031275;
		start[8] = 0.0065896;
		start[9] = 0.0033117;
		start[10] = 0.0033991;
		start[11] = 0.0078573;
		start[12] = 0.0022795;
		start[13] = 0.00062832;
		start[14] = 0.009366;
		start[15] = 0.00058615;
		start[16] = 0.00059701;
		start[17] = 0.00522;
		start[18] = 0.009759;
		start[19] = 0.00047424;
		start[20] = 0.0045261;
		start[21] = 0.0079221;
		start[22] = 0.0083458;
		start[23] = 0.0018193;
		start[24] = 0.0041189;
		start[25] = 0.0090057;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0039389;
		start[5] = 0.0058782;
		start[6] = 0.0032621;
		start[7] = 0.0027549;
		start[8] = 0.003593;
		start[9] = 0.0063109;
		start[10] = 0.0044402;
		start[11] = 0.0066229;
		start[12] = 0.0048072;
		start[13] = 0.0038556;
		start[14] = 0.0037468;
		start[15] = 0.0095666;
		start[16] = 0.001625;
		start[17] = 0.0047801;
		start[18] = 0.0096619;
		start[19] = 0.0092754;
		start[20] = 0.00013356;
		start[21] = 0.00019886;
		start[22] = 0.004371;
		start[23] = 0.000526;
		start[24] = 0.0085649;
		start[25] = 0.0082254;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0048014;
		start[5] = 0.0065241;
		start[6] = 0.0025571;
		start[7] = 0.0067602;
		start[8] = 0.0026618;
		start[9] = 0.0085944;
		start[10] = 0.0043678;
		start[11] = 0.0089911;
		start[12] = 0.0091649;
		start[13] = 0.0089484;
		start[14] = 0.0051824;
		start[15] = 0.0062545;
		start[16] = 0.0010915;
		start[17] = 0.00014247;
		start[18] = 0.004934;
		start[19] = 0.0040515;
		start[20] = 0.0099698;
		start[21] = 0.0099995;
		start[22] = 0.0029683;
		start[23] = 0.0050868;
		start[24] = 0.0084488;
		start[25] = 0.0057703;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0051397;
		start[5] = 0.00026323;
		start[6] = 0.00013057;
		start[7] = 0.0067377;
		start[8] = 0.0085409;
		start[9] = 0.0085715;
		start[10] = 0.0075694;
		start[11] = 0.0023128;
		start[12] = 0.0052374;
		start[13] = 0.0051621;
		start[14] = 0.0062449;
		start[15] = 0.0050475;
		start[16] = 0.0031446;
		start[17] = 0.00030914;
		start[18] = 0.00046257;
		start[19] = 0.0049717;
		start[20] = 0.00053717;
		start[21] = 0.0086872;
		start[22] = 0.0033438;
		start[23] = 0.0042339;
		start[24] = 2.8835e-05;
		start[25] = 0.0040978;
		std::vector<double> endEff (2);
		endEff[0] = -0.17878;
		endEff[1] = -2.5148;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0069392;
		start[5] = 0.0042761;
		start[6] = 0.0022234;
		start[7] = 0.0087562;
		start[8] = 0.0085277;
		start[9] = 0.0032129;
		start[10] = 0.0038193;
		start[11] = 0.0023624;
		start[12] = 0.0093563;
		start[13] = 0.0016531;
		start[14] = 0.0062762;
		start[15] = 0.0014012;
		start[16] = 0.0015649;
		start[17] = 0.0086229;
		start[18] = 0.001975;
		start[19] = 0.0096078;
		start[20] = 0.0036423;
		start[21] = 4.5435e-05;
		start[22] = 0.0085568;
		start[23] = 0.0084782;
		start[24] = 0.0054071;
		start[25] = 0.0047578;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0008498;
		start[5] = 0.0037965;
		start[6] = 0.0052353;
		start[7] = 0.0097085;
		start[8] = 0.0043866;
		start[9] = 0.0081195;
		start[10] = 0.0089609;
		start[11] = 0.0083003;
		start[12] = 0.0049004;
		start[13] = 0.0076742;
		start[14] = 0.00013291;
		start[15] = 0.0034845;
		start[16] = 0.0021423;
		start[17] = 0.0062958;
		start[18] = 0.003312;
		start[19] = 0.0019997;
		start[20] = 0.0086277;
		start[21] = 0.0071769;
		start[22] = 0.0033041;
		start[23] = 0.0094601;
		start[24] = 0.0059004;
		start[25] = 0.0043236;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.002049;
		start[5] = 0.0018898;
		start[6] = 0.0001079;
		start[7] = 0.0084693;
		start[8] = 0.0028046;
		start[9] = 0.0040184;
		start[10] = 0.0071685;
		start[11] = 0.0085084;
		start[12] = 0.0044536;
		start[13] = 0.007133;
		start[14] = 0.0088834;
		start[15] = 0.0063135;
		start[16] = 0.0098671;
		start[17] = 0.0017387;
		start[18] = 0.0062809;
		start[19] = 0.0068462;
		start[20] = 0.0016754;
		start[21] = 0.0014908;
		start[22] = 0.0014244;
		start[23] = 0.00092643;
		start[24] = 0.0091732;
		start[25] = 0.0085329;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0035713;
		start[5] = 0.00092066;
		start[6] = 0.0033221;
		start[7] = 0.00067462;
		start[8] = 0.00088046;
		start[9] = 0.0072305;
		start[10] = 0.0041071;
		start[11] = 0.0052011;
		start[12] = 0.0027022;
		start[13] = 0.0072381;
		start[14] = 0.0086884;
		start[15] = 0.0017667;
		start[16] = 0.0039583;
		start[17] = 0.0040243;
		start[18] = 0.0052259;
		start[19] = 0.0017485;
		start[20] = 0.0041572;
		start[21] = 4.5838e-05;
		start[22] = 0.0091544;
		start[23] = 0.0043166;
		start[24] = 0.0073638;
		start[25] = 0.001003;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.00083951;
		start[5] = 0.0078254;
		start[6] = 0.0092324;
		start[7] = 0.0022932;
		start[8] = 0.0045249;
		start[9] = 0.002072;
		start[10] = 0.0010689;
		start[11] = 0.00085949;
		start[12] = 0.0099291;
		start[13] = 0.0044082;
		start[14] = 0.0025406;
		start[15] = 0.0072943;
		start[16] = 0.0038614;
		start[17] = 0.0058807;
		start[18] = 0.0077756;
		start[19] = 0.0071136;
		start[20] = 0.0094709;
		start[21] = 0.0076005;
		start[22] = 0.0017469;
		start[23] = 0.0050068;
		start[24] = 0.002978;
		start[25] = 0.0086252;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0066326;
		start[5] = 0.0068682;
		start[6] = 0.0011615;
		start[7] = 0.0061;
		start[8] = 0.0036246;
		start[9] = 0.0040525;
		start[10] = 0.0019191;
		start[11] = 0.0083054;
		start[12] = 0.0094531;
		start[13] = 0.0090246;
		start[14] = 0.006006;
		start[15] = 0.0042408;
		start[16] = 8.3442e-05;
		start[17] = 0.00049245;
		start[18] = 0.0051998;
		start[19] = 0.0048235;
		start[20] = 0.00012774;
		start[21] = 0.0046038;
		start[22] = 0.00026195;
		start[23] = 0.0076894;
		start[24] = 0.0043195;
		start[25] = 0.0096156;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.005671;
		start[5] = 0.0087315;
		start[6] = 0.0057301;
		start[7] = 0.0047647;
		start[8] = 0.0059315;
		start[9] = 0.0093757;
		start[10] = 0.00012636;
		start[11] = 0.0028111;
		start[12] = 0.00075685;
		start[13] = 0.0066927;
		start[14] = 0.0062803;
		start[15] = 0.00093359;
		start[16] = 0.0066892;
		start[17] = 0.0010838;
		start[18] = 0.0063322;
		start[19] = 0.0017178;
		start[20] = 0.0030839;
		start[21] = 0.0033997;
		start[22] = 0.0084867;
		start[23] = 0.0022095;
		start[24] = 0.0081507;
		start[25] = 0.0057043;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0058773;
		start[5] = 0.0066532;
		start[6] = 0.0016757;
		start[7] = 0.0045634;
		start[8] = 0.0035986;
		start[9] = 0.001175;
		start[10] = 0.0026562;
		start[11] = 0.0074685;
		start[12] = 0.0060912;
		start[13] = 0.0033465;
		start[14] = 0.0035032;
		start[15] = 0.004752;
		start[16] = 0.0036452;
		start[17] = 0.007292;
		start[18] = 0.0049117;
		start[19] = 0.0005809;
		start[20] = 0.0098342;
		start[21] = 0.00072581;
		start[22] = 0.0039437;
		start[23] = 0.0074095;
		start[24] = 0.0069484;
		start[25] = 0.00011819;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.005445;
		start[5] = 0.0039925;
		start[6] = 0.0030912;
		start[7] = 0.0048562;
		start[8] = 0.0099853;
		start[9] = 0.0095556;
		start[10] = 0.00816;
		start[11] = 0.0096682;
		start[12] = 0.0033943;
		start[13] = 0.00883;
		start[14] = 0.0037319;
		start[15] = 0.0018473;
		start[16] = 0.0025346;
		start[17] = 0.0058072;
		start[18] = 0.0046905;
		start[19] = 0.0073102;
		start[20] = 0.0082382;
		start[21] = 0.0079241;
		start[22] = 0.0035492;
		start[23] = 0.0014558;
		start[24] = 0.0015396;
		start[25] = 0.0036137;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0049392;
		start[5] = 0.0049528;
		start[6] = 0.0042873;
		start[7] = 0.00086827;
		start[8] = 0.0076717;
		start[9] = 0.005064;
		start[10] = 0.0092929;
		start[11] = 0.0055282;
		start[12] = 0.0022414;
		start[13] = 0.0068714;
		start[14] = 0.0028167;
		start[15] = 0.0086348;
		start[16] = 0.0027789;
		start[17] = 0.006421;
		start[18] = 0.0051987;
		start[19] = 0.0071577;
		start[20] = 0.0048924;
		start[21] = 0.0022593;
		start[22] = 0.0041188;
		start[23] = 0.0027859;
		start[24] = 0.0033836;
		start[25] = 0.0069208;
		std::vector<double> endEff (2);
		endEff[0] = -3.4202;
		endEff[1] = -0.4314;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0072504;
		start[5] = 0.0059054;
		start[6] = 0.00065517;
		start[7] = 0.0023804;
		start[8] = 0.0014002;
		start[9] = 0.0080363;
		start[10] = 0.0080432;
		start[11] = 0.0067138;
		start[12] = 0.0041817;
		start[13] = 0.0036012;
		start[14] = 0.0011362;
		start[15] = 0.00089069;
		start[16] = 0.001889;
		start[17] = 0.0091845;
		start[18] = 0.0077513;
		start[19] = 0.0084887;
		start[20] = 0.0099647;
		start[21] = 0.0074852;
		start[22] = 0.00051783;
		start[23] = 0.00020481;
		start[24] = 0.0080937;
		start[25] = 0.0068546;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0092352;
		start[5] = 0.0084138;
		start[6] = 0.0057957;
		start[7] = 0.0076823;
		start[8] = 0.0085821;
		start[9] = 0.0039453;
		start[10] = 0.0011983;
		start[11] = 0.0044427;
		start[12] = 0.0021958;
		start[13] = 0.001199;
		start[14] = 0.00067575;
		start[15] = 0.0049871;
		start[16] = 0.0037964;
		start[17] = 0.0016567;
		start[18] = 0.0080223;
		start[19] = 1.0839e-05;
		start[20] = 0.005114;
		start[21] = 0.0028276;
		start[22] = 0.0030575;
		start[23] = 0.0065139;
		start[24] = 0.0045693;
		start[25] = 0.0062338;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0038824;
		start[5] = 0.0027827;
		start[6] = 0.0014668;
		start[7] = 0.0051689;
		start[8] = 0.0051884;
		start[9] = 0.0055042;
		start[10] = 0.0046848;
		start[11] = 0.006412;
		start[12] = 0.0038307;
		start[13] = 0.0020557;
		start[14] = 0.00038065;
		start[15] = 0.0089896;
		start[16] = 0.00050882;
		start[17] = 0.0029202;
		start[18] = 0.00062427;
		start[19] = 0.00094594;
		start[20] = 0.009583;
		start[21] = 0.0053011;
		start[22] = 0.0085535;
		start[23] = 0.0061731;
		start[24] = 0.0076558;
		start[25] = 0.00091338;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.004073;
		start[5] = 0.0097595;
		start[6] = 0.00028998;
		start[7] = 0.0012799;
		start[8] = 0.00031687;
		start[9] = 0.002248;
		start[10] = 0.0013651;
		start[11] = 0.0076082;
		start[12] = 0.00047129;
		start[13] = 0.0021023;
		start[14] = 0.0097491;
		start[15] = 0.0026885;
		start[16] = 0.00386;
		start[17] = 0.00060567;
		start[18] = 0.0020589;
		start[19] = 0.0084384;
		start[20] = 0.0061721;
		start[21] = 0.0073326;
		start[22] = 0.0014053;
		start[23] = 0.009859;
		start[24] = 0.0067176;
		start[25] = 0.0089216;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0024449;
		start[5] = 0.0061628;
		start[6] = 0.004249;
		start[7] = 0.0038616;
		start[8] = 0.0089895;
		start[9] = 0.0084447;
		start[10] = 0.0054526;
		start[11] = 0.0079906;
		start[12] = 0.00095856;
		start[13] = 0.0034348;
		start[14] = 0.0054042;
		start[15] = 0.0044457;
		start[16] = 0.0009843;
		start[17] = 0.0061227;
		start[18] = 0.0064434;
		start[19] = 0.0058035;
		start[20] = 0.0043436;
		start[21] = 0.0067801;
		start[22] = 0.001908;
		start[23] = 0.0090633;
		start[24] = 0.0098682;
		start[25] = 0.0094371;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0032037;
		start[5] = 0.00078127;
		start[6] = 0.00013044;
		start[7] = 0.0071532;
		start[8] = 0.0090004;
		start[9] = 0.0063062;
		start[10] = 0.0027089;
		start[11] = 0.0019063;
		start[12] = 0.0098272;
		start[13] = 0.0041028;
		start[14] = 0.0062116;
		start[15] = 0.0092549;
		start[16] = 0.0063196;
		start[17] = 0.0015363;
		start[18] = 0.0023176;
		start[19] = 0.0093865;
		start[20] = 0.0049248;
		start[21] = 0.0048541;
		start[22] = 0.0053412;
		start[23] = 0.0089809;
		start[24] = 0.0048309;
		start[25] = 0.00073014;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0076502;
		start[5] = 0.0048302;
		start[6] = 0.0016872;
		start[7] = 0.0029376;
		start[8] = 0.0027459;
		start[9] = 0.0029228;
		start[10] = 0.0094123;
		start[11] = 0.0040875;
		start[12] = 0.0041746;
		start[13] = 0.0046587;
		start[14] = 0.0031284;
		start[15] = 0.0086137;
		start[16] = 0.0055611;
		start[17] = 0.0055298;
		start[18] = 0.00080216;
		start[19] = 0.003529;
		start[20] = 0.0033384;
		start[21] = 0.008409;
		start[22] = 0.0030361;
		start[23] = 0.0033346;
		start[24] = 0.0079879;
		start[25] = 0.0060051;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.00098444;
		start[5] = 0.0074544;
		start[6] = 0.0034615;
		start[7] = 0.0027031;
		start[8] = 0.0023854;
		start[9] = 0.0084644;
		start[10] = 0.0056564;
		start[11] = 0.0023337;
		start[12] = 0.0027278;
		start[13] = 0.0089106;
		start[14] = 0.0051259;
		start[15] = 0.0080678;
		start[16] = 0.009038;
		start[17] = 0.0070002;
		start[18] = 0.004918;
		start[19] = 0.0041125;
		start[20] = 0.0049301;
		start[21] = 0.0056687;
		start[22] = 0.0082344;
		start[23] = 0.0051328;
		start[24] = 0.0020479;
		start[25] = 0.0048803;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.00035679;
		start[5] = 0.00030855;
		start[6] = 0.0059014;
		start[7] = 0.0020751;
		start[8] = 0.0060766;
		start[9] = 0.0010153;
		start[10] = 0.0023823;
		start[11] = 0.0083693;
		start[12] = 0.0091427;
		start[13] = 0.0011461;
		start[14] = 0.0017542;
		start[15] = 0.0092754;
		start[16] = 0.0088993;
		start[17] = 0.0098372;
		start[18] = 0.0069361;
		start[19] = 0.0073114;
		start[20] = 0.0093419;
		start[21] = 0.0071762;
		start[22] = 0.004764;
		start[23] = 0.0058934;
		start[24] = 0.0071384;
		start[25] = 0.0011135;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0081483;
		start[5] = 0.0048232;
		start[6] = 0.0068355;
		start[7] = 0.008935;
		start[8] = 0.0069174;
		start[9] = 9.6491e-05;
		start[10] = 0.0030258;
		start[11] = 0.0054746;
		start[12] = 0.0061827;
		start[13] = 0.0053502;
		start[14] = 0.0002137;
		start[15] = 0.0053632;
		start[16] = 0.00018578;
		start[17] = 0.0082237;
		start[18] = 0.0075125;
		start[19] = 0.0047088;
		start[20] = 0.0077028;
		start[21] = 0.0057909;
		start[22] = 0.0031939;
		start[23] = 0.0036429;
		start[24] = 0.0016727;
		start[25] = 0.003659;
		std::vector<double> endEff (2);
		endEff[0] = 2.1957;
		endEff[1] = -2.1336;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0086132;
		start[5] = 0.00044629;
		start[6] = 0.0096203;
		start[7] = 0.00017565;
		start[8] = 0.0048786;
		start[9] = 0.0038198;
		start[10] = 0.0068656;
		start[11] = 0.0055735;
		start[12] = 0.0088846;
		start[13] = 0.0056943;
		start[14] = 0.0070714;
		start[15] = 0.0050649;
		start[16] = 0.0064764;
		start[17] = 0.0019437;
		start[18] = 0.0047533;
		start[19] = 0.00099151;
		start[20] = 0.0095106;
		start[21] = 0.0036749;
		start[22] = 0.0085684;
		start[23] = 0.009791;
		start[24] = 0.0030717;
		start[25] = 0.0083777;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.00046142;
		start[5] = 0.0086849;
		start[6] = 0.0033562;
		start[7] = 0.003013;
		start[8] = 0.0089534;
		start[9] = 0.005194;
		start[10] = 0.0026054;
		start[11] = 0.009409;
		start[12] = 0.0080774;
		start[13] = 0.0010101;
		start[14] = 0.009859;
		start[15] = 0.0011586;
		start[16] = 0.0052387;
		start[17] = 0.0010325;
		start[18] = 0.00086141;
		start[19] = 0.0055755;
		start[20] = 0.0091831;
		start[21] = 0.00061185;
		start[22] = 0.0087951;
		start[23] = 0.004915;
		start[24] = 0.0034141;
		start[25] = 0.008232;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0094672;
		start[5] = 0.0064952;
		start[6] = 0.0097781;
		start[7] = 0.0050576;
		start[8] = 0.0090456;
		start[9] = 0.0054457;
		start[10] = 0.0081929;
		start[11] = 0.0087198;
		start[12] = 0.0082119;
		start[13] = 0.0078066;
		start[14] = 0.0088322;
		start[15] = 0.00031571;
		start[16] = 0.0098213;
		start[17] = 0.0087829;
		start[18] = 0.0064778;
		start[19] = 0.0070651;
		start[20] = 0.0094874;
		start[21] = 0.0043209;
		start[22] = 0.009112;
		start[23] = 0.0098316;
		start[24] = 0.0044098;
		start[25] = 0.0024976;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.006295;
		start[5] = 0.0059841;
		start[6] = 0.00010067;
		start[7] = 0.0053643;
		start[8] = 0.0051935;
		start[9] = 0.0092205;
		start[10] = 0.0017565;
		start[11] = 0.0059696;
		start[12] = 6.0182e-05;
		start[13] = 0.0080898;
		start[14] = 0.0058606;
		start[15] = 0.0028311;
		start[16] = 0.0023089;
		start[17] = 0.00096778;
		start[18] = 0.0053181;
		start[19] = 0.0013764;
		start[20] = 0.0076729;
		start[21] = 0.00046373;
		start[22] = 0.00012366;
		start[23] = 0.0082481;
		start[24] = 0.0083469;
		start[25] = 0.0017448;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0078822;
		start[5] = 0.0030733;
		start[6] = 0.0093152;
		start[7] = 0.0088077;
		start[8] = 0.002049;
		start[9] = 0.0053935;
		start[10] = 0.0073019;
		start[11] = 0.0080059;
		start[12] = 0.0084996;
		start[13] = 0.0047081;
		start[14] = 0.0086438;
		start[15] = 0.0063425;
		start[16] = 0.0022079;
		start[17] = 0.0014014;
		start[18] = 0.0014491;
		start[19] = 0.0077409;
		start[20] = 0.0050109;
		start[21] = 0.0033088;
		start[22] = 0.0083067;
		start[23] = 0.0073027;
		start[24] = 0.00035014;
		start[25] = 0.0079734;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.007571;
		start[5] = 0.0069326;
		start[6] = 0.0049987;
		start[7] = 0.0019477;
		start[8] = 0.006648;
		start[9] = 0.0016376;
		start[10] = 0.0038881;
		start[11] = 0.001531;
		start[12] = 0.00052722;
		start[13] = 0.0069233;
		start[14] = 0.0006835;
		start[15] = 0.0057102;
		start[16] = 0.0094973;
		start[17] = 0.0010883;
		start[18] = 0.0056636;
		start[19] = 0.001321;
		start[20] = 0.0011125;
		start[21] = 0.0080337;
		start[22] = 0.0089554;
		start[23] = 0.002901;
		start[24] = 0.0056557;
		start[25] = 0.00019829;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0030486;
		start[5] = 0.009091;
		start[6] = 0.0059356;
		start[7] = 0.0058154;
		start[8] = 0.0086718;
		start[9] = 0.0083429;
		start[10] = 0.0014447;
		start[11] = 0.00053051;
		start[12] = 0.0087752;
		start[13] = 0.0041981;
		start[14] = 0.0014725;
		start[15] = 0.0022557;
		start[16] = 0.0030741;
		start[17] = 0.0076326;
		start[18] = 0.0031356;
		start[19] = 0.00020349;
		start[20] = 0.0083005;
		start[21] = 0.0099177;
		start[22] = 0.0097927;
		start[23] = 0.0060536;
		start[24] = 0.0057903;
		start[25] = 0.0035951;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0087226;
		start[5] = 0.0082394;
		start[6] = 0.002758;
		start[7] = 0.00475;
		start[8] = 0.00066855;
		start[9] = 0.00011296;
		start[10] = 0.0083993;
		start[11] = 0.00097081;
		start[12] = 0.0050446;
		start[13] = 0.0083505;
		start[14] = 0.0065302;
		start[15] = 0.006201;
		start[16] = 0.0067578;
		start[17] = 0.0082924;
		start[18] = 0.0043836;
		start[19] = 0.00054221;
		start[20] = 0.0022086;
		start[21] = 0.0088166;
		start[22] = 0.0056476;
		start[23] = 0.0019281;
		start[24] = 0.00078109;
		start[25] = 0.0018982;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0096536;
		start[5] = 0.0037906;
		start[6] = 0.0073162;
		start[7] = 0.0012238;
		start[8] = 0.0061136;
		start[9] = 0.0023716;
		start[10] = 0.0078078;
		start[11] = 0.0030804;
		start[12] = 0.0029302;
		start[13] = 0.0088625;
		start[14] = 0.0032834;
		start[15] = 0.0013129;
		start[16] = 0.0052803;
		start[17] = 0.0021322;
		start[18] = 0.0090572;
		start[19] = 0.004227;
		start[20] = 0.0048682;
		start[21] = 0.00080793;
		start[22] = 0.0096277;
		start[23] = 0.0047928;
		start[24] = 0.0017165;
		start[25] = 0.00059001;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0080707;
		start[5] = 0.00011534;
		start[6] = 0.0019753;
		start[7] = 0.0071116;
		start[8] = 0.0050173;
		start[9] = 0.0082417;
		start[10] = 0.0016699;
		start[11] = 0.0084495;
		start[12] = 0.0029892;
		start[13] = 0.0058311;
		start[14] = 0.0097214;
		start[15] = 0.0085931;
		start[16] = 0.0072128;
		start[17] = 0.0067191;
		start[18] = 0.0079898;
		start[19] = 0.0080452;
		start[20] = 0.0034995;
		start[21] = 0.00021786;
		start[22] = 0.005562;
		start[23] = 0.0023754;
		start[24] = 0.0036348;
		start[25] = 0.0042251;
		std::vector<double> endEff (2);
		endEff[0] = 0.44038;
		endEff[1] = 1.1904;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0063487;
		start[5] = 0.008469;
		start[6] = 0.0087144;
		start[7] = 0.0094831;
		start[8] = 0.0089184;
		start[9] = 0.0011703;
		start[10] = 0.0071692;
		start[11] = 0.0064345;
		start[12] = 0.00068921;
		start[13] = 0.0077605;
		start[14] = 0.003134;
		start[15] = 0.0035655;
		start[16] = 0.0012126;
		start[17] = 0.0025697;
		start[18] = 0.0039293;
		start[19] = 0.0097653;
		start[20] = 0.0033241;
		start[21] = 0.0055729;
		start[22] = 0.0076989;
		start[23] = 0.005508;
		start[24] = 0.002881;
		start[25] = 0.0048269;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0066544;
		start[5] = 0.0095129;
		start[6] = 0.0090786;
		start[7] = 0.0084838;
		start[8] = 0.0076596;
		start[9] = 0.0013296;
		start[10] = 0.0066119;
		start[11] = 0.0059581;
		start[12] = 0.0043144;
		start[13] = 0.0072453;
		start[14] = 0.0096073;
		start[15] = 0.00061734;
		start[16] = 0.0027394;
		start[17] = 0.0056712;
		start[18] = 0.0077335;
		start[19] = 0.0022862;
		start[20] = 0.0049661;
		start[21] = 0.0021329;
		start[22] = 0.0097262;
		start[23] = 0.002968;
		start[24] = 0.0068592;
		start[25] = 0.003709;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.003861;
		start[5] = 0.0038021;
		start[6] = 0.005553;
		start[7] = 0.0032949;
		start[8] = 0.0025532;
		start[9] = 0.0086547;
		start[10] = 0.00075969;
		start[11] = 0.0086526;
		start[12] = 0.0083652;
		start[13] = 0.0046726;
		start[14] = 0.0077154;
		start[15] = 0.0076026;
		start[16] = 0.0017267;
		start[17] = 0.0096376;
		start[18] = 0.0088059;
		start[19] = 0.0060141;
		start[20] = 0.00021648;
		start[21] = 0.0018179;
		start[22] = 0.0029883;
		start[23] = 0.0054329;
		start[24] = 0.0070582;
		start[25] = 0.00034855;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0036604;
		start[5] = 0.0057263;
		start[6] = 0.0033599;
		start[7] = 0.002516;
		start[8] = 0.0056397;
		start[9] = 0.0080645;
		start[10] = 0.0093097;
		start[11] = 0.0083488;
		start[12] = 0.0090606;
		start[13] = 0.0057495;
		start[14] = 0.0094678;
		start[15] = 0.0004116;
		start[16] = 0.0072203;
		start[17] = 0.0063473;
		start[18] = 0.001983;
		start[19] = 0.0070695;
		start[20] = 0.0088864;
		start[21] = 0.00072334;
		start[22] = 0.0094025;
		start[23] = 0.006249;
		start[24] = 0.0031525;
		start[25] = 0.005612;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.00086083;
		start[5] = 0.0024652;
		start[6] = 0.0034768;
		start[7] = 0.0086459;
		start[8] = 0.00099368;
		start[9] = 0.007383;
		start[10] = 0.0028215;
		start[11] = 0.0012025;
		start[12] = 0.0083424;
		start[13] = 0.0076424;
		start[14] = 0.0076569;
		start[15] = 0.00094798;
		start[16] = 0.0071075;
		start[17] = 0.0046315;
		start[18] = 0.00094203;
		start[19] = 0.0058461;
		start[20] = 0.0082083;
		start[21] = 0.0014552;
		start[22] = 0.0086694;
		start[23] = 0.0096962;
		start[24] = 0.00095612;
		start[25] = 0.0038793;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0076943;
		start[5] = 0.0029492;
		start[6] = 0.0087588;
		start[7] = 0.0015291;
		start[8] = 0.0095053;
		start[9] = 0.0054616;
		start[10] = 0.0084099;
		start[11] = 0.0083052;
		start[12] = 0.0039128;
		start[13] = 0.0026306;
		start[14] = 0.0077361;
		start[15] = 0.0083317;
		start[16] = 0.0026932;
		start[17] = 0.0074736;
		start[18] = 0.0061617;
		start[19] = 0.0040655;
		start[20] = 0.0065509;
		start[21] = 0.0045696;
		start[22] = 0.0074925;
		start[23] = 0.0010342;
		start[24] = 0.001604;
		start[25] = 0.0055603;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0041515;
		start[5] = 0.0033179;
		start[6] = 0.0021473;
		start[7] = 0.0077185;
		start[8] = 0.0072746;
		start[9] = 0.0084173;
		start[10] = 0.0015228;
		start[11] = 0.008162;
		start[12] = 0.00093611;
		start[13] = 0.0052766;
		start[14] = 0.0017087;
		start[15] = 0.0063567;
		start[16] = 0.0014655;
		start[17] = 0.0092143;
		start[18] = 0.0075729;
		start[19] = 0.0092902;
		start[20] = 0.0083729;
		start[21] = 0.00013048;
		start[22] = 0.0032323;
		start[23] = 0.0029079;
		start[24] = 0.0011513;
		start[25] = 0.0085028;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.009789;
		start[5] = 0.005311;
		start[6] = 0.0055189;
		start[7] = 0.0053658;
		start[8] = 0.0032607;
		start[9] = 0.0055205;
		start[10] = 0.001793;
		start[11] = 0.0053366;
		start[12] = 0.0023547;
		start[13] = 0.004514;
		start[14] = 0.0024352;
		start[15] = 0.0085551;
		start[16] = 0.00050707;
		start[17] = 0.0019265;
		start[18] = 0.0089412;
		start[19] = 0.0077632;
		start[20] = 0.0013421;
		start[21] = 0.0035693;
		start[22] = 0.0054664;
		start[23] = 0.0013586;
		start[24] = 0.005612;
		start[25] = 0.0042161;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.003771;
		start[5] = 0.0057539;
		start[6] = 0.0085436;
		start[7] = 0.0084553;
		start[8] = 0.0078915;
		start[9] = 0.0015017;
		start[10] = 0.0061613;
		start[11] = 0.0017596;
		start[12] = 0.0046344;
		start[13] = 0.00044178;
		start[14] = 0.0084379;
		start[15] = 0.0053515;
		start[16] = 0.0092229;
		start[17] = 0.0073178;
		start[18] = 0.0039333;
		start[19] = 0.0061409;
		start[20] = 0.0060837;
		start[21] = 0.0089616;
		start[22] = 0.0096997;
		start[23] = 0.0057457;
		start[24] = 0.0033991;
		start[25] = 0.0087973;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0095677;
		start[5] = 0.007435;
		start[6] = 0.0091967;
		start[7] = 0.0067388;
		start[8] = 0.0093753;
		start[9] = 0.0013933;
		start[10] = 0.0051838;
		start[11] = 0.0048304;
		start[12] = 0.0045875;
		start[13] = 0.0074373;
		start[14] = 0.0049849;
		start[15] = 0.0015567;
		start[16] = 0.0067126;
		start[17] = 0.008081;
		start[18] = 0.0064173;
		start[19] = 0.0044048;
		start[20] = 0.0025951;
		start[21] = 0.00587;
		start[22] = 0.0031204;
		start[23] = 0.0082344;
		start[24] = 0.0032919;
		start[25] = 0.0055917;
		std::vector<double> endEff (2);
		endEff[0] = 2.7557;
		endEff[1] = 1.5555;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0090673;
		start[5] = 0.00057522;
		start[6] = 0.0067938;
		start[7] = 0.0070797;
		start[8] = 0.0094893;
		start[9] = 0.0095724;
		start[10] = 0.0081581;
		start[11] = 0.0086389;
		start[12] = 0.0016756;
		start[13] = 0.0032705;
		start[14] = 0.0077305;
		start[15] = 0.00034559;
		start[16] = 0.00038444;
		start[17] = 0.0099621;
		start[18] = 0.0022015;
		start[19] = 0.0066661;
		start[20] = 0.004555;
		start[21] = 0.0027278;
		start[22] = 0.0029322;
		start[23] = 0.0093274;
		start[24] = 0.0086571;
		start[25] = 0.00097843;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0013381;
		start[5] = 0.0067327;
		start[6] = 0.0041512;
		start[7] = 0.0051018;
		start[8] = 4.7751e-05;
		start[9] = 0.0078286;
		start[10] = 0.00012336;
		start[11] = 0.0068121;
		start[12] = 0.0089837;
		start[13] = 0.0096183;
		start[14] = 0.0023413;
		start[15] = 0.0038269;
		start[16] = 0.0057751;
		start[17] = 0.0081645;
		start[18] = 0.0087931;
		start[19] = 0.0016297;
		start[20] = 0.0054647;
		start[21] = 0.00030984;
		start[22] = 0.0059538;
		start[23] = 0.0088993;
		start[24] = 0.004809;
		start[25] = 0.0026191;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0018236;
		start[5] = 0.0086782;
		start[6] = 0.009163;
		start[7] = 0.0021662;
		start[8] = 0.0094462;
		start[9] = 0.0053436;
		start[10] = 0.0083928;
		start[11] = 0.0096747;
		start[12] = 0.009885;
		start[13] = 0.0036844;
		start[14] = 0.005249;
		start[15] = 0.0065191;
		start[16] = 0.0015322;
		start[17] = 0.00087787;
		start[18] = 0.002471;
		start[19] = 0.0013794;
		start[20] = 0.007176;
		start[21] = 0.00013686;
		start[22] = 0.0061835;
		start[23] = 0.0097393;
		start[24] = 0.0058631;
		start[25] = 0.0050484;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0015382;
		start[5] = 0.0066314;
		start[6] = 0.0070918;
		start[7] = 0.0058253;
		start[8] = 0.0059962;
		start[9] = 0.00023132;
		start[10] = 0.0074932;
		start[11] = 0.0066786;
		start[12] = 8.646e-05;
		start[13] = 0.006491;
		start[14] = 0.001438;
		start[15] = 0.006522;
		start[16] = 0.0033185;
		start[17] = 0.0064557;
		start[18] = 0.0061074;
		start[19] = 0.0080943;
		start[20] = 0.0088543;
		start[21] = 0.0072447;
		start[22] = 0.0055507;
		start[23] = 0.0069471;
		start[24] = 0.0054931;
		start[25] = 0.0040822;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0016529;
		start[5] = 0.0022888;
		start[6] = 0.0012374;
		start[7] = 0.0019959;
		start[8] = 0.00062243;
		start[9] = 0.008908;
		start[10] = 0.0073059;
		start[11] = 0.0012199;
		start[12] = 0.0019379;
		start[13] = 0.0039656;
		start[14] = 0.0072601;
		start[15] = 0.00374;
		start[16] = 0.0028528;
		start[17] = 0.0010571;
		start[18] = 0.0079673;
		start[19] = 0.0051178;
		start[20] = 0.0060353;
		start[21] = 0.00047707;
		start[22] = 0.0042048;
		start[23] = 0.0083915;
		start[24] = 0.003939;
		start[25] = 0.0041093;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0028077;
		start[5] = 0.0023808;
		start[6] = 0.0088342;
		start[7] = 0.00031945;
		start[8] = 0.0072938;
		start[9] = 0.0028853;
		start[10] = 0.0060584;
		start[11] = 0.0077226;
		start[12] = 0.0094862;
		start[13] = 0.0025522;
		start[14] = 0.0041486;
		start[15] = 0.00025017;
		start[16] = 0.0046906;
		start[17] = 0.0063012;
		start[18] = 0.0064233;
		start[19] = 0.0050316;
		start[20] = 0.0040438;
		start[21] = 0.0056616;
		start[22] = 0.0057639;
		start[23] = 0.0075452;
		start[24] = 0.0012114;
		start[25] = 0.0067305;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0084776;
		start[5] = 0.0028212;
		start[6] = 0.0076498;
		start[7] = 0.0047719;
		start[8] = 0.0016739;
		start[9] = 0.007671;
		start[10] = 0.0096077;
		start[11] = 0.0023792;
		start[12] = 0.00051349;
		start[13] = 0.0011;
		start[14] = 0.0080204;
		start[15] = 0.0038988;
		start[16] = 0.0057164;
		start[17] = 0.0014652;
		start[18] = 0.0050576;
		start[19] = 0.0094555;
		start[20] = 0.0090116;
		start[21] = 0.0041719;
		start[22] = 0.0045468;
		start[23] = 0.0032115;
		start[24] = 0.00099506;
		start[25] = 0.00431;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0031479;
		start[5] = 0.0098779;
		start[6] = 0.0043405;
		start[7] = 0.0040137;
		start[8] = 0.0039476;
		start[9] = 0.0016269;
		start[10] = 0.0041752;
		start[11] = 0.0050881;
		start[12] = 0.0028219;
		start[13] = 0.0065207;
		start[14] = 0.0062174;
		start[15] = 0.0070863;
		start[16] = 0.0025306;
		start[17] = 0.0049032;
		start[18] = 0.0032183;
		start[19] = 0.0064727;
		start[20] = 0.0098188;
		start[21] = 0.0078157;
		start[22] = 0.0039386;
		start[23] = 0.0045776;
		start[24] = 0.0055188;
		start[25] = 0.0066988;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0078608;
		start[5] = 0.0040834;
		start[6] = 0.0069455;
		start[7] = 0.0028088;
		start[8] = 0.0013691;
		start[9] = 0.00011907;
		start[10] = 0.00036224;
		start[11] = 0.0035284;
		start[12] = 0.0080363;
		start[13] = 0.004356;
		start[14] = 0.0014419;
		start[15] = 0.0031194;
		start[16] = 0.0012651;
		start[17] = 0.008397;
		start[18] = 0.004462;
		start[19] = 0.00098663;
		start[20] = 0.0094233;
		start[21] = 0.0096262;
		start[22] = 0.0027511;
		start[23] = 0.0084433;
		start[24] = 0.0087918;
		start[25] = 0.0062501;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.0046163;
		start[5] = 0.0069273;
		start[6] = 0.0031416;
		start[7] = 0.0051596;
		start[8] = 0.0034512;
		start[9] = 0.0055593;
		start[10] = 0.0071319;
		start[11] = 0.0036079;
		start[12] = 0.0016922;
		start[13] = 0.0065022;
		start[14] = 0.0033776;
		start[15] = 0.0052289;
		start[16] = 0.0062283;
		start[17] = 0.0067944;
		start[18] = 0.0092318;
		start[19] = 0.00086719;
		start[20] = 0.0051164;
		start[21] = 0.0059673;
		start[22] = 0.0073948;
		start[23] = 0.0017429;
		start[24] = 0.0055271;
		start[25] = 0.0069261;
		std::vector<double> endEff (2);
		endEff[0] = -0.80983;
		endEff[1] = -0.73074;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.030204;
		start[1] = -0.019808;
		start[2] = 0.040404;
		start[3] = -0.093151;
		start[4] = 0.0002344;
		start[5] = 0.0070493;
		start[6] = 0.0037755;
		start[7] = 0.0095822;
		start[8] = 0.0060463;
		start[9] = 0.0042028;
		start[10] = 0.0036301;
		start[11] = 0.00037323;
		start[12] = 0.0017441;
		start[13] = 0.004024;
		start[14] = 0.0023995;
		start[15] = 0.0012639;
		start[16] = 0.0042915;
		start[17] = 0.0098847;
		start[18] = 0.0054758;
		start[19] = 0.0050724;
		start[20] = 0.0030048;
		start[21] = 0.0056878;
		start[22] = 0.0031194;
		start[23] = 0.0004358;
		start[24] = 0.0047543;
		start[25] = 0.00703;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084476;
		start[1] = -0.030057;
		start[2] = 0.064832;
		start[3] = 0.064231;
		start[4] = 0.0070753;
		start[5] = 0.0058466;
		start[6] = 0.0038642;
		start[7] = 0.00045939;
		start[8] = 0.005812;
		start[9] = 0.0027821;
		start[10] = 0.0030756;
		start[11] = 0.005146;
		start[12] = 0.0034342;
		start[13] = 0.0078866;
		start[14] = 0.0076641;
		start[15] = 0.0026014;
		start[16] = 0.003646;
		start[17] = 0.002709;
		start[18] = 0.0049095;
		start[19] = 0.0045486;
		start[20] = 0.0091517;
		start[21] = 0.007899;
		start[22] = 0.0063164;
		start[23] = 0.0048338;
		start[24] = 0.0030209;
		start[25] = 0.0089929;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.034941;
		start[1] = -0.026709;
		start[2] = 0.043309;
		start[3] = 0.039321;
		start[4] = 0.0066377;
		start[5] = 0.007167;
		start[6] = 0.0020567;
		start[7] = 0.0082454;
		start[8] = 0.0068559;
		start[9] = 0.0072848;
		start[10] = 0.0074617;
		start[11] = 0.0053184;
		start[12] = 0.00041072;
		start[13] = 0.0046421;
		start[14] = 0.0069206;
		start[15] = 0.0079819;
		start[16] = 0.0046124;
		start[17] = 0.00081681;
		start[18] = 0.0068094;
		start[19] = 0.008216;
		start[20] = 0.0094377;
		start[21] = 0.0011208;
		start[22] = 0.0093234;
		start[23] = 0.0072997;
		start[24] = 0.0030465;
		start[25] = 0.0053793;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.078267;
		start[1] = -0.09577;
		start[2] = -0.070403;
		start[3] = 0.00078817;
		start[4] = 0.0068865;
		start[5] = 0.003812;
		start[6] = 0.007464;
		start[7] = 0.0094323;
		start[8] = 0.0087267;
		start[9] = 0.0099624;
		start[10] = 0.0022966;
		start[11] = 0.0014762;
		start[12] = 0.0038793;
		start[13] = 0.00076153;
		start[14] = 0.0051299;
		start[15] = 0.00074773;
		start[16] = 0.0090822;
		start[17] = 0.0050326;
		start[18] = 0.0014399;
		start[19] = 0.005688;
		start[20] = 0.00096747;
		start[21] = 0.0079104;
		start[22] = 0.0031583;
		start[23] = 0.0028802;
		start[24] = 0.0037797;
		start[25] = 0.0035824;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.075094;
		start[1] = -0.041923;
		start[2] = -0.090664;
		start[3] = 0.048201;
		start[4] = 0.0065274;
		start[5] = 0.0040014;
		start[6] = 0.0050392;
		start[7] = 0.0092828;
		start[8] = 0.0042747;
		start[9] = 0.0041839;
		start[10] = 0.0050095;
		start[11] = 0.0087289;
		start[12] = 0.0093123;
		start[13] = 0.0045699;
		start[14] = 0.0064871;
		start[15] = 0.0077091;
		start[16] = 0.0034944;
		start[17] = 0.0063381;
		start[18] = 0.0016829;
		start[19] = 0.002722;
		start[20] = 0.0021945;
		start[21] = 0.0041428;
		start[22] = 0.0028327;
		start[23] = 0.0045164;
		start[24] = 0.0074757;
		start[25] = 0.00228;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.084758;
		start[1] = -0.051112;
		start[2] = -0.031806;
		start[3] = 0.055106;
		start[4] = 0.0006604;
		start[5] = 0.00069694;
		start[6] = 0.0024047;
		start[7] = 0.0081092;
		start[8] = 0.0093132;
		start[9] = 0.003459;
		start[10] = 0.0042621;
		start[11] = 0.0049164;
		start[12] = 0.0087195;
		start[13] = 0.0067817;
		start[14] = 0.0049641;
		start[15] = 0.008631;
		start[16] = 0.004819;
		start[17] = 0.00049704;
		start[18] = 0.0059019;
		start[19] = 0.0049809;
		start[20] = 0.0093831;
		start[21] = 5.3368e-05;
		start[22] = 0.0096305;
		start[23] = 0.008281;
		start[24] = 0.0075494;
		start[25] = 0.0075178;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.063929;
		start[1] = 0.013353;
		start[2] = 0.022044;
		start[3] = 0.0046202;
		start[4] = 0.0017997;
		start[5] = 0.0030812;
		start[6] = 0.0093663;
		start[7] = 0.002756;
		start[8] = 0.0085712;
		start[9] = 0.0049042;
		start[10] = 0.001324;
		start[11] = 0.0060883;
		start[12] = 0.0072366;
		start[13] = 0.005807;
		start[14] = 0.0066509;
		start[15] = 0.0086566;
		start[16] = 0.0049763;
		start[17] = 0.002902;
		start[18] = 0.0099811;
		start[19] = 0.0049468;
		start[20] = 0.0083221;
		start[21] = 0.0073825;
		start[22] = 0.0050669;
		start[23] = 0.0096896;
		start[24] = 0.00055901;
		start[25] = 0.0010637;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = 0.046081;
		start[1] = -0.017765;
		start[2] = -0.093941;
		start[3] = -0.076137;
		start[4] = 0.0074287;
		start[5] = 0.00092436;
		start[6] = 0.0063214;
		start[7] = 0.0048096;
		start[8] = 0.0048371;
		start[9] = 0.0042257;
		start[10] = 0.0058256;
		start[11] = 0.0074969;
		start[12] = 0.0010576;
		start[13] = 0.004729;
		start[14] = 0.0010906;
		start[15] = 0.001335;
		start[16] = 0.0033825;
		start[17] = 0.00090852;
		start[18] = 0.0091239;
		start[19] = 0.006606;
		start[20] = 0.0058401;
		start[21] = 0.0015444;
		start[22] = 0.0058888;
		start[23] = 0.0062931;
		start[24] = 0.0028095;
		start[25] = 0.0015583;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.09269;
		start[1] = -0.0030012;
		start[2] = -0.0019797;
		start[3] = -0.01112;
		start[4] = 0.0052341;
		start[5] = 0.0025931;
		start[6] = 0.00079613;
		start[7] = 0.00045582;
		start[8] = 0.00074535;
		start[9] = 0.0048007;
		start[10] = 0.0053868;
		start[11] = 0.0086232;
		start[12] = 0.0029251;
		start[13] = 4.1182e-05;
		start[14] = 0.0054412;
		start[15] = 0.006187;
		start[16] = 0.0037255;
		start[17] = 0.0046685;
		start[18] = 0.0081786;
		start[19] = 0.00083205;
		start[20] = 0.00067287;
		start[21] = 0.0081545;
		start[22] = 0.0025109;
		start[23] = 0.0023185;
		start[24] = 0.003697;
		start[25] = 0.00011953;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (26);
		start[0] = -0.069933;
		start[1] = -0.067881;
		start[2] = 0.074314;
		start[3] = 0.04944;
		start[4] = 0.003863;
		start[5] = 0.0080915;
		start[6] = 0.0077791;
		start[7] = 0.0039034;
		start[8] = 0.0082892;
		start[9] = 0.0048069;
		start[10] = 0.0023741;
		start[11] = 0.0077234;
		start[12] = 0.0042072;
		start[13] = 0.0002762;
		start[14] = 0.0036445;
		start[15] = 0.0043302;
		start[16] = 0.0049133;
		start[17] = 0.0028658;
		start[18] = 0.0047391;
		start[19] = 0.0044859;
		start[20] = 0.0095657;
		start[21] = 0.0058936;
		start[22] = 0.0049128;
		start[23] = 0.0022242;
		start[24] = 0.0064723;
		start[25] = 0.0034088;
		std::vector<double> endEff (2);
		endEff[0] = -0.035586;
		endEff[1] = -2.3391;
		solveFor( start, endEff);
	}


  return 0;
}
