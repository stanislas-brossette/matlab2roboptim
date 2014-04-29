// compile command: 
// g++ -ggdb3 -I/usr/include/log4cxx `pkg-config --cflags roboptim-core` src/generatedFiles/@FUNCTION_NAME@.cc `pkg-config --libs roboptim-core` -o bin/@FUNCTION_NAME@
#include <iostream>
#include <boost/mpl/vector.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
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
int main ()
{
  // Set the starting point.
  roboptim::Function::vector_t start (22);
  start[0] = 0.39978;
	start[1] = 0.25987;
	start[2] = 0.80007;
	start[3] = 0.43141;
	start[4] = 0.0;
	start[5] = 0.0;
	start[6] = 0.0;
	start[7] = 0.0;
	start[8] = 0.0;
	start[9] = 0.0;
	start[10] = 0.0;
	start[11] = 0.0;
	start[12] = 0.0;
	start[13] = 0.0;
	start[14] = 0.0;
	start[15] = 0.0;
	start[16] = 0.0;
	start[17] = 0.0;
	start[18] = 0.0;
	start[19] = 0.0;
	start[20] = 0.0;
	start[21] = 0.0;

  double EE_1_1 = 0.53834;
	double EE_1_2 = 0.99613;

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
  roboptim::SolverFactory<solver_t> factory ("cfsqp", pb);
  solver_t& solver = factory ();

  solver_t::result_t res = solver.minimum ();

  
  std::cout << solver << std::endl;

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

        return 0;
      }

    case solver_t::SOLVER_VALUE_WARNINGS:
      {
        // Get the result.
        roboptim::ResultWithWarnings& result =
	  boost::get<roboptim::ResultWithWarnings> (res);

        // Display the result.
	std::cout << "A solution has been found: " << std::endl
                  << result << std::endl;

        return 0;
      }

    case solver_t::SOLVER_NO_SOLUTION:
    case solver_t::SOLVER_ERROR:
      {
	std::cout << "A solution should have been found. Failing..."
                  << std::endl
                  << boost::get<roboptim::SolverError> (res).what ()
                  << std::endl;

        return 2;
      }
    }

  return 0;
}