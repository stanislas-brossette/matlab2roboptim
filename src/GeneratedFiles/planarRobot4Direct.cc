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
    (4, 1, "CostFunction_planarRobot4"),
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

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0; 
			 grad[1] = 0; 
			 grad[2] = 0; 
			 grad[3] = 0; 
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
    (4, 1, "EEConstraint_1_planarRobot4"),
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
  
	result[0] = cos(q_01) - 1.0*EE_1_1 + cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02) + cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)));
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

  switch (id)
    {
      
		case 0: 
			 grad[0] = - 1.0*sin(q_01) - 1.0*cos(q_01)*sin(q_02) - 1.0*cos(q_02)*sin(q_01) - 1.0*sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[1] = - 1.0*cos(q_01)*sin(q_02) - 1.0*cos(q_02)*sin(q_01) - 1.0*sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[2] = - 1.0*sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[3] = - 1.0*sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
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
    (4, 1, "EEConstraint_2_planarRobot4"),
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
  
	result[0] = sin(q_01) - 1.0*EE_1_2 + cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01) + sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + sin(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + cos(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)));
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

  switch (id)
    {
      
		case 0: 
			 grad[0] = cos(q_01) + cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02) + cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[1] = cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02) + cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[2] = cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 grad[3] = cos(q_04)*(cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))) - 1.0*sin(q_04)*(sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))); 
			 break;
    default:
      assert (0 && "should never happen");
    }
}
int main ()
{
  // Set the starting point.
  roboptim::Function::vector_t start (4);
  start[0] = 0.34039;
	start[1] = 0.58527;
	start[2] = 0.22381;
	start[3] = 0.75127;

  double EE_1_1 = 0.44559;
	double EE_1_2 = 0.64631;

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

  // Create constraints.
  {
		EEConstraint_1<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint ( cstrFunc_1, bounds, scales); 
	}
	{
		EEConstraint_2<roboptim::EigenMatrixDense>::intervals_t bounds;
		solver_t::problem_t::scales_t scales;
		bounds.push_back(roboptim::Function::makeInterval (0., 0.));
		scales.push_back(1.);
		pb.addConstraint ( cstrFunc_2, bounds, scales); 
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
