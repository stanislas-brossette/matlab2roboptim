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
    (3, 1, "CostFunction_planarRobot3"),
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

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0; 
			 grad[1] = 0; 
			 grad[2] = 0; 
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
    (3, 1, "EEConstraint_1_planarRobot3"),
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
  
	result[0] = cos(q_01) - 1.0*EE_1_1 + cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02) + cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01));
}

template <typename T>
void
EEConstraint_1<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];

  switch (id)
    {
      
		case 0: 
			 grad[0] = - 1.0*sin(q_01) - 1.0*cos(q_01)*sin(q_02) - 1.0*cos(q_02)*sin(q_01) - 1.0*sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)); 
			 grad[1] = - 1.0*cos(q_01)*sin(q_02) - 1.0*cos(q_02)*sin(q_01) - 1.0*sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)); 
			 grad[2] = - 1.0*sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)); 
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
    (3, 1, "EEConstraint_2_planarRobot3"),
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
  
	result[0] = sin(q_01) - 1.0*EE_1_2 + cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01) + sin(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) + cos(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01));
}

template <typename T>
void
EEConstraint_2<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];

  switch (id)
    {
      
		case 0: 
			 grad[0] = cos(q_01) + cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02) + cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)); 
			 grad[1] = cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02) + cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)); 
			 grad[2] = cos(q_03)*(cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02)) - 1.0*sin(q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)); 
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
		std::vector<double> start (3);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		std::vector<double> endEff (2);
		endEff[0] = -1.4592;
		endEff[1] = 0.33191;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		std::vector<double> endEff (2);
		endEff[0] = -2.6356;
		endEff[1] = -0.57871;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		std::vector<double> endEff (2);
		endEff[0] = 1.242;
		endEff[1] = -0.02449;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		std::vector<double> endEff (2);
		endEff[0] = -0.12504;
		endEff[1] = -0.15731;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		std::vector<double> endEff (2);
		endEff[0] = 1.3271;
		endEff[1] = -0.13132;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		std::vector<double> endEff (2);
		endEff[0] = -1.5201;
		endEff[1] = 0.32406;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		std::vector<double> endEff (2);
		endEff[0] = -0.0085463;
		endEff[1] = -0.31123;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		std::vector<double> endEff (2);
		endEff[0] = 1.3655;
		endEff[1] = 0.6819;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		std::vector<double> endEff (2);
		endEff[0] = 0.039174;
		endEff[1] = -0.99108;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.063504;
		start[1] = 0.057571;
		start[2] = -0.061271;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.078256;
		start[1] = -0.035462;
		start[2] = 0.061023;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.025642;
		start[1] = -0.0079436;
		start[2] = 0.049772;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.04383;
		start[1] = 0.035767;
		start[2] = 0.092472;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.062372;
		start[1] = 0.090938;
		start[2] = 0.040495;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.039216;
		start[1] = -0.007277;
		start[2] = 0.07596;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.059053;
		start[1] = -0.00051015;
		start[2] = 0.054403;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.066256;
		start[1] = 0.077311;
		start[2] = -0.099865;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = -0.05166;
		start[1] = -0.003692;
		start[2] = 0.0046751;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (3);
		start[0] = 0.045498;
		start[1] = -0.092632;
		start[2] = 0.036111;
		std::vector<double> endEff (2);
		endEff[0] = 0.99425;
		endEff[1] = 1.2266;
		solveFor( start, endEff);
	}


  return 0;
}
