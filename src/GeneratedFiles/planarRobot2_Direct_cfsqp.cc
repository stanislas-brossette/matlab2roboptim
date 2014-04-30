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
    (2, 1, "CostFunction_planarRobot2"),
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
  
	result[0] = 0.0;
}

template <typename T>
void
CostFunction<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];

  switch (id)
    {
      
		case 0: 
			 grad[0] = 0; 
			 grad[1] = 0; 
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
    (2, 1, "EEConstraint_1_planarRobot2"),
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
  
	result[0] = cos(q_01) - 1.0*EE_1_1 + cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02);
}

template <typename T>
void
EEConstraint_1<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];

  switch (id)
    {
      
		case 0: 
			 grad[0] = - 1.0*sin(q_01) - 1.0*cos(q_01)*sin(q_02) - 1.0*cos(q_02)*sin(q_01); 
			 grad[1] = - 1.0*cos(q_01)*sin(q_02) - 1.0*cos(q_02)*sin(q_01); 
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
    (2, 1, "EEConstraint_2_planarRobot2"),
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
  
	result[0] = sin(q_01) - 1.0*EE_1_2 + cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01);
}

template <typename T>
void
EEConstraint_2<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];

  switch (id)
    {
      
		case 0: 
			 grad[0] = cos(q_01) + cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02); 
			 grad[1] = cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02); 
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
		std::vector<double> start (2);
		start[0] = -0.058557;
		start[1] = 0.04525;
		std::vector<double> endEff (2);
		endEff[0] = 1.2247;
		endEff[1] = -1.315;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023139;
		start[1] = 0.07969;
		std::vector<double> endEff (2);
		endEff[0] = 1.2247;
		endEff[1] = -1.315;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033993;
		start[1] = 0.030521;
		std::vector<double> endEff (2);
		endEff[0] = 1.2247;
		endEff[1] = -1.315;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033899;
		start[1] = 0.04094;
		std::vector<double> endEff (2);
		endEff[0] = 1.2247;
		endEff[1] = -1.315;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.036086;
		start[1] = 0.021245;
		std::vector<double> endEff (2);
		endEff[0] = 1.2247;
		endEff[1] = -1.315;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.021636;
		start[1] = 0.058607;
		std::vector<double> endEff (2);
		endEff[0] = 1.2247;
		endEff[1] = -1.315;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.044904;
		start[1] = -0.063861;
		std::vector<double> endEff (2);
		endEff[0] = 1.2247;
		endEff[1] = -1.315;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.085294;
		start[1] = -0.002824;
		std::vector<double> endEff (2);
		endEff[0] = 1.2247;
		endEff[1] = -1.315;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.061371;
		start[1] = -0.02058;
		std::vector<double> endEff (2);
		endEff[0] = 1.2247;
		endEff[1] = -1.315;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.060364;
		start[1] = 0.017921;
		std::vector<double> endEff (2);
		endEff[0] = 1.2247;
		endEff[1] = -1.315;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058557;
		start[1] = 0.04525;
		std::vector<double> endEff (2);
		endEff[0] = -0.36311;
		endEff[1] = 1.8204;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023139;
		start[1] = 0.07969;
		std::vector<double> endEff (2);
		endEff[0] = -0.36311;
		endEff[1] = 1.8204;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033993;
		start[1] = 0.030521;
		std::vector<double> endEff (2);
		endEff[0] = -0.36311;
		endEff[1] = 1.8204;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033899;
		start[1] = 0.04094;
		std::vector<double> endEff (2);
		endEff[0] = -0.36311;
		endEff[1] = 1.8204;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.036086;
		start[1] = 0.021245;
		std::vector<double> endEff (2);
		endEff[0] = -0.36311;
		endEff[1] = 1.8204;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.021636;
		start[1] = 0.058607;
		std::vector<double> endEff (2);
		endEff[0] = -0.36311;
		endEff[1] = 1.8204;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.044904;
		start[1] = -0.063861;
		std::vector<double> endEff (2);
		endEff[0] = -0.36311;
		endEff[1] = 1.8204;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.085294;
		start[1] = -0.002824;
		std::vector<double> endEff (2);
		endEff[0] = -0.36311;
		endEff[1] = 1.8204;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.061371;
		start[1] = -0.02058;
		std::vector<double> endEff (2);
		endEff[0] = -0.36311;
		endEff[1] = 1.8204;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.060364;
		start[1] = 0.017921;
		std::vector<double> endEff (2);
		endEff[0] = -0.36311;
		endEff[1] = 1.8204;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058557;
		start[1] = 0.04525;
		std::vector<double> endEff (2);
		endEff[0] = -0.53418;
		endEff[1] = 0.0076009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023139;
		start[1] = 0.07969;
		std::vector<double> endEff (2);
		endEff[0] = -0.53418;
		endEff[1] = 0.0076009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033993;
		start[1] = 0.030521;
		std::vector<double> endEff (2);
		endEff[0] = -0.53418;
		endEff[1] = 0.0076009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033899;
		start[1] = 0.04094;
		std::vector<double> endEff (2);
		endEff[0] = -0.53418;
		endEff[1] = 0.0076009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.036086;
		start[1] = 0.021245;
		std::vector<double> endEff (2);
		endEff[0] = -0.53418;
		endEff[1] = 0.0076009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.021636;
		start[1] = 0.058607;
		std::vector<double> endEff (2);
		endEff[0] = -0.53418;
		endEff[1] = 0.0076009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.044904;
		start[1] = -0.063861;
		std::vector<double> endEff (2);
		endEff[0] = -0.53418;
		endEff[1] = 0.0076009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.085294;
		start[1] = -0.002824;
		std::vector<double> endEff (2);
		endEff[0] = -0.53418;
		endEff[1] = 0.0076009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.061371;
		start[1] = -0.02058;
		std::vector<double> endEff (2);
		endEff[0] = -0.53418;
		endEff[1] = 0.0076009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.060364;
		start[1] = 0.017921;
		std::vector<double> endEff (2);
		endEff[0] = -0.53418;
		endEff[1] = 0.0076009;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058557;
		start[1] = 0.04525;
		std::vector<double> endEff (2);
		endEff[0] = -0.37869;
		endEff[1] = -0.76408;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023139;
		start[1] = 0.07969;
		std::vector<double> endEff (2);
		endEff[0] = -0.37869;
		endEff[1] = -0.76408;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033993;
		start[1] = 0.030521;
		std::vector<double> endEff (2);
		endEff[0] = -0.37869;
		endEff[1] = -0.76408;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033899;
		start[1] = 0.04094;
		std::vector<double> endEff (2);
		endEff[0] = -0.37869;
		endEff[1] = -0.76408;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.036086;
		start[1] = 0.021245;
		std::vector<double> endEff (2);
		endEff[0] = -0.37869;
		endEff[1] = -0.76408;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.021636;
		start[1] = 0.058607;
		std::vector<double> endEff (2);
		endEff[0] = -0.37869;
		endEff[1] = -0.76408;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.044904;
		start[1] = -0.063861;
		std::vector<double> endEff (2);
		endEff[0] = -0.37869;
		endEff[1] = -0.76408;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.085294;
		start[1] = -0.002824;
		std::vector<double> endEff (2);
		endEff[0] = -0.37869;
		endEff[1] = -0.76408;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.061371;
		start[1] = -0.02058;
		std::vector<double> endEff (2);
		endEff[0] = -0.37869;
		endEff[1] = -0.76408;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.060364;
		start[1] = 0.017921;
		std::vector<double> endEff (2);
		endEff[0] = -0.37869;
		endEff[1] = -0.76408;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058557;
		start[1] = 0.04525;
		std::vector<double> endEff (2);
		endEff[0] = 0.10038;
		endEff[1] = -1.3696;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023139;
		start[1] = 0.07969;
		std::vector<double> endEff (2);
		endEff[0] = 0.10038;
		endEff[1] = -1.3696;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033993;
		start[1] = 0.030521;
		std::vector<double> endEff (2);
		endEff[0] = 0.10038;
		endEff[1] = -1.3696;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033899;
		start[1] = 0.04094;
		std::vector<double> endEff (2);
		endEff[0] = 0.10038;
		endEff[1] = -1.3696;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.036086;
		start[1] = 0.021245;
		std::vector<double> endEff (2);
		endEff[0] = 0.10038;
		endEff[1] = -1.3696;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.021636;
		start[1] = 0.058607;
		std::vector<double> endEff (2);
		endEff[0] = 0.10038;
		endEff[1] = -1.3696;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.044904;
		start[1] = -0.063861;
		std::vector<double> endEff (2);
		endEff[0] = 0.10038;
		endEff[1] = -1.3696;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.085294;
		start[1] = -0.002824;
		std::vector<double> endEff (2);
		endEff[0] = 0.10038;
		endEff[1] = -1.3696;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.061371;
		start[1] = -0.02058;
		std::vector<double> endEff (2);
		endEff[0] = 0.10038;
		endEff[1] = -1.3696;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.060364;
		start[1] = 0.017921;
		std::vector<double> endEff (2);
		endEff[0] = 0.10038;
		endEff[1] = -1.3696;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058557;
		start[1] = 0.04525;
		std::vector<double> endEff (2);
		endEff[0] = -1.0496;
		endEff[1] = 1.2805;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023139;
		start[1] = 0.07969;
		std::vector<double> endEff (2);
		endEff[0] = -1.0496;
		endEff[1] = 1.2805;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033993;
		start[1] = 0.030521;
		std::vector<double> endEff (2);
		endEff[0] = -1.0496;
		endEff[1] = 1.2805;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033899;
		start[1] = 0.04094;
		std::vector<double> endEff (2);
		endEff[0] = -1.0496;
		endEff[1] = 1.2805;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.036086;
		start[1] = 0.021245;
		std::vector<double> endEff (2);
		endEff[0] = -1.0496;
		endEff[1] = 1.2805;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.021636;
		start[1] = 0.058607;
		std::vector<double> endEff (2);
		endEff[0] = -1.0496;
		endEff[1] = 1.2805;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.044904;
		start[1] = -0.063861;
		std::vector<double> endEff (2);
		endEff[0] = -1.0496;
		endEff[1] = 1.2805;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.085294;
		start[1] = -0.002824;
		std::vector<double> endEff (2);
		endEff[0] = -1.0496;
		endEff[1] = 1.2805;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.061371;
		start[1] = -0.02058;
		std::vector<double> endEff (2);
		endEff[0] = -1.0496;
		endEff[1] = 1.2805;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.060364;
		start[1] = 0.017921;
		std::vector<double> endEff (2);
		endEff[0] = -1.0496;
		endEff[1] = 1.2805;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058557;
		start[1] = 0.04525;
		std::vector<double> endEff (2);
		endEff[0] = -0.049387;
		endEff[1] = 0.67275;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023139;
		start[1] = 0.07969;
		std::vector<double> endEff (2);
		endEff[0] = -0.049387;
		endEff[1] = 0.67275;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033993;
		start[1] = 0.030521;
		std::vector<double> endEff (2);
		endEff[0] = -0.049387;
		endEff[1] = 0.67275;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033899;
		start[1] = 0.04094;
		std::vector<double> endEff (2);
		endEff[0] = -0.049387;
		endEff[1] = 0.67275;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.036086;
		start[1] = 0.021245;
		std::vector<double> endEff (2);
		endEff[0] = -0.049387;
		endEff[1] = 0.67275;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.021636;
		start[1] = 0.058607;
		std::vector<double> endEff (2);
		endEff[0] = -0.049387;
		endEff[1] = 0.67275;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.044904;
		start[1] = -0.063861;
		std::vector<double> endEff (2);
		endEff[0] = -0.049387;
		endEff[1] = 0.67275;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.085294;
		start[1] = -0.002824;
		std::vector<double> endEff (2);
		endEff[0] = -0.049387;
		endEff[1] = 0.67275;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.061371;
		start[1] = -0.02058;
		std::vector<double> endEff (2);
		endEff[0] = -0.049387;
		endEff[1] = 0.67275;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.060364;
		start[1] = 0.017921;
		std::vector<double> endEff (2);
		endEff[0] = -0.049387;
		endEff[1] = 0.67275;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058557;
		start[1] = 0.04525;
		std::vector<double> endEff (2);
		endEff[0] = -0.53133;
		endEff[1] = 0.33782;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023139;
		start[1] = 0.07969;
		std::vector<double> endEff (2);
		endEff[0] = -0.53133;
		endEff[1] = 0.33782;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033993;
		start[1] = 0.030521;
		std::vector<double> endEff (2);
		endEff[0] = -0.53133;
		endEff[1] = 0.33782;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033899;
		start[1] = 0.04094;
		std::vector<double> endEff (2);
		endEff[0] = -0.53133;
		endEff[1] = 0.33782;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.036086;
		start[1] = 0.021245;
		std::vector<double> endEff (2);
		endEff[0] = -0.53133;
		endEff[1] = 0.33782;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.021636;
		start[1] = 0.058607;
		std::vector<double> endEff (2);
		endEff[0] = -0.53133;
		endEff[1] = 0.33782;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.044904;
		start[1] = -0.063861;
		std::vector<double> endEff (2);
		endEff[0] = -0.53133;
		endEff[1] = 0.33782;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.085294;
		start[1] = -0.002824;
		std::vector<double> endEff (2);
		endEff[0] = -0.53133;
		endEff[1] = 0.33782;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.061371;
		start[1] = -0.02058;
		std::vector<double> endEff (2);
		endEff[0] = -0.53133;
		endEff[1] = 0.33782;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.060364;
		start[1] = 0.017921;
		std::vector<double> endEff (2);
		endEff[0] = -0.53133;
		endEff[1] = 0.33782;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058557;
		start[1] = 0.04525;
		std::vector<double> endEff (2);
		endEff[0] = 1.3546;
		endEff[1] = 0.81196;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023139;
		start[1] = 0.07969;
		std::vector<double> endEff (2);
		endEff[0] = 1.3546;
		endEff[1] = 0.81196;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033993;
		start[1] = 0.030521;
		std::vector<double> endEff (2);
		endEff[0] = 1.3546;
		endEff[1] = 0.81196;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033899;
		start[1] = 0.04094;
		std::vector<double> endEff (2);
		endEff[0] = 1.3546;
		endEff[1] = 0.81196;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.036086;
		start[1] = 0.021245;
		std::vector<double> endEff (2);
		endEff[0] = 1.3546;
		endEff[1] = 0.81196;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.021636;
		start[1] = 0.058607;
		std::vector<double> endEff (2);
		endEff[0] = 1.3546;
		endEff[1] = 0.81196;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.044904;
		start[1] = -0.063861;
		std::vector<double> endEff (2);
		endEff[0] = 1.3546;
		endEff[1] = 0.81196;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.085294;
		start[1] = -0.002824;
		std::vector<double> endEff (2);
		endEff[0] = 1.3546;
		endEff[1] = 0.81196;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.061371;
		start[1] = -0.02058;
		std::vector<double> endEff (2);
		endEff[0] = 1.3546;
		endEff[1] = 0.81196;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.060364;
		start[1] = 0.017921;
		std::vector<double> endEff (2);
		endEff[0] = 1.3546;
		endEff[1] = 0.81196;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058557;
		start[1] = 0.04525;
		std::vector<double> endEff (2);
		endEff[0] = -0.11366;
		endEff[1] = 0.89961;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023139;
		start[1] = 0.07969;
		std::vector<double> endEff (2);
		endEff[0] = -0.11366;
		endEff[1] = 0.89961;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033993;
		start[1] = 0.030521;
		std::vector<double> endEff (2);
		endEff[0] = -0.11366;
		endEff[1] = 0.89961;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.033899;
		start[1] = 0.04094;
		std::vector<double> endEff (2);
		endEff[0] = -0.11366;
		endEff[1] = 0.89961;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.036086;
		start[1] = 0.021245;
		std::vector<double> endEff (2);
		endEff[0] = -0.11366;
		endEff[1] = 0.89961;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.021636;
		start[1] = 0.058607;
		std::vector<double> endEff (2);
		endEff[0] = -0.11366;
		endEff[1] = 0.89961;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.044904;
		start[1] = -0.063861;
		std::vector<double> endEff (2);
		endEff[0] = -0.11366;
		endEff[1] = 0.89961;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.085294;
		start[1] = -0.002824;
		std::vector<double> endEff (2);
		endEff[0] = -0.11366;
		endEff[1] = 0.89961;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.061371;
		start[1] = -0.02058;
		std::vector<double> endEff (2);
		endEff[0] = -0.11366;
		endEff[1] = 0.89961;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.060364;
		start[1] = 0.017921;
		std::vector<double> endEff (2);
		endEff[0] = -0.11366;
		endEff[1] = 0.89961;
		solveFor( start, endEff);
	}


  return 0;
}
