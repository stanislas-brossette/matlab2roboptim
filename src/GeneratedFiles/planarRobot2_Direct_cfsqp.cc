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
		std::vector<double> start (2);
		start[0] = 0.045744;
		start[1] = -0.065836;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.019342;
		start[1] = 0.07281;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.050277;
		start[1] = 0.034234;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.089024;
		start[1] = -0.044114;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058285;
		start[1] = -0.095648;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.080163;
		start[1] = -0.039822;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.072837;
		start[1] = 0.036878;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.017422;
		start[1] = 0.094855;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		std::vector<double> endEff (2);
		endEff[0] = 1.5139;
		endEff[1] = -0.42266;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.045744;
		start[1] = -0.065836;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.019342;
		start[1] = 0.07281;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.050277;
		start[1] = 0.034234;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.089024;
		start[1] = -0.044114;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058285;
		start[1] = -0.095648;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.080163;
		start[1] = -0.039822;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.072837;
		start[1] = 0.036878;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.017422;
		start[1] = 0.094855;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		std::vector<double> endEff (2);
		endEff[0] = 0.55762;
		endEff[1] = 0.9338;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.045744;
		start[1] = -0.065836;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.019342;
		start[1] = 0.07281;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.050277;
		start[1] = 0.034234;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.089024;
		start[1] = -0.044114;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058285;
		start[1] = -0.095648;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.080163;
		start[1] = -0.039822;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.072837;
		start[1] = 0.036878;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.017422;
		start[1] = 0.094855;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		std::vector<double> endEff (2);
		endEff[0] = 0.49952;
		endEff[1] = -0.69749;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.045744;
		start[1] = -0.065836;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.019342;
		start[1] = 0.07281;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.050277;
		start[1] = 0.034234;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.089024;
		start[1] = -0.044114;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058285;
		start[1] = -0.095648;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.080163;
		start[1] = -0.039822;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.072837;
		start[1] = 0.036878;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.017422;
		start[1] = 0.094855;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		std::vector<double> endEff (2);
		endEff[0] = 0.36568;
		endEff[1] = -1.8849;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.045744;
		start[1] = -0.065836;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.019342;
		start[1] = 0.07281;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.050277;
		start[1] = 0.034234;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.089024;
		start[1] = -0.044114;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058285;
		start[1] = -0.095648;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.080163;
		start[1] = -0.039822;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.072837;
		start[1] = 0.036878;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.017422;
		start[1] = 0.094855;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		std::vector<double> endEff (2);
		endEff[0] = 0.15752;
		endEff[1] = 0.99376;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.045744;
		start[1] = -0.065836;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.019342;
		start[1] = 0.07281;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.050277;
		start[1] = 0.034234;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.089024;
		start[1] = -0.044114;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058285;
		start[1] = -0.095648;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.080163;
		start[1] = -0.039822;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.072837;
		start[1] = 0.036878;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.017422;
		start[1] = 0.094855;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		std::vector<double> endEff (2);
		endEff[0] = -0.35609;
		endEff[1] = 1.3619;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.045744;
		start[1] = -0.065836;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.019342;
		start[1] = 0.07281;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.050277;
		start[1] = 0.034234;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.089024;
		start[1] = -0.044114;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058285;
		start[1] = -0.095648;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.080163;
		start[1] = -0.039822;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.072837;
		start[1] = 0.036878;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.017422;
		start[1] = 0.094855;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		std::vector<double> endEff (2);
		endEff[0] = -0.55271;
		endEff[1] = -0.80283;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.045744;
		start[1] = -0.065836;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.019342;
		start[1] = 0.07281;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.050277;
		start[1] = 0.034234;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.089024;
		start[1] = -0.044114;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058285;
		start[1] = -0.095648;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.080163;
		start[1] = -0.039822;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.072837;
		start[1] = 0.036878;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.017422;
		start[1] = 0.094855;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		std::vector<double> endEff (2);
		endEff[0] = 0.89908;
		endEff[1] = 1.7318;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.045744;
		start[1] = -0.065836;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.019342;
		start[1] = 0.07281;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.050277;
		start[1] = 0.034234;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.089024;
		start[1] = -0.044114;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058285;
		start[1] = -0.095648;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.080163;
		start[1] = -0.039822;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.072837;
		start[1] = 0.036878;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.017422;
		start[1] = 0.094855;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		std::vector<double> endEff (2);
		endEff[0] = -0.22375;
		endEff[1] = -1.2349;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.045744;
		start[1] = -0.065836;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.0054324;
		start[1] = -0.028713;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.019342;
		start[1] = 0.07281;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.050277;
		start[1] = 0.034234;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.089024;
		start[1] = -0.044114;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.058285;
		start[1] = -0.095648;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.080163;
		start[1] = -0.039822;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.072837;
		start[1] = 0.036878;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = 0.017422;
		start[1] = 0.094855;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}
	{
		std::vector<double> start (2);
		start[0] = -0.023807;
		start[1] = -0.00017725;
		std::vector<double> endEff (2);
		endEff[0] = -0.02908;
		endEff[1] = -0.16837;
		solveFor( start, endEff);
	}


  return 0;
}
