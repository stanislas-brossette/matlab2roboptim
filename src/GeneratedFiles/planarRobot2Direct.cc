// compile command: 
// g++ -ggdb3 -I/usr/include/log4cxx `pkg-config --cflags roboptim-core` src/generatedFiles/planarRobot2Direct.cc `pkg-config --libs roboptim-core` -o bin/planarRobot2Direct
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
class planarRobot2Direct : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit planarRobot2Direct (const double& EE_1_1,
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
planarRobot2Direct<T>::planarRobot2Direct (const double& EE_1_1,
			 const double& EE_1_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (2, 2, "Direct EE position constraints planarRobot2"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2)
{}

template <typename T>
void
planarRobot2Direct<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];

  
	result[0] = cos(q_01) - 1.0*EE_1_1 + cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02);
	result[1] = sin(q_01) - 1.0*EE_1_2 + cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01);
}

template <typename T>
void
planarRobot2Direct<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
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
		case 1: 
			 grad[0] = cos(q_01) + cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02); 
			 grad[1] = cos(q_01)*cos(q_02) - 1.0*sin(q_01)*sin(q_02); 
			 break;
    default:
      assert (0 && "should never happen");
    }
}


int main ()
{
  double EE_1_1 = 0.96144;double EE_1_2 = 0.78366;

  boost::shared_ptr<planarRobot2Direct<roboptim::EigenMatrixDense> > endEffCstr =
    boost::make_shared<planarRobot2Direct<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2);

  // Create Null Cost Function
  roboptim::Function::vector_t offset (2);
  for(std::size_t i=0; i<2; ++i)
    offset[i] = 0.;
  roboptim::ConstantFunction cost(offset);

  //Create problem
  solver_t::problem_t pb (cost);

  // Set bounds for all optimization parameters
  pb.argumentBounds ()[0] = roboptim::Function::makeInterval (-3.15, 3.15);
	pb.argumentBounds ()[1] = roboptim::Function::makeInterval (-3.15, 3.15);

  // Set the starting point.
  roboptim::Function::vector_t start (2);
  start[0] = 0.48129;
	start[1] = 0.82058;

  // Create constraints.
  planarRobot2Direct<roboptim::EigenMatrixDense>::intervals_t bounds;
  solver_t::problem_t::scales_t scales;
  bounds.push_back(roboptim::Function::makeInterval (0., 0.));
	bounds.push_back(roboptim::Function::makeInterval (0., 0.));
  scales.push_back(1.);
	scales.push_back(1.);
  pb.addConstraint ( endEffCstr, bounds, scales);

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
