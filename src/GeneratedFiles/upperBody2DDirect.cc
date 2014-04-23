// compile command: 
// g++ -ggdb3 -I/usr/include/log4cxx `pkg-config --cflags roboptim-core` src/generatedFiles/upperBody2DDirect.cc `pkg-config --libs roboptim-core` -o bin/upperBody2DDirect
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
class upperBody2DDirect : public roboptim::GenericDifferentiableFunction<T>
{
public:
  ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
  (roboptim::GenericDifferentiableFunction<T>);
  
  explicit upperBody2DDirect (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ();

  void
  impl_compute (result_t& result, const argument_t& x) const throw ();
  void
  impl_gradient (gradient_t& grad, const argument_t& x, size_type)
  const throw ();

private:
  double EE_1_1;
	double EE_1_2;
	double EE_2_1;
	double EE_2_2;
};

template <typename T>
upperBody2DDirect<T>::upperBody2DDirect (const double& EE_1_1,
			 const double& EE_1_2,
			 const double& EE_2_1,
			 const double& EE_2_2) throw ()
  : roboptim::GenericDifferentiableFunction<T>
    (6, 4, "Direct EE position constraints upperBody2D"),
    EE_1_1 (EE_1_1),
		EE_1_2 (EE_1_2),
		EE_2_1 (EE_2_1),
		EE_2_2 (EE_2_2)
{}

template <typename T>
void
upperBody2DDirect<T>::impl_compute (result_t& result, const argument_t& x)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];

  
	result[0] = (sin(q_01)*sin(q_02))/4 - sin(q_01)/2 - (cos(q_01)*cos(q_02))/4 - (cos(q_01)*sin(q_02))/2 - (cos(q_02)*sin(q_01))/2 - EE_1_1 - (3*cos(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*sin(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 - (cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4 - (sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4;
	result[1] = cos(q_01)/2 - EE_1_2 + (cos(q_01)*cos(q_02))/2 - (cos(q_01)*sin(q_02))/4 - (cos(q_02)*sin(q_01))/4 - (sin(q_01)*sin(q_02))/2 + (3*cos(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 - (3*sin(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 + (cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4 - (sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4;
	result[2] = (cos(q_01)*cos(q_02))/4 - sin(q_01)/2 - EE_2_1 - (cos(q_01)*sin(q_02))/2 - (cos(q_02)*sin(q_01))/2 - (sin(q_01)*sin(q_02))/4 - (3*cos(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*sin(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 - (cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4 - (sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4;
	result[3] = cos(q_01)/2 - EE_2_2 + (cos(q_01)*cos(q_02))/2 + (cos(q_01)*sin(q_02))/4 + (cos(q_02)*sin(q_01))/4 - (sin(q_01)*sin(q_02))/2 + (3*cos(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 - (3*sin(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 + (cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4 - (sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4;
}

template <typename T>
void
upperBody2DDirect<T>::impl_gradient (gradient_t& grad, const argument_t& x, size_type id)
  const throw ()
{
  const double& q_01 = x[0];
	const double& q_02 = x[1];
	const double& q_03 = x[2];
	const double& q_04 = x[3];
	const double& q_05 = x[4];
	const double& q_06 = x[5];

  switch (id)
    {
      
		case 0: 
			 grad[0] = (cos(q_01)*sin(q_02))/4 - (cos(q_01)*cos(q_02))/2 - cos(q_01)/2 + (cos(q_02)*sin(q_01))/4 + (sin(q_01)*sin(q_02))/2 - (3*cos(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 + (3*sin(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4 + (sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4; 
			 grad[1] = (cos(q_01)*sin(q_02))/4 - (cos(q_01)*cos(q_02))/2 + (cos(q_02)*sin(q_01))/4 + (sin(q_01)*sin(q_02))/2 - (3*cos(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 + (3*sin(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4 + (sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4; 
			 grad[2] = (3*sin(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*cos(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 - (cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4 + (sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4; 
			 grad[3] = (3*sin(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*cos(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10; 
			 grad[4] = 0; 
			 grad[5] = 0; 
			 break;
		case 1: 
			 grad[0] = (sin(q_01)*sin(q_02))/4 - (cos(q_01)*cos(q_02))/4 - (cos(q_01)*sin(q_02))/2 - (cos(q_02)*sin(q_01))/2 - sin(q_01)/2 - (3*cos(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*sin(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 - (cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4 - (sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4; 
			 grad[1] = (sin(q_01)*sin(q_02))/4 - (cos(q_01)*sin(q_02))/2 - (cos(q_02)*sin(q_01))/2 - (cos(q_01)*cos(q_02))/4 - (3*cos(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*sin(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 - (cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4 - (sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4; 
			 grad[2] = - (3*cos(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*sin(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 - (cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4 - (sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4; 
			 grad[3] = - (3*cos(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*sin(q_04)*(cos(pi/2 + q_03)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(pi/2 + q_03)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10; 
			 grad[4] = 0; 
			 grad[5] = 0; 
			 break;
		case 2: 
			 grad[0] = (sin(q_01)*sin(q_02))/2 - (cos(q_01)*cos(q_02))/2 - (cos(q_01)*sin(q_02))/4 - (cos(q_02)*sin(q_01))/4 - cos(q_01)/2 - (3*cos(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 + (3*sin(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4 + (sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4; 
			 grad[1] = (sin(q_01)*sin(q_02))/2 - (cos(q_01)*sin(q_02))/4 - (cos(q_02)*sin(q_01))/4 - (cos(q_01)*cos(q_02))/2 - (3*cos(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 + (3*sin(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4 + (sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4; 
			 grad[2] = 0; 
			 grad[3] = 0; 
			 grad[4] = (3*sin(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*cos(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 - (cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4 + (sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4; 
			 grad[5] = (3*sin(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*cos(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10; 
			 break;
		case 3: 
			 grad[0] = (cos(q_01)*cos(q_02))/4 - sin(q_01)/2 - (cos(q_01)*sin(q_02))/2 - (cos(q_02)*sin(q_01))/2 - (sin(q_01)*sin(q_02))/4 - (3*cos(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*sin(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 - (cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4 - (sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4; 
			 grad[1] = (cos(q_01)*cos(q_02))/4 - (cos(q_01)*sin(q_02))/2 - (cos(q_02)*sin(q_01))/2 - (sin(q_01)*sin(q_02))/4 - (3*cos(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*sin(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 - (cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4 - (sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4; 
			 grad[2] = 0; 
			 grad[3] = 0; 
			 grad[4] = - (3*cos(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*sin(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10 - (cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)))/4 - (sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)))/4; 
			 grad[5] = - (3*cos(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01)) + sin(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02))))/10 - (3*sin(q_06)*(cos(q_05 - pi/2)*(cos(q_01)*cos(q_02) - sin(q_01)*sin(q_02)) - sin(q_05 - pi/2)*(cos(q_01)*sin(q_02) + cos(q_02)*sin(q_01))))/10; 
			 break;
    default:
      assert (0 && "should never happen");
    }
}


int main ()
{
  double EE_1_1 = 0.38676;double EE_1_2 = 0.33638;double EE_2_1 = 0.93115;double EE_2_2 = 0.49365;

  boost::shared_ptr<upperBody2DDirect<roboptim::EigenMatrixDense> > endEffCstr =
    boost::make_shared<upperBody2DDirect<roboptim::EigenMatrixDense> > (EE_1_1, EE_1_2, EE_2_1, EE_2_2);

  // Create Null Cost Function
  roboptim::Function::vector_t offset (6);
  for(std::size_t i=0; i<6; ++i)
    offset[i] = 0.;
  roboptim::ConstantFunction cost(offset);

  //Create problem
  solver_t::problem_t pb (cost);

  // Set bounds for all optimization parameters
  pb.argumentBounds ()[0] = roboptim::Function::makeInterval (-3.15, 3.15);
	pb.argumentBounds ()[1] = roboptim::Function::makeInterval (-3.15, 3.15);
	pb.argumentBounds ()[2] = roboptim::Function::makeInterval (-3.15, 3.15);
	pb.argumentBounds ()[3] = roboptim::Function::makeInterval (-3.15, 3.15);
	pb.argumentBounds ()[4] = roboptim::Function::makeInterval (-3.15, 3.15);
	pb.argumentBounds ()[5] = roboptim::Function::makeInterval (-3.15, 3.15);

  // Set the starting point.
  roboptim::Function::vector_t start (6);
  start[0] = 0.17283;
	start[1] = 0.084515;
	start[2] = 0.078573;
	start[3] = 0.1658;
	start[4] = 0.91378;
	start[5] = 0.22942;

  // Create constraints.
  upperBody2DDirect<roboptim::EigenMatrixDense>::intervals_t bounds;
  solver_t::problem_t::scales_t scales;
  bounds.push_back(roboptim::Function::makeInterval (0., 0.));
	bounds.push_back(roboptim::Function::makeInterval (0., 0.));
	bounds.push_back(roboptim::Function::makeInterval (0., 0.));
	bounds.push_back(roboptim::Function::makeInterval (0., 0.));
  scales.push_back(1.);
	scales.push_back(1.);
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
