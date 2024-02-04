/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LocalizationExample.cpp
 * @brief Simple robot localization example, with three "GPS-like" measurements
 * @author Frank Dellaert
 */

/**
 * A simple 2D pose slam example with "GPS" measurements
 *  - The robot moves forward 2 meter each iteration
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have "GPS-like" measurements implemented with a custom factor
 一个带有“ GPS”测量值的简单2D位姿slam示例
    -机器人每次迭代向前移动2米
    -机器人最初的朝向为X轴正向（水平，在2D中为向右）
    -位姿之间有完整的里程
    -我们通过一个自定义因子实现了“类GPS”的测量
 */

// We will use Pose2 variables (x, y, theta) to represent the robot positions
//我们将使用Pose2变量(x, y, theta)表示机器人位姿
#include <gtsam/geometry/Pose2.h>

// We will use simple integer Keys to refer to the robot poses.
//我们将使用简单的整数key来代表机器人位姿。
#include <gtsam/inference/Key.h>

// As in OdometryExample.cpp, we use a BetweenFactor to model odometry measurements.
//与OdometryExample.cpp中一样，我们使用BetweenFactor对里程计测量进行建模。
#include <gtsam/slam/BetweenFactor.h>

// We add all facors to a Nonlinear Factor Graph, as our factors are nonlinear.
//我们将所有因子添加到非线性因子图中，因为我们的因子是非线性的。
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
/*
GTSAM中的非线性求解器是迭代求解器，这意味着它们会在初始线性化点附近线性化非线性函数，
然后求解线性系统以更新线性化点。这会反复发生，直到求解器收敛到一组一致的变量值为止。
这要求我们为每个变量指定一个初始估计，将其保存在“值”容器中。
*/
#include <gtsam/nonlinear/Values.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
/*
最后，将所有因子添加到因子图中后，我们将需要求解/优化以找到最佳变量值集（最大后验）。
GTSAM包含多个非线性优化器来执行此步骤。在这里，我们将使用标准的Levenberg-Marquardt求解器。
*/
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
//一旦计算出最佳值，我们也可以计算所需变量的边际协方差（边缘分布）
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

// Before we begin the example, we must create a custom unary factor to implement a
// "GPS-like" functionality. Because standard GPS measurements provide information
// only on the position, and not on the orientation, we cannot use a simple prior to
// properly model this measurement.
//在开始示例之前，我们必须创建一个自定义一元因子以实现“类GPS”的功能。
//由于标准GPS测量仅提供有关位置的信息，而不能提供有关方向的信息，
//因此我们无法使用一个简单的先验来正确建模此测量。
//
// The factor will be a unary factor, affect only a single system variable. It will
// also use a standard Gaussian noise model. Hence, we will derive our new factor from
// the NoiseModelFactor1.
//该因子将是一元因子，仅影响单个系统变量。它还将使用标准的高斯噪声模型。
//因此，我们将从NoiseModelFactor1导出新因子。
#include <gtsam/nonlinear/NonlinearFactor.h>

class UnaryFactor: public NoiseModelFactor1<Pose2> { //NoiseModelFactor1为一元因子类

  // The factor will hold a measurement consisting of an (X,Y) location
  // We could this with a Point2 but here we just use two doubles
  //该因子将进行包含(X,Y)位置的测量
  //我们可以用Point2做到这一点，但在这里我们仅使用两个double型数据
  double mx_, my_;

public:
  /// shorthand for a smart pointer to a factor
  ///指向因子的智能指针的简写
  typedef boost::shared_ptr<UnaryFactor> shared_ptr;

  // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
  //构造函数需要变量的key，(X, Y)测量值和噪声模型
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {} //NoiseModelFactor1<Pose2>(model, j)是带有变量key和噪声模型的基础类

  virtual ~UnaryFactor() {}

  // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
  // The first is the 'evaluateError' function. This function implements the desired measurement
  // function, returning a vector of errors when evaluated at the provided variable value. It
  // must also calculate the Jacobians for this measurement function, if requested.
  //使用NoiseModelFactor1基类，必须重写两个函数。第一个是'evaluateError'函数。
  //此函数实现所需的测量功能，当评估所提供变量值时返回误差向量。
  //如果需要，它还必须计算此测量函数的雅可比矩阵。
  //q是Pose2的指针
  //H是可选的雅可比矩阵的指针，它使用了boost::optional并且默认为空指针
  Vector evaluateError(const Pose2& q, boost::optional<Matrix&> H = boost::none) const
  {
    // The measurement function for a GPS-like measurement is simple:
    // error_x = pose.x - measurement.x
    // error_y = pose.y - measurement.y
    // Consequently, the Jacobians are:
    // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [1 0 0]
    // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
	//Jacobians
	// note that use boost optional like a pointer
	// only calculate jacobian matrix when non-null pointer exists
    if (H) (*H) = (Matrix(2,3) << 1.0,0.0,0.0, 0.0,1.0,0.0).finished();
	// return error vector
    return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
  }

  // The second is a 'clone' function that allows the factor to be copied. Under most
  // circumstances, the following code that employs the default copy constructor should
  // work fine.
  //第二个是'clone'函数，它允许因子被复制。在大多数情况下，采用默认副本构造函数的以下代码应该可以正常工作。
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

  // Additionally, we encourage you the use of unit testing your custom factors,
  // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
  // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.
  //此外，我们建议您单独使用测试您的自定义因子（正如所有GTSAM因子一样），
  //在过程中您需要等值并打印，以满足Testable.h中定义的GTSAM_CONCEPT_TESTABLE_INST(T)，但下面的部分不需要这些。

}; // UnaryFactor


int main(int argc, char** argv) {

  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add odometry factors
  // For simplicity, we will use the same noise model for each odometry factor
  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  // Create odometry (Between) factors between consecutive poses
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2.0, 0.0, 0.0), odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2.0, 0.0, 0.0), odometryNoise);

  // 2b. Add "GPS-like" measurements
  // We will use our custom UnaryFactor for this.
  noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y
  graph.emplace_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise);
  graph.emplace_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise);
  graph.emplace_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise);
  graph.print("\nFactor Graph:\n"); // print

  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));
  initialEstimate.print("\nInitial Estimate:\n"); // print

  // 4. Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

  return 0;
}
