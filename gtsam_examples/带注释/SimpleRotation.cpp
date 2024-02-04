/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SimpleRotation.cpp
 * @brief This is a super-simple example of optimizing a single rotation according to a single prior
 * @date Jul 1, 2010
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

  /**
   * This example will perform a relatively trivial optimization on
   * a single variable with a single factor.
   *此示例将对具有单个因子的单个变量执行相对轻微的优化。
   */

// In this example, a 2D rotation will be used as the variable of interest
//在此示例中，将使用2D旋转作为目标变量
#include <gtsam/geometry/Rot2.h>

// Each variable in the system (poses) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use symbols
//系统中的每个变量（位姿）都必须使用唯一的key进行标识，我们可以使用简单的整数key(1, 2, 3, ...)或符号(X1, X2, L1).
//此处将使用符号
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// We will apply a simple prior on the rotation. We do so via the `addPrior` convenience
// method in NonlinearFactorGraph.
//在GTSAM中，量测函数表示为“因子”。库中提供了一些常见的因子来解决机器人技术/ SLAM /BA问题。
//我们将一个简单的先验应用到旋转上。我们通过NonlinearFactorGraph中的`addPrior`便捷方法来实现。

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
//创建因子后，我们会将其添加到因子图中。由于我们使用的因子是非线性因子，因此我们将需要一个非线性因子图。
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
//GTSAM中的非线性求解器是迭代求解器，这意味着它们会在初始线性化点附近线性化非线性函数，
//然后求解线性系统以更新线性化点。这会反复发生，直到求解器收敛到一组一致的变量值为止。
//这要求我们为保存在“值”容器中的每个变量指定一个初始猜测。
#include <gtsam/nonlinear/Values.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
//最后，将所有因子添加到因子图中后，我们将需要求解/优化以找到最佳变量值集（最大后验）。
//GTSAM包含多个非线性优化器来执行此步骤。在这里，我们将使用标准的Levenberg-Marquardt求解器。
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


using namespace std;
using namespace gtsam;

const double degree = M_PI / 180;

int main() {

  /**
   *    Step 1: Create a factor to express a unary constraint
   * The "prior" in this case is the measurement from a sensor,
   * with a model of the noise on the measurement.
   步骤1：创建一个表达一元约束的因子
   在这个例子中，“先验”是来自传感器的测量，并带有测量噪声的模型。
   *
   * The "Key" created here is a label used to associate parts of the
   * state (stored in "RotValues") with particular factors.  They require
   * an index to allow for lookup, and should be unique.
   此处创建的“Key”是用于将状态的某些部分（存储在“ RotValues”中）与特定因素相关联的标签。
   它们需要一个索引以允许查找，并且应该是唯一的。
   *
   * In general, creating a factor requires:
   *  - A key or set of keys labeling the variables that are acted upon
   *  - A measurement value
   *  - A measurement model with the correct dimensionality for the factor
   通常，创建一个因子需要：
    -标记要作用的变量的一个key或一组key
    -一个测量值
    -具有正确维度的测量模型
   */
  Rot2 prior = Rot2::fromAngle(30 * degree);
  prior.print("goal angle");
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(1, 1 * degree);//各项同性噪声
  Symbol key('x',1);

  /**
   *    Step 2: Create a graph container and add the factor to it
   * Before optimizing, all factors need to be added to a Graph container,
   * which provides the necessary top-level functionality for defining a
   * system of constraints.
   步骤2：创建一个graph容器并将因子添加到其中
  优化之前，所有因素都需要添加到Graph容器中，该容器提供了用于定义约束系统的必要顶层功能。
   *
   * In this case, there is only one factor, but in a practical scenario,
   * many more factors would be added.
   在本例中，只有一个因子，但是在实际情况下，将添加更多因子。
   */
  NonlinearFactorGraph graph; //定义因子图
  //PriorFactor<Rot2> factor(key, prior, model);
  graph.addPrior(key, prior, model); //为节点x1添加先验边prior，model为噪声
  graph.print("full graph");

  /**
   *    Step 3: Create an initial estimate
   * An initial estimate of the solution for the system is necessary to
   * start optimization.  This system state is the "RotValues" structure,
   * which is similar in structure to a STL map, in that it maps
   * keys (the label created in step 1) to specific values.
   步骤3：建立初始估计
  要开始优化，必须对系统的解进行初步估算。此系统的状态是“ RotValues”结构，
  该结构与STL映射的结构相似，因为它会将key（在步骤1中创建的标签）映射到特定值。
   *
   * The initial estimate provided to optimization will be used as
   * a linearization point for optimization, so it is important that
   * all of the variables in the graph have a corresponding value in
   * this structure.
   提供给优化的初始估计值将用作优化的线性化点，因此，重要的是，图中的所有变量在此结构中均具有对应的值。
   *
   * The interface to all RotValues types is the same, it only depends
   * on the type of key used to find the appropriate value map if there
   * are multiple types of variables.
   所有RotValues类型的接口都是相同的，如果存在多种类型的变量，则仅取决于用于查找适当的值的映射的key的类型。
   */
  Values initial;
  initial.insert(key, Rot2::fromAngle(20 * degree));//赋初值
  initial.print("initial estimate");

  /**
   *    Step 4: Optimize
   * After formulating the problem with a graph of constraints
   * and an initial estimate, executing optimization is as simple
   * as calling a general optimization function with the graph and
   * initial estimate.  This will yield a new RotValues structure
   * with the final state of the optimization.
   第4步：优化
   在用约束图和初始估计表述问题之后，执行优化就是使用图和初始估计调用通用优化函数。
   这将产生具有优化最终状态的新的RotValues。
   */
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("final result");

  return 0;
}
