#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "sophus/so3.hpp"
#include "liw/lidarFactor.h"
#include "liw/poseParameterization.h"
#include "tools/tool_color_printf.hpp"

using namespace std;

/*
 *   参考clic写的测试自动求导和解析求导的demo文件
 *   需要注意的是，最好参数都是vector的形式会比较好
 *   确保参差计算方式一模一样才可以
 */
namespace Eigen
{
     template <typename T>
     using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

     template <typename T>
     using aligned_deque = std::deque<T, Eigen::aligned_allocator<T>>;

     template <typename K, typename V>
     using aligned_map = std::map<K, V, std::less<K>,
                                  Eigen::aligned_allocator<std::pair<K const, V>>>;

     template <typename K, typename V>
     using aligned_unordered_map =
         std::unordered_map<K, V, std::hash<K>, std::equal_to<K>,
                            Eigen::aligned_allocator<std::pair<K const, V>>>;
}
class FactorTest
{
public:
     FactorTest()
     {
          parameterization = new RotationParameterization();
          autodiff_parameterization = new ceres::EigenQuaternionParameterization();

          solver_options_.max_num_iterations = 1;
          solver_options_.num_threads = 1;
          solver_options_.minimizer_progress_to_stdout = false;
          solver_options_.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

          double laser_point_cov = 1;
          CT_ICP::LidarPlaneNormFactor::t_il = Eigen::Vector3d::Zero();
          CT_ICP::LidarPlaneNormFactor::q_il = Eigen::Quaterniond::Identity();
          CT_ICP::LidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);
     }
     ~FactorTest() {}

     void TestPointPlaneFactorAutoDiff(Eigen::aligned_map<double *, Eigen::MatrixXd> &jacobians)
     {
          Eigen::Vector3d norm_vector(0.3, 1.5, -2);
          norm_vector.normalize();
          Eigen::Vector3d vector_neig(1, 3, 5);
          double norm_offset = -norm_vector.dot(vector_neig);
          Eigen::Vector3d point_end(10, 12, 14);
          double weight = 1;

          Eigen::Quaterniond end_quat = Eigen::Quaterniond(0.2, 0.6, 1.3, -0.9);
          end_quat.normalize();
          std::cout << "normal: " << norm_vector.transpose() << std::endl;
          Eigen::Vector3d end_t(11, 13, 15);

          auto *cost_function =
              CT_ICP::PointToPlaneFunctor::Create(vector_neig, point_end, norm_vector, weight);

          ceres::Problem problem(problem_options_);
          std::vector<double *> vec;
          vec.emplace_back(end_t.data());
          vec.emplace_back(end_t.data());
          problem.AddParameterBlock(&end_quat.x(), 4, autodiff_parameterization);
          problem.AddParameterBlock(&end_t.x(), 3);

          problem.AddResidualBlock(cost_function, nullptr, &end_t.x(), &end_quat.x());

          GetJacobian(vec, problem, cost_function->num_residuals(), jacobians);
     }

     void TestPointPlaneFactorAnalytic(Eigen::aligned_map<double *, Eigen::MatrixXd> &jacobians)
     {
          Eigen::Vector3d norm_vector(0.3, 1.5, -2);
          norm_vector.normalize();
          Eigen::Vector3d vector_neig(1, 3, 5);
          double norm_offset = -norm_vector.dot(vector_neig);
          Eigen::Vector3d point_end(10, 12, 14);
          double weight = 1;

          Eigen::Quaterniond end_quat = Eigen::Quaterniond(0.2, 0.6, 1.3, -0.9);
          end_quat.normalize();
          Eigen::Vector3d end_t(11, 13, 15);

          CT_ICP::LidarPlaneNormFactor *cost_function =
              new CT_ICP::LidarPlaneNormFactor(point_end, norm_vector, norm_offset, weight);

          ceres::Problem problem(problem_options_);
          std::vector<double *> vec;
          vec.emplace_back(end_t.data());
          vec.emplace_back(end_t.data());
          problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
          problem.AddParameterBlock(&end_t.x(), 3);

          problem.AddResidualBlock(cost_function, nullptr, &end_t.x(), &end_quat.x());

          GetJacobian(vec, problem, cost_function->num_residuals(), jacobians);
     }

     void CheckJacobian(
         std::string factor_descri,
         Eigen::aligned_map<double *, Eigen::MatrixXd> &jacobs_automatic,
         Eigen::aligned_map<double *, Eigen::MatrixXd> &jacobs_analytic,
         const std::vector<double *> &parameters = {},
         const std::vector<std::string> &param_descri = {})
     {
          bool check_pass = true;
          size_t cnt = 0;

          std::map<double *, int> parameters_map;
          if (!parameters.empty() && !param_descri.empty())
          {
               for (int i = 0; i < (int)parameters.size(); ++i)
                    parameters_map[parameters.at(i)] = i;
          }

          for (auto const &v : jacobs_analytic)
          {
               auto iter = jacobs_automatic.find(v.first);
               if (jacobs_automatic.end() != iter)
               {
                    Eigen::MatrixXd diff = iter->second - v.second;
                    if (diff.cwiseAbs().maxCoeff() > 1e-6)
                    {
                         //   按内存地址大小的距离，不是误差项中参数添加顺序的索引
                         int idx;
                         if (parameters.empty())
                         {
                              idx = std::distance(jacobs_automatic.begin(), iter);
                         }
                         else
                         {
                              idx = parameters_map.at(iter->first);
                         }

                         std::cout << std::setiosflags(ios::fixed) << std::setw(15)
                                   << std::setprecision(15);

                         if (parameters.empty())
                              cout << " ======== index " << idx << " ========\n";
                         else
                              cout << " ======== index " << idx << " " << param_descri.at(idx)
                                   << " ========\n";
                         cout << "auto diff\n"
                              << iter->second << "\nanalytic:\n"
                              << v.second << endl;
                         check_pass = false;

                         std::cout << std::setiosflags(ios::fixed) << std::setw(3)
                                   << std::setprecision(3);
                    }
                    else
                    {
                         cnt++;
                    }
               }
          }

          cout << factor_descri << " check [" << cnt << "/" << jacobs_analytic.size()
               << "] jacobians ok.\n\n";
          if (!check_pass)
          {
               cout << ANSI_COLOR_RED << factor_descri << " has some problems.\n"
                    << ANSI_COLOR_RESET;
          }
     }

     void GetJacobian(std::vector<double *> param_vec, ceres::Problem &problem,
                      int residual_num,
                      Eigen::aligned_map<double *, Eigen::MatrixXd> &jacobians)
     {
          double cost = 0.0;
          ceres::CRSMatrix J;
          std::vector<double> residuals;
          problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residuals,
                           nullptr, &J);

          Eigen::MatrixXd dense_jacobian(J.num_rows, J.num_cols);
          dense_jacobian.setZero();
          for (int r = 0; r < J.num_rows; ++r)
          {
               for (int idx = J.rows[r]; idx < J.rows[r + 1]; ++idx)
               {
                    const int c = J.cols[idx];
                    dense_jacobian(r, c) = J.values[idx];
               }
          }

          int cnt = 0;
          std::string right_descri = ")= ";
          if (residual_num > 1)
               right_descri += "\n";
          for (size_t i = 0; i < param_vec.size(); i++)
          {
               int local_size = problem.ParameterBlockLocalSize(param_vec.at(i));
               // if (i == 1)
               //      local_size = 4;
               Eigen::MatrixXd jacob = Eigen::MatrixXd::Zero(residual_num, local_size);
               jacob = dense_jacobian.block(0, cnt, residual_num, local_size);
               cnt += local_size;

               jacobians[param_vec.at(i)] = jacob;
               // jacobians[param_vec.at(i)] = jacob;
               cout << "J(" << std::setw(2) << i << right_descri << jacob << std::endl;
          }

          std::cout << "cost = " << cost << "; redisual: ";
          for (auto &r : residuals)
               std::cout << r << ", ";
          std::cout << "\n";

          // std::cout << "J = (" << J.num_rows << ", " << J.num_cols
          //           << ") with non - zero value; \n ";
          // for (int i = 0; i < J.num_rows; i++) {
          //   for (int j = J.rows[i]; j < J.rows[i + 1]; j++) {
          //     std::cout << "J(" << std::setw(2) << i << "," << std::setw(2)
          //               << J.cols[j] << ") = " << std::setw(10)
          //               << std::setiosflags(ios::fixed) << std::setprecision(3)
          //               << J.values[j] << "; ";
          //   }
          //   cout << endl;
          // }
     }

     void GetJacobian(std::vector<double *> param_vec,
                      const ceres::CostFunction *cost_function)
     {
          int num_residuals = cost_function->num_residuals();
          Eigen::MatrixXd residuals;
          residuals.setZero(num_residuals, 1);

          std::vector<double *> J_vec;
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
              Jacob[cost_function->parameter_block_sizes().size()];
          size_t cnt = 0;
          for (auto const v : cost_function->parameter_block_sizes())
          {
               Jacob[cnt].setZero(num_residuals, v);
               J_vec.emplace_back(Jacob[cnt++].data());
          }

          cost_function->Evaluate(param_vec.data(), residuals.data(), J_vec.data());
          cout << "residuals = " << residuals.transpose() << endl;

          for (size_t i = 0; i < J_vec.size(); ++i)
          {
               if (num_residuals == 1)
                    cout << "J[" << i << "] = " << Jacob[i] << endl;
               else
                    cout << "J[" << i << "] = \n"
                         << Jacob[i] << endl;
          }
     }

private:
     ceres::LocalParameterization *parameterization;
     ceres::LocalParameterization *autodiff_parameterization;
     ceres::Problem::Options problem_options_;
     ceres::Solver::Options solver_options_;
};

int main()
{
     FactorTest factor_test;
     std::cout << std::setiosflags(ios::fixed) << std::setprecision(3);
     std::cout << "\n ===== TEST ===== \n\n";

     std::string descri =
         "test point plane factor";
     Eigen::aligned_map<double *, Eigen::MatrixXd> jacobs_automatic;
     Eigen::aligned_map<double *, Eigen::MatrixXd> jacobs_analytic;

     factor_test.TestPointPlaneFactorAutoDiff(jacobs_automatic);
     factor_test.TestPointPlaneFactorAnalytic(jacobs_analytic);
     factor_test.CheckJacobian(descri, jacobs_automatic, jacobs_analytic);

     return 0;
}