#pragma once

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include <g2o/core/robust_kernel_factory.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam3d/g2o_types_slam3d_api.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/isometry3d_gradients.h"


#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/se3_ops.h"
#include "g2o/types/sba/types_sba.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <iostream>
#include <cmath>

using namespace std;
using namespace g2o;

class EdgeSE3_normfixed : public BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    double t_norm;

    EdgeSE3_normfixed(double tnorm);

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
      const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
      const VertexSE3Expmap* v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);

      SE3Quat C(_measurement);
      SE3Quat error_= v2->estimate().inverse()*C*v1->estimate();
      _error = error_.log();


      cout << "_error "<<endl<<_error<<endl;
      cout << "_error.matrix() "<<endl<<_error.matrix()<<endl;
      cout << "C "<<endl<<C<<endl;
      
      double t_error = 1000*fabs(t_norm - (error_.translation()).norm());
      SE3Quat i_quat;
      double r_error = 1000000000*( error_.rotation().matrix() - i_quat.rotation().matrix()).norm();

      _error[0] =_error[1] =_error[2] = r_error; 
      _error[3] =_error[4] =_error[5] = t_error; 

    }
};