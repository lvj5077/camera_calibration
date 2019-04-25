// for std
#include <stdio.h>
#include <iostream>
// for opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include <boost/concept_check.hpp>

// for g2o
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

#include <opencv2/core/eigen.hpp>

#include "myG2Oedge.h"
#include "edge_expmap_t_norm.h"

using namespace std;
using namespace cv;
using namespace g2o;

int main(int argc, char const **argv)
{
  
  vector< vector< Point3f > > object_points;
  vector< vector< Point2f > > image_points;
  vector< Point2f > corners;

  Mat img, gray;
  std::vector<Mat> imgs;
  int board_width, board_height, num_imgs;
  float square_size;

  img = imread("/Users/lingqiujin/Data/slider_01_11_2019/line20_01/Average/color/60.png", CV_LOAD_IMAGE_COLOR);
  imgs.push_back(img);
  img = imread("/Users/lingqiujin/Data/slider_01_11_2019/line20_01/Average/color/110.png", CV_LOAD_IMAGE_COLOR);
  imgs.push_back(img);
  img = imread("/Users/lingqiujin/Data/slider_01_11_2019/line20_01/Average/color/160.png", CV_LOAD_IMAGE_COLOR);
  imgs.push_back(img);
  img = imread("/Users/lingqiujin/Data/slider_01_11_2019/line20_01/Average/color/210.png", CV_LOAD_IMAGE_COLOR);
  imgs.push_back(img);
  img = imread("/Users/lingqiujin/Data/slider_01_11_2019/line20_01/Average/color/260.png", CV_LOAD_IMAGE_COLOR);
  imgs.push_back(img);
  double cx = 322.5009460449219;
  double cy = 242.32693481445312;
  double fx = 615.2476806640625;
  double fy = 615.4488525390625; 

  square_size = 50.0/1000; // mm to m
  bool found = false;
  board_width = 5;
  board_height = 4;
  Size board_size = Size(5, 4);
  vector< Point3f > obj;
  for (int i = 0; i < board_height; i++)
    for (int j = 0; j < board_width; j++)
      obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

  for (int i=0;i<imgs.size();i++){
    cv::cvtColor(imgs[i], gray, CV_BGR2GRAY);
    found = cv::findChessboardCorners(imgs[i], board_size, corners,
                                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    if (found) {
      cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

      // drawChessboardCorners(gray, board_size, corners, true);
      // cv::imshow( "checkerboard", gray );
      // cv::waitKey( 0 );  

      // cout << "corners"<<endl<< corners <<endl;

      image_points.push_back(corners);
      object_points.push_back(obj);
    }
  }

  // Mat rvec,tvec;
  double camera_matrix_data[3][3] = {
      {fx, 0, cx},
      {0, fy, cy},
      {0, 0, 1}
  };
  cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
  // solvePnP(obj, image_points[0], cameraMatrix, Mat(), rvec, tvec); 
  // cout << tvec<<endl;
  // solvePnP(obj, image_points[1], cameraMatrix, Mat(), rvec, tvec); 
  // cout << tvec<<endl;
  // solvePnP(obj, image_points[2], cameraMatrix, Mat(), rvec, tvec); 
  // cout << tvec<<endl;
//=========================================================================================================================
  g2o::SparseOptimizer optimizer;
    
  typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
  typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 
  std::unique_ptr<SlamLinearSolver> linearSolver ( new SlamLinearSolver());
  linearSolver->setBlockOrdering( false );
  std::unique_ptr<SlamBlockSolver> blockSolver ( new SlamBlockSolver ( std::move(linearSolver)));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(blockSolver) );
  optimizer.setAlgorithm( solver );
  optimizer.setVerbose( true );


  // all pseudo 3d points are fixed
  for ( size_t i=0; i<obj.size(); i++ )
  {
      g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
      v->setFixed( true ); 
      v->setId( i );
      
      double z = (obj[i]).z; // avoid zero depth
      double x = (obj[i]).x; 
      double y = (obj[i]).y; 
      v->setMarginalized(true);
      v->setEstimate( Eigen::Vector3d(x,y,z) );
      optimizer.addVertex( v );
  }

  // camera pose || reference is points on wall which is also fixed 
  for ( int i=0; i<imgs.size(); i++ )
  {
    // g2o::VertexSE3* v = new g2o::VertexSE3();
    g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap(); // camera pose
    v->setId( obj.size() + i );

    Mat R;
    Mat rvec,tvec;
    solvePnP(obj, image_points[i], cameraMatrix, Mat(), rvec, tvec); 
    cv::Rodrigues ( rvec, R );
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
          R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
          R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    v->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            Eigen::Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
                        ) );
    // v->setEstimate( g2o::SE3Quat() );
    optimizer.addVertex( v );
  }


  g2o::CameraParameters* camera = new g2o::CameraParameters( (fx+fy)/2, Eigen::Vector2d(cx, cy), 0 );
  camera->setId(0);
  optimizer.addParameter( camera );


  for (int idx =0;idx< imgs.size();idx++){
    for ( size_t i=0; i<(image_points[idx]).size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->vertices() [0] = optimizer.vertex( i );
        edge->vertices() [1] = optimizer.vertex( obj.size()+idx);
        edge->setMeasurement( Eigen::Vector2d(((image_points[idx])[i]).x, ((image_points[idx])[i]).y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
    }
  }

//=====================================add code wheel constarin===============================================================
  // for ( int i=0; i<imgs.size(); i++ )
  // {
  //     g2o::VertexSE3* v = new g2o::VertexSE3();
  //     v->setId(i+imgs.size()+obj.size()+1);
  //     v->setEstimate( g2o::SE3Quat() );
  //     optimizer.addVertex( v );

  //     g2o::EdgeSE3* edge_CtoW = new g2o::EdgeSE3();
  //     edge_CtoW->vertices() [0] = optimizer.vertex( i+1 );
  //     edge_CtoW->vertices() [1] = optimizer.vertex( i+imgs.size()+obj.size()+1 );
  //     Eigen::Matrix<double, 6, 6> information_CtoW = Eigen::Matrix< double, 6,6 >::Identity();
  //     information_CtoW(0,0) = information_CtoW(1,1) = information_CtoW(2,2) = 100;
  //     information_CtoW(3,3) = information_CtoW(4,4) = information_CtoW(5,5) = 10000000000000000; // 不考虑translation了
  //     edge_CtoW->setInformation( information_CtoW );
  //     edge_CtoW->setMeasurement( g2o::SE3Quat() ); ////需要求解
  //     optimizer.addEdge(edge_CtoW);
  // }


  Eigen::Isometry3d T_prior =  Eigen::Isometry3d::Identity();
  Mat cvR = cv::Mat::eye(3,3,CV_64F);
  Eigen::Matrix3d r_eigen;
  for ( int i=0; i<3; i++ )
      for ( int j=0; j<3; j++ ) 
          r_eigen(i,j) = cvR.at<double>(i,j);

  Eigen::AngleAxisd angle(r_eigen);
  T_prior = angle;
  T_prior(0,3) = 0.0; 
  T_prior(1,3) = 0.0; 
  T_prior(2,3) = 0.50;

  double t_norm = 0.5;
  for (int idx =1;idx< imgs.size();idx++){
    EdgeSE3_normfixed* edge_fixR = new EdgeSE3_normfixed(t_norm);
    // EdgeSE3Expmap* edge_fixR = new EdgeSE3Expmap();
    edge_fixR->vertices() [0] = optimizer.vertex( idx+obj.size()-1 );
    edge_fixR->vertices() [1] = optimizer.vertex( idx+obj.size());
    Eigen::Matrix<double, 6, 6> information_fixR = 1e7*Eigen::Matrix< double, 6,6 >::Identity();
    edge_fixR->setInformation( information_fixR );
    optimizer.addEdge(edge_fixR);


    // g2o::EdgeSE3ExpmapNorm* edge_fix = new g2o::EdgeSE3ExpmapNorm(t_norm); 
    // edge_fix->vertices() [0] = optimizer.vertex( idx+obj.size() -1);
    // edge_fix->vertices() [1] = optimizer.vertex( idx+obj.size());
    // Eigen::Matrix<double, 4, 4> information_fix = 1e7*Eigen::Matrix< double, 4,4 >::Identity();
    // edge_fix->setInformation( information_fix );
    // optimizer.addEdge(edge_fix);

  }

  optimizer.save("./result_before.g2o");
  optimizer.initializeOptimization();
  optimizer.optimize(100);
  optimizer.save("./result_after.g2o");

  // g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>( optimizer.vertex(1) );
  // Eigen::Isometry3d pose = v->estimate();
  // cout<<"Pose="<<endl<<pose.matrix()<<endl;


  // Mat testTTT;
  // eigen2cv(pose.matrix(),testTTT);
  // cout << testTTT<<endl;
    
    g2o::VertexSE3Expmap* v0 = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(20) );
    // g2o::VertexSE3Expmap* v1 = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(21) );
    // g2o::VertexSE3Expmap* v2 = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(22) );
    Eigen::Isometry3d p1 = v0->estimate();
    // Eigen::Isometry3d p2 = v1->estimate();
    // Eigen::Isometry3d p3 = v2->estimate();

    cout<<"Pose1 ="<<endl<<p1.matrix()<<endl;
    // cout<<"Pose2 ="<<endl<<p2.matrix()<<endl;
    // cout<<"Pose3 ="<<endl<<p3.matrix()<<endl;

    // cout<<"Pose12 ="<<endl<<(p2*(p1.inverse())).matrix()<<endl;
    // cout<<"Pose23 ="<<endl<<(p3*(p2.inverse())).matrix()<<endl;


  for (int idx =1;idx< imgs.size();idx++){
    g2o::VertexSE3Expmap* vi = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex( idx+obj.size()-1 ) );
    g2o::VertexSE3Expmap* vj = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex( idx+obj.size() ) );
    Eigen::Isometry3d pi = vi->estimate();
    Eigen::Isometry3d pj = vj->estimate();
    cout<<"Pose "<<idx-1 <<" to "<< idx << endl<<(pj*(pi.inverse())).matrix()<<endl;

  }

  return 0;
}