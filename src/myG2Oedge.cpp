#include "myG2Oedge.h"

EdgeSE3_normfixed::EdgeSE3_normfixed(double tnorm) :   
    BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>() {
    t_norm = tnorm;

}
bool EdgeSE3_normfixed::read(std::istream& is) {
  Vector7 meas;
  for (int i=0; i<7; i++)
    is >> meas[i];

  Vector3 t_read = Vector3( meas[4], meas[5], meas[6]);
  t_norm = t_read.norm();
  SE3Quat cam2world;
  cam2world.fromVector(meas);
  setMeasurement(cam2world.inverse());
  //TODO: Convert information matrix!!
  for (int i=0; i<6; i++)
    for (int j=i; j<6; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3_normfixed::write(std::ostream& os) const {
  SE3Quat cam2world(measurement().inverse());
  for (int i=0; i<7; i++)
    os << cam2world[i] << " ";
  for (int i=0; i<6; i++)
    for (int j=i; j<6; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}