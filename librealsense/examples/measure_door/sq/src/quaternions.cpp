#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "quaternions.h"

struct xyzw angletrans(double angle)
{
  struct xyzw tmp;
  //angle trans
  //旋转向量v（轴角）
  //旋转向量使用AngleAxisd(f),底层不直接是Matrix，但因为重载了运算符，运算可以当作矩阵
  Eigen::AngleAxisd v(M_PI/(180/angle),Eigen::Vector3d(0,0,1));//沿z轴旋转了(180/angle)度
  cout << "rotation vector: Angle is: " << v.angle() * (180 / M_PI)<<endl//旋转角
       << "  Axis is: " << v.axis().transpose() << endl<<endl;//旋转轴

  //将旋转向量转化为四元数q
  Eigen::Quaterniond q = Eigen::Quaterniond(v);
  //cout<<"q=\n"<<q.coeffs()<<endl<<endl;//coeffs的顺序:(x,y,z,w)
  cout<<"q=\n"<<q.x()<<endl<<q.y()<<endl<<q.z()<<endl<<q.w()<<endl<<endl;//四元数的另一种输出方式
  //end
  tmp.x = q.x();
  tmp.y = q.y();
  tmp.z = q.z();
  tmp.w = q.w();
  return tmp;
}







