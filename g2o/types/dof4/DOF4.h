#pragma once

#ifndef G2O_DOF4_H_
#define G2O_DOF4_H_

#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h"

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace g2o{

     class  DOF4
    {
        public:
            DOF4():_R(0),_t(0,0,0){};
            DOF4(double x,double y,double z,double yaw):_R(yaw),_t(x,y,z){};

            const Eigen::Vector3d& translation() const {return _t;}

            const Eigen::Rotation2Dd& rotation() const {return _R;}


            double operator [](int i) const {
                assert (i>=0 && i<4);
                if (i<3)
                    return _t(i);
                return _R.angle();
            }
            
            double& operator [](int i){
                assert (i>=0 && i<4);
                if (i<3)
                    return _t(i);
                return _R.angle();
            }

            void fromVector (const Eigen::Vector4d& v){
                *this=DOF4(v[0], v[1], v[2],v[3]);
            }

            Eigen::Vector4d toVector() const{
                Eigen::Vector4d ret;
                for (int i=0; i<4; i++){
                    ret(i)=(*this)[i];
                }
                return ret;
            }

            ~DOF4(){};


        protected:
            Eigen::Rotation2Dd _R;
            Eigen::Vector3d _t;
    };
}
#endif