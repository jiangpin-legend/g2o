#pragma once
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/base_binary_edge.h"
#include "DOF4.h"
#include "VertexDOF4.h"
#include "g2o_types_dof4_api.h"

using namespace Eigen;
namespace g2o{
class EdgeDOF4: public g2o::BaseBinaryEdge<4,Eigen::VectorXd,VertexDOF4, VertexDOF4>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW      

      EdgeDOF4();
      bool read(std::istream& in);

      bool write(std::ostream& out) const;
            
      virtual void computeError() override{
          //the rotation between the current frame i and world
          double Yawi;
          
         
          const VertexDOF4* v1 = static_cast<const VertexDOF4*>(_vertices[0]);
          const VertexDOF4* v2 = static_cast<const VertexDOF4*>(_vertices[1]);
          
          Yawi = (v1->estimate())[3];
          Eigen::Matrix3d Ri2w;
          
          Ri2w = AngleAxisd(_Rolli, Vector3d::UnitX()) *
                AngleAxisd(_Pitchi, Vector3d::UnitY()) *
                AngleAxisd(Yawi, Vector3d::UnitZ());

          auto delta_t = Ri2w.inverse()*((v2->estimate()).translation()-(v1->estimate()).translation())-_Tij;


          double delta_yaw = (v2->estimate())[3]-(v1->estimate())[3]-_Yawij;
          
          DOF4 delta(delta_t[0],delta_t[1],delta_t[2],delta_yaw);
          _error = delta.toVector();

      }

      
      virtual void linearizeOplus() override{
        
        const VertexDOF4* v1 = static_cast<const VertexDOF4*>(_vertices[0]);
        const VertexDOF4* v2 = static_cast<const VertexDOF4*>(_vertices[1]);
        double xi,yi,zi,xj,yj,zj,Yawi;
        xi = (v1->estimate())[0];
        yi = (v1->estimate())[1];
        zi = (v1->estimate())[2];
        Yawi = (v1->estimate())[3];

        xj = (v2->estimate())[0];
        yj = (v2->estimate())[1];
        zj = (v2->estimate())[2];

        Eigen::Matrix3d Ri2w;
        Ri2w = AngleAxisd(_Rolli, Vector3d::UnitX()) *
                AngleAxisd(_Pitchi, Vector3d::UnitY())*
                AngleAxisd(Yawi, Vector3d::UnitZ());

        Eigen::Matrix3d  Rw2i = Ri2w.inverse();

        Eigen::Vector3d dYawi,dTji;
        dTji<<(xj-xi),(yj-yi),(zj-zi);
        Eigen::Matrix3d dR;
        dR<<-sin(Yawi),cos(Yawi),0,
             -cos(Yawi),-sin(Yawi),0,
             0,0,0;

        // dYawi<<(cos(Yawi)*yi-sin(Yawi)*xi),(cos(Yawi)*xi-sin(Yawi))*yi,0;
        // dYawi = AngleAxisd(_Rolli, Vector3d::UnitX()) *
        //         AngleAxisd(_Pitchi, Vector3d::UnitY())*
        //         dR*dTji;

        dYawi = dR*(AngleAxisd(_Rolli, Vector3d::UnitX()) *
              AngleAxisd(_Pitchi, Vector3d::UnitY())).inverse()*
              dTji;


        for(int i=0;i<3;i++){
          for(int j=0;j<3;j++){
            _jacobianOplusXi(i,j) = -Rw2i(i,j);
          }
        }

        _jacobianOplusXi(0,3) = dYawi(0);
        _jacobianOplusXi(1,3) = dYawi(1);
        _jacobianOplusXi(2,3) = dYawi(2);
        _jacobianOplusXi(3,3) = -1;

        _jacobianOplusXi(3,0) = 0;
        _jacobianOplusXi(3,1) = 0;
        _jacobianOplusXi(3,2) = 0;


        for(int i=0;i<3;i++){
          for(int j=0;j<3;j++){
            _jacobianOplusXj(i,j) = Rw2i(i,j);
          }
        }

        _jacobianOplusXj(3,0) = 0;
        _jacobianOplusXj(3,1) = 0;
        _jacobianOplusXj(3,2) = 0;
        _jacobianOplusXj(3,3) = 1;

        _jacobianOplusXj(0,3) = 0;
        _jacobianOplusXj(1,3) = 0;
        _jacobianOplusXj(2,3) = 0;
        

      }   
      
      virtual void setMeasurement(Eigen::VectorXd m){
          _measurement = m;
          _Tij<<_measurement[0],_measurement[1],_measurement[2];
      
          _Rolli =  _measurement[3];
          _Pitchi = _measurement[4];
          _Yawij =  _measurement[5];

      }


    private:

    protected:
      double _Rolli,_Pitchi,_Yawij;
      Eigen::Vector3d _Tij;
  };


  class  EdgeDOF4WriteGnuplotAction: public WriteGnuplotAction {
    public:
      EdgeDOF4WriteGnuplotAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
              HyperGraphElementAction::Parameters* params_);
    };
  #ifdef DRAW_DOF4
  #ifdef G2O_HAVE_OPENGL
    class  EdgeDOF4DrawAction: public DrawAction{
    public:
      EdgeDOF4DrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
              HyperGraphElementAction::Parameters* params_);
    protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
      FloatProperty *_triangleX, *_triangleY;
    };
  #endif
  #endif

}