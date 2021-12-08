#pragma once
#ifndef G2O_EDGE_DOF4_H
#define G2O_EDGE_DOF4_H

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/factory.h"
#include "DOF4.h"
#include "g2o_types_dof4_api.h"
// #include <include/global_manager/DOF4.h>
#include <math.h>

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

#define PI 3.1415926

namespace g2o{
    class  VertexDOF4: public g2o::BaseVertex<4, DOF4>
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexDOF4();

        virtual bool read(std::istream& is);

        virtual bool write(std::ostream& os) const;
        
        virtual void setToOriginImpl()
        {
            _estimate = DOF4();
        }

        virtual void oplusImpl(const double* update) override
        {
            double x = update[0];
            double y = update[1];
            double z = update[2];
            double yaw = update[3];

            _estimate[0] += update[0]*cos(yaw)-update[1]*sin(yaw);
            _estimate[1] += update[0]*sin(yaw)+update[1]*cos(yaw);
            _estimate[2] += update[2];
            _estimate[3] += update[3];
            _estimate[3] = Mod(_estimate[3],2*PI);
        }

        double& operator [](int i) {
            return _estimate[i];
        }

        double Mod(double a ,double b){
            double c = (int)(a/b);
            double result = a-c*b;
            return result;
        }

    };
    class VertexDOF4WriteGnuplotAction: public WriteGnuplotAction {
    public:
        VertexDOF4WriteGnuplotAction();
        virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
                HyperGraphElementAction::Parameters* params_);
    };
    #ifdef DRAW_DOF4
    #ifdef G2O_HAVE_OPENGL
        class VertexDOF4DrawAction: public DrawAction{
        public:
            VertexDOF4DrawAction();
            virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
                    HyperGraphElementAction::Parameters* params_);
        protected:
            virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
            FloatProperty *_triangleX, *_triangleY;
        };
    #endif
    #endif
}
#endif
