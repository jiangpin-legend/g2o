#include "EdgeDOF4.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o{
    EdgeDOF4::EdgeDOF4():
        g2o::BaseBinaryEdge<4,Eigen::VectorXd,VertexDOF4, VertexDOF4>()
        {

        }

        bool EdgeDOF4::read(std::istream& in){
            Eigen::VectorXd p= Eigen::Matrix<double, 1,  Eigen::Dynamic>(6);
            in >> p[0]>> p[1] >> p[2]>> p[3]>> p[4]>> p[5];
            _measurement=p;

            for (int i = 0; i <4; ++i)
                for (int j = i; j < 4; ++j) {
                in >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
            return true;
        }


        bool EdgeDOF4::write(std::ostream& out) const{

            Eigen::VectorXd p= Eigen::Matrix<double, 1,  Eigen::Dynamic>(6);
            p = _measurement;
            out << p[0] << " " << p[1]<< " " << p[2]<<" "
                << p[3] << " " << p[4]<< " " << p[5];
            
            for (int i = 0; i < 4; ++i)
                for (int j = i; j < 4; ++j)
                out << " " << information()(i, j);
            return out.good();
        }

        EdgeDOF4WriteGnuplotAction::EdgeDOF4WriteGnuplotAction(): WriteGnuplotAction(typeid(EdgeDOF4).name()){}

        HyperGraphElementAction* EdgeDOF4WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
            if (typeid(*element).name()!=_typeName)
            return nullptr;
            WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
            if (!params->os){
            std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
            return nullptr;
            }

            EdgeDOF4* e =  static_cast<EdgeDOF4*>(element);
            VertexDOF4* fromEdge = static_cast<VertexDOF4*>(e->vertex(0));
            VertexDOF4* toEdge   = static_cast<VertexDOF4*>(e->vertex(1));
            *(params->os) << fromEdge->estimate().translation().x() << " " << fromEdge->estimate().translation().y()
            << " " <<fromEdge->estimate().translation().z()<<" "<<fromEdge->estimate().rotation().angle() << std::endl;
            *(params->os) << toEdge->estimate().translation().x() << " " << toEdge->estimate().translation().y()
            << " " <<fromEdge->estimate().translation().z()<<" "<<toEdge->estimate().rotation().angle() << std::endl;
            *(params->os) << std::endl;
            return this;
        }
        #ifdef DRAW_DOF4
        #ifdef G2O_HAVE_OPENGL
        EdgeDOF4DrawAction::EdgeDOF4DrawAction()
            : DrawAction(typeid(EdgeDOF4).name()), _triangleX(nullptr), _triangleY(nullptr) {}

        bool EdgeDOF4DrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
            if (!DrawAction::refreshPropertyPtrs(params_))
            return false;
            if (_previousParams){
            _triangleX = _previousParams->makeProperty<FloatProperty>(_typeName + "::GHOST_TRIANGLE_X", .2f);
            _triangleY = _previousParams->makeProperty<FloatProperty>(_typeName + "::GHOST_TRIANGLE_Y", .05f);
            } else {
            _triangleX = 0;
            _triangleY = 0;
            }
            return true;
        }

        HyperGraphElementAction* EdgeDOF4DrawAction::operator()(HyperGraph::HyperGraphElement* element,HyperGraphElementAction::Parameters* params_){
            if (typeid(*element).name()!=_typeName)
                return nullptr;
            refreshPropertyPtrs(params_);
            if (! _previousParams)
                return this;

            if (_show && !_show->value())
                return this;

            EdgeDOF4* e =  static_cast<EdgeDOF4*>(element);
            VertexDOF4* fromEdge = static_cast<VertexDOF4*>(e->vertices()[0]);
            VertexDOF4* toEdge   = static_cast<VertexDOF4*>(e->vertices()[1]);
            if (! fromEdge || ! toEdge)
            return this;
            glColor3f(POSE_EDGE_COLOR);
            glPushAttrib(GL_ENABLE_BIT);
            glDisable(GL_LIGHTING);
            glBegin(GL_LINES);
            glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),(float)fromEdge->estimate().translation().z());
            glVertex3f((float)toEdge->estimate().translation().x(),(float)toEdge->estimate().translation().y(),(float)toEdge->estimate().translation().z());
            glEnd();
            glPopAttrib();
            return this;
            
        }
        #endif
        #endif

}