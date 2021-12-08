#include "VertexDOF4.h"

namespace g2o{
    VertexDOF4::VertexDOF4() :
        g2o::BaseVertex<4, DOF4>()
    {
    }

    // bool VertexDOF4::read(std::istrean& is)
    // {
    //     Vector6 est;
    //     bool state = internal::readVector(is, est);
    //     setEstimate(internal::fromVectorQT(est));
    //     return state;
    // }
    
    bool VertexDOF4::read(std::istream& is) 
    {
        std::cout << "in read" << std::endl;
        Eigen::Vector4d p;
        bool state = internal::readVector(is, p);
        std::cout << "state " << state << std::endl;

        // is >> p[0] >> p[1] >> p[2]>>p[3];
        _estimate.fromVector(p);
        std::cout << "end read" << std::endl;
        return true;
    }

    bool VertexDOF4::write(std::ostream& os) const 
    {
        Eigen::Vector4d p = estimate().toVector();
        os << p[0] << " " << p[1] << " " << p[2]<<" "<<p[3];
        // std::cout<<"write Vertex"<<std::endl;
        return os.good();
    }
    

    VertexDOF4WriteGnuplotAction::VertexDOF4WriteGnuplotAction(): WriteGnuplotAction(typeid(VertexDOF4).name()){}

    HyperGraphElementAction* VertexDOF4WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
        if (typeid(*element).name()!=_typeName)
            return nullptr;
        WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
        if (!params || !params->os){
            std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid output stream specified" << std::endl;
            return nullptr;
        }

        VertexDOF4* v =  static_cast<VertexDOF4*>(element);
        *(params->os) << v->estimate().translation().x() << " " << v->estimate().translation().y()<<
         " "<<v->estimate().translation().z()<< " " << v->estimate().rotation().angle() << std::endl;
        return this;
    }
    #ifdef DRAW_DOF4
    #ifdef G2O_HAVE_OPENGL
    VertexDOF4DrawAction::VertexDOF4DrawAction()
        : DrawAction(typeid(VertexDOF4).name()) {}

    bool VertexDOF4DrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
        if (!DrawAction::refreshPropertyPtrs(params_))
        return false;
        if (_previousParams){
        _triangleX = _previousParams->makeProperty<FloatProperty>(_typeName + "::TRIANGLE_X", .2f);
        _triangleY = _previousParams->makeProperty<FloatProperty>(_typeName + "::TRIANGLE_Y", .05f);
        } else {
        _triangleX = 0;
        _triangleY = 0;
        }
        return true;
    }


    HyperGraphElementAction* VertexDOF4DrawAction::operator()(HyperGraph::HyperGraphElement* element,
                    HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
        return nullptr;
        initializeDrawActionsCache();
        refreshPropertyPtrs(params_);

        if (! _previousParams)
        return this;

        if (_show && !_show->value())
        return this;

        VertexDOF4* that = static_cast<VertexDOF4*>(element);

        glColor3f(POSE_VERTEX_COLOR);
        glPushMatrix();
        glTranslatef((float)that->estimate().translation().x(),(float)that->estimate().translation().y(),(float)that->estimate().translation().z());
        glRotatef((float)RAD2DEG(that->estimate().rotation().angle()),0.f,0.f,1.f);
        opengl::drawArrow2D((float)_triangleX->value(), (float)_triangleY->value(), (float)_triangleX->value()*.4f);
        drawCache(that->cacheContainer(), params_);
        drawUserData(that->userData(), params_);
        glPopMatrix();
        return this;
    }
    #endif
    #endif

}