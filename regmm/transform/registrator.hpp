#ifndef _REGISTRATOR_HPP_
#define _REGISTRATOR_HPP_

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif


#include "regmm/io/points.hpp"
#include "regmm/io/mesh.hpp"
#include "regmm/io/cast.hpp"

namespace regmm
{
    enum RegType
    {
        RIGID,
        NONRIGID
    };

    enum DataType
    {
        POINT_CLOUD,
        MESH
    };


    template <typename Scalar, int Dim>
    class Registrator
    {
    protected:
        RegType         reg_type_;
        DataType        src_type_;

        // original data
        PointSet*       ps_src_;
        MeshObject*     mo_src_;

        PointSet*       ps_tgt_;
        MeshObject*     mo_tgt_;

        // matrix data
        TMatrixD        model_;
        TMatrixD        data_;

        int             M_;
        int             N_;

    public:
        Registrator();
        virtual ~Registrator();

        void setRegType(RegType rt);
        void setDataType(DataType dt);

        void setSource(PointSet& source);
        void setTarget(PointSet& target);

        void setSource(MeshObject& source);
        void setTarget(MeshObject& target);

        virtual void compute() = 0;

    protected:
        void fillMatrixSource();
        void fillMatrixTarget();
        void rewriteOriginalSource();
    };
}

namespace regmm
{
    template <typename Scalar, int Dim>
    Registrator<Scalar, Dim>::Registrator()
        :reg_type_(RIGID),
        src_type_(POINT_CLOUD),
        ps_src_(nullptr),
        ps_tgt_(nullptr),
        mo_src_(nullptr),
        mo_tgt_(nullptr)
    {

    }

    template <typename Scalar, int Dim>
    Registrator<Scalar, Dim>::~Registrator()
    {

    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::setRegType(RegType rt)
    {
        reg_type_ = rt;
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::setDataType(DataType dt)
    {
        src_type_ = dt;
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::setSource(PointSet& source)
    {
        assert(src_type_ == POINT_CLOUD ? true : ("Source need to be POINT CLOUD!" && false));

        ps_src_ = &source;

        fillMatrixSource();
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::setTarget(PointSet& target)
    {
        ps_tgt_ = &target;

        fillMatrixTarget();
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::setSource(MeshObject& source)
    {
        assert(src_type_ == MESH ? true : ("Source need to be MESH!" && false));

        mo_src_ = &source;

        fillMatrixSource();
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::setTarget(MeshObject& target)
    {
        mo_tgt_ = &target;

        ps_tgt_ = &mo_tgt_->getVertices();

        fillMatrixTarget();
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::fillMatrixSource()
    {
        if (src_type_ == POINT_CLOUD)
        {
            PointSet_To_TMatrixD<Scalar, Dim>(*ps_src_, model_);
            M_ = ps_src_->size();
        }

        else 
        {
            MeshObject_To_TMatrixD<Scalar, Dim>(*mo_src_, model_);
            M_ = mo_src_->size();
        }
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::fillMatrixTarget()
    {
        PointSet_To_TMatrixD<Scalar, Dim>(*ps_tgt_, data_);
        N_ = ps_tgt_->size();
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::rewriteOriginalSource()
    {
        if (src_type_ == POINT_CLOUD)
            TMatrixD_To_PointSet<Scalar, Dim>(model_, *ps_src_);
        else 
            TMatrixD_To_MeshObject<Scalar, Dim>(model_, *mo_src_);
    }
}


#endif