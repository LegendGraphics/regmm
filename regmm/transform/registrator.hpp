#ifndef _REGISTRATOR_HPP_
#define _REGISTRATOR_HPP_


#include "regmm/io/points.hpp"
#include "regmm/io/mesh.hpp"

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
    private:
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
        assert(src_type_ == MESH ? true : ("Source need to be MESH!" && false));

        ps_src_ = &source;
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::setTarget(PointSet& target)
    {
        ps_tgt_ = &target;
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::setSource(MeshObject& source)
    {
        assert(src_type_ == POINT_CLOUD ? true : ("Source need to be POINT CLOUD!" && false));

        mo_src_ = &source;
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::setTarget(MeshObject& target)
    {
        mo_tgt_ = &target;

        ps_tgt_ = &mo_tgt_->getVertices();
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::fillMatrixSource()
    {
        if (src_type_ == POINT_CLOUD)
        {
            for (PointType& pt : ps_src_)
            {
                model_ << pt.x(), pt.y(), pt.z();
            }

            M_ = ps_src_->size();
        }

        else 
        {
            VerticesArray& vts = mo_src_->getVertices();
            for (VertexType& vt : vts)
            {
                model_ << vt.x(), vt.y(), vt.z();
            }

            N_ = mo_src_->size();
        }
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::rewriteOriginalSource()
    {
        if (src_type_ == POINT_CLOUD)
        {
            for (size_t i = 0, i_end = ps_src_->size(); i < i_end; ++ i)
            {
                PointType& pt = ps_src_[i];
                pt.x() = model_(i, 0);
                pt.y() = model_(i, 1);
                pt.z() = model_(i, 2);
            }
        }

        else 
        {
            VerticesArray& vts = mo_src_->getVertices();
            for (size_t i = 0, i_end = mo_src_->size(); i < i_end; ++ i)
            {
                VertexType& vt = vts[i];
                vt.x() = model_(i, 0);
                vt.y() = model_(i, 1);
                vt.z() = model_(i, 2);
            }
        }
    }
}


#endif