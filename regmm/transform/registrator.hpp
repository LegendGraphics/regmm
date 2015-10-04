#ifndef _REGISTRATOR_HPP_
#define _REGISTRATOR_HPP_

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

        PointSet*       ps_src_;
        MeshObject*     mo_src_;

        PointSet*       ps_tgt_;
        MeshObject*     mo_tgt_;

        TMatrixD*       mat_src_;
        TMatrixD*       mat_tgt_;

    public:
        Registrator();
        virtual ~Registrator();

        void setRegType(RegType rt);
        void setDataType(DataType dt);

        void setSource(PointSet& source);
        void setTarget(PointSet& target);

        void setSource(MeshObject& source);
        void setTarget(MeshObject& target);

    protected:
        void normalize();
        void denormalize();
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
    void Registrator<Scalar, Dim>::normalize()
    {

    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::denormalize()
    {

    }
}


#endif