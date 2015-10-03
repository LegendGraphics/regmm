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
        RegType     reg_type_;
        DataType    source_type_;

    public:
        void setRegType(RegType rt);
        void setDataType(DataType dt);
    };
}

namespace regmm
{
    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::setRegType(RegType rt)
    {
        reg_type_ = rt;
    }

    template <typename Scalar, int Dim>
    void Registrator<Scalar, Dim>::setDataType(DataType dt)
    {
        source_type_ = dt;
    }
}


#endif