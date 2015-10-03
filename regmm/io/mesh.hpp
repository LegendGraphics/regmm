#ifndef _MESH_HPP_
#define _MESH_HPP_

#include "basic_types.hpp"

namespace regmm
{
     
}

namespace regmm
{
    template <typename Scalar, int Dim>
    class Vertices
    {
    private:
        std::vector<VertexType> _data;

    public:
        VertexType& operator[](int index);
        void push_back(VertexType& point);
    };

    template <typename Scalar, int Dim>
    VertexType& Vertices<Scalar, Dim>::operator[](int index)
    {
        return _data[index];
    }

    template <typename Scalar, int Dim>
    void Vertices<Scalar, Dim>::push_back(VertexType& point)
    {
        _data.push_back(point);
    }
}

namespace regmm
{
    template <typename Scalar, int Dim>
    class Normals
    {
    private:
        std::vector<NormalType> _data;

    public:
        NormalType& operator[](int index);
        void push_back(NormalType& point);
    };

    template <typename Scalar, int Dim>
    NormalType& Normals<Scalar, Dim>::operator[](int index)
    {
        return _data[index];
    }

    template <typename Scalar, int Dim>
    void Normals<Scalar, Dim>::push_back(NormalType& point)
    {
        _data.push_back(point);
    }
}

namespace regmm
{
    template <typename Scalar, int Dim>
    class Faces
    {
    private:
        std::vector<Face> _data;

    public:
        Face& operator[](int index);
        void push_back(Face& face);
    };

    template <typename Scalar, int Dim>
    Face& Faces<Scalar, Dim>::operator[](int index)
    {
        return _data[index];
    }

    template <typename Scalar, int Dim>
    void Faces<Scalar, Dim>::push_back(Face& point)
    {
        _data.push_back(point);
    }
}



#endif