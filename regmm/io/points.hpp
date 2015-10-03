#ifndef _POINTS_HPP_
#define _POINTS_HPP_

#include "basic_types.hpp"

namespace regmm
{
    template <typename Scalar, int Dim>
    class Points
    {
    private:
        std::vector<PointType> _data;

    public:
        PointType& operator[](int index);
        void push_back(PointType& point);
    };
}

namespace regmm
{
    template <typename Scalar, int Dim>
    PointType& Points<Scalar, Dim>::operator[](int index)
    {
        return _data[index];
    }

    template <typename Scalar, int Dim>
    void Points<Scalar, Dim>::push_back(PointType& point)
    {
        _data.push_back(point);
    }
}

#endif