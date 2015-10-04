#ifndef _POINTS_HPP_
#define _POINTS_HPP_

#include "basic_types.hpp"

namespace regmm
{
    template <typename Scalar, int Dim>
    struct PtsType
    {
        typedef Arrays<PointType, Scalar, Dim> Points;
    };

#define     PointsArray     typename PtsType<PointType, Scalar, Dim>::Points
#define     PointSet        PointsArray
}

#endif