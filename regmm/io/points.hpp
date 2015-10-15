#ifndef _POINTS_HPP_
#define _POINTS_HPP_

#include <string>

#include "regmm/io/basic_types.hpp"

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

namespace regmm
{
    template <typename Scalar, int Dim>
    void loadPointSet(const std::string& file, PointSet& ps)
    {

    }
}

#endif