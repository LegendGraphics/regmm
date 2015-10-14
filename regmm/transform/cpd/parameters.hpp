/**
 * @file
 * @author  Xiaochen Fan <fan.daybreak@gmail.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * 
 */

#ifndef _PARAMETERS_HPP_
#define _PARAMETERS_HPP_

namespace regmm
{
    template <typename Scalar, int Dim>
    struct RigidParas  
    {
        TMatrix         R_;
        TVector         t_;
        Scalar          s_;
        Scalar          sigma2_;
    };

    template <typename Scalar, int Dim>
    struct AffineParas  
    {
        TMatrix         B_;
        TVector         t_;
        Scalar          sigma2_;
    };

    template <typename Scalar, int Dim>
    struct NRigidParas
    {
        TMatrix         W_;
        Scalar          sigma2_;
        Scalar          lambda_;
        Scalar          beta_;
    };

    template <typename Scalar, int Dim>
    struct Normalize
    {
        TVector         means_;
        Scalar          scale_;
    };
}

#endif