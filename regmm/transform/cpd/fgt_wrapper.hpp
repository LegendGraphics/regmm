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

#ifndef FGT_WRAPPER_HPP
#define FGT_WRAPPER_HPP

#define FIGTREE_NO_ANN 

#include "regmm/3rdparty/figtree/include/figtree.h"
#include "regmm/io/basic_types.hpp"

namespace regmm
{
    template <typename Scalar, int Dim>
    TMatrix fgt(const TMatrix& x, const TMatrix& y, const TMatrix& q, Scalar h, 
        Scalar epsilon = 1e-3,
        int evalMethod = FIGTREE_EVAL_AUTO,
        int ifgtParamMethod = FIGTREE_PARAM_NON_UNIFORM,
        int ifgtTruncMethod = FIGTREE_TRUNC_CLUSTER,
        int verbose = 0)
    {
        // very strange usage...
        typename MatrixType<double, Dim>::MatrixD x_r = x.template cast<double>();
        typename MatrixType<double, Dim>::MatrixD y_r = y.template cast<double>();
        typename MatrixType<double, Dim>::Matrix q_r = q.template cast<double>().transpose();

        int d = Dim;
        int N = x_r.rows();
        int M = y_r.rows();
        int W = q_r.rows();

        double *X, *Y, *Q;
        X = x_r.data();
        Y = y_r.data();
        Q = q_r.data();

        typename MatrixType<double, Dim>::Matrix G(W, M);

        figtree(d, N, M, W, X, h, Q, Y, epsilon, G.data(), evalMethod, ifgtParamMethod, ifgtTruncMethod, verbose);

        TMatrix g = G.template cast<Scalar>().transpose();

        return g;
    }
}

#endif