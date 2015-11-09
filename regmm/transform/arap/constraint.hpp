#ifndef _CONSTRAINT_HPP_
#define _CONSTRAINT_HPP_

#include "regmm/io/basic_types.hpp"

namespace regmm
{
    template <typename Scalar, int Dim>
    class Constraint
    {
    public:
        inline std::vector<TSparseMatrix>& A() { return A_; }
        inline TMatrixD& b(){ return b_; }

        virtual void build() = 0;
        virtual void projection() = 0;
        virtual void update() = 0;

    protected:
        virtual void buildA() = 0;
        virtual void buildb() = 0;

    protected:
        std::vector<TSparseMatrix> A_;
        TMatrixD b_;
    };
}

#endif