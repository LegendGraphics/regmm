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

#ifndef _CPD_BASE_HPP_
#define _CPD_BASE_HPP_

#include "regmm/transform/cpd/parameters.hpp"
#include "regmm/transform/registrator.hpp"

namespace regmm
{
    template <typename Scalar, int Dim>
    class CPDBase: public Registrator<Scalar, Dim>
    {
    public:
        CPDBase();
        virtual ~CPDBase();

        void setIterativeNumber(size_t iter_num);
        void setVarianceTolerance(Scalar v_tol);
        void setEnergyTolerance(Scalar e_tol);
        void setOutlierWeight(Scalar w);
        void setFgtFlag(bool fgt);
        void setFgtEpsilon(Scalar fgt_eps);
        void setLowRankFlag(bool lr);
        void setKLowRank(int K);
        void setLRMaxIteration(size_t lr_maxitr);

        void normalize();
        void denormalize();

        inline const TMatrix& getTransform() const { return T_; } 
        inline const TMatrix& getCorrespondences() const { return corres_; }

        virtual void compute() = 0;

    private:
        virtual void initialization() = 0;
        virtual void e_step() = 0;
        virtual void m_step() = 0;
        virtual void align() = 0;
        virtual void correspondences() = 0;

    protected:
        void updateModel();
        void initTransform();

    protected:
        size_t      iter_num_;
        Scalar      v_tol_;
        Scalar      e_tol_;
        Scalar      w_;

        TMatrix     corres_;
        TMatrix     T_;

        TMatrix     P1_;
        TMatrix     PT1_;
        TMatrix     PX_;

        bool        fgt_;
        Scalar      fgt_eps_;

        bool        lr_;
        int         K_;
        size_t      lr_maxitr_;

        Normalize<Scalar, Dim>    normalize_model_;
        Normalize<Scalar, Dim>    normalize_data_;
    };
}

namespace regmm
{
    template <typename Scalar, int Dim>
    CPDBase<Scalar, Dim>::CPDBase()
        : iter_num_(50), v_tol_(1e-3), e_tol_(1e-3), w_(0),
        fgt_(false), fgt_eps_(1e-3), lr_(false), K_(10), lr_maxitr_(40)
    {}

    template <typename Scalar, int Dim>
    CPDBase<Scalar, Dim>::~CPDBase(){}

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::setIterativeNumber(size_t iter_num)
    {
        iter_num_ = iter_num;
    }

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::setVarianceTolerance(Scalar v_tol)
    {
        v_tol_ = v_tol;
    }

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::setEnergyTolerance(Scalar e_tol)
    {
        e_tol_ = e_tol;
    }

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::setOutlierWeight(Scalar w)
    {
        w_ = w;
    }

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::setFgtFlag(bool fgt)
    {
        fgt_ = fgt;
    }

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::setFgtEpsilon(Scalar fgt_eps)
    {
        fgt_eps_ = fgt_eps;
    }

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::setLowRankFlag(bool lr)
    {
        lr_ = lr;
    }

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::setKLowRank(int K)
    {
        K_ = K;
    }

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::setLRMaxIteration(size_t lr_maxitr)
    {
        lr_maxitr_ = lr_maxitr;
    }

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::updateModel()
    {
        this->model_ = T_;
    }

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::initTransform()
    {
        T_ = this->model_;
    }

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::normalize()
    {
        normalize_model_.means_ = this->model_.colwise().mean();
        normalize_data_.means_ = this->data_.colwise().mean();

        this->model_ = this->model_ - normalize_model_.means_.transpose().replicate(this->M_, 1);
        this->data_ = this->data_ - normalize_data_.means_.transpose().replicate(this->N_, 1);

        normalize_model_.scale_ = sqrt(this->model_.array().square().sum() / this->M_);
        normalize_data_.scale_ = sqrt(this->data_.array().square().sum() / this->N_);

        this->model_ = this->model_ / normalize_model_.scale_;
        this->data_ = this->data_ / normalize_data_.scale_;
    }

    template <typename Scalar, int Dim>
    void CPDBase<Scalar, Dim>::denormalize()
    {
        this->model_ = this->model_ * normalize_data_.scale_ + normalize_data_.means_.transpose().replicate(this->M_, 1);
        this->data_ = this->data_ * normalize_data_.scale_ + normalize_data_.means_.transpose().replicate(this->N_, 1);
    }
}

#endif