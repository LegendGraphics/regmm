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

#ifndef _CPD_NRIGID_HPP_
#define _CPD_NRIGID_HPP_

#define _USE_MATH_DEFINES

#include <cmath>
#include <vector>

#include <Eigen/SVD>

#include "cpd/cpd_base.hpp"
#include "cpd/parameters.hpp"
//#include "fast/fgt_wrapper.hpp"
//#include "fast/low_rank.hpp"
//#include "disp/render_thread.hpp"

namespace regmm
{
    template <typename Scalar, int Dim>
    class CPDNRigid: public CPDBase<Scalar, Dim>
    {
    public:
        CPDNRigid();
        virtual ~CPDNRigid();

        void setLambda(Scalar lambda);
        void setBeta(Scalar beta);
        inline NRigidParas<Scalar, Dim>& getParameters(){ return paras_; }

        inline TMatrix& getG(){ return G_; }

        void run();

    private:
        void initialization();
        void e_step();
        void m_step();
        void align();
        void correspondences();

        void constructG();
        Scalar computeGaussianExp(size_t m, size_t n);
        Scalar energy();

        void P1_correlation();

    private:
        NRigidParas<Scalar, Dim>   paras_;
        TMatrix             G_;
        TMatrix             Q_;
        TMatrix             S_;
    };
}

namespace regmm
{
    template <typename Scalar, int Dim>
    CPDNRigid<Scalar, Dim>::CPDNRigid()
    {
        paras_.lambda_ = 2;
        paras_.beta_ = 2;
    }

    template <typename Scalar, int Dim>
    CPDNRigid<Scalar, Dim>::~CPDNRigid(){}

    template <typename Scalar, int Dim>
    void CPDNRigid<Scalar, Dim>::setLambda(Scalar lambda)
    {
        paras_.lambda_ = lambda;
    }

    template <typename Scalar, int Dim>
    void CPDNRigid<Scalar, Dim>::setBeta(Scalar beta)
    {
        paras_.beta_ = beta;
    }

    template <typename Scalar, int Dim>
    void CPDNRigid<Scalar, Dim>::run()
    {
        size_t iter_num = 0;
        Scalar e_tol = 10 + this->e_tol_;
        Scalar e = 0;
        
        this->normalize();
        initialization();

        /*if (this->_vision)
        {
        RenderThread<Scalar, Dim>::instance()->updateModel(this->model_);
        RenderThread<Scalar, Dim>::instance()->updateData(this->data_);
        RenderThread<Scalar, Dim>::instance()->startThread();
        }*/

        while (iter_num < this->iter_num_ && e_tol > this->e_tol_ && paras_.sigma2_ > 10 * this->v_tol_)
        {

            e_step();
            
            Scalar old_e = e;
            e = energy();
            e += paras_.lambda_/2 * (paras_.W_.transpose()*G_*paras_.W_).trace();
            e_tol = fabs((e - old_e) / e);

            m_step();

            /*if (this->_vision == true)
            RenderThread<Scalar, Dim>::instance()->updateModel(this->T_);*/

            iter_num ++;
        }
        
        correspondences();
        this->updateModel();
        this->denormalize();
        /*RenderThread<Scalar, Dim>::instance()->cancel();*/
    }

    template <typename Scalar, int Dim>
    void CPDNRigid<Scalar, Dim>::correspondences()
    {
        this->corres_.setZero(this->M_, this->N_);

        for (size_t n = 0; n < this->N_; n ++)
        {
            typename std::vector<Scalar> t_exp;
            Scalar sum_exp = 0;
            Scalar c = pow((2*M_PI*paras_.sigma2_), 0.5*Dim) * (this->w_/(1-this->w_)) * (Scalar(this->M_)/this->N_);
            for (size_t m = 0; m < this->M_; m ++)
            {
                Scalar m_exp = computeGaussianExp(m, n);
                t_exp.push_back(m_exp);
                sum_exp += m_exp;
            }

            for (size_t m = 0; m < this->M_; m ++)
            {
                this->corres_(m, n) = t_exp.at(m) / (sum_exp + c);
            }
        }
    }

    template <typename Scalar, int Dim>
    void CPDNRigid<Scalar, Dim>::initialization()
    {

        paras_.W_ = MatrixType<Scalar, Dim>::Matrix::Zero(this->M_, Dim);

        Scalar sigma_sum = this->M_*(this->data_.transpose()*this->data_).trace() + 
            this->N_*(this->model_.transpose()*this->model_).trace() - 
            2*(this->data_.colwise().sum())*(this->model_.colwise().sum()).transpose();
        paras_.sigma2_ = sigma_sum / (Dim*this->N_*this->M_);

        this->initTransform();
        constructG();
        
        if (this->lr_)
            lr_approximate<Scalar, Dim>(G_, Q_, S_, this->K_, this->lr_maxitr_);
    }

    template<typename Scalar, int Dim>
    void CPDNRigid<Scalar, Dim>::e_step()
    {
        if (!this->fgt_)
        {
            correspondences();
            this->P1_ = this->corres_ * TVector(this->N_).setOnes();
            this->PT1_ = this->corres_.transpose() * TVector(this->M_).setOnes();
            this->PX_ = this->corres_ * this->data_;
        }
        else
        {
            Scalar c = pow((2*M_PI*paras_.sigma2_), 0.5*Dim) * (this->w_/(1-this->w_)) * (Scalar(this->M_)/this->N_);
            TMatrix KT1 = fgt<Scalar, Dim>(this->T_, this->data_, TVector(this->M_).setOnes(), sqrt(2*paras_.sigma2_), this->fgt_eps_);
            TVector a = (TVector(KT1) + c*TVector(this->N_).setOnes()).cwiseInverse();

            TMatrix aX = MatrixType<Scalar, Dim>::Matrix::Zero(this->N_, Dim);
            for (size_t i = 0; i < Dim; i ++)
            {
                aX.col(i) = this->data_.col(i).cwiseProduct(a);
            }

            this->PT1_ = TVector(this->N_).setOnes() - c * a;
            this->P1_ = fgt<Scalar, Dim>(this->data_, this->T_, a, sqrt(2*paras_.sigma2_), this->fgt_eps_);
            this->PX_ = fgt<Scalar, Dim>(this->data_, this->T_, aX, sqrt(2*paras_.sigma2_), this->fgt_eps_);
        }
    }

    template<typename Scalar, int Dim>
    void CPDNRigid<Scalar, Dim>::m_step()
    {
        Scalar N_P = this->P1_.sum();

        if (!this->lr_)
        {
            TMatrix A = (this->P1_.asDiagonal()*G_ + paras_.lambda_*paras_.sigma2_*MatrixType<Scalar, Dim>::Matrix::Identity(this->M_, this->M_));
            TMatrix B = this->PX_ - this->P1_.asDiagonal() * this->model_;
            paras_.W_ = A.inverse() * B;
        }
        else
        {
            P1_correlation();
            TMatrix A1 = ((1/(paras_.lambda_*paras_.sigma2_))*this->P1_).asDiagonal();
            TMatrix A2 = Q_ * (S_.inverse() + Q_.transpose()*A1*Q_).inverse() * Q_.transpose();
            TMatrix A_inv = A1 - A1 * A2 * A1;
            TMatrix B = this->P1_.cwiseInverse().asDiagonal() * this->PX_ - this->model_;
            paras_.W_ = A_inv * B;
        }

        align();

        paras_.sigma2_ = 1/(N_P*Dim) * ((this->data_.transpose()*this->PT1_.asDiagonal()*this->data_).trace() -
            2*(this->PX_.transpose()*this->T_).trace() + (this->T_.transpose()*this->P1_.asDiagonal()*this->T_).trace());
        paras_.sigma2_ = fabs(paras_.sigma2_);

    }

    template<typename Scalar, int Dim>
    void CPDNRigid<Scalar, Dim>::align()
    {
        this->T_ = this->model_ + G_ * paras_.W_;
    }

    template <typename Scalar, int Dim>
    Scalar CPDNRigid<Scalar, Dim>::energy()
    {
        Scalar e = 0;
        
        for (size_t n = 0; n < this->N_; n ++)
        {
            Scalar sp = 0;
            for (size_t m = 0; m < this->M_; m ++)
            {
                sp += computeGaussianExp(m, n);
            }

            sp += pow((2*M_PI*paras_.sigma2_), 0.5*Dim) * (this->w_/(1-this->w_)) * (Scalar(this->M_)/this->N_);

            e += -log(sp);

        }

        e += this->N_ * Dim * log(paras_.sigma2_) / 2;

        return e;
    }

    template <typename Scalar, int Dim>
    Scalar CPDNRigid<Scalar, Dim>::computeGaussianExp(size_t m, size_t n)
    {
        TVector vec = TVector(this->T_.row(m) - this->data_.row(n));
        Scalar g_exp = exp(-vec.squaredNorm()/(2*paras_.sigma2_));
        return g_exp;
    }

    template <typename Scalar, int Dim>
    void CPDNRigid<Scalar, Dim>::constructG()
    {
        G_ = MatrixType<Scalar, Dim>::Matrix::Zero(this->M_, this->M_);

        for (size_t i = 0; i < this->M_; i ++)
        {
            for (size_t j = 0; j < this->M_; j ++)
            {
                G_(i, j) = exp(-TVector(this->model_.row(i)-this->model_.row(j)).squaredNorm()/(2*paras_.beta_*paras_.beta_));
            }
        }
    }

    template <typename Scalar, int Dim>
    void CPDNRigid<Scalar, Dim>::P1_correlation()
    {
        Scalar min_numerics = std::numeric_limits<Scalar>::epsilon();

        for (size_t i = 0, i_end = this->P1_.rows(); i < i_end; i ++)
        {
            if (this->P1_(i, 0) < min_numerics)
                this->P1_(i, 0) = min_numerics;
        }
    }
}



#endif