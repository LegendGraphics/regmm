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

#ifndef _CPD_RIGID_HPP_
#define _CPD_RIGID_HPP_

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#include <vector>

#include <Eigen/SVD>
#include <Eigen/LU>

#include "regmm/transform/cpd/cpd_base.hpp"
#include "regmm/transform/cpd/parameters.hpp"
#include "regmm/transform/cpd/fgt_wrapper.hpp"
//#include "disp/render_thread.hpp"

namespace regmm
{
    template <typename Scalar, int Dim>
    class CPDRigid: public CPDBase<Scalar, Dim>
    {
    public:
        CPDRigid();
        virtual ~CPDRigid();

        inline RigidParas<Scalar, Dim>& getParameters() const { return paras_; }

        void compute();

    private:
        void initialization();
        void e_step();
        void m_step();
        void align();
        void correspondences();

        Scalar computeGaussianExp(size_t m, size_t n);
        Scalar energy();

    private:
        RigidParas<Scalar, Dim>    paras_;
    };
}

namespace regmm
{
    template <typename Scalar, int Dim>
    CPDRigid<Scalar, Dim>::CPDRigid()
    {
    }

    template <typename Scalar, int Dim>
    CPDRigid<Scalar, Dim>::~CPDRigid(){}

    template <typename Scalar, int Dim>
    void CPDRigid<Scalar, Dim>::compute()
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
            e_tol = fabs((e - old_e) / e);

            m_step();
            align();

            /* if (this->_vision == true)
            RenderThread<Scalar, Dim>::instance()->updateModel(this->T_);*/

            iter_num ++;
        }
        
        correspondences();
        this->updateModel();
        this->denormalize();
        this->rewriteOriginalSource();
        /*RenderThread<Scalar, Dim>::instance()->cancel();*/
    }

    template <typename Scalar, int Dim>
    void CPDRigid<Scalar, Dim>::initialization()
    {
        // determine data number
        this->M_ = this->model_.rows();
        this->N_ = this->data_.rows();

        // initialization
        paras_.R_ = TMatrix::Identity(Dim, Dim);
        paras_.t_ = TVector::Zero(Dim, 1);
        paras_.s_ = 1;

        Scalar sigma_sum = this->M_*(this->data_.transpose()*this->data_).trace() + 
            this->N_*(this->model_.transpose()*this->model_).trace() - 
            2*(this->data_.colwise().sum())*(this->model_.colwise().sum()).transpose();
        paras_.sigma2_ = sigma_sum / (Dim*this->N_*this->M_);

        this->initTransform();
    }

    

    template <typename Scalar, int Dim>
    void CPDRigid<Scalar, Dim>::correspondences()
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
    void CPDRigid<Scalar, Dim>::e_step()
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

            TMatrix aX = TMatrix::Zero(this->N_, Dim);
            for (size_t i = 0; i < Dim; i ++)
            {
                aX.col(i) = this->data_.col(i).cwiseProduct(a);
            }

            this->PT1_ = TVector(this->N_).setOnes() - c * a;
            this->P1_ = fgt<Scalar, Dim>(this->data_, this->T_, a, sqrt(2*paras_.sigma2_), this->fgt_eps_);
            this->PX_ = fgt<Scalar, Dim>(this->data_, this->T_, aX, sqrt(2*paras_.sigma2_), this->fgt_eps_);
        }
    }

    template <typename Scalar, int Dim>
    void CPDRigid<Scalar, Dim>::m_step()
    {
        Scalar N_P = this->P1_.sum();
        TVector mu_x = this->data_.transpose() * this->PT1_ / N_P;
        TVector mu_y = this->model_.transpose() * this->P1_ / N_P;

        TMatrixD X_hat = this->data_ - TMatrix(TVector(this->N_).setOnes() * mu_x.transpose());
        TMatrixD Y_hat = this->model_ - TMatrix(TVector(this->M_).setOnes() * mu_y.transpose());

        TMatrix A = (this->PX_-this->P1_*mu_x.transpose()).transpose() * 
            (this->model_ - TMatrix(TVector(this->M_).setOnes() * mu_y.transpose()));

        Eigen::JacobiSVD<TMatrix> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        TMatrix U = svd.matrixU();
        TMatrix V = svd.matrixV();
        Scalar det_uv = TMatrix(U*V.transpose()).determinant();
        TMatrix C = TMatrix::Identity(Dim, Dim);
        C(Dim-1, Dim-1) = det_uv;
        paras_.R_ = U * C * V.transpose();

        Scalar s_upper = TMatrix(A.transpose()*paras_.R_).trace();
        Scalar s_lower = TMatrix(Y_hat.transpose()*this->P1_.asDiagonal()*Y_hat).trace();
        paras_.s_ =  s_upper / s_lower; 
            
        paras_.t_ = mu_x - paras_.s_ * paras_.R_ * mu_y;

        Scalar tr_f = TMatrix(X_hat.transpose()*this->PT1_.asDiagonal()*X_hat).trace();
        Scalar tr_b = TMatrix(A.transpose()*paras_.R_).trace();
        paras_.sigma2_ = (tr_f - paras_.s_ * tr_b) / (N_P * Dim);
        paras_.sigma2_ = fabs(paras_.sigma2_);

    }

    template <typename Scalar, int Dim>
    void CPDRigid<Scalar, Dim>::align()
    {
        this->T_ = (paras_.s_) * (this->model_) * (paras_.R_).transpose() + 
            TVector(this->M_).setOnes() * (paras_.t_).transpose();
    }

    template <typename Scalar, int Dim>
    Scalar CPDRigid<Scalar, Dim>::computeGaussianExp(size_t m, size_t n)
    {
        TVector vec = TVector(this->T_.row(m) - this->data_.row(n));
        Scalar g_exp = exp(-vec.squaredNorm()/(2*paras_.sigma2_));
        return g_exp;
    }

    template <typename Scalar, int Dim>
    Scalar CPDRigid<Scalar, Dim>::energy() 
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
}
#endif