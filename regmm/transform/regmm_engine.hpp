#ifndef _REGMM_ENGINE_HPP_
#define _REGMM_ENGINE_HPP_

#include "regmm/transform/cpd/cpd_rigid.hpp"
#include "regmm/transform/cpd/cpd_nonrigid.hpp"
#include "regmm/transform/arap/arap_solver.hpp"

namespace regmm
{
    template <typename Scalar, int Dim>
    class RegmmEngine
    {
    public:
        RegmmEngine();
        virtual ~RegmmEngine();

    public:
        void initEngine(RegType rt, DataType dt);

        void setSource(PointSet& source);
        void setTarget(PointSet& target);

        void setSource(MeshObject& source);
        void setTarget(MeshObject& target);

    public:
        void setIterativeNumber(int iter_num);

    public:
        void setVarianceTolerance(Scalar v_tol);
        void setEnergyTolerance(Scalar e_tol);
        void setOutlierWeight(Scalar w);
        void setFgtFlag(bool fgt);
        void setFgtEpsilon(Scalar fgt_eps);
        void setLowRankFlag(bool lr);
        void setKLowRank(int K);
        void setLRMaxIteration(size_t lr_maxitr);

    public:
        void setEpsilon(Scalar eps);
        void setDataFittingWeight(Scalar data_fitting);
        void setARAPWeight(Scalar arap);
        void setNoise(Scalar noise_p);

    public:
        void compute();

    private:
        RegType         reg_type_;
        DataType        src_type_;

    private:
        CPDBase<Scalar, Dim>*      cpdbase_part_;
        ARAPSolver<Scalar, Dim>*    arapsolver_part_;
    };
}

namespace regmm
{
    template <typename Scalar, int Dim>
    RegmmEngine<Scalar, Dim>::RegmmEngine()
        :cpdbase_part_(nullptr),
        arapsolver_part_(nullptr)
    {

    }

    template <typename Scalar, int Dim>
    RegmmEngine<Scalar, Dim>::~RegmmEngine()
    {
        delete cpdbase_part_;
        delete arapsolver_part_;
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::initEngine(RegType rt, DataType dt)
    {
        delete cpdbase_part_;
        delete arapsolver_part_;

        reg_type_ = rt;
        src_type_ = dt;

        if (reg_type_ == RIGID && src_type_ == POINT_CLOUD)
        {
            cpdbase_part_ = new CPDRigid<Scalar, Dim>;
            cpdbase_part_->setRegType(reg_type_);
            cpdbase_part_->setDataType(src_type_);
        }

        else if (reg_type_ == NONRIGID && src_type_ == POINT_CLOUD)
        {
            cpdbase_part_ = new CPDNRigid<Scalar, Dim>;
            cpdbase_part_->setRegType(reg_type_);
            cpdbase_part_->setDataType(src_type_);
        }

        else if (reg_type_ == RIGID && src_type_ == MESH)
        {
            cpdbase_part_ = new CPDRigid<Scalar, Dim>;
            cpdbase_part_->setRegType(reg_type_);
            cpdbase_part_->setDataType(src_type_);
        }

        else
        {
            arapsolver_part_ = new ARAPSolver<Scalar, Dim>;
            arapsolver_part_->setRegType(reg_type_);
            arapsolver_part_->setDataType(src_type_);
        }
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setSource(PointSet& source)
    {
        if (reg_type_ == NONRIGID && src_type_ == MESH)
            arapsolver_part_->setSource(source);
        else 
            cpdbase_part_->setSource(source);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setSource(MeshObject& source)
    {
        if (reg_type_ == NONRIGID && src_type_ == MESH)
            arapsolver_part_->setSource(source);
        else 
            cpdbase_part_->setSource(source);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setTarget(PointSet& target)
    {
        if (reg_type_ == NONRIGID && src_type_ == MESH)
            arapsolver_part_->setTarget(target);
        else 
            cpdbase_part_->setTarget(target);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setTarget(MeshObject& target)
    {
        if (reg_type_ == NONRIGID && src_type_ == MESH)
            arapsolver_part_->setTarget(target);
        else 
            cpdbase_part_->setTarget(target);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setIterativeNumber(int iter_num)
    {
        if (reg_type_ == NONRIGID && src_type_ == MESH)
            arapsolver_part_->setIterativeNumber(iter_num);
        else 
            cpdbase_part_->setIterativeNumber(iter_num);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setVarianceTolerance(Scalar v_tol)
    {
        assert(arapsolver_part_ == nullptr);
        cpdbase_part_->setVarianceTolerance(v_tol);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setEnergyTolerance(Scalar e_tol)
    {
        assert(arapsolver_part_ == nullptr);
        cpdbase_part_->setEnergyTolerance(e_tol);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setOutlierWeight(Scalar w)
    {
        assert(arapsolver_part_ == nullptr);
        cpdbase_part_->setOutlierWeight(w);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setFgtFlag(bool fgt)
    {
        assert(arapsolver_part_ == nullptr);
        cpdbase_part_->setFgtFlag(fgt);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setFgtEpsilon(Scalar fgt_eps)
    {
        assert(arapsolver_part_ == nullptr);
        cpdbase_part_->setFgtEpsilon(fgt_eps);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setLowRankFlag(bool lr)
    {
        assert(arapsolver_part_ == nullptr);
        cpdbase_part_->setLowRankFlag(lr);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setKLowRank(int K)
    {
        assert(arapsolver_part_ == nullptr);
        cpdbase_part_->setKLowRank(K);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setLRMaxIteration(size_t lr_maxitr)
    {
        assert(arapsolver_part_ == nullptr);
        cpdbase_part_->setLRMaxIteration(lr_maxitr);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setNoise(Scalar noise_p)
    {
        assert(cpdbase_part_ == nullptr);
        arapsolver_part_->setNoise(noise_p);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setEpsilon(Scalar eps)
    {
        assert(cpdbase_part_ == nullptr);
        arapsolver_part_->setEpsilon(eps);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setDataFittingWeight(Scalar data_fitting)
    {
        assert(cpdbase_part_ == nullptr);
        arapsolver_part_->setDataFittingWeight(data_fitting);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::setARAPWeight(Scalar arap)
    {
        assert(cpdbase_part_ == nullptr);
        arapsolver_part_->setARAPWeight(arap);
    }

    template <typename Scalar, int Dim>
    void RegmmEngine<Scalar, Dim>::compute()
    {
        if (cpdbase_part_ != nullptr)
            cpdbase_part_->compute();
        
        if (arapsolver_part_ != nullptr)
            arapsolver_part_->compute();
    }
}


#endif