#ifndef _ARAP_HPP_
#define _ARAP_HPP_

#include <Eigen/SVD>
#include "regmm/transform/arap/constraint.hpp"

namespace regmm
{
    template <typename Scalar, int Dim>
    class ARAP: public Constraint<Scalar, Dim>
    {
    public:
        void build();
        void projection();
        void update();

    protected:
        void buildA();
        void buildb();

    protected:
        void initRotation();
        void updateRotation();
    };
}

namespace regmm
{
    template <typename Scalar, int Dim>
    void ARAP<Scalar, Dim>::build()
    {
        initRotation();
        buildA();
        buildb();
    }

    template <typename Scalar, int Dim>
    void ARAP<Scalar, Dim>::projection()
    {
        updateRotation();
        return;
    }

    template <typename Scalar, int Dim>
    void ARAP<Scalar, Dim>::update()
    {
        buildb();
    }

    template <typename Scalar, int Dim>
    void ARAP<Scalar, Dim>::buildA()
    {
        ARAPSolver<Scalar, Dim>::DeformModel& deform_model = ARAPSolver<Scalar, Dim>::deform_model_;
        TCovMatrix& cov_matrix = deform_model._cov_matrix;
        TWeightMatrix& weight_matrix = deform_model._weight_matrix;
        TAdjList& adj_list = deform_model._adj_list;
        int ver_num = deform_model._model_matrix.rows();

        std::vector<std::vector<Eigen::Triplet<Scalar>>> weight_sums;
        // for x,y,z coordinates
        weight_sums.resize(Dim);
        A_.resize(Dim);

        for (int i = 0; i < ver_num; ++i) 
        {
            double wi_x = 0, wi_y = 0, wi_z = 0;

            for (size_t j = 0, j_end = deform_model._adj_list[i].size(); j < j_end; ++j)
            {
                int id_j = deform_model._adj_list[i][j];
                wi_x += weight_matrix.coeffRef(i, id_j);
                wi_y += weight_matrix.coeffRef(i, id_j);
                if (Dim == 3) wi_z += weight_matrix.coeffRef(i, id_j);
            }

            weight_sums[0].push_back(Eigen::Triplet<Scalar>(i, i, wi_x));
            weight_sums[1].push_back(Eigen::Triplet<Scalar>(i, i, wi_y));
            if (Dim == 3) weight_sums[2].push_back(Eigen::Triplet<Scalar>(i, i, wi_z));
        }

        TSparseMatrix diag_coeff_x(ver_num, ver_num);
        TSparseMatrix diag_coeff_y(ver_num, ver_num);
        TSparseMatrix diag_coeff_z(ver_num, ver_num);
        diag_coeff_x.setFromTriplets(weight_sums[0].begin(), weight_sums[0].end());
        diag_coeff_y.setFromTriplets(weight_sums[1].begin(), weight_sums[1].end());
        if (Dim == 3) diag_coeff_z.setFromTriplets(weight_sums[2].begin(), weight_sums[2].end());

        A_[0] = diag_coeff_x - weight_matrix;
        A_[1] = diag_coeff_y - weight_matrix;
        if (Dim == 3) A_[2] = diag_coeff_z - weight_matrix;
    }

    template <typename Scalar, int Dim>
    void ARAP<Scalar, Dim>::buildb()
    {
        ARAPSolver<Scalar, Dim>::DeformModel& deform_model = ARAPSolver<Scalar, Dim>::deform_model_;
        TModelMatrix& origin_model = deform_model._origin_model;
        TAdjList& adj_list = deform_model._adj_list;
        TWeightMatrix& weight_matrix = deform_model._weight_matrix;
        TRotList& R_list = deform_model._R_list;
        int ver_num = origin_model.rows();

        b_.resize(ver_num, Dim);

        for (size_t i = 0; i < ver_num; ++i) 
        {
            b_.row(i) = Eigen::Matrix<Scalar, Dim, 1>::Zero();
            for (size_t j = 0, j_end = adj_list[i].size(); j < j_end; ++j)
            {
                b_.row(i) += (weight_matrix.coeffRef(i, adj_list[i][j])/2) * 
                    (origin_model.row(i) - origin_model.row(adj_list[i][j])) * (R_list[i]+R_list[adj_list[i][j]]).transpose();
            }
        }
    }

    template <typename Scalar, int Dim>
    void ARAP<Scalar, Dim>::initRotation()
    {
        ARAPSolver<Scalar, Dim>::DeformModel& deform_model = ARAPSolver<Scalar, Dim>::deform_model_;
        TModelMatrix& origin_model = deform_model._origin_model;
        TRotList R_list;

        for (size_t j = 0, j_end = origin_model.rows(); j < j_end; ++ j)
        {
            R_list.push_back(Eigen::Matrix<Scalar, Dim, Dim>::Identity());
        }

        deform_model._R_list = R_list;
    }

    template <typename Scalar, int Dim>
    void ARAP<Scalar, Dim>::updateRotation()
    {
        ARAPSolver<Scalar, Dim>::DeformModel& deform_model = ARAPSolver<Scalar, Dim>::deform_model_;
        TModelMatrix& origin_model = deform_model._origin_model;
        TModelMatrix& model_matrix = deform_model._model_matrix;
        TRotList& rot_list = deform_model._R_list;
        TAdjList& adj_list = deform_model._adj_list;
        TWeightMatrix& weight_matrix = deform_model._weight_matrix;

        TMatrix Si;
        TMatrix Di;

        TMatrixD Pi_Prime;
        TMatrixD Pi;

        for (size_t i = 0, i_end = rot_list.size(); i < i_end; ++i) 
        {
            Di = TMatrix::Zero(adj_list[i].size(), adj_list[i].size());
            Pi_Prime.resize(adj_list[i].size(), Dim);
            Pi.resize(adj_list[i].size(), Dim);

            for (size_t j = 0, j_end = adj_list[i].size(); j < j_end; ++j) 
            {
                Di(j, j) = weight_matrix.coeffRef(i, adj_list[i][j]);
                Pi.row(j) = origin_model.row(i) - origin_model.row(adj_list[i][j]);
                Pi_Prime.row(j) = model_matrix.row(i) - model_matrix.row(adj_list[i][j]);
            }

            Si = Pi.transpose() * Di * Pi_Prime;
            TMatrix Ui;
            TMatrix Vi;
            Eigen::JacobiSVD<TMatrix> svd(Si, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Ui = svd.matrixU();
            Vi = svd.matrixV();
            Scalar det_uv = (Ui * Vi.transpose()).determinant();
            TMatrix C = TMatrix::Identity(Dim, Dim);
            C(Dim-1, Dim-1) = det_uv;
            rot_list[i] = Ui * C * Vi.transpose();
        }
    }
}

#endif