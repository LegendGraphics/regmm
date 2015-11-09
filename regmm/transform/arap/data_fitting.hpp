#ifndef _DATA_FITTING_HPP_
#define _DATA_FITTING_HPP_

#include "regmm/transform/arap/constraint.hpp"

namespace regmm
{
    template <typename Scalar, int Dim>
    class DataFitting: public Constraint<Scalar, Dim>
    {
    public:
        void build();
        void projection();
        void update();

    protected:
        void buildA();
        void buildb();

    protected:
        Scalar zero_correction(Scalar value);
    };
}

namespace regmm
{
    template <typename Scalar, int Dim>
    void DataFitting<Scalar, Dim>::build()
    {
        buildA();
        buildb();
    }

    template <typename Scalar, int Dim>
    void DataFitting<Scalar, Dim>::projection()
    {
        return;
    }

    template <typename Scalar, int Dim>
    void DataFitting<Scalar, Dim>::update()
    {
        return;
    }

    template <typename Scalar, int Dim>
    void DataFitting<Scalar, Dim>::buildA()
    {
        ARAPSolver<Scalar, Dim>::DeformModel& deform_model = ARAPSolver<Scalar, Dim>::deform_model_;
        TCovMatrix& cov_matrix = deform_model._cov_matrix;
        TCorresMatrix& corres_matrix = deform_model._data_corres;
        int ver_num = deform_model._model_matrix.rows();

        std::vector<std::vector<Eigen::Triplet<Scalar>>> diag_terms;
        // for x,y,z coordinates
        diag_terms.resize(Dim); 
        A_.resize(Dim);

        for (int i = 0; i < ver_num; ++i) 
        {
            double wi_x = 0, wi_y = 0, wi_z = 0;

            wi_x += zero_correction(ARAPSolver<Scalar, Dim>::lambda_data_fitting_*(2/cov_matrix.row(i)[0])*corres_matrix.row(i).sum());
            wi_y += zero_correction(ARAPSolver<Scalar, Dim>::lambda_data_fitting_*(2/cov_matrix.row(i)[1])*corres_matrix.row(i).sum());
            if (Dim == 3) wi_z += zero_correction(ARAPSolver<Scalar, Dim>::lambda_data_fitting_*(2/cov_matrix.row(i)[2])*corres_matrix.row(i).sum());

            diag_terms[0].push_back(Eigen::Triplet<Scalar>(i, i, wi_x));
            diag_terms[1].push_back(Eigen::Triplet<Scalar>(i, i, wi_y));
            if (Dim == 3) diag_terms[2].push_back(Eigen::Triplet<Scalar>(i, i, wi_z));
        }

        TSparseMatrix diag_coeff_x(ver_num, ver_num);
        TSparseMatrix diag_coeff_y(ver_num, ver_num);
        TSparseMatrix diag_coeff_z(ver_num, ver_num);
        diag_coeff_x.setFromTriplets(diag_terms[0].begin(), diag_terms[0].end());
        diag_coeff_y.setFromTriplets(diag_terms[1].begin(), diag_terms[1].end());
        if (Dim == 3) diag_coeff_z.setFromTriplets(diag_terms[2].begin(), diag_terms[2].end());

        A_[0] = diag_coeff_x;
        A_[1] = diag_coeff_y;
        if (Dim == 3) A_[2] = diag_coeff_z;
    }

    template <typename Scalar, int Dim>
    void DataFitting<Scalar, Dim>::buildb()
    {
        ARAPSolver<Scalar, Dim>::DeformModel& deform_model = ARAPSolver<Scalar, Dim>::deform_model_;
        TModelMatrix& origin_model = deform_model._origin_model;
        TDataMatrix& data_matrix = deform_model._data_matrix;
        TCorresMatrix& corres_matrix = deform_model._data_corres;
        TCovMatrix& cov_matrix = deform_model._cov_matrix;
        int ver_num = origin_model.rows();

        b_.resize(ver_num, Dim);

        for (size_t i = 0; i < ver_num; ++ i)
        {
            TVector weight_cloud = TVector::Zero(Dim);
            for (size_t n = 0, n_end = corres_matrix.cols(); n < n_end; ++ n)
            {
                weight_cloud += corres_matrix(i, n)*data_matrix.row(n);
            }

            b_.row(i) = ARAPSolver<Scalar, Dim>::lambda_data_fitting_*2*cov_matrix.row(i).asDiagonal().inverse()*weight_cloud;
        }
    }

    template <typename Scalar, int Dim>
    Scalar DataFitting<Scalar, Dim>::zero_correction(Scalar value)
    {
        Scalar min_double = std::numeric_limits<Scalar>::min();
        if (min_double > value)
            return min_double;
        return value;
    }
}

#endif