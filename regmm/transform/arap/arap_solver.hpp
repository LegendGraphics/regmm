#ifndef _ARAP_SOLVER_HPP_
#define _ARAP_SOLVER_HPP_

#include <array>
#include <set>

#include <Eigen/LU>

#include "regmm/transform/registrator.hpp"
#include "regmm/transform/arap/arap.hpp"
#include "regmm/transform/arap/data_fitting.hpp"

namespace regmm
{
    template <typename Scalar, int Dim>
    class ARAPSolver: public Registrator<Scalar, Dim>
    {
    public:
        typedef struct 
        {
            TModelMatrix            _origin_model;
            TModelMatrix            _model_matrix;
            TDataMatrix             _data_matrix;

            TCorresMatrix           _data_corres;
            TCovMatrix              _cov_matrix;

            TWeightList             _data_weights;


            TAdjList                _adj_list;
            TFaceList               _face_list;
            TWeightMatrix           _weight_matrix;
            TRotList                _R_list;

            void findSharedVertex(int pi, int pj, std::vector<int>& share_vertex)
            {
                std::vector<int> vertices;
                set_intersection(_adj_list[pi].begin(), _adj_list[pi].end(), _adj_list[pj].begin(), _adj_list[pj].end(), back_inserter(vertices));
                for (auto &i : vertices) 
                {
                    std::vector<int> f;
                    f.push_back(pi);
                    f.push_back(pj);
                    f.push_back(i);
                    sort(f.begin(), f.end());
                    std::vector<Eigen::Vector3i>::iterator it = std::find(_face_list.begin(), _face_list.end(), Eigen::Map<Eigen::Vector3i>(&f[0]));
                    if (it != _face_list.end()) {
                        if ((*it)(0) != pi && (*it)(0) != pj) share_vertex.push_back((*it)(0));
                        else if ((*it)(1) != pi && (*it)(1) != pj) share_vertex.push_back((*it)(1));
                        else share_vertex.push_back((*it)(2));
                    }
                }
                if (share_vertex.size() > 2) {
                    std::cout << "share vertices number warning: " << share_vertex.size() << std::endl;
                }
            }

            Scalar wij(int pi, int pj, int si, int sj = -1)
            {
                TVector p1 = _origin_model.row(pi);
                TVector p2 = _origin_model.row(pj);
                TVector p3 = _origin_model.row(si);

                Scalar e12 = (p1 - p2).norm();
                Scalar e13 = (p1 - p3).norm();
                Scalar e23 = (p2 - p3).norm();
                Scalar alpha_cos = fabs((e23*e23 + e13*e13 - e12*e12)/(2*e23*e13));

                Scalar beta_cos = 0;
                if (sj != -1)
                {
                    TVector p4 = _origin_model.row(sj);
                    Scalar e14 = (p1 - p4).norm();
                    Scalar e24 = (p2 - p4).norm();
                    beta_cos = fabs((e14*e14+e24*e24-e12*e12)/(2*e14*e24));
                }

                return ((alpha_cos/sqrt(1-alpha_cos*alpha_cos))+(beta_cos/sqrt(1-beta_cos*beta_cos)))/2;
            }

        }DeformModel;

    public:
        ARAPSolver();
        virtual ~ARAPSolver();

    public:
        void setIterativeNumber(int iter_num);
        void setEpsilon(Scalar eps);
        void setDataFittingWeight(Scalar data_fitting);
        void setARAPWeight(Scalar arap);
        void setNoise(Scalar noise_p);

    public:
        void compute();

    protected:
        void deform();
        void deforming();

        void e_step();
        Scalar m_step();

        void initBuild();
        void left_sys();
        void right_sys();

        Scalar solve();
        Scalar energy();

        void projection();
        void update();

        Scalar gaussian(int m_id, int c_id);

    protected:
        void init();
        void initFittingParas();
        void initMeshParas();
        Scalar zero_correction(Scalar value);

    private:
        DataFitting<Scalar, Dim> data_term_;
        ARAP<Scalar, Dim> arap_term_;

        std::vector<TSparseMatrix> A_; // x, y, z
        TMatrixD b_;

        Eigen::SparseLU<TSparseMatrix> lu_solver_;

    public:
        static DeformModel deform_model_;

    public:
        static int iter_num_;
        static Scalar eps_;

        static Scalar lambda_data_fitting_;
        static Scalar lambda_arap_;
        static Scalar noise_p_;
    };
}


namespace regmm
{
    template <typename Scalar, int Dim>
    int ARAPSolver<Scalar, Dim>::iter_num_ = 50;

    template <typename Scalar, int Dim>
    Scalar ARAPSolver<Scalar, Dim>::eps_ = 1e-3;
   
    template <typename Scalar, int Dim>
    Scalar ARAPSolver<Scalar, Dim>::lambda_data_fitting_ = 1e-2;
 
    template <typename Scalar, int Dim>
    Scalar ARAPSolver<Scalar, Dim>::lambda_arap_ = 1;
   
    template <typename Scalar, int Dim>
    Scalar ARAPSolver<Scalar, Dim>::noise_p_ = 0.0;

    template <typename Scalar, int Dim>
    typename ARAPSolver<Scalar, Dim>::DeformModel ARAPSolver<Scalar, Dim>::deform_model_;

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::setIterativeNumber(int iter_num)
    {
        iter_num_ = iter_num;
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::setEpsilon(Scalar eps)
    {
        eps_ = eps;
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::setDataFittingWeight(Scalar data_fitting)
    {
        lambda_data_fitting_ = data_fitting;
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::setARAPWeight(Scalar arap)
    {
        lambda_arap_ = arap;
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::setNoise(Scalar noise_p)
    {
        noise_p_ = noise_p;
    }

    template <typename Scalar, int Dim>
    ARAPSolver<Scalar, Dim>::ARAPSolver()
    {

    }

    template <typename Scalar, int Dim>
    ARAPSolver<Scalar, Dim>::~ARAPSolver()
    {

    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::compute()
    {
        init();

        deform();
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::deform()
    {
        int iter_num = 0;
        double eps = 1;

        double e = 0;

        std::cout << "Start EM Iteration..." << std::endl;

        do 
        {
            e_step();

            double e_n = m_step();

            eps = std::fabs((e_n - e) / e_n);
            e = e_n;

            std::cout << "In EM Iteration \t" << "iter: " << ++ iter_num << "\tdelta: " << eps << std::endl;

        } while (iter_num < iter_num_ && eps > eps_ );

        deforming();
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::deforming()
    {
        TModelMatrix& pm = deform_model_._model_matrix;
        this->model_ = pm;
        
        // updateNormals();

        rewriteOriginalSource();
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::e_step()
    {
        std::cout << "E-Step:" << std::endl;


        TCorresMatrix& corres_mat = deform_model_._data_corres;
        TWeightList& weight_list = deform_model_._data_weights;

        for (size_t i = 0, i_end = corres_mat.cols(); i < i_end; ++ i)
        {
            for (size_t j = 0, j_end = corres_mat.rows(); j < j_end; ++ j)
            {
                corres_mat(j, i) = gaussian(j, i) * weight_list[j];
            }
        }

        for (size_t i = 0, i_end = corres_mat.cols(); i < i_end; ++ i)
        {
            Scalar sum_gaussian = corres_mat.col(i).sum() + noise_p_;

            for (size_t j = 0, j_end = corres_mat.rows(); j < j_end; ++ j)
            {
                corres_mat(j, i) = corres_mat(j, i) / zero_correction(sum_gaussian);
            }
        }

    }

    template <typename Scalar, int Dim>
    Scalar ARAPSolver<Scalar, Dim>::m_step()
    {
        std::cout << "M-Step:" << std::endl;

        TCorresMatrix& corres_mat = deform_model_._data_corres;
        TWeightList& weight_list = deform_model_._data_weights;
        for (size_t i = 0, i_end = weight_list.size(); i < i_end; ++ i)
        {
            weight_list[i] = corres_mat.row(i).sum() / corres_mat.cols();
        }


        // update expectation
        int iter = 0;
        Scalar eps = 0;

        Scalar e = 0;

        initBuild();
        left_sys();
        right_sys();

        do {
            Scalar e_n = solve();
            eps = std::fabs((e_n - e) / e_n);
            e = e_n;

            projection();
            update();
            right_sys(); // the update only effects right side 

            iter ++;

        }while(eps > eps_ && iter < iter_num_);

        return e;
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::init()
    {
        A_.resize(Dim);

        initMeshParas();

        initFittingParas();

        std::cout << "finish solver initialization" << std::endl;
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::initFittingParas()
    {
        // init data matrix
        deform_model_._data_matrix = this->data_;

        // init model matrix
        deform_model_._model_matrix = this->model_;

        // init origin matrix
        deform_model_._origin_model = this->model_;

        // init data correspondence matrix
        deform_model_._data_corres = TCorresMatrix::Zero(this->M_, this->N_);

        // init inner weight 
        deform_model_._data_weights = std::vector<Scalar>(this->M_, 1.0 / this->M_);
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::initMeshParas()
    {
        // init origin matrix
        deform_model_._origin_model = this->model_;

        // init adjacent list
        const size_t pre_ver_num = 20000;
        const size_t ver_num = this->M_;

        if (pre_ver_num < ver_num)
        {
            std::cerr << "vertice number exceeds pre-setting!" << std::endl;
            return;
        }

        FacesArray& fs = this->mo_src_->getFaces();
        std::array<std::set<int>, pre_ver_num > index_list;

        for (size_t i = 0, i_end = fs.size(); i < i_end; i ++)
        {
            Face& face = fs[i];
            int x = face[0];
            int y = face[1];
            int z = face[2];

            index_list[x].insert(y);
            index_list[x].insert(z);

            index_list[y].insert(x);
            index_list[y].insert(z);

            index_list[z].insert(x);
            index_list[z].insert(y);
        }

        for (size_t i = 0, i_end = ver_num; i < i_end; i ++)
        {
            std::set<int> index_i = index_list.at(i);
            std::vector<int> adj_list(index_i.begin(), index_i.end());
            std::sort(adj_list.begin(), adj_list.end());
            deform_model_._adj_list.push_back(adj_list);
        }


        // init face list
        std::vector<std::vector<int>> triangle_list;
        for (size_t i = 0, i_end = fs.size(); i < i_end; i ++)
        {
            Face& face = fs[i];
            std::vector<int> triangle;
            triangle.push_back(face[0]);
            triangle.push_back(face[1]);
            triangle.push_back(face[2]);
            triangle_list.push_back(triangle);
        }

        TFaceList& face_list = deform_model_._face_list;
        face_list.clear();
        for (size_t i = 0, i_end = triangle_list.size(); i < i_end; ++ i)
        {
            std::vector<int> face = triangle_list[i];
            std::sort(face.begin(), face.end());
            assert(face.size() == 3);
            face_list.push_back(Eigen::Vector3i(face[0], face[1], face[2]));
        }


        // init weight matrix
        std::vector<Eigen::Triplet<Scalar>> weight_list;
        TWeightMatrix& weight_matrix = deform_model_._weight_matrix;

        for (size_t i = 0; i != ver_num; ++i) 
        {
            for (size_t j = 0, j_end = deform_model_._adj_list[i].size(); j != j_end; ++j) 
            {
                int id_j = deform_model_._adj_list[i][j];

                std::vector<int> share_vertex;
                deform_model_.findSharedVertex(i, id_j, share_vertex);

                Scalar wij = 0;
                if (share_vertex.size()==2) wij = deform_model_.wij(i, id_j, share_vertex[0], share_vertex[1]);
                else wij = deform_model_.wij(i, id_j, share_vertex[0]);

                weight_list.push_back(Eigen::Triplet<Scalar>(i, id_j, wij));
            }
        }
        weight_matrix.resize(ver_num, ver_num);
        weight_matrix.setFromTriplets(weight_list.begin(), weight_list.end());


        // init covariance matrix 
        TAdjList& adj_list = deform_model_._adj_list;
        TModelMatrix& origin_model = deform_model_._origin_model;
        TCovMatrix& cov_matrix = deform_model_._cov_matrix;
        cov_matrix.resize(this->M_, 3);

        for (size_t k = 0, k_end = adj_list.size(); k < k_end; ++ k)
        {
            TVector c = origin_model.row(k);
            int adj_size = adj_list[k].size();
            Scalar s_x = 0, s_y = 0, s_z = 0;

            for (size_t j = 0, j_end = adj_size; j < j_end; ++ j)
            {
                int id_j = adj_list[k][j];
                TVector v = origin_model.row(id_j);

                s_x += fabs(c[0] - v[0]);
                s_y += fabs(c[1] - v[1]);
                s_z += fabs(c[2] - v[2]);
            }

            cov_matrix.row(k) << pow(s_x/adj_size, 2.0), pow(s_y/adj_size, 2.0), pow(s_z/adj_size, 2.0); 
        }
    }

    template <typename Scalar, int Dim>
    Scalar ARAPSolver<Scalar, Dim>::zero_correction(Scalar value)
    {
        Scalar min_double = std::numeric_limits<Scalar>::min();
        if (min_double > value)
            return min_double;
        return value;
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::initBuild()
    {
        data_term_.build();
        arap_term_.build();
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::left_sys()
    {
        for (int i = 0; i < Dim; ++ i)
        {
            A_[i] = data_term_.A()[i] + arap_term_.A()[i];
        }
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::right_sys()
    {
        b_ = data_term_.b() + arap_term_.b();
    }

    template <typename Scalar, int Dim>
    Scalar ARAPSolver<Scalar, Dim>::solve()
    {
        for (size_t i = 0; i < Dim; ++ i)
        {
            /*TMatrix A = A_[i];
            TVector b = b_.col(i).transpose();

            TVector next_values = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);*/

            lu_solver_.analyzePattern(A_[i]);
            lu_solver_.factorize(A_[i]);
            TVector next_pos = lu_solver_.solve(b_.col(i));
            deform_model_._model_matrix.col(i) = next_pos;
        }

        return energy();
    }

    template <typename Scalar, int Dim>
    Scalar ARAPSolver<Scalar, Dim>::energy()
    {
        DeformModel& deform_model = deform_model_;
        TCorresMatrix& data_corres = deform_model._data_corres;
        TDataMatrix& data_matrix = deform_model._data_matrix;
        TModelMatrix& model_matrix = deform_model._model_matrix;
        TCovMatrix& cov_matrix = deform_model._cov_matrix;
        TModelMatrix& origin_model = deform_model._origin_model;
        TRotList& rot_list = deform_model._R_list;
        TWeightMatrix& weight_matrix = deform_model._weight_matrix;
        TAdjList& adj_list = deform_model._adj_list;


        Scalar e1 = 0, e2 = 0;

        for (size_t k = 0, k_end = model_matrix.rows(); k < k_end; ++ k)
        {

            for (size_t n = 0, n_end = data_corres.cols(); n < n_end; ++ n)
            {
                TVector cm = data_matrix.row(n) - model_matrix.row(k);
                e1 += data_corres(k, n) * cm.transpose() * cov_matrix.row(k).asDiagonal().inverse() * cm;
            }

            for (size_t j = 0, j_end = adj_list[k].size(); j < j_end; ++ j)
            {
                TVector mkj = model_matrix.row(k) - model_matrix.row(adj_list[k][j]);
                TVector pre_mkj = origin_model.row(k) - origin_model.row(adj_list[k][j]);
                e2 += weight_matrix.coeffRef(k, adj_list[k][j]) * (mkj-rot_list[k]*pre_mkj).squaredNorm();
            }
        }

        return (ARAPSolver<Scalar, Dim>::lambda_data_fitting_ * e1 + ARAPSolver<Scalar, Dim>::lambda_arap_ * e2);
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::projection()
    {
        data_term_.projection();
        arap_term_.projection();
    }

    template <typename Scalar, int Dim>
    void ARAPSolver<Scalar, Dim>::update()
    {
        data_term_.update();
        arap_term_.update();
    }

    template <typename Scalar, int Dim>
    Scalar ARAPSolver<Scalar, Dim>::gaussian(int m_id, int c_id)
    {
        Scalar p;

        TCovMatrix& cov_mat = deform_model_._cov_matrix;
        TDataMatrix& data_mat = deform_model_._data_matrix;
        TModelMatrix& model_mat = deform_model_._model_matrix;

        TVector xu = data_mat.row(c_id) - model_mat.row(m_id);
        p = pow(2*M_PI, -3/2.0) * pow((cov_mat.row(m_id).asDiagonal()).toDenseMatrix().determinant(), -1/2.0) * 
            exp((-1/2.0)*xu.transpose()*cov_mat.row(m_id).asDiagonal().inverse()*xu);

        return p;
    }
}
#endif