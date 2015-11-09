#ifndef _BASIC_TYPES_HPP_
#define _BASIC_TYPES_HPP_

#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace regmm
{
    template <typename Scalar, int Dim>
    class Array 
    {
    public:
        Array()
        {
            assert(Dim == 2 || Dim == 3);
            _d = new Scalar[Dim];
            for (Scalar& d : _d) d = 0.0;
        }

        Array(Scalar x, Scalar y)
        {
            assert(Dim == 2 ? true : ("The Array dimension is 2!" && false));
            _d = new Scalar[Dim];
            _d[0] = x;
            _d[1] = y;
        }

        Array(Scalar x, Scalar y, Scalar z)
        {
            assert(Dim == 3 ? true : ("The Array dimension is 3!" && false));
            _d = new Scalar[Dim];
            _d[0] = x;
            _d[1] = y;
            _d[2] = z;
        }

        ~Array()
        {
            delete _d;
        }

        Array(const Array& ary)
        {
            _d = new Scalar[Dim];
            memcpy(_d, ary.data(), sizeof(Scalar)*Dim);
        }

        void operator=(const Array& ary)
        {
            _d = new Scalar[Dim];
            memcpy(_d, ary.data(), sizeof(Scalar)*Dim);
        }

        inline Scalar& x(){ return _d[0]; }
        inline Scalar& y(){ return _d[1]; }
        inline Scalar& z(){ assert(Dim == 3 ? true : ("z doesn't exists because dimension is only 2!" && false)); return _d[2]; }

        inline const Scalar* data() const { return _d; }

        Array operator+(const Array& ary)
        {
            if (Dim == 2) return Array(x()+ary.x(), y()+ary.y());
            else return Array(x()+ary.x(), y()+ary.y(), z()+ary.z());
        }

        Array operator-(const Array& ary)
        {
            if (Dim == 2) return Array(x()-ary.x(), y()-ary.y());
            else return Array(x()-ary.x(), y()-ary.y(), z()-ary.z());
        }

        Array operator*(Scalar s)
        {
            if (Dim == 2) return Array(x()*s, y()*s);
            else return Array(x()*s, y()*s, z()*s);
        }

        Array operator/(Scalar s)
        {
            if (Dim == 2) return Array(x()/s, y()/s);
            else return Array(x()/s, y()/s, z()/s);
        }


        void normalize()
        {
            Scalar s = norm();
            x() = x() / s;
            y() = y() / s;
            if (Dim == 3) z() = z() / s;
        }

        Scalar norm()
        {
            if (Dim == 2) return std::sqrt(x()*x()+y()*y());
            else return std::sqrt(x()*x()+y()*y()+z()*z());
        }

    private:
        Scalar* _d;
    };

    template<typename ArrayType, typename Scalar, int Dim>
    class Arrays
    {
    private:
        std::vector<ArrayType> data_;

    public:
        ArrayType& operator[](int index)
        {
            return data_[index];
        }

        void push_back(ArrayType& point)
        {
            data_.push_back(point);
        }

        int size() const
        {
            return data_.size();
        }
    };



    template <typename Scalar, int Dim>
    struct PtType
    {
        typedef Array<Scalar, Dim> Point;
    };

    template <typename Scalar, int Dim>
    struct VerType
    {
        typedef Array<Scalar, Dim> Vertex;
    };

    template <typename Scalar, int Dim>
    struct NmlType
    {
        typedef Array<Scalar, Dim> Normal;
    };

    struct Face
    {
        Face(int v0, int v1, int v2)
        {
            _idx[0] = v0;
            _idx[1] = v1;
            _idx[2] = v2;
        }

        int _idx[3];
        int& operator[](int idx)
        {
            return _idx[idx];
        }
    };

#define     PointType       typename PtType<Scalar, Dim>::Point
#define     VertexType      typename VerType<Scalar, Dim>::Vertex
#define     NormalType      typename NmlType<Scalar, Dim>::Normal


    template <typename Scalar, int Dim>
    struct MatrixType
    {
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;

        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Dim, Eigen::RowMajor> MatrixD;

        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix;

        typedef Eigen::SparseMatrix<Scalar> SparseMatrix;
    };

#define     TVector             typename MatrixType<Scalar, Dim>::Vector
#define     TMatrixD            typename MatrixType<Scalar, Dim>::MatrixD
#define     TMatrix             typename MatrixType<Scalar, Dim>::Matrix
#define     TSparseMatrix       typename MatrixType<Scalar, Dim>::SparseMatrix


    template <typename Scalar, int Dim>
    struct ARAPType
    {
        typedef std::vector<Scalar> WeightList;
        typedef std::vector<std::vector<int>> AdjList;
        typedef std::vector<Eigen::Vector3i> FaceList;
        typedef Eigen::SparseMatrix<double> WeightMatrix;
        typedef std::vector<Eigen::Matrix<Scalar, Dim, Dim>> RotList;
    };

#define     TCorresMatrix    TMatrix
#define     TDataMatrix      TMatrixD
#define     TModelMatrix     TMatrixD
#define     TCovMatrix       TMatrixD
#define     TWeightMatrix    TSparseMatrix
#define     TWeightList      typename ARAPType<Scalar, Dim>::WeightList
#define     TAdjList         typename ARAPType<Scalar, Dim>::AdjList
#define     TFaceList        typename ARAPType<Scalar, Dim>::FaceList
#define     TRotList         typename ARAPType<Scalar, Dim>::RotList
}

#endif