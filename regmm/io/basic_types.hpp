#ifndef _BASIC_TYPES_HPP_
#define _BASIC_TYPES_HPP_

#include <vector>

#include <Eigen/Core>

namespace regmm
{
    template <typename Scalar, int Dim>
    class Array 
    {
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
            delete[] _d;
        }

        Scalar& x(){ return _d[0]; }
        Scalar& y(){ return _d[1]; }
        Scalar& z(){ assert(Dim == 3 ? true : ("z doesn't exists because dimension is only 2!" && false)); return _d[2]; }

        void normalize()
        {

        }

        void length()
        {

        }

    private:
        Scalar _d[];
    };

    template<typename ArrayType, typename Scalar, int Dim>
    class Arrays
    {
    private:
        std::vector<ArrayType> _data;

    public:
        ArrayType& operator[](int index)
        {
            return _data[index];
        }

        void push_back(ArrayType& point)
        {
            _data.push_back(point);
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

    typedef struct  
    {
        int _idx[3];
        int& operator[](int idx)
        {
            return _idx[idx];
        }
    }Face;

#define     PointType       typename PtType<Scalar, Dim>::Point
#define     VertexType      typename VerType<Scalar, Dim>::Vertex
#define     NormalType      typename NmlType<Scalar, Dim>::Normal


    template <typename Scalar, int Dim>
    struct MatrixType
    {
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;

        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Dim, Eigen::RowMajor> MatrixD;

        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix;
    };

#define     TVector             typename MatrixType<Scalar, Dim>::Vector
#define     TMatrixD            typename MatrixType<Scalar, Dim>::MatrixD
#define     TMatrix             typename MatrixType<Scalar, Dim>::Matrix
}

#endif