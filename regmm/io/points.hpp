#ifndef _POINTS_HPP_
#define _POINTS_HPP_

namespace regmm
{
    template <typename Scalar, int NumAtCompileTime>
    class Points
    {
    private:
        struct Point 
        {
            Scalar x;
            Scalar y;
            Scalar z;
        };

        Point _data[];

    public:
        Points();
        virtual ~Points();

        Point& operator[](int index);
    };

    template<typename Scalar, int NumAtCompileTime>
    Points<Scalar, NumAtCompileTime>::Points()
    {
        _data = new int[NumAtCompileTime];
    }

    template<typename Scalar, int NumAtCompileTime>
    Points<Scalar, NumAtCompileTime>::~Points()
    {
        delete[] _data;
    }

    template<typename Scalar, int NumAtCompileTime>
    Point& Points<Scalar, NumAtCompileTime>::operator[](int index)
    {
        return _data[index];
    }
}

#endif