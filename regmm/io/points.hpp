#ifndef _POINTS_HPP_
#define _POINTS_HPP_

#include <vector>
#include <fstream>
#include <string>

#include "regmm/io/basic_types.hpp"
#include "regmm/3rdparty/tinyobj/tiny_obj_loader.h"
#include "regmm/3rdparty/tinyobj/obj_writer.h"

namespace regmm
{
    template <typename Scalar, int Dim>
    struct PtsType
    {
        typedef Arrays<PointType, Scalar, Dim> Points;
    };

#define     PointsArray     typename PtsType<Scalar, Dim>::Points
#define     PointSet        PointsArray

#define     PointSetInstance(S, Dim)  regmm::PtsType<S, Dim>::Points
}

namespace regmm
{
    template <typename Scalar, int Dim>
    void loadPointSet(const std::string& filename, PointSet& ps)
    {
        std::fstream fin(filename.c_str(), std::ios_base::in);

        if (!fin)
        {
            std::cout << "cannot open the file!" << std::endl;
            return;
        }

        std::vector<Scalar> m_values;
        Scalar tmp = 0;
        size_t i = 0, j = 0;
        while (fin >> tmp)
        {
            m_values.push_back(tmp);
        }

        size_t points_size = m_values.size() / Dim;

        if (points_size * Dim != m_values.size())
        {
            std::cout << "File is broken! Or Dimension is incorrect!" << std::endl;
            return;
        }

        for (size_t i = 0; i < points_size; ++ i)
        {
            if (Dim == 3)
                ps.push_back(PointType(m_values[3*i], m_values[3*i+1], m_values[3*i+2]));
            else 
                ps.push_back(PointType(m_values[2*i], m_values[2*i+1]));
        }

        fin.close();
    }

    template <typename Scalar, int Dim>
    void savePointSet(const std::string& filename, PointSet& ps)
    {
        std::fstream fout(filename.c_str(), std::ios_base::out);

        for (size_t i = 0; i < ps.size(); ++ i)
        {
            if (Dim == 3)
                fout << ps[i].x() << " " << ps[i].y() << " " << ps[i].z() << "\n";
            else
                fout << ps[i].x() << " " << ps[i].y() << "\n";
        }

        fout.close();
    }
}

#endif