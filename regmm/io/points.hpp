#ifndef _POINTS_HPP_
#define _POINTS_HPP_

#include <string>
#include <vector>

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

#define     PointsArray     typename PtsType<PointType, Scalar, Dim>::Points
#define     PointSet        PointsArray
}

namespace regmm
{
    template <typename Scalar, int Dim>
    void loadPointSet(const std::string& filename, PointSet& ps)
    {
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;

        std::string err = tinyobj::LoadObj(shapes, materials, filename.c_str());

        if (!err.empty()) {
            std::cerr << err << std::endl;
            exit(1);
        }

        assert(shapes.size() == 1 ? true : ("obj should be only one shape" && false));

        tinyobj::mesh_t& mesh = shapes[0].mesh;
        assert((positions.size() % 3) == 0);

        for (size_t v = 0; v < mesh.positions.size() / 3; v++) 
            ps.push_back(PointType(mesh.positions[3*v+0], mesh.positions[3*v+1], mesh.positions[3*v+2]));
    }

    template <typename Scalar, int Dim>
    void savePointSet(const std::string& filename, const PointSet& ps)
    {
        std::vector<tinyobj::shape_t> out_shape(1);
        std::vector<tinyobj::material_t> out_material;

        tinyobj::mesh_t& mesh = out_shape[0].mesh;
        for (size_t v = 0; v < ps.size(); ++ v)
        {
            mesh.positions.push_back(ps[v].x());
            mesh.positions.push_back(ps[v].y());
            mesh.positions.push_back(ps[v].z());
        }

        bool ret = WriteObj(filename, out_shape, out_material, false);
        assert(ret);
    }
}

#endif