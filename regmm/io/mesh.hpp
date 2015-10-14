#ifndef _MESH_HPP_
#define _MESH_HPP_

#include <string>

#include "regmm/io/basic_types.hpp"

namespace regmm
{
    template <typename Scalar, int Dim>
    struct VtsType
    {
        typedef Arrays<VertexType, Scalar, Dim> Vertices;
    };

#define     VerticesArray     typename VtsType<Scalar, Dim>::Vertices
}

namespace regmm
{
    template <typename Scalar, int Dim>
    struct NmlsType
    {
        typedef Arrays<NormalType, Scalar, Dim> Normals;
    };

#define     NormalsArray     typename NmlsType<Scalar, Dim>::Normals
}

namespace regmm
{
    template <typename Scalar, int Dim>
    struct FacesType
    {
        typedef Arrays<Face, Scalar, Dim> Faces;
    };

#define     FacesArray     typename FacesType<Scalar, Dim>::Faces
}


namespace regmm
{
    template <typename Scalar, int Dim>
    class Mesh
    {
    private:
        VerticesArray   vts_;
        NormalsArray    nmls_;
        FacesArray      fs_;

    public:
        bool load(const std::string& filename);
        void save(const std::string& filename);

        VerticesArray& getVertices() const { return vts_; }
        NormalsArray& getNormals() const { return nmls_; }
        FacesArray& getFaces() const { return fs_; }
    };

#define     MeshObject    Mesh<Scalar, Dim>
}


#endif