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
       /* bool load(const std::string& filename);
        void save(const std::string& filename);*/

        VerticesArray& getVertices() { return vts_; }
        NormalsArray& getNormals() { return nmls_; }
        FacesArray& getFaces() { return fs_; }

        int size() const { return vts_.size(); }
    };

#define     MeshObject    Mesh<Scalar, Dim>

#define     MeshObjectInstance(S, D)  regmm::Mesh<S, D>

}

namespace regmm
{
    template <typename Scalar, int Dim>
    void loadMeshObject(const std::string& filename, MeshObject& mo)
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
        assert((mesh.positions.size() % 3) == 0);
        assert((mesh.normals.size() % 3) == 0);
        assert((mesh.indices.size() % 3) == 0);

        VerticesArray& vts = mo.getVertices();
        NormalsArray& nls = mo.getNormals();
        FacesArray& fs = mo.getFaces();

        for (size_t v = 0; v < mesh.positions.size() / 3; v++) 
            vts.push_back(VertexType(mesh.positions[3*v+0], mesh.positions[3*v+1], mesh.positions[3*v+2]));

        for (size_t n = 0; n < mesh.normals.size() / 3; n++) 
            nls.push_back(NormalType(mesh.normals[3*n+0], mesh.normals[3*n+1], mesh.normals[3*n+2]));

        for (size_t f = 0; f < mesh.indices.size() / 3; f++) 
            fs.push_back(Face(mesh.indices[3*f+0], mesh.indices[3*f+1], mesh.indices[3*f+2]));
    }

    template <typename Scalar, int Dim>
    void saveMeshObject(const std::string& filename, MeshObject& mo)
    {
        std::vector<tinyobj::shape_t> out_shape(1);
        std::vector<tinyobj::material_t> out_material;

        tinyobj::mesh_t& mesh = out_shape[0].mesh;

        VerticesArray& vts = mo.getVertices();
        NormalsArray& nls = mo.getNormals();
        FacesArray& fs = mo.getFaces();

        for (size_t v = 0; v < vts.size(); ++ v)
        {
            mesh.positions.push_back(vts[v].x());
            mesh.positions.push_back(vts[v].y());
            mesh.positions.push_back(vts[v].z());
        }

        for (size_t n = 0; n < nls.size(); ++ n)
        {
            mesh.normals.push_back(nls[n].x());
            mesh.normals.push_back(nls[n].y());
            mesh.normals.push_back(nls[n].z());
        }

        for (size_t f = 0; f < fs.size(); ++ f)
        {
            mesh.indices.push_back(fs[f][0]);
            mesh.indices.push_back(fs[f][1]);
            mesh.indices.push_back(fs[f][2]);
        }

        bool ret = WriteObj(filename, out_shape, out_material, false);
        assert(ret);
    }
}


#endif