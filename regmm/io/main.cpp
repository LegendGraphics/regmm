#include <iostream>
#include "regmm/transform/cpd/cpd_rigid.hpp"
#include "regmm/io/mesh.hpp"

#define SCALAR float
#define DIM 3

int main()
{
    PointSetInstance(SCALAR, DIM)* source = new PointSetInstance(SCALAR, DIM);
    PointSetInstance(SCALAR, DIM)* target = new PointSetInstance(SCALAR, DIM);

    MeshObjectInstance(SCALAR, DIM)* test = new MeshObjectInstance(SCALAR, DIM);

    std::string file = "d:/cube.obj";
    regmm::loadPointSet<SCALAR, DIM>(file, *source);
    regmm::loadMeshObject<SCALAR, DIM>(file, *test);
    int a;
    std::cin >> a;
   /* regmm::Registrator<SCALAR, DIM>* registrator = new regmm::CPDRigid<SCALAR, DIM>();
    registrator->setDataType(regmm::POINT_CLOUD);
    registrator->setRegType(regmm::RIGID);*/

    return 0;
}