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

    std::string file = "d:/petal.obj";
    std::string newfile = "d:/npetal.obj";
    regmm::loadPointSet<SCALAR, DIM>(file, *source);
    regmm::loadPointSet<SCALAR, DIM>(file, *target);
    //regmm::loadMeshObject<SCALAR, DIM>(file, *test);

    //regmm::savePointSet<SCALAR, DIM>(newfile, *source);
    //regmm::saveMeshObject<SCALAR, DIM>(newfile, *test);

    
    regmm::Registrator<SCALAR, DIM>* registrator = new regmm::CPDRigid<SCALAR, DIM>();
    registrator->setDataType(regmm::POINT_CLOUD);
    registrator->setRegType(regmm::RIGID);
    registrator->setSource(*source);
    registrator->setTarget(*target);
    registrator->compute();

    regmm::savePointSet<SCALAR, DIM>(newfile, *source);

    int a;
    std::cin >> a;

    return 0;
}