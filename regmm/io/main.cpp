#include <iostream>
#include "regmm/io/mesh.hpp"
//#include "regmm/transform/cpd/cpd_rigid.hpp"
//#include "regmm/transform/cpd/cpd_nonrigid.hpp"
#include "regmm/transform/arap/arap_solver.hpp"


#define SCALAR float
#define DIM 3

int main()
{
    //PointSetInstance(SCALAR, DIM)* source = new PointSetInstance(SCALAR, DIM);
    //PointSetInstance(SCALAR, DIM)* target = new PointSetInstance(SCALAR, DIM);

    //std::string file = "d:/petal.obj";
    //std::string newfile = "d:/npetal.obj";
    //regmm::loadPointSet<SCALAR, DIM>(file, *source);
    //regmm::loadPointSet<SCALAR, DIM>(file, *target);
    //
    ///*regmm::Registrator<SCALAR, DIM>* registrator = new regmm::CPDRigid<SCALAR, DIM>();*/
    //regmm::Registrator<SCALAR, DIM>* registrator = new regmm::CPDNRigid<SCALAR, DIM>();
    //registrator->setDataType(regmm::POINT_CLOUD);
    //registrator->setRegType(regmm::RIGID);
    //registrator->setSource(*source);
    //registrator->setTarget(*target);
    //registrator->compute();

    //regmm::savePointSet<SCALAR, DIM>(newfile, *source);

    MeshObjectInstance(SCALAR, DIM)* source = new MeshObjectInstance(SCALAR, DIM);
    MeshObjectInstance(SCALAR, DIM)* target = new MeshObjectInstance(SCALAR, DIM);

    regmm::Registrator<SCALAR, DIM>* registrator = new regmm::ARAPSolver<SCALAR, DIM>();

    int a;
    std::cin >> a;

    return 0;
}