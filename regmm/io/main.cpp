#include <iostream>
#include "regmm/io/mesh.hpp"
#include "regmm/transform/regmm_engine.hpp"


#define SCALAR float
#define DIM 3

int main()
{
    PointSetInstance(SCALAR, DIM)* source = new PointSetInstance(SCALAR, DIM);
    PointSetInstance(SCALAR, DIM)* target = new PointSetInstance(SCALAR, DIM);

    std::string file = "d:/petal.obj";
    std::string newfile = "d:/npetal.obj";
    regmm::loadPointSet<SCALAR, DIM>(file, *source);
    regmm::loadPointSet<SCALAR, DIM>(file, *target);

    regmm::RegmmEngine<SCALAR, DIM>* regmm_engine = new regmm::RegmmEngine<SCALAR, DIM>();
    regmm_engine->initEngine(regmm::NONRIGID, regmm::POINT_CLOUD);
    regmm_engine->setSource(*source);
    regmm_engine->setTarget(*target);
    regmm_engine->compute();

    regmm::savePointSet<SCALAR, DIM>(newfile, *source);


    
    /*MeshObjectInstance(SCALAR, DIM)* source = new MeshObjectInstance(SCALAR, DIM);
    MeshObjectInstance(SCALAR, DIM)* target = new MeshObjectInstance(SCALAR, DIM);

    std::string file = "d:/petal.obj";
    std::string newfile = "d:/npetal.obj";
    regmm::loadMeshObject<SCALAR, DIM>(file, *source);
    regmm::loadMeshObject<SCALAR, DIM>(file, *target);

    regmm::RegmmEngine<SCALAR, DIM>* regmm_engine = new regmm::RegmmEngine<SCALAR, DIM>();
    regmm_engine->initEngine(regmm::NONRIGID, regmm::POINT_CLOUD);

    regmm_engine->setSource(*source);
    regmm_engine->setTarget(*target);
    regmm_engine->compute();

    regmm::saveMeshObject<SCALAR, DIM>(newfile, *source);*/

    int a;
    std::cin >> a;

    return 0;
}