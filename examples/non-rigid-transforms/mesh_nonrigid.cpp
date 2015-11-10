#include <iostream>

#include "regmm/io/mesh.hpp"
#include "regmm/transform/regmm_engine.hpp"

#define SCALAR float    // floating-point precision
#define DIM 3           // data dimension: 2 or 3

int main()
{
    // source and target instance
    // target could be either points or mesh, since we only deform source towards target
    MeshObject(SCALAR, DIM)* source = new MeshObject(SCALAR, DIM);
    MeshObject(SCALAR, DIM)* target = new MeshObject(SCALAR, DIM);

    // source and target files, points supports xyz format and mesh supports obj format
    std::string source_file = "D:/baidu disk/WorkSpace/regmm/data/nonrigid/face_source.xyz";
    std::string target_file = "D:/baidu disk/WorkSpace/regmm/data/nonrigid/face_target.xyz";

    // deformed source file
    std::string deformed_source = "D:/baidu disk/WorkSpace/regmm/data/nonrigid/face_deformed.xyz";

    // load source and target
    regmm::loadMeshObject<SCALAR, DIM>(source_file, *source);
    regmm::loadMeshObject<SCALAR, DIM>(target_file, *target);

    // RegmmEngine instance
    regmm::RegmmEngine<SCALAR, DIM>* regmm_engine = new regmm::RegmmEngine<SCALAR, DIM>();

    // initialize RegmmEngine, set registration type and source data type
    regmm_engine->initEngine(regmm::NONRIGID, regmm::MESH);

    // set source and target
    regmm_engine->setSource(*source);
    regmm_engine->setTarget(*target);

    /* set parameters, default values have been set
    regmm_engine->setIterativeNumber(60);           // iterative number, default = 50
    regmm_engine->setDataFittingWeight(1e-1);       // data fitting weight, default = 1e-2
    regmm_engine->setARAPWeight(1);                 // arap weight, default = 1
    regmm_engine->setEpsilon(1e-2);                 // epsilon tolerance, default = 1e-3
    regmm_engine->setNoise(0.0);                    // noise weight, default = 0.0
    */

    // compute deformation
    regmm_engine->compute();


    // save deformed source
    regmm::saveMeshObject<SCALAR, DIM>(deformed_source, *source);

    return 0;
}