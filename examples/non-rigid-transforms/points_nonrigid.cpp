#include <iostream>

#include "regmm/io/points.hpp"
#include "regmm/transform/regmm_engine.hpp"

#define SCALAR float    // floating-point precision
#define DIM 3           // data dimension: 2 or 3

int main()
{
    // source and target instance
    // target could be either points or mesh, since we only deform source towards target
    PointSetInstance(SCALAR, DIM)* source = new PointSetInstance(SCALAR, DIM);
    PointSetInstance(SCALAR, DIM)* target = new PointSetInstance(SCALAR, DIM);

    // source and target files, points supports xyz format and mesh supports obj format
    std::string source_file = "D:/regmm/data/nonrigid/face_source.xyz";
    std::string target_file = "D:/regmm/data/nonrigid/face_target.xyz";

    // new source file
    std::string new_source = "D:/regmm/data/nonrigid/face_source_new.xyz";

    // load source and target
    regmm::loadPointSet<SCALAR, DIM>(source_file, *source);
    regmm::loadPointSet<SCALAR, DIM>(target_file, *target);

    // RegmmEngine instance
    regmm::RegmmEngine<SCALAR, DIM>* regmm_engine = new regmm::RegmmEngine<SCALAR, DIM>();

    // initialize RegmmEngine, set registration type and source data type
    regmm_engine->initEngine(regmm::NONRIGID, regmm::POINT_CLOUD);

    // set source and target
    regmm_engine->setSource(*source);
    regmm_engine->setTarget(*target);

    /* set parameters, default values have been set
    // basic paras
    regmm_engine->setIterativeNumber(60);           // iterative number, default = 50
    regmm_engine->setVarianceTolerance(1e-6);       // variance tolerance, default = 1e-3
    regmm_engine->setEnergyTolerance(1e-5);         // energy tolerance, default = 1e-3
    regmm_engine->setOutlierWeight(0.0);            // outliers weight, default = 0.0
    // optional, for speeding up
    regmm_engine->setFgtFlag(true);                 // fast gaussian transform for speed, default = false
    regmm_engine->setFgtEpsilon(1e-4);              // fgt epsilon, default = 1e-3
    regmm_engine->setLowRankFlag(false);            // low rank approximation for speed, default = false
    regmm_engine->setKLowRank(30);                  // reserve K rank, default = 10
    regmm_engine->setLRMaxIteration(50);            // low rank iterative number, default = 40;
    */

    // compute transformation
    regmm_engine->compute();


    // save new source
    regmm::savePointSet<SCALAR, DIM>(new_source, *source);

    return 0;
}