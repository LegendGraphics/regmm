#include <iostream>
#include "regmm/transform/cpd/cpd_rigid.hpp"

#define SCALAR float
#define DIM 3

int main()
{
    PointSetInstance(SCALAR, DIM)* source = new PointSetInstance(SCALAR, DIM);
    PointSetInstance(SCALAR, DIM)* target = new PointSetInstance(SCALAR, DIM);

    //regmm::loadPointSet<SCALAR, DIM>("d:/source.obj", source);

   /* regmm::Registrator<SCALAR, DIM>* registrator = new regmm::CPDRigid<SCALAR, DIM>();
    registrator->setDataType(regmm::POINT_CLOUD);
    registrator->setRegType(regmm::RIGID);*/

    return 0;
}