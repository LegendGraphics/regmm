#ifndef __OBJ_WRITER_H__
#define __OBJ_WRITER_H__

#include "tiny_obj_loader.h"

extern bool WriteObj(const std::string& filename, const std::vector<tinyobj::shape_t>& shapes, bool coordTransform = false);


#endif // __OBJ_WRITER_H__
