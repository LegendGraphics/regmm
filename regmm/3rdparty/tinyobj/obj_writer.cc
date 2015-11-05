//
// Simple wavefront .obj writer
//

#include <cstdio>
#include "obj_writer.h"


bool WriteObj(const std::string& filename, const std::vector<tinyobj::shape_t>& shapes, bool coordTransform) {
  FILE* fp = fopen(filename.c_str(), "w");
  if (!fp) {
    fprintf(stderr, "Failed to open file [ %s ] for write.\n", filename.c_str());
    return false;
  }

  if (shapes.size() != 1) {
      fprintf(stderr, "Shape size is not one.\n");
      return false;
  }

  for (size_t i = 0; i < shapes.size(); i++) {

    bool has_vn = false;
    bool has_vt = false;

    if (shapes[i].name.empty()) {
      fprintf(fp, "g Unknown\n");
    } else {
      fprintf(fp, "g %s\n", shapes[i].name.c_str());
    }

    for (size_t k = 0; k < shapes[i].mesh.positions.size() / 3; k++){
        fprintf(fp, "v %f %f %f\n",
            shapes[i].mesh.positions[3*k+0],
            shapes[i].mesh.positions[3*k+1],
            shapes[i].mesh.positions[3*k+2]);
    }

    for (size_t k = 0; k < shapes[i].mesh.normals.size() / 3; k++){
        fprintf(fp, "vn %f %f %f\n",
            shapes[i].mesh.normals[3*k+0],
            shapes[i].mesh.normals[3*k+1],
            shapes[i].mesh.normals[3*k+2]);
    }

    for (size_t k = 0; k < shapes[i].mesh.texcoords.size() / 2; k++){
        fprintf(fp, "vt %f %f\n",
            shapes[i].mesh.texcoords[2*k+0],
            shapes[i].mesh.texcoords[2*k+1]);
    }

    if (shapes[i].mesh.normals.size() > 0) has_vn = true;
    if (shapes[i].mesh.texcoords.size() > 0) has_vt = true;

    for (size_t k = 0; k < shapes[i].mesh.indices.size() / 3; k++) {

        // Face index is 1-base.
        int v0 = shapes[i].mesh.indices[(3*k + 0)] + 1;
        int v1 = shapes[i].mesh.indices[(3*k + 1)] + 1;
        int v2 = shapes[i].mesh.indices[(3*k + 2)] + 1;

        if (has_vn && has_vt) {
            fprintf(fp, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
                v0, v0, v0, v1, v1, v1, v2, v2, v2);
        } else if (has_vn && !has_vt) {
            fprintf(fp, "f %d//%d %d//%d %d//%d\n", v0, v0, v1, v1, v2, v2);
        } else if (!has_vn && has_vt) {
            fprintf(fp, "f %d/%d %d/%d %d/%d\n", v0, v0, v1, v1, v2, v2);
        } else {
            fprintf(fp, "f %d %d %d\n", v0, v1, v2);
        }

    }

  }

  fclose(fp);

  return true;
}


