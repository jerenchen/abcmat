#ifndef ABCMAT_ABCLOAD_H
#define ABCMAT_ABCLOAD_H

#include <Alembic/AbcGeom/IPolyMesh.h>
#include <Alembic/AbcGeom/IXform.h>

#include <Eigen/Dense>

#include <functional>
#include <string>
#include <stack>

namespace abcmat
{
class AbcLoader
{
  private:
    std::map<std::string, Alembic::AbcGeom::IPolyMesh> meshes;

  public:
    inline bool load(const std::string& path);

    inline bool read(
      const std::string& mesh_name,
      const unsigned int sample_index,
      Eigen::MatrixXd& V,
      Eigen::MatrixXi& F
    );

    std::vector<std::string> mesh_names() const
    {
      std::vector<std::string> names;
      for (const auto& it : meshes)
      {
        names.push_back(it.first);
      }
      return names;
    }

    size_t num_samples(const std::string& mesh_name)
    {
      if (meshes.find(mesh_name) == meshes.end())
      {
        return 0;
      }
      return meshes[mesh_name].getSchema().getNumSamples();
    }
};
};

// Implementation

#include <Alembic/AbcCoreFactory/All.h>
#include <Alembic/AbcGeom/All.h>

inline bool abcmat::AbcLoader::load(const std::string& filename)
{
  using namespace Alembic;

  meshes.clear();

  AbcCoreFactory::IFactory factory;
  factory.setPolicy(Abc::ErrorHandler::kQuietNoopPolicy);
  Abc::IArchive archive = factory.getArchive(filename.c_str());
  
  if (!archive.valid())
  {
    std::cerr << "Error loading file " << filename << std::endl;
    return false;
  }

  Alembic::Abc::IObject top = archive.getTop();

  if (!top.valid())
  {
    std::cerr << "Invalid top-level object in " << filename << std::endl;
    return false;
  }

  std::function<void(Abc::IObject)> traverse = [&](Abc::IObject obj)
  {
    if (AbcGeom::IPolyMesh::matches(obj.getHeader()))
    {
      std::cout << "Adding mesh \"" << obj.getFullName() << "\"..." << std::endl;
      meshes[obj.getFullName()] = AbcGeom::IPolyMesh(obj, Abc::kWrapExisting);
    }

    for (unsigned int ii = 0; ii < obj.getNumChildren(); ++ii)
    {
      traverse(obj.getChild(ii));
    }
  };

  traverse(top);

  return true;
}

inline bool abcmat::AbcLoader::read(
  const std::string& mesh_name,
  const unsigned int sample_index,
  Eigen::MatrixXd& V,
  Eigen::MatrixXi& F
)
{
  using namespace Alembic;

  if (meshes.find(mesh_name) == meshes.end())
  {
    std::cerr << "Failed to find mesh " << mesh_name << std::endl;
    return false;
  }

  AbcGeom::IPolyMeshSchema mesh = meshes[mesh_name].getSchema();

  AbcCoreAbstract::TimeSamplingPtr sampling = mesh.getTimeSampling();
  Alembic::Abc::ISampleSelector selector(sampling->getSampleTime(sample_index));

  // Read polygon face indices
  Alembic::Abc::Int32ArraySamplePtr face_indices;
  mesh.getFaceIndicesProperty().get(face_indices, selector);
  Alembic::Abc::Int32ArraySamplePtr face_counts;
  mesh.getFaceCountsProperty().get(face_counts, selector);

  std::vector<std::vector<int> > triangles;
  for (unsigned int ff = 0, ii = 0; ff < face_counts->size(); ++ff)
  {
    // Read as reverse since Alembic stores vertices clock-wise
    std::stack<int> indices;
    int v1, v2, v3;
    // We will connect every triangle to v3 (first polygon vertex),
    //   this might yield narrow triangles if polygon has many sides
    v3 = (*face_indices)[ii++];
    for (unsigned int fi = 1; fi < (*face_counts)[ff]; ++fi)
    {
      indices.push((*face_indices)[ii++]);
    }

    // Begin to append triangles from the last vertex...
    v1 = indices.top();
    indices.pop();

    while (!indices.empty())
    {
      v2 = indices.top();
      indices.pop();

      triangles.push_back(std::vector<int>{v1, v2, v3});

      v1 = v2;
    }
  }

  F.resize(triangles.size(), 3);
  for (unsigned int fi = 0; fi < triangles.size(); ++fi)
  {
    const auto& indices = triangles[fi];
    F.row(fi) << indices[0], indices[1], indices[2];
  }

  Abc::P3fArraySamplePtr points;
  mesh.getPositionsProperty().get(points, selector);

  V.resize(points->size(), 3);
  for (unsigned int vi = 0; vi < points->size(); ++vi)
  {
    const auto& point = (*points)[vi];
    V.row(vi) << point.x, point.y, point.z;
  }

  return true;
}

#endif