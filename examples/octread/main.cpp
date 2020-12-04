#include "AbcLoad.h"


int main(int argc, char * argv[])
{
  abcmat::AbcLoader loader;
  if (!loader.load(OCTOPUS_FILE))
  {
    std::cerr << "Failed to load octopus example!" << std::endl;
    return 1;
  }

  std::string octopus = loader.mesh_names()[0];
  size_t n_samples = loader.num_samples(octopus);

  // read mesh of the last frame/sample
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  if (loader.read(octopus, n_samples - 1, V, F))
  {
    std::cout << "Loaded octopus mesh with " << 
      V.rows() << " vertices / " <<
      F.rows() << " triangles" << std::endl;
  }

  return 0;
}