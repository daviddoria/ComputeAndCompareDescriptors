#ifndef ComputeNormals_hpp
#define ComputeNormals_hpp

template <typename TInput, typename TOutput>
void ComputeNormals<TInput, TOutput>::operator()(typename pcl::PointCloud<TInput>::Ptr input, typename pcl::PointCloud<TOutput>::Ptr output)
{
  // Pass the points through
  copyPointCloud(*input, *output);

  // Compute the normals
  pcl::NormalEstimation<TInput, TOutput> normalEstimation;
  normalEstimation.setInputCloud (input);

  this->Tree = typename TreeType::Ptr(new TreeType);
  normalEstimation.setSearchMethod (this->Tree);

  normalEstimation.setRadiusSearch (0.1);

  normalEstimation.compute (*output);
}

#endif
