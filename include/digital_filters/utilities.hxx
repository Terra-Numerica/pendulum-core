#pragma once

#include <sstream>


namespace digital_filters {

template<class Scalar>
std::vector<Scalar> polyProd(
  const std::vector<Scalar>& p1,
  const std::vector<Scalar>& p2
)
{
  // We would like to have p1 and p2 "sorted" so that p2 has at least same
  // size as p1. To achieve something similar, use references!
  const auto& q1 = p1.size() < p2.size() ? p1 : p2;
  const auto& q2 = p1.size() < p2.size() ? p2 : p1;
  unsigned int d1 = q1.size();
  unsigned int d2 = q2.size();

  // preallocate the space
  std::vector<Scalar> p(d1+d2-1);

  // initialize the elements while already doing some calculations
  for(unsigned int i=0; i<d1; i++)
    p[i] = q1[i] * q2[0];
  for(unsigned int j=1; j<d2; j++)
    p[j+d1-1] = q1[d1-1] * q2[j];

  // complete the polynomial
  for(unsigned int i=0; i<d1-1; i++) {
    for(unsigned int j=1; j<d2; j++) {
      p[i+j] = p[i+j] + q1[i] * q2[j];
    }
  }

  // done, return the result
  return p;
}


template <class Scalar>
std::string vec2str(
  const std::vector<Scalar>& vec,
  const std::string& sep
)
{
  std::stringstream ss;
  if(vec.size() > 0) {
    ss << vec[0];
    for(unsigned int i=1; i<vec.size(); i++)
      ss << sep << vec[i];
  }
  return ss.str();
}

}
