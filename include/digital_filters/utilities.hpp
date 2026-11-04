#pragma once

#include <vector>
#include <string>


namespace digital_filters {

/// Computes a polynomial product.
/** @tparam Scalar type of the polynomal coefficients.
  * @param p1 Coefficients of the first polynomial, starting from degree zero.
  * @param p2 Coefficients of the second polynomial, starting from degree zero.
  * @return Coefficients of the polynomal product. As an example, if the two
  *   polynomials were `[a0, a1]` and `[b0, 0, b2]` the result would be
  *   `[ a0*b0, a1*b0, a0*b2, a1*b2 ]`.
  */
template<class Scalar>
std::vector<Scalar> polyProd(
  const std::vector<Scalar>& p1,
  const std::vector<Scalar>& p2
);


/// Generates a human readable string that represents the given input vector.
template <class Scalar>
std::string vec2str(
  const std::vector<Scalar>& vec,
  const std::string& sep=" "
);

} // namespace digital_filters

// implementation
#include <digital_filters/utilities.hxx>
