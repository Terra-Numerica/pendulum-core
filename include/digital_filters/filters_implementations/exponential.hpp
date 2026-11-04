#pragma once

#include <digital_filters/filter.hpp>


namespace digital_filters {

/// Computes the numerator and denominator of an exponential filter.
/** The exponential filter is defined by the following difference equation:
  *   \f[ y_k = \alpha x_k + (1-\alpha) y_{k-1} \f]
  * This function computes the numerator and denominator of the corresponding
  * transfer function.
  * @param[in] alpha smoothing coefficient of the filter.
  * @param[out] numerator numerator of the transfer function.
  * @param[out] denominator denominator of the transfer function.
  */
template <class CoeffType>
void exponential(
  const CoeffType& alpha,
  std::vector<CoeffType>& numerator,
  std::vector<CoeffType>& denominator
);


/// Computes the numerator and denominator of an exponential filter.
/** The exponential filter is defined by the following difference equation:
  *   \f[ y_k = \alpha x_k + (1-\alpha) y_{k-1} \f]
  * wherein \f$ \alpha = 1 - e^{-\frac{\Delta t}{\tau}} \f$, with \f$\Delta t\f$
  * being the sampling time and \f$\tau\f$ the time constant of the filter. The
  * output will reach respectively the 63%, 86% and 95% at the times \f$\tau\f$,
  * \f$2\tau\f$ and \f$3\tau\f$.
  * @param[in] sampling sampling time \f$\Delta t\f$.
  * @param[in] time_constant time constant \f$\tau\f$.
  * @param[out] numerator numerator of the transfer function.
  * @param[out] denominator denominator of the transfer function.
  */
template <class CoeffType>
void exponential(
  const CoeffType& sampling,
  const CoeffType& time_constant,
  std::vector<CoeffType>& numerator,
  std::vector<CoeffType>& denominator
);


/// Returns an exponential filter.
/** @ingroup CommonFilters
  */
template <class DataType, class CoeffType>
Filter<DataType,CoeffType> exponential(
  const CoeffType& alpha
);


/// Returns an exponential filter.
/** @ingroup CommonFilters
  */
template <class DataType, class CoeffType>
Filter<DataType,CoeffType> exponential(
  const CoeffType& sampling,
  const CoeffType& time_constant
);

} // namespace digital_filters

#include <digital_filters/filters_implementations/exponential.hxx>
