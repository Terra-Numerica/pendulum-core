#pragma once

#include <digital_filters/filter.hpp>


namespace digital_filters {

/// Computes the numerator and denominator of a butterworth filter.
template <class CoeffType>
void butterworth(
  unsigned int order,
  const CoeffType& cutoff,
  const CoeffType& sampling,
  std::vector<CoeffType>& numerator,
  std::vector<CoeffType>& denominator
);


/// Returns a butterworth filter.
/** @ingroup CommonFilters
  */
template <class DataType, class CoeffType>
Filter<DataType,CoeffType> butterworth(
  unsigned int order,
  const CoeffType& cutoff,
  const CoeffType& sampling
);

} // namespace digital_filters

#include <digital_filters/filters_implementations/butterworth.hxx>
