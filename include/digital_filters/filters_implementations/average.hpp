#pragma once

#include <digital_filters/filter.hpp>


namespace digital_filters {

/// Computes the numerator and denominator of an average filter.
template <class CoeffType>
void average(
  int window_size,
  const CoeffType& gain,
  std::vector<CoeffType>& numerator,
  std::vector<CoeffType>& denominator
);


/// Computes the numerator and denominator of an average filter.
template <class CoeffType>
void average(
  int window_size,
  std::vector<CoeffType>& numerator,
  std::vector<CoeffType>& denominator
);


/// Returns an average filter.
/** @ingroup CommonFilters
  */
template <class DataType, class CoeffType>
Filter<DataType,CoeffType> average(
  int window_size,
  const CoeffType& gain
);


/// Returns an average filter.
/** @ingroup CommonFilters
  */
template <class DataType, class CoeffType>
Filter<DataType,CoeffType> average(
  int window_size
);

} // namespace digital_filters

#include <digital_filters/filters_implementations/average.hxx>
