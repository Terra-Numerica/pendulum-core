#pragma once

#include <cmath>


namespace digital_filters {


template <class CoeffType>
void exponential(
  const CoeffType& alpha,
  std::vector<CoeffType>& numerator,
  std::vector<CoeffType>& denominator
)
{
  numerator = {alpha};
  denominator = {1,alpha-1};
}


template <class CoeffType>
void exponential(
  const CoeffType& sampling,
  const CoeffType& time_constant,
  std::vector<CoeffType>& numerator,
  std::vector<CoeffType>& denominator
)
{
  using ::exp;
  exponential<CoeffType>(1-exp(-sampling/time_constant), numerator, denominator);
}


template <class DataType, class CoeffType>
Filter<DataType,CoeffType> exponential(
  const CoeffType& alpha
)
{
  std::vector<CoeffType> num, den;
  exponential<CoeffType>(alpha, num, den);
  return Filter<DataType,CoeffType>(num, den);
}


template <class DataType, class CoeffType>
Filter<DataType,CoeffType> exponential(
  const CoeffType& sampling,
  const CoeffType& time_constant
)
{
  std::vector<CoeffType> num, den;
  exponential<CoeffType>(sampling, time_constant, num, den);
  return Filter<DataType,CoeffType>(num, den);
}

} // namespace digital_filters
