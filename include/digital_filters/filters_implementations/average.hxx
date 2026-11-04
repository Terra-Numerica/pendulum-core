#pragma once

namespace digital_filters {

template <class CoeffType>
void average(
  int window_size,
  const CoeffType& gain,
  std::vector<CoeffType>& numerator,
  std::vector<CoeffType>& denominator
)
{
  numerator = std::vector<CoeffType>(window_size, gain);
  denominator = {static_cast<CoeffType>(window_size)};
}


template <class CoeffType>
void average(
  int window_size,
  std::vector<CoeffType>& numerator,
  std::vector<CoeffType>& denominator
)
{
  average<CoeffType>(window_size, 1, numerator, denominator);
}


template <class DataType, class CoeffType>
Filter<DataType,CoeffType> average(
  int window_size,
  const CoeffType& gain
)
{
  std::vector<CoeffType> num, den;
  average<CoeffType>(window_size, gain, num, den);
  return Filter<DataType,CoeffType>(num, den);
}


template <class DataType, class CoeffType>
Filter<DataType,CoeffType> average(
  int window_size
)
{
  std::vector<CoeffType> num, den;
  average<CoeffType>(window_size, num, den);
  return Filter<DataType,CoeffType>(num, den);
}

} // namespace digital_filters
