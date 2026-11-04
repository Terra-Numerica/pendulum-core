#pragma once

#include <digital_filters/utilities.hpp>
#include <cmath>


namespace digital_filters {


template <class CoeffType>
void butterworth(
  unsigned int order,
  const CoeffType& cutoff,
  const CoeffType& sampling,
  std::vector<CoeffType>& numerator,
  std::vector<CoeffType>& denominator
)
{
  // check that the cutoff frequency is less than Nyquist's one
  if(cutoff*2 > sampling)
    throw std::runtime_error("In fuction butterworth: cutoff frequency should be less than half of sampling frequency");

  // initialize num/den
  numerator = std::vector<CoeffType>(1,1);
  denominator = std::vector<CoeffType>(1,1);

  // If the order is one, this is not really a filter...
  if(order == 0)
    return;

  // Evaluate recurring constants
  const CoeffType gc = std::tan(M_PI*cutoff/sampling);
  const CoeffType gc2 = gc*gc;

  std::vector<CoeffType> di(3, 1);
  std::vector<CoeffType> ni(2, 1);

  // evaluate the numerator, which is simply (1+z^-1)^N
  for(unsigned int i=0; i<order; i++)
    numerator = polyProd(numerator, ni);

  // evaluate complex conjugate poles
  for(unsigned int i=0; i<(order/2); i++) {
    CoeffType ci = 2*gc * std::cos( (order+1+2*i)*M_PI/(2*order) );
    di[1] = 2 * (gc2-1) / (1+gc2-ci);
    di[2] = (1+gc2+ci) / (1+gc2-ci);
    denominator = polyProd(denominator, di);
  }

  // if 'order' is odd, add a single real pole
  if(order % 2 > 0)
    denominator = polyProd(denominator, std::vector<CoeffType>{1, (gc-1)/(gc+1)});

  // normalize
  CoeffType Sn=numerator[0], Sd=denominator[0];
  for(unsigned int i=1; i<numerator.size(); i++)
    Sn = Sn + numerator[i];
  for(unsigned int i=1; i<denominator.size(); i++)
    Sd = Sd + denominator[i];
  for(auto& n : numerator)
    n = n * Sd / Sn;
}


template <class DataType, class CoeffType>
Filter<DataType,CoeffType> butterworth(
  unsigned int order,
  const CoeffType& cutoff,
  const CoeffType& sampling
)
{
  std::vector<CoeffType> num, den;
  butterworth<CoeffType>(order, cutoff, sampling, num, den);
  return Filter<DataType,CoeffType>(num, den);
}

} // namespace digital_filters
