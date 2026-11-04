#pragma once

#include <stdexcept>
#include <algorithm>
#include <digital_filters/utilities.hpp>


namespace digital_filters {

template<class DataType, class CoeffType>
Filter<DataType,CoeffType>::Filter(
  const std::vector<CoeffType>& b_num,
  const std::vector<CoeffType>& a_den
)
: b_(b_num)
, a_(a_den)
, in_(b_num.size())
, out_(a_den.size())
{
  // check sizes
  if(b_num.size() == 0)
    throw std::runtime_error("Filter: numerator (b) is empty");
  if(a_den.size() == 0)
    throw std::runtime_error("Filter: denominator (a) is empty");

  // normalize numerator and denominator
  for(auto& bi : b_)
    bi = bi / a_den[0];
  for(auto& ai : a_)
    ai = ai / a_den[0];
}


template<class DataType, class CoeffType>
Filter<DataType,CoeffType> Filter<DataType,CoeffType>::operator*(
  const Filter<DataType,CoeffType>& other
) const
{
  return Filter<DataType,CoeffType>( polyProd(b_, other.b_), polyProd(a_, other.a_) );
}


template<class DataType, class CoeffType>
template<class OtherDataType, class OtherCoeffType>
Filter<OtherDataType,OtherCoeffType> Filter<DataType,CoeffType>::as() const
{
  // copy-convert numerator and denominator
  std::vector<OtherCoeffType> a(a_.begin(), a_.end());
  std::vector<OtherCoeffType> b(b_.begin(), b_.end());
  // create new filter
  auto filter = Filter<OtherDataType,OtherCoeffType>(b,a);
  // make sure to initialize the inputs and outputs
  filter.initInput(std::vector<OtherCoeffType>(in_.begin(), in_.end()-1));
  filter.initOutput(std::vector<OtherCoeffType>(out_.begin(), out_.end()-1));
  // return the result
  return filter;
}


template<class DataType, class CoeffType>
void Filter<DataType,CoeffType>::initInput(
  const DataType& input
)
{
  for(auto& val : in_)
    val = input;
}


template<class DataType, class CoeffType>
void Filter<DataType,CoeffType>::initInput(
  const std::vector<DataType>& input
)
{
  // check sizes
  if(input.size() != in_.size()-1) {
    throw std::runtime_error(
      "Filter::initInput: 'input' has " + std::to_string(input.size()) +
      " elements, but only " + std::to_string(in_.size()-1) +
      " elements are allowed"
    );
  }
  // copy the values
  for(unsigned int i=0; i<input.size(); i++)
    in_[i] = input[input.size()-i-1];
}


template<class DataType, class CoeffType>
void Filter<DataType,CoeffType>::initOutput(
  const DataType& output
)
{
  for(auto& val : out_)
    val = output;
}


template<class DataType, class CoeffType>
void Filter<DataType,CoeffType>::initOutput(
  const std::vector<DataType>& output
)
{
  // check sizes
  if(output.size() != out_.size()-1) {
    throw std::runtime_error(
      "Filter::initInput: 'output' has " + std::to_string(output.size()) +
      " elements, but only " + std::to_string(out_.size()-1) +
      " elements are allowed"
    );
  }
  // copy the values
  for(unsigned int i=0; i<output.size(); i++)
    out_[i] = output[output.size()-i-1];
}


template<class DataType, class CoeffType>
const DataType& Filter<DataType,CoeffType>::filter(
  const DataType& x
)
{
  // remove unnecessary input/output
  in_.pop_back();
  out_.pop_back();

  // add new input/output
  in_.push_front(x);
  out_.push_front(x * b_[0]);

  // complete the filtering
  auto& y = out_[0];

  for(unsigned int i=1; i<b_.size(); i++)
    y = y + b_[i] * in_[i];
  for(unsigned int i=1; i<a_.size(); i++)
    y = y - a_[i] * out_[i];
  return y;
}


template<class DataType, class CoeffType>
std::vector<DataType> Filter<DataType,CoeffType>::filter(
  const std::vector<DataType>& x,
  const std::vector<DataType>& x0,
  const std::vector<DataType>& y0
) const
{
  // check sizes
  if(x0.size() != b_.size() - 1) {
    throw std::runtime_error(
      "Filter::filter: initial conditions 'x0' have " +
      std::to_string(x0.size()) + " elements, but " +
      std::to_string(b_.size()) + " are required"
    );
  }
  if(y0.size() != a_.size() - 1) {
    throw std::runtime_error(
      "Filter::filter: initial conditions 'y0' have " +
      std::to_string(y0.size()) + " elements, but " +
      std::to_string(a_.size()) + " are required"
    );
  }

  // Prepare the output vector
  std::vector<DataType> y;
  y.reserve(x.size());

  // main loop: do the filtering
  for(unsigned int k=0; k<x.size(); k++) {
    // Init the "current" output from the corresponding input
    DataType yk = b_[0] * x[k];

    // Add the contributions from past inputs
    for(unsigned int i=1; i<b_.size(); i++) {
      if(i > k) {
        // use values from the initial conditions
        yk = yk + b_[i] * x0[i-k-1];
      }
      else {
        // use values from the input vector
        yk = yk + b_[i] * x[k-i];
      }
    }

    // Add the contributions from past outputs
    for(unsigned int i=1; i<a_.size(); i++) {
      if(i > k) {
        // use values from the initial conditions
        yk = yk - a_[i] * y0[i-k-1];
      }
      else {
        // use values from the input vector
        yk = yk - a_[i] * y[k-i];
      }
    }

    // push the computed value into the result vector
    y.push_back(yk);
  }

  // return the filtered vector
  return y;
}


template<class DataType, class CoeffType>
std::vector<DataType> Filter<DataType,CoeffType>::filter2(
  const std::vector<DataType>& x
) const
{
  // filter on one side
  std::vector<DataType> y = filter(
    x,
    std::vector<DataType>(b_.size()-1, x.at(0)),
    std::vector<DataType>(a_.size()-1, x.at(0))
  );
  // reverse the signal
  std::reverse(y.begin(), y.end());
  // filter again
  std::vector<DataType> z = filter(
    y,
    std::vector<DataType>(b_.size()-1, y.at(0)),
    std::vector<DataType>(a_.size()-1, y.at(0))
  );
  // reverse the final result and return it
  std::reverse(z.begin(), z.end());
  return z;
}

} // namespace digital_filters
