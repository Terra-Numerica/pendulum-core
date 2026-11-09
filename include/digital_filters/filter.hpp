/** @file filter.hpp
  * @brief Header file containing the Filter class.
  */
#pragma once

#include <vector>
#include <deque>

namespace digital_filters {
    /// Simple class that represents a digital filter.
    /**
      * @tparam DataType Type of the input/output signals
      * @tparam CoefficientType Type of the coefficients of the transfer function.
      */
    template<class DataType, class CoefficientType>
    class Filter {
    public:
        /// Generates a filter from given coefficients.
        /** Creates a filter whose transfer function in the discrete domain is given
          * as:
          * \f[
          *   H(z) = \frac{b(z)}{a(z)}
          *        = \frac{b_0 + b_1 z^{-1} + \cdots}{a_0 + a_1 z^{-1} + \cdots}
          * \f]
          * which corresponds to the difference equation:
          * \f[
          *   a_0 y_k + a_1 y_{k-1} + a_2 y_{k-2} + \cdots =
          *     b_0 x_k + b_1 x_{k-1} + b_2 x_{k-2} + \cdots
          * \f]
          * The denominator will be normalized internally, i.e., each element of the
          * denominator and numerator will be divided by \f$ a_0 \f$. This adds a
          * requirement on the coefficients type: they must support operator/.
          * In addition, this requires \f$ a_0 \f$ to be non-zero.
          * @param b_num Numerator of the discrete transfer function. Entries should
          *   be ordered such that `b_num[i]` contains \f$ b_i \f$.
          * @param a_den Denominator of the discrete transfer function. Entries should
          *   be ordered such that `a_den[i]` contains \f$ a_i \f$. Note that the
          *   first element must not be zero.
          */
        Filter(
            const std::vector<CoefficientType> &b_num,
            const std::vector<CoefficientType> &a_den
        );

        /// Filters concatenation.
        /** This overladed operator allows to concatenate two filters.
          * Given the filters \f$H_1(z)\f$ and \f$H_2(z)\f$, the new filter
          * \f$H_3(z) = H_2(z)H_1(z)\f$ operates on a signal in the same way as
          * filtering first with \f$H_1(z)\f$ and then using \f$H_2(z)\f$ (actually,
          * the order of application of the two filters does not matter, since filter
          * concatenation is a commutative operation).
          * @param other filter that should be concatenated to `this`.
          * @return a new filter which is the concatenation of `this` and `other`.
          */
        Filter operator*(
            const Filter &other
        ) const;

        /// Access the numerator.
        [[nodiscard]] const std::vector<CoefficientType> &numerator() const { return b_; }

        /// Access the denominator.
        [[nodiscard]] const std::vector<CoefficientType> &denominator() const { return a_; }

        /// Set initial conditions on the input.
        /** To evaluate the output of the filter at the discrete time-step \f$ k \f$,
          * it is necessary to use the past values of the input,
          * *i.e.*, \f$ x_{k-1}, x_{k-2}, \cdots \f$. This function allows to
          * set these conditions to a given value.
          * @param input value \f$ x^\star \f$ to be used to initialize all samples,
          *   *i.e.*, \f$ x_{k}=x^\star, x_{k-1}=x^\star, x_{k-2}=x^\star, \cdots \f$.
          */
        void initInput(
            const DataType &input
        );

        /// Set initial conditions on the input.
        /** To evaluate the output of the filter at the discrete time-step \f$ k \f$,
          * it is necessary to use the current and the past values of the input,
          * *i.e.*, \f$ x_{k}, x_{k-1}, x_{k-2}, \cdots \f$. This function allows to
          * set the past values of \f$x\f$ (the current one will be set while calling
          * `filter()`).
          * @param input values \f$ \cdots, x_{k-2}, x_{k-1} \f$. Samples should be
          *   sorted in time-ascending order. In other words, the sequence starts
          *   with the oldest value and ends with the most recent sample. Note
          *   that the number of required values equals the dimension of the
          *   denominator minus one.
          */
        void initInput(
            const std::vector<DataType> &input
        );

        /// Set initial conditions on the output.
        /** To evaluate the output of the filter at the discrete time-step \f$ k \f$,
          * it is necessary to use the past values of the output,
          * *i.e.*, \f$ y_{k-1}, y_{k-2}, \cdots \f$. This function allows to
          * set these conditions to a given value.
          * @param output value \f$ y^\star \f$ to be used to initialize all samples,
          *   *i.e.*, \f$ y_{k-1}=y^\star, y_{k-2}=y^\star, \cdots \f$.
          */
        void initOutput(
            const DataType &output
        );

        /// Set initial conditions on the output.
        /** To evaluate the output of the filter at the discrete time-step \f$ k \f$,
          * it is necessary to use the past values of the output,
          * *i.e.*, \f$ y_{k-1}, y_{k-2}, \cdots \f$. This function allows to
          * set the past values of \f$y\f$.
          * @param output values \f$ \cdots, y_{k-2}, y_{k-1} \f$. Samples should be
          *   sorted in time-ascending order. In other words, the sequence starts
          *   with the oldest value and ends with the most recent sample. Note
          *   that the number of required values equals the dimension of the
          *   denominator minus one.
          */
        void initOutput(
            const std::vector<DataType> &output
        );

        /// Filter the current input.
        const DataType &filter(
            const DataType &x
        );

        /// Filter a whole sequence.
        /** Internal buffers will not be updated, meaning that the initial conditions
          * of the filter will not change after calling this method.
          * @param x input signal to be filtered. It should be sorted in
          *   time-ascending order, *i.e.*, so that `x[k]` corresponds to the
          *   discrete-time sample \f$ x_k \f$.
          * @param x0 initial input conditions. It should contain "past" input
          *   samples sorted in time-ascending order, *i.e.*,
          *   \f$ \cdots, x_{k-3}, x_{k-2}, x_{k-1} \f$
          * @param y0 initial output conditions. It should contain "past" output
          *   samples sorted in time-ascending order, *i.e.*,
          *   \f$ \cdots, y_{-3}, y_{-2}, y_{-1} \f$
          * @return sequence of filtered outputs \f$ y_0, y_1, y_2, \cdots \f$
          */
        [[nodiscard]] std::vector<DataType> filter(
            const std::vector<DataType> &x,
            const std::vector<DataType> &x0,
            const std::vector<DataType> &y0
        ) const;

        /// Bi-directional filtering of a whole sequence.
        /** Filter a given signal from both sides by applying the filter once and
          * then a second time on the reversed result.
          * @param x input signal to be filtered. It should be sorted in
          *   time-ascending order, *i.e.*, so that `x[k]` corresponds to the
          *   discrete-time sample \f$ x_k \f$.
          * @return sequence of filtered outputs \f$ y_0, y_1, y_2, \cdots \f$
          * @note for the moment, the initial conditions are obtained by holding
          *   the signal on the left and on the right.
          */
        [[nodiscard]] std::vector<DataType> filter2(
            const std::vector<DataType> &x
        ) const;

    private:
        std::vector<CoefficientType> b_; ///< Numerator of the transfer function.
        std::vector<CoefficientType> a_; ///< Denominator of the transfer function.
        std::deque<DataType> in_; ///< Input samples.
        std::deque<DataType> out_; ///< Output samples.
    };
} // namespace digital_filters

#include <digital_filters/filter.hxx>
