#ifndef JOINT_PID_HPP
#define JOINT_PID_HPP

#include <type_traits>
#include <array>

template <typename T, int S, 
          std::enable_if_t<std::is_arithmetic<T>::value, bool> = true>
class JointPid
{
public:
    JointPid(float P, float I, float D)
        : P(P), I(I), D(D)
    {
    }

    std::array<T, S> update(std::array<T, S> current, std::array<T, S> expected)
    {
        std::array<T, S> output;
        for (unsigned i = 0; i < S; ++i)
        {
            current_error_[i] = expected[i] - current[i];
            cumulative_error_[i] += current_error_[i];
            output[i] = P * current_error_[i] +
                        I * cumulative_error_[i] +
                        D * (current_error_[i] - previous_error_[i]);
            previous_error_[i] = current_error_[i];
        }

        return output;
    }

private:
    std::array<T, S> current_error_;
    std::array<T, S> previous_error_;
    std::array<T, S> cumulative_error_;

    float P;
    float I;
    float D;
};

#endif // JOINT_PID_HPP