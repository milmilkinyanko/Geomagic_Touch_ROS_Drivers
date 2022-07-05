#pragma once

#include <vector>
#include <iostream>

class IIRFilter
{
private:
    int N;
    std::vector<double> m_coeff_input;
    std::vector<double> m_prev_input;
    std::vector<double> m_coeff_output;
    std::vector<double> m_prev_output;

    void updateInput(double input)
    {
        for (int i = 0; i < N; i++) {
            m_prev_input.at(N - i) = m_prev_input.at(N - i - 1);
        }
        m_prev_input.at(0) = input;
    }

    void updateOutput(double output)
    {
        for (int i = 0; i < N; i++) {
            m_prev_output.at(N - i) = m_prev_output.at(N - i - 1);
        }
        m_prev_output.at(0) = output;
    }

public:
    IIRFilter(int N = 0, std::vector<double> coeff_input = {0.0}, std::vector<double> coeff_output = {0.0}) : N(N)
    {
        this->m_coeff_input.resize(N + 1);
        this->m_coeff_output.resize(N + 1);
        this->m_prev_input.resize(N + 1);
        this->m_prev_output.resize(N + 1);
        this->m_coeff_output.at(0) = 0.0;  // output index: 1 to N, ignore prev_output[0]
        for (int i = 0; i < N; i++) {
            this->m_coeff_output.at(i + 1) = coeff_output.at(i);
        }
        for (int i = 0; i < N + 1; i++) {
            this->m_coeff_input.at(i) = coeff_input.at(i);
            this->m_prev_input.at(i) = 0.0;
            this->m_prev_output.at(i) = 0.0;
        }
    }

    double control(double input)
    {
        updateInput(input);
        double output = 0.0;
        for (int i = 0; i < N + 1; i++) {
            output += m_coeff_input.at(i) * m_prev_input.at(i)
                      + m_coeff_output.at(i) * m_prev_output.at(i);
        }
        updateOutput(output);
        return output;
    }
};
