// Copyright 2025 Yu Okamoto
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

inline double vInteg(const double v0, const double a, const double dt) {
    return v0 + a * dt;
}

inline double pInteg(const double p0, const double v0, const double a, const double dt) {
    return p0 + v0 * dt + 0.5 * a * dt * dt;
}

inline double normalizeAxis(const double input){
    double output = fmod(input + M_PI, 2 * M_PI);
    if (output < 0)
    {
        output += 2 * M_PI;
    }
    return output - M_PI;
}

class TwoPointInterpolation {
private:
    bool _pointSetted;
    bool _constraintsSetted;
    bool _initialStateSetted;
    bool _trajectoryCalced;
    bool _verbose;

    double _t0;
    double _p0;
    double _v0;
    double _pe;
    double _ve;
    double _amax_accel;
    double _amax_decel;
    double _vmax;
    std::vector<double> _dt;
    std::vector<double> _a;
    std::vector<double> _v;
    std::vector<double> _p;
    int _caseNum;

public:
    TwoPointInterpolation(const bool verbose = false) {
        _pointSetted = false;
        _constraintsSetted = false;
        _initialStateSetted = false;
        _trajectoryCalced = false;
        _verbose = verbose;
    }

    void setInitial(const double t0, const double p0, const double v0 = 0) {
        _t0 = t0;
        _p0 = p0;
        _v0 = v0;
        _initialStateSetted = true;
    }

    void setPoint(const double pe, const double ve = 0) {
        _pe = pe;
        _ve = ve;
        _pointSetted = true;
    }

    void setConstraints(const double amax, const double vmax, const double dec_max = -1.0) {
        if (amax <= 0) {
            throw std::invalid_argument("amax must be positive");
        }
        if (vmax <= 0) {
            throw std::invalid_argument("vmax must be positive");
        }
        _amax_accel = amax;
        _vmax = vmax;
        
        // Default deceleration to acceleration if not specified
        if (dec_max < 0) {
            _amax_decel = amax;
        } else {
            if (dec_max == 0) {
                throw std::invalid_argument("dec_max must be positive (non-zero)");
            }
            _amax_decel = dec_max;
        }
        _constraintsSetted = true;
    }

    bool isInitialized()
    {
        return _pointSetted && _constraintsSetted && _initialStateSetted && _trajectoryCalced;
    }

    void init(const double p0, const double pe, 
              const double amax, const double vmax, 
              const double t0 = 0, const double v0 = 0, 
              const double ve = 0, const double dec_max = -1.0) {
        setInitial(t0, p0, v0);
        setPoint(pe, ve);
        setConstraints(amax, vmax, dec_max);
    }

    double calcTrajectory() {
        if (!_pointSetted) {
            throw std::runtime_error("End point not set. Call setPoint() first.");
        }
        if (!_constraintsSetted) {
            throw std::runtime_error("Constraints not set. Call setConstraints() first.");
        }
        if (!_initialStateSetted) {
            throw std::runtime_error("Initial state not set. Call setInitial() first.");
        }

        double dp = _pe - _p0;
        double dv = _ve - _v0;

        // Check if start and end positions are the same
        if (dp == 0) {
            if (dv == 0) {
                // No movement needed, trajectory is already complete
                _dt.clear();
                _a.clear();
                _v.clear();
                _p.clear();
                _v.push_back(_v0);
                _p.push_back(_p0);
                _caseNum = -1;  // Special case for no movement
                _trajectoryCalced = true;
                return 0.0;
            } else {
                throw std::invalid_argument("Cannot have different velocities at the same position (dp=0, but dv!=0)");
            }
        }

        _dt.clear();
        _a.clear();
        _v.clear();
        _p.clear();

        _v.push_back(_v0);
        _p.push_back(_p0);

        // Direction sign
        double sign = dp / std::fabs(dp);
        double acc = _amax_accel * sign;
        double dec = _amax_decel * sign;
        
        // Calculate coefficients for quadratic equation
        double ratio = acc / dec;
        double a_coeff = 0.5 * acc * (1 + ratio);
        double b_coeff = _v0 * (1 + ratio);
        double c_coeff = -dp + (_v0 * _v0 - _ve * _ve) / (2 * dec);
        
        // Calculate discriminant
        double discriminant = b_coeff * b_coeff - 4 * a_coeff * c_coeff;
        
        if (discriminant > 0) { 
            // Solve for t1 (acceleration duration)
            // Quadratic equation has two solutions, choose the positive one
            double sqrt_disc = std::sqrt(discriminant);
            double dt01_plus = (-b_coeff + sqrt_disc) / (2 * a_coeff);
            double dt01_minus = (-b_coeff - sqrt_disc) / (2 * a_coeff);
            
            // Choose the positive solution
            double dt01;
            if (dt01_plus > 0 && dt01_minus > 0) {
                // Both positive: choose the smaller one (more efficient)
                dt01 = std::min(dt01_plus, dt01_minus);
            } else if (dt01_plus > 0) {
                dt01 = dt01_plus;
            } else if (dt01_minus > 0) {
                dt01 = dt01_minus;
            } else {
                throw std::runtime_error("No positive time solution found for trajectory");
            }
            
            double v1 = vInteg(_v0, acc, dt01);
            if (std::fabs(v1) < _vmax) { // not reach the vmax
                _caseNum = 0;
                double p1 = pInteg(_p0, _v0, acc, dt01);
                double dt1e = std::fabs((v1 - _ve) / dec);
                _dt.push_back(dt01);
                _dt.push_back(dt1e);
                _a.push_back(acc);
                _a.push_back(-dec);
                _v.push_back(v1);
                _p.push_back(p1);
            } else {
                _caseNum = 1;
                
                // Phase 1: Acceleration (_v0 → vmax)
                v1 = _vmax * dp / std::fabs(dp);
                dt01 = (v1 - _v0) / acc;
                double p1 = pInteg(_p0, _v0, acc, dt01);
                
                _dt.push_back(dt01);
                _a.push_back(acc);
                _v.push_back(v1);
                _p.push_back(p1);

                // Phase 3: Deceleration (vmax → ve)
                double v2 = v1;
                double dt2e = std::fabs((v2 - _ve) / dec);
                double dp2e = pInteg(0, v2, -dec, dt2e);
                
                // Phase 2: Constant velocity (vmax maintained)
                double dt12 = (_pe - p1 - dp2e) / v1;
                
                // Mathematical note: dt12 should always be >= 0 in theory
                // because Case 0's solution satisfies pe exactly, and limiting v1 to vmax
                // reduces the required distance. If dt12 < 0, it indicates:
                // - Numerical error (floating-point precision limits)
                // - Implementation bug
                // - Invalid input data
                if (dt12 < 0) {
                    throw std::runtime_error(
                        "Invalid trajectory: cannot reach target with given constraints. "
                        "Distance too short (" + std::to_string(std::fabs(dp)) + ") for vmax (" + 
                        std::to_string(_vmax) + "). Consider reducing vmax or increasing distance."
                    );
                }
                
                double p2 = _pe - dp2e;
                _dt.push_back(dt12);
                _dt.push_back(dt2e);
                _a.push_back(0.0);
                _a.push_back(-dec);
                _v.push_back(v2);
                _p.push_back(p2);
            }
        } else {
            if (_verbose) {
                std::cout << "TwoPointInterpolation::calcTrajectory error" << std::endl;
            }
            throw std::runtime_error("No valid trajectory found (discriminant < 0). "
                                   "The constraints might be too restrictive for the given end conditions.");
        }

        if (_verbose) {
            std::cout << "case " << _caseNum << std::endl;
            std::cout << "dt ";
            for (double t : _dt) {
                std::cout << t << " ";
            }
            std::cout << std::endl;
            std::cout << "a ";
            for (double acc2 : _a) {
                std::cout << acc2 << " ";
            }
            std::cout << std::endl;
            std::cout << "v ";
            for (double vel : _v) {
                std::cout << vel << " ";
            }
            std::cout << std::endl;
            std::cout << "p ";
            for (double pos : _p) {
                std::cout << pos << " ";
            }
            std::cout << std::endl;
        }

        _trajectoryCalced = true;

        double totalDt = 0;
        for (double t : _dt) {
            totalDt += t;
        }

        return totalDt;
    }

    double calcTrajectory(const double p0, const double pe, 
                          const double amax, const double vmax, 
                          const double t0 = 0, const double v0 = 0, 
                          const double ve = 0, const double dec_max = -1.0) {
        init(p0, pe, amax, vmax, t0, v0, ve, dec_max);
        return calcTrajectory();
    }

    std::vector<double> getPoint(const double t) const {
        double a = 0;
        double v = 0;
        double pos = 0;

        // Handle special case where no movement is needed (dp=0, dv=0)
        if (_caseNum == -1) {
            a = 0.0;
            v = _v0;
            pos = _p0;
            std::vector<double> result = {pos, v, a};
            return result;
        }

        double tau = t - _t0;

        if (tau < 0) {
            a = 0.0;
            v = _v0;
            pos = _p0;
        } else if (tau >= sum(_dt, _dt.size() + 1)) {
            a = 0.0;
            v = _ve;
            pos = _pe;
        } else {
            double a_in = 0.0;
            double v_in = 0.0;
            double p_in = 0.0;
            double t_in = tau;
            for (size_t i = 0; i < _dt.size(); i++) {
                double dt_i = sum(_dt, i + 1);
                if (tau <= dt_i) {
                    t_in = tau - sum(_dt, i);
                    a_in = _a[i];
                    v_in = _v[i];
                    p_in = _p[i];
                    break;
                }
            }

            a = a_in;
            v = vInteg(v_in, a_in, t_in);
            pos = pInteg(p_in, v_in, a_in, t_in);
        }

        std::vector<double> result = {pos, v, a};
        return result;
    }

    // Getters for testing and debugging
    double getVmax() const { return _vmax; }
    double getAmaxAccel() const { return _amax_accel; }
    double getAmaxDecel() const { return _amax_decel; }
    const std::vector<double>& getDt() const { return _dt; }

private:
    double sum(const std::vector<double>& values, const size_t count) const {
        double total = 0.0;
        for (size_t i = 0; i < count && i < values.size(); ++i) {
            total += values[i];
        }
        return total;
    }
};

class TwoAngleInterpolation : public TwoPointInterpolation {
private:
    bool _normalize_output = true;
public:
    TwoAngleInterpolation(const bool verbose = false) : TwoPointInterpolation(verbose) {}

    void init(const double p0, const double pe, 
              const double amax, const double vmax, 
              const double t0 = 0, const double v0 = 0, 
              const double ve = 0, const double dec_max = -1.0) {
        
        const double p0n = normalizeAxis(p0);
        const double pen = normalizeAxis(pe);
        const double dp = normalizeAxis(pen - p0n);

        setInitial(t0, p0n, v0);
        setPoint(p0n+dp, ve);
        setConstraints(amax, vmax, dec_max);
    }

    double calcTrajectory(const double p0, const double pe, 
                          const double amax, const double vmax, 
                          const double t0 = 0, const double v0 = 0, 
                          const double ve = 0, const double dec_max = -1.0) {
        init(p0, pe, amax, vmax, t0, v0, ve, dec_max);
        return TwoPointInterpolation::calcTrajectory();
    }

    std::vector<double> getPoint(const double t, const bool normalize = true) const {
        std::vector<double> result = TwoPointInterpolation::getPoint(t);
        if (normalize)
        {
            result[0] = normalizeAxis(result[0]);
        }
        return result;
    }
};