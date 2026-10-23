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
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

// Relative tolerance for recognising an acceleration-only arrival despite
// square-root rounding: both the peak velocity against ve and the distance
// covered by accelerating from v0 to ve against |dp| must agree within it.
constexpr double ACCELERATION_ONLY_REL_TOL = 1e-9;

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
            } else if (_ve == 0) {
                // Moving through the target: stop beyond it, then come back.
                return calcStopThenReturn();
            } else {
                // TODO(two_points_interpolation_py#9): reverse profile -- move away and come
                // back to change speed without changing position.
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
        
        // Peak velocity v1 at the end of the acceleration phase, from the distance
        // covered by the two phases (accelerate v0 -> v1, decelerate v1 -> ve):
        //
        //   |dp| = (v1^2 - v0^2) / (2 * amax_accel) + (v1^2 - ve^2) / (2 * amax_decel)
        //
        // This fixes |v1| but not its sign. Solving the equivalent quadratic in t1
        // instead yields two roots, v1 = +|v1| and v1 = -|v1|; the second ends the
        // acceleration phase still moving away from pe, and its "deceleration" then
        // drives further away, so the trajectory never reaches pe. Both roots are
        // positive exactly when v0 points away from pe, so the root has to be chosen
        // by direction, not by size: v1 points toward pe.
        {
            const double a_mag = _amax_accel;
            const double d_mag = _amax_decel;
            const double v1_sq = (std::fabs(dp) + _v0 * _v0 / (2 * a_mag) + _ve * _ve / (2 * d_mag)) /
                                 (1 / (2 * a_mag) + 1 / (2 * d_mag));
            double v1 = sign * std::sqrt(v1_sq);

            // An acceleration-only arrival has v1 == ve exactly in real arithmetic, but
            // the square root can land a rounding error below ve. Snap the peak onto ve,
            // so the deceleration phase has zero length instead of the trajectory being
            // refused as unreachable -- but only when it really is one: accelerating from
            // v0 to ve must cover |dp|. A peak close to ve alone is not enough (a tiny
            // dec_max puts the peak near ve while ve is far out of reach).
            const double gap = sign * (_ve - v1);
            if (gap > 0 && gap <= ACCELERATION_ONLY_REL_TOL * std::max(std::fabs(v1), std::fabs(_ve))) {
                const double acceleration_only_distance = (_ve * _ve - _v0 * _v0) / (2 * a_mag);
                const double distance_error = std::fabs(acceleration_only_distance - std::fabs(dp));
                if (distance_error <= ACCELERATION_ONLY_REL_TOL * std::fabs(dp)) {
                    v1 = _ve;
                }
            }

            // Acceleration phase duration. Not positive when the body already moves toward
            // pe at least as fast as v1, i.e. it cannot slow down to ve within |dp|.
            double dt01 = (v1 - _v0) / acc;
            // TODO(two_points_interpolation_py#9): reverse profile (decelerate -> accelerate)
            // for overshoot / back-up trajectories; both checks below would then have a
            // trajectory to return.
            if (dt01 <= 0) {
                // Too fast to slow down to ve before pe: overshoot, stop, and come back.
                const double decel_distance = (_v0 * _v0 - _ve * _ve) / (2 * _amax_decel);
                if (sign * _v0 > 0 && decel_distance > std::fabs(dp) && sign * _ve <= 0) {
                    return calcStopThenReturn();
                }
                throwCannotStopInTime(dp, dec, sign);
            }
            // The deceleration phase must slow v1 down to ve; if ve is faster than v1 in
            // the direction of travel, ve cannot be reached within |dp|.
            if (sign * (v1 - _ve) < 0) {
                throwEndVelocityUnreachable(dp);
            }

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
                
                // Phase 1: Acceleration or deceleration to reach vmax
                v1 = _vmax * sign;
                double a_phase1;
                double p1;
                if (sign * _v0 < sign * v1) {
                    // Normal case: accelerate to vmax
                    dt01 = std::fabs((v1 - _v0) / acc);
                    a_phase1 = acc;
                } else {
                    // Overspeed case: already moving toward the target faster than vmax,
                    // so decelerate to vmax
                    if (_verbose) {
                        std::cout << "Initial velocity (" << std::fabs(_v0) << ") exceeds vmax (" << _vmax
                                  << "); the trajectory starts by decelerating to vmax." << std::endl;
                    }
                    dt01 = std::fabs((v1 - _v0) / dec);
                    a_phase1 = -dec;
                }
                p1 = pInteg(_p0, _v0, a_phase1, dt01);

                _dt.push_back(dt01);
                _a.push_back(a_phase1);
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
        }

        if (_verbose) {
            std::cout << "case " << _caseNum << std::endl;
            std::cout << "dt ";
            for (double t : _dt) {
                std::cout << t << " ";
            }
            std::cout << std::endl;
            std::cout << "a ";
            for (double a_i : _a) {
                std::cout << a_i << " ";
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
    // Case 2: brake at the full deceleration until the body stands still, then plan an
    // ordinary trajectory from there back to (pe, ve). Used when v0 carries the body to
    // or past pe before it can slow down to ve, which no accelerate-then-decelerate
    // profile can reach. The first phase is the braking one; the return phases follow.
    double calcStopThenReturn() {
        const double t_stop = std::fabs(_v0) / _amax_decel;
        const double a_stop = _v0 > 0 ? -_amax_decel : _amax_decel;
        const double p_stop = pInteg(_p0, _v0, a_stop, t_stop);

        TwoPointInterpolation back(_verbose);
        back.init(p_stop, _pe, _amax_accel, _vmax, 0.0, 0.0, _ve, _amax_decel);
        back.calcTrajectory();

        _dt.assign(1, t_stop);
        _a.assign(1, a_stop);
        _v.assign(1, _v0);
        _p.assign(1, _p0);
        for (size_t i = 0; i < back._dt.size(); ++i) {
            _dt.push_back(back._dt[i]);
            _a.push_back(back._a[i]);
            _v.push_back(back._v[i]);
            _p.push_back(back._p[i]);
        }
        _caseNum = 2;
        _trajectoryCalced = true;

        if (_verbose) {
            std::cout << "case 2: stop at " << p_stop << " after " << t_stop << " s, then return" << std::endl;
        }
        return sum(_dt, _dt.size());
    }

    // The body already moves toward the target faster than it can slow down to ve
    // within |dp|.
    [[noreturn]] void throwCannotStopInTime(const double dp, const double dec, const double sign) const {
        // Minimum distance needed to slow from v0 to ve: d = (v0^2 - ve^2) / (2 * dec)
        const double decel_distance = (_v0 * _v0 - _ve * _ve) / (2 * std::fabs(dec));
        const bool moving_toward_target = sign * _v0 > 0;
        std::ostringstream msg;
        msg << std::fixed << std::setprecision(4);
        if (moving_toward_target && decel_distance > std::fabs(dp)) {
            msg << "Insufficient distance to decelerate: current velocity " << std::fabs(_v0)
                << " requires " << decel_distance << " distance to reach target velocity "
                << std::fabs(_ve) << ", but only " << std::fabs(dp) << " available. "
                << "Consider reducing initial velocity or increasing distance.";
        } else {
            // Exact boundary: the deceleration distance equals |dp| (t1 == 0), so a
            // pure deceleration would arrive exactly. Refused, as before; tracked as
            // a follow-up in two_points_interpolation_py#8.
            msg << "Insufficient distance for trajectory planning: no time is left for the "
                << "acceleration phase. Distance: " << std::fabs(dp) << ", v0: " << _v0
                << ", ve: " << _ve << ", acc_max: " << _amax_accel << ", dec_max: " << _amax_decel;
        }
        throw std::runtime_error(msg.str());
    }

    // ve is faster, in the direction of travel, than any speed reachable within |dp|
    // without first moving away from the target.
    [[noreturn]] void throwEndVelocityUnreachable(const double dp) const {
        // Accelerating toward the target over the whole distance (after first stopping,
        // if v0 points away) ends at v^2 = v0^2 + 2 * acc_max * |dp|. That is the limit
        // for this accelerate-then-decelerate profile only: backing away first to gain
        // run-up distance would reach ve (two_points_interpolation_py#9).
        const double reachable = std::sqrt(_v0 * _v0 + 2 * _amax_accel * std::fabs(dp));
        std::ostringstream msg;
        msg << std::fixed << std::setprecision(4)
            << "End velocity unreachable without first moving away from the target: "
            << "the highest speed reachable over distance " << std::fabs(dp) << " from v0 "
            << _v0 << " is " << reachable << ", but ve " << _ve
            << " was requested. acc_max: " << _amax_accel << ", dec_max: " << _amax_decel;
        throw std::runtime_error(msg.str());
    }

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
