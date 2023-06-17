#include <iostream>
#include <cmath>
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
    double _amax;
    double _vmax;
    std::vector<double> _dt;
    std::vector<double> _a;
    std::vector<double> _v;
    std::vector<double> _p;
    double _aSigned;
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

    void setConstraints(const double amax, const double vmax) {
        _amax = amax;
        _vmax = vmax;
        _constraintsSetted = true;
    }

    bool isInitialized()
    {
        return _pointSetted && _constraintsSetted && _initialStateSetted && _trajectoryCalced;
    }

    void init(const double p0, const double pe, 
              const double amax, const double vmax, 
              const double t0 = 0, const double v0 = 0, 
              const double ve = 0) {
        setInitial(t0, p0, v0);
        setPoint(pe, ve);
        setConstraints(amax, vmax);
    }

    double calcTrajectory() {
        double dp = _pe - _p0;
        double dv = _ve - _v0;

        _dt.clear();
        _a.clear();
        _v.clear();
        _p.clear();

        _v.push_back(_v0);
        _p.push_back(_p0);

        _aSigned = _amax * dp / std::fabs(dp); 
        double b = _v0 / _aSigned;
        double c = (-dv * (_ve + _v0) * 0.5 / _aSigned - dp) / _aSigned;
        if (b * b - c > 0) { 
            double dt01 = -b + std::sqrt(b * b - c);
            double v1 = vInteg(_v0, _aSigned, dt01);
            if (std::fabs(v1) < _vmax) { // not reach the vmax
                _caseNum = 0;
                double p1 = pInteg(_p0, _v0, _aSigned, dt01);
                double dt1e = dt01 - dv / _aSigned;
                _dt.push_back(dt01);
                _dt.push_back(dt1e);
                _a.push_back(_aSigned);
                _a.push_back(-_aSigned);
                _v.push_back(v1);
                _p.push_back(p1);
            } else {
                _caseNum = 1;
                v1 = _vmax * dp / std::fabs(dp);
                dt01 = (v1 - _v0) / _aSigned;
                double p1 = pInteg(_p0, _v0, _aSigned, dt01);
                _dt.push_back(dt01);
                _a.push_back(_aSigned);
                _v.push_back(v1);
                _p.push_back(p1);
                double v2 = v1;
                double dt2e = (_ve - v2) / -_aSigned;
                double dp2e = pInteg(0, v2, -_aSigned, dt2e);
                double dt12 = (_pe - p1 - dp2e) / v1;
                double p2 = _pe - dp2e;
                _dt.push_back(dt12);
                _dt.push_back(dt2e);
                _a.push_back(0.0);
                _a.push_back(-_aSigned);
                _v.push_back(v2);
                _p.push_back(p2);
            }
        } else {
            if (_verbose) {
                std::cout << "TwoPointInterpolation::calcTrajectory error" << std::endl;
            }
            return -1;
        }

        if (_verbose) {
            std::cout << "case " << _caseNum << std::endl;
            std::cout << "dt ";
            for (double t : _dt) {
                std::cout << t << " ";
            }
            std::cout << std::endl;
            std::cout << "a ";
            for (double acc : _a) {
                std::cout << acc << " ";
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
                          const double ve = 0) {
        init(p0, pe, amax, vmax, t0, v0, ve);
        return calcTrajectory();
    }

    std::vector<double> getPoint(const double t) const {
        double a = 0;
        double v = 0;
        double pos = 0;

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
            for (int i = 0; i < _dt.size(); i++) {
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

private:
    double sum(const std::vector<double>& values, const int count) const {
        double total = 0.0;
        for (int i = 0; i < count && i < values.size(); ++i) {
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
              const double ve = 0) {
        
        const double p0n = normalizeAxis(p0);
        const double pen = normalizeAxis(pe);
        const double dp = normalizeAxis(pen - p0n);

        setInitial(t0, p0n, v0);
        setPoint(p0n+dp, ve);
        setConstraints(amax, vmax);
    }

    double calcTrajectory(const double p0, const double pe, 
                          const double amax, const double vmax, 
                          const double t0 = 0, const double v0 = 0, 
                          const double ve = 0) {
        init(p0, pe, amax, vmax, t0, v0, ve);
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