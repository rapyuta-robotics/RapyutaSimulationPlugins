// Class representing a first-order lag system
class FirstOrderSystem
{
public:
    // Constructor
    FirstOrderSystem(const double tau = 0.0, const double minDt = 1e-6) : _tau(tau), _y(0.0), _lastUpdateTime(-1.0), _minDt(minDt)
    {
    }

    // One-step update function
    double update(double currentTime, double input)
    {
        if (_lastUpdateTime < 0.0 || (currentTime - _lastUpdateTime) < _minDt)
        {
            // Return output as is if it's the first call or if currentTime and _lastUpdateTime are too close
            _lastUpdateTime = currentTime;
            return _y;
        }

        double dt = currentTime - _lastUpdateTime;
        _y = (_tau * _y + dt * input) / (_tau + dt);
        _lastUpdateTime = currentTime;
        return _y;
    }

    // One-step update function by dt
    double update2(double dt, double input)
    {
        if (dt < _minDt)
        {
            // Return output as is if dt is too small
            return _y;
        }
        _y = (_tau * _y + dt * input) / (_tau + dt);
        return _y;
    }

    // Function to get the current output
    double getOutput() const
    {
        return _y;
    }

    void setTau(const double tau)
    {
        _tau = tau;
    }

private:
    double _tau;               // Time constant
    double _y;                 // Output
    double _lastUpdateTime;    // Last update time
    const double _minDt;       // Minimum dt threshold
};
