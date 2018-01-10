// Evan Grissino
// 04/06/2016
// PID object class

#include "PID.h"

PID::PID(double set, double init) {
    // Gains
    P = 2;
    I = 1;
    D = 0;
    error_gain = 1;
    output_gain = 1;
    
    // State variables
    Derivator = 0;
    Integrator = 0;
    Integrator_max = 100;
    Integrator_min = -100;
    
    set_point = set;
    error = init - set;
    
    // calulated values
    P_val = 0;
    I_val = 0;
    D_val = 0;
};

void PID::begin(long time, int div) {
    start = time;       // start time in millisecond
    old_time = start;
    time_div = div;
}

void PID::set_limits(double low, double high) {
    Integrator_max = high;
    Integrator_min = low;
}

void PID::set_P(double p_val) {
    P = p_val;
};

void PID::set_I(double i_val) {
    I = i_val;
};

void PID::set_D(double d_val) {
    D = d_val;
};

void PID::change( double delta ) {
    set_point += delta;
}

void PID::set(double set) {
    set_point = set;
};

void PID::setTimeDiv(double _time_div) {
    time_div = _time_div;
};

void PID::reset( long time ) {
    Integrator = 0;
    Derivator = 0;
    begin(time, time_div);
}

void PID::setPID ( float PID_vals[] ) {
    set_P( PID_vals[0] );
    set_I( PID_vals[1] );
    set_D( PID_vals[2] );
}

void PID::setPID (double p_val, double i_val, double d_val) {
    set_P( p_val );
    set_I( i_val );
    set_D( d_val );
}

void PID::setGains (double e_gain, double o_gain) {
    error_gain  = e_gain;
    output_gain = o_gain;
}

double PID::update(double current) {
    double old_error = error;
    error = (set_point - current) * error_gain;
    
    Integrator += error ;
    Derivator = (error - old_error);
    
    P_val = P * error;
    I_val = I * Integrator;
    D_val = D * Derivator;
    
    if (Integrator >= Integrator_max) {
        Integrator = Integrator_max;
    }
    if (Integrator <= Integrator_min) {
        Integrator = Integrator_min;
    }
    
    return (P_val + I_val + D_val) * output_gain;
};

double PID::update(double current, long time) {
    dt = time - old_time;
    old_time = time;
    
    double old_error = error;
    error = (set_point - current) * error_gain;
    
    Integrator += error * dt / time_div ;
    Derivator = (error - old_error) / dt * time_div;
    
    P_val = P * error;
    I_val = I * Integrator;
    D_val = D * Derivator;
    
    return (P_val + I_val + D_val) * output_gain;
};
