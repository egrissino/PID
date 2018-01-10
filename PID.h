// Evan Grissino
// 04/06/2016
//
// Update LOG
// 11/30/17 - Added reset time and setPID(float [] )
// PID object class

#ifndef PID_H
#define PID_H

class PID {
    private:
        int P, I, D;
        int Integrator, Derivator;
        int Integrator_max, Integrator_min;
        int time_div;
        long start, old_time;
        double P_val, I_val, D_val, dt;
        double set_point, error, error_gain, output_gain;
    
    public:
    //PID();
    PID(double set, double init);
    
    double update(double current, long time);
    double update(double current);
    
    void set_P (double p_val);
    void set_I (double i_val);
    void set_D (double d_val);
    void setPID (double p_val, double i_val, double d_val);
    void setPID (float PID_vals[]);
    
    void setTimeDiv(double _time_div);
    void setGains( double e_gain, double o_gain );
    void set_limits( double low, double high );
    
    void change( double delta );
    void set ( double set );
    void begin( long time, int div );
    void reset( long time );
};

#endif