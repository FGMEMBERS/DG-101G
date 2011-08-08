# ############################################################################################
# ############################################################################################
# Nasal script for a total energy compensated variometer
#
# ############################################################################################
# Author: Klaus Kerner
# Version: 2011-08-02
#
# ############################################################################################
# Concepts:
# 1. check, whether the properties are created
# 1a. get current status and write to previus properties
# 2. after wait-cycle 
# 2a. get current status
# 2b. calculate te-vario
# 2c. write current status to previous properties
# 3. be happy and fly

# existing properties
# /position/altitude-ft
# /velocities/airspeed-kt

# new properties created by this script
# /instrumentation/te-vario/tmp/current-height-m
# /instrumentation/te-vario/tmp/previous-height-m
# /instrumentation/te-vario/tmp/current-airspeed-mps
# /instrumentation/te-vario/tmp/previous-airspeed-mps
# /instrumentation/te-vario/tmp/current-systime
# /instrumentation/te-vario/tmp/previous-systime
# /instrumentation/te-vario/tmp/delta-secs
# /instrumentation/te-vario/tmp/initialized
# /instrumentation/te-vario/te-reading-mps

# 

# ############################################################################################
# ############################################################################################
# calculate total energy variometer 
# 
# script from Flightgear Wiki used as basis



var tevariometer = func {

    var square = func(x) {return x*x};
    
    var location = "instrumentation/te-vario/";
    ##
    # expose all variables to the property tree for inspection and debugging
    var current_height_meters_prop    = location ~ "tmp/current-height-m";
    var previous_height_meters_prop   = location ~ "tmp/previous-height-m";
    
    var current_airspeed_mps_prop     = location ~ "tmp/current-airspeed-mps";
    var previous_airspeed_mps_prop    = location ~ "tmp/previous-airspeed-mps";
    
    var current_systime_prop          = location ~ "tmp/current-systime";
    var previous_systime_prop         = location ~ "tmp/previous-systime";
    var update_rate_sec_prop          = location ~ "tmp/delta-secs";
    
    var te_reading_prop               = location ~ "te-reading-mps";
    var tevario_init                  = location ~ "tmp/initialized";
    
    # initialize all properties with some defaults, if not allready existent:
    if (getprop(tevario_init) == nil) 
    {
        setprop(current_height_meters_prop,  getprop("position/altitude-ft") * FT2M);
        setprop(previous_height_meters_prop, getprop("position/altitude-ft") * FT2M);
        
        setprop(current_airspeed_mps_prop,   getprop("velocities/airspeed-kt") * KT2MPS);
        setprop(previous_airspeed_mps_prop,  getprop("velocities/airspeed-kt") * KT2MPS);
        
        setprop(current_systime_prop,        systime() );
        setprop(previous_systime_prop,       systime() );
        setprop(update_rate_sec_prop,        getprop(current_systime_prop) - 
                                              getprop(previous_systime_prop));
        setprop(te_reading_prop,             5.0);
        setprop(tevario_init,                1);
    }
    
    setprop(previous_airspeed_mps_prop,  getprop(current_airspeed_mps_prop));
    setprop(previous_systime_prop,       getprop(current_systime_prop) );
    setprop(previous_height_meters_prop, getprop(current_height_meters_prop));
    setprop(current_systime_prop,        systime() );
    setprop(update_rate_sec_prop,        getprop(current_systime_prop) - 
                                           getprop(previous_systime_prop));
    setprop(current_height_meters_prop,  getprop("position/altitude-ft") * FT2M);
    setprop(current_airspeed_mps_prop,   getprop("velocities/airspeed-kt") * KT2MPS);
    
    
    ## formula:
    # TE reading = (h2-h1)/t + (v2^2 - v1^2) / (19.62*t)
    var uncompensated = (getprop(current_height_meters_prop) - 
                          getprop(previous_height_meters_prop)) 
                          / getprop(update_rate_sec_prop);
    var adjustment = (square(getprop(current_airspeed_mps_prop)) - 
                       square(getprop(previous_airspeed_mps_prop))) 
                       / (19.62 * getprop(update_rate_sec_prop));
    
    
    ## some kind of damping to get a more smooth needle
    # get difference from last to current, correct delta with factor and add to last
    var te_reading_old   = getprop(te_reading_prop);              # old value
    var te_reading_the   = uncompensated + adjustment;            # theoretical value now
    var te_reading_del   = te_reading_the - te_reading_old;       # delta now to old
    var te_reading_cor   = 0.1 * te_reading_del;                  # amount of delta to add
    if ( getprop(current_airspeed_mps_prop) == 0 ) {
         var te_reading = 0.0;
     }
     else {
         var te_reading       = te_reading_old + te_reading_cor;
     }
    #
    # publish value to property tree
    setprop(te_reading_prop,te_reading);
    
    settimer(tevariometer, 0.1 );
  }


setlistener("sim/signals/fdm-initialized", tevariometer);
