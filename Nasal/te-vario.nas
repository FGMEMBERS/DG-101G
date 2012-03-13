# ####################################################################################
# ####################################################################################
# Nasal script for a total energy compensated variometer
#
# script from Flightgear Wiki used as basis

# ####################################################################################
# Author: Klaus Kerner
# Version: 2012-03-08
#
# ####################################################################################
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
# /instrumentation/te-vario/tmp/delta-secs
# /instrumentation/te-vario/te-reading-mps

# 

# helper function
var square = func(x) {return x*x};


var initTeVario = func {
  # root node for TE Vario properties
  var location = "instrumentation/te-vario/";
  # nodes for TE Vario properties
  var node_current_height_m         = location ~ "tmp/current-height-m";
  var node_previous_height_m        = location ~ "tmp/previous-height-m";
  var node_current_airspeed_mps     = location ~ "tmp/current-airspeed-mps";
  var node_previous_airspeed_mps    = location ~ "tmp/previous-airspeed-mps";
  var node_update_rate_sec          = location ~ "tmp/delta-secs";
  var node_te_reading               = location ~ "te-reading-mps";
  
  # initialize all properties with some defaults, if not allready existent:
  setprop(node_current_height_m,       getprop("position/altitude-ft") * FT2M);
  setprop(node_previous_height_m,      getprop("position/altitude-ft") * FT2M);
  setprop(node_current_airspeed_mps,   getprop("velocities/airspeed-kt") * KT2MPS);
  setprop(node_previous_airspeed_mps,  getprop("velocities/airspeed-kt") * KT2MPS);
  setprop(node_update_rate_sec,        getprop("sim/time/delta-sec"));
  setprop(node_te_reading,             5.0);
  
  # start running TE Vario 1sec after system is up
  settimer(runTeVario, 1.0 );
}

var runTeVario = func {
  # root node for TE Vario properties
  var location = "instrumentation/te-vario/";
  # nodes for TE Vario properties
  var node_current_height_m         = location ~ "tmp/current-height-m";
  var node_previous_height_m        = location ~ "tmp/previous-height-m";
  var node_current_airspeed_mps     = location ~ "tmp/current-airspeed-mps";
  var node_previous_airspeed_mps    = location ~ "tmp/previous-airspeed-mps";
  var node_update_rate_s            = location ~ "tmp/delta-secs";
  var node_te_reading               = location ~ "te-reading-mps";
  
  # shift former current properties to now previous properties
  setprop(node_previous_airspeed_mps,  getprop(node_current_airspeed_mps));
  setprop(node_previous_height_m, getprop(node_current_height_m));
  # get time increment since last run
  setprop(node_update_rate_s,        getprop("sim/time/delta-sec"));
  # get current properties
  setprop(node_current_height_m,  getprop("position/altitude-ft") * FT2M);
  setprop(node_current_airspeed_mps,   getprop("velocities/airspeed-kt") * KT2MPS);
  
  # formula (see also wiki):
  # TE reading = (h2-h1)/t + (v2^2 - v1^2) / (19.62*t)
  var uncompensated = (getprop(node_current_height_m) - 
                        getprop(node_previous_height_m)) 
                        / getprop(node_update_rate_s);
  var adjustment = (square(getprop(node_current_airspeed_mps)) - 
                     square(getprop(node_previous_airspeed_mps))) 
                     / (19.62 * getprop(node_update_rate_s));
  
  # some kind of damping to get a more smooth needle
  # get difference from last to current, correct delta with factor and add to last
  var te_reading_old   = getprop(node_te_reading);             # old value
  var te_reading_the   = uncompensated + adjustment;           # theoretical value now
  var te_reading_del   = te_reading_the - te_reading_old;      # delta now to old
  # correction value, can be adjusted by last factor to simulate different latencies
  var te_reading_cor   = getprop(node_update_rate_s) * te_reading_del * 0.5;
  if ( getprop(node_current_airspeed_mps) == 0 ) {
    var te_reading = 0.0;
  }
  else {
    var te_reading       = te_reading_old + te_reading_cor;
  }
  
  # publish value to property tree
  setprop(node_te_reading,te_reading);
  
  settimer(runTeVario, 0 );
}

setlistener("sim/signals/fdm-initialized", initTeVario);
