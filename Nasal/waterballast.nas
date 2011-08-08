# ############################################################################################
# ############################################################################################
# Nasal script to manage waterballast for the DG-101G
#
# ############################################################################################
# Author: Klaus Kerner
# Version: 2010-08-02
#
# ############################################################################################
# Concepts:
# 1. check water level
# 2. toggle dumping water
# 3. refill if desired
# 4. be happy and fly

# existing proterties, that are used to handle the waterballast

# ## required properties from the jsbsim config file,point masses
# /fdm/jsbsim/inertia/pointmass-weight-lbs[1]        tank 1, created by jsbsim config file
# /fdm/jsbsim/inertia/pointmass-weight-lbs[2]        tank 2, created by jsbsim config file

# ## new properties to handle the balast and animations
# /sim/glider/ballast/mass_per_tank_lbs              initial ballast per tank
# /sim/glider/ballast/mass_lbs_per_second            amount of water for dropping
# /sim/glider/ballast/drop                           flag for dropping water balast
#                                                      1: drop water 
#                                                      0: do not drop water



# ############################################################################################
# ############################################################################################
# initialise water ballast system
var initBallast = func {
  # close the valve
  setprop("sim/glider/ballast/drop", 0);
  # check for defined max amount of water per tank by config file
  # if not set a default value of 50kg (110lbs) per tank
  if ( getprop("sim/glider/ballast/mass_per_tank_lbs") == nil ) {
    atc_msg("amount of ballast not defined by plane, use default setting of 110lbs");
    setprop("sim/glider/ballast/mass_per_tank_lbs", 110);
  }
  # check for defined water flow at dropping ballast, 
  # if not set a default value
  if ( getprop("sim/glider/ballast/mass_lbs_per_second") == nil ) {
    atc_msg(" ballast drop not defined by plane, use default setting of 2lbs per second");
    setprop("sim/glider/ballast/mass_lbs_per_second", 2);
  }
}



# ############################################################################################
# ############################################################################################
# load water ballast
var loadBallast = func {
  
  # fill the tanks
  setprop("fdm/jsbsim/inertia/pointmass-weight-lbs[1]", 
              getprop("sim/glider/ballast/mass_per_tank_lbs") );
  setprop("fdm/jsbsim/inertia/pointmass-weight-lbs[2]", 
              getprop("sim/glider/ballast/mass_per_tank_lbs") );
  
  atc_msg("tanks loaded with water");
  
}


# ############################################################################################
# ############################################################################################
# dump water ballast
var toggleBallastDump = func {
  
  var deltaballast = getprop("sim/glider/ballast/mass_lbs_per_second");
  var tank1 = getprop("fdm/jsbsim/inertia/pointmass-weight-lbs[1]");
  var tank2 = getprop("fdm/jsbsim/inertia/pointmass-weight-lbs[2]");
  var status = getprop("sim/glider/ballast/drop");
  
  if ( status > 0 ) {
    if ( tank1 <= deltaballast ) {
      interpolate("fdm/jsbsim/inertia/pointmass-weight-lbs[1]", 0, 1);
    }
    else {
      interpolate("fdm/jsbsim/inertia/pointmass-weight-lbs[1]", tank1-deltaballast, 1);
    }
    
    if ( tank2 <= deltaballast ) {
      interpolate("fdm/jsbsim/inertia/pointmass-weight-lbs[2]", 0, 1);
    }
    else {
      interpolate("fdm/jsbsim/inertia/pointmass-weight-lbs[2]", tank2-deltaballast, 1);
    }
    
    interpolate("sim/glider/ballast/drop", 0 , 1);
  }
  else {
    interpolate("sim/glider/ballast/drop", 1 , 1);
    
    var time1 = tank1/deltaballast;
    var time2 = tank2/deltaballast;
    
    interpolate("fdm/jsbsim/inertia/pointmass-weight-lbs[1]", 0, time1);
    interpolate("fdm/jsbsim/inertia/pointmass-weight-lbs[2]", 0, time2);
  }
  
}

var initializing_ballast = setlistener("sim/signals/fdm-initialized", initBallast);
