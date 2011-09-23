# ############################################################################################
# ############################################################################################
# Nasal script for dialogs
#
# ############################################################################################
# Author: Klaus Kerner
# Version: 2011-09-23
#

# ############################################################################################
# basic fucntions to create dialogs
var config_dialog = gui.Dialog.new("sim/gui/dialogs/dg101g/config/dialog", 
                                   "Aircraft/DG-101G/Dialogs/config.xml");

var dragger_list = gui.Dialog.new("sim/gui/dialogs/dg101g/dragger/dialog", 
                                  "Aircraft/DG-101G/Dialogs/draggerlist.xml");

var robot_dialog = gui.Dialog.new("sim/gui/dialogs/dg101g/robot/dialog", 
                                  "Aircraft/DG-101G/Dialogs/robot.xml");

var winch_dialog = gui.Dialog.new("sim/gui/dialogs/dg101g/winch/dialog", 
                                  "Aircraft/DG-101G/Dialogs/winch.xml");


# ############################################################################################
# config dialog: helper function to display masses in SI units rather than imperial units
var guiUpdateConfig = func {
    # pilot's mass 
    if ( getprop("/fdm/jsbsim/inertia/pointmass-weight-lbs") == nil ) {
        var mass_pilot_kg = 80;
    }
    else {
        var mass_pilot_kg = getprop("/fdm/jsbsim/inertia/pointmass-weight-lbs") * 0.45359237;
    }
    setprop("sim/glider/gui/config/mass_pilot_kg", mass_pilot_kg);
    
    # water tank masses 
    if ( getprop("/fdm/jsbsim/inertia/pointmass-weight-lbs[1]") == nil ) {
        var mass_tank_kg = 0;
    }
    else {
        var mass_tank_kg = getprop("/fdm/jsbsim/inertia/pointmass-weight-lbs[1]") * 0.45359237;
        # cleanup a bug as the gui slider does only update one property, not two
        setprop("/fdm/jsbsim/inertia/pointmass-weight-lbs[2]", 
                getprop("/fdm/jsbsim/inertia/pointmass-weight-lbs[1]") );
    }
    setprop("sim/glider/gui/config/mass_tank_kg", mass_tank_kg);
    
    # payload mass 
    if ( getprop("/fdm/jsbsim/inertia/pointmass-weight-lbs[3]") == nil ) {
        var mass_payload_kg = 0;
    }
    else {
        var mass_payload_kg = getprop("/fdm/jsbsim/inertia/pointmass-weight-lbs[3]") * 0.45359237;
    }
    setprop("sim/glider/gui/config/mass_payload_kg", mass_payload_kg);
}

var guiconfiginit = setlistener("sim/signals/fdm-initialized", 
                                 guiUpdateConfig,,0);
var guiconfig1    = setlistener("/fdm/jsbsim/inertia/pointmass-weight-lbs", 
                                 guiUpdateConfig,,0);
var guiconfig2    = setlistener("/fdm/jsbsim/inertia/pointmass-weight-lbs[1]", 
                                 guiUpdateConfig,,0);
var guiconfig3    = setlistener("/fdm/jsbsim/inertia/pointmass-weight-lbs[3]", 
                                 guiUpdateConfig,,0);


# ############################################################################################
# winch dialog: helper function to display forces in SI units rather than imperial units
var guiUpdateWinchForces = func {
    # max. pull force 
    if ( getprop("sim/glider/winch/conf/pull_max_lbs") == nil ) {
        var pull_max_daN = 600 * 0.45359237;
    }
    else {
        var pull_max_daN = getprop("sim/glider/winch/conf/pull_max_lbs") * 0.45359237;
    }
    setprop("sim/glider/gui/winch/pull_max_daN", pull_max_daN);
}

var guiwinchforceinit = setlistener("sim/sginals/fdm-initialized", 
                                     guiUpdateWinchForces,,0);
var guiwinchforce     = setlistener("sim/glider/winch/conf/pull_max_lbs", 
                                     guiUpdateWinchForces,,0);


# ############################################################################################
# winch dialog: helper function to display winch speed in km/h rather than in m/s
var guiUpdateWinchSpeed = func {
    # max. winch speed 
    if ( getprop("sim/glider/winch/conf/pull_max_speed_mps") == nil ) {
        var pull_max_speed_kmh = 40 * 3.6;
    }
    else {
        var pull_max_speed_kmh = getprop("sim/glider/winch/conf/pull_max_speed_mps") * 3.6;
    }
    setprop("sim/glider/gui/winch/pull_max_speed_kmh", pull_max_speed_kmh);
}

var guiwinchspeedinit = setlistener("sim/sginals/fdm-initialized", 
                                     guiUpdateWinchSpeed,,0);
var guiwinchspeed     = setlistener("sim/glider/winch/conf/pull_max_speed_mps", 
                                     guiUpdateWinchSpeed,,0);


# ############################################################################################
# winch dialog: helper function to display winch operation points
var guiUpdateWinchParam = func {
    if ( getprop("sim/glider/winch/conf/pull_max_lbs") == nil ) {
        var pull_max_lbs = 1100;
    }
    else {
        var pull_max_lbs = getprop("sim/glider/winch/conf/pull_max_lbs");
    }
    if ( getprop("sim/glider/winch/conf/pull_max_speed_mps") == nil ) {
        var pull_max_speed_mps = 40;
    }
    else {
        var pull_max_speed_mps = getprop("sim/glider/winch/conf/pull_max_speed_mps");
    }
    
  # for the speed correction, relative factors are used: 
    #  k_speed_x1 = speed_x1 / pull_max_speed_mps
    #  or: speed_x1 = k_speed_x1 * pull_max_speed_mps
    if ( getprop("sim/glider/winch/conf/k_speed_x1") == nil ) {
        var k_speed_x1 = 0.85;
    }
    else {
        var k_speed_x1 = getprop("sim/glider/winch/conf/k_speed_x1");
    }
    var speed_x1 = k_speed_x1 * pull_max_speed_mps * 3.6;
    setprop("sim/glider/gui/winch/speed_x1", speed_x1);
    
    #  k_speed_y2 = speed_y2 / pull_max_lbs
    #  or: speed_y2 = k_speed_y2 * pull_max_lbs
    if ( getprop("sim/glider/winch/conf/k_speed_y2") == nil ) {
        var k_speed_y2 = 0.00;
    }
    else {
        var k_speed_y2 = getprop("sim/glider/winch/conf/k_speed_y2");
    }
    var speed_y2 = k_speed_y2 * pull_max_lbs * 0.45359237;
    setprop("sim/glider/gui/winch/speed_y2", speed_y2);
    
  # for the angle correction, relative factors are used: 
    #  k_angle_x1 = angle_x1 / 70 (as 70° is hard-coded, not changeable)
    #  or: angle_x1 = k_angle_x1 * 70
    if ( getprop("sim/glider/winch/conf/k_angle_x1") == nil ) {
        var k_angle_x1 = 0.75;
    }
    else {
        var k_angle_x1 = getprop("sim/glider/winch/conf/k_angle_x1");
    }
    var angle_x1 = k_angle_x1 * 70;
    setprop("sim/glider/gui/winch/angle_x1", angle_x1);
    
    #  k_angle_y2 = angle_y2 / pull_max_lbs
    #  or: angle_y2 = k_angle_y2 * pull_max_lbs
    if ( getprop("sim/glider/winch/conf/k_angle_y2") == nil ) {
        var k_angle_y2 = 0.30;
    }
    else {
        var k_angle_y2 = getprop("sim/glider/winch/conf/k_angle_y2");
    }
    var angle_y2 = k_angle_y2 * pull_max_lbs * 0.45359237;
    setprop("sim/glider/gui/winch/angle_y2", angle_y2);
}

var guiwinchparaminit = setlistener("sim/sginals/fdm-initialized", 
                                     guiUpdateWinchParam,,0);
var guiwinchspeed_x   = setlistener("sim/glider/winch/conf/pull_max_speed_mps", 
                                     guiUpdateWinchParam,,0);
var guiwinchforce_x   = setlistener("sim/glider/winch/conf/pull_max_lbs", 
                                     guiUpdateWinchParam,,0);
var guikspeedx1       = setlistener("sim/glider/winch/conf/k_speed_x1", 
                                     guiUpdateWinchParam,,0);
var guikspeedy2       = setlistener("sim/glider/winch/conf/k_speed_y2", 
                                     guiUpdateWinchParam,,0);
var guikanglex1       = setlistener("sim/glider/winch/conf/k_angle_x1", 
                                     guiUpdateWinchParam,,0);
var guikangley2       = setlistener("sim/glider/winch/conf/k_angle_y2", 
                                     guiUpdateWinchParam,,0);
  