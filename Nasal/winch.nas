# ############################################################################################
# ############################################################################################
# Nasal script to manage winch-launch for the DG-101G
#
# ############################################################################################
# Author: Klaus Kerner
# Version: 2011-08-04
#
# ############################################################################################
# Concepts:
# 1. check, whether the initial conditions are fullfilled:
#    - we are on ground
#    - we have no speed
#    - up to now there is no winch placed
#    and if we are fine,place a winch in front of the plane and 
#    initialize all relevant parameters
# 2. launch the winch
# 3. release the winch
#    - by reaching conditions
#    - manually
#    and remove it 
# 4. be happy and fly

# ## existing proterties, that are used to handle the winch
# /gear/gear/wow                                             indicating contact to ground
# /orientation/heading-deg                                   indicating current heading
# /orientation/pitch-deg                                     indicating current pitch
# /orientation/roll-deg                                      indicating current roll
# /sim/presets/heading-deg                                   initial heading at runway
# /sim/presets/latitude-deg                                  initial position at runway
# /sim/presets/longitude-deg                                 initial position at runway

# ## required proterties from the jsbsim config file, that are used to handle the winch
# /fdm/jsbsim/fcs/winch-cmd-norm                             created by jsbsim config file
#                                                              1: winch is engaged
#                                                              0: winch is not engaged
# /fdm/jsbsim/external_reactions/winchx/magnitude            created by jsbsim config file
# /fdm/jsbsim/external_reactions/winchy/magnitude            created by jsbsim config file
# /fdm/jsbsim/external_reactions/winchz/magnitude            created by jsbsim config file

# ## new properties, used to manage the winch
# /sim/glider/winch/conf/rope_initial_length_m               initial rope length
# /sim/glider/winch/conf/pull_max_lbs                        max. pulling force
#                                                              will be used in the future for
#                                                              a more realistic winch model
# /sim/glider/winch/glob/rope_initial_length_m               global rope length
# /sim/glider/winch/glob/pull_max_lbs                        global max. pulling force
# /sim/glider/winch/work/wp-lat-deg                          storing winch position
# /sim/glider/winch/work/wp-lon-deg                          storing winch position
# /sim/glider/winch/work/wp-alti-m                           storing winch position
# /sim/glider/winch/work/speed                               current speed of rope
#                                                              will be used in the future for
#                                                              a more realistic winch model
# /sim/glider/winch/work/rope_m                              current length of rope
# /sim/glider/winch/work/placed                              bool storing existence of winch
#                                                              1: winch placed
#                                                              0: winch not placed
# /sim/glider/winch/work/used                                bool storing allready used 
#                                                              winch to prevent against
#                                                              reconnecting
# /sim/glider/winch/flag/pull                                bool storing working of winch
#                                                              1: winch is working
#                                                              0: winch is not working



# ############################################################################################
# ############################################################################################
# global variables for this script
  var winch_timeincrement_s = 0.1;                                       # timer increment



# ############################################################################################
# ############################################################################################
# set winch parameters to global values, if not properly defined by plane setup-file
# store global values or plane-specific values to prepare for reset option
var globalsWinch = func {
  var glob_rope_initial_length_m = 800;
  var glob_pull_max_lbs = 600;
  # set initial rope length if not defined from "plane"-set.xml 
  if ( getprop("sim/glider/winch/conf/rope_initial_length_m") == nil ) {
    atc_msg("initial rope length not defined by plane");
    atc_msg(" use default setting of ", glob_rope_initial_length_m, "m");
    setprop("sim/glider/winch/conf/rope_initial_length_m", glob_rope_initial_length_m);
    setprop("sim/glider/winch/glob/rope_initial_length_m", glob_rope_initial_length_m);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/rope_initial_length_m", 
            getprop("sim/glider/winch/conf/rope_initial_length_m"));
  }
  
  # set max force for pulling, if not defined from "plane"-set.xml
  if ( getprop("sim/glider/winch/conf/pull_max_lbs") == nil ) {
    atc_msg("initial max force winch not defined by plane");
    atc_msg(" use default setting of ", glob_pull_max_lbs, "lbs");
    setprop("sim/glider/winch/conf/pull_max_lbs", glob_pull_max_lbs);
    setprop("sim/glider/winch/glob/pull_max_lbs", glob_pull_max_lbs);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/pull_max_lbs", 
            getprop("sim/glider/winch/conf/pull_max_lbs"));
  }
  
} # End Function globalsWinch



# ############################################################################################
# ############################################################################################
# reset winch parameters to global values
var resetWinch = func {
  # set rope length to global
  setprop("sim/glider/winch/conf/rope_initial_length_m", 
            getprop("sim/glider/winch/glob/rope_initial_length_m"));
  
  # set max force for pulling to global
  setprop("sim/glider/winch/conf/pull_max_lbs", 
            getprop("sim/glider/winch/glob/pull_max_lbs"));
  
} # End Function resetWinch



# ############################################################################################
# ############################################################################################
# Place winch model in correct location 
var placeWinch = func {
  # first check for an existing winch
  # if: winch exists, 
  #   print message and exit
  # else: (winch does not exist, )
  #   if plane is on ground
  #     get the position of the plane
  #     if plane is close enough to initial position on runway
  #       calculate position of winch relative to initial position on runway, 
  #     else: (plane is too far away from initial position)
  #       calculate position of winch relative to current position of plane
  #     place model
  #     set properties winch/work/placed and jsbsim/fcs/winch-cmd-norm
  #     print message
  #   else: (plane is in air)
  #     print message and exit
  
  var rope_initial_length_m = getprop("sim/glider/winch/conf/rope_initial_length_m");
  
  
  if ( getprop("sim/glider/winch/work/placed") == 1 ) {
    atc_msg("Winch allready placed"); 
  }
  else {
    if ( getprop("gear/gear/wow") ) {
      var ipos_lat_deg = getprop("sim/presets/latitude-deg");
      var ipos_lon_deg = getprop("sim/presets/longitude-deg");
      var ipos_hd_deg  = getprop("sim/presets/heading-deg");
      var ipos_alt_m = geo.elevation(ipos_lat_deg,ipos_lon_deg);
      var ipos_geo = geo.Coord.new().set_latlon( 
                      ipos_lat_deg, ipos_lon_deg, ipos_alt_m);   # get initial runway position
      
      var glider_geo = geo.aircraft_position();                   # get position of aircraft
      var glider_hd_deg  = getprop("orientation/heading-deg");   # get heading of aircraft
      
      var deviation = (glider_geo.distance_to(ipos_geo));         # offset to initial position
      if ( deviation > 200) { 
        var wp = glider_geo.apply_course_distance( glider_hd_deg , 
                   rope_initial_length_m );                       # winch position
        var wpalt = geo.elevation(wp.lat(), wp.lon());            # height at winch position
      }
      else {
        var wp = ipos_geo.apply_course_distance( ipos_hd_deg , 
                   rope_initial_length_m );                       # winch position
        var wpalt = geo.elevation(wp.lat(), wp.lon());            # height at winch position
      }
      
      setprop("sim/glider/winch/work/wp-lat-deg", wp.lat());     # stores winch position
      setprop("sim/glider/winch/work/wp-lon-deg", wp.lon());     # stores winch position
      setprop("sim/glider/winch/work/wp-alti-m", wpalt);         # stores winch position
      
      geo.put_model("/Models/Airport/supacat_winch.ac", 
                      wp.lat(), wp.lon(), wpalt, glider_hd_deg);
      setprop("sim/glider/winch/work/placed",1);                 # winch is placed
      
      atc_msg("Winch placed in front of you");
      
      setprop("sim/glider/winch/work/used",0);                   # prepare for 1 use only
    }
    else { 
      atc_msg("winch in air useless, no winch placed"); 
    }
  }

} # End Function placeWinch



# ############################################################################################
# ############################################################################################
# starts the winch
var startWinch = func {
  # first check for an existing winch
  # if the winch exists 
  #   if the winch was never used
  #     close the hook, 
  #     level the plane
  #     gets positions of plane and winch
  #     calculates distance and assign to property rope_m
  #     sets property speed
  #     set flags used and pull
  #   if used exit
  # if not exit
  
  if ( getprop("sim/glider/winch/work/placed") == 1 ) {        # check for placed winch
    if ( getprop("sim/glider/winch/work/used") == 0 ) {        # check for unused winch
      setprop("fdm/jsbsim/fcs/winch-cmd-norm",1);              # closes the hook
      atc_msg("hook closed"); 
      setprop("orientation/roll-deg",0);                       # level the plane horizontal
      atc_msg("glider leveled"); 
      atc_msg("winch starts running"); 
      var wp = geo.Coord.new().set_latlon( 
          (getprop("sim/glider/winch/work/wp-lat-deg")),
          (getprop("sim/glider/winch/work/wp-lon-deg")),
          (getprop("sim/glider/winch/work/wp-alti-m")));       # gets winch position
      var ac = geo.aircraft_position();                         # gets aircraft position
      var dd_m = (ac.direct_distance_to(wp));                   # gets distance 
      setprop("sim/glider/winch/work/rope_m", dd_m );          # set the rope length
      setprop("sim/glider/winch/work/speed",0);                # winch has speed 0
      setprop("sim/glider/winch/work/used", 1);                # one time hooked, never 
                                                                # hook again
      setprop("sim/glider/winch/flag/pull",1);                 # winch is pulling
    }
    else {
      atc_msg("Sorry, only one time hooking");
    }
  }
  else {                                                        # failure: no winch placed
    atc_msg("no winch");
  }

} # End Function startWinch



# ############################################################################################
# ############################################################################################
# release the winch
var releaseWinch = func {
  # first check for the winch is pulling
  # if yes, 
  #   opens the hook, 
  #   sets the forces to zero
  #   print a message
  # if no, 
  #   print a message and exit
  
  if ( getprop("sim/glider/winch/flag/pull") == 1 ) {           # is the winch pulling?
    setprop("fdm/jsbsim/fcs/winch-cmd-norm",0);                 # opens the hook
    setprop("fdm/jsbsim/external_reactions/winchx/magnitude", 0);  # set the forces to zero
    setprop("fdm/jsbsim/external_reactions/winchy/magnitude", 0);  # set the forces to zero
    setprop("fdm/jsbsim/external_reactions/winchz/magnitude", 0);  # set the forces to zero
    setprop("sim/glider/winch/flag/pull",0);                    # winch is not pulling
    setprop("sim/glider/winch/work/speed",0);                   # winch has speed 0
    atc_msg("Hook opened, tow released");
  }
  else {                                                         # failure: winch not working
    atc_msg("not hooked to a winch");
  }
  
  settimer(removeWinch, 5.0);                                    # remove winch completely
  atc_msg("Removing winch in 5sec");
  
} # End Function releaseWinch



# ############################################################################################
# ############################################################################################
# remove the winch
var removeWinch = func {
  # first check for the winch is still existent
  # if yes, 
  #   remove the winch from the property tree
  #   remove the model from the scenery
  # if no, 
  #   do nothing
  
  # local variables
  var modelsNode = {};
  
  if ( getprop("sim/glider/winch/work/placed") == 1 ) {         # is the winch placed?
    # remove 3d model from scenery
    # identification is /models/model[x]/path = supacat_winch.ac
    modelsNode = props.globals.getNode("models", 1).getChildren();
    foreach (var modelsmember; modelsNode)  {
      if ( (var iswinch = modelsmember.getNode("path")) != nil) {
        var modelpath = iswinch.getValue();
        if (modelpath == "/Models/Airport/supacat_winch.ac")  {
          atc_msg("Winch model found");
          modelsmember.remove();
        }
      }
    }
    # remove winch working properties
    # node is /sim/glider/winch/work
    props.globals.getNode("sim/glider/winch/work").remove();
    atc_msg("Winch removed");
    
  }
  else {                                                         # do nothing
    atc_msg("Winch does not exist");
  }

} # End Function removeWinch



# ############################################################################################
# ############################################################################################
# let the winch pull the plane up into the sky
var runWinch = func {
  # strategy, 
  # - pull with max. force
  # - pilot to control speed by aoa
  
  # if the winch is pulling
  #   get the position of the winch
  #   get the position of the glider
  #   get heading of glider
  #   get heading to winch of glider
  #   get direct distance (hypothenuse)
  #   get distance (kathete)
  #   get pitch of glider
  #   get roll of glider
  #   get the old rope-length
  #   calculate current rope-speed
  #   corredt false length
  #   calculate relevant angles for force-transformation
  #   calculate according sine and cosine values
  #   calculate the force vectors at the plane
  #   calculate the angle of resulting force vector an glider
  #   if angle is high enough
  #     release winch
  #   if angle is flat
  #     apply values to coresponding properties
  
  
  var roperelease_deg = 70;  # release winch automatically

  var pullmax = getprop("sim/glider/winch/conf/pull_max_lbs");

  if ( winch_timeincrement_s == 0 ) {
    var deltatime_s = getprop("sim/time/delta-sec");
  }
  else {
    var deltatime_s = winch_timeincrement_s;
  }
  
  if (getprop ("sim/glider/winch/flag/pull")) {         # is a winch placed and working
    var wp = geo.Coord.new().set_latlon( 
        (getprop("sim/glider/winch/work/wp-lat-deg")),
        (getprop("sim/glider/winch/work/wp-lon-deg")),
        (getprop("sim/glider/winch/work/wp-alti-m")));       # gets winch position
    var ac = geo.aircraft_position();                    # aircraft position
    var hd_deg = getprop("orientation/heading-deg");    # heading of aircraft
    var hw_deg = (ac.course_to(wp));                     # heading to winch
    var dd_m = (ac.direct_distance_to(wp));              # distance to winch, the rope length
    var dp_m = (ac.distance_to(wp));                     # projected distance to winch
    var pt_deg = getprop("orientation/pitch-deg");      # pitch of aircraft
    var rt_deg = getprop("orientation/roll-deg");       # roll of aircraft
    var dd_old_m = getprop("sim/glider/winch/work/rope_m");  # rope length from last step
    var ropespeed = (dd_m - dd_old_m)/deltatime_s;       # the speed of the rope
    
    if (dp_m > dd_m) { dp_m = dd_m;}                     # correct a failure, if the projected
                                                         # length is larger than direct length
    
    var alpha = math.acos( (dp_m / dd_m) );
    var beta = ( hw_deg - hd_deg ) * 0.01745;
    var gamma = pt_deg * 0.01745;
    var delta = rt_deg * 0.01745;
    
    var sina = math.sin(alpha);
    var cosa = math.cos(alpha);
    var sinb = math.sin(beta);
    var cosb = math.cos(beta);
    var sing = math.sin(gamma);
    var cosg = math.cos(gamma);
    var sind = math.sin(delta);
    var cosd = math.cos(delta);
    
    # global forces: alpha beta
    var fglobalx = pullmax * cosa * cosb;
    var fglobaly = pullmax * cosa * sinb;
    var fglobalz = pullmax * sina;
    # local forces by pitch: gamma
    var flpitchx = fglobalx * cosg - fglobalz * sing;
    var flpitchy = fglobaly;
    var flpitchz = fglobalx * sing + fglobalz * cosg;
    # local forces by roll: delta
    var flrollx  = flpitchx;
    var flrolly  = flpitchy * cosd + flpitchz * sind;
    var flrollz  = flpitchy * sind + flpitchz * cosd;
    # asigning to LOCAL coord of plane
    var forcex = flrollx;
    var forcey = flrolly;
    var forcez = flrollz;
    
    
    var force_deg = math.asin( math.sqrt(forcey * forcey + forcez * forcez) 
      / math.sqrt(forcex * forcex + forcey * forcey + forcez * forcez) ) 
      / 0.01745;                                    # calculates the force angle
    
    
    if (force_deg > roperelease_deg) {              # automatic release if criteria is reached
      releaseWinch(); 
    } 
    else  {                                         # set the current forces
      setprop("fdm/jsbsim/external_reactions/winchx/magnitude",  forcex);
      setprop("fdm/jsbsim/external_reactions/winchy/magnitude",  forcey);
      setprop("fdm/jsbsim/external_reactions/winchz/magnitude",  forcez);
      setprop("sim/glider/winch/work/speed", ropespeed); 
      setprop("sim/glider/winch/work/rope_m", dd_m ); 
    }
    
    settimer(runWinch, winch_timeincrement_s);
    
  }

} # End Function runWinch

var pulling = setlistener("sim/glider/winch/flag/pull", runWinch);
var initializing_winch = setlistener("sim/signals/fdm-initialized", globalsWinch);
