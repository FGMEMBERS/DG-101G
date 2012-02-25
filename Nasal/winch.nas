# ####################################################################################
# ####################################################################################
# Nasal script to manage winch-launch for JSBSIM gliders (like DG-101G)
#
# ####################################################################################
# Author: Klaus Kerner
# Version: 2011-10-31
#
# ####################################################################################
# Concepts:
# 1. check, whether the initial conditions are fullfilled:
#    - we are on ground
#    - we have no speed
#    - up to now there is no winch placed
#    and if we are fine,place a winch in front of the plane and 
#    initialize all relevant parameters
# 2. attach to and run winch
#    The winch applies a certain force on the rope contact location at the plane. This
#    force is dependent on several factors: rope speed, rope angle (glider altitude), 
#    nominal pull force (dependend on glider type), ...
#    Rope speed: The force is constant up to 85% of max rope speed. 
#                From 85% up to 100% of max. rope speed the force decreases to 0%. 
#    Rope angle: Up to 70% the force does not change. From 70% up to 100% (the 
#                automatic release angle) the force will decrease to 30%. 
# 3. release from winch
#    - by reaching conditions
#    - manually
# 4. and finally remove it 


# ###  IMPORTANT  ###  IMPORTANT  ###  IMPORTANT #####################################
# ## essential proterties from the jsbsim FDM, that are required for working winch
# /fdm/jsbsim/fcs/winch-cmd-norm                         created by jsbsim config file
#                                                          1: winch is engaged
#                                                          0: winch is not engaged
# /fdm/jsbsim/external_reactions/winchx/magnitude        created by jsbsim config file
# /fdm/jsbsim/external_reactions/winchy/magnitude        created by jsbsim config file
# /fdm/jsbsim/external_reactions/winchz/magnitude        created by jsbsim config file

# ## existing proterties, that are used to handle the winch
# /gear/gear/wow                                         indicating contact to ground
# /orientation/heading-deg                               indicating current heading
# /orientation/pitch-deg                                 indicating current pitch
# /orientation/roll-deg                                  indicating current roll
# /sim/presets/heading-deg                               initial heading at runway
# /sim/presets/latitude-deg                              initial position at runway
# /sim/presets/longitude-deg                             initial position at runway

# ## new properties, used to manage the winch
# /sim/glider/winch/conf/rope_initial_length_m           initial rope length
# /sim/glider/winch/conf/pull_max_lbs                    max. pulling force
# /sim/glider/winch/conf/pull_max_speed_mps              max. pulling speed
# /sim/glider/winch/conf/k_speed_x1                      parameter for force reduction
# /sim/glider/winch/conf/k_speed_y1                      parameter for force reduction
# /sim/glider/winch/conf/k_speed_x2                      parameter for force reduction
# /sim/glider/winch/conf/k_speed_y2                      parameter for force reduction
# /sim/glider/winch/conf/k_angle_x1                      parameter for force reduction
# /sim/glider/winch/conf/k_angle_y1                      parameter for force reduction
# /sim/glider/winch/conf/k_angle_x2                      parameter for force reduction
# /sim/glider/winch/conf/k_angle_y2                      parameter for force reduction
# /sim/glider/winch/glob/rope_initial_length_m           global rope length
# /sim/glider/winch/glob/pull_max_lbs                    global max. pulling force
# /sim/glider/winch/glob/pull_max_speed_mps              global max. pulling speed
# /sim/glider/winch/glob/k_speed_x1                      parameter for force reduction
# /sim/glider/winch/glob/k_speed_y1                      parameter for force reduction
# /sim/glider/winch/glob/k_speed_x2                      parameter for force reduction
# /sim/glider/winch/glob/k_speed_y2                      parameter for force reduction
# /sim/glider/winch/glob/k_angle_x1                      parameter for force reduction
# /sim/glider/winch/glob/k_angle_y1                      parameter for force reduction
# /sim/glider/winch/glob/k_angle_x2                      parameter for force reduction
# /sim/glider/winch/glob/k_angle_y2                      parameter for force reduction
# /sim/glider/winch/work/wp-lat-deg                      storing winch position
# /sim/glider/winch/work/wp-lon-deg                      storing winch position
# /sim/glider/winch/work/wp-alti-m                       storing winch position
# /sim/glider/winch/work/speed                           current speed of rope
# /sim/glider/winch/work/rope_m                          current length of rope
# /sim/glider/winch/work/placed                          flag for existence of winch
#                                                          1: winch placed
#                                                          0: winch not placed
# /sim/glider/winch/work/used                            bool storing allready used 
#                                                          winch to prevent against
#                                                          reconnecting
# /sim/glider/winch/flag/pull                            bool storing working of winch
#                                                          1: winch is working
#                                                          0: winch is not working



# ####################################################################################
# ####################################################################################
# global variables for this script
  var winch_timeincrement_s = 0.1;                       # timer increment



# ####################################################################################
# ####################################################################################
# set winch parameters to global values, if not properly defined by plane setup-file
# store global values or plane-specific values to prepare for reset option
var globalsWinch = func {
  var glob_rope_initial_length_m = 800;                 # default length 800m
  var glob_pull_max_lbs = 1102;                         # default force 1102lbs=500daN
  var glob_pull_max_speed_mps = 40;                     # default speed 40m/s
  var glob_k_speed_x1 = 0.85;
  var glob_k_speed_y1 = 1.00;
  var glob_k_speed_x2 = 1.00;
  var glob_k_speed_y2 = 0.00;
  var glob_k_angle_x1 = 0.75;
  var glob_k_angle_y1 = 1.00;
  var glob_k_angle_x2 = 1.00;
  var glob_k_angle_y2 = 0.30;
  # set initial rope length if not defined from "plane"-set.xml 
  if ( getprop("sim/glider/winch/conf/rope_initial_length_m") == nil ) {
    setprop("sim/glider/winch/conf/rope_initial_length_m", 
             glob_rope_initial_length_m);
    setprop("sim/glider/winch/glob/rope_initial_length_m", 
             glob_rope_initial_length_m);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/rope_initial_length_m", 
            getprop("sim/glider/winch/conf/rope_initial_length_m"));
  }
  
  # set max force for pulling, if not defined from "plane"-set.xml
  if ( getprop("sim/glider/winch/conf/pull_max_lbs") == nil ) {
    setprop("sim/glider/winch/conf/pull_max_lbs", glob_pull_max_lbs);
    setprop("sim/glider/winch/glob/pull_max_lbs", glob_pull_max_lbs);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/pull_max_lbs", 
            getprop("sim/glider/winch/conf/pull_max_lbs"));
  }
  
  # set max speed for pulling, if not defined from "plane"-set.xml
  if ( getprop("sim/glider/winch/conf/pull_max_speed_mps") == nil ) {
    setprop("sim/glider/winch/conf/pull_max_speed_mps", glob_pull_max_speed_mps);
    setprop("sim/glider/winch/glob/pull_max_speed_mps", glob_pull_max_speed_mps);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/pull_max_speed_mps", 
            getprop("sim/glider/winch/conf/pull_max_speed_mps"));
  }
  
  # set ref-poing x1 for speed correction, if not defined from "plane"-set.xml
  if ( getprop("sim/glider/winch/conf/k_speed_x1") == nil ) {
    setprop("sim/glider/winch/conf/k_speed_x1", glob_k_speed_x1);
    setprop("sim/glider/winch/glob/k_speed_x1", glob_k_speed_x1);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/k_speed_x1", 
            getprop("sim/glider/winch/conf/k_speed_x1"));
  }
  
  # set ref-poing y1 for speed correction, if not defined from "plane"-set.xml
  if ( getprop("sim/glider/winch/conf/k_speed_y1") == nil ) {
    setprop("sim/glider/winch/conf/k_speed_y1", glob_k_speed_y1);
    setprop("sim/glider/winch/glob/k_speed_y1", glob_k_speed_y1);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/k_speed_y1", 
            getprop("sim/glider/winch/conf/k_speed_y1"));
  }
  
  # set ref-poing x2 for speed correction, if not defined from "plane"-set.xml
  if ( getprop("sim/glider/winch/conf/k_speed_x2") == nil ) {
    setprop("sim/glider/winch/conf/k_speed_x2", glob_k_speed_x2);
    setprop("sim/glider/winch/glob/k_speed_x2", glob_k_speed_x2);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/k_speed_x2", 
            getprop("sim/glider/winch/conf/k_speed_x2"));
  }
  
  # set ref-poing y2 for speed correction, if not defined from "plane"-set.xml
  if ( getprop("sim/glider/winch/conf/k_speed_y2") == nil ) {
    setprop("sim/glider/winch/conf/k_speed_y2", glob_k_speed_y2);
    setprop("sim/glider/winch/glob/k_speed_y2", glob_k_speed_y2);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/k_speed_y2", 
            getprop("sim/glider/winch/conf/k_speed_y2"));
  }
  
  # set ref-poing x1 for angle correction, if not defined from "plane"-set.xml
  if ( getprop("sim/glider/winch/conf/k_angle_x1") == nil ) {
    setprop("sim/glider/winch/conf/k_angle_x1", glob_k_angle_x1);
    setprop("sim/glider/winch/glob/k_angle_x1", glob_k_angle_x1);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/k_angle_x1", 
            getprop("sim/glider/winch/conf/k_angle_x1"));
  }
  
  # set ref-poing y1 for angle correction, if not defined from "plane"-set.xml
  if ( getprop("sim/glider/winch/conf/k_angle_y1") == nil ) {
    setprop("sim/glider/winch/conf/k_angle_y1", glob_k_angle_y1);
    setprop("sim/glider/winch/glob/k_angle_y1", glob_k_angle_y1);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/k_angle_y1", 
            getprop("sim/glider/winch/conf/k_angle_y1"));
  }
  
  # set ref-poing x2 for angle correction, if not defined from "plane"-set.xml
  if ( getprop("sim/glider/winch/conf/k_angle_x2") == nil ) {
    setprop("sim/glider/winch/conf/k_angle_x2", glob_k_angle_x2);
    setprop("sim/glider/winch/glob/k_angle_x2", glob_k_angle_x2);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/k_angle_x2", 
            getprop("sim/glider/winch/conf/k_angle_x2"));
  }
  
  # set ref-poing y2 for angle correction, if not defined from "plane"-set.xml
  if ( getprop("sim/glider/winch/conf/k_angle_y2") == nil ) {
    setprop("sim/glider/winch/conf/k_angle_y2", glob_k_angle_y2);
    setprop("sim/glider/winch/glob/k_angle_y2", glob_k_angle_y2);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("sim/glider/winch/glob/k_angle_y2", 
            getprop("sim/glider/winch/conf/k_angle_y2"));
  }
  
} # End Function globalsWinch



# ####################################################################################
# ####################################################################################
# reset winch parameters to global values
var resetWinch = func {
  # set rope length to global
  setprop("sim/glider/winch/conf/rope_initial_length_m", 
            getprop("sim/glider/winch/glob/rope_initial_length_m"));
  
  # set max force for pulling to global
  setprop("sim/glider/winch/conf/pull_max_lbs", 
            getprop("sim/glider/winch/glob/pull_max_lbs"));
  
  # set max speed for pulling to global
  setprop("sim/glider/winch/conf/pull_max_speed_mps", 
            getprop("sim/glider/winch/glob/pull_max_speed_mps"));
  
  # set speed correction for pulling to global
  setprop("sim/glider/winch/conf/k_speed_x1", 
            getprop("sim/glider/winch/glob/k_speed_x1"));
  setprop("sim/glider/winch/conf/k_speed_y1", 
            getprop("sim/glider/winch/glob/k_speed_y1"));
  setprop("sim/glider/winch/conf/k_speed_x2", 
            getprop("sim/glider/winch/glob/k_speed_x2"));
  setprop("sim/glider/winch/conf/k_speed_y2", 
            getprop("sim/glider/winch/glob/k_speed_y2"));
  
  # set angle correction for pulling to global
  setprop("sim/glider/winch/conf/k_angle_x1", 
            getprop("sim/glider/winch/glob/k_angle_x1"));
  setprop("sim/glider/winch/conf/k_angle_y1", 
            getprop("sim/glider/winch/glob/k_angle_y1"));
  setprop("sim/glider/winch/conf/k_angle_x2", 
            getprop("sim/glider/winch/glob/k_angle_x2"));
  setprop("sim/glider/winch/conf/k_angle_y2", 
            getprop("sim/glider/winch/glob/k_angle_y2"));
  
} # End Function resetWinch



# ####################################################################################
# ####################################################################################
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
      # get initial runway position
      var ipos_geo = geo.Coord.new().set_latlon( 
                      ipos_lat_deg, ipos_lon_deg, ipos_alt_m); 
      # get position and heading of aircraft
      var glider_geo = geo.aircraft_position(); 
      var glider_hd_deg  = getprop("orientation/heading-deg"); 
      # offset to initial position
      var deviation = (glider_geo.distance_to(ipos_geo)); 
      # if deviation is too much, locate winch in front of glider, otherwise locate 
      # winch to end of runway
      if ( deviation > 200) { 
        var wp = glider_geo.apply_course_distance( glider_hd_deg , 
                   rope_initial_length_m ); 
        var wpalt = geo.elevation(wp.lat(), wp.lon()); 
      }
      else {
        var wp = ipos_geo.apply_course_distance( ipos_hd_deg , 
                   rope_initial_length_m ); 
        var wpalt = geo.elevation(wp.lat(), wp.lon()); 
      }
      # set winch location
      setprop("sim/glider/winch/work/wp-lat-deg", wp.lat()); 
      setprop("sim/glider/winch/work/wp-lon-deg", wp.lon()); 
      setprop("sim/glider/winch/work/wp-alti-m", wpalt); 
      # place model for winch
      geo.put_model("/Models/Airport/supacat_winch.ac", 
                      wp.lat(), wp.lon(), wpalt, glider_hd_deg);
      # set flags for placed winch and prevent against additional winches
      setprop("sim/glider/winch/work/placed",1);       # winch is placed
      setprop("sim/glider/winch/work/used",0);         # prepare for 1 use only
      
      atc_msg("Winch placed in front of you");
    }
    else { 
      atc_msg("winch in air useless, no winch placed"); 
    }
  }
} # End Function placeWinch



# ####################################################################################
# ####################################################################################
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
  
  if ( getprop("sim/glider/winch/work/placed") == 1 ) {     # check for placed winch
    if ( getprop("sim/glider/winch/work/used") == 0 ) {     # check for unused winch
      setprop("fdm/jsbsim/fcs/winch-cmd-norm",1);           # closes the hook
      atc_msg("hook closed"); 
      setprop("orientation/roll-deg",0);                    # level the plane
      atc_msg("glider leveled"); 
      atc_msg("winch starts running"); 
      var wp = geo.Coord.new().set_latlon( 
          (getprop("sim/glider/winch/work/wp-lat-deg")),
          (getprop("sim/glider/winch/work/wp-lon-deg")),
          (getprop("sim/glider/winch/work/wp-alti-m")));    # gets winch position
      var ac = geo.aircraft_position();                     # gets aircraft position
      var dd_m = (ac.direct_distance_to(wp));               # gets distance 
      setprop("sim/glider/winch/work/rope_m", dd_m );       # set the rope length
      setprop("sim/glider/winch/work/speed",0);             # winch has speed 0
      setprop("sim/glider/winch/work/used", 1);             # one time hooked, never 
                                                            # hook again
      setprop("sim/glider/winch/flag/pull",1);              # winch is pulling
    }
    else {
      atc_msg("Sorry, only one time hooking");
    }
  }
  else {                                                    # failure: no winch placed
    atc_msg("no winch");
  }

} # End Function startWinch



# ####################################################################################
# ####################################################################################
# release the winch
var releaseWinch = func {
  # first check for the winch is pulling
  # if yes, 
  #   opens the hook, 
  #   sets the forces to zero
  #   print a message
  # if no, 
  #   print a message and exit
  
  if ( getprop("sim/glider/winch/flag/pull") == 1 ) {           # is the winch pulling
    setprop("fdm/jsbsim/fcs/winch-cmd-norm",0);                 # opens the hook
    setprop("fdm/jsbsim/external_reactions/winchx/magnitude", 0);  # set the
    setprop("fdm/jsbsim/external_reactions/winchy/magnitude", 0);  # forces 
    setprop("fdm/jsbsim/external_reactions/winchz/magnitude", 0);  # to zero
    setprop("sim/glider/winch/flag/pull",0);                    # winch is not pulling
    setprop("sim/glider/winch/work/speed",0);                   # winch has speed 0
    atc_msg("Hook opened, tow released");
  }
  else {                                                        # winch not working
    atc_msg("not hooked to a winch");
  }
  
  settimer(removeWinch, 5.0);                                   # remove winch
  atc_msg("Removing winch in 5sec");
  
} # End Function releaseWinch



# ####################################################################################
# ####################################################################################
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



# ####################################################################################
# ####################################################################################
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
  var speedmax = getprop("sim/glider/winch/conf/pull_max_speed_mps");
  
  var k_speed_x1 = getprop("sim/glider/winch/conf/k_speed_x1");
  var k_speed_y1 = getprop("sim/glider/winch/conf/k_speed_y1");
  var k_speed_x2 = getprop("sim/glider/winch/conf/k_speed_x2");
  var k_speed_y2 = getprop("sim/glider/winch/conf/k_speed_y2");
  var k_angle_x1 = getprop("sim/glider/winch/conf/k_angle_x1");
  var k_angle_y1 = getprop("sim/glider/winch/conf/k_angle_y1");
  var k_angle_x2 = getprop("sim/glider/winch/conf/k_angle_x2");
  var k_angle_y2 = getprop("sim/glider/winch/conf/k_angle_y2");
  

  if ( winch_timeincrement_s == 0 ) {
    var deltatime_s = getprop("sim/time/delta-sec");
  }
  else {
    var deltatime_s = winch_timeincrement_s;
  }
  
  if (getprop ("sim/glider/winch/flag/pull")) {        # is a winch placed and working
    var wp = geo.Coord.new().set_latlon( 
        (getprop("sim/glider/winch/work/wp-lat-deg")),
        (getprop("sim/glider/winch/work/wp-lon-deg")),
        (getprop("sim/glider/winch/work/wp-alti-m"))); # gets winch position
    var ac = geo.aircraft_position();                  # aircraft position
    var hd_deg = getprop("orientation/heading-deg");   # heading of aircraft
    var hw_deg = (ac.course_to(wp));                   # heading to winch
    var dd_m = (ac.direct_distance_to(wp));            # the actual rope length
    var dp_m = (ac.distance_to(wp));                   # projected distance to winch
    var pt_deg = getprop("orientation/pitch-deg");     # pitch of aircraft
    var rt_deg = getprop("orientation/roll-deg");      # roll of aircraft
    var dd_old_m = getprop("sim/glider/winch/work/rope_m");  # last rope length
    var ropespeed = (dd_old_m - dd_m)/deltatime_s;     # the speed of the rope
    
    # correct a failure, if the projected length is larger than direct length
    if (dp_m > dd_m) { dp_m = dd_m;}                   
    
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
    
    var k_force_speed = k_speed_y1 + (k_speed_y2 - k_speed_y1)/
                                     (k_speed_x2 - k_speed_x1)*
                                     (ropespeed/speedmax - k_speed_x1);
    var k_force_angle = k_angle_y1 + (k_angle_y2 - k_angle_y1)/
                                     (k_angle_x2 - k_angle_x1)*
                                     (alpha / 0.01745 / roperelease_deg - k_angle_x1);
    if ( k_force_speed > k_speed_y1 ) { k_force_speed = k_speed_y1; }
    if ( k_force_speed < k_speed_y2 ) { k_force_speed = k_speed_y2; }
    if ( k_force_angle > k_angle_y1 ) { k_force_angle = k_angle_y1; }
    if ( k_force_angle < k_angle_y2 ) { k_force_angle = k_angle_y2; }
    
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
    
    # calculates the force angle
    var force_deg = math.asin( math.sqrt(forcey * forcey + forcez * forcez) 
      / math.sqrt(forcex * forcex + forcey * forcey + forcez * forcez) ) 
      / 0.01745; 
    # automatic release if criteria is reached
    if (force_deg > roperelease_deg) { 
      releaseWinch(); 
    } 
    # otherwise set the current forces
    else  { 
      forcex = forcex * k_force_speed * k_force_angle;
      forcey = forcey * k_force_speed * k_force_angle;
      forcez = forcez * k_force_speed * k_force_angle;
      setprop("fdm/jsbsim/external_reactions/winchx/magnitude",  forcex );
      setprop("fdm/jsbsim/external_reactions/winchy/magnitude",  forcey );
      setprop("fdm/jsbsim/external_reactions/winchz/magnitude",  forcez );
      setprop("sim/glider/winch/work/speed", ropespeed); 
      setprop("sim/glider/winch/work/rope_m", dd_m ); 
    }
    
    settimer(runWinch, winch_timeincrement_s);
    
  }

} # End Function runWinch

var pulling = setlistener("sim/glider/winch/flag/pull", runWinch);
var initializing_winch = setlistener("sim/signals/fdm-initialized", globalsWinch);
