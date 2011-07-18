# ############################################################################################
# ############################################################################################
# Nasal script to handle aerotowing, with AI-dragger
#
# ############################################################################################
# Author: Klaus Kerner
# Version: 2011-07-18
#
# ############################################################################################
# Concepts:
# 1. search for allready existing dragger in the property tree
# 2. if an existing dragger is too far away or no dragger is available create a new one
# 3. hook in to the dragger, that is close to the glider
# 4. lift up into the air
# 5. finish towing


# existing properties from ai branch, to handle the dragger (or the drag-robot)
# /ai/models/xyz[x]                                           the dragger that lifts me up
#  ./id                                                       the id of the ai-model
#  ./callsign                                                 the callsign of the dragger
#  ./position/latitude-deg                                    latitude of dragger
#  ./position/longitude-deg                                   longitude of dragger
#  ./position/altitude-ft                                     height of dragger
#  ./orientation/true-heading-deg                             heading
#  ./orientation/pitch-deg                                    pitch
#  ./orientation/roll-deg                                     roll
#  ./velocities/true-airspeed-kt                              speed
#
# ## existing properties to get glider orientation
# /orientation/heading-deg
# /orientation/pitch-deg
# /orientation/roll-deg

# ## existing proterties from jsbsim config file, that are used to handle the towing forces
# /fdm/jsbsim/fcs/dragger-cmd-norm                            created by jsbsim config file
#                                                               1: dragger engaged
#                                                               0: drager not engaged
# /fdm/jsbsim/external_reactions/dragx/magnitude              created by jsbsim config file
# /fdm/jsbsim/external_reactions/dragy/magnitude              created by jsbsim config file
# /fdm/jsbsim/external_reactions/dragz/magnitude              created by jsbsim config file

# new properties to handle the dragger
# /sim/glider/towing/conf/rope_length_m         length of rope, set by config file or default
# /sim/glider/towing/conf/nominal_towforce_lbs    nominal force at nominal distance
# /sim/glider/towing/glob/rope_length_m         length of rope, set by config file or default
# /sim/glider/towing/glob/nominal_towforce_lbs    nominal force at nominal distance

# /sim/glider/towing/dragid                  the ID of /ai/models/xyz[x]/id
# /sim/glider/towing/breaking_towforce_lbs   max. force of tow
# /sim/glider/towing/hooked                 flag to control engaged tow
#                                              1: rope hooked in
#                                              0: rope not hooked in
# /sim/glider/towing/list/candidate[x]        keeps possible draggers
# /sim/glider/towing/list/candidate[x]/type        MP=multiplayer, AI=ai-plane, DR=drag roboter
# /sim/glider/towing/list/candidate[x]/id          the id from /ai/models/xyz[x]/id
# /sim/glider/towing/list/candidate[x]/callsign    the according callsign
# /sim/glider/towing/list/candidate[x]/distance    the distance to the glider
# /sim/glider/towing/list/candidate[x]/selected    boolean for choosen candidate




# ############################################################################################
# ############################################################################################
# global variables in this module
var towing_timeincrement = 0;                        # timer increment



# ############################################################################################
# ############################################################################################
# set aerotowing parameters to global values, if not properly defined by plane setup-file
# store global values or plane-specific values to prepare for reset option
var globalsTowing = func {
  var glob_rope_length_m = 80;
  var glob_nominal_towforce_lbs = 1000;
  var glob_breaking_towforce_lbs = 9999;
  
  # set rope length if not defined from "plane"-set.xml 
  if ( getprop("/sim/glider/towing/conf/rope_length_m") == nil ) {
    atc_msg("rope length not defined by plane");
    atc_msg(" use default setting of ", glob_rope_length_m, "m");
    setprop("/sim/glider/towing/conf/rope_initial_length_m", glob_rope_length_m);
    setprop("/sim/glider/towing/glob/rope_initial_length_m", glob_rope_length_m);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("/sim/glider/towing/glob/rope_length_m", 
            getprop("/sim/glider/towing/conf/rope_length_m"));
  }
  
  # set nominal force for pulling, if not defined from "plane"-set.xml
  if ( getprop("/sim/glider/towing/conf/nominal_towforce_lbs") == nil ) {
    atc_msg("nominal tow force not defined by plane");
    atc_msg(" use default setting of ", glob_nominal_towforce_lbs, "lbs");
    setprop("/sim/glider/towing/conf/nominal_towforce_lbs", glob_nominal_towforce_lbs);
    setprop("/sim/glider/towing/glob/nominal_towforce_lbs", glob_nominal_towforce_lbs);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("/sim/glider/towing/glob/nominal_towforce_lbs", 
            getprop("/sim/glider/towing/conf/nominal_towforce_lbs"));
  }
  
  # set breaking force for pulling, if not defined from "plane"-set.xml
  if ( getprop("/sim/glider/towing/conf/breaking_towforce_lbs") == nil ) {
    atc_msg("breaking tow force not defined by plane");
    atc_msg(" use default setting of ", glob_breaking_towforce_lbs, "lbs");
    setprop("/sim/glider/towing/conf/breaking_towforce_lbs", glob_breaking_towforce_lbs);
    setprop("/sim/glider/towing/glob/breaking_towforce_lbs", glob_breaking_towforce_lbs);
  }
  else { # if defined, set global to plane specific for reset option
    setprop("/sim/glider/towing/glob/breaking_towforce_lbs", 
            getprop("/sim/glider/towing/conf/breaking_towforce_lbs"));
  }
  
} # End Function globalsWinch


# ############################################################################################
# ############################################################################################
# reset aerotowing parameters to global values
var resetTowing = func {
  # set rope length to global
  setprop("/sim/glider/towing/conf/rope_length_m", 
            getprop("/sim/glider/towing/glob/rope_length_m"));
  
  # set nominal force for pulling to global
  setprop("/sim/glider/towing/conf/nominal_towforce_lbs", 
            getprop("/sim/glider/towing/glob/nominal_towforce_lbs"));
  
  # set breaking force for pulling to global
  setprop("/sim/glider/towing/conf/breaking_towforce_lbs", 
            getprop("/sim/glider/towing/glob/breaking_towforce_lbs"));
  
} # End Function resetWinch



# ############################################################################################
# ############################################################################################
# restore position to location before towing dialog
var restorePosition = func {
  # set rope length to global
  setprop("position/latitude-deg", getprop("sim/glider/towing/list/init_lat_deg"));
  setprop("position/longitude-deg", getprop("sim/glider/towing/list/init_lon_deg"));
  setprop("position/altitude-ft", getprop("sim/glider/towing/list/init_alt_ft"));
  setprop("orientation/heading-deg", getprop("sim/glider/towing/list/init_head_deg"));
  setprop("orientation/pitch-deg", 0);
  setprop("orientation/roll-deg", 0);
} # End Function resetWinch



# ############################################################################################
# ############################################################################################
# listCandidates
var listCandidates = func {
  
  # first check for available multiplayer, ai-planes and drag roboter
  # if ai-objects are available 
  #   store them in an array
  #   get the glider position
  #   for every ai-object
  #     calculate the distance to the glider
  #     if the distance is lower than max. tow length
  #       get id
  #       get callsign
  #       print details to the console
  
  # local variables
  var aiobjects = [];                            # keeps the ai-objects from the property tree
  var candidates_id = [];                        # keeps all found candidates
  var candidates_dst_m = [];                     # keeps the distance to the glider
  var candidates_callsign = [];                  # keeps the callsigns
  var candidates_type = [];                      # keeps the type of candidate MP, AI, DR
  var dragid = 0;                                # id of dragger
  var callsign = 0;                              # callsign of dragger
  var cur = geo.Coord.new();                     # current processed ai-object
  var lat_deg = 0;                               # latitude of current processed aiobject
  var lon_deg = 0;                               # longitude of current processed aiobject
  var alt_m = 0;                                 # altitude of current processed aiobject
  var glider = geo.Coord.new();                  # coordinates of glider
  var distance_m = 0;                            # distance to ai-plane
  var counter = 0;                               # temporary counter
  var listbasis = "/sim/glider/towing/list/";    # string keeping basis of drag candidates list
  
  
  glider = geo.aircraft_position(); 
  
  # first scan for multiplayers
  aiobjects = props.globals.getNode("ai/models").getChildren("multiplayer"); 
  
  print("found MP: ", size(aiobjects));
  
  if (size(aiobjects) > 0 ) {
    foreach (var aimember; aiobjects) { 
      lat_deg = aimember.getNode("position/latitude-deg").getValue(); 
      lon_deg = aimember.getNode("position/longitude-deg").getValue(); 
      alt_m = aimember.getNode("position/altitude-ft").getValue() * FT2M; 
      cur = geo.Coord.set_latlon( lat_deg, lon_deg, alt_m );
      distance_m = (glider.distance_to(cur)); 
      
      append( candidates_id, aimember.getNode("id").getValue() );
      append( candidates_callsign, aimember.getNode("callsign").getValue() );
      append( candidates_dst_m, distance_m );
      append( candidates_type, "MP" );
    }
  }
  
  # second scan for ai-planes
  aiobjects = props.globals.getNode("ai/models").getChildren("aircraft"); 
  
  print("found AI: ", size(aiobjects));
  
  if (size(aiobjects) > 0 ) {
    foreach (var aimember; aiobjects) { 
      lat_deg = aimember.getNode("position/latitude-deg").getValue(); 
      lon_deg = aimember.getNode("position/longitude-deg").getValue(); 
      alt_m = aimember.getNode("position/altitude-ft").getValue() * FT2M; 
      cur = geo.Coord.set_latlon( lat_deg, lon_deg, alt_m );
      distance_m = (glider.distance_to(cur)); 
      
      append( candidates_id, aimember.getNode("id").getValue() );
      append( candidates_callsign, aimember.getNode("callsign").getValue() );
      append( candidates_dst_m, distance_m );
      append( candidates_type, "AI" );
    }
  }
  
  # third scan for drag roboter
  aiobjects = props.globals.getNode("ai/models").getChildren("dragger"); 
  
  print("found robot: ", size(aiobjects));
  
  if (size(aiobjects) > 0 ) {
    foreach (var aimember; aiobjects) { 
      lat_deg = aimember.getNode("position/latitude-deg").getValue(); 
      lon_deg = aimember.getNode("position/longitude-deg").getValue(); 
      alt_m = aimember.getNode("position/altitude-ft").getValue() * FT2M; 
      cur = geo.Coord.set_latlon( lat_deg, lon_deg, alt_m );
      distance_m = (glider.distance_to(cur)); 
      
      append( candidates_id, aimember.getNode("id").getValue() );
      append( candidates_callsign, aimember.getNode("callsign").getValue() );
      append( candidates_dst_m, distance_m );
      append( candidates_type, "DR" );
    }
  }
  
  # some kind of sorting, criteria is distance, 
  # but only if there are more than 1 candidate
  if (size(candidates_id) > 1) {
    # first push the closest candidate on the first position
    for (var index = 1; index < size(candidates_id); index += 1 ) {
      if ( candidates_dst_m[0] > candidates_dst_m[index] ) {
        var tmp_id = candidates_id[index];
        var tmp_cs = candidates_callsign[index];
        var tmp_dm = candidates_dst_m[index];
        var tmp_tp = candidates_type[index];
        candidates_id[index] = candidates_id[0];
        candidates_callsign[index] = candidates_callsign[0];
        candidates_dst_m[index] = candidates_dst_m[0];
        candidates_type[index] = candidates_type[0];
        candidates_id[0] = tmp_id;
        candidates_callsign[0] = tmp_cs;
        candidates_dst_m[0] = tmp_dm;
        candidates_type[0] = tmp_tp;
      }
    }
    # then sort all the remaining candidates, if there are more than 2
    if (size(candidates_id) > 2) {
      # do all other sorting
      for (var index = 2; index < size(candidates_id); index += 1) {
        # compare and change
        var bubble = index;
        while (( candidates_dst_m[bubble] < candidates_dst_m[bubble - 1] ) and (bubble >1)) {
          # exchange elements
          var tmp_id = candidates_id[bubble];
          var tmp_cs = candidates_callsign[bubble];
          var tmp_dm = candidates_dst_m[bubble];
          var tmp_tp = candidates_type[bubble];
          candidates_id[bubble] = candidates_id[bubble - 1];
          candidates_callsign[bubble] = candidates_callsign[bubble - 1];
          candidates_dst_m[bubble] = candidates_dst_m[bubble - 1];
          candidates_type[bubble] = candidates_type[bubble - 1];
          candidates_id[bubble - 1] = tmp_id;
          candidates_callsign[bubble - 1] = tmp_cs;
          candidates_dst_m[bubble - 1] = tmp_dm;
          candidates_type[bubble - 1] = tmp_tp;
          bubble = bubble - 1;
        }
      }
    }
  }
  
  # now, finally write the five closest candidates to the property tree
  # if there are less than five, fill up with empty objects
  for (var index = 0; index < 5; index += 1 ) {
    if (index >= size(candidates_id)) {
      var candidate_x_id_prop = "sim/glider/towing/list/candidate[" ~ index ~ "]/id";
      var candidate_x_cs_prop = "sim/glider/towing/list/candidate[" ~ index ~ "]/callsign";
      var candidate_x_dm_prop = "sim/glider/towing/list/candidate[" ~ index ~ "]/distance_m";
      var candidate_x_tp_prop = "sim/glider/towing/list/candidate[" ~ index ~ "]/type";
      var candidate_x_sl_prop = "sim/glider/towing/list/candidate[" ~ index ~ "]/selected";
      setprop(candidate_x_id_prop, -1);
      setprop(candidate_x_cs_prop, "undef");
      setprop(candidate_x_dm_prop, -9999);
      setprop(candidate_x_tp_prop, "XX");
      setprop(candidate_x_sl_prop, 0);
    }
    else {
      var candidate_x_id_prop = "sim/glider/towing/list/candidate[" ~ index ~ "]/id";
      var candidate_x_cs_prop = "sim/glider/towing/list/candidate[" ~ index ~ "]/callsign";
      var candidate_x_dm_prop = "sim/glider/towing/list/candidate[" ~ index ~ "]/distance_m";
      var candidate_x_tp_prop = "sim/glider/towing/list/candidate[" ~ index ~ "]/type";
      var candidate_x_sl_prop = "sim/glider/towing/list/candidate[" ~ index ~ "]/selected";
      setprop(candidate_x_id_prop, candidates_id[index]);
      setprop(candidate_x_cs_prop, candidates_callsign[index]);
      setprop(candidate_x_dm_prop, candidates_dst_m[index]);
      setprop(candidate_x_tp_prop, candidates_type[index]);
      setprop(candidate_x_sl_prop, 0);
    }
  }
  # and write the initial position of the glider to the property tree for cancel posibility
  setprop("sim/glider/towing/list/init_lat_deg", 
           getprop("position/latitude-deg"));
  setprop("sim/glider/towing/list/init_lon_deg", 
           getprop("position/longitude-deg"));
  setprop("sim/glider/towing/list/init_alt_ft", 
           getprop("position/altitude-ft"));
  setprop("sim/glider/towing/list/init_head_deg", 
           getprop("orientation/heading-deg"));

  
} # End Function listCandidates




# ############################################################################################
# ############################################################################################
# selectCandidates
var selectCandidates = func (select) {
  var candidates = [];
  var aiobjects = [];
  var initpos_geo = geo.Coord.new();
  var dragpos_geo = geo.Coord.new();
  
  # first reset all candidate selections and then set selected
  candidates = props.globals.getNode("sim/glider/towing/list").getChildren("candidate");
  foreach (var camember; candidates) { 
    camember.getNode("selected").setValue(0); 
  }
  var candidate_x_sl_prop = "sim/glider/towing/list/candidate[" ~ select ~ "]/selected";
  var candidate_x_id_prop = "sim/glider/towing/list/candidate[" ~ select ~ "]/id";
  var candidate_x_tp_prop = "sim/glider/towing/list/candidate[" ~ select ~ "]/type";
  setprop(candidate_x_sl_prop, 1);
  
  # next set properties for dragid
  setprop("sim/glider/towing/dragid", getprop(candidate_x_id_prop));
  
  # and finally place the glider a few meters behind chosen dragger
  aiobjects = props.globals.getNode("ai/models").getChildren(); 
  foreach (var aimember; aiobjects) { 
    if ( (var c = aimember.getNode("id") ) != nil ) { 
      var testprop = c.getValue();
      if ( testprop == getprop(candidate_x_id_prop)) {
        # get coordinates
        drlat = aimember.getNode("position/latitude-deg").getValue(); 
        drlon = aimember.getNode("position/longitude-deg").getValue(); 
        dralt = (aimember.getNode("position/altitude-ft").getValue()) * FT2M; 
        drhed = aimember.getNode("orientation/true-heading-deg").getValue();
      }
    }
  }
  dragpos_geo.set_latlon(drlat, drlon, dralt);
  initpos_geo.set_latlon(drlat, drlon, dralt);
  if (drhed > 180) {
    initpos_geo.apply_course_distance( (drhed - 180), 35 );
  }
  else {
    initpos_geo.apply_course_distance( (drhed + 180), 35 );
  }
  var initelevation_m = geo.elevation( initpos_geo.lat(), initpos_geo.lon() );
  setprop("position/latitude-deg", initpos_geo.lat());
  setprop("position/longitude-deg", initpos_geo.lon());
  setprop("position/altitude-ft", initelevation_m * M2FT);
  setprop("orientation/heading-deg", drhed);
  setprop("/orientation/roll-deg", 0);
  
} # End Function selectCandidates




# ############################################################################################
# ############################################################################################
# clearredoutCandidates
var clearredoutCandidates = func {
  # remove redout blackout caused by selectCandidates()
  setprop("/sim/rendering/redout/enabled", "false");
  setprop("/sim/rendering/redout/alpha",0);
  
} # End Function clearredoutCandidates




# ############################################################################################
# ############################################################################################
# removeCandidates
var removeCandidates = func {
  # and finally remove the list of candidates
  props.globals.getNode("/sim/glider/towing/list").remove();
  
} # End Function removeCandidates




# ############################################################################################
# ############################################################################################
# findDragger
var findDragger = func {
  
  # first check for available ai-planes
  # if ai-planes are available 
  #   store them in an array
  #   get the glider position
  #   for every ai-plane
  #     calculate the distance to the glider
  #     if the distance is lower than max. tow length
  #       get id
  #       get callsign
  #       if callsign = dragger
  #         send message
  #         set property dragid
  #         leave loop
  
  # local variables
  var aiobjects = [];                            # keeps the ai-planes from the property tree
  var dragid = 0;                                # id of dragger
  var callsign = 0;                              # callsign of dragger
  var cur = geo.Coord.new();                     # current processed ai-plane
  var lat_deg = 0;                               # latitude of current processed aiobject
  var lon_deg = 0;                               # longitude of current processed aiobject
  var alt_m = 0;                                 # altitude of current processed aiobject
  var glider = geo.Coord.new();                  # coordinates of glider
  var distance_m = 0;                            # distance to ai-plane
  
  
  # set rope length if not defined from settings-file
#  if ( getprop("/sim/glider/towing/conf/rope_length_m") == nil ) {
#    atc_msg("rope length not defined by plane");
#    atc_msg(" use default setting of 100m");
#    setprop("/sim/glider/towing/conf/rope_length_m", 80.0);
#  }

  var towlength_m = getprop("/sim/glider/towing/conf/rope_length_m");
  
  
  aiobjects = props.globals.getNode("ai/models").getChildren(); 
  glider = geo.aircraft_position(); 
  
  foreach (var aimember; aiobjects) { 
    if ( (var c = aimember.getNode("callsign") ) != nil ) { 
      callsign = c.getValue();
      dragid = aimember.getNode("id").getValue();
      if ( callsign == "dragger" ) {
        lat_deg = aimember.getNode("position/latitude-deg").getValue(); 
        lon_deg = aimember.getNode("position/longitude-deg").getValue(); 
        alt_m = aimember.getNode("position/altitude-ft").getValue() * FT2M; 
        
        cur = geo.Coord.set_latlon( lat_deg, lon_deg, alt_m ); 
        distance_m = (glider.distance_to(cur)); 
        
        if ( distance_m < towlength_m ) { 
          atc_msg("callsign %s with id %s nearby in %s m", callsign, dragid, distance_m);
          setprop("/sim/glider/towing/dragid", dragid); 
          break; 
        }
        else {
          atc_msg("callsign %s with id %s too far at %s m", callsign, dragid, distance_m);
        }
      }
    }
    else {
      atc_msg("no dragger found");
    }
  }
  
} # End Function findDragger




# ############################################################################################
# ############################################################################################
# hookDragger
var hookDragger = func {
  
  # if dragid > 0
  #  set property /fdm/jsbsim/fcs/dragger-cmd-norm
  #  level plane
  
  if ( getprop("/sim/glider/towing/dragid") != nil ) { 
    setprop("/fdm/jsbsim/fcs/dragger-cmd-norm", 1);                # closes the hook
    setprop("/sim/glider/towing/hooked", 1); 
    atc_msg("hook closed"); 
    setprop("/orientation/roll-deg", 0); 
    atc_msg("glider leveled"); 
  }
  else { 
    atc_msg("no dragger nearby"); 
  }
  
} # End Function hookDragger




# ############################################################################################
# ############################################################################################
# releaseDragger
var releaseDragger = func {
  
  # first check for dragger is pulling
  # if yes
  #   opens the hook
  #   sets the forces to zero
  #   print a message
  # if no
  #   print a message
  # exit
  
  if ( getprop ("/sim/glider/towing/hooked") ) {
    setprop  ("/fdm/jsbsim/fcs/dragger-cmd-norm",0);                 # opens the hook
    setprop("/fdm/jsbsim/external_reactions/dragx/magnitude", 0);    # set the forces to zero
    setprop("/fdm/jsbsim/external_reactions/dragy/magnitude", 0);    # set the forces to zero
    setprop("/fdm/jsbsim/external_reactions/dragz/magnitude", 0);    # set the forces to zero
    setprop("/sim/glider/towing/hooked",0);                         # dragger is not pulling
    atc_msg("Hook opened, tow released");
  }
  else {                                                       # failure: winch not working
    atc_msg("Hook already opened");
  }
  
} # End Function releaseDragger



# ############################################################################################
# ############################################################################################
# let the dragger pull the plane up into the sky
var runDragger = func {
  
  # strategy:
  # get current positions and orientations of glider and dragger
  # calculate the forces with respect of distance and spring-coefficient of tow
  # calculate force distribution in main axes
  # do this as long as the tow is engaged at the glider
  
  # local constants describing tow properties
  var tl0 = 0.15;                      # relative length below no forces exist
  var tf0 = 0;                         # coresponding force
  # local variables
  var forcex = 0;                      # the force in x-direction, body ref system
  var forcey = 0;                      # the force in y-direction, body ref system
  var forcez = 0;                      # the force in z-direction, body ref system
  var glider = geo.Coord.new();        # keeps the glider position
  var gliderhead = 0;                  # keeps the glider heading
  var gliderpitch = 0;                 # keeps the glider pitch
  var gliderroll = 0;                  # keeps the glider roll
  var dragger = geo.Coord.new();       # keeps the dragger position
  var drlat = 0;                       # temporary latitude of dragger
  var drlon = 0;                       # temporary longitude of dragger
  var dralt = 0;                       # temporary altitude of dragger
  var dragheadto = 0;                  # heading to dragger
  var aiobjects = [];                     # keeps the ai-planes from the property tree
  var distance = 0;                    # distance glider to dragger
  var distancepr = 0;                  # projected distance glider to dragger
  var reldistance = 0;                 # relative distance glider to dragger
  var dragid = 0;                      # id of dragger
  var planeid = 0;                     # id of current processed plane
  
  var nominaltowforce = getprop("/sim/glider/towing/conf/nominal_towforce_lbs");
  var breakingtowforce = getprop("/sim/glider/towing/conf/breaking_towforce_lbs");
  var towlength_m = getprop("/sim/glider/towing/conf/rope_length_m");
  
  # do all the stuff
  
  
  if ( getprop("/sim/glider/towing/hooked") == 1 ) {                   # is a dragger engaged
    
    glider = geo.aircraft_position();                                # current glider position
    gliderpitch = getprop("/orientation/pitch-deg");
    gliderroll = getprop("/orientation/roll-deg");
    gliderhead = getprop("/orientation/heading-deg");
    
    dragid = getprop("/sim/glider/towing/dragid");               # id of former found dragger
    
    aiobjects = props.globals.getNode("ai/models").getChildren(); 
    foreach (var aimember; aiobjects) { 
      if ( (var c = aimember.getNode("id") ) != nil ) { 
        var testprop = c.getValue();
        if ( testprop == dragid ) {
          # get coordinates
          drlat = aimember.getNode("position/latitude-deg").getValue(); 
          drlon = aimember.getNode("position/longitude-deg").getValue(); 
          dralt = (aimember.getNode("position/altitude-ft").getValue()) * FT2M; 
        }
      }
    }
    
    dragger = geo.Coord.set_latlon( drlat, drlon, dralt );         # position of current plane
    
    distance = (glider.direct_distance_to(dragger));              # distance to plane in meter
    distancepr = (glider.distance_to(dragger));
    dragheadto = (glider.course_to(dragger));
    reldistance = distance / towlength_m;
    
    if ( reldistance < tl0 ) {
      forcetow = tf0;
    }
    else {
      # calculate tow force by multiplying the nominal force with reldistance powered 4
      # this gives a smooth pulling for start and a high force when the tow is nearly
      # completely taut
      forcetow = reldistance * reldistance * reldistance * reldistance * nominaltowforce;
    }
    
    if ( forcetow < breakingtowforce ) {
      
      # correct a failure, if the projected length is larger than direct length
      if (distancepr > distance) { distancepr = distance;} 
      
      
      var alpha = math.acos( (distancepr / distance) );
      var beta = ( dragheadto - gliderhead ) * 0.01745;
      var gamma = gliderpitch * 0.01745;
      var delta = gliderroll * 0.01745;
      
      
      var sina = math.sin(alpha);
      var cosa = math.cos(alpha);
      var sinb = math.sin(beta);
      var cosb = math.cos(beta);
      var sing = math.sin(gamma);
      var cosg = math.cos(gamma);
      var sind = math.sin(delta);
      var cosd = math.cos(delta);
      
      # global forces: alpha beta
      var fglobalx = forcetow * cosa * cosb;
      var fglobaly = forcetow * cosa * sinb;
      var fglobalz = forcetow * sina;
      if ( dragger.alt() > glider.alt()) {
        fglobalz = -fglobalz;
      }
      
      
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
      
      
      
      # do all the stuff
      setprop("/fdm/jsbsim/external_reactions/dragx/magnitude",  forcex);
      setprop("/fdm/jsbsim/external_reactions/dragy/magnitude",  forcey);
      setprop("/fdm/jsbsim/external_reactions/dragz/magnitude",  forcez);
    }
    else {
      releaseDragger();
    }
  }
  
  settimer(runDragger, towing_timeincrement);
  
  
} # End Function runDragger

var dragging = setlistener("/sim/glider/towing/hooked", runDragger); 
var initializing_towing = setlistener("/sim/signals/fdm-initialized", globalsTowing);
