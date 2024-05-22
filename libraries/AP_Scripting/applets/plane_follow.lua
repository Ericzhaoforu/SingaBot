--[[
 support follow "mode" in plane. Theis will actually use GUIDED mode with 
 a scripting switch to allow guided to track 

 This is a very simple implementation intended to act as a framework
 for development of a custom solution
--]]

local REFRESH_RATE = 0.1   -- in seconds, so 10Hz

local PARAM_TABLE_KEY = 83
local PARAM_TABLE_PREFIX = "FOLL_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local MODE_GUIDED = 15
local MODE_RTL = 11
local MODE_LOITER = 12

local ALT_FRAME_ABSOLUTE = 0

local follow_enabled = false

-- bind a parameter to a variable
function bind_param(name)
   return Parameter(name)
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup follow mode specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')

-- This uses the exisitng FOLL_* parameters and just adds a couple specific to this script
-- FOLL_
-- FOLL_OFS_X = bind_param('FOLL_OFS_X')
-- FOLL_OFS_Y = bind_param('FOLL_OFS_Y')
-- FOLL_OFS_Z = bind_param('FOLL_OFS_Z')
-- FOLL_POS_P = bind_param('FOLL_POS_P')

-- we need these existing FOLL_ parametrs
FOLL_ALT_TYPE = bind_param('FOLL_ALT_TYPE')

-- Add these FOLL_ parameters specifically for this script
--[[
  // @Param: FOLL_FAIL_MODE
  // @DisplayName: Plane Follow lost target mode
  // @Description: Mode to switch to if the target is lost (no signal or > FOLL_DIST_MAX). 
  // @User: Standard
--]]
FOLL_FAIL_MODE = bind_add_param('FAIL_MODE', 7, MODE_RTL)

--[[
  // @Param: FOLL_EXIT_MODE
  // @DisplayName: Plane Follow exit mode
  // @Description: Mode to switch to when follow mode is exited normally
  // @User: Standard
--]]
FOLL_EXIT_MODE = bind_add_param('EXIT_MODE', 8, MODE_LOITER)

--[[
    // @Param: FOLL_ACT_FN
    // @DisplayName: Plane Follow Scripting ActivationFunction
    // @Description: Setting an RC channel's _OPTION to this value will use it for Plane Follow enable/disable
    // @Range: 300 307
--]]
FOLL_ACT_FN = bind_add_param("ACT_FN", 9, 301)

last_follow_active_state = rc:get_aux_cached(FOLL_ACT_FN:get())

local have_target = false

--[[
   return true if we are in a state where follow can apply
--]]
local function follow_active()
   local mode = vehicle:get_mode()

   if mode == MODE_GUIDED then
      -- support precision loiter under pilot control
      if follow_enabled then
        if follow:have_target() then
            return true
        else
            gcs:send_text(MAV_SEVERITY.INFO, "Plane Follow: enabled but no target")
            return false
        end
      end
   end

   return false
end

--[[
   check for user activating follow using an RC switch set HIGH
--]]
local function follow_check()
   local active_state = rc:get_aux_cached(FOLL_ACT_FN:get())
   if (active_state ~= last_follow_active_state) then
      if( active_state == 0) then
         if follow_enabled then
            -- Follow disabled - return to EXIT mode
            vehicle:set_mode(FOLL_EXIT_MODE:get())
            follow_enabled = false
            gcs:send_text(MAV_SEVERITY.INFO, "Plane Follow: disabled")
         end
      elseif (active_state == 2) then
         -- Follow enabled - switch to guided mode
         vehicle:set_mode(MODE_GUIDED)
         -- Force Guided Mode to not loiter by setting a tiny loiter radius. Otherwise Guide Mode will try loiter around the target vehichle when it gets inside WP_LOITER_RAD
         vehicle:set_guided_radius_and_direction(2, false)
         follow_enabled = true
         gcs:send_text(MAV_SEVERITY.INFO, "Plane Follow: enabled")
      else
         -- Don't know what to do with the 3rd switch position right now.
      end
      last_follow_active_state = active_state
   end
end

-- main update function
local function update()
   follow_check()
   if not follow_active() then
    return
   end
   -- gcs:send_text(MAV_SEVERITY.INFO, "Plane Follow: Active")
   local target_sysid = follow:get_target_sysid()
   -- gcs:send_text(MAV_SEVERITY.INFO, string.format("Plane Location: target sysid %d", target_sysid))

   --[[
      get the current navigation target. Note that we must get this
      before we check if we are in a follow to prevent a race condition
      with vehicle:update_target_location()
   --]]
   local target_location -- = Location()     of the target
   local target_velocity -- = Vector3f()     -- velocity of lead vehicle
   local target_distance -- = Vector3f()     -- vector to lead vehicle
   local target_offsets -- = Vector3f()      -- vector to lead vehicle + offsets

   local vehicle_location = ahrs:get_location()

   -- just because of the methods available on AP_Follow, need to call these two methods 
   -- to get target_location, target_velocity, target distance and target 
   -- and yes target_offsets (hopefully the same value) is returned by both methods
   -- even worse - both internally call get_target_lcoation_and_Velocity, but making a single method
   -- in AP_Follow is probably a high flash cost, so we just take the runtime hit
   target_location, target_offsets = follow:get_target_location_and_velocity_ofs()
   target_distance, target_offsets, target_velocity = follow:get_target_dist_and_vel_ned()
   if target_distance == nil or target_offsets == nil or target_velocity == nil then
      if FOLL_FAIL_MODE:get() ~= nil then
         vehicle:set_mode(FOLL_FAIL_MODE:get())
      end
      follow_enabled = false
      gcs:send_text(MAV_SEVERITY.INFO, "Plane Follow: lost target")
      return
   end

   -- will delete this debug later
   -- gcs:send_text(MAV_SEVERITY.INFO, string.format("Plane Follow: xy_dist %f", xy_dist))
   -- gcs:send_text(MAV_SEVERITY.INFO, string.format("Plane Follow: target_distance:length() %f", target_distance:length()))
   --gcs:send_text(MAV_SEVERITY.INFO, string.format("Plane Location: x %f y %f z %f", target_offsets:x(), target_offsets:y(), target_offsets:z()))
   --gcs:send_text(MAV_SEVERITY.INFO, string.format("Plane Velocity: x %f y %f z %f", target_velocity:x(), target_velocity:y(), target_velocity:z()))
   --gcs:send_text(MAV_SEVERITY.INFO, "Plane Follow: --------- ")
   --gcs:send_text(MAV_SEVERITY.INFO, string.format("Plane Follow: target velocity %f", target_airspeed))
   -- gcs:send_text(MAV_SEVERITY.INFO, string.format("Plane Follow: vehicle airspeed %f %f", vehicle_airspeed, xy_dist - vehicle_airspeed * REFRESH_RATE * 2))
   if target_location ~= nil and target_velocity ~= nil and target_offsets ~= nil then
      local xy_dist = target_offsets:length()
      local vehicle_airspeed = ahrs:airspeed_estimate()
      local target_airspeed = target_velocity:length()

      -- first we tell the vehicle where we want to fly to. Guided mode figures how how to fly there.
      -- my version
      --local followok = vehicle:set_target_location(target_location)
      -- plane_guided_follow version
      local current_target = vehicle:get_target_location()
      if not current_target then
         return
      end

      -- set the target frame as per user set parameter
      local altitude_frame = FOLL_ALT_TYPE:get()
      if altitude_frame == nil then
         altitude_frame = 0
      end
      target_location:change_alt_frame(altitude_frame)
      -- update the target position from the follow library, which includes the offsets
      vehicle:update_target_location(current_target, target_location)
      
      -- if the current velocity will not catch up to the target then we need to speed up use 12 seconds as a reasonable time to try to catch up
      -- first figure out how far away we will be from the required location in 12 seconds if we maintain the current vehicle and target airspeeds
      -- There is nothing magic about 12, it is just "what works" 
      local projected_distance = xy_dist - vehicle_airspeed * 12 + target_airspeed * 12
      -- gcs:send_text(MAV_SEVERITY.INFO, string.format("Plane Follow: projected distance %f", projected_distance))
   
      -- now calculate what airspeed we will need to fly for that 12 seconds to catch up the projected distance
      local desired_airspeed = projected_distance / 12
      -- gcs:send_text(MAV_SEVERITY.INFO, string.format("Plane Follow: desired velocity %f", desired_airspeed))
      vehicle:set_desired_speed(desired_airspeed)
      -- the desired airspeed will never be acheived, don't worry -we will do this calculation again in REFRESH_RATE seconds, so we can adjust
   end
end

gcs:send_text(MAV_SEVERITY.INFO, "Plane Follow: Loaded")

-- wrapper around update(). This calls update() at 20Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
local function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 1000 * REFRESH_RATE
end

-- start running update loop
return protected_wrapper()

