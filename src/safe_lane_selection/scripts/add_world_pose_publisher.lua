-- Add only the /rm0/world_pose publisher to the currently opened scene.
-- This does not move, rotate, reload, or otherwise touch the RoboMaster model.
--
-- In CoppeliaSim Commander run:
-- dofile('/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project/src/safe_lane_selection/scripts/add_world_pose_publisher.lua')

sim = require 'sim'

local SCENE_PATH = '/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project/scenes/safe_lane_selection_scene.ttt'

local rmHandle = sim.getObject('/rm0', {noError = true})
if rmHandle == nil or rmHandle < 0 then
    sim.addLog(sim.verbosity_errors, 'Cannot add world pose publisher: /rm0 not found in this scene')
    return
end

local scriptText = [=[
sim = require 'sim'

function sysCall_init()
    worldPosePub = nil
    rmHandle = sim.getObject('/rm0', {noError = true})
    if rmHandle < 0 then
        sim.addLog(sim.verbosity_warnings, 'Safe lane world pose publisher: /rm0 not found')
        return
    end

    local ok, ros2 = pcall(require, 'simROS2')
    if not ok then
        sim.addLog(sim.verbosity_warnings, 'Safe lane world pose publisher: simROS2 not available')
        return
    end
    simROS2 = ros2
    worldPosePub = simROS2.createPublisher('/rm0/world_pose', 'geometry_msgs/msg/Pose2D')
end

function sysCall_sensing()
    if worldPosePub == nil or rmHandle == nil or rmHandle < 0 then
        return
    end
    local p = sim.getObjectPosition(rmHandle, sim.handle_world)
    local e = sim.getObjectOrientation(rmHandle, sim.handle_world)
    simROS2.publish(worldPosePub, {x = p[1], y = p[2], theta = e[3]})
end

function sysCall_cleanup()
    if worldPosePub ~= nil then
        simROS2.shutdownPublisher(worldPosePub)
    end
end
]=]

if sim.createScript then
    sim.createScript(sim.scripttype_simulation, scriptText)
else
    local script = sim.addScript(sim.scripttype_childscript)
    sim.setScriptText(script, scriptText)
    sim.associateScriptWithObject(script, rmHandle)
end

sim.saveScene(SCENE_PATH)
sim.addLog(sim.verbosity_scriptinfos, 'Added /rm0/world_pose publisher without touching /rm0; scene saved at: ' .. SCENE_PATH)
