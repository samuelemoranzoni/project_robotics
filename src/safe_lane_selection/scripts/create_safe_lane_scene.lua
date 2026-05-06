-- Safe Lane Selection scene generator.
-- In CoppeliaSim open Developer tools > Commander > Commander and run:
-- dofile('/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project/src/safe_lane_selection/scripts/create_safe_lane_scene.lua')
--
-- The script creates and saves:
-- /Users/samuelemoranzoni/Desktop/usi/robotics/lab/project/scenes/safe_lane_selection_scene.ttt

sim = require 'sim'
simIM = require 'simIM'

local PROJECT_ROOT = '/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project'
local BASE_WORKSPACE = '/Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster'
local SCENE_PATH = PROJECT_ROOT .. '/scenes/safe_lane_selection_scene.ttt'

math.randomseed(os.time())

local function removeObjectOrModel(handle)
    if handle == nil or handle < 0 then
        return
    end
    local ok = pcall(sim.removeModel, handle)
    if not ok then
        pcall(sim.removeObject, handle)
    end
end

local function removeExisting(path)
    while true do
        local handle = sim.getObject(path, {noError = true})
        if handle == nil or handle < 0 then
            break
        end
        removeObjectOrModel(handle)
    end
end

local defaultFloor = sim.getObject('/Floor', {noError = true})
if defaultFloor >= 0 then
    sim.removeObject(defaultFloor)
end
removeExisting('/SafeLaneSelectionRoot')
removeExisting('/road_surface')

local allVisibleObjects = {}

local root = sim.createDummy(0.02)
sim.setObjectAlias(root, 'SafeLaneSelectionRoot')
sim.setObjectPosition(root, {0, 0, 0}, sim.handle_world)

local ROAD_TOP_Z = 0.04
local PAINT_Z = ROAD_TOP_Z + 0.010
local MARKER_Z = ROAD_TOP_Z + 0.014
-- The RoboMaster EP ToF model is authored with its local X axis as "up".
-- Rotate local X onto world Z so the wheels sit on the road and the camera
-- looks along the lane instead of down into the floor.
local ROBOT_ORIENTATION = {0, -math.pi / 2, 0}
local ROBOT_GROUND_CLEARANCE = 0.005

local function setColor(handle, color)
    sim.setShapeColor(handle, '', sim.colorcomponent_ambient_diffuse, color)
end

local function setShapePhysics(handle, respondable)
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, respondable and 1 or 0)
    pcall(sim.setObjectInt32Param, handle, sim.shapeintparam_respondable_mask, 65535)
end

local function cuboid(alias, size, position, color, yaw, respondable)
    local isRespondable = respondable == true
    local h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
    sim.setObjectAlias(h, alias)
    sim.setObjectPosition(h, position, sim.handle_world)
    sim.setObjectOrientation(h, {0, 0, yaw or 0}, sim.handle_world)
    -- Keep static physical colliders directly under the world. CoppeliaSim warns
    -- when a static respondable shape is parented to a non-static scene tree.
    if not isRespondable then
        sim.setObjectParent(h, root, true)
    end
    setColor(h, color)
    setShapePhysics(h, isRespondable)
    table.insert(allVisibleObjects, h)
    return h
end

local function markerPixelColor(pixel)
    local ok, data = pcall(function() return (Vector(pixel) / 255):data() end)
    if ok then
        return data
    end
    if type(pixel) == 'table' then
        return {pixel[1] / 255, pixel[2] / 255, pixel[3] / 255}
    end
    local value = pixel / 255
    return {value, value, value}
end

local function addArucoMarker(alias, markerId, size, thickness, position, yaw)
    local markerRoot = sim.createDummy(0.01)
    sim.setObjectAlias(markerRoot, alias)
    sim.setObjectPosition(markerRoot, position, sim.handle_world)
    sim.setObjectOrientation(markerRoot, {0, 0, yaw or 0}, sim.handle_world)
    sim.setObjectParent(markerRoot, root, true)

    local dict = simIM.getMarkerDictionary(simIM.dict_type._4X4_50)
    local pxSize = simIM.getMarkerBitSize(simIM.dict_type._4X4_50) + 2
    local ok, img = pcall(simIM.drawMarker, dict, markerId, pxSize)
    if not ok then
        sim.addLog(sim.verbosity_errors, 'Could not draw ArUco marker id ' .. markerId)
        return markerRoot
    end
    simIM.gray2rgb(img, true)

    local cell = size / pxSize
    for i = 0, pxSize - 1 do
        for j = 0, pxSize - 1 do
            local h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, {cell, thickness, cell}, 0)
            sim.setObjectAlias(h, alias .. '_px')
            sim.setObjectParent(h, markerRoot, false)
            sim.setObjectPosition(
                h,
                {
                    cell * (i - (pxSize - 1) / 2),
                    0,
                    cell * (j - (pxSize - 1) / 2),
                },
                sim.handle_parent
            )
            setColor(h, markerPixelColor(simIM.get(img, {i, j})))
            -- Disable dynamics entirely on marker pixels: they are decorative only.
            -- Leaving them dynamic was triggering "non-convex shapes" warnings and
            -- slowing the physics step enough to break the RoboMaster heartbeat.
            sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
            sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
            pcall(sim.setObjectInt32Param, h, sim.objintparam_dynamic_simulation_disabled, 1)
            pcall(sim.setObjectInt32Param, h, sim.objintparam_collection_self_collision_indicator, 0)
            pcall(sim.setObjectInt32Param, h, sim.shapeintparam_culling, 1)
            table.insert(allVisibleObjects, h)
        end
    end
    simIM.destroy(img)
    return markerRoot
end

local function addCar(alias, markerId, x, y, color)
    local body = cuboid(alias .. '_body', {0.72, 0.42, 0.23}, {x, y, 0.14}, color, 0)
    local roof = cuboid(alias .. '_roof', {0.34, 0.36, 0.16}, {x - 0.02, y, 0.335}, {color[1] * 0.75, color[2] * 0.75, color[3] * 0.75}, 0)
    sim.setObjectParent(roof, body, true)
    addArucoMarker(alias .. '_marker_' .. markerId, markerId, 0.28, 0.01, {x - 0.38, y, 0.31}, math.pi / 2)
    return body
end

local function addObstacle(alias, markerId, x, y)
    cuboid(alias .. '_box', {0.50, 0.50, 0.50}, {x, y, 0.25}, {0.55, 0.55, 0.55}, 0)
    addArucoMarker(alias .. '_marker_' .. markerId, markerId, 0.25, 0.01, {x - 0.27, y, 0.36}, math.pi / 2)
end

local function buildRoad()
    cuboid('road_surface', {14.5, 4.2, 0.08}, {0, 0, 0.00}, {0.16, 0.16, 0.16}, 0, true)
    cuboid('left_border_yellow', {14.2, 0.055, 0.014}, {0, 2.02, PAINT_Z}, {1.0, 0.78, 0.05}, 0)
    cuboid('right_border_yellow', {14.2, 0.055, 0.014}, {0, -2.02, PAINT_Z}, {1.0, 0.78, 0.05}, 0)

    local laneLines = {-1.30, 0.0, 1.30}
    for lineIndex, y in ipairs(laneLines) do
        for dash = 0, 10 do
            cuboid(
                'lane_' .. lineIndex .. '_dash_' .. dash,
                {0.70, 0.055, 0.016},
                {-6.25 + dash * 1.25, y, PAINT_Z},
                {1.0, 1.0, 1.0},
                0
            )
        end
    end

    -- A green finish region at the end of the test road.
    cuboid('safe_goal_zone', {0.90, 3.85, 0.018}, {6.25, 0, MARKER_Z}, {0.05, 0.55, 0.12}, 0)
end

local function buildScenario()
    -- Intermediate-step scene: no traffic cars and no obstacles.
    -- The focus is ego-lane identification and orientation relative to the lane.
    cuboid('start_zone', {0.75, 1.10, 0.02}, {-5.85, -1.55, MARKER_Z}, {0.08, 0.25, 0.65}, 0)
    cuboid('direction_arrow_tail', {1.15, 0.12, 0.024}, {-4.70, -1.55, MARKER_Z}, {0.05, 0.75, 0.20}, 0)
    cuboid('direction_arrow_head_left', {0.48, 0.12, 0.024}, {-4.10, -1.39, MARKER_Z}, {0.05, 0.75, 0.20}, math.rad(34))
    cuboid('direction_arrow_head_right', {0.48, 0.12, 0.024}, {-4.10, -1.71, MARKER_Z}, {0.05, 0.75, 0.20}, math.rad(-34))

    -- Side landmarks useful for optional localization experiments.
    addArucoMarker('localization_landmark_41', 41, 0.33, 0.01, {-3.20, 2.22, 0.58}, 0)
    addArucoMarker('localization_landmark_42', 42, 0.33, 0.01, {2.40, -2.22, 0.58}, math.pi)
end

local function modelBottomOffsetZ(handle)
    local minX = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_min_x)
    local maxX = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_max_x)
    local minY = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_min_y)
    local maxY = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_max_y)
    local minZ = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_min_z)
    local maxZ = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_max_z)
    local matrix = sim.getObjectMatrix(handle, sim.handle_world)
    local bottom = math.huge
    for _, x in ipairs({minX, maxX}) do
        for _, y in ipairs({minY, maxY}) do
            for _, z in ipairs({minZ, maxZ}) do
                local p = sim.multiplyVector(matrix, {x, y, z})
                bottom = math.min(bottom, p[3])
            end
        end
    end
    return bottom
end

local function tryLoadRobot()
    local existing = sim.getObject('/rm0', {noError = true})
    if existing ~= nil and existing >= 0 then
        sim.addLog(sim.verbosity_scriptinfos, 'Keeping existing RoboMaster /rm0 pose and orientation.')
        return existing
    end

    local candidateModels = {
        BASE_WORKSPACE .. '/src/robomaster_example/models/robomaster_ep_tof_v2.ttm',
        BASE_WORKSPACE .. '/src/robomaster_example/models/robomaster_ep_tof.ttm',
        sim.getStringParam(sim.stringparam_application_path) .. '/models/robots/mobile/RoboMasterEP.ttm',
    }
    for _, path in ipairs(candidateModels) do
        local ok, handle = pcall(sim.loadModel, path)
        if ok and handle and handle >= 0 then
            sim.setObjectAlias(handle, 'rm0')
            -- The RoboMaster model frame has local X as its vertical axis.
            -- Rotate it onto the road so the wheels, not the rear, touch down.
            sim.setObjectPosition(handle, {0, 0, 0}, sim.handle_world)
            sim.setObjectOrientation(handle, ROBOT_ORIENTATION, sim.handle_world)
            local spawnZ = ROAD_TOP_Z - modelBottomOffsetZ(handle) + ROBOT_GROUND_CLEARANCE
            sim.setObjectPosition(handle, {-5.9, -1.55, spawnZ}, sim.handle_world)
            sim.addLog(sim.verbosity_scriptinfos, 'Loaded RoboMaster model: ' .. path)
            return handle
        end
    end
    sim.addLog(sim.verbosity_warnings, 'Could not load a RoboMaster model automatically.')
    return -1
end

local function addWorldPosePublisher()
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
        sim.associateScriptWithObject(script, root)
    end
end

buildRoad()
buildScenario()
tryLoadRobot()
addWorldPosePublisher()

local camera = sim.getObject('/DefaultCamera', {noError = true})
if camera >= 0 then
    -- Keep a conservative CoppeliaSim-style perspective that looks at the origin.
    -- This avoids opening the scene with the camera pointed at the sky.
    sim.setObjectPosition(camera, {1.12, -1.90, 1.08}, sim.handle_world)
    sim.setObjectOrientation(camera, {-1.936, -0.501, 2.960}, sim.handle_world)
    sim.setObjectFloatParam(camera, sim.camerafloatparam_far_clipping, 100.0)
    sim.setObjectFloatParam(camera, sim.camerafloatparam_near_clipping, 0.01)
end

for viewIndex = 0, 7 do
    pcall(sim.cameraFitToView, viewIndex, allVisibleObjects, 0, 1.25)
end
sim.saveScene(SCENE_PATH)
sim.addLog(sim.verbosity_scriptinfos, 'Safe Lane Selection scene saved at: ' .. SCENE_PATH)
