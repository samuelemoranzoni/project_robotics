-- Run this script in CoppeliaSim from an empty scene to create the demo track.
-- Change SCENARIO_LEVEL to "easy", "medium", "incident", "random", or "all".

sim = require 'sim'
simIM = require 'simIM'

local PROJECT_ROOT = '/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project'
local BASE_WORKSPACE = '/Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster'
local SCENARIO_LEVEL = 'random'

math.randomseed(os.time())

local root = sim.createDummy(0.02)
sim.setObjectAlias(root, 'GuardianDriveRoot')
sim.setObjectPosition(root, {0, 0, 0}, sim.handle_world)

local function setColor(handle, color)
    sim.setShapeColor(handle, '', sim.colorcomponent_ambient_diffuse, color)
end

local function cuboid(alias, size, position, color, yaw)
    local h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
    sim.setObjectAlias(h, alias)
    sim.setObjectPosition(h, position, sim.handle_world)
    sim.setObjectOrientation(h, {0, 0, yaw or 0}, sim.handle_world)
    sim.setObjectParent(h, root, true)
    setColor(h, color)
    sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
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

local function addArucoMarker(alias, markerId, size, thickness, position, yaw, bodyColor)
    local markerRoot = sim.createDummy(0.01)
    sim.setObjectAlias(markerRoot, alias)
    sim.setObjectPosition(markerRoot, position, sim.handle_world)
    sim.setObjectOrientation(markerRoot, {0, 0, yaw or 0}, sim.handle_world)
    sim.setObjectParent(markerRoot, root, true)

    local body = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid,
        {size * 1.18, 0.18, size * 1.30},
        0
    )
    sim.setObjectAlias(body, alias .. '_body')
    sim.setObjectParent(body, markerRoot, false)
    sim.setObjectPosition(body, {0, -0.12, -0.02}, sim.handle_parent)
    sim.setObjectOrientation(body, {0, 0, 0}, sim.handle_parent)
    setColor(body, bodyColor or {0.75, 0.05, 0.05})
    sim.setObjectInt32Param(body, sim.shapeintparam_static, 1)

    local dictType = simIM.dict_type._4X4_50
    local dict = simIM.getMarkerDictionary(dictType)
    local pxSize = simIM.getMarkerBitSize(dictType) + 2
    local ok, img = pcall(simIM.drawMarker, dict, markerId, pxSize)
    if not ok then
        sim.addLog(sim.verbosity_errors, 'Could not draw ArUco marker id ' .. markerId)
        return markerRoot
    end
    simIM.gray2rgb(img, true)

    local cell = size / pxSize
    for i = 0, pxSize - 1 do
        for j = 0, pxSize - 1 do
            local pixel = simIM.get(img, {i, j})
            local h = sim.createPrimitiveShape(
                sim.primitiveshape_cuboid,
                {cell, thickness, cell},
                0
            )
            sim.setObjectAlias(h, alias .. '_px')
            sim.setObjectParent(h, markerRoot, false)
            sim.setObjectPosition(
                h,
                {
                    cell * (i - (pxSize - 1) / 2),
                    0.0,
                    cell * (j - (pxSize - 1) / 2),
                },
                sim.handle_parent
            )
            setColor(h, markerPixelColor(pixel))
            sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
        end
    end
    simIM.destroy(img)
    return markerRoot
end

local function tryLoadRobot()
    local candidateModels = {
        BASE_WORKSPACE .. '/src/robomaster_example/models/robomaster_ep_tof_v2.ttm',
        BASE_WORKSPACE .. '/src/robomaster_example/models/robomaster_ep_tof.ttm',
        sim.getStringParam(sim.stringparam_application_path) .. '/models/robots/mobile/RoboMasterEP.ttm',
    }
    for _, path in ipairs(candidateModels) do
        local ok, handle = pcall(sim.loadModel, path)
        if ok and handle and handle >= 0 then
            sim.setObjectAlias(handle, 'rm0')
            sim.setObjectPosition(handle, {-5.25, 0, 0.08}, sim.handle_world)
            sim.setObjectOrientation(handle, {0, 0, 0}, sim.handle_world)
            sim.addLog(sim.verbosity_scriptinfos, 'Loaded RoboMaster model: ' .. path)
            return handle
        end
    end
    sim.addLog(sim.verbosity_warnings, 'Could not load a RoboMaster model automatically.')
    return -1
end

local function buildRoad()
    cuboid('road', {12.5, 3.2, 0.04}, {0, 0, -0.02}, {0.18, 0.18, 0.18}, 0)
    cuboid('left_lane_line', {12.0, 0.08, 0.012}, {0, 0.82, 0.012}, {1, 1, 1}, 0)
    cuboid('right_lane_line', {12.0, 0.08, 0.012}, {0, -0.82, 0.012}, {1, 1, 1}, 0)

    for k = 0, 8 do
        cuboid(
            'center_dash_' .. k,
            {0.58, 0.055, 0.014},
            {-5.1 + k * 1.25, 0, 0.015},
            {1.0, 0.82, 0.05},
            0
        )
    end

    cuboid('left_soft_wall', {12.5, 0.08, 0.25}, {0, 1.75, 0.125}, {0.18, 0.25, 0.33}, 0)
    cuboid('right_soft_wall', {12.5, 0.08, 0.25}, {0, -1.75, 0.125}, {0.18, 0.25, 0.33}, 0)
    cuboid('parking_zone', {1.20, 1.25, 0.018}, {5.30, 0, 0.025}, {0.0, 0.55, 0.18}, 0)
    addArucoMarker('goal_marker_99', 99, 0.46, 0.012, {5.85, 0.0, 0.62}, math.pi / 2, {0.05, 0.45, 0.10})
    addArucoMarker('stop_marker_30', 30, 0.38, 0.012, {-0.70, -1.05, 0.58}, math.pi / 2, {0.85, 0.35, 0.05})
end

local slots = {
    easy = {
        {-2.10, 0.00, 0.58},
        {-0.95, 0.22, 0.58},
        {0.35, -0.18, 0.58},
    },
    medium = {
        {1.25, 0.56, 0.58},
        {2.20, -0.48, 0.58},
        {2.85, 0.18, 0.58},
    },
    incident = {
        {-3.25, 0.02, 0.58},
        {0.70, 0.00, 0.58},
        {3.75, -0.06, 0.58},
    },
}

local markerIds = {easy = 11, medium = 12, incident = 13}
local colors = {
    easy = {0.10, 0.45, 0.95},
    medium = {0.95, 0.62, 0.05},
    incident = {0.95, 0.05, 0.05},
}

local function addIntruder(level, slotIndex)
    local p = slots[level][slotIndex]
    local alias = 'intruder_' .. level .. '_' .. slotIndex .. '_id_' .. markerIds[level]
    addArucoMarker(alias, markerIds[level], 0.42, 0.012, p, math.pi / 2, colors[level])
    sim.addLog(
        sim.verbosity_scriptinfos,
        string.format('Scenario %s: intruder at x=%.2f y=%.2f', level, p[1], p[2])
    )
end

local function buildScenario()
    if SCENARIO_LEVEL == 'all' then
        addIntruder('easy', 1)
        addIntruder('medium', 1)
        addIntruder('incident', 2)
        return
    end
    local level = SCENARIO_LEVEL
    if level == 'random' then
        local levels = {'easy', 'medium', 'incident'}
        level = levels[math.random(#levels)]
    end
    local candidates = slots[level] or slots.easy
    addIntruder(level, math.random(#candidates))
end

buildRoad()
buildScenario()
tryLoadRobot()

sim.addLog(sim.verbosity_scriptinfos, 'Guardian driving scene generated.')
sim.addLog(sim.verbosity_scriptinfos, 'Save it manually as: ' .. PROJECT_ROOT .. '/scenes/guardian_drive_scene.ttt')

