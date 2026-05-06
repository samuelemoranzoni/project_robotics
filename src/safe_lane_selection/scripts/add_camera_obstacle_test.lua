-- Add camera-detectable red/green cars to the currently opened Safe Lane scene.
-- This does not move, rotate, reload, or otherwise touch /rm0.
--
-- In CoppeliaSim Commander run:
-- dofile('/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project/src/safe_lane_selection/scripts/add_camera_obstacle_test.lua')

sim = require 'sim'
simIM = require 'simIM'

local SCENE_PATH = '/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project/scenes/safe_lane_selection_scene.ttt'

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

local function setColor(handle, color)
    sim.setShapeColor(handle, '', sim.colorcomponent_ambient_diffuse, color)
end

local function setShapePhysics(handle, respondable)
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, respondable and 1 or 0)
    pcall(sim.setObjectInt32Param, handle, sim.shapeintparam_respondable_mask, 65535)
    pcall(sim.setObjectInt32Param, handle, sim.objintparam_dynamic_simulation_disabled, 1)
end

local function cuboid(alias, size, position, color, yaw, respondable)
    local h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
    sim.setObjectAlias(h, alias)
    sim.setObjectPosition(h, position, sim.handle_world)
    sim.setObjectOrientation(h, {0, 0, yaw or 0}, sim.handle_world)
    setColor(h, color)
    setShapePhysics(h, respondable == true)
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
            setShapePhysics(h, false)
            pcall(sim.setObjectInt32Param, h, sim.shapeintparam_culling, 1)
        end
    end
    simIM.destroy(img)
    return markerRoot
end

local function addCar(alias, markerId, x, y, color)
    local body = cuboid(alias .. '_body', {0.72, 0.42, 0.23}, {x, y, 0.18}, color, 0, true)
    local roof = cuboid(
        alias .. '_roof',
        {0.34, 0.36, 0.16},
        {x - 0.02, y, 0.36},
        {color[1] * 0.75, color[2] * 0.75, color[3] * 0.75},
        0,
        false
    )
    sim.setObjectParent(roof, body, true)
    addArucoMarker(alias .. '_marker_' .. markerId, markerId, 0.28, 0.01, {x - 0.38, y, 0.34}, math.pi / 2)
end

if sim.getObject('/rm0', {noError = true}) < 0 then
    sim.addLog(sim.verbosity_errors, 'Cannot add camera test cars: /rm0 not found in this scene')
    return
end

removeExisting('/camera_test_obstacle_31_box')
removeExisting('/camera_test_obstacle_31_marker_31')
removeExisting('/camera_test_car_21_body')
removeExisting('/camera_test_car_21_roof')
removeExisting('/camera_test_car_21_marker_21')
removeExisting('/camera_test_car_22_body')
removeExisting('/camera_test_car_22_roof')
removeExisting('/camera_test_car_22_marker_22')

-- Two vehicle-like objects: only red/green, matching the camera detector.
addCar('camera_test_car_21', 21, 0.20, 0.65, {0.85, 0.04, 0.03})
addCar('camera_test_car_22', 22, 2.35, -0.65, {0.03, 0.72, 0.10})

sim.saveScene(SCENE_PATH)
sim.addLog(sim.verbosity_scriptinfos, 'Added red/green camera test cars; scene saved at: ' .. SCENE_PATH)
