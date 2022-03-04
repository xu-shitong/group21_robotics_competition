package.path= package.path..";/Users/xushitong/code_log/robotics/?.lua"
require("key_point_localization");

-- NOTE: This code is NOT originated from our team, but from the following reference
-- Reference: http://rosettacode.org/wiki/Fast_Fourier_transform#Lua
-- operations on complex number
complex = {__mt={} }
 
function complex.new (r, i) 
  local new={r=r, i=i or 0} 
  setmetatable(new,complex.__mt)
  return new
end
 
function complex.__mt.__add (c1, c2)
  return complex.new(c1.r + c2.r, c1.i + c2.i)
end
 
function complex.__mt.__sub (c1, c2)
  return complex.new(c1.r - c2.r, c1.i - c2.i)
end
 
function complex.__mt.__mul (c1, c2)
  return complex.new(c1.r*c2.r - c1.i*c2.i,
                      c1.r*c2.i + c1.i*c2.r)
end
 
function complex.expi (i)
  return complex.new(math.cos(i),math.sin(i))
end
 
function complex.__mt.__tostring(c)
  return "("..c.r..","..c.i..")"
end

-- Cooley–Tukey FFT (in-place, divide-and-conquer)
-- Higher memory requirements and redundancy although more intuitive
function fft(vect)
  local n=#vect
  if n<=1 then return vect end
  -- divide  
  local odd,even={},{}
  for i=1,n,2 do
    odd[#odd+1]=vect[i]
    even[#even+1]=vect[i+1]
  end
  -- conquer
  fft(even);
  fft(odd);
  -- combine
  for k=1,n/2 do
    local t=even[k] * complex.expi(-2*math.pi*(k-1)/n)
    vect[k] = odd[k] + t;
    vect[k+n/2] = odd[k] - t;
  end
  return vect
end
 
function toComplex(vectr)
  vect={}
  for i,r in ipairs(vectr) do
    vect[i]=complex.new(r)
  end
  return vect
end

-- ========= end of reference code =========

-- Move robot to a location (only for use in random setup, not from your code!)
function setRobotPose(handle, x, y, theta)
  allModelObjects = sim.getObjectsInTree(handle) -- get all objects in the model
  sim.setThreadAutomaticSwitch(false)
  for i=1,#allModelObjects,1 do
      sim.resetDynamicObject(allModelObjects[i]) -- reset all objects in the model
  end
  pos = sim.getObjectPosition(handle, -1)
  sim.setObjectPosition(handle, -1, {x, y, pos[3]})
  sim.setObjectOrientation(handle, -1, {0, 0, theta})
  sim.setThreadAutomaticSwitch(true)
end

function createRandomBumpyFloor()
  print ("Generating new random bumpy floor.")
  sim.setThreadAutomaticSwitch(false)

  -- Remove existing bumpy floor if there already is one
  if (heightField ~= nil) then
      sim.setObjectPosition(heightField, heightField, {0.05, 0, 0})
      return
  end
  --  Create random bumpy floor for robot to drive on
  floorSize = 5
  --heightFieldResolution = 0.3
  --heightFieldNoise = 0.00000005
  heightFieldResolution = 0.1
  heightFieldNoise = 0.0000008
  cellsPerSide = floorSize / heightFieldResolution
  cellHeights = {}
  for i=1,cellsPerSide*cellsPerSide,1 do
      table.insert(cellHeights, gaussian(0, heightFieldNoise))
  end
  heightField=sim.createHeightfieldShape(0, 0, cellsPerSide, cellsPerSide, floorSize, cellHeights)
  -- Make the floor invisible
  sim.setObjectInt32Param(heightField,10,0)
  sim.setThreadAutomaticSwitch(true)
end

function get_walls()
  -- Disable error reporting
  local savedState=sim.getInt32Param(sim.intparam_error_report_mode)
  sim.setInt32Param(sim.intparam_error_report_mode,0)
  local N = 1
  while true do
      local handle = sim.getObjectHandle("Wall"..tostring(N))
      if handle <= 0 then
          break
      end

      -- Read position and shape of wall
      -- Assume here that it is thin and oriented either along the x axis or y axis

      -- We can now get the propertries of these walls, e.g....
      local pos = sim.getObjectPosition(handle, -1)
      local res,minx = sim.getObjectFloatParameter(handle,15)
      local res,maxx = sim.getObjectFloatParameter(handle,18)
      local res,miny = sim.getObjectFloatParameter(handle,16)
      local res,maxy = sim.getObjectFloatParameter(handle,19)
  
      --print("Position of Wall " .. tostring(N) .. ": " .. tostring(pos[1]) .. "," .. tostring(pos[2]) .. "," .. tostring(pos[3]))
      --print("minmax", minx, maxx, miny, maxy)

      local Ax, Ay, Bx, By
      if (maxx - minx > maxy - miny) then
          print("Wall " ..tostring(N).. " along x axis")
          Ax = pos[1] + minx
          Ay = pos[2]
          Bx = pos[1] + maxx
          By = pos[2]
      else
          print("Wall " ..tostring(N).. " along y axis")
          Ax = pos[1]
          Ay = pos[2] + miny
          Bx = pos[1]
          By = pos[2] + maxy
      end
      print (Ax, Ay, Bx, By)

      walls[N] = {Ax, Ay, Bx, By}
      N = N + 1
  end
  -- enable error reporting
  sim.setInt32Param(sim.intparam_error_report_mode,savedState)

  return N - 1
end

function get_goals()
  -- Disable error reporting
  local savedState=sim.getInt32Param(sim.intparam_error_report_mode)
  sim.setInt32Param(sim.intparam_error_report_mode,0)
  local N = 1
  while true do
      local handle = sim.getObjectHandle("Goal"..tostring(N))
      if handle <= 0 then
          break
      end

      -- Read position of goal
      local pos = sim.getObjectPosition(handle, -1)
  
      print("Position of Goal " .. tostring(N) .. ": " .. tostring(pos[1]) .. "," .. tostring(pos[2]) .. "," .. tostring(pos[3]))

      goals[N] = {pos[1], pos[2]}
      N = N + 1
  end
  -- enable error reporting
  sim.setInt32Param(sim.intparam_error_report_mode,savedState)

  return N - 1
end

-- Robot should call this function when it thinks it has reached goal N
-- Second argument is the robot's handle
function reachedGoal(N, handle)        
  local pos = sim.getObjectPosition(handle, -1)
  local xerr = pos[1] - goals[N][1]
  local yerr = pos[2] - goals[N][2]
  local err = math.sqrt(xerr^2 + yerr^2)
  local localpts = 0
  if (err < 0.05) then
      localpts = 3
  elseif (err < 0.1) then
      localpts = 2
  elseif (err < 0.2) then
      localpts = 1
  end
  
  -- if we're not at final goal (which is where we started)
  if (localpts > 0 and goalsReached[N] == false) then
      goalsReached[N] = true
      totalPoints = totalPoints + localpts
      print ("Reached Goal" ..tostring(N).. " with error " ..tostring(err).. ": Points: " ..tostring(localpts))
  end

  -- at final goal: have we reached all goals?
  if (N == startGoal and localpts > 0) then
      local allGoalsReached = true
      for i=1,N_GOALS do
          if (goalsReached[i] == false) then
              allGoalsReached = false
          end
      end
      -- Yes... all goals achieved so calculate time
      if (allGoalsReached == true) then
          tt = sim.getSimulationTime() 
          timeTaken = tt - startTime
          timePoints = 0
          if (timeTaken < 60) then
              timePoints = 5
          elseif (timeTaken < 90) then
              timePoints = 4
          elseif (timeTaken < 120) then
              timePoints = 3
          elseif (timeTaken < 180) then
              timePoints = 2
          elseif (timeTaken < 240) then
              timePoints = 1
          end
          totalPoints = totalPoints + timePoints
          print ("FINISH at time" ..tostring(timeTaken).. " with total points " ..tostring(totalPoints))

          sim.pauseSimulation()
      end
  end

end

-- ========== functions from practical 5 ==========

function getMaxMotorAngleFromTarget(posL, posR)

  -- How far are the left and right motors from their targets? Find the maximum
  maxAngle = 0
  if (speedBaseL > 0) then
      remaining = motorAngleTargetL - posL
      if (remaining > maxAngle) then
          maxAngle = remaining
      end
  end
  if (speedBaseL < 0) then
      remaining = posL - motorAngleTargetL
      if (remaining > maxAngle) then
          maxAngle = remaining
      end
  end
  if (speedBaseR > 0) then
      remaining = motorAngleTargetR - posR
      if (remaining > maxAngle) then
          maxAngle = remaining
      end
  end
  if (speedBaseR < 0) then
      remaining = posR - motorAngleTargetR
      if (remaining > maxAngle) then
          maxAngle = remaining
      end
  end

  return maxAngle
end


-- get wall distance
function dot(x1, y1, x2, y2)
  return x1 * x2 + y1 * y2
end
function getDistance(x, y, theta)
  minDistance = 100 -- dummy big distance in meter
  closestWall = {0, 0, 0, 0}
  for i, corners in ipairs(walls) do 
    Ax, Ay, Bx, By = corners[1], corners[2], corners[3], corners[4]

    -- implementation 2: use dot product
    d = ((By - Ay) * (Ax - x) - (Bx - Ax) * (Ay - y)) / ((By - Ay) * math.cos(theta) - (Bx - Ax) * math.sin(theta))

    -- intersection point
    interX = x + d * math.cos(theta)
    interY = y + d * math.sin(theta)

    if dot((Ax - interX), (Ay - interY), (Ax - Bx), (Ay - By)) > 0 and 
       dot((Bx - interX), (By - interY), (Bx - Ax), (By - Ay)) > 0 and
       d > 0 then
      minDistance = math.min(d, minDistance)
      closestWall = corners
    end
  end
  if minDistance == 100 then 
    return 0
  end
  return minDistance
end

function normalize()
  s = 0
  for i=1,#weightArray do 
    s = s + weightArray[i]
  end
  for i=1,#weightArray do 
    weightArray[i] = weightArray[i] / s
  end
end

function resampling()
  -- construct sampling cdf
  samplingCDF = {}
  boundary = 0
  for i=1,#weightArray do
    boundary = boundary + weightArray[i]
    samplingCDF[i] = boundary
  end

  -- sampling
  xArrayCopy = {}
  yArrayCopy = {}
  thetaArrayCopy = {}
  for i=1,#weightArray do
    r = math.random()
    for j=1,#samplingCDF do 
      if r < samplingCDF[j] then
        xArrayCopy[i] = xArray[j]
        yArrayCopy[i] = yArray[j]
        thetaArrayCopy[i] = thetaArray[j]
        break
      end
    end
  end

  -- copy back x y theta array
  xArray = xArrayCopy
  yArray = yArrayCopy
  thetaArray = thetaArrayCopy  
  
end

function weightedMean(xs, weight)
  sum = 0
  for i=1, #xs do
      sum = sum + xs[i] * weight[i]
  end
  return sum
end

-- ========== new self defined functions ==========

function updatePredictedPositions()
  operation = stepList[stepCounter - 2][1]
  for i = 1, dummy_N do 
    if operation == "forward" then
      -- predict forward
      xArray[i] = xArray[i] + (D + gaussian(0, e_var * D)) * math.cos(thetaArray[i])
      yArray[i] = yArray[i] + (D + gaussian(0, e_var * D)) * math.sin(thetaArray[i])
      -- use accumulated theta, since if angle crosses theta = pi position, average give opposite direction
      thetaArray[i] = thetaArray[i] + gaussian(0, f_var * D)
  
    elseif operation == "turn" then
      -- predict rotation
      thetaArray[i] = thetaArray[i] + alpha + gaussian(0, math.abs(g_var / (math.pi / 2) * alpha))

    else
      print("Warning: unknown minor operation: ", operation, ", skip updating dummies")
    end

    -- update weight of each prediction using meansurement
    weightArray[i] = calculateLikelihood(xArray[i], yArray[i], thetaArray[i])

    sim.setObjectPosition(dummyArray[i], -1, {xArray[i],yArray[i],0})
    sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]})
  end

  -- -- normalize weight
  -- normalize()

  -- softmax layer, further split difference between loss
  softmax()

  -- resampling 
  resampling()
  for i = 1, dummy_N do 
    weightArray[i] = 1 / dummy_N
    sim.setObjectPosition(dummyArray[i], -1, {xArray[i],yArray[i],0})
    sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]})
  end
end

function softmax()
  s = 0
  for i=1,#weightArray do 
    s = s + math.exp(weightArray[i])
  end
  for i=1,#weightArray do 
    weightArray[i] = math.exp(weightArray[i]) / s
  end
end



-- cast theta in range [-pi, pi]
function castTheta(theta)
  theta = math.fmod(theta, 2 * math.pi)
  if theta > math.pi then
    return theta - 2 * math.pi
  elseif theta < -math.pi then
    return theta + 2 * math.pi
  end
  return theta
end

-- get optimum reading at x, y position
function estimateRead(x, y, startAngle)
  local readings = {}
  for i=0,dir_N-1 do
    local turret_angle = i * 2 * math.pi / dir_N + startAngle
    m = getDistance(x, y, turret_angle)
    -- readings
    readings[i+1] = m
  end

  return readings
end

-- estimate mse using fft from 2 input signal
function fftMSELoss(fft1, fft2)

  local amplitude1, amplitude2, mse = 0, 0, 0
  for i=1,#fft1 do
    amplitude1 = math.sqrt(fft1[i].r * fft1[i].r + fft1[i].i * fft1[i].i)
    amplitude2 = math.sqrt(fft2[i].r * fft2[i].r + fft2[i].i * fft2[i].i)

    -- mse
    mse = mse + MSEComponent(amplitude1, amplitude2)
  end
  return mse / #fft1
end

-- -- get theta prediction from estimate reading and actual readings
-- -- Implementation 1: elementwise mse
-- function getPhaseShift(estimate, actual)
--   local optim_mse = 100000
--   local optim_angle = 0
--   for i=1,dir_N do
--     local mse = 0
--     for j=1,#estimate-1 do
--       mse = mse + math.pow(estimate[math.mod(i + j, #estimate) + 1] - actual[j], 2)
--     end
--     if (mse / #estimate < optim_mse) then
--       optim_mse = mse / #estimate
--       optim_angle = i
--     end
--   end
--   return castTheta(2 * math.pi * optim_angle / dir_N)
-- end 

-- -- -- Implementation 2: BRIEF key point localization
-- -- function getPhaseShift(estimate, actual)
-- --   return BRIEF(estimate, actual)
-- -- end

-- Implementation 3: elementwise mse + refine scale
function getPhaseShift(estimate, actual)
  return mseRefinedShift(estimate, actual)
end

-- calculate one dummy likelihood
-- -- Implementation 1:
-- function calculateLikelihood(x, y, theta)
--   -- read optim 
--   local estimates = estimateRead(x, y, theta)
--   local eps = 0.000001 -- for computation integrity

--   local angle = getPhaseShift(estimates, readings)
--   local estimates_fft = fft(toComplex(copylist(estimates)))

--   local loss = fftMSELoss(estimates_fft, readings_fft) + eps
--   return 1 / (loss * angle)
-- end

-- -- Implementation2: 
-- function calculateLikelihood(x, y, theta)
--   -- read optim 
--   local estimates = estimateRead(x, y, theta)
--   local eps = 0.000001 -- for computation integrity

--   return likelihoodComponent(estimates[1], readings[1])
-- end

-- Implementation3: get highest likelihood for each shift
function calculateLikelihood(x, y, theta)
  -- read optim 
  local estimates = estimateRead(x, y, theta)
  local eps = 0.000001 -- for computation integrity

  -- TODO: use multiple direction likelihood
  local locationLikelihood = 0
  
  for i=1,dir_N do
    locationLikelihood = locationLikelihood + likelihoodComponent(estimates[i], readings[i])
  end
  return locationLikelihood + getPhaseShift(estimates, readings)
end

-- construct minor step list from waypoints
function constructMinorStep()
  local nextIndex = 0
  if (generalStepCnt + 1 >= #generalStepList) then nextIndex = 1 else nextIndex = generalStepCnt + 1 end
  local currGoal, nextGoal = generalStepList[generalStepCnt], generalStepList[nextIndex]
  waypoints = waypoint_map[currGoal][nextGoal]

  stepList = {}
  for i=1,#waypoints do 
    baseIndex = 7 * (i - 1) + 1
    stepList[baseIndex] = {"set_waypoint", waypoints[i][1], waypoints[i][2]}
    stepList[baseIndex + 1] = {"turn"}
    stepList[baseIndex + 2] = {"stop"}
    stepList[baseIndex + 3] = {"localize"}
    stepList[baseIndex + 4] = {"forward"}
    stepList[baseIndex + 5] = {"stop"}
    stepList[baseIndex + 6] = {"localize"}
  end
  stepList[#stepList + 1] = {"set_waypoint", waypoints[#waypoints][1], waypoints[#waypoints][2]}
  stepList[#stepList + 1] = {"turn"}
  stepList[#stepList + 1] = {"stop"}
  stepList[#stepList + 1] = {"localize"}
  stepList[#stepList + 1] = {"forward"}
  stepList[#stepList + 1] = {"stop"}
  stepList[#stepList + 1] = {"localize"}

  stepList[#stepList + 1] = {"end"}
end

-- clear sensor read, prepare for a new reading
function clearSensorRead()
  dir_index = 0
  clearList(readings, dir_N)
  sim.setJointTargetPosition(turretMotor, 0)
end
-- helperfunction for extracting reading, return true if in this iteration the reading finish
function sensorRead()

  local noisyDistance = 5
  local result,cleanDistance=sim.readProximitySensor(turretSensor)
  if (result>0) then
      noisyDistance= cleanDistance + gaussian(0.0, sensorVariance)
      --print ("Depth sensor reading ", noisyDistance)
  end

  -- signal 
  if dir_index > 0 then readings[dir_index] = noisyDistance end -- remove first reading, which is always 0
  
  if (dir_index >= dir_N) then
    -- finish reading
    readings_fft = fft(toComplex(copylist(readings)))
    return true

  else 
    -- set sensor angle for the next meansurement
    local turret_angle = dir_index * 2 * math.pi / dir_N
    sim.setJointTargetPosition(turretMotor, turret_angle)
    dir_index = dir_index + 1
    return false
  end
end

-- ========= end self define functions =========


-- This function is executed exactly once when the scene is initialised
function sysCall_init()

  startTime = sim.getSimulationTime()
  print("Start Time", startTime)
        
  robotBase=sim.getObjectHandle(sim.handle_self) -- robot handle
  leftMotor=sim.getObjectHandle("leftMotor") -- Handle of the left motor
  rightMotor=sim.getObjectHandle("rightMotor") -- Handle of the right motor
  turretMotor=sim.getObjectHandle("turretMotor") -- Handle of the turret motor
  turretSensor=sim.getObjectHandle("turretSensor")
   
  -- Please use noisyDistance= cleanDistance + gaussian(0.0, sensorVariance) for all sonar sensor measurements
  sensorStandardDeviation = 0.1
  sensorVariance = sensorStandardDeviation^2

  -- Create bumpy floor for robot to drive on
  createRandomBumpyFloor()

  -- Data structure for walls (your program can use this)
  walls = {}
  -- Fill it by parsing the scene in the GUI
  N_WALLS = get_walls()
  -- walls now is an array of arrays with the {Ax, Ay, Bx, By} wall coordinates



  -- Data structure for goals (your program can use this)
  goals = {}
  -- Fill it by parsing the scene in the GUI
  N_GOALS = get_goals()
  -- goals now is an array of arrays with the {Gx, Gy} goal coordinates

  for g=1,N_GOALS do
      print ("Goal" ..tostring(g).. " Gx " ..tostring(goals[g][1]).. " Gy " ..tostring(goals[g][2]))
  end



  -- Randomise robot start position to one of the goals with random orientation
  startGoal = math.random(N_GOALS)
  startx = goals[startGoal][1]
  starty = goals[startGoal][2]
  startOrientation = math.random() * 2 * math.pi
  setRobotPose(robotBase, startx, starty, startOrientation)


  -- These variables are for keeping score, and they will be changed by reachedGoal() --- don't change them directly!
  totalPoints = 0
  goalsReached = {}
  for i=1,N_GOALS do
      goalsReached[i] = false
  end


     
  -- Your code here!

  -- summary file for giving output
  -- file = io.open ("/Users/xushitong/code_log/robotics/summary.log", "w")
  -- io.output(file)

  -- general step list deciding which goal to move to
  goalSequence = {1,2,5,3,4} -- todo: hard code visit sequence  
  generalStep = ""
  generalStepCnt = 0
  generalStepFinish = true
  generalStepList = {}
  clearList(generalStepList, #goalSequence + 1) -- leave space for the rest goal points
  generalStepList[1] = "localize"
  generalStepList[6] = "repeat"
  
  -- motor speed values
  speedBase = 10
  turretSpeed = 0
  speedBaseL = 0
  speedBaseR = 0

  -- sensing consts
  sim.readProximitySensor(turretSensor) -- remove first reading
  dir_N = 64 -- number of meansurement around robot
  dir_index = 0 -- index of current meansure direction

  -- sensing variables
  readings = {}
  clearList(readings, dir_N)

  -- dummy points
  xArray = {}
  yArray = {}
  thetaArray = {}
  weightArray = {}
  dummyArray = {}
  dummy_N = 50
  for i=1,dummy_N do
    xArray[i] = 0
    yArray[i] = 0
    thetaArray[i] = 0
    weightArray[i] = 1/dummy_N
    dummyArray[i] = sim.createDummy(0.05)
    sim.setObjectPosition(dummyArray[i], -1, {0,0,0})
    sim.setObjectOrientation(dummyArray[i], -1, {0,0,0})
  end

  -- Target positions for joints
  motorAngleTargetL = 0.0
  motorAngleTargetR = 0.0

   -- To calibrate
  motorAnglePerMetre = 24.8
  motorAnglePerRadian = 3.05

  e_var = 2.290651837638239e-02
  f_var = 6.329178105143052e-02
  g_var = 0.013306540074499342

  -- minor step list
  waypoint_max_N = 40
  waypoints = {}

  waypoint_map = {}
  for i=1,N_GOALS do
    waypoint_map[i] = {}
    for j=1,N_GOALS do
      waypoint_map[i][j] = {}
      for k=1,waypoint_max_N do
        waypoint_map[i][j][k] = {0, 0}
      end
    end
  end
  waypoint_map[1][2] = {{1,1}, {0.5,1.5}, {-0.5,1.5}, {-2,1}, {-2, 2}}
  waypoint_map[2][5] = {{-2,0.5}, {-2, -1}}
  waypoint_map[5][3] = {{-1,-1.5}, {-2,-2}, {-1.5,-2.3}, {-1,-2.25}, {-0.5,-2.25}, {0,-2.25}, {0.5, -2.25},  {1, -2.25},  {1.7, -2.3},  {2, -2}}
  waypoint_map[3][4] = {{2, -1}, {2, 0}, {2, 1}, {2, 2}}
  waypoint_map[4][1] = {{1,1}, {0, 0}}


  stepCounter = 1 -- initialized to number not 0, set to 0 after general step construction finish
  stepCompletedFlag = false
  stepList = {}
  local session = {"set_waypoint", "turn", "stop", "localization", "forward", "stop", "localization"} -- example session of each waypoint move
  for i=1,waypoint_max_N * #session + 2 do
    stepList[i] = {"dummy_operation", {0, 0}}
  end
  
end

function sysCall_sensing()
  
end

function sysCall_actuation() 
  tt = sim.getSimulationTime()
  -- print("actuation hello", tt)
    
  -- Get and plot current angles of motor joints
  posL = sim.getJointPosition(leftMotor)
  posR = sim.getJointPosition(rightMotor)


  -- Your code here!

  -- new step check, new general step only general and minor step finished
  if (generalStepFinish) then 
    generalStepFinish = false
    generalStepCnt = generalStepCnt + 1
    generalStep = generalStepList[generalStepCnt]

    if (generalStep == "localize") then

    elseif (generalStep == "repeat") then
      generalStepCnt = 1
    else
      -- start of new inter goal point move
      constructMinorStep()

      -- allow enter minor step list
      stepCompletedFlag = true
      stepCounter = 0
      -- disable enter general step list
      generalStep = "running"
    end
  end

  -- general step action/finish check

  if (generalStep == "localize") then 
    local finishReading = sensorRead()
    if finishReading then
      -- get optim_id
      -- io.write("l = [" ..table.concat(readings, ", ").."];\n")
      local optim_id, min_loss = 0, 100000
      for g=1,N_GOALS do 
        local estimates = estimateRead(goals[g][1], goals[g][2], 0)
        -- io.write("g"..g.." = [" ..table.concat(estimates, ", ").."];\n")
        local loss = fftMSELoss(fft(toComplex(copylist(estimates))), readings_fft)

        if min_loss > loss then
          optim_id = g
          min_loss = loss
        end
      end

      -- construct generalStepList from optim_id
      local startIndex = 1
      for i=1,#goalSequence do
        if optim_id == goalSequence[i] then
          startIndex = i
          break
        end
      end
      for i=0,#goalSequence - startIndex + 1 do
        generalStepList[i + 1] = goalSequence[i + startIndex]
      end
      for i=1,startIndex - 1 do 
        generalStepList[#goalSequence - startIndex + i + 1] = goalSequence[i]
      end
      print("general step:", generalStepList)

      constructMinorStep()

      -- move all dummy to start goal point
      local theta = getPhaseShift(estimateRead(goals[optim_id][1], goals[optim_id][2], 0), readings)
      for i=1,dummy_N do
        xArray[i] = goals[optim_id][1]
        yArray[i] = goals[optim_id][2]
        thetaArray[i] = theta
        weightArray[i] = 1/dummy_N
        sim.setObjectPosition(dummyArray[i], -1, {xArray[i],yArray[i],0})
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]})
      end

      -- allow enter minor step list
      stepCompletedFlag = true
      stepCounter = 0
      -- disable enter general step list
      generalStep = "running"

      clearSensorRead()
    end
  else

  end

  -- minor step action/finish check
  if (stepCompletedFlag == true or stepCounter == 0) then
    stepCounter = stepCounter + 1
    stepCompletedFlag = false


    newStepType = stepList[stepCounter][1]

    if (newStepType == "end") then
        print("index: ", wrap(generalStepCnt + 1, 1, 5))
        reachedGoal(generalStepList[wrap(generalStepCnt + 1, 1, 5)], robotBase)
        -- finish moving to the next goal
        generalStepFinish = true
        stepCounter = 0
        return
    elseif (newStepType == "set_waypoint") then
        targetX, targetY = stepList[stepCounter][2], stepList[stepCounter][3]
        estimateX, estimateY = weightedMean(xArray, weightArray), weightedMean(yArray, weightArray)
        diffX, diffY = targetX - estimateX, targetY - estimateY

        D = math.sqrt(diffX * diffX + diffY * diffY)
        alpha = math.atan2(diffY, diffX)

        meanTheta = weightedMean(thetaArray, weightArray)
        meanTheta = castTheta(meanTheta, 2 * math.pi)
        alpha = alpha - meanTheta
        alpha = castTheta(alpha, 2 * math.pi)

        stepCompletedFlag = true
    elseif (newStepType == "forward") then
        -- Forward step: set new joint targets
        motorAngleTargetL = posL + D * motorAnglePerMetre
        motorAngleTargetR = posR + D * motorAnglePerMetre

    elseif (newStepType == "turn") then
        -- Turn step: set new targets
        motorAngleTargetL = posL - alpha * motorAnglePerRadian
        motorAngleTargetR = posR + alpha * motorAnglePerRadian

    elseif (newStepType == "stop") then
        -- print ("Stopping!")
    elseif (newStepType == "localize") then 
        -- print ("Localizing!")
    else
        print("WARNING: unknown minor step type: ", newStepType)
    end
  end


  -- Handle current ongoing step
  stepType = stepList[stepCounter][1]

  if (stepType == "turn") then
      if (alpha >= 0) then
          speedBaseL = -speedBase
          speedBaseR = speedBase
      else
          speedBaseL = speedBase
          speedBaseR = -speedBase
      end
      motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
      -- Slow down when close
      if (motorAngleFromTarget < 3) then
          speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
          speedBaseL = speedBaseL * speedScaling
          speedBaseR = speedBaseR * speedScaling
      end
      if (motorAngleFromTarget == 0) then
          stepCompletedFlag = true
      end
  elseif (stepType == "forward") then
      speedBaseL = speedBase
      speedBaseR = speedBase
      motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
      -- Slow down when close
      if (motorAngleFromTarget < 3) then
          speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
          speedBaseL = speedBaseL * speedScaling
          speedBaseR = speedBaseR * speedScaling
      end
      if (motorAngleFromTarget == 0) then
          stepCompletedFlag = true
      end
      
  elseif (stepType == "stop") then
      speedBaseL = 0
      speedBaseR = 0

      -- Check to see if the robot is stationary to within a small threshold
      linearVelocity,angularVelocity=sim.getVelocity(robotBase)
      vLin = math.sqrt(linearVelocity[1]^2 + linearVelocity[2]^2 + linearVelocity[3]^2)
      vAng = math.sqrt(angularVelocity[1]^2 + angularVelocity[2]^2 + angularVelocity[3]^2)
      --print ("stop", linearVelocity, vLin, vAng)

      if (vLin < 0.001 and vAng < 0.01) then
          stepCompletedFlag = true
      end
  elseif (stepType == "localize") then 
      local finishReading = sensorRead() 
      if finishReading then
        updatePredictedPositions()
        clearSensorRead()
        stepCompletedFlag = true
      end
  end

  -- Set the motor velocities for the current step
  sim.setJointTargetVelocity(leftMotor,speedBaseL)
  sim.setJointTargetVelocity(rightMotor,speedBaseR)        




end

function sysCall_cleanup()
  --simUI.destroy(ui)
  -- io.close(file)
end 
