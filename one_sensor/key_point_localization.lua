-- Generate a sample from a Gaussian distribution
function gaussian (mean, variance)
  return  math.sqrt(-2 * variance * math.log(math.random() + 0.00001)) *
          math.cos(2 * math.pi * math.random()) + mean
end

function copylist(list)
  local copy = {}
  for i=1,#list do 
    copy[i] = list[i]
  end
  return copy
end

-- inplace set list element to 0
function clearList(list, n) 
  for i=1,n do
    list[i] = 0
  end
end

function inRange(inter, a, b)
  return (a <= inter and inter <= b) or (b <= inter and inter <= a)
end

function wrap(i, min, max)
  if i < min then 
    return max - (min - i - 1)
  elseif i > max then 
    return min + (i - max - 1)
  else
    return i
  end
end

function euclideanDist(n)
  local cnt = 0
  while n ~= 0 do 
    if n & 1 == 1 then 
      cnt = cnt + 1
    end
    n = n >> 1
  end
  return cnt
end

-- get average of list, due to angle being cyclic, [180, -179] should be averaged to -179.5
function cyclicWeightedAverage(list)
  local avg1, avg2 = 0, 0
  local loss1, loss2 = 0, 0
  for i, angle in ipairs(list) do
    avg1 = avg1 + wrap(angle, 0, dir_N) / #list
    avg2 = avg2 + wrap(angle, -dir_N / 2, dir_N / 2) / #list
    loss1 = loss1 + math.abs(avg1 - angle)
    loss2 = loss2 + math.abs(avg2 - angle)
  end
  if loss1 < loss2 then 
    return avg1
  else
    return avg2
  end
end

-- -- component used in each iteration for calculating MSE
function MSEComponent(z, m)
  return (m - z) * (m - z)
end

-- component used in each iteration for calculating likelihood
function likelihoodComponent(z, m)
  local var = 0.01
  return math.exp(-math.pow((z - m), 2) / (2 * var))
end

-- NOTE: kernel size has to be odd
function convolution(list, kernel)
  local padding = math.floor(#kernel / 2)
  local stride = 1
  local result = {}
  clearList(result, #list)
  for i=1,#list do
    for j=-padding,padding do
      if (i + j >= 1 and i + j <= #list) then 
        result[i] = result[i] + kernel[j + padding + 1] * list[i + j]
      end
    end
  end
  return result
end

-- gaussian kernel
function gaussianKernel(sigma)
  local side = 3 * sigma
  local kernel = {}
  for i=-side,side do
      kernel[i + side + 1] = math.exp(-i*i / (2 * sigma*sigma)) / (math.sqrt(2 * math.pi) * sigma)
  end
  return kernel
end

function DoG(list, sigmas)
  local result = {}
  local prev = {}
  clearList(prev, #list)

  -- get multiple gaussian filter result
  for i, sigma in ipairs(sigmas) do 
    result[i] = {}
    local curr = convolution(list, gaussianKernel(sigma))
    if i == 1 then 
      prev = curr
    else
      for j=1,#list do
        result[i-1][j] = curr[j] - prev[j]
      end
    end
  end
  return result
end

-- get derivative of list, the first point's previous point is the last point in the list
function getDerivative(list)
  local result = {}
  for i=1,#list do
    local prev, next = wrap(i - 1, 1, #list), wrap(i + 1, 1, #list)
    result[i] = (list[next] - list[prev]) / 2
  end
  return result
end

function getKeyPoints(list, threshould)
  local keyPoints = {}

  for i=1,#list do
    local prev, next = wrap(i - 1, 1, #list), wrap(i + 1, 1, #list)
    if ((list[prev] < list[i] and list[next] < list[i]) 
         or (list[prev] > list[i] and list[next] > list[i])) 
        and math.abs(list[i]) > threshould then 
      keyPoints[#keyPoints + 1] = i
    end
  end
  return keyPoints
end

function getKeyDesPairs(list, sigmas, requiredLength)
  -- 1. use DoG for key point localization
  local listDoG = DoG(list, sigmas)[1]

  -- io.write("dog = ["..table.concat(listDoG, ", ").."];\n")

  -- 2. scale space estrema
  local listKeys = {}
  local threshould = 0.4
  while #listKeys < requiredLength do
    listKeys = getKeyPoints(listDoG, threshould)
    threshould = threshould - 0.01
  end
  -- print("length: ", #listKeys, "keypoints: "..table.concat(listKeys, ", "))

  -- 3. key point refine
  local oneDerivative = getDerivative(listDoG)
  local twoDerivative = getDerivative(oneDerivative)
  -- print("one deri: ", oneDerivative)
  -- print("two deri: ", twoDerivative)
  for i, key in ipairs(listKeys) do
    listKeys[i] = listKeys[i] - oneDerivative[key] / twoDerivative[key]
  end
  -- print("after refine: ", listKeys)

  -- 4. get point descriptor
  local BRIEF_RAND = {}
  local RANGE = 16
  local STEP = 1
  for i=-RANGE,RANGE,STEP do
    for j=-RANGE,RANGE,STEP do
      BRIEF_RAND[#BRIEF_RAND + 1] = {i, j}
    end
  end
  local keyDesPairs = {}
  for i=1,#listKeys do
    local descriptor = 0
    for j, pair in ipairs(BRIEF_RAND) do
      local deltaL, deltaR = pair[1], pair[2]
      local L, R = wrap(math.floor(listKeys[i] + deltaL + 0.5), 1, #listDoG), wrap(math.floor(listKeys[i] + deltaR + 0.5), 1, #listDoG)
      local cmp = 0
      
      if inRange(L, 1, #listDoG) and inRange(R, 1, #listDoG) and listDoG[L] > listDoG[R] then 
        cmp = 1
      end
      descriptor = (descriptor << 1) + cmp
    end
    keyDesPairs[i] = {listKeys[i], descriptor}
  end
  
  return keyDesPairs
end

function BRIEF(estimate, actual)

  -- file = io.open ("/Users/xushitong/code_log/robotics/summary.log", "w")
  -- io.output(file)
  
  local sigmas = {2, 2 * math.sqrt(2)}
  local actualPairs = getKeyDesPairs(actual, sigmas, 1) -- require at least certain number of key points
  local estimatePairs = getKeyDesPairs(estimate, sigmas, #actualPairs) -- require at least same amount of key point as accurate meansure

  -- 4. find pairs of key point that match up, calculate shift as average of the shift]
  local shifts = {}
  for i, actualPoint in ipairs(actualPairs) do
    -- get closest feature of this estimatepoint
    local closest = estimatePairs[1]
    for j, estimatePoint in ipairs(estimatePairs) do
      if euclideanDist(actualPoint[2] ~ estimatePoint[2]) < euclideanDist(actualPoint[2] ~ closest[2]) then
        closest = estimatePoint
      end
    end
    shifts[#shifts + 1] = closest[1] - actualPoint[1]
  end

  -- io.close(file)
  return castTheta(2 * math.pi * cyclicWeightedAverage(shifts) / dir_N)
end

-- get key, refined key entities
function getKeyDoGEntities(list, sigmas, requiredLength)
  -- 1. use DoG for key point localization
  local listDoG = DoG(list, sigmas)[1]

  -- io.write("dog = ["..table.concat(listDoG, ", ").."];\n")

  -- 2. scale space estrema
  local listKeys = {}
  local threshould = 0.4
  while #listKeys < requiredLength do
    listKeys = getKeyPoints(listDoG, threshould)
    threshould = threshould - 0.01
  end
  -- print("length: ", #listKeys, "keypoints: "..table.concat(listKeys, ", "))

  -- 3. key point refine, and interpolate DoG
  local oneDerivative = getDerivative(listDoG)
  local twoDerivative = getDerivative(oneDerivative)

  local keyDoGEntities = {}
  for i, key in ipairs(listKeys) do
    local delta = oneDerivative[key] / twoDerivative[key]
    keyDoGEntities[#keyDoGEntities + 1] = {key, key - delta}
  end

  return keyDoGEntities
end

-- interpolate DoG reading, get shift with the lowest mse DoG loss
function mseRefinedShift(estimate, actual)

  -- 1. mse for getting mapping of key points
  local optim_mse = 100000
  local shift_coarse = 0
  for i=1,dir_N do
    local mse = 0
    for j=1,#estimate do
      mse = mse + math.pow(estimate[math.mod(i + j, #estimate) + 1] - actual[j], 2)
    end
    if (mse / #estimate < optim_mse) then
      optim_mse = mse / #estimate
      shift_coarse = i
    end
  end

  -- file = io.open ("/Users/xushitong/code_log/robotics/summary.log", "w")
  -- io.output(file)
  -- io.write("estimate = ["..table.concat(estimate, ", ").."];\n")
  -- io.write("actual = ["..table.concat(actual, ", ").."];\n")


  -- 2. get key point pairs
  local sigmas = {2, 2 * math.sqrt(2)}
  local estimateEntities = getKeyDoGEntities(estimate, sigmas, 4)
  local actualDoG = DoG(actual, sigmas)[1]
  -- io.write("dog2 = ["..table.concat(actualDoG, ", ").."];\n")
  local actualOneDeriv = getDerivative(actualDoG)
  local actualTwoDetiv = getDerivative(actualOneDeriv)
  local shifts = {}
  for i, estimateEntity in ipairs(estimateEntities) do
    -- get DoG of actual signal at shifted direction
    local actualKey = wrap(estimateEntity[1] - shift_coarse, 1, #actualDoG)
    local refinedActualKey = actualKey - actualOneDeriv[actualKey] / actualTwoDetiv[actualKey]
    shifts[#shifts + 1] = estimateEntity[2] - refinedActualKey
  end

  -- print("mse shift: ", shift_coarse, "shifts: ", shifts, "final shift: ", cyclicWeightedAverage(shifts))

  -- io.close(file)
  return castTheta(2 * math.pi * cyclicWeightedAverage(shifts) / dir_N)

end
