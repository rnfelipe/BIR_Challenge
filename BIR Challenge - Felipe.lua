-- This is the Epuck principal control script. It is threaded

actualizeLEDs = function()
    if (relLedPositions == nil) then
        relLedPositions = {{-0.0343,0,0.0394},{-0.0297,0.0171,0.0394},{0,0.0343,0.0394},
                    {0.0297,0.0171,0.0394},{0.0343,0,0.0394},{0.0243,-0.0243,0.0394},
                    {0.006,-0.0338,0.0394},{-0.006,-0.0338,0.0394},{-0.0243, -0.0243,0.0394}}
    end
    if (drawingObject) then
        sim.removeDrawingObject(drawingObject)
    end
    type = sim.drawing_painttag+sim.drawing_followparentvisibility+sim.drawing_spherepoints+
        sim.drawing_50percenttransparency+sim.drawing_itemcolors+sim.drawing_itemsizes+
        sim.drawing_backfaceculling+sim.drawing_emissioncolor
    drawingObject=sim.addDrawingObject(type,0,0,bodyElements,27)
    m = sim.getObjectMatrix(ePuckBase,-1)
    itemData = {0,0,0,0,0,0,0}
    sim.setLightParameters(ledLight,0)
    for i = 1,9,1 do
        if (ledColors[i][1]+ledColors[i][2]+ledColors[i][3]~=0) then
            p = sim.multiplyVector(m,relLedPositions[i])
            itemData[1] = p[1]
            itemData[2] = p[2]
            itemData[3] = p[3]
            itemData[4] = ledColors[i][1]
            itemData[5] = ledColors[i][2]
            itemData[6] = ledColors[i][3]
            sim.setLightParameters(ledLight,1,{ledColors[i][1],ledColors[i][2],ledColors[i][3]})
            for j = 1,3,1 do
                itemData[7] = j*0.003
                sim.addDrawingObjectItem(drawingObject,itemData)
            end
        end
    end
end

-- Function to read the Light Sensors 
getLightSensors = function()
    data = sim.receiveData(0,'EPUCK_lightSens')
    if (data) then
        lightSens=sim.unpackFloatTable(data)
    end
    return lightSens
end

turnLeft = function()
    velLeft = -1
    velRight = 1
    sim.setJointTargetVelocity(leftMotor,velLeft)
    sim.setJointTargetVelocity(rightMotor,velRight)
end

turnRight = function()
    velLeft = 1
    velRight = -1
    sim.setJointTargetVelocity(leftMotor,velLeft)
    sim.setJointTargetVelocity(rightMotor,velRight)
end

function sysCall_threadmain()
    -- Objects Instantiation
    sim.setThreadSwitchTiming(200) -- We will manually switch in the main loop
    -- ePuck body objects instantiation
    bodyElements = sim.getObjectHandle('ePuck_bodyElements')
    leftMotor = sim.getObjectHandle('ePuck_leftJoint')
    rightMotor = sim.getObjectHandle('ePuck_rightJoint')
    ePuck = sim.getObjectHandle('ePuck')
    ePuckBase = sim.getObjectHandle('ePuck_base')
    ledLight = sim.getObjectHandle('ePuck_ledLight')

    -- Finish Point instantiation
    finishPoint = sim.getObjectHandle('obstacle_1')

    -- ePuck's Camera instantiantion
    camera = sim.getObjectHandle('ePuck_camera')

    -- Walls instantiation
    wallBack = sim.getObjectHandle('wall_back')
    wallFront = sim.getObjectHandle('wall_front')
    wallLeft = sim.getObjectHandle('wall_left')
    wallRight = sim.getObjectHandle('wall_right')

    -- ePuck's Proximity Sensors instantiation
    proxSens = {-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,8,1 do
        proxSens[i] = sim.getObjectHandle('ePuck_proxSensor'..i)
    end
    maxVel = 300*math.pi/180
    ledColors = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}}

    -- Braitenberg weights for the 4 front prox sensors (avoidance):
    braitFrontSens_leftMotor = {1,2,-2,-1}
    -- Braitenberg weights for the 2 side prox sensors (following):
    braitSideSens_leftMotor = {-1,0}
    -- Braitenberg weights for the 8 sensors (following):
    braitAllSensFollow_leftMotor = {-3,-1.5,-0.5,0.8,1,0,0,-4}
    braitAllSensFollow_rightMotor = {0,1,0.8,-0.5,-1.5,-3,-4,0}
    braitAllSensAvoid_leftMotor = {0,0.5,1,-1,-0.5,-0.5,0,0}
    braitAllSensAvoid_rightMotor = {-0.5,-0.5,-1,1,0.5,0,0,0}

    -- Here we execute the regular main code:
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        st = sim.getSimulationTime() -- function to get the simulation time
        velLeft = 0
        velRight = 0
        opMode = sim.getScriptSimulationParameter(sim.handle_self,'opMode')
        lightSens = getLightSensors()
        s = sim.getObjectSizeFactor(bodyElements) -- make sure that if we scale the robot during simulation, other values are scaled too!
        noDetectionDistance = 0.065*s -- minimum detection distance for the proximity sensors, based in the size ratio of the ePuck's body elements
        -- Initialization of the detection distance for the proximity sensors
        proxSensDist = {noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
        for i = 1,8,1 do
            res,dist = sim.readProximitySensor(proxSens[i])
            if (res>0) and (dist<noDetectionDistance) then
                proxSensDist[i] = dist
            end
        end
        -- Check if the two front proxSens recognize the big Cylinder
        proxSens2fin = sim.checkProximitySensor(proxSens[2],finishPoint)
        proxSens3fin = sim.checkProximitySensor(proxSens[3],finishPoint)
        proxSens4fin = sim.checkProximitySensor(proxSens[4],finishPoint)
        proxSens5fin = sim.checkProximitySensor(proxSens[5],finishPoint)

        -- Check if the two front, and two diagonal/front proxSens recognize the Front Wall
        proxSens2wf = sim.checkProximitySensor(proxSens[2],wallFront)
        proxSens3wf = sim.checkProximitySensor(proxSens[3],wallFront)
        proxSens4wf = sim.checkProximitySensor(proxSens[4],wallFront)
        proxSens5wf = sim.checkProximitySensor(proxSens[5],wallFront)

        -- Check if the two front, and two diagonal/front proxSens recognize the Back Wall
        proxSens2wb = sim.checkProximitySensor(proxSens[2],wallBack)
        proxSens3wb = sim.checkProximitySensor(proxSens[3],wallBack)
        proxSens4wb = sim.checkProximitySensor(proxSens[4],wallBack)
        proxSens5wb = sim.checkProximitySensor(proxSens[5],wallBack)

        -- Check if the two front, and two diagonal/front proxSens recognize the Left Wall
        proxSens2wl = sim.checkProximitySensor(proxSens[2],wallLeft)
        proxSens3wl = sim.checkProximitySensor(proxSens[3],wallLeft)
        proxSens4wl = sim.checkProximitySensor(proxSens[4],wallLeft)
        proxSens5wl = sim.checkProximitySensor(proxSens[5],wallLeft)

        -- Check if the two front, and two diagonal/front proxSens recognize the Right Wall
        proxSens2wr = sim.checkProximitySensor(proxSens[2],wallRight)
        proxSens3wr = sim.checkProximitySensor(proxSens[3],wallRight)
        proxSens4wr = sim.checkProximitySensor(proxSens[4],wallRight)
        proxSens5wr = sim.checkProximitySensor(proxSens[5],wallRight)
        
        -- Read the detection of the front and side proxSens
        ReadSens1 = sim.readProximitySensor(proxSens[1])
        ReadSens2 = sim.readProximitySensor(proxSens[2])
        ReadSens3 = sim.readProximitySensor(proxSens[3])
        ReadSens4 = sim.readProximitySensor(proxSens[4])
        ReadSens5 = sim.readProximitySensor(proxSens[5])
        ReadSens6 = sim.readProximitySensor(proxSens[6])
        
        -- Functions and logic for the ePuck to get back to the line after avoiding any obstacle in any position
        if lightSens and ((lightSens[1]<0.5) and (lightSens[2]<0.5) and (lightSens[3]<0.5)) then   
            sensDist6 = sim.readProximitySensor(proxSens[6]) 
            sensDist1 = sim.readProximitySensor(proxSens[1]) 
            if(sensDist1 > 0) then
                totalTime = sim.getSimulationTime()
                now = totalTime
                interval = 0.8
                velLeft = 0
                velRight = 0
                while (now-totalTime<(interval)) do
                    turnRight()
                    now = sim.getSimulationTime()
                    sim.switchThread()
                end 
            end
            if (sensDist6 > 0) then
               totalTime = sim.getSimulationTime()
               now = totalTime
               interval = 0.8
               velLeft = 0
               velRight = 0
               while (now-totalTime<(interval)) do
                   turnRight()
                   now=sim.getSimulationTime()
                   sim.switchThread()
               end 
            end
        end

        -- Functions and logic to recognize the line when walking astray
        if lightSens and ((lightSens[1]<0.5) and (lightSens[2]<0.5) and (lightSens[3]<0.5)) then
            if proxSens and (ReadSens1 == 0) and ((ReadSens2 == 0) and (ReadSens3 == 0) and (ReadSens4 == 0) and (ReadSens5 == 0) and (ReadSens6 == 0)) then
                totalTime = sim.getSimulationTime()
                now = totalTime
                interval = 0.5
                velLeft = 0
                velRight = 0
                while (now-totalTime<(interval)) do
                    -- After recognizing the line, the ePuck must rotate to start following the line
                    turnRight()
                    now = sim.getSimulationTime()
                    sim.switchThread()
                end        
            end
        end

        if (opMode == 0) then -- We wanna follow the line
            if (math.mod(st,2)>1.5) then
                intensity = 1
            else
                intensity = 0
            end
            for i = 1,9,1 do
                ledColors[i] = {intensity,0,0} -- red
            end
            -- Now make sure the light sensors have been read, we have a line and the 4 front prox. sensors didn't detect anything:
            
            if lightSens and ((lightSens[1]<0.5)or(lightSens[2]<0.5)or(lightSens[3]<0.5)) and (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) then
                if (lightSens[1]>0.5) then 
                    velLeft = maxVel
                else
                    velLeft = maxVel*0.25
                end
                if (lightSens[3]>0.5) then 
                    velRight = maxVel
                else
                    velRight = maxVel*0.25
                end
            else
                velRight = maxVel
                velLeft = maxVel
                if (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) then
                    -- Nothing in front. Maybe we have an obstacle on the side, in which case we wanna keep a constant distance with it:
                    if (proxSensDist[1]>0.25*noDetectionDistance) then
                        velLeft = velLeft+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[1]/noDetectionDistance))
                        velRight = velRight+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[1]/noDetectionDistance))
                    end
                    if (proxSensDist[6]>0.25*noDetectionDistance) then
                        velLeft = velLeft+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[6]/noDetectionDistance))
                        velRight = velRight+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[6]/noDetectionDistance))
                    end
                else
                    -- Obstacle in front
                    velLeft = 0
                    velRight = 0
                    -- Check if recognize the Left Wall, and if they do, stop the ePuck and turn it around
                    if (proxSens2wl==1 or proxSens3wl==1 or proxSens4wl==1 or proxSens5wl==1) then
                        velLeft = 0
                        velRight = 0
                        totalTime = sim.getSimulationTime()
                        now = totalTime
                        interval = 3.0
                        while (now-totalTime<(interval)) do
                            turnLeft()
                            now = sim.getSimulationTime()
                            sim.switchThread()
                        end
                    else
                        -- Check if recognize the Right Wall
                        if (proxSens2wr==1 or proxSens3wr==1 or proxSens4wr==1 or proxSens5wr==1) then
                            velLeft = 0
                            velRight = 0
                            totalTime = sim.getSimulationTime()
                            now = totalTime
                            interval = 3.0
                            while (now-totalTime<(interval)) do
                                turnRight()
                                now = sim.getSimulationTime()
                                sim.switchThread()
                            end
                        else
                            -- Check if recognize the Front Wall
                            if (proxSens2wf==1 or proxSens3wf==1 or proxSens4wf==1 or proxSens5wf==1) then
                                velLeft = 0
                                velRight = 0
                                totalTime = sim.getSimulationTime()
                                now = totalTime
                                interval = 3.0
                                while (now-totalTime<(interval)) do
                                    turnRight()
                                    now = sim.getSimulationTime()
                                    sim.switchThread()
                                end
                            else
                                -- Check if recognize the Back wall
                                if (proxSens2wb==1 or proxSens3wb==1 or proxSens4wb==1 or proxSens5wb==1) then
                                    velLeft = 0
                                    velRight = 0
                                    totalTime=sim.getSimulationTime()
                                    now = totalTime
                                    interval = 3.0
                                    while (now-totalTime<(interval)) do
                                        turnRight()
                                        now = sim.getSimulationTime()
                                        sim.switchThread()
                                    end
                                else
                                    -- Check if recognize the Finish Point, and if recognized, stop the ePuck
                                    if (proxSens2fin==1 or proxSens3fin==1 or proxSens4fin==1 or proxSens5fin==1) then
                                        velLeft = 0
                                        velRight = 0
                                    else
                                        -- Use Braitenberg to avoid any other obstacle.
                                        for i = 1,4,1 do
                                            velLeft = velLeft+maxVel*braitFrontSens_leftMotor[i]*(1-(proxSensDist[1+i]/noDetectionDistance))
                                            velRight = velRight+maxVel*braitFrontSens_leftMotor[5-i]*(1-(proxSensDist[1+i]/noDetectionDistance))
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        sim.setJointTargetVelocity(leftMotor,velLeft)
        sim.setJointTargetVelocity(rightMotor,velRight)
        actualizeLEDs()
        sim.switchThread() -- Don't waste too much time in here (simulation time will anyway only change in next thread switch)
    end
end

function sysCall_cleanup()
    -- Put some clean-up code here:
    for i = 1,9,1 do
        ledColors[i] = {0,0,0} -- no light
    end
    actualizeLEDs()
end