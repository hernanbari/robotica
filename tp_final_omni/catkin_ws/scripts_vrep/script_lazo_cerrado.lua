ENCODER_TICKS = 500

function callback_vleft(msg)
    vLeft = msg.data
end

function callback_vright(msg)
    vRight = msg.data
end

function getTicks(motor)
    -- Este valor esta en el rango [-PI, PI]
    joint_pos = simGetJointPosition(motor)
    -- Devolvemos los ticks en el rango [0, ENCODER_TICKS]
    return math.floor( ENCODER_TICKS * (joint_pos + math.pi) / (2.0 * math.pi) )
end

function deltaTicks(oldTicks, newTicks)
    -- check for wrap around
    if (3*ENCODER_TICKS/4) < oldTicks and newTicks < (ENCODER_TICKS/4) then
        return ENCODER_TICKS + newTicks - oldTicks
    elseif oldTicks < (ENCODER_TICKS/4) and (3*ENCODER_TICKS/4) < newTicks then
        return newTicks - oldTicks - ENCODER_TICKS
    else
        return newTicks - oldTicks
    end
end

function publishTF(frameid, child_frameid, position, qorientation)
    local tf_message = {
        header={
            stamp=currentTime,
            frame_id= frameid
        },
        child_frame_id= child_frameid,
        transform={
            translation = {x = position[1],y = position[2],z = position[3]},
            rotation = {x = qorientation[1],y = qorientation[2],z = qorientation[3],w = qorientation[4]}
        }
    }

    simExtRosInterface_sendTransform(tf_message)
end

function publishGroundTruth(currentTime, dt)

    local transformationMatrix=simGetObjectMatrix(ref,-1)

    --local refPosition=simGetObjectPosition(ref,-1)
    --local refQuaternion=simGetObjectQuaternion(ref,-1)
    local refPosition={transformationMatrix[4],transformationMatrix[8],transformationMatrix[12]}
    local refQuaternion=simGetQuaternionFromMatrix(transformationMatrix)

    local oldInverse=simGetInvertedMatrix(oldTransformationMatrix)
    local relativeTransformationMatrix=simMultiplyMatrices(oldInverse,transformationMatrix)

    local relativeTranslation={relativeTransformationMatrix[4],relativeTransformationMatrix[8],relativeTransformationMatrix[12]}
    --local relativeOrientation=simGetQuaternionFromMatrix(relativeTransformationMatrix)

    if dt>0 then
        relativeVelocity[1]=relativeTranslation[1]/dt
        relativeVelocity[2]=relativeTranslation[2]/dt
        relativeVelocity[3]=relativeTranslation[3]/dt
    end

    local message = {
        header={
            stamp=currentTime,
            frame_id="odom"
        },
        child_frame_id="base_link",
        pose={
              pose={
                    position={x=refPosition[1],y=refPosition[2],z=refPosition[3]},
                    orientation={x=refQuaternion[1],y=refQuaternion[2],z=refQuaternion[3],w=refQuaternion[4]},
                   }
             },
        twist={
            twist={
                linear={x=relativeVelocity[1],y=relativeVelocity[2],z=relativeVelocity[3]},
                -- TODO
                --abgular={x=...,y=...,z=...},
              }
        }
    }

    simExtRosInterface_publish(gt_publisher, message)

    oldTransformationMatrix=simCopyMatrix(transformationMatrix)
end

if (sim_call_type==sim_childscriptcall_initialization) then
    
    ref=simGetObjectHandle('Pioneer_p3dx')
    motorLeft=simGetObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=simGetObjectHandle("Pioneer_p3dx_rightMotor")

    vLeft=0
    vRight=0

    -- Check if the required RosInterface is there:
    moduleName=0
    index=0
    rosInterfacePresent=false
    while moduleName do
        moduleName=simGetModuleName(index)
        if (moduleName=='RosInterface') then
            rosInterfacePresent=true
        end
        index=index+1
    end

    -- Nos subscribimos al mensaje del teclado
    if rosInterfacePresent then
        subscriber=simExtRosInterface_subscribe('/robot/left_wheel/cmd_vel','std_msgs/Float64','callback_vleft', 1)
        subscriber=simExtRosInterface_subscribe('/robot/right_wheel/cmd_vel','std_msgs/Float64','callback_vright', 1)
        publisher=simExtRosInterface_advertise('/robot/encoders','robmovil_msgs/EncoderTicks')
        gt_publisher=simExtRosInterface_advertise('/robot/ground_truth','nav_msgs/Odometry')
    end

    -- initialize values

    lastTime=simGetSimulationTime()

    relativeVelocity = {0.0, 0.0, 0.0}
    oldTransformationMatrix=simGetObjectMatrix(ref,-1)

    lastTicksLeft = getTicks(motorLeft)
    lastTicksRight = getTicks(motorRight)

    totalTicksLeft = 0
    totalTicksRight = 0
end 

if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 


if (sim_call_type==sim_childscriptcall_sensing) then 

   local currentTime=simGetSimulationTime()
   local dt=currentTime-lastTime

    local ticksLeft = getTicks(motorLeft)
    local ticksRight = getTicks(motorRight)
    --simAddStatusbarMessage('ticks:'..ticks_left)

    -- check for wrap around
    totalTicksLeft = totalTicksLeft + deltaTicks(lastTicksLeft, ticksLeft)
    totalTicksRight = totalTicksRight + deltaTicks(lastTicksRight, ticksRight)
    --simAddStatusbarMessage('ticks: '..totalTicksLeft..', '..totalTicksRight)

    -- construct message
    local message = {
        header={
            stamp=currentTime,
            frame_id="odom"
        },
        ticks_left={data=totalTicksLeft},
        ticks_right={data=totalTicksRight},
    }

    simExtRosInterface_publish(publisher,message)

    lastTime=currentTime
    lastTicksLeft = ticksLeft
    lastTicksRight = ticksRight

    local refposition=simGetObjectPosition(ref,-1)
    local refquaternion=simGetObjectQuaternion(ref,-1)

    publishGroundTruth(currentTime, dt)

    publishTF("odom", "base_link_gt", refposition, refquaternion)
end

if (sim_call_type==sim_childscriptcall_actuation) then 
    simSetJointTargetVelocity(motorLeft,vLeft)
    simSetJointTargetVelocity(motorRight,vRight)
end 
