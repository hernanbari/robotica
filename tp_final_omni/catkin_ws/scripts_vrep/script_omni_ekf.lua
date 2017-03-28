-- ROS wheel velocities and encoder simulation extension:
ENCODER_TICKS = 500 -- Encoder resolution

function callback_vfront_left(msg)
    ros_vfront_left = msg.data
end

function callback_vfront_right(msg)
    ros_vfront_right = msg.data
end

function callback_vrear_left(msg)
    ros_vrear_left = msg.data
end

function callback_vrear_right(msg)
    ros_vrear_right = msg.data
end

function getTicksFromJoint(motor)
    -- This value is on range [-PI, PI]
    joint_pos = simGetJointPosition(motor)

    -- Calculate ticks between [0, ENCODER_TICKS]
    return math.floor( ENCODER_TICKS * (joint_pos + math.pi) / (2.0 * math.pi) )
end

function getTicksFromOrientation(motor)
    -- This value is on range [-PI, PI]
    joint_pos = simGetObjectOrientation(motor,-1)

    -- Calculate ticks between [0, ENCODER_TICKS]
    return math.floor( ENCODER_TICKS * (joint_pos[3] + math.pi) / (2.0 * math.pi) )
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

function publishTF(frameid, child_frameid, currentTime, transform)
    local position = {transform[4], transform[8], transform[12]}
    local qorientation = simGetQuaternionFromMatrix(transform)
    local tf_message = {
        header={
            stamp=currentTime,
            frame_id=frameid
        },
        child_frame_id=child_frameid,
        transform={
            translation = {x = position[1],y = position[2],z = position[3]},
            rotation = {x = qorientation[1],y = qorientation[2],z = qorientation[3],w = qorientation[4]}
        }
    }

    simExtRosInterface_sendTransform(tf_message)
end


function publishGroundTruth(currentTime, dt)

    local transformationMatrix=simGetObjectMatrix(youBotRef,-1)

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



-- This example script is non-threaded (executed at each simulation pass)
-- The functionality of this script (or parts of it) could be implemented
-- in an extension module (plugin) and be hidden. The extension module could
-- also allow connecting to and controlling the real robot.

if (sim_call_type==sim_childscriptcall_initialization) then 
    -- First time we execute this script. 

    -- Make sure we have version 2.4.12 or above (the omni-wheels are not supported otherwise)
    v=simGetInt32Parameter(sim_intparam_program_version)
    if (v<20412) then
        simDisplayDialog('Warning','The YouBot model is only fully supported from V-REP version 2.4.12 and above.&&nThis simulation will not run as expected!',sim_dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end

    --Prepare initial values and retrieve handles:
    wheelJoints={-1,-1,-1,-1} -- front left, rear left, rear right, front right
    wheelJoints[1]=simGetObjectHandle('rollingJoint_fl')
    wheelJoints[2]=simGetObjectHandle('rollingJoint_rl')
    wheelJoints[3]=simGetObjectHandle('rollingJoint_rr')
    wheelJoints[4]=simGetObjectHandle('rollingJoint_fr')
    youBot=simGetObjectHandle('youBot')
    youBotRef=simGetObjectHandle('youBot_ref')



    ui=simGetUIHandle('youBot_UI')
    simSetUIButtonLabel(ui,0,simGetObjectName(youBot)..' user interface') -- Set the UI title (with the name of the current robot)

    forwBackVelRange={-240*math.pi/180,240*math.pi/180}  -- min and max wheel rotation vel. for backward/forward movement
    leftRightVelRange={-240*math.pi/180,240*math.pi/180} -- min and max wheel rotation vel. for left/right movement
    rotVelRange={-240*math.pi/180,240*math.pi/180}       -- min and max wheel rotation vel. for left/right rotation movement

    forwBackVel=0
    leftRightVel=0
    rotVel=0
    initSizeFactor=simGetObjectSizeFactor(youBot) -- only needed if we scale the robot up/down

    -- ROS Interface initialization
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

    oldTransformationMatrix=simGetObjectMatrix(youBotRef,-1)
    relativeVelocity = {0.0, 0.0, 0.0}
    -- Wheels velocity subscriptions and encoders publication
    if rosInterfacePresent then
        subscriber=simExtRosInterface_subscribe('/robot/front_left_wheel/cmd_vel','std_msgs/Float64','callback_vfront_left', 1)
        subscriber=simExtRosInterface_subscribe('/robot/front_right_wheel/cmd_vel','std_msgs/Float64','callback_vfront_right', 1)
        subscriber=simExtRosInterface_subscribe('/robot/rear_left_wheel/cmd_vel','std_msgs/Float64','callback_vrear_left', 1)
        subscriber=simExtRosInterface_subscribe('/robot/rear_right_wheel/cmd_vel','std_msgs/Float64','callback_vrear_right', 1)
        encoders_publisher=simExtRosInterface_advertise('/robot/encoders','robmovil_msgs/MultiEncoderTicks')
        gt_publisher=simExtRosInterface_advertise('/robot/ground_truth','nav_msgs/Odometry')
    end

    -- Getting ROS comply reference frame (X-forward, Y-left, Z-upwards)
    ros_ref = simGetObjectHandle('KUKA_ROS_ref')

    -- This KuKa model seems to have the joints with inverted frames, ticks decrease
    -- while going forward. We invert the sign of ticks to fix this.
    kuka_joint_sign = -1

    -- Ground Truth position and orientations: 
    -- if robot isnt centered on scene this pose will be used to
    -- generate the ground truth information
    starting_Twr = simGetObjectMatrix(ros_ref, -1)
    Tow = starting_Twr -- Transformpation between odom and world
    simInvertMatrix(Tow)

    last_Twr = starting_Twr
   
    -- Time for message construction
    lastTime = simGetSimulationTime()

    -- Absolute acumulation of ticks
    totalTicksFrontLeft = 0
    totalTicksRearLeft = 0
    totalTicksFrontRight = 0
    totalTicksRearRight = 0

    intermediate_links={-1,-1,-1,-1} -- front left, rear left, rear right, front right
    intermediate_links[1]=simGetObjectHandle('intermediateLink_fl')
    intermediate_links[2]=simGetObjectHandle('intermediateLink_rl')
    intermediate_links[3]=simGetObjectHandle('intermediateLink_rr')
    intermediate_links[4]=simGetObjectHandle('intermediateLink_fr')

    -- Last encoder ticks obtained
    --lastTicksFrontLeft = getTicksFromOrientation(intermediate_links[1])
    --lastTicksRearLeft = getTicksFromOrientation(intermediate_links[2])
    --lastTicksRearRight = getTicksFromOrientation(intermediate_links[3])
    --lastTicksFrontRight = getTicksFromOrientation(intermediate_links[4])

    lastTicksFrontLeft = getTicksFromJoint(wheelJoints[1])
    lastTicksRearLeft = getTicksFromJoint(wheelJoints[2])
    lastTicksRearRight = getTicksFromJoint(wheelJoints[3])
    lastTicksFrontRight = getTicksFromJoint(wheelJoints[4])

    --simAddStatusbarMessage('Start ticks:'..lastTicksFrontLeft..' '..lastTicksFrontLeft..' '..
    --lastTicksRearLeft..' '..lastTicksRearRight)

    -- Velocities commands recived from ROS messages
    ros_vfront_left = 0
    ros_vrear_left = 0
    ros_vfront_right = 0
    ros_vrear_right = 0
end 

if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end

if (sim_call_type==sim_childscriptcall_sensing) then 
    local currentTime = simGetSimulationTime()
    local dt = currentTime-lastTime
    -- This KuKa model seems to have the joints with inverted frames, ticks decrease
    -- while going forward. We invert the sign of delta ticks applied to fix this.
    
    --local ticksFrontLeft = getTicksFromOrientation(intermediate_links[1])
    --local ticksRearLeft = getTicksFromOrientation(intermediate_links[2])
    --local ticksRearRight = getTicksFromOrientation(intermediate_links[3])
    --local ticksFrontRight = getTicksFromOrientation(intermediate_links[4])

    local ticksFrontLeft = getTicksFromJoint(wheelJoints[1])
    local ticksRearLeft = getTicksFromJoint(wheelJoints[2])
    local ticksRearRight = getTicksFromJoint(wheelJoints[3])
    local ticksFrontRight = getTicksFromJoint(wheelJoints[4])
   
    --simAddStatusbarMessage('Ticks:'..lastTicksFrontLeft..' '..lastTicksFrontRight..' '..
    --    lastTicksRearLeft..' '..lastTicksRearRight)

    -- check for wrap around
    totalTicksFrontLeft = totalTicksFrontLeft + kuka_joint_sign * deltaTicks(lastTicksFrontLeft, ticksFrontLeft)
    totalTicksRearLeft = totalTicksRearLeft + kuka_joint_sign * deltaTicks(lastTicksRearLeft, ticksRearLeft)
    totalTicksFrontRight = totalTicksFrontRight + kuka_joint_sign * deltaTicks(lastTicksFrontRight, ticksFrontRight)
    totalTicksRearRight = totalTicksRearRight + kuka_joint_sign * deltaTicks(lastTicksRearRight, ticksRearRight)
   
    --simAddStatusbarMessage('Total ticks: '..totalTicksFrontLeft..' '..totalTicksFrontRight..' '..
    --    totalTicksRearLeft..' '..totalTicksRearRight)

    -- construct message
    local message = {
        header={
            stamp=currentTime,
            frame_id="odom"
        },
        ticks={{data=totalTicksFrontLeft}, {data=totalTicksFrontRight},{data=totalTicksRearLeft},{data=totalTicksRearRight}},
    }

    simExtRosInterface_publish(encoders_publisher,message)

    lastTime = currentTime
   
    lastTicksFrontLeft = ticksFrontLeft
    lastTicksRearLeft = ticksRearLeft
    lastTicksFrontRight = ticksFrontRight
    lastTicksRearRight = ticksRearRight

    -- Robot position and orientation
    local Twr = simGetObjectMatrix(ros_ref, -1)
    -- Robot position and orientations in reference to the place where it started
    local Tor = simMultiplyMatrices(Tow, Twr)

    publishGroundTruth(currentTime, dt)

    publishTF("odom", "base_link_gt", currentTime, Tor)
end

if (sim_call_type==sim_childscriptcall_actuation) then 
    -- s will scale a few values hereafter (has only an effect if the robot is scaled down/up)
    s=simGetObjectSizeFactor(youBot)
    
    buttonID=simGetUIEventButton(ui)
    if (buttonID==200) then -- Forward/backward slider was changed
        forwBackVel=forwBackVelRange[1]+simGetUISlider(ui,buttonID)*0.001*(forwBackVelRange[2]-forwBackVelRange[1])
    end
    if (buttonID==201) then -- left/right slider was changed
        leftRightVel=leftRightVelRange[1]+simGetUISlider(ui,buttonID)*0.001*(leftRightVelRange[2]-leftRightVelRange[1])
    end
    if (buttonID==202) then -- left/right rotation slider was changed
        rotVel=rotVelRange[1]+simGetUISlider(ui,buttonID)*0.001*(rotVelRange[2]-rotVelRange[1])
    end
    if (buttonID==212) then -- stop button was clicked
        forwBackVel=0
        leftRightVel=0
        rotVel=0
        -- Reset the wheel movement sliders to the neutral position:
        simSetUISlider(ui,200,500)
        simSetUISlider(ui,201,500)
        simSetUISlider(ui,202,500)
    end
    
    -- Now apply the desired wheel velocities:
    -- ROS extension apply velocities recived from messages plus any velocity got from KUKA UI.
    -- NOTE: Wheel joints seems to be inverted, thats why we invert sign of velocities
    simSetJointTargetVelocity(wheelJoints[1], (kuka_joint_sign * ros_vfront_left) + -forwBackVel-leftRightVel-rotVel)
    simSetJointTargetVelocity(wheelJoints[2], (kuka_joint_sign * ros_vrear_left) + -forwBackVel+leftRightVel-rotVel)
    simSetJointTargetVelocity(wheelJoints[3], (kuka_joint_sign * ros_vrear_right) + -forwBackVel-leftRightVel+rotVel)
    simSetJointTargetVelocity(wheelJoints[4], (kuka_joint_sign * ros_vfront_right) + -forwBackVel+leftRightVel+rotVel)
end 
