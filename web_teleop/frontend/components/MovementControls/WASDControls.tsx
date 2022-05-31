import { Switch } from "@mantine/core"
import { useEffect, useState } from "react"
import ROSLIB, { Vector3 } from "roslib"
import { round } from "../../utils/helpers"

type WASDControlsProps = {
    ros: ROSLIB.Ros
}

const PRECISION = 3
const CMD_INTERVAL_MS = 100
const MOVE_VELOCITY_MAGNITUDE = 1
const TURN_VELOCITY_MAGNITUDE = Math.PI / 4
const FORWARD = MOVE_VELOCITY_MAGNITUDE
const BACK = -MOVE_VELOCITY_MAGNITUDE
const LEFT = TURN_VELOCITY_MAGNITUDE
const RIGHT = -TURN_VELOCITY_MAGNITUDE


export default function WASDControls({ ros }: WASDControlsProps) {

    const [moveVel, setMoveVel] = useState(0.0)
    const [turnVel, setTurnVel] = useState(0.0)
    const [isEnabled, setEnabled] = useState(false)

    const movementTopic = new ROSLIB.Topic({
        ros: ros,
        name: 'cmd_vel',
        messageType: 'geometry_msgs/Twist'
    })

    function downHandler({ key }:any) {
        if (key === 'w') {
            setMoveVel(FORWARD)
        } else if (key === 'a'){
            setTurnVel(LEFT)
        } else if (key === 's'){
            setMoveVel(BACK)
        } else if (key === 'd'){
            setTurnVel(RIGHT)
        }
    }
    
    function upHandler({ key }:any) {
        if (key === 'w') {
            setMoveVel(0)
        } else if (key === 'a') {
            setTurnVel(0)
        } else if (key === 's') {
            setMoveVel(0)
        } else if (key === 'd') {
            setTurnVel(0)
        }
    }


    useEffect(() => {
        if (isEnabled) {
            window.addEventListener("keydown", downHandler);
            window.addEventListener("keyup", upHandler);
        }

        return () => { 
            window.removeEventListener("keydown", downHandler);
            window.removeEventListener("keyup", upHandler);
        }
    }, [isEnabled])


    useEffect(() => {
        if (round(moveVel, PRECISION) !== 0 || round(turnVel, PRECISION) !== 0) {
            const msg = new ROSLIB.Message({
                linear: new Vector3({ x: moveVel }),
                angular: new Vector3({ z: turnVel }),
            })
            const interval = setInterval(
                () => {
                    movementTopic.publish(msg)
                },
                CMD_INTERVAL_MS
            )
    
            return () => {
                clearInterval(interval)
            }
        }
    }, [moveVel, turnVel])

    return (
        <div>
            <h3>WASDMovement</h3>
            <Switch
                checked={isEnabled}
                onLabel="On"
                offLabel="Off"
                label="WASD"
                onChange={(e) => setEnabled(e.currentTarget.checked)}
                size="xl"
            />
        </div>
        
    )
}