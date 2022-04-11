import { Button } from "@mantine/core";
import { useEffect, useState } from "react";
import ROSLIB, { Vector3 } from "roslib";
import { round } from "../../utils/helpers";
import styles from "./MovementControls.module.css"

type MovementControlsProps = {
    ros: ROSLIB.Ros
}

const CMD_INTERVAL_MS = 100
const MOVE_VELOCITY_MAGNITUDE = 0.1
const TURN_VELOCITY_MAGNITUDE = Math.PI / 8
const FORWARD = MOVE_VELOCITY_MAGNITUDE
const BACK = -MOVE_VELOCITY_MAGNITUDE
const LEFT = TURN_VELOCITY_MAGNITUDE
const RIGHT = -TURN_VELOCITY_MAGNITUDE
const PRECISION = 3

export default function MovementControls({ ros }: MovementControlsProps) {
    const [moveVel, setMoveVel] = useState(0.0)
    const [turnVel, setTurnVel] = useState(0.0)
    const movementTopic = new ROSLIB.Topic({
        ros: ros,
        name: 'cmd_vel',
        messageType: 'geometry_msgs/Twist'
    })

    const createMoveHandlers = (moveVel: number) => {
        return {
            onMouseDown:() => setMoveVel(val => val + moveVel),
            onMouseUp:() => setMoveVel(val => val - moveVel)
        }
    }

    const createTurnHandlers = (moveVel: number) => {
        return {
            onMouseDown:() => setTurnVel(val => val + moveVel),
            onMouseUp:() => setTurnVel(val => val - moveVel)
        }
    }

    useEffect(() => {
        if (round(moveVel, PRECISION) !== 0 || round(turnVel, PRECISION) !== 0) {
            const msg = new ROSLIB.Message({
                linear: new Vector3({ x: moveVel }),
                angular: new Vector3({ z: turnVel }),
            })
            const interval = setInterval(
                () => {
                    console.log(msg);
                    movementTopic.publish(msg)
                },
                CMD_INTERVAL_MS
            )
    
            return () => clearInterval(interval)
        }
    }, [moveVel, turnVel])

    return (
        <div>
            <h3>Movement Controls</h3>
            <div className={styles.topButtons}>
                <Button {...createMoveHandlers(FORWARD)}>Forward</Button>
            </div>
            <div className={styles.bottomButtons}>
                <Button {...createTurnHandlers(LEFT)}>Left</Button>
                <Button {...createMoveHandlers(BACK)}>Back</Button>
                <Button {...createTurnHandlers(RIGHT)}>Right</Button>
            </div>
        </div>
    )
}