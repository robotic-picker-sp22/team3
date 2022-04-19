import { Button, Slider } from "@mantine/core"
import { useEffect, useState } from "react"
import ROSLIB from "roslib"

type ArmControlsProps = {
    ros: ROSLIB.Ros
}

// TODO: Add range details and integrate with backend
/*
    Missing Slider attributes
    min={}
    max={}
    step={}
    marks={}
    defaultValue={0}
*/

const SHOULDER_PAN = 1.32
const SHOULDER_LIFT = 1.4
const UPPERARM_ROLL = -0.2
const ELBOW_FLEX = 1.72
const FOREARM_ROLL = 7.55e-6
const WRIST_FLEX = 1.66
const WRIST_ROLL = 1.28e-6

export default function ArmControls({ ros }: ArmControlsProps) {

    const [shoulderPan , setShoulderPan] = useState(SHOULDER_PAN)
    const [shoulderLift , setShoulderLift] = useState(SHOULDER_LIFT)
    const [upperarmRoll , setUpperarmRoll] = useState(UPPERARM_ROLL)
    const [elbowFlex , setElbowFlex] = useState(ELBOW_FLEX)
    const [forearmRoll , setForearmRoll] = useState(FOREARM_ROLL)
    const [wristFlex , setWristFlex] = useState(WRIST_FLEX)
    const [wristRoll , setWristRoll] = useState(WRIST_ROLL)

    function degsToRads(deg: number) {return (deg * Math.PI) / 180.0}

    const service = new ROSLIB.Service({
        ros,
        name: "web_teleop/set_arm",
        serviceType: "web_teleop/SetArm"
    })
    useEffect(() => {
        console.log("Trying to move arm");
        const request = new ROSLIB.ServiceRequest({
            shoulder_pan: shoulderPan,
            shoulder_lift: shoulderLift,
            upperarm_roll: upperarmRoll,
            elbow_flex: elbowFlex,
            forearm_roll: forearmRoll,
            wrist_flex: wristFlex,
            wrist_roll: wristRoll,
        })
        service.callService(
            request,
            () => { console.log(`Finished moving arm`) },
            (e) => { console.log(`Failed to move arm`, e) },
        )
    }, [shoulderLift, shoulderPan, upperarmRoll, elbowFlex, wristFlex, wristRoll])
    return (
        <div>
            <h3>Arm Controls</h3>
            <h4>Shoulder Pan</h4>
            <Slider onChangeEnd={setShoulderPan}
                min={degsToRads(-92)}
                max={degsToRads(92)}
                step={degsToRads(1)}
            />
            <h4>Shoulder Lift</h4>
            <Slider onChangeEnd={setShoulderLift}
                min={degsToRads(-70)}
                max={degsToRads(87)}
                step={degsToRads(1)}
            />
            <h4>Upperarm Roll</h4>
            <Slider onChangeEnd={setUpperarmRoll}
                min={degsToRads(0)}
                max={degsToRads(360)}
                step={degsToRads(1)}
            />
            <h4>Elbow Flex</h4>
            <Slider onChangeEnd={setElbowFlex}
                min={degsToRads(-129)}
                max={degsToRads(129)}
                step={degsToRads(1)}
            />
            <h4>Forearm Roll</h4>
            <Slider onChangeEnd={setForearmRoll}
                min={degsToRads(0)}
                max={degsToRads(360)}
                step={degsToRads(1)}
            />
            <h4>Wrist Flex</h4>
            <Slider onChangeEnd={setWristFlex}
                min={degsToRads(-125)}
                max={degsToRads(125)}
                step={degsToRads(1)}
            />
            <h4>Wrist Roll</h4>
            <Slider onChangeEnd={setWristRoll}
                min={degsToRads(0)}
                max={degsToRads(360)}
                step={degsToRads(1)}
            />
            <Button onClick={() => {
                setShoulderPan(SHOULDER_PAN)
                setShoulderLift(SHOULDER_LIFT)
                setUpperarmRoll(UPPERARM_ROLL)
                setElbowFlex(ELBOW_FLEX)
                setForearmRoll(FOREARM_ROLL)
                setWristFlex(WRIST_FLEX)
                setWristRoll(WRIST_ROLL)
            }}>Default Position</Button>
            <Button onClick={() => {
                setShoulderPan(0.0)
                setShoulderLift(0.0)
                setUpperarmRoll(0.0)
                setElbowFlex(0.0)
                setForearmRoll(0.0)
                setWristFlex(0.0)
                setWristRoll(0.0)
            }}>Straight</Button>
        </div>
    )
}