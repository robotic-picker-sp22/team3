import { Slider } from "@mantine/core"
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

export default function ArmControls({ ros }: ArmControlsProps) {

    const [shoulderPan , setShoulderPan] = useState(0.0)
    const [shoulderLift , setShoulderLift] = useState(0.0)
    const [upperarmRoll , setUpperarmRoll] = useState(0.0)
    const [elbowFlex , setElbowFlex] = useState(0.0)
    const [forearmRoll , setForearmRoll] = useState(0.0)
    const [wristFlex , setWristFlex] = useState(0.0)
    const [wristRoll , setWristRoll] = useState(0.0)

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
            <Slider onChangeEnd={setShoulderPan}/>
            <h4>Shoulder Lift</h4>
            <Slider onChangeEnd={setShoulderLift}/>
            <h4>Upperarm Roll</h4>
            <Slider onChangeEnd={setUpperarmRoll}/>
            <h4>Elbow Flex</h4>
            <Slider onChangeEnd={setElbowFlex}/>
            <h4>Forearm Roll</h4>
            <Slider onChangeEnd={setForearmRoll}/>
            <h4>Wrist Flex</h4>
            <Slider onChangeEnd={setWristFlex}/>
            <h4>Wrist Roll</h4>
            <Slider onChangeEnd={setWristRoll}/>
        </div>
    )
}