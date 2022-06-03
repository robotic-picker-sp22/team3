import { Button, Slider } from "@mantine/core"
import { useToggle } from "@mantine/hooks"
import { useEffect, useState } from "react"
import ROSLIB from "roslib"
import { deg2rad, round } from "../../utils/helpers"
import { useTopicSubscriber } from "../../utils/RosHooks"
import styles from './HeadControls.module.css'

type HeadControlsProps = {
    ros: ROSLIB.Ros
}

const PREC = 2
const MSG_TYPE = "std_msgs/Float64"
const PAN_NAME = "joint_state_republisher/head_pan"
const TILT_NAME = "joint_state_republisher/head_tilt"

const SLIDER_STEP = 5 // Degrees
const PAN_MIN = -90
const PAN_MAX = 90
const TILT_MIN = -45
const TILT_MAX = 90
const PAN_MARKS = [
    { value: PAN_MIN, label: "Left" },
    { value: PAN_MAX, label: "Right"},
    { value: 0, label: "Front" }
]

const TILT_MARKS = [
    { value: TILT_MIN, label: "Up" },
    { value: TILT_MAX, label: "Down"},
    { value: 0, label: "Middle" }
]

export default function HeadControls({ ros }: HeadControlsProps) {

    const [editEnabled, toggleEnabled] = useToggle(false, [true, false])
    const head_pan_topic = new ROSLIB.Topic({
        ros,
        name: PAN_NAME,
        messageType: MSG_TYPE
    })

    const head_tilt_topic = new ROSLIB.Topic({
        ros,
        name: TILT_NAME,
        messageType: MSG_TYPE
    })

    const panMsg = useTopicSubscriber(head_pan_topic) as any
    const tiltMsg = useTopicSubscriber(head_tilt_topic) as any

    const [headTilt, setHeadTilt] = useState(round(tiltMsg.data || 0, PREC))
    const [headPan, setHeadPan] = useState(round(panMsg.data || 0, PREC))
    const service = new ROSLIB.Service({
        ros,
        name: "/web_teleop/set_head",
        serviceType: "web_teleop/SetHead",
    })


    useEffect(() => {

        const newHeadAngles = { pan: deg2rad(-headPan), tilt: deg2rad(headTilt) }
        console.log("Setting head angles to ", newHeadAngles);
        const request = new ROSLIB.ServiceRequest(newHeadAngles)
        service.callService(
            request,
            () => { console.log("Finished setting head angles to ", newHeadAngles) },
            (err) => { console.log("Failed to set head angles to ", newHeadAngles, err) },
        )

    }, [headPan, headTilt]) 
    
    return (
        <div>
            <h3>Head Controls</h3>
            <section>
                <p>Current Tilt: {round(tiltMsg.data || 0.0, PREC)} rad</p>
                <Slider 
                    min={TILT_MIN}
                    max={TILT_MAX}
                    step={SLIDER_STEP}
                    marks={TILT_MARKS}
                    defaultValue={0}
                    onChangeEnd={setHeadTilt}
                    disabled={!editEnabled}
                />
            </section>
            <section>
                <p>Current Pan: {round(panMsg.data || 0.0, PREC)} rad</p>
                <Slider 
                    min={PAN_MIN}
                    max={PAN_MAX}
                    step={SLIDER_STEP}
                    marks={PAN_MARKS}
                    defaultValue={0}
                    value={editEnabled ? headPan : round(panMsg.data || 0.0, PREC)}
                    onChangeEnd={setHeadPan}
                    disabled={!editEnabled}
                />
                <div className={styles.spacing}></div>
            </section>
            <Button color={!editEnabled ? "green" : "red"} onClick={() => toggleEnabled()}>
                {editEnabled ? "Disable" : "Enable"}
            </Button>
        </div>
    )
}