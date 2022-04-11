import { Button, NumberInput, Slider } from "@mantine/core"
import { useState } from "react"
import ROSLIB from "roslib"
import { round } from "../../utils/helpers"
import { useTopicSubscriber } from "../../utils/RosHooks"
import styles from "./TorsoControls.module.css"

type TorsoControlsProps = {
    ros: ROSLIB.Ros
}

const PRECISION = 3
const MAX_HEIGHT = 0.4
const MARKS = [
    { value: 0, label: "0m" },
    { value: MAX_HEIGHT/4, label: MAX_HEIGHT/4 + "m" },
    { value: MAX_HEIGHT/2, label: MAX_HEIGHT/2 + "m" },
    { value: round(MAX_HEIGHT / 4 * 3, 1), label: round(MAX_HEIGHT / 4 * 3, 1) + "m" },
    { value: MAX_HEIGHT, label: MAX_HEIGHT + "m" }
]
export default function TorsoControls({ ros }: TorsoControlsProps) {
    const rosTopic = new ROSLIB.Topic({
        ros: ros,
        name: 'joint_state_republisher/torso_lift_joint',
        messageType: 'std_msgs/Float64'
    })
    const torsoMsg = useTopicSubscriber(rosTopic) as any
    const torsoHeight = torsoMsg.data || 0 as number
    const service = new ROSLIB.Service({
        ros,
        name: "/web_teleop/set_torso",
        serviceType: "web_teleop/SetTorso",
    })

    const [height, setHeight] = useState(0.0)
    
    const handleChange = (val: number) => {
        val = round(val, PRECISION)
        console.log("Setting torso height to %fm", val);
        const request = new ROSLIB.ServiceRequest({ height: val })
        service.callService(
            request,
            () => { console.log("Finished setting torso height to %fm", val) },
            () => { console.log("Failed to set torso height to %fm", val) },
        )
    }
    
    return (
        <section>
            <h3>Torso Controls</h3>
            <div className={styles.heightText}>Current Torso Height: {round(torsoHeight, PRECISION)}m</div>
            <Slider
                className={styles.slider}
                showLabelOnHover
                min={0}
                max={MAX_HEIGHT}
                step={MAX_HEIGHT / 100}
                precision={PRECISION}
                defaultValue={torsoHeight}
                marks={MARKS} onChangeEnd={handleChange}/>
            <div className={styles.inputBox}>
                <NumberInput
                    placeholder="Torso Height"
                    max={MAX_HEIGHT}
                    min={0}
                    step={MAX_HEIGHT/20}
                    precision={PRECISION}
                    onChange={(val) => setHeight(val || 0)}
                />
                <Button variant="outline" onClick={() => handleChange(height)}>Set</Button>
            </div>
        </section>
    )
}