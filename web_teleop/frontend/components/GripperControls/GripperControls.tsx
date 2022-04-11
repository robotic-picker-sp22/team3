import { Switch } from "@mantine/core"
import { useEffect, useState } from "react"
import ROSLIB from "roslib"

type GripperControlsProps = {
    ros: ROSLIB.Ros
}
const CLOSED_EFFORT = 100 // Newtons
const OPEN_EFFORT = 0 // Newtons

export default function GripperControls({ ros }: GripperControlsProps) {
    const service = new ROSLIB.Service({
        ros,
        name: "/web_teleop/set_gripper",
        serviceType: "web_teleop/SetGripper",
    })
    const [closed, setClosed] = useState(false)

    useEffect(() => {
        console.log(`Trying to ${closed ? "close" : "open"} gripper`)
        const request = new ROSLIB.ServiceRequest({ close: closed ? CLOSED_EFFORT : OPEN_EFFORT })
        service.callService(
            request,
            () => { console.log(`Finished ${closed ? "clos" : "open"}ing gripper`) },
            (e) => { console.log(`Failed to ${closed ? "close" : "open"} gripper`, e) },
        )
    }, [closed])

    return (
        <div>
            <h3>Gripper Controls</h3>
            <Switch
                checked={closed}
                onLabel="Closed"
                offLabel="Open"
                label="Gripper"
                onChange={(e) => setClosed(e.currentTarget.checked)}
                size="xl"
            />
        </div>
        
    )
}