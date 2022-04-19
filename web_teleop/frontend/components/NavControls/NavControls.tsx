import { Button, Input } from "@mantine/core"
import { useState } from "react"
import ROSLIB from "roslib"

type NavControlsProps = {
    ros: ROSLIB.Ros
}

export default function NavControls({ ros }: NavControlsProps) {
    const service = new ROSLIB.Service({
        ros,
        name: "/web_teleop/nav_goal",
        serviceType: "web_teleop/Nav",
    })

    const [name, setName] = useState<string>("")
    const [locations, setLocations] = useState<string[]>([])

    return (
        <div>
            <h3>Nav Controls</h3>
            <Button onClick={() => {
                const req = new ROSLIB.ServiceRequest({ func_name: "get_locations" })
                service.callService(
                    req,
                    (res) => setLocations(res.locations),
                    () => console.log("Failed to get locations")
                )
            }}>Get Locations</Button>
            <Input 
                value={name}
                onChange={(e: any) => {
                    console.log(e.target.value)
                    
                    setName(e.target.value)
                }
                }/>
            <Button onClick={() => {
                const req = new ROSLIB.ServiceRequest({ pose_name: name, func_name: "goto" })
                service.callService(
                    req,
                    () => console.log("Moving to " + name),
                    () => console.log("Failed to send move to " + name)
                )
            }}>Go To</Button>
            <Button onClick={() => {
                const req = new ROSLIB.ServiceRequest({ pose_name: name, func_name: "delete" })
                service.callService(
                    req,
                    () => console.log("Delete to " + name),
                    () => console.log("Failed to send delete to " + name)
                )
            }}>Delete</Button>
            <Button onClick={() => {
                const req = new ROSLIB.ServiceRequest({ pose_name: name, func_name: "save_current_pose" })
                service.callService(
                    req,
                    () => console.log("Saving location " + name),
                    () => console.log("Failed to save location " + name)
                )
            }}>Save Pose</Button>
            <ul>
                {locations.map(location => <li key={location}>{location}</li>)}
            </ul>
        </div>
    )
}