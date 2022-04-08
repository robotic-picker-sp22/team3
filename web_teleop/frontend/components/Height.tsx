import { useEffect, useState } from "react"

import ROSLIB from "roslib"

export default function Height() {

    const [height, setHeight] = useState(0.0)

    const handleMessage = function(message: any) {
        setHeight(message.data)
    }
    useEffect(() => {
        let ros = new ROSLIB.Ros({
            url: "ws://localhost:9090"
        })
        ros.on('connection', () => {
            console.log("Connected to websocket server")
        })
        ros.on('error', (error) => {
            console.log("Error connecting to websocket", error)
        })
        ros.on('close', () => {
            console.log("Closed connection to websocket server")
        })
        let listener = new ROSLIB.Topic({
            ros: ros,
            name: 'joint_state_republisher/torso_lift_joint',
            messageType: 'std_msgs/Float64'
        })
        listener.subscribe(handleMessage)
    }, [])

    return <p>{height}</p>
}