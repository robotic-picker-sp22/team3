import { useEffect, useState } from "react";
import ROSLIB from "roslib"
import { round } from "./helpers";


export function useRos(url: string="ws://localhost:9090", reconnect_ms: number=5000) {
    const [ros, setRos] = useState<ROSLIB.Ros>(new ROSLIB.Ros({url}))

    ros.on('connection', () => {
        console.log("Connected to websocket server")
    })
    ros.on('error', (error) => {
        console.log("Error connecting to websocket", error)
        setTimeout(() => {
            if (!ros.isConnected) {
                ros.close()
                console.log("Reconnecting ROS");
                ros.connect(url)
            }
        }, reconnect_ms)
    })
    ros.on('close', () => {
        console.log("Closed connection to websocket server")
    })

    useEffect(() => {
        return () => ros.close()
    }, [])

    return ros
}

export function useTorsoHeight(ros: ROSLIB.Ros, decimalPlaces: number=3) {
    const [height, setHeight] = useState<number>(0.0)
    const [listener, setListener] = useState(new ROSLIB.Topic({
        ros: ros,
        name: 'joint_state_republisher/torso_lift_joint',
        messageType: 'std_msgs/Float64'
    }))

    useEffect(() => {
        if (!listener.hasListeners()) {
            listener.subscribe((msg: any) => {
                const newHeight = round(msg.data as number, decimalPlaces)
                if (newHeight !== height) {
                    setHeight(newHeight)
                }
            })
            console.log("Subscribed to Torso Height")
        }
        return () => { listener.unsubscribe(() => {console.log("Unsubscribed to TorsoHeight")}) }
    }, [listener])

    return height
}


export function useMovementTopic(ros: ROSLIB.Ros, decimalPlaces: number=3) {
    const [topic, setTopic] = useState(new ROSLIB.Topic({
        ros: ros,
        name: 'cmd_vel',
        messageType: 'geometry_msgs/Twist'
    }))

    useEffect(() => {
    }, [])

    return topic
}