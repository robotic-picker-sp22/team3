import { useEffect, useState } from "react";
import ROSLIB, { Topic } from "roslib"


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

export function useTopicSubscriber(rosTopic: ROSLIB.Topic) {
    const [msg, setMsg] = useState(new ROSLIB.Message(null))
    useEffect(() => {
        rosTopic.subscribe(setMsg)
        return rosTopic.unsubscribe
    }, [])

    return msg
}