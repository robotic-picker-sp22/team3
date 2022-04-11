import { useEffect, useState } from "react"

import ROSLIB from "roslib"

const PRECISION = 0.0001

export type HeightProps = {
    ros: ROSLIB.Ros
}

export default function Height({ ros }: HeightProps) {

    const [height, setHeight] = useState(0.0)

    const handleMessage = function(message: any) {
        let newHeight: number = message.data as number
        newHeight = Math.round(newHeight/PRECISION) * PRECISION
        setHeight(newHeight)
    }
    useEffect(() => {
        let listener = new ROSLIB.Topic({
            ros: ros,
            name: 'joint_state_republisher/torso_lift_joint',
            messageType: 'std_msgs/Float64'
        })
        listener.subscribe(handleMessage)
    }, [])

    return <span>{height}</span>
}