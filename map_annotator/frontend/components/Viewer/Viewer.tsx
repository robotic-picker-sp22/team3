import { useEffect, useRef } from "react"
//@ts-ignore
import * as ROS3D from "ros3d"
import ROSLIB from "roslib"

type ViewerProps = {
    ros: ROSLIB.Ros
}
export default function Viewer({ ros }: ViewerProps) {

    const divRef = useRef<HTMLDivElement>(null)

    useEffect(() => {
        if (divRef.current?.children.length == 0) {
            const viewer = new ROS3D.Viewer({
                ros,
                divID: "viewer",
                width: 800,
                height: 600,
                antialias: true,
            })
            
            const tfClient = new ROSLIB.TFClient({
                ros : ros,
                angularThres : 0.01,
                transThres : 0.01,
                rate : 10.0,
                fixedFrame : '/rotating_frame'
            })
    
            const imClient = new ROS3D.InteractiveMarkerClient({
                ros : ros,
                tfClient : tfClient,
                topic : '/basic_controls',
                camera : viewer.camera,
                rootObject : viewer.selectableObjects
            })
        }
        
    }, [])
    
    return (
        <div ref={divRef} id="viewer"></div>
    )
}