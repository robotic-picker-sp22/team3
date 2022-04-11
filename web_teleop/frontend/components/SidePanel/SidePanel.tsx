import ROSLIB from "roslib";
import MovementControls from "../MovementControls/MovementControls";
import SetTorsoHeight from "../SetTorsoHeight/SetTorsoHeight";
import HeadControls from "../HeadControls/HeadControls";
import GripperControls from "../GripperControls/GripperControls";

type SidePanelProps = {
    ros: ROSLIB.Ros
}
export default function SidePanel({ ros }: SidePanelProps) {
    return (
        <div>
            <HeadControls ros={ros}/>
            <SetTorsoHeight ros={ros}/>
            <GripperControls ros={ros}/>
            <MovementControls ros={ros} />
        </div>
    )
}