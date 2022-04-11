import ROSLIB from "roslib";
import MovementControls from "../MovementControls/MovementControls";
import SetTorsoHeight from "../SetTorsoHeight/SetTorsoHeight";

type SidePanelProps = {
    ros: ROSLIB.Ros
}
export default function SidePanel({ ros }: SidePanelProps) {
    return (
        <div>
            <SetTorsoHeight ros={ros}/>
            <MovementControls ros={ros} />
        </div>
    )
}