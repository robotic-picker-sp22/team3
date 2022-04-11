import ROSLIB from "roslib";
import MovementControls from "../MovementControls/MovementControls";
import TorsoControls from "../TorsoControls/TorsoControls";
import HeadControls from "../HeadControls/HeadControls";

type SidePanelProps = {
    ros: ROSLIB.Ros
}
export default function SidePanel({ ros }: SidePanelProps) {
    return (
        <div>
            <HeadControls ros={ros}/>
            <TorsoControls ros={ros}/>
            <MovementControls ros={ros} />
        </div>
    )
}