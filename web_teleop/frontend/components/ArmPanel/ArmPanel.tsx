import GripperControls from "../GripperControls/GripperControls";
import ArmControls from "../ArmControls/ArmControls";
type ArmPanelProps = {
    ros: ROSLIB.Ros
}
export default function ArmPanel({ ros }: ArmPanelProps) {

    return (
        <div>
            <ArmControls ros={ros} />
            <GripperControls ros={ros} />
        </div>
    )
}