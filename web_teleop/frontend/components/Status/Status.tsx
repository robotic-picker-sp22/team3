import ROSLIB from "roslib";
import { useTorsoHeight } from "../../utils/RosHooks";

type StatusProps = {
    ros: ROSLIB.Ros
}
export default function Status({ ros }: StatusProps) {
    const torsoHeight = useTorsoHeight(ros)

    return (
        <div>
            Torso Height: {torsoHeight}
        </div>
    )
}