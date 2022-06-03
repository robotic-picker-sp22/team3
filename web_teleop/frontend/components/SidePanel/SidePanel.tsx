import ROSLIB from "roslib";
import MovementControls from "../MovementControls/MovementControls";
import TorsoControls from "../TorsoControls/TorsoControls";
import HeadControls from "../HeadControls/HeadControls";
import NavControls from "../NavControls/NavControls";
import { Accordion, Button } from "@mantine/core";
import styles from "./SidePanel.module.css";
import { useRouter } from "next/router";

type SidePanelProps = {
    ros: ROSLIB.Ros
}
export default function SidePanel({ ros }: SidePanelProps) {
    const router = useRouter()
    return (
        <div>
            <Button onClick={() => router.push("/list")}>Go to Picker List Page</Button>
        {/* <Accordion className={styles.sidpanelWidth}> */}
            {/* <Accordion.Item label="Head Controls"> */}
                <HeadControls ros={ros}/>
            {/* </Accordion.Item> */}
            {/* <Accordion.Item label="Torso Controls"> */}
                <TorsoControls ros={ros}/>
            {/* </Accordion.Item> */}
            {/* <Accordion.Item label="Movement Controls"> */}
                <MovementControls ros={ros} />
            {/* </Accordion.Item> */}
            {/* <Accordion.Item label="Nav Controls"> */}
                <NavControls ros={ros}/>
            {/* </Accordion.Item> */}
        {/* </Accordion> */}
        </div>
    )
}