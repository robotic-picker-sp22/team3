import ROSLIB from "roslib";
import MovementControls from "../MovementControls/MovementControls";
import TorsoControls from "../TorsoControls/TorsoControls";
import HeadControls from "../HeadControls/HeadControls";
import NavControls from "../NavControls/NavControls";
import { Accordion } from "@mantine/core";
import styles from "./SidePanel.module.css";

type SidePanelProps = {
    ros: ROSLIB.Ros
}
export default function SidePanel({ ros }: SidePanelProps) {
    return (
        <Accordion className={styles.sidpanelWidth}>
            <Accordion.Item label="Head Controls"><HeadControls ros={ros}/></Accordion.Item>
            <Accordion.Item label="Torso Controls"><TorsoControls ros={ros}/></Accordion.Item>
            <Accordion.Item label="Movement Controls"><MovementControls ros={ros} /></Accordion.Item>
            <Accordion.Item label="Nav Controls"><NavControls ros={ros}/></Accordion.Item>
        </Accordion>
    )
}