
import { Button } from "@mantine/core"
import styles from "./ObjectListItem.module.css"

type ObjectListItemProps = {
    object: string
    deleteCallback: (object: string) => void
}
export default function ObjectListItem({ object, deleteCallback }: ObjectListItemProps) {
    return (
        <li className={styles.main}>
            <img src={`objects/${object}.png`} alt={object}/>
            <p className={styles.item}>{object.split("_").join(" ")}</p>
            <Button variant="outline" color="red" onClick={() => deleteCallback(object)}>
                X
            </Button>
        </li>
    )
}