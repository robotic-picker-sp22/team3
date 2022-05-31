
import styles from "./ObjectListItem.module.css"

type ObjectListItemProps = {
    object: string
    deleteCallback: (object: string) => void
}
export default function ObjectListItem({ object, deleteCallback }: ObjectListItemProps) {
    return (
        <li className={styles.main}>
            <p className={styles.item}>{object}</p>
            <button className={styles.button} onClick={() => deleteCallback(object)}>X</button>
        </li>
    )
}