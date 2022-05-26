
type ObjectListItemProps = {
    object: string
    deleteCallback: (object: string) => void
}
export default function ObjectListItem({ object, deleteCallback }: ObjectListItemProps) {
    return (
        <li>
            <p>{object}</p>
            <button onClick={() => deleteCallback(object)}>X</button>
        </li>
    )
}