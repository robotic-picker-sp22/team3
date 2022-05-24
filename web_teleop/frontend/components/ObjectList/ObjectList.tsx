import { useState } from "react"
import ROSLIB from "roslib"


type ObjectListProps = {
    ros: ROSLIB.Ros
}
const SERVICE_NAME = "/web_teleop/set_object_list"
const SERVICE_TYPE = "/web_teleop/SetObjectList"

export default function ObjectList({ ros }: ObjectListProps) {
    const [objects, setObjects] = useState<string[]>([])
    const [inputText, setInputText] = useState<string>("")

    const service = new ROSLIB.Service({
        ros,
        name: SERVICE_NAME,
        serviceType: SERVICE_TYPE,
    })
    return (
        <div>
            <ul>
                {objects.map((object, i) => <li key={`${object}-${i}`}>{object}</li>)}
            </ul>
            <input type="text" onChange={e => {
                setInputText(e.currentTarget.value || "")
            }}
                value={inputText}
            />
            <button onClick={() => {
                setObjects(old => {
                    return [...old, inputText]
                })
                setInputText("")
            }}>Add Item</button>
            <button onClick={() => {

                const request = new ROSLIB.ServiceRequest({ objects })
                service.callService(request, (e) => {
                    console.log("Recieved Response", e);
                },
                (e) => {
                    console.log("Failed", e);
                    
                })
                
            }}>Send List</button>
        </div>
    )
}