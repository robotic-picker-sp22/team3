import { useState } from "react"
import ROSLIB from "roslib"
import { Autocomplete } from '@mantine/core';
import { objectList } from '../../utils/constants'
import ObjectListItem from "../ObjectListItem/ObjectListItem";

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

    const createDeleteCallback = (i: number) => {
        return (objectName: string) => {
            setObjects(oldList => {
                if (oldList[i] == objectName) {
                    return oldList.splice(i, 1)
                } else {
                    console.error(`Could not remove item: ${objectName} at index ${i}`)
                    return oldList
                }
            }
            )
        }
    }
    return (
        <div>
            <ul>
                {objects.map((object, i) => (
                    <ObjectListItem key={`${object}-${i}`} object={object} deleteCallback={createDeleteCallback(i)}/>
                ))}
            </ul>
            <input type="text" onChange={e => {
                setInputText(e.currentTarget.value || "")
            }}
                value={inputText}
            />
            <Autocomplete
                onChange={setInputText}
                value={inputText}
                data={objectList}
                error={!objectList.includes(inputText)}
            />
            <button onClick={() => {
                if (objectList.includes(inputText)) {
                    setObjects(old => {
                        return [...old, inputText]
                    })
                    setInputText("")
                }
            }}>Add Item</button>
            <button onClick={() => {

                const request = new ROSLIB.ServiceRequest({ objects })
                service.callService(request, (e) => {
                    console.log("Received Response", e);
                },
                (e) => {
                    console.log("Failed", e);
                    
                })
                
            }}>Send List</button>
        </div>
    )
}