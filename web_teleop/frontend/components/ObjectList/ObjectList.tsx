import { useState } from "react"
import ROSLIB from "roslib"
import { Autocomplete } from '@mantine/core';
import { objectList } from '../../utils/constants'
import ObjectListItem from "../ObjectListItem/ObjectListItem";
import { formList, useForm } from "@mantine/form";

type ObjectListProps = {
    ros: ROSLIB.Ros
}
const SERVICE_NAME = "/web_teleop/set_object_list"
const SERVICE_TYPE = "/web_teleop/SetObjectList"

export default function ObjectList({ ros }: ObjectListProps) {
    const [inputText, setInputText] = useState<string>("")
    const form = useForm({
        initialValues: {
            objects: formList<string>([])
        }
    })

    const service = new ROSLIB.Service({
        ros,
        name: SERVICE_NAME,
        serviceType: SERVICE_TYPE,
    })
    return (
        <div>
            <ul>
                {form.values.objects.map((object, i) => (
                    <ObjectListItem key={`${object}-${i}`} object={object} deleteCallback={() => form.removeListItem('objects', i)}/>
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
                    form.addListItem("objects", inputText)
                }
            }}>Add Item</button>
            <button onClick={() => {
                const request = new ROSLIB.ServiceRequest({ objects: form.values.objects })
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