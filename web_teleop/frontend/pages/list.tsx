import ObjectList from "../components/ObjectList/ObjectList";
import { useRos } from "../utils/RosHooks";



export default function list() {
  const ros = useRos()
    return (
        <ObjectList ros={ros}/>
    )
}