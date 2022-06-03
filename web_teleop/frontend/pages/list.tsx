import Head from "next/head";
import ObjectList from "../components/ObjectList/ObjectList";
import { useRos } from "../utils/RosHooks";



export default function list() {
  const ros = useRos()
    return (
      <>
        <Head>
          <title>Robot Pick List</title>
        </Head>
        <ObjectList ros={ros}/>
      </>
    )
}