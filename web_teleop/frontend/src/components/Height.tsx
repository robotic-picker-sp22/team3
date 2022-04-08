import { render } from "@testing-library/react"
import React from "react"

import ROSLIB from "roslib"
import 

type HeightState = {
    height: number
}

export default class Height extends React.Component<{}, HeightState> {

    state: HeightState = {
        height: 0
    }

    componentDidMount() {
        let ros = new ROSLIB.Ros({
            url: "ws://localhost:9090"
        })
        let listener = new ROSLIB.Topic({
            ros: ros,
            name: 'joint_state_republisher/torso_lift_joint',
            messageType: 'Float64'
        })
        listener.subscribe(this.handleMessage.bind(this))
    }

    handleMessage(message: ROSLIB.Message) {
        this.setState({
            height: message.values.height
        })
    }

    render() {
        return (
            <div>
                <p>current height: {this.state.height}</p>
            </div>
        )
    }
}