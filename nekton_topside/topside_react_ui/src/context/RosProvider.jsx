import {useState, useEffect, createContext, useContext } from 'react';
import ROSLIB, { Ros } from "roslib";

const RosContext = createContext(null);

export const RosProvider = ({children, url="ws://localhost:9090"}) => {
    const [ros, setRosInstance] = useState(null);
    const [isConnected, setIsConnected] = useState(false);

    useEffect(() => {
    // Connecting to ROS
    const newRos = new ROSLIB.Ros({
    url,
    });
    newRos.on("connection", function () {
    console.log("Connected to websocket server.");
    setIsConnected(true);
    });
    newRos.on("error", function (error) {
    console.log("Error connecting to websocket server: ", error);
    setIsConnected(false);
    });
    newRos.on("close", function () {
    console.log("Connection to websocket server closed.");
    setIsConnected(false);
    });

    setRosInstance(newRos);

    return () => {
    newRos.close();
    }
    }, [url]);

    return (
        <RosContext.Provider value={{ ros, isConnected }}>
            {children}
        </RosContext.Provider>
    );
};
export const useRos = () => useContext(RosContext);