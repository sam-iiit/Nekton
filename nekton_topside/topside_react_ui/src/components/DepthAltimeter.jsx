import React, { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import { useRos } from "../context/RosProvider";

const MAX_POINTS = 200;

export default function DepthAltimeter() {
  const { ros, isConnected } = useRos();

  const depthCanvasRef = useRef(null);
  const altitudeCanvasRef = useRef(null);

  const [depth, setDepth] = useState(0);
  const [altitude, setAltitude] = useState(2.5);

  const depthHistory = useRef([]);
  const altitudeHistory = useRef([]);

  useEffect(() => {
    if (!ros || !isConnected) return;

    // Subscribing to a Topic
    const pressureListener = new ROSLIB.Topic({
      ros: ros,
      name: "/nekton/pressure",
      messageType: "sensor_msgs/msg/FluidPressure",
    });
    pressureListener.subscribe(function (message) {
      console.log("Received message on " + pressureListener.name + ": " + message.fluid_pressure);
      const x = message.fluid_pressure / 10051.8; //not sure which formula is correct
      // const x = message.fluid_pressure / (1000 * 9.81);
      const myElement = document.getElementById('my-element');
      myElement.innerHTML = "Depth: " + x;
      console.log("Depth: " + x);
      // setDepth(depth);
      // listener.unsubscribe();

    });

    const odomListener = new ROSLIB.Topic({
      ros,
      name: "/nekton/odometry",
      messageType: "nav_msgs/msg/Odometry",
    });

    odomListener.subscribe((msg) => {
      const z = msg.pose.pose.position.z;
      // Using fake values for now
      const simDepth = z;
      const simAltitude = Math.max(0, 2.5 - z);
      setDepth(simDepth);
      setAltitude(simAltitude);

      depthHistory.current.push(simDepth);
      altitudeHistory.current.push(simAltitude);

      if (depthHistory.current.length > MAX_POINTS) {
        depthHistory.current.shift();
        altitudeHistory.current.shift();
      }

      drawGraph(depthCanvasRef.current, depthHistory.current, "#4da3ff");
      drawGraph(altitudeCanvasRef.current, altitudeHistory.current, "#2ecc71");
    });

    return () => odomListener.unsubscribe();
  }, [ros, isConnected]);

  const drawGraph = (canvas, data, color) => {
    if (!canvas) return;

    const ctx = canvas.getContext("2d");
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    // background
    ctx.fillStyle = "#0b1220";
    ctx.fillRect(0, 0, w, h);

    if (data.length < 2) return;

    const min = Math.min(...data);
    const max = Math.max(...data);
    const range = max - min || 1;

    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();

    data.forEach((v, i) => {
      const x = (i / (MAX_POINTS - 1)) * w;
      const y = h - ((v - min) / range) * h;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });

    ctx.stroke();
  };

  return (
    <div
      style={{
        display: "grid",
        gridTemplateColumns: "1fr 1fr",
        gap: "12px",
        width: "100%",
      }}
    >
      {/* DEPTH */}
      <div style={{display: "flex", flexDirection: "column"}}>
        <div style={{ fontWeight: "bold", marginBottom: 4 }}>Depth</div>
        <div style={{ marginBottom: 6 }}>{depth.toFixed(2)} m</div>
        <canvas
          ref={depthCanvasRef}
          width={300}
          height={120}
          style={{
            width: "100%",
            height: "120px",
            border: "1px solid #333",
            background: "#0b1220",
          }}
        />
      </div>

      {/* ALTITUDE */}
      <div style={{display: "flex", flexDirection: "column"}}>
        <div style={{ fontWeight: "bold", marginBottom: 4 }}>Altitude</div>
        <div style={{ marginBottom: 6 }}>{altitude.toFixed(2)} m</div>
        <canvas
          ref={altitudeCanvasRef}
          width={300}
          height={120}
          style={{
            width: "100%",
            height: "120px",
            border: "1px solid #333",
            background: "#0b1220",
          }}
        />
      </div>
    </div>
  );
}
