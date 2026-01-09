import React, { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import { useRos } from "../context/RosProvider";

export default function ArtificialHorizonCanvas() {
  const { ros, isConnected } = useRos();

  const canvasRef = useRef(null);

  const [imuData, setImuData] = useState({
    roll: 0,
    pitch: 0,
    yaw: 0,
    angular_velocity: { x: 0, y: 0, z: 0 },
    linear_acceleration: { x: 0, y: 0, z: 0 },
  });

  // Function to convert quaternion to Euler angles
  function quaternionToEuler(x, y, z, w) {
    const roll = Math.atan2(
      2.0 * (w * x + y * z),
      1.0 - 2.0 * (x * x + y * y)
    );

    const pitch = Math.asin(
      Math.max(-1, Math.min(1, 2.0 * (w * y - z * x)))
    );

    const yaw = Math.atan2(
      2.0 * (w * z + x * y),
      1.0 - 2.0 * (y * y + z * z)
    );

    return { roll, pitch, yaw };
  }

  // Function to draw submarine artificial horizon
  function drawSubmarineArtificialHorizon(roll = 0, pitch = 0) {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext("2d");

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    ctx.save();
    ctx.translate(canvas.width / 2, canvas.height / 2);
    ctx.rotate(-roll);

    // Grid
    ctx.strokeStyle = "lightblue";
    ctx.lineWidth = 1;
    for (let i = -2; i <= 2; i++) {
      ctx.beginPath();
      ctx.moveTo(-50, i * 20);
      ctx.lineTo(50, i * 20);
      ctx.stroke();
    }

    // Crosshair
    ctx.strokeStyle = "red";
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.moveTo(-10, pitch * 10);
    ctx.lineTo(10, pitch * 10);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(0, -10 + pitch * 10);
    ctx.lineTo(0, 10 + pitch * 10);
    ctx.stroke();

    ctx.restore();
  }

  useEffect(() => {
    if (!ros || !isConnected) return;

    // Subscribing to IMU data
    const imuListener = new ROSLIB.Topic({
      ros,
      name: "/nekton/imu",
      messageType: "sensor_msgs/Imu",
    });

    // Update IMU listener to draw the artificial horizon
    imuListener.subscribe((imuMessage) => {
      console.log("Raw IMU message:", imuMessage);
      const { x, y, z, w } = imuMessage.orientation;
      const { roll, pitch, yaw } = quaternionToEuler(x, y, z, w);

      setImuData({
        roll,
        pitch,
        yaw,
        angular_velocity: imuMessage.angular_velocity,
        linear_acceleration: imuMessage.linear_acceleration,
      });

      drawSubmarineArtificialHorizon(roll, pitch);
    });

    return () => imuListener.unsubscribe();
  }, [ros, isConnected]);

  // Initial draw
  useEffect(() => {
    drawSubmarineArtificialHorizon(0, 0);
  }, []);

return (
  <div
    style={{
      display: "flex",
      flexDirection: "column",
      // alignItems: "flex-start",
      // alignItems: "stretch",
      justifyContent: "flex-start",
      height: "100%",
      width: "100%",
      gap: "16px",
      padding: "10px",
      background: "#111",
    }}
  >
    {/* Artificial Horizon */}
    <canvas
      ref={canvasRef}
      width={220}
      height={220}
      style={{
        alignSelf: "flex-start",
        justifyContent: "flex-start",
        border: "2px solid #ddd",
        borderRadius: "8px",
        background: "black",
      }}
    />

    <div
      style={{
        display: "flex",
        flexDirection: "column",
        height: "50%",
        alignSelf: "flex-start",
        justifyContent: "flex-start",
        color: "white",
        fontFamily: "monospace",
        fontSize: "13px",
        lineHeight: "1.4",
        // minWidth: "170px",
        width: "220px",
        textAlign: "center",
        overflow: "hidden",
      }}
    >

    {/* Update IMU data text with negative values */}
    <p style={{ margin: "0 0 8px", fontWeight: "bold" }}>Orientation</p>

    <p style={{ margin: "0 0 4px" }}>
      Roll: {(-imuData.roll * 180 / Math.PI).toFixed(2)}°
    </p>
    <p style={{ margin: "0 0 4px" }}>
      Pitch: {(-imuData.pitch * 180 / Math.PI).toFixed(2)}°
    </p>
    <p style={{ margin: "0 0 10px" }}>
      Yaw: {(-imuData.yaw * 180 / Math.PI).toFixed(2)}°
    </p>

    <p style={{ margin: "0 0 8px", fontWeight: "bold" }}>
      Angular Velocity
    </p>

    <p style={{ margin: "0 0 4px" }}>
      X: {imuData.angular_velocity.x.toFixed(3)} rad/s
    </p>
    <p style={{ margin: "0 0 4px" }}>
      Y: {imuData.angular_velocity.y.toFixed(3)} rad/s
    </p>
    <p style={{ margin: "0 0 10px" }}>
      Z: {imuData.angular_velocity.z.toFixed(3)} rad/s
    </p>

    <p style={{ margin: "0 0 8px", fontWeight: "bold" }}>
      Linear Acceleration
    </p>

    <p style={{ margin: "0 0 4px" }}>
      X: {imuData.linear_acceleration.x.toFixed(3)} m/s²
    </p>
    <p style={{ margin: "0 0 4px" }}>
      Y: {imuData.linear_acceleration.y.toFixed(3)} m/s²
    </p>
    <p style={{ margin: 0 }}>
      Z: {imuData.linear_acceleration.z.toFixed(3)} m/s²
    </p>
    </div>
  </div>
);

}
