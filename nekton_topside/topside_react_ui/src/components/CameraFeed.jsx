import React, { useEffect, useRef } from "react";
import ROSLIB from "roslib";
import { useRos } from "../context/RosProvider";
export default function CameraFeed() {
  const { ros, isConnected } = useRos();
  const canvasRef = useRef(null);
  useEffect(() => {
    if (!ros || !isConnected) return;
    const imageListener = new ROSLIB.Topic({
      ros,
      name: "/nekton/camera_forward",
      messageType: "sensor_msgs/msg/Image",
    });
    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    const handleImage = (image_message) => {
      canvas.width = canvas.parentElement.clientWidth;
      canvas.height = canvas.parentElement.clientHeight;
      // Create ImageData
      const imgData = ctx.createImageData(image_message.width, image_message.height);
      const data = imgData.data;
      const inData = atob(image_message.data);
      let j = 0, i = 0;
      while (j < inData.length) {
        const w1 = inData.charCodeAt(j++);
        const w2 = inData.charCodeAt(j++);
        const w3 = inData.charCodeAt(j++);
        if (!image_message.is_bigendian) {
          data[i++] = w1; // R
          data[i++] = w2; // G
          data[i++] = w3; // B
        } else {
          data[i++] = (w1 >> 8) + ((w1 & 0xff) << 8);
          data[i++] = (w2 >> 8) + ((w2 & 0xff) << 8);
          data[i++] = (w3 >> 8) + ((w3 & 0xff) << 8);
        }
        data[i++] = 255; // alpha
      }
      // Create a temporary canvas to scale ImageData
      const tempCanvas = document.createElement("canvas");
      tempCanvas.width = image_message.width;
      tempCanvas.height = image_message.height;
      const tempCtx = tempCanvas.getContext("2d");
      tempCtx.putImageData(imgData, 0, 0);
      // Draw scaled to main canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.drawImage(tempCanvas, 0, 0, canvas.width, canvas.height);
    };
    imageListener.subscribe(handleImage);
    return () => imageListener.unsubscribe();
  }, [ros, isConnected]);
  return (
    <div style={{ width: "100%", height: "100%" }}>
      <canvas ref={canvasRef} style={{ width: "80%", height: "80%", display: "block" }} />
    </div>
  );
}