import React, { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import Plotly from "plotly.js-basic-dist";
import { useRos } from "../context/RosProvider";

export default function LaserScanPlot() {
  const { ros, isConnected } = useRos();

  const [scanX, setScanX] = useState([]);
  const [scanY, setScanY] = useState([]);

  const el = useRef(null);

  const FAN_SCALE = 4; // widen the fan visually for horizontal spread

  // ------------------------------------------------------------
  // ROS LASER SCAN SUBSCRIBER
  // ------------------------------------------------------------
  useEffect(() => {
    if (!ros || !isConnected) return;

    const laserScanListener = new ROSLIB.Topic({
      ros,
      name: "/nekton/sonar_forward",
      messageType: "sensor_msgs/LaserScan",
    });

    laserScanListener.subscribe((msg) => {
      const { angle_min, angle_increment, ranges, range_min, range_max } = msg;

      const xArr = [];
      const yArr = [];

      for (let i = 0; i < ranges.length; i++) {
        const r = ranges[i];
        if (r < range_min || r > range_max) continue;

        const angle = angle_min + i * angle_increment;

        // ROS frame -> screen frame
        const x = r * Math.cos(angle);
        const y = r * Math.sin(angle);

        xArr.push(-y);
        yArr.push(x);
      }

      setScanX(xArr);
      setScanY(yArr);
    });

    return () => laserScanListener.unsubscribe();
  }, [ros, isConnected]);

  // ------------------------------------------------------------
  // PLOTLY RENDER
  // ------------------------------------------------------------
  useEffect(() => {
    if (!el.current || scanX.length === 0) return;

    //------------------------------------------------------------
    // COMPUTE ARC SHAPE
    //------------------------------------------------------------
    const maxR = Math.max(...scanX.map((x, i) => Math.hypot(x, scanY[i])));

    // Use hardcoded 36° fan angle for sonar for now
    const deg = (45 * Math.PI) / 180; // Convert to radians
    const minA = -deg;
    const maxA = deg;


    const arcX = [];
    const arcY = [];
    const steps = 200;

    for (let i = 0; i <= steps; i++) {
      const a = minA + (i / steps) * (maxA - minA);
      arcX.push(Math.sin(a) * maxR);
      arcY.push(Math.cos(a) * maxR);
    }

    //------------------------------------------------------------
    // RANGE RINGS (CLIPPED TO FAN)
    //------------------------------------------------------------
    const ringCount = 4;
    const ringTraces = [];
    const labelTraces = [];

    for (let i = 1; i <= ringCount; i++) {
      const r = (i / ringCount) * maxR;

      const ringX = [];
      const ringY = [];
      const steps = 120;

      for (let j = 0; j <= steps; j++) {
        const a = minA + (j / steps) * (maxA - minA);
        ringX.push(Math.sin(a) * r);
        ringY.push(Math.cos(a) * r);
      }

      ringTraces.push({
        type: "scatter",
        mode: "lines",
        x: ringX,
        y: ringY,
        line: { color: "rgba(200,200,200,0.15)", width: 1, dash: "dot" },
        hoverinfo: "skip",
      });

      const labelAngle = maxA;
      labelTraces.push({
        type: "scatter",
        mode: "text",
        x: [Math.sin(labelAngle) * r],
        y: [Math.cos(labelAngle) * r],
        text: [`${r.toFixed(2)} m`],
        textposition: "middle right",
        textfont: { color: "rgba(200,200,200,0.6)", size: 12 },
        hoverinfo: "skip",
      });
    }

    //------------------------------------------------------------
    // ANGULAR WEDGE LINES (radial lines for horizontal spread)
    //------------------------------------------------------------
    const wedgeTraces = [];
    const wedgeCount = 5;

    for (let i = 0; i <= wedgeCount; i++) {
      const a = minA + (i / wedgeCount) * (maxA - minA);
      const endX = Math.sin(a) * maxR;
      const endY = Math.cos(a) * maxR;

      wedgeTraces.push({
        type: "scatter",
        mode: "lines",
        x: [0, endX],
        y: [0, endY],
        line: { color: "rgba(100,150,200,0.2)", width: 1 },
        hoverinfo: "skip",
      });

      // Angle labels
      const labelR = maxR * 1.1;
      const labelX = Math.sin(a) * labelR;
      const labelY = Math.cos(a) * labelR;
      const angleDeg = (a * 180) / Math.PI;

      wedgeTraces.push({
        type: "scatter",
        mode: "text",
        x: [labelX],
        y: [labelY],
        text: [`${angleDeg.toFixed(0)}°`],
        textfont: { color: "rgba(150,150,150,0.6)", size: 10 },
        hoverinfo: "skip",
      });
    }

    //------------------------------------------------------------
    // LAYERS
    //------------------------------------------------------------
    const glowLayer = {
      type: "scatter",
      mode: "markers",
      x: scanX,
      y: scanY,
      marker: { size: 6, color: "rgba(0, 60, 255, 0.10)" },
      hoverinfo: "skip",
    };

    const coreLayer = {
      type: "scatter",
      mode: "markers",
      x: scanX,
      y: scanY,
      marker: { size: 2, color: "rgba(0, 120, 255, 1.0)" },
      hoverinfo: "skip",
    };

    const arcOutline = {
      type: "scatter",
      mode: "lines",
      x: arcX,
      y: arcY,
      line: { color: "rgba(200,200,200,0.25)", width: 2 },
      hoverinfo: "skip",
    };

    const leftEdge = {
      type: "scatter",
      mode: "lines",
      x: [0, arcX[0]],
      y: [0, arcY[0]],
      line: { color: "rgba(200,200,200,0.25)", width: 2 },
      hoverinfo: "skip",
    };

    const rightEdge = {
      type: "scatter",
      mode: "lines",
      x: [0, arcX.at(-1)],
      y: [0, arcY.at(-1)],
      line: { color: "rgba(200,200,200,0.25)", width: 2 },
      hoverinfo: "skip",
    };

    //------------------------------------------------------------
    // FINAL PLOT
    //------------------------------------------------------------
    const data = [
      ...ringTraces,
      ...labelTraces,
      ...wedgeTraces,
      arcOutline,
      leftEdge,
      rightEdge,
      glowLayer,
      coreLayer,
    ];

    const layout = {
      plot_bgcolor: "black",
      paper_bgcolor: "black",
      margin: { t: 90, b: 90, l: 10, r: 10 },
      xaxis: { range: [-maxR, maxR], visible: false },
      yaxis: { range: [0, maxR], visible: false },
      showlegend: false,
      height: 500,
      width: 500,
    };

    Plotly.react(el.current, data, layout, { responsive: true });

    return () => {
      try { Plotly.purge(el.current); } catch {}
    };
  }, [scanX, scanY]);

  return (
    <div ref={el} id="laser-scan-plot" style={{ width: "100%", height: "100%" }} />
  );
}
