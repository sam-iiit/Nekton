import React, { useState, useEffect, useRef } from 'react'
import ROSLIB from "roslib";
import Plotly from 'plotly.js-basic-dist'
import { useRos } from '../context/RosProvider';

export default function LaserScanPlot() {
  const {ros, isConnected} = useRos();
  const [scanX, setScanX] = useState([]);
  const [scanY, setScanY] = useState([]);
  
  useEffect(() => {
    console.log(ros);
    if (!ros || !isConnected) return;

    const laserScanListener = new ROSLIB.Topic({
      ros: ros,
      name: "/nekton/sonar_forward",
      messageType: "sensor_msgs/LaserScan",
    });

    laserScanListener.subscribe(function(laserScanMessage) {
      //Get angle info
      const angleMin = laserScanMessage.angle_min;
      const angleMax = laserScanMessage.angle_max;
      const angleIncrement = laserScanMessage.angle_increment;
      //Get range info
      const ranges = laserScanMessage.ranges;
      const rangeMin = laserScanMessage.range_min;
      const rangeMax = laserScanMessage.range_max;
      let xArr = [];
      let yArr = [];
      for (let i = 0; i < ranges.length; i++) {
        //Get current range to work with
        const range = ranges[i];
        //Filter out invalid ranges
        if (range >= rangeMin && range <= rangeMax) {
          const angle = angleMin + i * angleIncrement;
          const xVal = range * Math.cos(angle);
          const yVal = range * Math.sin(angle);
          xArr.push(-yVal);
          yArr.push(xVal);
        }
      }
      console.log("X points: ", -yArr);
      console.log("Y points: ", xArr);
      setScanX(xArr);
      setScanY(yArr);
  }); 
  return () => {
      laserScanListener.unsubscribe();
    }
  }, [ros, isConnected])



  const el = useRef(null)

  useEffect(() => {
    if (!el.current) return

    const glowLayer = {
      type: "scatter",
      mode: "markers",
      x: scanX,
      y: scanY,
      marker: {
        size: 6,
        color: "rgba(0, 60, 255, 0.10)", 
      },
      hoverinfo: "skip"
    }

    const coreLayer = {
      type: "scatter",
      mode: "markers",
      x: scanX,
      y: scanY,
      marker: {
        size: 2,
        color: "rgba(0, 120, 255, 1.0)"  
      }
    }

    const data = [glowLayer, coreLayer]

    // Laout
    const layout = {
      plot_bgcolor: 'black',
      paper_bgcolor: 'black',
      title: {
        text: '2D Laser Scan Visualization',
        font: { family: 'Courier New, monospace', size: 20, color: 'white' },
        x: 0.5,
      },
      xaxis: {
        tickfont: { color: 'white' },
        range: [0, 2],
        autorange: false,
      },
      yaxis: {
        tickfont: { color: 'white' },
        range: [2, 10],
        autorange: false,
      },
      margin: { t: 90, b: 30, l: 10, r: 10 },
      height: 500,
      width: 500,
      showlegend: false,
    }

    Plotly.newPlot(el.current, data, layout, { responsive: true })
    // Plotly.react(el.current, data, layout, { responsive: true })

    return () => {
      try { Plotly.purge(el.current) } catch {}
    }
  }, [scanX, scanY])

  return (
    <div
      ref={el}
      id="laser-scan-plot"
      style={{ width: '100%', height: '100%' }}
    />
  )
}
