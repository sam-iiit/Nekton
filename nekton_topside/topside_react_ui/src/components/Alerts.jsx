import React, { useState, useEffect } from "react";

const boxStyle = {
  display: "flex",
  flexWrap: "wrap",
  justifyContent: "center",
  alignItems: "center",
  padding: 10,
};

const alertStyle = {
  textAlign: "center",
  height: 50,
  width: 150,
  margin: 10,
  padding: 5,
  borderRadius: 15,
  color: "white",
  fontWeight: "bold",
  lineHeight: "50px",
};

const buttonStyle = {
  height: 50,
  margin: 10,
  padding: "0 20px",
};

export default function Alerts({
  alerts = {},
  onTest = null,
  className = "",
  style = {},
}) {
  const defaults = {
    overheated: false,
    bell: false,
    lowBattery: false,
    leakSensor: false,
    alarm: false,
  };

  const [state, setState] = useState(() => ({ ...defaults, ...alerts }));
 const [testMode, setTestMode] = useState(false);


  // Only update local state when incoming alerts actually change values.
useEffect(() => {
  if (testMode) return; // 🚫 do not overwrite test state

  const merged = { ...defaults, ...alerts };
  setState(merged);
  // eslint-disable-next-line react-hooks/exhaustive-deps
}, [alerts, testMode]);


const handleTest = () => {
  const exampleMessage = {
    overheated: true,
    bell: false,
    lowBattery: true,
    leakSensor: false,
    alarm: true,
  };

  setTestMode(true);
  setState(exampleMessage);
};


  const renderAlert = (id, label) => (
    <div
      key={id}
      id={id}
      style={{
        ...alertStyle,
        backgroundColor: state[id] ? "red" : "green",
      }}
    >
      {label}
    </div>
  );

  return (
    <div className={className} style={{ ...style }}>
      <div className="alertBox" style={{ display: "block" }}>
        <div className="alertBoxGroup" style={boxStyle}>
          {renderAlert("overheated", "Overheated")}
          {renderAlert("bell", "Bell")}
          {renderAlert("lowBattery", "Low Battery")}
          {renderAlert("leakSensor", "Leak Sensor")}
          {renderAlert("alarm", "Alarm")}
          <button id="testButton" type="button" style={buttonStyle} onClick={handleTest}>
            Test Button
          </button>
        </div>
      </div>
    </div>
  );
}