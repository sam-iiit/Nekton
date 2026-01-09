import React, { useEffect, useRef, useState } from "react";
import { useRos } from '../context/RosProvider';

export default function SimControls({ onStatus }) {
  const { ros, isConnected } = useRos();
  const servicesRef = useRef({ pause: null, unpause: null });
  const [ready, setReady] = useState(false);

  useEffect(() => {
    if (!ros || !isConnected) {
      setReady(false);
      return;
    }

    let canceled = false;

    try {
      const pauseService = new ros.Service({
        ros,
        name: "/pause_physics",
        serviceType: "std_srvs/srv/Empty",
      });

      const unpauseService = new ros.Service({
        ros,
        name: "/unpause_physics",
        serviceType: "std_srvs/srv/Empty",
      });

      if (!canceled) {
        servicesRef.current = { pause: pauseService, unpause: unpauseService };
        setReady(true);
        onStatus?.("SimControls services created");
      }
    } catch (err) {
      console.error("Failed to create ROS services:", err);
      onStatus?.(`Failed to create services: ${err?.message ?? err}`);
      setReady(false);
    }

    return () => {
      canceled = true;
      servicesRef.current = { pause: null, unpause: null };
      setReady(false);
    };
  }, [ros, isConnected, onStatus]);

  const callService = (srv) => {
    if (!srv) return onStatus?.("Service not initialized");

    try {
      srv.callService(
        new ros.ServiceRequest({}),
        () => onStatus?.("Service call completed"),
        (err) => {
          console.error("Service call error:", err);
          onStatus?.(`Service call error: ${err?.message ?? err}`);
        }
      );
    } catch (err) {
      console.error("Service call failed:", err);
      onStatus?.(`Service call failed: ${err?.message ?? err}`);
    }
  };

  return (
    <div className="card">
      <div className="header">
        <h3>Nekton Simulation</h3>
        <span className={`status ${ready ? "ok" : "warn"}`}>{ready ? "Connected" : "Not Ready"}</span>
      </div>
      <p className="subtitle">Pause or resume physics in the simulator</p>
      <div className="buttons">
        <button className="btn pause" onClick={() => callService(servicesRef.current.pause)} disabled={!ready}>
          ⏸ Pause
        </button>
        <button className="btn play" onClick={() => callService(servicesRef.current.unpause)} disabled={!ready}>
          ▶ Unpause
        </button>
      </div>
      

      <style jsx>{`
        .card {
          min-width: 260px;
          padding: 16px 18px 18px;
          border-radius: 16px;
          background: linear-gradient(180deg, #ffffff, #f6f7f9);
          box-shadow: 0 10px 25px rgba(0, 0, 0, 0.08);
          font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
        }

        .header {
          display: flex;
          align-items: center;
          justify-content: space-between;
        }

        h3 {
          margin: 0;
          font-size: 16px;
          font-weight: 600;
        }

        .status {
          font-size: 11px;
          padding: 4px 8px;
          border-radius: 999px;
          font-weight: 500;
        }

        .status.ok {
          background: #e6f7ec;
          color: #1f7a3f;
        }

        .status.warn {
          background: #fff2e0;
          color: #8a4b00;
        }

        .subtitle {
          margin: 6px 0 14px;
          font-size: 12px;
          color: #555;
        }

        .buttons {
          display: flex;
          gap: 10px;
        }

        .btn {
          flex: 1;
          padding: 10px 0;
          border-radius: 10px;
          border: none;
          font-size: 13px;
          font-weight: 500;
          cursor: pointer;
          transition: transform 0.05s ease, box-shadow 0.15s ease, opacity 0.15s ease;
        }

        .btn:active:not(:disabled) {
          transform: translateY(1px);
        }

        .btn:disabled {
          opacity: 0.5;
          cursor: default;
        }

        .btn.pause {
          background: #ffecec;
          color: #a30000;
        }

        .btn.pause:hover:not(:disabled) {
          box-shadow: 0 6px 16px rgba(163, 0, 0, 0.2);
        }

        .btn.play {
          background: #e8f3ff;
          color: #004b9a;
        }

        .btn.play:hover:not(:disabled) {
          box-shadow: 0 6px 16px rgba(0, 75, 154, 0.25);
        }
      `}</style>
    </div>

  );
}
