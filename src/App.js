import { useState, useEffect, useRef, useCallback } from "react";

const WS_URL = "ws://localhost:9090";
const LINEAR_SPEED = 0.6;
const ANGULAR_SPEED = 1.0;

export default function App() {
  const ws = useRef(null);
  const [connected, setConnected] = useState(false);
  const [phase, setPhase] = useState("manual");
  const [mapData, setMapData] = useState(null);
  const [robotPos, setRobotPos] = useState(null);
  const [goalClick, setGoalClick] = useState(null);
  const [pathData, setPathData] = useState(null);
  const [status, setStatus] = useState("Connecting...");
  
  const canvasRef = useRef(null);
  const pressedKeys = useRef(new Set());
  const cmdInterval = useRef(null);

  // NEW: A reference to the current phase so the WebSocket callback can always see the latest state
  const phaseRef = useRef(phase);
  
  // Keep the ref updated whenever the phase state changes
  useEffect(() => {
    phaseRef.current = phase;
  }, [phase]);

  const send = useCallback((msg) => {
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      ws.current.send(JSON.stringify(msg));
    }
  }, []);

  const publishTwist = useCallback((linear, angular) => {
    send({
      op: "publish",
      topic: "/diff_drive_controller/cmd_vel",
      type: "geometry_msgs/TwistStamped", 
      msg: {
        header: { frame_id: "base_link" },
        twist: {
          linear: { x: linear, y: 0.0, z: 0.0 },
          angular: { x: 0.0, y: 0.0, z: angular },
        },
      },
    });
  }, [send]);

  const stopRobot = useCallback(() => publishTwist(0.0, 0.0), [publishTwist]);

  useEffect(() => {
    const connect = () => {
      const socket = new WebSocket(WS_URL);
      ws.current = socket;
      
      socket.onopen = () => {
        setConnected(true);
        setStatus("Connected to ROS");
        send({ op: "subscribe", topic: "/map", type: "nav_msgs/OccupancyGrid" });
        send({ op: "subscribe", topic: "/slam_pose", type: "nav_msgs/Odometry" });
        send({ op: "subscribe", topic: "/planned_path", type: "nav_msgs/Path" });
        send({ op: "advertise", topic: "/ui_goal", type: "geometry_msgs/Point" });
      };
      
      socket.onmessage = (e) => {
        const data = JSON.parse(e.data);
        
        // FIXED: Only update the map data if we are currently in manual mapping mode
        if (data.topic === "/map") {
          if (phaseRef.current === "manual") {
             setMapData(data.msg);
          }
        }
        
        if (data.topic === "/slam_pose") {
          const p = data.msg.pose.pose.position;
          setRobotPos({ x: p.x, y: p.y });
        }
        
        if (data.topic === "/planned_path") {
          setPathData(data.msg.poses);
        }
      };
      
      socket.onclose = () => {
        setConnected(false);
        setStatus("Disconnected — retrying...");
        setTimeout(connect, 2000);
      };
      
      socket.onerror = () => socket.close();
    };
    
    connect();
    return () => { ws.current?.close(); clearInterval(cmdInterval.current); };
  }, [send]);

  useEffect(() => {
    if (phase !== "manual") { clearInterval(cmdInterval.current); return; }

    const getVel = () => {
      const k = pressedKeys.current;
      let lin = 0, ang = 0;
      if (k.has("ArrowUp") || k.has("w")) lin = LINEAR_SPEED;
      if (k.has("ArrowDown") || k.has("s")) lin = -LINEAR_SPEED;
      if (k.has("ArrowLeft") || k.has("a")) ang = ANGULAR_SPEED;
      if (k.has("ArrowRight") || k.has("d")) ang = -ANGULAR_SPEED;
      return { lin, ang };
    };

    const onKeyDown = (e) => {
      if (!["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "w", "a", "s", "d"].includes(e.key)) return;
      e.preventDefault();
      pressedKeys.current.add(e.key);
    };
    
    const onKeyUp = (e) => {
      pressedKeys.current.delete(e.key);
      const { lin, ang } = getVel();
      if (lin === 0 && ang === 0) stopRobot();
    };

    cmdInterval.current = setInterval(() => {
      const { lin, ang } = getVel();
      publishTwist(lin, ang);
    }, 100);

    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      clearInterval(cmdInterval.current);
      stopRobot();
    };
  }, [phase, publishTwist, stopRobot]);

  useEffect(() => {
    if (!mapData || !canvasRef.current) return;
    const { info, data } = mapData;
    const { width, height } = info;
    const canvas = canvasRef.current;
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext("2d");
    const img = ctx.createImageData(width, height);
    
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const val = data[(height - 1 - y) * width + x];
        const i = (y * width + x) * 4;
        if (val === -1) { img.data[i] = 180; img.data[i + 1] = 180; img.data[i + 2] = 200; }
        else if (val === 0) { img.data[i] = 255; img.data[i + 1] = 255; img.data[i + 2] = 255; }
        else { img.data[i] = 30; img.data[i + 1] = 30; img.data[i + 2] = 50; }
        img.data[i + 3] = 255;
      }
    }
    ctx.putImageData(img, 0, 0);

    if (robotPos && info) {
      const rx = (robotPos.x - info.origin.position.x) / info.resolution;
      const ry = height - (robotPos.y - info.origin.position.y) / info.resolution;
      ctx.beginPath(); ctx.arc(rx, ry, 6, 0, 2 * Math.PI);
      ctx.fillStyle = "#2563eb"; ctx.fill();
      ctx.strokeStyle = "white"; ctx.lineWidth = 2; ctx.stroke();
    }

    if (goalClick) {
      ctx.beginPath(); ctx.arc(goalClick.px, goalClick.py, 6, 0, 2 * Math.PI);
      ctx.fillStyle = "#dc2626"; ctx.fill();
      ctx.strokeStyle = "white"; ctx.lineWidth = 2; ctx.stroke();
    }
    
    if (pathData && pathData.length > 0 && info) {
      ctx.beginPath();
      ctx.strokeStyle = "#22c55e"; // Bright green
      ctx.lineWidth = 3;
      pathData.forEach((pose, idx) => {
        const px = (pose.pose.position.x - info.origin.position.x) / info.resolution;
        const py = height - (pose.pose.position.y - info.origin.position.y) / info.resolution;
        
        if (idx === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }
    
  }, [mapData, robotPos, goalClick, pathData]);

  const handleMapClick = (e) => {
    if (phase !== "navigate" || !mapData) return;
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const px = (e.clientX - rect.left) * scaleX;
    const py = (e.clientY - rect.top) * scaleY;
    const { info } = mapData;
    const worldX = px * info.resolution + info.origin.position.x;
    const worldY = (canvas.height - py) * info.resolution + info.origin.position.y;
    setGoalClick({ px, py, worldX, worldY });

    // SEND GOAL TO QBOT CONTROLLER
    send({
      op: "publish",
      topic: "/ui_goal",
      type: "geometry_msgs/Point",
      msg: { x: worldX, y: worldY, z: 0.0 }
    });

    setStatus(`Navigating to (${worldX.toFixed(2)}, ${worldY.toFixed(2)})...`);
  };

  const ArrowBtn = ({ label, onPress, onRelease, color = "#1e40af" }) => (
    <button
      onMouseDown={onPress}
      onMouseUp={onRelease}
      onMouseLeave={onRelease}
      onTouchStart={(e) => { e.preventDefault(); onPress(); }}
      onTouchEnd={(e) => { e.preventDefault(); onRelease(); }}
      style={{
        width: 60, height: 60, borderRadius: 10, border: "none",
        background: color, color: "white", fontSize: 20,
        cursor: "pointer", userSelect: "none",
        display: "flex", alignItems: "center", justifyContent: "center",
      }}
    >{label}</button>
  );

  return (
    <div style={{ fontFamily: "sans-serif", maxWidth: 920, margin: "0 auto", padding: 16 }}>

      <div style={{ display: "flex", alignItems: "center", gap: 10, marginBottom: 14 }}>
        <div style={{ width: 10, height: 10, borderRadius: "50%", background: connected ? "#16a34a" : "#dc2626" }} />
        <span style={{ fontSize: 13, color: "#6b7280" }}>{status}</span>
        <div style={{ marginLeft: "auto", display: "flex", gap: 8 }}>
          {[["manual", "Manual Drive"], ["navigate", "Click to Navigate"]].map(([p, label]) => (
            <button key={p} onClick={() => { 
                setPhase(p); 
                stopRobot(); 
                setGoalClick(null); 
                setPathData(null); 
                setStatus(p === "manual" ? "Manual mode" : "Click on map to set goal"); 
              }}
              style={{
                padding: "6px 14px", borderRadius: 8, border: "1.5px solid",
                borderColor: phase === p ? "#1e40af" : "#d1d5db",
                background: phase === p ? "#1e40af" : "white",
                color: phase === p ? "white" : "#374151",
                fontWeight: 500, cursor: "pointer", fontSize: 13,
              }}>{label}</button>
          ))}
        </div>
      </div>

      <div style={{ display: "flex", gap: 16 }}>
        <div style={{ flex: 1 }}>
          <div style={{
            background: "#f1f5f9", border: "1px solid #e2e8f0", borderRadius: 12,
            overflow: "hidden", cursor: phase === "navigate" ? "crosshair" : "default",
            minHeight: 400,
          }}>
            {mapData ? (
              <canvas ref={canvasRef} onClick={handleMapClick}
                style={{ width: "100%", display: "block", imageRendering: "pixelated" }} />
            ) : (
              <div style={{ height: 400, display: "flex", flexDirection: "column", alignItems: "center", justifyContent: "center", color: "#94a3b8" }}>
                <div style={{ fontSize: 36, marginBottom: 8 }}>◎</div>
                <div style={{ fontSize: 14 }}>Waiting for map from SLAM...</div>
                <div style={{ fontSize: 12, marginTop: 4, color: "#cbd5e1" }}>Drive the robot to start mapping</div>
              </div>
            )}
          </div>
        </div>

        <div style={{ width: 180, display: "flex", flexDirection: "column", gap: 14 }}>
          {phase === "manual" && (
            <div style={{ background: "white", border: "1px solid #e2e8f0", borderRadius: 12, padding: 14 }}>
              <div style={{ fontSize: 12, fontWeight: 500, color: "#6b7280", marginBottom: 10 }}>Drive controls</div>
              <div style={{ display: "flex", flexDirection: "column", alignItems: "center", gap: 6 }}>
                <ArrowBtn label="▲" onPress={() => publishTwist(LINEAR_SPEED, 0)} onRelease={stopRobot} />
                <div style={{ display: "flex", gap: 6 }}>
                  <ArrowBtn label="◄" onPress={() => publishTwist(0, ANGULAR_SPEED)} onRelease={stopRobot} />
                  <ArrowBtn label="■" onPress={stopRobot} onRelease={() => { }} color="#374151" />
                  <ArrowBtn label="►" onPress={() => publishTwist(0, -ANGULAR_SPEED)} onRelease={stopRobot} />
                </div>
                <ArrowBtn label="▼" onPress={() => publishTwist(-LINEAR_SPEED, 0)} onRelease={stopRobot} />
              </div>
            </div>
          )}

          {phase === "navigate" && (
            <div style={{ background: "white", border: "1px solid #e2e8f0", borderRadius: 12, padding: 14 }}>
              <div style={{ fontSize: 12, fontWeight: 500, color: "#6b7280", marginBottom: 8 }}>How to navigate</div>
              <div style={{ fontSize: 12, color: "#374151", lineHeight: 1.6 }}>
                1. Click on the map<br />
                2. Nav2 takes over<br />
                3. Costmaps avoid walls
              </div>
            </div>
          )}

          {robotPos && (
            <div style={{ background: "white", border: "1px solid #e2e8f0", borderRadius: 12, padding: 14 }}>
              <div style={{ fontSize: 12, fontWeight: 500, color: "#6b7280", marginBottom: 6 }}>Robot position</div>
              <div style={{ fontSize: 13 }}>X: <strong>{robotPos.x.toFixed(2)}</strong></div>
              <div style={{ fontSize: 13 }}>Y: <strong>{robotPos.y.toFixed(2)}</strong></div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
