import { useEffect, useState } from "react";
import { connectToRobot, connectToSimulator, initNT, } from "./nt";
import Dashboard from "./DashboardWidget";

type ConnectionMode = "robot" | "simulator";
type AppState = "selecting" | "connecting" | "connected" | "error";

export default function App() {
  const [appState, setAppState] = useState<AppState>("selecting");
  const [mode, setMode] = useState<ConnectionMode>("robot");
  const [teamNumber, setTeamNumber] = useState("3620");
  const [simHost, setSimHost] = useState("localhost");
  const [errorMsg, setErrorMsg] = useState("");

  const handleConnect = async () => {
    setAppState("connecting");
    try {
      if (mode === "robot") {
        connectToRobot(parseInt(teamNumber));
      } else {
        connectToSimulator(simHost);
      }
      await initNT();
      setAppState("connected");
    } catch (e) {
      setErrorMsg(e instanceof Error ? e.message : "Unknown error");
      setAppState("error");
    }
  };

  if (appState === "selecting" || appState === "error") {
    return (
      <div>
        <h1>Average Joes Dashboard</h1>
        <h2>Connect to...</h2>

        <div>
          <label>
            <input
              type="radio"
              value="robot"
              checked={mode === "robot"}
              onChange={() => setMode("robot")}
            />
            Real Robot
          </label>
          <label>
            <input
              type="radio"
              value="simulator"
              checked={mode === "simulator"}
              onChange={() => setMode("simulator")}
            />
            Simulator
          </label>
        </div>

        {mode === "robot" ? (
          <div>
            <label>Team Number</label>
            <input
              value={teamNumber}
              onChange={(e) => setTeamNumber(e.target.value)}
              placeholder="3620"
            />
          </div>
        ) : (
          <div>
            <label>Simulator Host</label>
            <input
              value={simHost}
              onChange={(e) => setSimHost(e.target.value)}
              placeholder="localhost"
            />
          </div>
        )}

        {appState === "error" && <p style={{ color: "red" }}>Error: {errorMsg}</p>}

        <button onClick={handleConnect}>Connect</button>
      </div>
    );
  }

  if (appState === "connecting") {
    return <p>Connecting...</p>;
  }

return (
  <div>
    <h1>Average Joes Dashboard</h1>
    <p>✅ Connected ({mode === "robot" ? `Team ${teamNumber}` : `Simulator @ ${simHost}`})</p>
    <Dashboard />
  </div>
);
}