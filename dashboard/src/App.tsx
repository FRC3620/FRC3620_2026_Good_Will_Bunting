import { useEffect, useState } from "react";
import { connectToRobot, connectToSimulator, initNT, disconnectNT, isConnected } from "./nt";
import Dashboard from "./DashboardWidget";
import buntingLogoBig from "./assets/BuntingLogoBig.png";
import buntingLogoSmall from "./assets/BuntingLogoSmall.png";
import buntingLogoSmall2 from "./assets/BuntingLogoSmall2.png";

// Add these styles inside App.tsx via a <style> tag at the top of each return,
// or just apply classNames if you set up CSS modules.
// Here's the full styled version using inline + a shared stylesheet approach:

// At the very top of App.tsx, add this after your imports:
const styles = {
  page: {
    minHeight: "100vh",
    display: "flex",
    flexDirection: "column" as const,
  },
  header: {
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between",
    padding: "1rem 2rem",
    borderBottom: "1px solid var(--border)",
    background: "rgba(7, 21, 37, 0.95)",
    backdropFilter: "blur(10px)",
    position: "sticky" as const,
    top: 0,
    zIndex: 100,
  },
  headerTitle: {
    fontFamily: "var(--font-display)",
    fontSize: "1.8rem",
    fontWeight: 700,
    letterSpacing: "0.15em",
    textTransform: "uppercase" as const,
    color: "var(--maize)",
    textShadow: "0 0 20px rgba(255, 203, 5, 0.4)",
  },
  headerSub: {
    fontSize: "0.75rem",
    color: "var(--text-secondary)",
    letterSpacing: "0.2em",
    textTransform: "uppercase" as const,
    fontFamily: "var(--text-mono)",
  },
  statusBadge: (connected: boolean) => ({
    display: "flex",
    alignItems: "center",
    gap: "0.5rem",
    padding: "0.4rem 1rem",
    borderRadius: "2px",
    border: `1px solid ${connected ? "rgba(255,203,5,0.4)" : "rgba(255,80,80,0.4)"}`,
    background: connected ? "rgba(255,203,5,0.08)" : "rgba(255,80,80,0.08)",
    fontFamily: "var(--text-mono)",
    fontSize: "0.75rem",
    color: connected ? "var(--maize)" : "#ff5050",
    letterSpacing: "0.1em",
  }),
  dot: (connected: boolean) => ({
    width: "8px",
    height: "8px",
    borderRadius: "50%",
    background: connected ? "var(--maize)" : "#ff5050",
    boxShadow: connected ? "0 0 8px var(--maize)" : "0 0 8px #ff5050",
    animation: "pulse 2s infinite",
  }),
  connectPage: {
    flex: 1,
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
  },
  connectCard: {
    background: "var(--bg-panel)",
    border: "1px solid var(--border)",
    borderRadius: "4px",
    padding: "2.5rem",
    width: "420px",
    boxShadow: "0 0 40px rgba(255, 203, 5, 0.05), inset 0 1px 0 rgba(255,203,5,0.1)",
  },
  connectTitle: {
    fontFamily: "var(--font-display)",
    fontSize: "1.1rem",
    fontWeight: 600,
    letterSpacing: "0.25em",
    textTransform: "uppercase" as const,
    color: "var(--text-secondary)",
    marginBottom: "1.5rem",
  },
  radioGroup: {
    display: "grid",
    gridTemplateColumns: "1fr 1fr",
    gap: "0.75rem",
    marginBottom: "1.5rem",
  },
  radioLabel: (active: boolean) => ({
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    gap: "0.5rem",
    padding: "0.75rem",
    border: `1px solid ${active ? "var(--maize)" : "var(--border)"}`,
    borderRadius: "2px",
    background: active ? "var(--maize-glow)" : "transparent",
    color: active ? "var(--maize)" : "var(--text-secondary)",
    cursor: "pointer",
    fontFamily: "var(--font-display)",
    fontWeight: 600,
    letterSpacing: "0.1em",
    fontSize: "0.9rem",
    transition: "all 0.15s ease",
  }),
  inputGroup: {
    marginBottom: "1.5rem",
  },
  inputLabel: {
    display: "block",
    fontFamily: "var(--text-mono)",
    fontSize: "0.7rem",
    color: "var(--text-secondary)",
    letterSpacing: "0.2em",
    textTransform: "uppercase" as const,
    marginBottom: "0.4rem",
  },
  input: {
    width: "100%",
    background: "var(--bg)",
    border: "1px solid var(--border)",
    borderRadius: "2px",
    padding: "0.6rem 0.75rem",
    color: "var(--text-primary)",
    fontFamily: "var(--text-mono)",
    fontSize: "0.9rem",
    outline: "none",
    transition: "border-color 0.15s",
  },
  connectBtn: {
    width: "100%",
    padding: "0.85rem",
    background: "var(--maize)",
    color: "var(--navy)",
    border: "none",
    borderRadius: "2px",
    fontFamily: "var(--font-display)",
    fontWeight: 700,
    fontSize: "1rem",
    letterSpacing: "0.2em",
    textTransform: "uppercase" as const,
    cursor: "pointer",
    transition: "all 0.15s ease",
  },
  errorText: {
    fontFamily: "var(--text-mono)",
    fontSize: "0.75rem",
    color: "#ff5050",
    marginBottom: "1rem",
    padding: "0.5rem",
    border: "1px solid rgba(255,80,80,0.3)",
    borderRadius: "2px",
    background: "rgba(255,80,80,0.05)",
  },
};

type ConnectionMode = "robot" | "simulator";
type AppState = "selecting" | "connecting" | "connected" | "error";

export default function App() {
  const [appState, setAppState] = useState<AppState>("selecting");
  const [mode, setMode] = useState<ConnectionMode>("robot");
  const [teamNumber, setTeamNumber] = useState("3620");
  const [simHost, setSimHost] = useState("localhost");
  const [errorMsg, setErrorMsg] = useState("");
  const [ntConnected, setNtConnected] = useState(false);

  useEffect(() => {
    if (appState !== "connected") return;

    const interval = setInterval(() => {
      setNtConnected(isConnected());
    }, 500);

    return () => clearInterval(interval);
  }, [appState]);

  const handleDisconnect = () => {
    disconnectNT();
    setNtConnected(false);
    setAppState("selecting");
  };

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
    // Only the connection screen return goes here
    return (
      <div style={styles.page}>
        <header style={styles.header}>
          <div>
            <div style={styles.headerTitle}>Average Joes · BUNTING</div>
            <div style={styles.headerSub}>FRC Team 3620 · Dashboard</div>
            <div ></div>
          </div>
        </header>

        <div style={styles.connectPage}>
          <div style={styles.connectCard}>
            {/* Logo */}
            <div style={{
              display: "flex",
              justifyContent: "center",
              marginBottom: "1.5rem",
            }}>
              <img
                src={buntingLogoBig}
                alt="BUNTING Dashboard"
                style={{
                  width: "350px",
                  height: "auto",
                  filter: "drop-shadow(0 0 18px rgba(255, 203, 5, 0.2))",
                }}
              />
            </div>

            <div style={styles.connectTitle}>Establish Connection</div>

            <div style={styles.radioGroup}>
              <label style={styles.radioLabel(mode === "robot")} onClick={() => setMode("robot")}>
                <input type="radio" value="robot" checked={mode === "robot"}
                  onChange={() => setMode("robot")} style={{ display: "none" }} />
                🤖 Robot
              </label>
              <label style={styles.radioLabel(mode === "simulator")} onClick={() => setMode("simulator")}>
                <input type="radio" value="simulator" checked={mode === "simulator"}
                  onChange={() => setMode("simulator")} style={{ display: "none" }} />
                💻 Simulator
              </label>
            </div>

            {mode === "robot" ? (
              <div style={styles.inputGroup}>
                <label style={styles.inputLabel}>Team Number</label>
                <input style={styles.input} value={teamNumber}
                  onChange={(e) => setTeamNumber(e.target.value)} placeholder="3620" />
              </div>
            ) : (
              <div style={styles.inputGroup}>
                <label style={styles.inputLabel}>Simulator Host</label>
                <input style={styles.input} value={simHost}
                  onChange={(e) => setSimHost(e.target.value)} placeholder="localhost" />
              </div>
            )}

            {appState === "error" && (
              <div style={styles.errorText}>⚠ {errorMsg}</div>
            )}

            <button style={styles.connectBtn} onClick={handleConnect}
              onMouseEnter={e => (e.currentTarget.style.background = "var(--maize-dim)")}
              onMouseLeave={e => (e.currentTarget.style.background = "var(--maize)")}>
              Initialize Link
            </button>
          </div>
        </div>
      </div>
    );
  }

  if (appState === "connecting") {
    return <p style={{ fontFamily: "var(--text-mono)", color: "var(--maize)", padding: "2rem" }}>Initializing link...</p>;
  }

  // Connected screen is outside both if blocks, at the bottom
  return (
    <div style={styles.page}>
      <header style={styles.header}>
        <div style={{ display: "flex", alignItems: "center", gap: "0.5rem" }}>
          <img
            src={buntingLogoSmall}
            alt="BUNTING"
            style={{
              height: "55px",
              width: "auto",
              filter: "drop-shadow(0 0 8px rgba(255, 203, 5, 0.25))",
              mixBlendMode: "screen",
            }}
          />
          <div>
            <div style={styles.headerTitle}>Average Joes · BUNTING</div>
            <div style={styles.headerSub}>FRC Team 3620 · Dashboard</div>
          </div>
          <img
            src={buntingLogoSmall2}
            alt="BUNTING"
            style={{
              height: "55px",
              width: "auto",
              filter: "drop-shadow(0 0 8px rgba(255, 203, 5, 0.25))",
              mixBlendMode: "screen",
            }}
          />
        </div>

        {/* Right side of header */}
        <div style={{ display: "flex", alignItems: "center", gap: "0.75rem" }}>

          {/* Connection status */}
          <div style={styles.statusBadge(ntConnected)}>
            <div style={styles.dot(ntConnected)} />
            {mode === "robot" ? `TEAM ${teamNumber}` : `SIM · ${simHost}`}
          </div>

          {/* Disconnect / back to menu */}
          <button
            onClick={handleDisconnect}
            style={{
              background: "transparent",
              color: "var(--text-secondary)",
              border: "1px solid var(--border)",
              padding: "0.4rem 0.9rem",
              borderRadius: "2px",
              fontFamily: "var(--text-mono)",
              fontSize: "0.7rem",
              letterSpacing: "0.15em",
              textTransform: "uppercase",
              cursor: "pointer",
              transition: "all 0.15s ease",
            }}
            onMouseEnter={e => {
              e.currentTarget.style.borderColor = "rgba(255,80,80,0.5)";
              e.currentTarget.style.color = "#ff5050";
            }}
            onMouseLeave={e => {
              e.currentTarget.style.borderColor = "var(--border)";
              e.currentTarget.style.color = "var(--text-secondary)";
            }}
          >
            ← Menu
          </button>
        </div>
      </header>

      <div style={{ padding: "0.75rem" }}>
        <Dashboard />
      </div>
    </div>
  );
}