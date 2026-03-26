import { useState, useEffect, useRef } from "react";
import { setValue, subscribeToValue } from "../nt";
import { type TopicConfig } from "../topics";
import fieldImage from "../assets/field_image.png";

const FIELD_WIDTH_M = 16.54;
const FIELD_HEIGHT_M = 8.07;

interface Translation3d { x: number; y: number; z: number; }

// One color per target so they're visually distinct on the field
const TARGET_COLORS: Record<string, string> = {
  "/dashboard/target/hub": "rgba(0, 120, 255, 0.85)",
  "/dashboard/target/op_pass": "rgba(162, 0, 255, 0.85)",
  "/dashboard/target/depot_pass": "rgba(255, 106, 193, 0.85)",
};

export default function MultiTargetFieldWidget({ topics }: { topics: TopicConfig[] }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);
  const [imageLoaded, setImageLoaded] = useState(false);
  const [selectedKey, setSelectedKey] = useState(topics[0].key);

  // Store all target positions in one map keyed by topic key
  const [positions, setPositions] = useState<Record<string, Translation3d>>(() =>
    Object.fromEntries(
      topics.map(t => {
        const def = t.defaultValue as number[];
        return [t.key, { x: def[0], y: def[1], z: def[2] }];
      })
    )
  );

  // Z input per target
  const [zValues, setZValues] = useState<Record<string, number>>(() =>
    Object.fromEntries(topics.map(t => [t.key, (t.defaultValue as number[])[2]]))
  );

  useEffect(() => {
    const img = new Image();
    img.src = fieldImage;
    img.onload = () => { imageRef.current = img; setImageLoaded(true); };
  }, []);

  // Subscribe to all topics
  useEffect(() => {
    topics.forEach(topic => {
      subscribeToValue(topic.key, (value) => {
        if (Array.isArray(value) && value.length >= 3) {
          setPositions(prev => ({
            ...prev,
            [topic.key]: { x: value[0], y: value[1], z: value[2] }
          }));
        }
      });
    });
  }, [topics]);

  // Redraw whenever any position changes
  useEffect(() => {
    const canvas = canvasRef.current;
    const img = imageRef.current;
    if (!canvas || !img || !imageLoaded) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const W = canvas.width;
    const H = canvas.height;
    ctx.drawImage(img, 0, 0, W, H);

    // Draw all targets
    topics.forEach(topic => {
      const pos = positions[topic.key];
      const px = (pos.x / FIELD_WIDTH_M) * W;
      const py = H - (pos.y / FIELD_HEIGHT_M) * H;
      const color = TARGET_COLORS[topic.key] ?? "rgba(255,255,255,0.85)";
      const isSelected = topic.key === selectedKey;
      const size = isSelected ? 24 : 18; // selected target is bigger

      ctx.save();
      ctx.translate(px, py);

      // Highlight ring for selected target
      if (isSelected) {
        ctx.strokeStyle = "white";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.arc(0, 0, size, 0, Math.PI * 2);
        ctx.stroke();
      }

      ctx.fillStyle = color;
      ctx.strokeStyle = "white";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(0, 0, size / 2, 0, Math.PI * 2); // circle instead of square for targets
      ctx.fill();
      ctx.stroke();

      // Label
      ctx.fillStyle = isSelected ? "#00274C" : "#00274C";
      ctx.font = `bold ${isSelected ? 13 : 11}px 'Share Tech Mono', monospace`;
      ctx.fillText(topic.label, size / 2 + 4, 4);

      ctx.restore();
    });
  }, [positions, imageLoaded, selectedKey, topics]);

  const handleCanvasClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const px = (e.clientX - rect.left) * scaleX;
    const py = (e.clientY - rect.top) * scaleY;

    const x = parseFloat(((px / canvas.width) * FIELD_WIDTH_M).toFixed(3));
    const y = parseFloat(((1 - py / canvas.height) * FIELD_HEIGHT_M).toFixed(3));
    const z = zValues[selectedKey];

    setPositions(prev => ({ ...prev, [selectedKey]: { x, y, z } }));
    setValue(selectedKey, [x, y, z]);
  };

  const selectedPos = positions[selectedKey];

  return (
    <div style={{
      background: "var(--bg-panel)",
      border: "1px solid var(--border)",
      borderRadius: "4px",
      padding: "1.25rem",
      boxShadow: "0 0 30px rgba(255,203,5,0.03)",
    }}>
      {/* Header */}
      <div style={{
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
        marginBottom: "1rem",
      }}>
        <label style={{
          fontFamily: "var(--font-display)",
          fontWeight: 700,
          fontSize: "1rem",
          letterSpacing: "0.2em",
          textTransform: "uppercase",
          color: "var(--maize)",
        }}>
          Field Targets
        </label>

        {/* Z input */}
        <div style={{ display: "flex", alignItems: "center", gap: "0.5rem" }}>
          <span style={{
            fontFamily: "var(--text-mono)",
            fontSize: "0.7rem",
            color: "var(--text-secondary)",
            letterSpacing: "0.15em",
            textTransform: "uppercase",
          }}>
            Z ({topics.find(t => t.key === selectedKey)?.label}):
          </span>
          <input
            type="number"
            value={zValues[selectedKey]}
            onChange={(e) => setZValues(prev => ({
              ...prev,
              [selectedKey]: parseFloat(e.target.value) || 0
            }))}
            style={{
              width: "70px",
              background: "var(--bg)",
              border: "1px solid var(--border)",
              borderRadius: "2px",
              padding: "0.3rem 0.5rem",
              color: "var(--maize)",
              fontFamily: "var(--text-mono)",
              fontSize: "0.85rem",
              outline: "none",
              textAlign: "center",
            }}
          />
          <span style={{ fontFamily: "var(--text-mono)", fontSize: "0.7rem", color: "var(--text-secondary)" }}>ft</span>
        </div>
      </div>

      {/* Target selector tabs */}
      <div style={{ display: "flex", gap: "0.5rem", marginBottom: "0.75rem" }}>
        {topics.map(t => (
          <button
            key={t.key}
            onClick={() => setSelectedKey(t.key)}
            style={{
              background: selectedKey === t.key ? TARGET_COLORS[t.key] : "var(--bg-card)",
              color: selectedKey === t.key ? "#00274C" : "var(--text-secondary)",
              border: `1px solid ${selectedKey === t.key ? TARGET_COLORS[t.key] : "var(--border)"}`,
              padding: "0.35rem 1rem",
              borderRadius: "2px",
              cursor: "pointer",
              fontFamily: "var(--font-display)",
              fontWeight: 700,
              fontSize: "0.85rem",
              letterSpacing: "0.1em",
              textTransform: "uppercase",
              transition: "all 0.15s ease",
            }}
          >
            {t.label}
          </button>
        ))}
      </div>

      {/* Canvas */}
      <canvas
        ref={canvasRef}
        width={800}
        height={400}
        onClick={handleCanvasClick}
        style={{
          width: "100%",
          border: "1px solid var(--border-bright)",
          borderRadius: "2px",
          cursor: "crosshair",
          display: "block",
          boxShadow: "0 0 20px rgba(255,203,5,0.08)",
        }}
      />

      {/* Readout */}
      <div style={{
        marginTop: "0.75rem",
        padding: "0.5rem 0.75rem",
        background: "var(--bg)",
        border: "1px solid var(--border)",
        borderRadius: "2px",
        fontFamily: "var(--text-mono)",
        fontSize: "0.75rem",
        color: "var(--maize)",
        letterSpacing: "0.05em",
      }}>
        <span style={{ color: "var(--text-secondary)", marginRight: "0.5rem" }}>
          {topics.find(t => t.key === selectedKey)?.label.toUpperCase()} →
        </span>
        X: {selectedPos.x.toFixed(3)}m &nbsp;|&nbsp;
        Y: {selectedPos.y.toFixed(3)}m &nbsp;|&nbsp;
        Z: {selectedPos.z.toFixed(2)}ft
      </div>
    </div>
  );
}