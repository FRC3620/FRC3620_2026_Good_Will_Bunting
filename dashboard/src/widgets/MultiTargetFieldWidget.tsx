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
    <div>
      <label style={{ fontWeight: "bold" }}>Field Targets</label>

      {/* Target selector tabs */}
      <div style={{ display: "flex", gap: "0.5rem", margin: "0.5rem 0" }}>
        {topics.map(t => (
          <button
            key={t.key}
            onClick={() => setSelectedKey(t.key)}
            style={{
              background: selectedKey === t.key ? TARGET_COLORS[t.key] : "#333",
              color: "white",
              border: "none",
              padding: "4px 12px",
              borderRadius: "4px",
              cursor: "pointer",
            }}
          >
            {t.label}
          </button>
        ))}
      </div>

      {/* Z input for the currently selected target */}
      <div style={{ marginBottom: "0.5rem" }}>
        <label>Z for {topics.find(t => t.key === selectedKey)?.label} (ft): </label>
        <input
          type="number"
          value={zValues[selectedKey]}
          onChange={(e) => setZValues(prev => ({
            ...prev,
            [selectedKey]: parseFloat(e.target.value) || 0
          }))}
          style={{ width: "80px" }}
        />
      </div>

      <canvas
        ref={canvasRef}
        width={800}
        height={400}
        onClick={handleCanvasClick}
        style={{ width: "100%", border: "2px solid #444", cursor: "crosshair", borderRadius: "4px" }}
      />

      <p style={{ fontFamily: "monospace", marginTop: "0.25rem" }}>
        {topics.find(t => t.key === selectedKey)?.label} → x: {selectedPos.x.toFixed(3)}m, y: {selectedPos.y.toFixed(3)}m, z: {selectedPos.z.toFixed(2)}ft
      </p>
    </div>
  );
}