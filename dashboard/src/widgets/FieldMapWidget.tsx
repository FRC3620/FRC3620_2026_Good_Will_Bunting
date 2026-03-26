import { useState, useEffect, useRef } from "react";
import { setValue, subscribeToValue } from "../nt";
import { type TopicConfig } from "../topics";
import fieldImage from "../assets/field_image.png";

const FIELD_WIDTH_M = 16.54;
const FIELD_HEIGHT_M = 8.07;

interface Translation3d {
  x: number;       // meters from left
  y: number;       // meters from bottom
  z: number; // Feet
}

export default function FieldWidget({ topic }: { topic: TopicConfig }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);
  const [pose, setPose] = useState<Translation3d>({ x: 0, y: 0, z: 0 });
  const [z, setZ] = useState(0);
  const [imageLoaded, setImageLoaded] = useState(false);

  // Load the field image once
  useEffect(() => {
    const img = new Image();
    img.src = fieldImage;
    img.onload = () => {
      imageRef.current = img;
      setImageLoaded(true);
    };
  }, []);

  // Subscribe to pose updates from NT
  useEffect(() => {
    subscribeToValue(topic.key, (value) => {
      if (Array.isArray(value) && value.length >= 3) {
        setPose({ x: value[0], y: value[1], z: value[2] });
      }
    });
  }, [topic.key]);

  // Redraw canvas whenever pose or image changes
  useEffect(() => {
    const canvas = canvasRef.current;
    const img = imageRef.current;
    if (!canvas || !img || !imageLoaded) return;

    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const W = canvas.width;
    const H = canvas.height;

    // Draw field background
    ctx.drawImage(img, 0, 0, W, H);

    // Convert meters to pixels
    const px = (pose.x / FIELD_WIDTH_M) * W;
    const py = H - (pose.y / FIELD_HEIGHT_M) * H; // flip Y axis

    // Draw robot as an arrow showing heading
    const robotSize = 20;

    ctx.save();
    ctx.translate(px, py);

    // Robot body
    ctx.fillStyle = "rgba(0, 120, 255, 0.85)";
    ctx.strokeStyle = "white";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.rect(-robotSize / 2, -robotSize / 2, robotSize, robotSize);
    ctx.fill();
    ctx.stroke();

    // Direction arrow (front of robot)
    ctx.fillStyle = "white";
    ctx.beginPath();
    ctx.moveTo(0, -robotSize / 2 - 8);
    ctx.lineTo(-6, -robotSize / 2);
    ctx.lineTo(6, -robotSize / 2);
    ctx.closePath();
    ctx.fill();

    ctx.restore();

    // Coordinates label
    ctx.fillStyle = "white";
    ctx.font = "bold 13px monospace";
    ctx.fillText(
      `x: ${pose.x.toFixed(2)}m  y: ${pose.y.toFixed(2)}m  θ: ${pose.z.toFixed(1)}°`,
      8,
      H - 8
    );
  }, [pose, imageLoaded]);

  const handleCanvasClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();

    // Convert click position to canvas pixels
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const px = (e.clientX - rect.left) * scaleX;
    const py = (e.clientY - rect.top) * scaleY;

    // Convert canvas pixels to field meters
    const x = (px / canvas.width) * FIELD_WIDTH_M;
    const y = (1 - py / canvas.height) * FIELD_HEIGHT_M; // flip Y

    const newPose: Translation3d = { x, y, z };
    setPose(newPose);
    setValue(topic.key, [
      parseFloat(x.toFixed(3)),
      parseFloat(y.toFixed(3)),
      z,
    ]);
  };

  return (
    <div>
      <label style={{ fontWeight: "bold" }}>{topic.label}</label>

      <div style={{ marginBottom: "0.5rem" }}>
        <label>Z (Ft): </label>
        <input
          type="number"
          value={z}
          onChange={(e) => setZ(parseFloat(e.target.value) || 0)}
          style={{ width: "80px" }}
        />
      </div>

      <canvas
        ref={canvasRef}
        width={800}
        height={400}
        onClick={handleCanvasClick}
        style={{
          width: "100%",
          border: "2px solid #444",
          cursor: "crosshair",
          borderRadius: "4px",
        }}
      />

      <p style={{ fontFamily: "monospace", marginTop: "0.25rem" }}>
        Last sent → x: {pose.x.toFixed(3)}m, y: {pose.y.toFixed(3)}m, θ: {pose.z.toFixed(1)}°
      </p>
    </div>
  );
}