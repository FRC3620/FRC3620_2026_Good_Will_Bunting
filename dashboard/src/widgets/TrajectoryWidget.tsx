import { useEffect, useRef, useState } from "react";
import { subscribeToValue } from "../nt";
import { type TopicConfig } from "../topics";
import fieldImage from "../assets/field_image.png";

const FIELD_WIDTH_M = 16.54;
const FIELD_HEIGHT_M = 8.07;
const G = 32.2; // ft/s²

interface ShotData {
  robotX: number;
  robotY: number;
  robotHeading: number;
  targetX: number;
  targetY: number;
  targetZ: number;
  exitAngleDeg: number;
  exitVelocityFtps: number;
  hDistanceFt: number;
  fieldAngleDeg: number;
}

// Compute parabolic arc points in the vertical (side) plane
// Returns array of {t, x, y} where x is horizontal distance and y is height
function computeArcPoints(
  exitVelocityFtps: number,
  exitAngleDeg: number,
  targetHeightFt: number,
  steps = 60
): { x: number; y: number }[] {
  const angleRad = (exitAngleDeg * Math.PI) / 180;
  const vx = exitVelocityFtps * Math.cos(angleRad);
  const vy = exitVelocityFtps * Math.sin(angleRad);

  // Time to reach target height (quadratic formula)
  // y = vy*t - 0.5*G*t^2 = targetHeight
  // 0.5G*t^2 - vy*t + targetHeight = 0
  const disc = vy * vy - 2 * G * targetHeightFt;
  const tFlight = disc >= 0
    ? (vy + Math.sqrt(disc)) / G
    : (vy * 2) / G; // fallback: full parabola if target unreachable

  const points: { x: number; y: number }[] = [];
  for (let i = 0; i <= steps; i++) {
    const t = (i / steps) * tFlight;
    const x = vx * t;
    const y = vy * t - 0.5 * G * t * t;
    points.push({ x, y });
  }
  return points;
}

export default function TrajectoryWidget({ topic }: { topic: TopicConfig }) {
  const topDownRef = useRef<HTMLCanvasElement>(null);
  const sideRef = useRef<HTMLCanvasElement>(null);
  const fieldImgRef = useRef<HTMLImageElement | null>(null);
  const [fieldLoaded, setFieldLoaded] = useState(false);
  const [shot, setShot] = useState<ShotData>(() => {
    const d = topic.defaultValue as number[];
    return {
      robotX: d[0], robotY: d[1], robotHeading: d[2],
      targetX: d[3], targetY: d[4], targetZ: d[5],
      exitAngleDeg: d[6], exitVelocityFtps: d[7],
      hDistanceFt: d[8], fieldAngleDeg: d[9],
    };
  });

  useEffect(() => {
    const img = new Image();
    img.src = fieldImage;
    img.onload = () => { fieldImgRef.current = img; setFieldLoaded(true); };
  }, []);

  useEffect(() => {
    subscribeToValue(topic.key, (value) => {
      if (Array.isArray(value) && value.length >= 10) {
        setShot({
          robotX: value[0], robotY: value[1], robotHeading: value[2],
          targetX: value[3], targetY: value[4], targetZ: value[5],
          exitAngleDeg: value[6], exitVelocityFtps: value[7],
          hDistanceFt: value[8], fieldAngleDeg: value[9],
        });
      }
    });
  }, [topic.key]);

  // Draw top-down view
  useEffect(() => {
    const canvas = topDownRef.current;
    const img = fieldImgRef.current;
    if (!canvas || !img || !fieldLoaded) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.scale(dpr, dpr);
    const W = rect.width;
    const H = rect.height;

    ctx.drawImage(img, 0, 0, W, H);

    const toCanvasX = (mx: number) => (mx / FIELD_WIDTH_M) * W;
    const toCanvasY = (my: number) => H - (my / FIELD_HEIGHT_M) * H;

    const rx = toCanvasX(shot.robotX);
    const ry = toCanvasY(shot.robotY);
    const tx = toCanvasX(shot.targetX);
    const ty = toCanvasY(shot.targetY);

    // Trajectory line (projected onto field top-down)
    const arcPoints = computeArcPoints(
      shot.exitVelocityFtps,
      shot.exitAngleDeg,
      shot.targetZ * 3.281, // meters to feet
    );
    const totalArcDist = arcPoints[arcPoints.length - 1].x; // ft
    const totalArcDistM = totalArcDist / 3.281;
    const angleRad = (shot.fieldAngleDeg * Math.PI) / 180;

    ctx.beginPath();
    arcPoints.forEach((pt, i) => {
      const frac = pt.x / (totalArcDist || 1);
      const cx = rx + Math.cos(angleRad) * frac * totalArcDistM * (W / FIELD_WIDTH_M);
      const cy = ry - Math.sin(angleRad) * frac * totalArcDistM * (H / FIELD_HEIGHT_M);
      i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy);
    });
    ctx.strokeStyle = "rgba(255, 203, 5, 0.8)";
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 3]);
    ctx.stroke();
    ctx.setLineDash([]);

    // Robot
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(-(shot.robotHeading * Math.PI) / 180);
    ctx.fillStyle = "rgba(0, 120, 255, 0.9)";
    ctx.strokeStyle = "white";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.rect(-10, -10, 20, 20);
    ctx.fill();
    ctx.stroke();
    // heading arrow
    ctx.fillStyle = "white";
    ctx.beginPath();
    ctx.moveTo(0, -18);
    ctx.lineTo(-5, -10);
    ctx.lineTo(5, -10);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    // Target
    ctx.save();
    ctx.translate(tx, ty);
    ctx.fillStyle = "rgba(255, 203, 5, 0.9)";
    ctx.strokeStyle = "#00274C";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.restore();

    // Labels
    ctx.font = "bold 11px 'Share Tech Mono', monospace";
    ctx.fillStyle = "#00274C";
    ctx.fillText("ROBOT", rx + 12, ry + 4);
    ctx.fillStyle = "#00274C";
    ctx.fillText("TARGET", tx + 10, ty + 4);

  }, [shot, fieldLoaded]);

  // Draw side view
  useEffect(() => {
    const canvas = sideRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.scale(dpr, dpr);
    const W = rect.width;
    const H = rect.height;

    const PAD = { top: 20, right: 20, bottom: 30, left: 40 };
    const plotW = W - PAD.left - PAD.right;
    const plotH = H - PAD.top - PAD.bottom;

    // Background
    ctx.fillStyle = "#040d1a";
    ctx.fillRect(0, 0, W, H);

    // Grid lines
    ctx.strokeStyle = "rgba(255,203,5,0.07)";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 5; i++) {
      const y = PAD.top + (i / 5) * plotH;
      ctx.beginPath();
      ctx.moveTo(PAD.left, y);
      ctx.lineTo(PAD.left + plotW, y);
      ctx.stroke();
    }
    for (let i = 0; i <= 6; i++) {
      const x = PAD.left + (i / 6) * plotW;
      ctx.beginPath();
      ctx.moveTo(x, PAD.top);
      ctx.lineTo(x, PAD.top + plotH);
      ctx.stroke();
    }

    const targetZFt = shot.targetZ * 3.281;
    const arcPoints = computeArcPoints(
      shot.exitVelocityFtps,
      shot.exitAngleDeg,
      targetZFt,
    );

    const maxX = arcPoints[arcPoints.length - 1].x || 1;
    const maxY = Math.max(...arcPoints.map(p => p.y), targetZFt, 1) * 1.2;

    const toPlotX = (ft: number) => PAD.left + (ft / maxX) * plotW;
    const toPlotY = (ft: number) => PAD.top + plotH - (ft / maxY) * plotH;

    // Floor line
    ctx.strokeStyle = "rgba(255,203,5,0.2)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(PAD.left, toPlotY(0));
    ctx.lineTo(PAD.left + plotW, toPlotY(0));
    ctx.stroke();

    // Arc fill gradient
    const grad = ctx.createLinearGradient(0, PAD.top, 0, PAD.top + plotH);
    grad.addColorStop(0, "rgba(255,203,5,0.2)");
    grad.addColorStop(1, "rgba(255,203,5,0)");

    ctx.beginPath();
    arcPoints.forEach((pt, i) => {
      i === 0
        ? ctx.moveTo(toPlotX(pt.x), toPlotY(pt.y))
        : ctx.lineTo(toPlotX(pt.x), toPlotY(pt.y));
    });
    ctx.lineTo(toPlotX(maxX), toPlotY(0));
    ctx.lineTo(toPlotX(0), toPlotY(0));
    ctx.closePath();
    ctx.fillStyle = grad;
    ctx.fill();

    // Arc line
    ctx.beginPath();
    arcPoints.forEach((pt, i) => {
      i === 0
        ? ctx.moveTo(toPlotX(pt.x), toPlotY(pt.y))
        : ctx.lineTo(toPlotX(pt.x), toPlotY(pt.y));
    });
    ctx.strokeStyle = "var(--maize)";
    ctx.lineWidth = 2.5;
    ctx.stroke();

    // Target height line
    ctx.strokeStyle = "rgba(255,80,80,0.7)";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(toPlotX(maxX) - 4, toPlotY(targetZFt));
    ctx.lineTo(toPlotX(maxX) + 4, toPlotY(targetZFt));
    ctx.moveTo(toPlotX(maxX), toPlotY(0));
    ctx.lineTo(toPlotX(maxX), toPlotY(targetZFt));
    ctx.stroke();
    ctx.setLineDash([]);

    // Target dot
    ctx.fillStyle = "#ff5050";
    ctx.beginPath();
    ctx.arc(toPlotX(maxX), toPlotY(targetZFt), 5, 0, Math.PI * 2);
    ctx.fill();

    // Launch dot
    ctx.fillStyle = "rgba(0,120,255,0.9)";
    ctx.beginPath();
    ctx.arc(toPlotX(0), toPlotY(0), 5, 0, Math.PI * 2);
    ctx.fill();

    // Axes labels
    ctx.fillStyle = "rgba(255,203,5,0.5)";
    ctx.font = "10px 'Share Tech Mono', monospace";
    ctx.fillText(`${maxX.toFixed(0)}ft`, PAD.left + plotW - 20, PAD.top + plotH + 20);
    ctx.fillText(`${maxY.toFixed(0)}ft`, PAD.left - 36, PAD.top + 8);
    ctx.fillText("0", PAD.left - 12, toPlotY(0) + 4);

    // Angle label
    ctx.fillStyle = "rgba(255,203,5,0.7)";
    ctx.font = "bold 11px 'Share Tech Mono', monospace";
    ctx.fillText(`θ: ${shot.exitAngleDeg.toFixed(1)}°`, PAD.left + 6, PAD.top + 14);
    ctx.fillText(`v: ${shot.exitVelocityFtps.toFixed(1)} ft/s`, PAD.left + 6, PAD.top + 26);

  }, [shot]);

  return (
    <div style={{
      background: "var(--bg-panel)",
      border: "1px solid var(--border)",
      borderRadius: "4px",
      padding: "1.25rem",
      display: "flex",
      flexDirection: "column",
      gap: "1rem",
    }}>
      <label style={{
        fontFamily: "var(--font-display)",
        fontWeight: 700,
        fontSize: "1rem",
        letterSpacing: "0.2em",
        textTransform: "uppercase",
        color: "var(--maize)",
      }}>
        Trajectory Preview
      </label>

      {/* Top-down */}
      <div>
        <div style={{
          fontFamily: "var(--text-mono)",
          fontSize: "0.65rem",
          color: "var(--text-secondary)",
          letterSpacing: "0.15em",
          textTransform: "uppercase",
          marginBottom: "0.4rem",
        }}>
          Top Down
        </div>
        <canvas
          ref={topDownRef}
          style={{
            width: "100%",
            aspectRatio: `${FIELD_WIDTH_M} / ${FIELD_HEIGHT_M}`,
            display: "block",
            border: "1px solid var(--border)",
            borderRadius: "2px",
          }}
        />
      </div>

      {/* Side profile */}
      <div>
        <div style={{
          fontFamily: "var(--text-mono)",
          fontSize: "0.65rem",
          color: "var(--text-secondary)",
          letterSpacing: "0.15em",
          textTransform: "uppercase",
          marginBottom: "0.4rem",
        }}>
          Side Profile
        </div>
        <canvas
          ref={sideRef}
          style={{
            width: "100%",
            aspectRatio: "3 / 1",
            display: "block",
            border: "1px solid var(--border)",
            borderRadius: "2px",
          }}
        />
      </div>

      {/* Data readout */}
      <div style={{
        display: "grid",
        gridTemplateColumns: "1fr 1fr",
        gap: "0.5rem",
        fontFamily: "var(--text-mono)",
        fontSize: "0.72rem",
      }}>
        {[
          ["EXIT ANGLE", `${shot.exitAngleDeg.toFixed(1)}°`],
          ["EXIT VEL", `${shot.exitVelocityFtps.toFixed(1)} ft/s`],
          ["H DISTANCE", `${shot.hDistanceFt.toFixed(1)} ft`],
          ["FIELD ANGLE", `${shot.fieldAngleDeg.toFixed(1)}°`],
        ].map(([label, value]) => (
          <div key={label} style={{
            padding: "0.4rem 0.6rem",
            background: "var(--bg)",
            border: "1px solid var(--border)",
            borderRadius: "2px",
          }}>
            <div style={{ color: "var(--text-secondary)", fontSize: "0.6rem", letterSpacing: "0.15em" }}>
              {label}
            </div>
            <div style={{ color: "var(--maize)", marginTop: "0.15rem" }}>
              {value}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}