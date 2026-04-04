import { useEffect, useRef, useState } from "react";
import { subscribeToValue } from "../nt";
import { type TopicConfig } from "../topics";
import fieldImage from "../assets/field_image.png";

const FIELD_WIDTH_M = 16.54;
const FIELD_HEIGHT_M = 8.07;
const G = 32.2; // ft/s²
const TURRET_LINE_LENGTH = 40; // pixels, length of turret aim lines on top-down view
const TURRET_HEIGHT_FT = 1.8;

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
  turretAngleDeg: number;
}

interface TurretLimits {
  min: number;
  max: number;
}

function computeArcPoints(
  exitVelocityFtps: number,
  exitAngleDeg: number,
  targetHeightFt: number,
  steps = 80
): { x: number; y: number }[] {
  const angleRad = (exitAngleDeg * Math.PI) / 180;
  const vx = exitVelocityFtps * Math.cos(angleRad);
  const vy = exitVelocityFtps * Math.sin(angleRad);

  // Delta height the projectile needs to travel from launch point to target
  const deltaZ = targetHeightFt - TURRET_HEIGHT_FT;

  // Solve for flight time: deltaZ = vy*t - 0.5*G*t^2
  // 0.5G*t^2 - vy*t + deltaZ = 0
  const disc = vy * vy - 2 * G * deltaZ;
  const tFlight = disc >= 0
    ? (vy + Math.sqrt(disc)) / G
    : (vy * 2) / G;

  const points: { x: number; y: number }[] = [];
  for (let i = 0; i <= steps; i++) {
    const t = (i / steps) * tFlight;
    // y is offset by turret launch height so arc starts and ends at correct heights
    points.push({
      x: vx * t,
      y: TURRET_HEIGHT_FT + vy * t - 0.5 * G * t * t,
    });
  }
  return points;
}


function parseShotData(arr: number[]): ShotData {
  return {
    robotX: arr[0] ?? 0,
    robotY: arr[1] ?? 0,
    robotHeading: arr[2] ?? 0,
    targetX: arr[3] ?? 0,
    targetY: arr[4] ?? 0,
    targetZ: arr[5] ?? 0,
    exitAngleDeg: arr[6] ?? 0,
    exitVelocityFtps: arr[7] ?? 0,
    hDistanceFt: arr[8] ?? 0,
    fieldAngleDeg: arr[9] ?? 0,
    turretAngleDeg: arr[10] ?? 0,
  };
}

// Draws a turret aim line from robot center in a given direction
function drawTurretLine(
  ctx: CanvasRenderingContext2D,
  rx: number,
  ry: number,
  robotHeadingDeg: number,
  turretAngleDeg: number,
  color: string,
  label: string,
  W: number,
  H: number,
) {
  // Turret angle is relative to robot, robot heading is field-relative
  const absoluteAngleRad = ((robotHeadingDeg + turretAngleDeg) * Math.PI) / 180;

  // Project a long line to the edge of canvas to show aim direction
  const reach = Math.max(W, H);
  const ex = rx + Math.cos(absoluteAngleRad) * reach;
  const ey = ry - Math.sin(absoluteAngleRad) * reach;

  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.setLineDash([5, 4]);
  ctx.globalAlpha = 0.7;
  ctx.beginPath();
  ctx.moveTo(rx, ry);
  ctx.lineTo(ex, ey);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.globalAlpha = 1;

  // Dot at tip of short line for clarity
  const tipX = rx + Math.cos(absoluteAngleRad) * TURRET_LINE_LENGTH;
  const tipY = ry - Math.sin(absoluteAngleRad) * TURRET_LINE_LENGTH;
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(tipX, tipY, 3, 0, Math.PI * 2);
  ctx.fill();

  // Label
  ctx.font = "bold 10px 'Share Tech Mono', monospace";
  ctx.fillStyle = color;
  ctx.fillText(label, tipX + 5, tipY - 3);
  ctx.restore();
}

// Draws the turret range-of-motion arc around the robot
function drawTurretLimitArc(
  ctx: CanvasRenderingContext2D,
  rx: number,
  ry: number,
  robotHeadingDeg: number,
  limits: TurretLimits,
) {
  const radius = TURRET_LINE_LENGTH + 8;
  const headingRad = (robotHeadingDeg * Math.PI) / 180;

  // Canvas angles: subtract because canvas Y is flipped
  // Also canvas arc goes clockwise, field angles go counterclockwise
  const startRad = -(headingRad + (limits.max * Math.PI) / 180);
  const endRad = -(headingRad + (limits.min * Math.PI) / 180);

  ctx.save();
  ctx.strokeStyle = "rgba(180, 180, 180, 0.35)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(rx, ry, radius, startRad, endRad, false);
  ctx.stroke();

  // Hard stop tick marks
  [limits.min, limits.max].forEach(limitDeg => {
    const angleRad = headingRad + (limitDeg * Math.PI) / 180;
    const ix = rx + Math.cos(angleRad) * (radius - 5);
    const iy = ry - Math.sin(angleRad) * (radius - 5);
    const ox = rx + Math.cos(angleRad) * (radius + 5);
    const oy = ry - Math.sin(angleRad) * (radius + 5);
    ctx.strokeStyle = "rgba(255, 80, 80, 0.6)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(ix, iy);
    ctx.lineTo(ox, oy);
    ctx.stroke();
  });

  ctx.restore();
}

export default function TrajectoryWidget({ topics }: { topics: TopicConfig[] }) {
  const topDownRef = useRef<HTMLCanvasElement>(null);
  const sideRef = useRef<HTMLCanvasElement>(null);
  const fieldImgRef = useRef<HTMLImageElement | null>(null);
  const [fieldLoaded, setFieldLoaded] = useState(false);

  const defaultData = parseShotData(topics[0]?.defaultValue as number[] ?? []);
  const [calcShot, setCalcShot] = useState<ShotData>(defaultData);
  const [actualShot, setActualShot] = useState<ShotData>(defaultData);
  const [turretLimits, setTurretLimits] = useState<TurretLimits>({ min: -298, max: 135 });

  useEffect(() => {
    const img = new Image();
    img.src = fieldImage;
    img.onload = () => { fieldImgRef.current = img; setFieldLoaded(true); };
  }, []);

  // Subscribe to both shot topics and turret limits
  useEffect(() => {
    topics.forEach(topic => {
      subscribeToValue(topic.key, (value) => {
        if (!Array.isArray(value) || value.length < 10) return;
        if (topic.key.includes("Calculated") || topic.key.includes("calculated")) {
          setCalcShot(parseShotData(value));
        } else {
          setActualShot(parseShotData(value));
        }
      });
    });

    subscribeToValue("/dashboard/turretLimits", (value) => {
      if (Array.isArray(value) && value.length >= 2) {
        setTurretLimits({ min: value[0], max: value[1] });
      }
    });
  }, [topics]);

  // ── Top-down view ────────────────────────────────────────────────────────────
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

    const toX = (mx: number) => (mx / FIELD_WIDTH_M) * W;
    const toY = (my: number) => H - (my / FIELD_HEIGHT_M) * H;

    const rx = toX(calcShot.robotX);
    const ry = toY(calcShot.robotY);
    const tx = toX(calcShot.targetX);
    const ty = toY(calcShot.targetY);

    // Helper: draw projected arc onto field top-down plane
    const drawTopDownArc = (shot: ShotData, color: string) => {
      const arcPoints = computeArcPoints(
        shot.exitVelocityFtps,
        shot.exitAngleDeg,
        shot.targetZ,
      );
      const totalDistFt = arcPoints[arcPoints.length - 1].x || 1;
      const totalDistM = totalDistFt / 3.281;
      const angleRad = (shot.fieldAngleDeg * Math.PI) / 180;

      ctx.beginPath();
      arcPoints.forEach((pt, i) => {
        const frac = pt.x / totalDistFt;
        const cx = rx + Math.cos(angleRad) * frac * totalDistM * (W / FIELD_WIDTH_M);
        const cy = ry - Math.sin(angleRad) * frac * totalDistM * (H / FIELD_HEIGHT_M);
        i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy);
      });
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
    };

    // Draw arcs — actual first so calculated renders on top
    drawTopDownArc(actualShot, "rgba(0, 210, 255, 0.75)");  // cyan = actual
    drawTopDownArc(calcShot, "rgba(255, 203, 5, 0.85)");  // maize = calculated

    // Turret limit arc
    drawTurretLimitArc(ctx, rx, ry, calcShot.robotHeading, turretLimits);

    // Turret aim lines
    drawTurretLine(ctx, rx, ry, calcShot.robotHeading, calcShot.turretAngleDeg, "rgba(255, 203, 5, 0.9)", "CALC", W, H);
    drawTurretLine(ctx, rx, ry, actualShot.robotHeading, actualShot.turretAngleDeg, "rgba(0, 210, 255, 0.9)", "ACTL", W, H);

    // Robot body
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(-(calcShot.robotHeading * Math.PI) / 180);
    ctx.fillStyle = "rgba(30, 80, 160, 0.9)";
    ctx.strokeStyle = "white";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.rect(-10, -10, 20, 20);
    ctx.fill();
    ctx.stroke();
    // Heading arrow
    ctx.fillStyle = "white";
    ctx.beginPath();
    ctx.moveTo(0, -18);
    ctx.lineTo(-5, -10);
    ctx.lineTo(5, -10);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    // Target dot
    ctx.save();
    ctx.translate(tx, ty);
    ctx.fillStyle = "rgba(255, 203, 5, 0.95)";
    ctx.strokeStyle = "#00274C";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.restore();

    // Labels
    ctx.font = "bold 10px 'Share Tech Mono', monospace";
    ctx.fillStyle = "#00274C";
    ctx.fillText("ROBOT", rx + 13, ry + 4);
    ctx.fillText("TARGET", tx + 10, ty + 4);

    // Legend
    const legendX = 8;
    let legendY = 16;
    [
      { color: "rgba(255, 203, 5, 0.9)", label: "Calculated" },
      { color: "rgba(0, 210, 255, 0.9)", label: "Actual" },
      { color: "rgba(180,180,180,0.5)", label: "Turret range" },
      { color: "rgba(255, 80, 80, 0.7)", label: "Hard stops" },
    ].forEach(({ color, label }) => {
      ctx.fillStyle = color;
      ctx.fillRect(legendX, legendY - 8, 14, 3);
      ctx.fillStyle = "#00274C";
      ctx.font = "9px 'Share Tech Mono', monospace";
      ctx.fillText(label, legendX + 18, legendY);
      legendY += 14;
    });

  }, [calcShot, actualShot, turretLimits, fieldLoaded]);

  // ── Side profile view ────────────────────────────────────────────────────────
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

    const PAD = { top: 24, right: 20, bottom: 30, left: 44 };
    const plotW = W - PAD.left - PAD.right;
    const plotH = H - PAD.top - PAD.bottom;

    // Background
    ctx.fillStyle = "#040d1a";
    ctx.fillRect(0, 0, W, H);

    const targetZFt = calcShot.targetZ;
    const calcArc = computeArcPoints(calcShot.exitVelocityFtps, calcShot.exitAngleDeg, targetZFt);
    const actualArc = computeArcPoints(actualShot.exitVelocityFtps, actualShot.exitAngleDeg, targetZFt);

    const allPoints = [...calcArc, ...actualArc];
    const maxX = Math.max(...allPoints.map(p => p.x), 1);
    const maxY = Math.max(...allPoints.map(p => p.y), targetZFt, 1) * 1.2;

    const toPlotX = (ft: number) => PAD.left + (ft / maxX) * plotW;
    const toPlotY = (ft: number) => PAD.top + plotH - (ft / maxY) * plotH;

    // Grid
    ctx.strokeStyle = "rgba(255,203,5,0.06)";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 5; i++) {
      const y = PAD.top + (i / 5) * plotH;
      ctx.beginPath(); ctx.moveTo(PAD.left, y); ctx.lineTo(PAD.left + plotW, y); ctx.stroke();
    }
    for (let i = 0; i <= 6; i++) {
      const x = PAD.left + (i / 6) * plotW;
      ctx.beginPath(); ctx.moveTo(x, PAD.top); ctx.lineTo(x, PAD.top + plotH); ctx.stroke();
    }

    // Floor
    ctx.strokeStyle = "rgba(255,203,5,0.2)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(PAD.left, toPlotY(0));
    ctx.lineTo(PAD.left + plotW, toPlotY(0));
    ctx.stroke();

    // Draw one arc with fill + stroke
    const drawSideArc = (
      points: { x: number; y: number }[],
      strokeColor: string,
      fillColor: string,
    ) => {
      // Gradient fill
      const grad = ctx.createLinearGradient(0, PAD.top, 0, PAD.top + plotH);
      grad.addColorStop(0, fillColor);
      grad.addColorStop(1, "rgba(0,0,0,0)");

      ctx.beginPath();
      points.forEach((pt, i) => {
        i === 0
          ? ctx.moveTo(toPlotX(pt.x), toPlotY(pt.y))
          : ctx.lineTo(toPlotX(pt.x), toPlotY(pt.y));
      });
      ctx.lineTo(toPlotX(points[points.length - 1].x), toPlotY(0));
      ctx.lineTo(toPlotX(0), toPlotY(0));
      ctx.closePath();
      ctx.fillStyle = grad;
      ctx.fill();

      // Arc line
      ctx.beginPath();
      points.forEach((pt, i) => {
        i === 0
          ? ctx.moveTo(toPlotX(pt.x), toPlotY(pt.y))
          : ctx.lineTo(toPlotX(pt.x), toPlotY(pt.y));
      });
      ctx.strokeStyle = strokeColor;
      ctx.lineWidth = 2.5;
      ctx.stroke();
    };

    // Actual arc behind calculated
    drawSideArc(actualArc, "rgba(0, 210, 255, 0.9)", "rgba(0, 210, 255, 0.12)");
    drawSideArc(calcArc, "rgba(255, 203, 5, 0.9)", "rgba(255, 203, 5, 0.12)");

    // Target height marker
    ctx.strokeStyle = "rgba(255, 80, 80, 0.7)";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(toPlotX(maxX), toPlotY(0));
    ctx.lineTo(toPlotX(maxX), toPlotY(targetZFt));
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = "#ff5050";
    ctx.beginPath();
    ctx.arc(toPlotX(maxX), toPlotY(targetZFt), 5, 0, Math.PI * 2);
    ctx.fill();

    // Launch dot — positioned at turret height, not floor
    ctx.fillStyle = "rgba(0, 120, 255, 0.9)";
    ctx.beginPath();
    ctx.arc(toPlotX(0), toPlotY(TURRET_HEIGHT_FT), 5, 0, Math.PI * 2);
    ctx.fill();

    // Turret height tick on the left axis
    ctx.strokeStyle = "rgba(0, 120, 255, 0.4)";
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(PAD.left, toPlotY(TURRET_HEIGHT_FT));
    ctx.lineTo(PAD.left + plotW, toPlotY(TURRET_HEIGHT_FT));
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = "rgba(0,120,255,0.5)";
    ctx.font = "9px 'Share Tech Mono', monospace";
    ctx.fillText(`${TURRET_HEIGHT_FT.toFixed(1)}ft`, PAD.left - 40, toPlotY(TURRET_HEIGHT_FT) + 4);

    // Axis labels
    ctx.fillStyle = "rgba(255,203,5,0.45)";
    ctx.font = "10px 'Share Tech Mono', monospace";
    ctx.fillText(`${maxX.toFixed(0)}ft`, PAD.left + plotW - 22, PAD.top + plotH + 20);
    ctx.fillText(`${maxY.toFixed(0)}ft`, PAD.left - 40, PAD.top + 8);
    ctx.fillText("0", PAD.left - 12, toPlotY(0) + 4);

    // Stats overlay — calculated vs actual
    ctx.font = "bold 10px 'Share Tech Mono', monospace";
    ctx.fillStyle = "rgba(255, 203, 5, 0.8)";
    ctx.fillText(`CALC  θ:${calcShot.exitAngleDeg.toFixed(1)}°  v:${calcShot.exitVelocityFtps.toFixed(1)}fps`, PAD.left + 6, PAD.top + 13);
    ctx.fillStyle = "rgba(0, 210, 255, 0.8)";
    ctx.fillText(`ACTL  θ:${actualShot.exitAngleDeg.toFixed(1)}°  v:${actualShot.exitVelocityFtps.toFixed(1)}fps`, PAD.left + 6, PAD.top + 25);

  }, [calcShot, actualShot]);

  // ── Render ───────────────────────────────────────────────────────────────────
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
          Top Down · Turret Aim
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
          Side Profile · Height
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

      {/* Data readout grid */}
      <div style={{
        display: "grid",
        gridTemplateColumns: "1fr 1fr",
        gap: "0.4rem",
        fontFamily: "var(--text-mono)",
        fontSize: "0.72rem",
      }}>
        {[
          ["EXIT ANGLE", `${calcShot.exitAngleDeg.toFixed(1)}°`, `${actualShot.exitAngleDeg.toFixed(1)}°`],
          ["EXIT VEL", `${calcShot.exitVelocityFtps.toFixed(1)} fps`, `${actualShot.exitVelocityFtps.toFixed(1)} fps`],
          ["TURRET ANGLE", `${calcShot.turretAngleDeg.toFixed(1)}°`, `${actualShot.turretAngleDeg.toFixed(1)}°`],
          ["H DISTANCE", `${calcShot.hDistanceFt.toFixed(1)} ft`, "—"],
        ].map(([label, calc, actual]) => (
          <div key={label} style={{
            padding: "0.4rem 0.6rem",
            background: "var(--bg)",
            border: "1px solid var(--border)",
            borderRadius: "2px",
          }}>
            <div style={{ color: "var(--text-secondary)", fontSize: "0.6rem", letterSpacing: "0.15em", marginBottom: "0.25rem" }}>
              {label}
            </div>
            <div style={{ display: "flex", justifyContent: "space-between" }}>
              <span style={{ color: "var(--maize)" }}>{calc}</span>
              <span style={{ color: "rgba(0, 210, 255, 0.9)" }}>{actual}</span>
            </div>
          </div>
        ))}
      </div>

      {/* Turret error indicator */}
      {(() => {
        const err = Math.abs(calcShot.turretAngleDeg - actualShot.turretAngleDeg);
        const isClose = err < 2;
        return (
          <div style={{
            padding: "0.4rem 0.75rem",
            background: isClose ? "rgba(0,255,120,0.06)" : "rgba(255,80,80,0.06)",
            border: `1px solid ${isClose ? "rgba(0,255,120,0.25)" : "rgba(255,80,80,0.3)"}`,
            borderRadius: "2px",
            display: "flex",
            justifyContent: "space-between",
            alignItems: "center",
            fontFamily: "var(--text-mono)",
            fontSize: "0.72rem",
          }}>
            <span style={{ color: "var(--text-secondary)", letterSpacing: "0.1em" }}>TURRET ERROR</span>
            <span style={{ color: isClose ? "rgba(0,255,120,0.9)" : "#ff5050", fontWeight: "bold" }}>
              {err.toFixed(2)}° {isClose ? "✓ ON TARGET" : "⚠ TRACKING"}
            </span>
          </div>
        );
      })()}
    </div>
  );
}