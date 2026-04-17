import { useEffect, useRef } from "react";
import { Mesh, Program, Renderer, Triangle } from "ogl";

import "./LiquidChrome.css";

type LiquidChromeProps = {
  baseColor?: [number, number, number];
  speed?: number;
  amplitude?: number;
  frequencyX?: number;
  frequencyY?: number;
  interactive?: boolean;
  renderScale?: number;
  maxFps?: number;
  className?: string;
};

function LiquidChrome({
  baseColor = [0.1, 0.1, 0.1],
  speed = 0.2,
  amplitude = 0.3,
  frequencyX = 3,
  frequencyY = 3,
  interactive = true,
  renderScale = 1,
  maxFps = 60,
  className,
}: LiquidChromeProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    const container = containerRef.current;

    if (!container) {
      return;
    }

    const element = container;

    const renderer = new Renderer({ antialias: false, powerPreference: "low-power" });
    const gl = renderer.gl;
    gl.clearColor(1, 1, 1, 1);

    const vertexShader = `
      attribute vec2 position;
      attribute vec2 uv;
      varying vec2 vUv;
      void main() {
        vUv = uv;
        gl_Position = vec4(position, 0.0, 1.0);
      }
    `;

    const fragmentShader = `
      precision mediump float;
      uniform float uTime;
      uniform vec3 uResolution;
      uniform vec3 uBaseColor;
      uniform float uAmplitude;
      uniform float uFrequencyX;
      uniform float uFrequencyY;
      uniform vec2 uMouse;
      varying vec2 vUv;

      vec4 renderImage(vec2 uvCoord) {
          vec2 fragCoord = uvCoord * uResolution.xy;
          vec2 uv = (2.0 * fragCoord - uResolution.xy) / min(uResolution.x, uResolution.y);

          for (float i = 1.0; i < 4.0; i++){
              uv.x += uAmplitude / i * cos(i * uFrequencyX * uv.y + uTime + uMouse.x * 3.14159);
              uv.y += uAmplitude / i * cos(i * uFrequencyY * uv.x + uTime + uMouse.y * 3.14159);
          }

          vec3 color = uBaseColor / abs(sin(uTime - uv.y - uv.x));
          return vec4(color, 1.0);
      }

      void main() {
          gl_FragColor = renderImage(vUv);
      }
    `;

    const geometry = new Triangle(gl);
    const program = new Program(gl, {
      vertex: vertexShader,
      fragment: fragmentShader,
      uniforms: {
        uTime: { value: 0 },
        uResolution: {
          value: new Float32Array([gl.canvas.width, gl.canvas.height, gl.canvas.width / gl.canvas.height]),
        },
        uBaseColor: { value: new Float32Array(baseColor) },
        uAmplitude: { value: amplitude },
        uFrequencyX: { value: frequencyX },
        uFrequencyY: { value: frequencyY },
        uMouse: { value: new Float32Array([0, 0]) },
      },
    });
    const mesh = new Mesh(gl, { geometry, program });

    function resize() {
      const visualWidth = Math.max(1, element.offsetWidth);
      const visualHeight = Math.max(1, element.offsetHeight);
      const bufferWidth = Math.max(1, Math.floor(visualWidth * renderScale));
      const bufferHeight = Math.max(1, Math.floor(visualHeight * renderScale));

      renderer.setSize(bufferWidth, bufferHeight);
      gl.canvas.style.width = `${visualWidth}px`;
      gl.canvas.style.height = `${visualHeight}px`;

      const resolution = program.uniforms.uResolution.value as Float32Array;
      resolution[0] = gl.canvas.width;
      resolution[1] = gl.canvas.height;
      resolution[2] = gl.canvas.width / gl.canvas.height;
    }

    function handleMouseMove(event: MouseEvent) {
      const rect = element.getBoundingClientRect();
      const x = (event.clientX - rect.left) / rect.width;
      const y = 1 - (event.clientY - rect.top) / rect.height;
      const mouse = program.uniforms.uMouse.value as Float32Array;
      mouse[0] = x;
      mouse[1] = y;
    }

    function handleTouchMove(event: TouchEvent) {
      if (!event.touches.length) {
        return;
      }

      const touch = event.touches[0];
      const rect = element.getBoundingClientRect();
      const x = (touch.clientX - rect.left) / rect.width;
      const y = 1 - (touch.clientY - rect.top) / rect.height;
      const mouse = program.uniforms.uMouse.value as Float32Array;
      mouse[0] = x;
      mouse[1] = y;
    }

    window.addEventListener("resize", resize);
    resize();

    if (interactive) {
      element.addEventListener("mousemove", handleMouseMove);
      element.addEventListener("touchmove", handleTouchMove);
    }

    let animationId = 0;
    let lastRenderTime = 0;
    const minFrameTime = 1000 / Math.max(1, maxFps);

    function update(tick: number) {
      animationId = requestAnimationFrame(update);
      if (tick - lastRenderTime < minFrameTime) {
        return;
      }
      lastRenderTime = tick;
      program.uniforms.uTime.value = tick * 0.001 * speed;
      renderer.render({ scene: mesh });
    }

    animationId = requestAnimationFrame(update);

    gl.canvas.classList.add("liquidChrome-canvas");
    element.appendChild(gl.canvas);

    return () => {
      cancelAnimationFrame(animationId);
      window.removeEventListener("resize", resize);

      if (interactive) {
        element.removeEventListener("mousemove", handleMouseMove);
        element.removeEventListener("touchmove", handleTouchMove);
      }

      if (gl.canvas.parentElement) {
        gl.canvas.parentElement.removeChild(gl.canvas);
      }

      gl.getExtension("WEBGL_lose_context")?.loseContext();
    };
  }, [baseColor, speed, amplitude, frequencyX, frequencyY, interactive, renderScale, maxFps]);

  return <div ref={containerRef} className={`liquidChrome-container ${className ?? ""}`.trim()} />;
}

export default LiquidChrome;
