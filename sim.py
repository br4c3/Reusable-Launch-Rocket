#!/usr/bin/env python3
"""Reusable launch vehicle landing simulation.

This is a small 6DOF vertical-landing model with:
- x/y/z position and velocity
- roll/pitch/yaw attitude and angular rates
- gravity, drag, fuel burn, throttle, engine gimbal, and yaw RCS
- a guidance loop that targets a landing pad
- simple attitude controllers
- optional terminal animation and CSV export

It is intentionally lightweight and uses only the Python standard library.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import sys
import time
from dataclasses import astuple, dataclass
from pathlib import Path


G = 9.80665
RHO0 = 1.225
SCALE_HEIGHT_M = 8500.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class RocketConfig:
    dry_mass_kg: float = 22_000.0
    fuel_mass_kg: float = 8_500.0
    max_thrust_n: float = 760_000.0
    isp_s: float = 300.0
    drag_area_m2: float = 13.0
    drag_coefficient: float = 0.82
    ixx_kgm2: float = 0.85e6
    iyy_kgm2: float = 0.85e6
    izz_kgm2: float = 0.18e6
    engine_arm_m: float = 18.0
    max_gimbal_rad: float = math.radians(12.0)
    max_tilt_rad: float = math.radians(18.0)
    max_yaw_torque_nm: float = 55_000.0


@dataclass
class RocketState:
    t: float = 0.0
    x_m: float = 480.0
    y_m: float = -260.0
    z_m: float = 1800.0
    vx_mps: float = -42.0
    vy_mps: float = 24.0
    vz_mps: float = -78.0
    roll_rad: float = math.radians(-3.0)
    pitch_rad: float = math.radians(4.0)
    yaw_rad: float = math.radians(8.0)
    p_radps: float = 0.0
    q_radps: float = 0.0
    r_radps: float = 0.0
    fuel_kg: float = 8_500.0


@dataclass
class Control:
    throttle: float
    gimbal_roll_rad: float
    gimbal_pitch_rad: float
    yaw_torque_nm: float
    target_roll_rad: float
    target_pitch_rad: float
    target_yaw_rad: float
    target_vx_mps: float
    target_vy_mps: float
    target_vz_mps: float


@dataclass
class Sample:
    t: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    p_degps: float
    q_degps: float
    r_degps: float
    fuel: float
    throttle: float
    gimbal_roll_deg: float
    gimbal_pitch_deg: float


class LandingController:
    """Closed-loop landing controller tuned for this simple model."""

    def __init__(self, config: RocketConfig) -> None:
        self.config = config

    def command(self, state: RocketState, mass_kg: float) -> Control:
        # Horizontal guidance: bleed lateral position into lateral velocity.
        target_vx = clamp(-0.05 * state.x_m, -42.0, 42.0)
        target_vy = clamp(-0.05 * state.y_m, -42.0, 42.0)
        ax_cmd = clamp(-0.035 * state.x_m - 0.72 * state.vx_mps, -4.0, 4.0)
        ay_cmd = clamp(-0.035 * state.y_m - 0.72 * state.vy_mps, -4.0, 4.0)
        target_pitch = clamp(math.asin(clamp(ax_cmd / G, -0.45, 0.45)), -self.config.max_tilt_rad, self.config.max_tilt_rad)
        target_roll = clamp(-math.asin(clamp(ay_cmd / G, -0.45, 0.45)), -self.config.max_tilt_rad, self.config.max_tilt_rad)
        target_yaw = 0.0

        # Vertical guidance: descend quickly up high, then converge to a soft touchdown.
        if state.z_m > 450.0:
            target_vz = -65.0
        else:
            # A square-root braking curve gives a reasonable suicide-burn style profile.
            target_vz = -max(0.6, min(24.0, 0.65 * math.sqrt(max(state.z_m, 0.0))))

        az_cmd = 1.6 * (target_vz - state.vz_mps)
        tilt_loss = max(math.cos(state.roll_rad) * math.cos(state.pitch_rad), 0.2)
        required_thrust = mass_kg * (G + az_cmd) / tilt_loss
        throttle = clamp(required_thrust / self.config.max_thrust_n, 0.0, 1.0)

        # Attitude loops. Roll and pitch use engine gimbal; yaw uses a small RCS torque.
        roll_error = target_roll - state.roll_rad
        pitch_error = target_pitch - state.pitch_rad
        yaw_error = (target_yaw - state.yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
        roll_torque_cmd = 6.0e6 * roll_error - 3.0e6 * state.p_radps
        pitch_torque_cmd = 6.0e6 * pitch_error - 3.0e6 * state.q_radps
        yaw_torque = clamp(1.4e5 * yaw_error - 7.0e4 * state.r_radps, -self.config.max_yaw_torque_nm, self.config.max_yaw_torque_nm)
        thrust = max(throttle * self.config.max_thrust_n, 1.0)
        gimbal_roll = clamp(roll_torque_cmd / (thrust * self.config.engine_arm_m), -self.config.max_gimbal_rad, self.config.max_gimbal_rad)
        gimbal_pitch = clamp(-pitch_torque_cmd / (thrust * self.config.engine_arm_m), -self.config.max_gimbal_rad, self.config.max_gimbal_rad)

        return Control(throttle, gimbal_roll, gimbal_pitch, yaw_torque, target_roll, target_pitch, target_yaw, target_vx, target_vy, target_vz)


def air_density(altitude_m: float) -> float:
    return RHO0 * math.exp(-max(altitude_m, 0.0) / SCALE_HEIGHT_M)


def step(state: RocketState, control: Control, config: RocketConfig, dt: float) -> RocketState:
    dry_mass = config.dry_mass_kg
    mass = dry_mass + state.fuel_kg
    throttle = control.throttle if state.fuel_kg > 0.0 else 0.0
    thrust = throttle * config.max_thrust_n

    fuel_flow_kgps = thrust / (config.isp_s * G)
    fuel_used = min(state.fuel_kg, fuel_flow_kgps * dt)
    if fuel_flow_kgps > 0.0 and fuel_used < fuel_flow_kgps * dt:
        thrust *= fuel_used / (fuel_flow_kgps * dt)

    thrust_roll = state.roll_rad + control.gimbal_roll_rad
    thrust_pitch = state.pitch_rad + control.gimbal_pitch_rad
    thrust_dir_x = math.sin(thrust_pitch)
    thrust_dir_y = -math.sin(thrust_roll)
    thrust_dir_z = math.cos(thrust_roll) * math.cos(thrust_pitch)
    ax_thrust = thrust * thrust_dir_x / mass
    ay_thrust = thrust * thrust_dir_y / mass
    az_thrust = thrust * thrust_dir_z / mass

    speed = math.sqrt(state.vx_mps * state.vx_mps + state.vy_mps * state.vy_mps + state.vz_mps * state.vz_mps)
    rho = air_density(state.z_m)
    drag_force = 0.5 * rho * speed * speed * config.drag_coefficient * config.drag_area_m2
    if speed > 1e-6:
        ax_drag = -drag_force * state.vx_mps / speed / mass
        ay_drag = -drag_force * state.vy_mps / speed / mass
        az_drag = -drag_force * state.vz_mps / speed / mass
    else:
        ax_drag = ay_drag = az_drag = 0.0

    ax = ax_thrust + ax_drag
    ay = ay_thrust + ay_drag
    az = az_thrust + az_drag - G

    roll_torque = thrust * math.sin(control.gimbal_roll_rad) * config.engine_arm_m
    pitch_torque = -thrust * math.sin(control.gimbal_pitch_rad) * config.engine_arm_m
    yaw_torque = control.yaw_torque_nm
    p_dot = roll_torque / config.ixx_kgm2
    q_dot = pitch_torque / config.iyy_kgm2
    r_dot = yaw_torque / config.izz_kgm2

    return RocketState(
        t=state.t + dt,
        x_m=state.x_m + state.vx_mps * dt + 0.5 * ax * dt * dt,
        y_m=state.y_m + state.vy_mps * dt + 0.5 * ay * dt * dt,
        z_m=max(0.0, state.z_m + state.vz_mps * dt + 0.5 * az * dt * dt),
        vx_mps=state.vx_mps + ax * dt,
        vy_mps=state.vy_mps + ay * dt,
        vz_mps=state.vz_mps + az * dt,
        roll_rad=state.roll_rad + state.p_radps * dt + 0.5 * p_dot * dt * dt,
        pitch_rad=state.pitch_rad + state.q_radps * dt + 0.5 * q_dot * dt * dt,
        yaw_rad=state.yaw_rad + state.r_radps * dt + 0.5 * r_dot * dt * dt,
        p_radps=state.p_radps + p_dot * dt,
        q_radps=state.q_radps + q_dot * dt,
        r_radps=state.r_radps + r_dot * dt,
        fuel_kg=state.fuel_kg - fuel_used,
    )


def make_sample(state: RocketState, control: Control) -> Sample:
    return Sample(
        t=state.t,
        x=state.x_m,
        y=state.y_m,
        z=state.z_m,
        vx=state.vx_mps,
        vy=state.vy_mps,
        vz=state.vz_mps,
        roll_deg=math.degrees(state.roll_rad),
        pitch_deg=math.degrees(state.pitch_rad),
        yaw_deg=math.degrees(state.yaw_rad),
        p_degps=math.degrees(state.p_radps),
        q_degps=math.degrees(state.q_radps),
        r_degps=math.degrees(state.r_radps),
        fuel=state.fuel_kg,
        throttle=control.throttle,
        gimbal_roll_deg=math.degrees(control.gimbal_roll_rad),
        gimbal_pitch_deg=math.degrees(control.gimbal_pitch_rad),
    )


def simulate(config: RocketConfig, initial: RocketState, dt: float, max_time: float) -> tuple[list[Sample], str]:
    controller = LandingController(config)
    state = initial
    samples: list[Sample] = []
    result = "timeout"

    while state.t < max_time:
        mass = config.dry_mass_kg + state.fuel_kg
        control = controller.command(state, mass)
        samples.append(make_sample(state, control))

        if state.z_m <= 0.0:
            lateral_ok = math.hypot(state.x_m, state.y_m) <= 10.0
            vertical_ok = abs(state.vz_mps) <= 3.0
            horizontal_ok = math.hypot(state.vx_mps, state.vy_mps) <= 1.8
            attitude_ok = math.hypot(math.degrees(state.roll_rad), math.degrees(state.pitch_rad)) <= 6.0
            result = "landed" if lateral_ok and vertical_ok and horizontal_ok and attitude_ok else "crashed"
            break

        if state.fuel_kg <= 0.0 and state.z_m > 0.0:
            result = "out_of_fuel"

        state = step(state, control, config, dt)

    if samples and samples[-1].z > 0.0 and state.z_m <= 0.0:
        samples.append(make_sample(state, controller.command(state, config.dry_mass_kg + state.fuel_kg)))

    return samples, result


def render(samples: list[Sample], result: str, realtime: bool, speed: float) -> None:
    columns, rows = shutil.get_terminal_size((100, 32))
    plot_h = max(12, rows - 10)
    plot_w = max(40, columns - 4)
    max_alt = max(sample.z for sample in samples) or 1.0
    max_abs_x = max(max(abs(sample.x) for sample in samples), 50.0)
    stride = 1 if realtime else max(1, len(samples) // 180)

    for sample in samples[::stride]:
        canvas = [[" " for _ in range(plot_w)] for _ in range(plot_h)]
        pad_col = plot_w // 2
        ground_row = plot_h - 1
        for col in range(max(0, pad_col - 4), min(plot_w, pad_col + 5)):
            canvas[ground_row][col] = "="

        col = int(round((sample.x / max_abs_x) * (plot_w * 0.45) + pad_col))
        row = int(round((1.0 - sample.z / max_alt) * (plot_h - 2)))
        col = clamp(col, 0, plot_w - 1)
        row = clamp(row, 0, plot_h - 2)
        symbol = "|" if math.hypot(sample.roll_deg, sample.pitch_deg) < 6 else ("/" if sample.pitch_deg > 0 else "\\")
        canvas[int(row)][int(col)] = symbol

        sys.stdout.write("\033[2J\033[H")
        sys.stdout.write("\n".join("".join(line) for line in canvas))
        sys.stdout.write(
            f"\n t={sample.t:5.1f}s  x={sample.x:7.1f}m  y={sample.y:7.1f}m  z={sample.z:7.1f}m"
            f"  vx={sample.vx:6.1f}m/s  vy={sample.vy:6.1f}m/s  vz={sample.vz:6.1f}m/s"
            f"\n throttle={sample.throttle:4.2f}  gimbal=({sample.gimbal_roll_deg:5.1f}, {sample.gimbal_pitch_deg:5.1f})deg"
            f"  rpy=({sample.roll_deg:5.1f}, {sample.pitch_deg:5.1f}, {sample.yaw_deg:5.1f})deg  fuel={sample.fuel:7.1f}kg"
        )
        sys.stdout.flush()
        if realtime:
            time.sleep(max(0.0, 0.02 / max(speed, 0.1)))

    final = samples[-1]
    sys.stdout.write(
        f"\n\nResult: {result.upper()}  touchdown x={final.x:.2f}m, y={final.y:.2f}m, "
        f"vx={final.vx:.2f}m/s, vy={final.vy:.2f}m/s, vz={final.vz:.2f}m/s, fuel={final.fuel:.1f}kg\n"
    )


def write_csv(samples: list[Sample], path: Path) -> None:
    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(Sample.__dataclass_fields__.keys())
        for sample in samples:
            writer.writerow(astuple(sample))


def configure_matplotlib_cache() -> None:
    os.environ.setdefault("MPLCONFIGDIR", str(Path(".matplotlib-cache").resolve()))
    os.environ.setdefault("XDG_CACHE_HOME", str(Path(".cache").resolve()))


def require_matplotlib() -> tuple[object, object]:
    configure_matplotlib_cache()
    try:
        import matplotlib.pyplot as plt
        from matplotlib import animation
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required for plotting. Run: "
            "python3 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt"
        ) from exc
    return plt, animation


def rocket_axis_from_attitude(roll_deg: float, pitch_deg: float) -> tuple[float, float, float]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    axis = (math.sin(pitch), -math.sin(roll), math.cos(roll) * math.cos(pitch))
    norm = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
    return axis[0] / norm, axis[1] / norm, axis[2] / norm


def cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])
    if norm < 1e-9:
        return 1.0, 0.0, 0.0
    return vector[0] / norm, vector[1] / norm, vector[2] / norm


def build_rocket_mesh(sample: Sample, length_m: float = 90.0, radius_m: float = 7.0, segments: int = 18) -> list[list[tuple[float, float, float]]]:
    base = (sample.x, sample.y, sample.z)
    axis = rocket_axis_from_attitude(sample.roll_deg, sample.pitch_deg)
    yaw = math.radians(sample.yaw_deg)
    yaw_reference = (math.cos(yaw), math.sin(yaw), 0.0)
    dot = axis[0] * yaw_reference[0] + axis[1] * yaw_reference[1] + axis[2] * yaw_reference[2]
    radial_a = normalize(
        (
            yaw_reference[0] - dot * axis[0],
            yaw_reference[1] - dot * axis[1],
            yaw_reference[2] - dot * axis[2],
        )
    )
    radial_b = normalize(cross(axis, radial_a))
    top = (
        base[0] + axis[0] * length_m,
        base[1] + axis[1] * length_m,
        base[2] + axis[2] * length_m,
    )

    base_ring: list[tuple[float, float, float]] = []
    top_ring: list[tuple[float, float, float]] = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        radial = (
            math.cos(angle) * radial_a[0] + math.sin(angle) * radial_b[0],
            math.cos(angle) * radial_a[1] + math.sin(angle) * radial_b[1],
            math.cos(angle) * radial_a[2] + math.sin(angle) * radial_b[2],
        )
        base_ring.append((base[0] + radius_m * radial[0], base[1] + radius_m * radial[1], base[2] + radius_m * radial[2]))
        top_ring.append((top[0] + radius_m * radial[0], top[1] + radius_m * radial[1], top[2] + radius_m * radial[2]))

    faces: list[list[tuple[float, float, float]]] = []
    for index in range(segments):
        next_index = (index + 1) % segments
        faces.append([base_ring[index], base_ring[next_index], top_ring[next_index], top_ring[index]])
    faces.append(list(reversed(base_ring)))
    faces.append(top_ring)
    return faces


def plot_telemetry(samples: list[Sample], result: str, output: Path | None) -> None:
    configure_matplotlib_cache()
    if output and not os.environ.get("MPLBACKEND"):
        import matplotlib

        matplotlib.use("Agg")

    plt, _ = require_matplotlib()
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    times = [sample.t for sample in samples]

    fig = plt.figure(figsize=(12, 8), constrained_layout=True)
    fig.suptitle(f"Reusable Launch Rocket Landing: {result.upper()}")

    ax_traj = fig.add_subplot(2, 2, 1, projection="3d")
    ax_vel = fig.add_subplot(2, 2, 2)
    ax_throttle = fig.add_subplot(2, 2, 3)
    ax_attitude = fig.add_subplot(2, 2, 4)

    ax_traj.plot([sample.x for sample in samples], [sample.y for sample in samples], [sample.z for sample in samples], color="#2f6f9f", linewidth=2.2)
    ax_traj.scatter([0], [0], [0], marker="s", color="#2e7d32", label="landing pad")
    rocket_mesh = Poly3DCollection(build_rocket_mesh(samples[-1]), facecolor="#c44536", edgecolor="#7f1d1d", linewidth=0.35, alpha=0.9)
    ax_traj.add_collection3d(rocket_mesh)
    ax_traj.set_title("3D Trajectory")
    ax_traj.set_xlabel("X (m)")
    ax_traj.set_ylabel("Y (m)")
    ax_traj.set_zlabel("Altitude (m)")
    ax_traj.legend()

    ax_vel.plot(times, [sample.vx for sample in samples], label="vx", color="#6d597a")
    ax_vel.plot(times, [sample.vy for sample in samples], label="vy", color="#287271")
    ax_vel.plot(times, [sample.vz for sample in samples], label="vz", color="#b56576")
    ax_vel.set_title("Velocity")
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel("m/s")
    ax_vel.grid(True, alpha=0.3)
    ax_vel.legend()

    ax_throttle.plot(times, [sample.throttle for sample in samples], color="#287271")
    ax_throttle.set_title("Throttle")
    ax_throttle.set_xlabel("Time (s)")
    ax_throttle.set_ylabel("0-1")
    ax_throttle.set_ylim(-0.05, 1.05)
    ax_throttle.grid(True, alpha=0.3)

    ax_attitude.plot(times, [sample.roll_deg for sample in samples], label="roll", color="#d08c60")
    ax_attitude.plot(times, [sample.pitch_deg for sample in samples], label="pitch", color="#3d5a80")
    ax_attitude.plot(times, [sample.yaw_deg for sample in samples], label="yaw", color="#6d597a")
    ax_attitude.set_title("Attitude")
    ax_attitude.set_xlabel("Time (s)")
    ax_attitude.set_ylabel("Degrees")
    ax_attitude.grid(True, alpha=0.3)
    ax_attitude.legend()

    if output:
        fig.savefig(output, dpi=160)
        print(f"Plot saved to {output}")
    else:
        plt.show()


def animate_trajectory(samples: list[Sample], result: str, output: Path | None) -> None:
    configure_matplotlib_cache()
    if output and not os.environ.get("MPLBACKEND"):
        import matplotlib

        matplotlib.use("Agg")

    plt, animation = require_matplotlib()
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    if animation is None:
        raise SystemExit("matplotlib animation module is unavailable")

    stride = max(1, len(samples) // 360)
    frames = samples[::stride]
    max_alt = max(sample.z for sample in samples) or 1.0
    max_abs_xy = max(max(math.hypot(sample.x, sample.y) for sample in samples), 50.0)

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(f"Landing Animation: {result.upper()}")
    ax.set_xlim(-max_abs_xy * 1.1, max_abs_xy * 1.1)
    ax.set_ylim(-max_abs_xy * 1.1, max_abs_xy * 1.1)
    ax.set_zlim(-20, max_alt * 1.05)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Altitude (m)")
    ax.plot([-12, 12], [0, 0], [0, 0], color="#2e7d32", linewidth=5, solid_capstyle="butt")
    path_line, = ax.plot([], [], [], color="#2f6f9f", linewidth=1.8)
    rocket_body = Poly3DCollection(build_rocket_mesh(frames[0]), facecolor="#c44536", edgecolor="#7f1d1d", linewidth=0.3, alpha=0.92)
    ax.add_collection3d(rocket_body)
    engine_axis, = ax.plot([], [], [], color="#1f2933", linewidth=2.0)
    status = ax.text2D(0.02, 0.96, "", transform=ax.transAxes, va="top")

    def update(frame_index: int) -> tuple[object, object, object, object]:
        sample = frames[frame_index]
        history = frames[: frame_index + 1]
        path_line.set_data([frame.x for frame in history], [frame.y for frame in history])
        path_line.set_3d_properties([frame.z for frame in history])
        rocket_body.set_verts(build_rocket_mesh(sample))
        axis = rocket_axis_from_attitude(sample.roll_deg, sample.pitch_deg)
        engine_axis.set_data([sample.x, sample.x + axis[0] * 110.0], [sample.y, sample.y + axis[1] * 110.0])
        engine_axis.set_3d_properties([sample.z, sample.z + axis[2] * 110.0])
        status.set_text(
            f"t={sample.t:.1f}s  x={sample.x:.1f}m  y={sample.y:.1f}m  z={sample.z:.1f}m\n"
            f"vx={sample.vx:.1f}m/s  vy={sample.vy:.1f}m/s  vz={sample.vz:.1f}m/s  fuel={sample.fuel:.0f}kg\n"
            f"roll={sample.roll_deg:.1f}  pitch={sample.pitch_deg:.1f}  yaw={sample.yaw_deg:.1f}"
        )
        return path_line, rocket_body, engine_axis, status

    ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=30, blit=False)
    if output:
        if output.suffix.lower() == ".gif":
            ani.save(output, writer="pillow", fps=30)
        else:
            ani.save(output, fps=30)
        print(f"Animation saved to {output}")
    else:
        plt.show()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Reusable launch rocket landing simulation")
    parser.add_argument("--x", type=float, default=480.0, help="initial horizontal offset in meters")
    parser.add_argument("--y", type=float, default=-260.0, help="initial crossrange offset in meters")
    parser.add_argument("--altitude", type=float, default=1800.0, help="initial altitude in meters")
    parser.add_argument("--vx", type=float, default=-42.0, help="initial horizontal velocity in m/s")
    parser.add_argument("--vy", type=float, default=24.0, help="initial crossrange velocity in m/s")
    parser.add_argument("--vz", type=float, default=-78.0, help="initial vertical velocity in m/s")
    parser.add_argument("--roll", type=float, default=-3.0, help="initial roll in degrees")
    parser.add_argument("--pitch", type=float, default=4.0, help="initial pitch in degrees")
    parser.add_argument("--yaw", type=float, default=8.0, help="initial yaw in degrees")
    parser.add_argument("--fuel", type=float, default=8500.0, help="initial fuel mass in kg")
    parser.add_argument("--dt", type=float, default=0.05, help="simulation time step in seconds")
    parser.add_argument("--max-time", type=float, default=180.0, help="maximum simulation time in seconds")
    parser.add_argument("--no-render", action="store_true", help="print only the final result")
    parser.add_argument("--realtime", action="store_true", help="animate more slowly")
    parser.add_argument("--speed", type=float, default=1.0, help="animation speed multiplier")
    parser.add_argument("--csv", type=Path, help="write telemetry to a CSV file")
    parser.add_argument("--plot", action="store_true", help="show matplotlib telemetry plots")
    parser.add_argument("--plot-file", type=Path, help="save matplotlib telemetry plots to an image file")
    parser.add_argument("--animate-plot", action="store_true", help="show a matplotlib trajectory animation")
    parser.add_argument("--animation-file", type=Path, help="save matplotlib trajectory animation, for example landing.gif")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    config = RocketConfig(fuel_mass_kg=args.fuel)
    initial = RocketState(
        x_m=args.x,
        y_m=args.y,
        z_m=args.altitude,
        vx_mps=args.vx,
        vy_mps=args.vy,
        vz_mps=args.vz,
        roll_rad=math.radians(args.roll),
        pitch_rad=math.radians(args.pitch),
        yaw_rad=math.radians(args.yaw),
        fuel_kg=args.fuel,
    )
    samples, result = simulate(config, initial, args.dt, args.max_time)

    if args.csv:
        write_csv(samples, args.csv)

    if args.plot or args.plot_file:
        plot_telemetry(samples, result, args.plot_file)

    if args.animate_plot or args.animation_file:
        animate_trajectory(samples, result, args.animation_file)

    if args.no_render or args.plot or args.plot_file or args.animate_plot or args.animation_file:
        final = samples[-1]
        print(
            f"Result: {result.upper()} | t={final.t:.1f}s x={final.x:.2f}m y={final.y:.2f}m z={final.z:.2f}m "
            f"vx={final.vx:.2f}m/s vy={final.vy:.2f}m/s vz={final.vz:.2f}m/s fuel={final.fuel:.1f}kg"
        )
    else:
        render(samples, result, args.realtime, args.speed)

    return 0 if result == "landed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
