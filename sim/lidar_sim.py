from __future__ import annotations

import argparse
import json
import math
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path


DEFAULT_COURSE_JSON = Path(__file__).with_name("bb_course_fall2026.json")
WALL_HEIGHT_IN = 2.0

MARGIN_PX = 36
PIXELS_PER_INCH = 8

LIDAR_FOV_DEG = 120
LIDAR_RES_DEG = 10
LIDAR_MAX_RANGE_IN = 240.0

COLOR_GRID = "#d1d5db"
COLOR_WALL = "#111827"
COLOR_BOT = "#7c3aed"
COLOR_GROUND = "#ef4444"
COLOR_WALL_LOW = "#f97316"   # z < 0.5
COLOR_WALL_MID = "#2563eb"   # 0.5 <= z < 1.0
COLOR_WALL_HIGH = "#16a34a"  # 1.0 <= z <= 2.0


@dataclass(slots=True)
class RobotPose:
	x_in: float = 0.0
	y_in: float = 0.0
	heading_deg: float = 0.0


@dataclass(slots=True)
class LidarConfig:
	x_in: float = 0.0
	y_in: float = 0.0
	z_in: float = 2.0
	yaw_deg: float = 0.0
	pitch_deg: float = 30.0
	roll_deg: float = 0.0


def cumulative_positions(lengths: list[float]) -> list[float]:
	positions = [0.0]
	total = 0.0
	for length in lengths:
		total += float(length)
		positions.append(total)
	return positions


def rotation_zyx(yaw_deg: float, pitch_down_deg: float, roll_deg: float) -> list[list[float]]:
	"""Return Rz(yaw) * Ry(+pitch_down) * Rx(roll).

	pitch is positive downward as requested.
	"""
	psi = math.radians(yaw_deg)
	theta = math.radians(pitch_down_deg)
	phi = math.radians(roll_deg)

	cpsi, spsi = math.cos(psi), math.sin(psi)
	cth, sth = math.cos(theta), math.sin(theta)
	cphi, sphi = math.cos(phi), math.sin(phi)

	rz = [
		[cpsi, -spsi, 0.0],
		[spsi, cpsi, 0.0],
		[0.0, 0.0, 1.0],
	]
	ry = [
		[cth, 0.0, sth],
		[0.0, 1.0, 0.0],
		[-sth, 0.0, cth],
	]
	rx = [
		[1.0, 0.0, 0.0],
		[0.0, cphi, -sphi],
		[0.0, sphi, cphi],
	]

	return matmul(matmul(rz, ry), rx)


def matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
	return [
		[
			a[row][0] * b[0][col] + a[row][1] * b[1][col] + a[row][2] * b[2][col]
			for col in range(3)
		]
		for row in range(3)
	]


def matvec(a: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
	return (
		a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
		a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
		a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
	)


def wrap_deg(angle: float) -> float:
	wrapped = angle % 360.0
	if wrapped > 180.0:
		wrapped -= 360.0
	return wrapped


class LidarSimApp:
	def __init__(self, course_path: Path, show_rays: bool = False) -> None:
		self.course_path = course_path
		self.show_rays = show_rays
		self.payload = json.loads(self.course_path.read_text())

		course = self.payload["course"]
		self.row_heights_in = [float(value) for value in course["row_heights_in"]]
		self.col_widths_in = [float(value) for value in course["col_widths_in"]]
		self.wall_height_in = float(course.get("wall_height_in", WALL_HEIGHT_IN))

		self.row_positions_in = cumulative_positions(self.row_heights_in)
		self.col_positions_in = cumulative_positions(self.col_widths_in)

		# Origin is interior grid point (row=1, col=1): robot starts at x=y=0 there.
		self.origin_x_in = self.row_positions_in[1]
		self.origin_y_in = self.col_positions_in[1]

		self.horizontal_walls = {
			(int(item["row"]), int(item["col"]))
			for item in self.payload.get("horizontal_walls", [])
		}
		self.vertical_walls = {
			(int(item["row"]), int(item["col"]))
			for item in self.payload.get("vertical_walls", [])
		}

		self.lidar_config = self._load_or_create_lidar_config()
		self.robot = RobotPose(0.0, 0.0, 0.0)
		self.lidar_hits: list[tuple[float, float, str]] = []
		self.lidar_rays: list[tuple[float, float, float, float, str]] = []

		self.root = tk.Tk()
		self.root.title("LiDAR Course Simulator")

		canvas_width = int((self.col_positions_in[-1] * PIXELS_PER_INCH) + 2 * MARGIN_PX)
		canvas_height = int((self.row_positions_in[-1] * PIXELS_PER_INCH) + 2 * MARGIN_PX)
		self.canvas = tk.Canvas(
			self.root,
			width=canvas_width,
			height=canvas_height,
			background="white",
			highlightthickness=0,
		)
		self.canvas.pack(fill=tk.BOTH, expand=True)

		self.status_var = tk.StringVar()
		status = tk.Label(self.root, textvariable=self.status_var, anchor="w", padx=10, pady=8)
		status.pack(fill=tk.X)

		self.root.bind("<Up>", self.on_up)
		self.root.bind("<Down>", self.on_down)
		self.root.bind("<Left>", self.on_left)
		self.root.bind("<Right>", self.on_right)

		self.update_lidar_hits()
		self.redraw()

	def _load_or_create_lidar_config(self) -> LidarConfig:
		data = self.payload.get("robot_lidar_config")
		if data is None:
			data = {
				"lidar_position_in_bot_in": {
					"x_in": 0.0,
					"y_in": 0.0,
					"z_in": 1.5,
				},
				"lidar_euler_zyx_deg": {
					"yaw_deg": 0.0,
					"pitch_deg": 15.0,
					"roll_deg": 0.0,
				},
				"notes": {
					"z_zero_reference": "ground_level",
					"pitch_sign": "positive_pitch_points_down",
				},
			}
			self.payload["robot_lidar_config"] = data
			self._save_course()

		position = data.get("lidar_position_in_bot_in", {})
		euler = data.get("lidar_euler_zyx_deg", {})

		return LidarConfig(
			x_in=float(position.get("x_in", 0.0)),
			y_in=float(position.get("y_in", 0.0)),
			z_in=float(position.get("z_in", 1.5)),
			yaw_deg=float(euler.get("yaw_deg", 0.0)),
			pitch_deg=float(euler.get("pitch_deg", 15.0)),
			roll_deg=float(euler.get("roll_deg", 0.0)),
		)

	def _save_course(self) -> None:
		self.course_path.write_text(json.dumps(self.payload, indent=2) + "\n")

	def on_up(self, _event: tk.Event[tk.Misc]) -> None:
		heading = math.radians(self.robot.heading_deg)
		self.robot.x_in += math.cos(heading) * 1.0
		self.robot.y_in += math.sin(heading) * 1.0
		self.update_lidar_hits()
		self.redraw()

	def on_down(self, _event: tk.Event[tk.Misc]) -> None:
		heading = math.radians(self.robot.heading_deg)
		self.robot.x_in -= math.cos(heading) * 1.0
		self.robot.y_in -= math.sin(heading) * 1.0
		self.update_lidar_hits()
		self.redraw()

	def on_left(self, _event: tk.Event[tk.Misc]) -> None:
		self.robot.heading_deg = wrap_deg(self.robot.heading_deg + 45.0)
		self.update_lidar_hits()
		self.redraw()

	def on_right(self, _event: tk.Event[tk.Misc]) -> None:
		self.robot.heading_deg = wrap_deg(self.robot.heading_deg - 45.0)
		self.update_lidar_hits()
		self.redraw()

	def redraw(self) -> None:
		self.canvas.delete("all")
		self.draw_grid()
		self.draw_walls()
		self.draw_bot()
		self.draw_lidar_hits()
		self.status_var.set(
			f"Pose x={self.robot.x_in:.1f} in, y={self.robot.y_in:.1f} in, "
			f"heading={self.robot.heading_deg:.0f} deg | "
			f"rays={'on' if self.show_rays else 'off'} | "
			"Controls: Up/Down move 1 in, Left/Right rotate 45 deg"
		)

	def world_to_canvas(self, x_in: float, y_in: float) -> tuple[float, float]:
		canvas_x = MARGIN_PX + (y_in + self.origin_y_in) * PIXELS_PER_INCH
		canvas_y = MARGIN_PX + (x_in + self.origin_x_in) * PIXELS_PER_INCH
		return canvas_x, canvas_y

	def draw_grid(self) -> None:
		for row_x in self.row_positions_in:
			y_px = MARGIN_PX + row_x * PIXELS_PER_INCH
			self.canvas.create_line(
				MARGIN_PX,
				y_px,
				MARGIN_PX + self.col_positions_in[-1] * PIXELS_PER_INCH,
				y_px,
				fill=COLOR_GRID,
			)

		for col_y in self.col_positions_in:
			x_px = MARGIN_PX + col_y * PIXELS_PER_INCH
			self.canvas.create_line(
				x_px,
				MARGIN_PX,
				x_px,
				MARGIN_PX + self.row_positions_in[-1] * PIXELS_PER_INCH,
				fill=COLOR_GRID,
			)

	def draw_walls(self) -> None:
		for row, col in sorted(self.horizontal_walls):
			x1 = self.row_positions_in[row] - self.origin_x_in
			y1 = self.col_positions_in[col] - self.origin_y_in
			x2 = self.row_positions_in[row] - self.origin_x_in
			y2 = self.col_positions_in[col + 1] - self.origin_y_in
			cx1, cy1 = self.world_to_canvas(x1, y1)
			cx2, cy2 = self.world_to_canvas(x2, y2)
			self.canvas.create_line(cx1, cy1, cx2, cy2, fill=COLOR_WALL, width=3)

		for row, col in sorted(self.vertical_walls):
			x1 = self.row_positions_in[row] - self.origin_x_in
			y1 = self.col_positions_in[col] - self.origin_y_in
			x2 = self.row_positions_in[row + 1] - self.origin_x_in
			y2 = self.col_positions_in[col] - self.origin_y_in
			cx1, cy1 = self.world_to_canvas(x1, y1)
			cx2, cy2 = self.world_to_canvas(x2, y2)
			self.canvas.create_line(cx1, cy1, cx2, cy2, fill=COLOR_WALL, width=3)

	def draw_bot(self) -> None:
		cx, cy = self.world_to_canvas(self.robot.x_in, self.robot.y_in)
		r = 6
		self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill=COLOR_BOT, outline="")
		head = math.radians(self.robot.heading_deg)
		tip_x, tip_y = self.world_to_canvas(
			self.robot.x_in + 3.0 * math.cos(head),
			self.robot.y_in + 3.0 * math.sin(head),
		)
		self.canvas.create_line(cx, cy, tip_x, tip_y, fill=COLOR_BOT, width=3)

	def draw_lidar_hits(self) -> None:
		if self.show_rays:
			for o_x, o_y, x_hit, y_hit, color in self.lidar_rays:
				start_x, start_y = self.world_to_canvas(o_x, o_y)
				end_x, end_y = self.world_to_canvas(x_hit, y_hit)
				self.canvas.create_line(
					start_x,
					start_y,
					end_x,
					end_y,
					fill=color,
					width=1,
					dash=(4, 3),
				)

		for x_hit, y_hit, color in self.lidar_hits:
			cx, cy = self.world_to_canvas(x_hit, y_hit)
			r = 3
			self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill=color, outline="")

	def update_lidar_hits(self) -> None:
		self.lidar_hits.clear()
		self.lidar_rays.clear()

		angles = [
			(-LIDAR_FOV_DEG / 2.0) + step * LIDAR_RES_DEG
			for step in range(int(LIDAR_FOV_DEG / LIDAR_RES_DEG) + 1)
		]

		for azimuth_deg in angles:
			hit = self.cast_ray(azimuth_deg)
			if hit is not None:
				o_x, o_y, x_hit, y_hit, color = hit
				self.lidar_hits.append((x_hit, y_hit, color))
				self.lidar_rays.append((o_x, o_y, x_hit, y_hit, color))

	def cast_ray(self, azimuth_deg: float) -> tuple[float, float, float, float, str] | None:
		# Lidar origin in world coordinates.
		head = math.radians(self.robot.heading_deg)
		o_x = self.robot.x_in + self.lidar_config.x_in * math.cos(head) - self.lidar_config.y_in * math.sin(head)
		o_y = self.robot.y_in + self.lidar_config.x_in * math.sin(head) + self.lidar_config.y_in * math.cos(head)
		o_z = self.lidar_config.z_in

		# Ray direction in lidar frame for horizontal scan.
		az = math.radians(azimuth_deg)
		d_lidar = (math.cos(az), math.sin(az), 0.0)

		r_lidar_to_robot = rotation_zyx(
			self.lidar_config.yaw_deg,
			self.lidar_config.pitch_deg,
			self.lidar_config.roll_deg,
		)
		d_robot = matvec(r_lidar_to_robot, d_lidar)

		# Robot heading rotates around +z in course frame.
		r_heading = rotation_zyx(self.robot.heading_deg, 0.0, 0.0)
		d_world = matvec(r_heading, d_robot)

		dx, dy, dz = d_world
		norm = math.sqrt(dx * dx + dy * dy + dz * dz)
		if norm <= 1e-9:
			return None
		dx, dy, dz = dx / norm, dy / norm, dz / norm

		best_t = LIDAR_MAX_RANGE_IN
		best_hit: tuple[float, float, float, float, str] | None = None

		# Ground hit.
		if dz < -1e-9 and o_z > 0.0:
			t_ground = -o_z / dz
			if 0.0 < t_ground <= best_t:
				x_hit = o_x + t_ground * dx
				y_hit = o_y + t_ground * dy
				best_t = t_ground
				best_hit = (o_x, o_y, x_hit, y_hit, COLOR_GROUND)

		# Wall hits.
		for wall in self.iter_wall_segments():
			t_wall = self.intersect_wall_2d(o_x, o_y, dx, dy, wall)
			if t_wall is None:
				continue
			if not (0.0 < t_wall <= best_t):
				continue

			z_hit = o_z + t_wall * dz
			if z_hit < 0.0 or z_hit > self.wall_height_in:
				continue

			if z_hit < 0.5:
				color = COLOR_WALL_LOW
			elif z_hit < 1.0:
				color = COLOR_WALL_MID
			else:
				color = COLOR_WALL_HIGH

			x_hit = o_x + t_wall * dx
			y_hit = o_y + t_wall * dy
			best_t = t_wall
			best_hit = (o_x, o_y, x_hit, y_hit, color)

		return best_hit

	def iter_wall_segments(self) -> list[tuple[float, float, float, float]]:
		segments: list[tuple[float, float, float, float]] = []
		for row, col in self.horizontal_walls:
			x = self.row_positions_in[row] - self.origin_x_in
			y1 = self.col_positions_in[col] - self.origin_y_in
			y2 = self.col_positions_in[col + 1] - self.origin_y_in
			segments.append((x, y1, x, y2))

		for row, col in self.vertical_walls:
			x1 = self.row_positions_in[row] - self.origin_x_in
			x2 = self.row_positions_in[row + 1] - self.origin_x_in
			y = self.col_positions_in[col] - self.origin_y_in
			segments.append((x1, y, x2, y))
		return segments

	@staticmethod
	def intersect_wall_2d(
		o_x: float,
		o_y: float,
		dx: float,
		dy: float,
		segment: tuple[float, float, float, float],
	) -> float | None:
		x1, y1, x2, y2 = segment
		eps = 1e-9

		if abs(x1 - x2) < eps:
			# x = const segment
			if abs(dx) < eps:
				return None
			t = (x1 - o_x) / dx
			if t <= 0.0:
				return None
			y = o_y + t * dy
			if min(y1, y2) - eps <= y <= max(y1, y2) + eps:
				return t
			return None

		if abs(y1 - y2) < eps:
			# y = const segment
			if abs(dy) < eps:
				return None
			t = (y1 - o_y) / dy
			if t <= 0.0:
				return None
			x = o_x + t * dx
			if min(x1, x2) - eps <= x <= max(x1, x2) + eps:
				return t
			return None

		return None

	def run(self) -> None:
		self.root.mainloop()


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="LiDAR simulator over course JSON")
	parser.add_argument(
		"--course",
		type=Path,
		default=DEFAULT_COURSE_JSON,
		help="Path to course JSON file.",
	)
	parser.add_argument(
		"--rays",
		action="store_true",
		help="Show dashed lidar rays for returns that hit ground or walls.",
	)
	return parser.parse_args()


def main() -> None:
	args = parse_args()
	LidarSimApp(args.course, show_rays=args.rays).run()


if __name__ == "__main__":
	main()
