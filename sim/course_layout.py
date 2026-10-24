"""Interactive course layout editor.

The course geometry is defined by interior intersection points arranged in a
6-row by 13-column grid. Including the outer border, the course therefore has:

* 8 rows of intersection points
* 15 columns of intersection points
* 7 cell heights, with the bottom-most cell shortened to 6 inches
* 14 cell widths, with the right-most cell shortened to 5 inches

The editor draws the grid, keeps border walls enabled, lets the user toggle
interior wall segments by clicking near edges, and lets the user toggle circle
targets by clicking interior intersection points.
"""

from __future__ import annotations

import argparse
import json
import math
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import messagebox, simpledialog
from typing import Iterable


INNER_POINT_ROWS = 6
INNER_POINT_COLS = 13
DEFAULT_SEGMENT_INCHES = 7
BOTTOM_SEGMENT_INCHES = 6
RIGHT_SEGMENT_INCHES = 5
WALL_HEIGHT_INCHES = 2

MARGIN_PX = 36
PIXELS_PER_INCH = 18
POINT_RADIUS_PX = 5
TARGET_RADIUS_PX = 10
CLICK_POINT_TOLERANCE_PX = 12
CLICK_EDGE_TOLERANCE_PX = 10

GRID_COLOR = "#c9ced6"
POINT_COLOR = "#57606a"
WALL_COLOR = "#111827"
TARGET_COLOR = "#dc2626"
DEFAULT_COURSE_AXES = {
	"x_direction": "down_rows",
	"y_direction": "right_cols",
}
DEFAULT_TARGET_MARKER = {
	"type": "aruco",
	"x_axis_aligned_with_course_x": True,
}
HELP_TEXT = (
	"Click interior edges to toggle walls. Click interior intersections to "
	"toggle circle targets. Press E to save JSON."
)

DEFAULT_OUTPUT_PATH = Path("course.json")


def cumulative_positions(lengths: Iterable[int]) -> list[int]:
	"""Return cumulative positions beginning at zero."""

	positions = [0]
	total = 0
	for length in lengths:
		total += length
		positions.append(total)
	return positions


@dataclass(slots=True)
class CourseGeometry:
	"""Geometry derived from the course specification."""

	row_heights_in: list[int] = field(
		default_factory=lambda: [DEFAULT_SEGMENT_INCHES] * INNER_POINT_ROWS
		+ [BOTTOM_SEGMENT_INCHES]
	)
	col_widths_in: list[int] = field(
		default_factory=lambda: [DEFAULT_SEGMENT_INCHES] * INNER_POINT_COLS
		+ [RIGHT_SEGMENT_INCHES]
	)
	y_positions_in: list[int] = field(init=False)
	x_positions_in: list[int] = field(init=False)

	def __post_init__(self) -> None:
		self.y_positions_in = cumulative_positions(self.row_heights_in)
		self.x_positions_in = cumulative_positions(self.col_widths_in)

	@property
	def point_rows(self) -> int:
		return len(self.y_positions_in)

	@property
	def point_cols(self) -> int:
		return len(self.x_positions_in)

	@property
	def width_in(self) -> int:
		return self.x_positions_in[-1]

	@property
	def height_in(self) -> int:
		return self.y_positions_in[-1]

	def canvas_x(self, col: int) -> int:
		return MARGIN_PX + round(self.x_positions_in[col] * PIXELS_PER_INCH)

	def canvas_y(self, row: int) -> int:
		return MARGIN_PX + round(self.y_positions_in[row] * PIXELS_PER_INCH)


class CourseLayoutApp:
	"""Tkinter application for editing course walls and targets."""

	def __init__(
		self,
		layout_path: Path | None = None,
		output_path: Path = DEFAULT_OUTPUT_PATH,
	) -> None:
		self.layout_path = layout_path
		self.output_path = output_path
		self.geometry = CourseGeometry()
		self.wall_height_in = WALL_HEIGHT_INCHES
		self.course_axes = dict(DEFAULT_COURSE_AXES)
		self.target_marker = dict(DEFAULT_TARGET_MARKER)
		self.horizontal_walls: set[tuple[int, int]] = set()
		self.vertical_walls: set[tuple[int, int]] = set()
		self.targets: dict[tuple[int, int], int] = {}

		if self.layout_path is None:
			self._initialize_border_walls()
		else:
			self.load_layout(self.layout_path)

		self.root = tk.Tk()
		self.root.title("Course Layout Editor")

		canvas_width = self.geometry.canvas_x(self.geometry.point_cols - 1) + MARGIN_PX
		canvas_height = self.geometry.canvas_y(self.geometry.point_rows - 1) + MARGIN_PX

		self.canvas = tk.Canvas(
			self.root,
			width=canvas_width,
			height=canvas_height,
			background="white",
			highlightthickness=0,
		)
		self.canvas.pack(fill=tk.BOTH, expand=True)

		controls = tk.Frame(self.root)
		controls.pack(fill=tk.X, padx=12, pady=(0, 12))

		tk.Button(controls, text="Clear inner walls", command=self.clear_inner_walls).pack(
			side=tk.LEFT
		)
		tk.Button(controls, text="Clear targets", command=self.clear_targets).pack(
			side=tk.LEFT, padx=(8, 0)
		)
		tk.Button(controls, text="Save JSON", command=self.export_layout).pack(
			side=tk.LEFT, padx=(8, 0)
		)

		self.status_var = tk.StringVar()
		status_label = tk.Label(
			self.root,
			textvariable=self.status_var,
			anchor="w",
			justify=tk.LEFT,
			padx=12,
			pady=8,
		)
		status_label.pack(fill=tk.X)

		self.canvas.bind("<Button-1>", self.on_click)
		self.root.bind("<e>", lambda _event: self.export_layout())
		self.root.bind("<E>", lambda _event: self.export_layout())
		self.root.bind("<Escape>", lambda _event: self.root.destroy())

		self.redraw()

	def load_layout(self, layout_path: Path) -> None:
		payload = json.loads(layout_path.read_text())
		course = payload.get("course", {})

		self.geometry = CourseGeometry(
			row_heights_in=course.get(
				"row_heights_in",
				[DEFAULT_SEGMENT_INCHES] * INNER_POINT_ROWS + [BOTTOM_SEGMENT_INCHES],
			),
			col_widths_in=course.get(
				"col_widths_in",
				[DEFAULT_SEGMENT_INCHES] * INNER_POINT_COLS + [RIGHT_SEGMENT_INCHES],
			),
		)
		self.wall_height_in = int(course.get("wall_height_in", WALL_HEIGHT_INCHES))
		self.course_axes = dict(course.get("course_axes", DEFAULT_COURSE_AXES))
		self.target_marker = dict(course.get("target_marker", DEFAULT_TARGET_MARKER))

		self.horizontal_walls = {
			(int(wall["row"]), int(wall["col"]))
			for wall in payload.get("horizontal_walls", [])
		}
		self.vertical_walls = {
			(int(wall["row"]), int(wall["col"]))
			for wall in payload.get("vertical_walls", [])
		}
		self.targets = self._load_targets(payload.get("targets", []))
		self._initialize_border_walls()

	def _load_targets(self, targets: list[dict[str, int]]) -> dict[tuple[int, int], int]:
		loaded_targets: dict[tuple[int, int], int] = {}
		used_ids: set[int] = set()

		for target in targets:
			row = int(target["row"])
			col = int(target["col"])
			target_id = target.get("id")
			if target_id is None:
				target_id = self.next_target_id(used_ids)
			else:
				target_id = int(target_id)
			loaded_targets[(row, col)] = target_id
			used_ids.add(target_id)

		return loaded_targets

	def _initialize_border_walls(self) -> None:
		last_row = self.geometry.point_rows - 1
		last_col = self.geometry.point_cols - 1

		for col in range(last_col):
			self.horizontal_walls.add((0, col))
			self.horizontal_walls.add((last_row, col))

		for row in range(last_row):
			self.vertical_walls.add((row, 0))
			self.vertical_walls.add((row, last_col))

	def clear_inner_walls(self) -> None:
		self.horizontal_walls = {
			(row, col)
			for (row, col) in self.horizontal_walls
			if row in {0, self.geometry.point_rows - 1}
		}
		self.vertical_walls = {
			(row, col)
			for (row, col) in self.vertical_walls
			if col in {0, self.geometry.point_cols - 1}
		}
		self.redraw()

	def clear_targets(self) -> None:
		self.targets.clear()
		self.redraw()

	def next_target_id(self, used_ids: set[int] | None = None) -> int:
		used = set(self.targets.values()) if used_ids is None else set(used_ids)
		candidate = 0
		while candidate in used:
			candidate += 1
		return candidate

	def redraw(self) -> None:
		self.canvas.delete("all")
		self._draw_background_grid()
		self._draw_walls()
		self._draw_points()
		self._draw_targets()
		self._update_status()

	def _draw_background_grid(self) -> None:
		last_row = self.geometry.point_rows - 1
		last_col = self.geometry.point_cols - 1

		for row in range(self.geometry.point_rows):
			y = self.geometry.canvas_y(row)
			self.canvas.create_line(
				self.geometry.canvas_x(0),
				y,
				self.geometry.canvas_x(last_col),
				y,
				fill=GRID_COLOR,
				width=1,
			)

		for col in range(self.geometry.point_cols):
			x = self.geometry.canvas_x(col)
			self.canvas.create_line(
				x,
				self.geometry.canvas_y(0),
				x,
				self.geometry.canvas_y(last_row),
				fill=GRID_COLOR,
				width=1,
			)

	def _draw_walls(self) -> None:
		for row, col in self.horizontal_walls:
			self.canvas.create_line(
				self.geometry.canvas_x(col),
				self.geometry.canvas_y(row),
				self.geometry.canvas_x(col + 1),
				self.geometry.canvas_y(row),
				fill=WALL_COLOR,
				width=4,
			)

		for row, col in self.vertical_walls:
			self.canvas.create_line(
				self.geometry.canvas_x(col),
				self.geometry.canvas_y(row),
				self.geometry.canvas_x(col),
				self.geometry.canvas_y(row + 1),
				fill=WALL_COLOR,
				width=4,
			)

	def _draw_points(self) -> None:
		for row in range(self.geometry.point_rows):
			y = self.geometry.canvas_y(row)
			for col in range(self.geometry.point_cols):
				x = self.geometry.canvas_x(col)
				self.canvas.create_oval(
					x - POINT_RADIUS_PX,
					y - POINT_RADIUS_PX,
					x + POINT_RADIUS_PX,
					y + POINT_RADIUS_PX,
					fill=POINT_COLOR,
					outline="",
				)

	def _draw_targets(self) -> None:
		for (row, col), target_id in self.targets.items():
			x = self.geometry.canvas_x(col)
			y = self.geometry.canvas_y(row)
			self.canvas.create_oval(
				x - TARGET_RADIUS_PX,
				y - TARGET_RADIUS_PX,
				x + TARGET_RADIUS_PX,
				y + TARGET_RADIUS_PX,
				outline=TARGET_COLOR,
				width=3,
			)
			self.canvas.create_text(
				x,
				y - TARGET_RADIUS_PX - 10,
				text=str(target_id),
				fill=TARGET_COLOR,
				font=("TkDefaultFont", 10, "bold"),
			)

	def _update_status(self) -> None:
		self.status_var.set(
			f"{HELP_TEXT}  Inner walls: {self.inner_wall_count()} | "
			f"Targets: {len(self.targets)} | "
			f"Course: {self.geometry.width_in} in × {self.geometry.height_in} in | "
			f"Wall height: {self.wall_height_in} in"
		)

	def inner_wall_count(self) -> int:
		horizontal = sum(
			1 for row, _col in self.horizontal_walls if row not in {0, self.geometry.point_rows - 1}
		)
		vertical = sum(
			1 for _row, col in self.vertical_walls if col not in {0, self.geometry.point_cols - 1}
		)
		return horizontal + vertical

	def on_click(self, event: tk.Event[tk.Misc]) -> None:
		point = self.nearest_interior_point(event.x, event.y)
		if point is not None:
			if point in self.targets:
				del self.targets[point]
			else:
				target_id = self.prompt_for_target_id(self.next_target_id())
				if target_id is None:
					return
				self.targets[point] = target_id
			self.redraw()
			return

		edge = self.nearest_interior_edge(event.x, event.y)
		if edge is None:
			return

		orientation, row, col = edge
		if orientation == "h":
			wall_key = (row, col)
			if wall_key in self.horizontal_walls:
				self.horizontal_walls.remove(wall_key)
			else:
				self.horizontal_walls.add(wall_key)
		else:
			wall_key = (row, col)
			if wall_key in self.vertical_walls:
				self.vertical_walls.remove(wall_key)
			else:
				self.vertical_walls.add(wall_key)
		self.redraw()

	def prompt_for_target_id(self, default_id: int) -> int | None:
		while True:
			target_id = simpledialog.askinteger(
				"Target ID",
				"Enter the ArUco target id:",
				parent=self.root,
				initialvalue=default_id,
				minvalue=0,
			)
			if target_id is None:
				return None

			if target_id in self.targets.values():
				messagebox.showerror(
					"Duplicate target ID",
					f"Target id {target_id} is already in use. Choose a unique id.",
					parent=self.root,
				)
				continue

			return target_id

	def nearest_interior_point(self, x: int, y: int) -> tuple[int, int] | None:
		best: tuple[int, int] | None = None
		best_distance = float("inf")

		for row in range(1, self.geometry.point_rows - 1):
			py = self.geometry.canvas_y(row)
			for col in range(1, self.geometry.point_cols - 1):
				px = self.geometry.canvas_x(col)
				distance = math.dist((x, y), (px, py))
				if distance <= CLICK_POINT_TOLERANCE_PX and distance < best_distance:
					best_distance = distance
					best = (row, col)

		return best

	def nearest_interior_edge(self, x: int, y: int) -> tuple[str, int, int] | None:
		best: tuple[str, int, int] | None = None
		best_distance = float("inf")

		for row in range(1, self.geometry.point_rows - 1):
			y0 = self.geometry.canvas_y(row)
			for col in range(self.geometry.point_cols - 1):
				x1 = self.geometry.canvas_x(col)
				x2 = self.geometry.canvas_x(col + 1)
				if x1 <= x <= x2:
					distance = abs(y - y0)
					if distance <= CLICK_EDGE_TOLERANCE_PX and distance < best_distance:
						best_distance = distance
						best = ("h", row, col)

		for row in range(self.geometry.point_rows - 1):
			y1 = self.geometry.canvas_y(row)
			y2 = self.geometry.canvas_y(row + 1)
			for col in range(1, self.geometry.point_cols - 1):
				x0 = self.geometry.canvas_x(col)
				if y1 <= y <= y2:
					distance = abs(x - x0)
					if distance <= CLICK_EDGE_TOLERANCE_PX and distance < best_distance:
						best_distance = distance
						best = ("v", row, col)

		return best

	def export_layout(self) -> None:
		payload = {
			"course": {
				"inner_point_rows": len(self.geometry.row_heights_in) - 1,
				"inner_point_cols": len(self.geometry.col_widths_in) - 1,
				"row_heights_in": self.geometry.row_heights_in,
				"col_widths_in": self.geometry.col_widths_in,
				"wall_height_in": self.wall_height_in,
				"course_axes": self.course_axes,
				"target_marker": self.target_marker,
			},
			"targets": [
				{"row": row, "col": col, "id": target_id}
				for (row, col), target_id in sorted(self.targets.items())
			],
			"horizontal_walls": [
				{"row": row, "col": col}
				for row, col in sorted(self.horizontal_walls)
			],
			"vertical_walls": [
				{"row": row, "col": col}
				for row, col in sorted(self.vertical_walls)
			],
		}
		self.output_path.parent.mkdir(parents=True, exist_ok=True)
		self.output_path.write_text(json.dumps(payload, indent=2) + "\n")
		self.status_var.set(
			f"Layout saved to {self.output_path}. Inner walls: {self.inner_wall_count()} | "
			f"Targets: {len(self.targets)}"
		)

	def run(self) -> None:
		self.root.mainloop()


def main() -> None:
	parser = argparse.ArgumentParser(description="Interactive course layout editor")
	parser.add_argument(
		"course_json",
		nargs="?",
		type=Path,
		help="Optional course JSON file to load.",
	)
	parser.add_argument(
		"-o",
		"--output",
		type=Path,
		default=DEFAULT_OUTPUT_PATH,
		help="Output JSON path. Defaults to course.json.",
	)
	args = parser.parse_args()
	CourseLayoutApp(args.course_json, args.output).run()


if __name__ == "__main__":
	main()