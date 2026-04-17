#!/usr/bin/env python3
"""
Waypoint Grid Editor
  - 10x10 grid, each cell 0.6 m x 0.6 m, origin at center
  - Phase 1: click cells to mark them as occupied (obstacles)
  - Phase 2: click anywhere (including mid-cell) to place waypoints
  - Press "Done / Save" to write waypoints.yaml
"""

import tkinter as tk
from tkinter import messagebox, filedialog
import os
import sys

# ---------------------------------------------------------------------------
# Grid / display constants
# ---------------------------------------------------------------------------
GRID_N    = 10          # 10×10 cells
CELL_M    = 0.6         # metres per cell
HALF      = GRID_N * CELL_M / 2   # 3.0 m  → coords run -3 … +3

CELL_PX   = 70          # pixels per cell
MARGIN    = 55          # pixel margin around grid (room for axis labels)
GRID_PX   = GRID_N * CELL_PX                         # 700 px
CANVAS_W  = GRID_PX + 2 * MARGIN                      # 810 px
CANVAS_H  = GRID_PX + 2 * MARGIN + 20                 # + status bar

# Colours
C_EMPTY    = "#f5f5f5"
C_OCCUPIED = "#d9534f"
C_WAYPT    = "#1a7abf"
C_PATH     = "#5bc0de"
C_AXIS0    = "#cc0000"   # centre axes
C_GRID     = "#bbbbbb"
C_HOVER    = "#ffe0b2"

OUTPUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "waypoints.yaml")


# ---------------------------------------------------------------------------
# Coordinate helpers
# ---------------------------------------------------------------------------
def w2c(wx, wy):
    """World (m) → canvas (px).  +Y is up in world, down in canvas."""
    cx = MARGIN + (wx + HALF) / CELL_M * CELL_PX
    cy = MARGIN + (HALF - wy) / CELL_M * CELL_PX
    return cx, cy


def c2w(cx, cy):
    """Canvas (px) → world (m)."""
    wx = (cx - MARGIN) / CELL_PX * CELL_M - HALF
    wy = HALF - (cy - MARGIN) / CELL_PX * CELL_M
    return wx, wy


def px_in_grid(cx, cy):
    return MARGIN <= cx <= MARGIN + GRID_PX and MARGIN <= cy <= MARGIN + GRID_PX


def px_to_cell(cx, cy):
    col = int((cx - MARGIN) / CELL_PX)
    row = int((cy - MARGIN) / CELL_PX)
    col = max(0, min(GRID_N - 1, col))
    row = max(0, min(GRID_N - 1, row))
    return col, row


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------
class WaypointEditor:
    def __init__(self, root):
        self.root = root
        self.root.title("Waypoint Grid Editor  —  " + OUTPUT_FILE)
        self.root.resizable(False, False)

        self.phase     = "obstacles"   # or "waypoints"
        self.occupied  = set()         # {(col, row), …}
        self.waypoints = []            # [(wx, wy), …]
        self._hover_cell = None

        self._build_ui()
        self._redraw()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def _build_ui(self):
        # ── top bar ───────────────────────────────────────────────────
        top = tk.Frame(self.root, bg="#2c3e50", height=44)
        top.pack(fill=tk.X)
        top.pack_propagate(False)

        self.phase_lbl = tk.Label(
            top,
            text="Phase 1  ·  Click cells to mark obstacles",
            fg="white", bg="#2c3e50",
            font=("Arial", 12, "bold")
        )
        self.phase_lbl.pack(side=tk.LEFT, padx=14, pady=10)

        # ── main area (canvas + sidebar) ──────────────────────────────
        main = tk.Frame(self.root, bg="#ecf0f1")
        main.pack(fill=tk.BOTH, expand=True)

        # Canvas
        self.canvas = tk.Canvas(
            main,
            width=CANVAS_W, height=CANVAS_H,
            bg="white", cursor="crosshair",
            highlightthickness=1, highlightbackground="#999"
        )
        self.canvas.pack(side=tk.LEFT, padx=(8, 4), pady=8)
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<Motion>",   self._on_hover)
        self.canvas.bind("<Leave>",    self._on_leave)

        # Sidebar
        side = tk.Frame(main, bg="#ecf0f1", width=210)
        side.pack(side=tk.LEFT, fill=tk.Y, padx=(4, 8), pady=8)
        side.pack_propagate(False)

        def btn(parent, text, cmd, bg, fg="white"):
            return tk.Button(
                parent, text=text, command=cmd,
                bg=bg, fg=fg, activebackground=bg,
                font=("Arial", 10, "bold"),
                relief=tk.FLAT, bd=0,
                padx=8, pady=6, cursor="hand2"
            )

        self.phase_btn = btn(side,
            "→  Start Placing Waypoints",
            self._toggle_phase, "#27ae60")
        self.phase_btn.pack(fill=tk.X, pady=(0, 4))

        btn(side, "Undo Last Waypoint",  self._undo,            "#e67e22").pack(fill=tk.X, pady=2)
        btn(side, "Clear All Waypoints", self._clear_waypoints, "#c0392b").pack(fill=tk.X, pady=2)
        btn(side, "Clear All Obstacles", self._clear_obstacles, "#8e44ad").pack(fill=tk.X, pady=2)

        sep = tk.Frame(side, bg="#bdc3c7", height=1)
        sep.pack(fill=tk.X, pady=10)

        tk.Label(side, text="Waypoint list", font=("Arial", 9, "bold"),
                 bg="#ecf0f1", fg="#555").pack(anchor="w")

        listframe = tk.Frame(side, bg="#ecf0f1")
        listframe.pack(fill=tk.BOTH, expand=True)

        sb = tk.Scrollbar(listframe)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox = tk.Listbox(
            listframe, font=("Courier", 9),
            yscrollcommand=sb.set, activestyle="none",
            selectbackground="#1a7abf", selectforeground="white"
        )
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.config(command=self.listbox.yview)

        sep2 = tk.Frame(side, bg="#bdc3c7", height=1)
        sep2.pack(fill=tk.X, pady=10)

        self.done_btn = btn(side, "✔  Done  /  Save Waypoints",
                            self._save, "#2980b9")
        self.done_btn.pack(fill=tk.X)

        # ── status bar ────────────────────────────────────────────────
        self.status_var = tk.StringVar(value="Move mouse over the grid to see coordinates")
        status = tk.Label(
            self.root, textvariable=self.status_var,
            bg="#2c3e50", fg="#ecf0f1",
            font=("Courier", 9), anchor="w", padx=8
        )
        status.pack(fill=tk.X)

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------
    def _redraw(self):
        self.canvas.delete("all")
        self._draw_cells()
        self._draw_grid_lines()
        self._draw_axes()
        self._draw_tick_labels()
        self._draw_path()
        self._draw_waypoints()

    def _draw_cells(self):
        for col in range(GRID_N):
            for row in range(GRID_N):
                x0 = MARGIN + col * CELL_PX
                y0 = MARGIN + row * CELL_PX
                x1 = x0 + CELL_PX
                y1 = y0 + CELL_PX
                if (col, row) in self.occupied:
                    fill = C_OCCUPIED
                elif (col, row) == self._hover_cell and self.phase == "obstacles":
                    fill = C_HOVER
                else:
                    fill = C_EMPTY
                self.canvas.create_rectangle(x0, y0, x1, y1,
                                             fill=fill, outline="", tags="cell")

    def _draw_grid_lines(self):
        for i in range(GRID_N + 1):
            x = MARGIN + i * CELL_PX
            y = MARGIN + i * CELL_PX
            # vertical
            self.canvas.create_line(x, MARGIN, x, MARGIN + GRID_PX,
                                    fill=C_GRID, width=1)
            # horizontal
            self.canvas.create_line(MARGIN, y, MARGIN + GRID_PX, y,
                                    fill=C_GRID, width=1)

    def _draw_axes(self):
        """Draw the X=0 and Y=0 centre lines in red."""
        ox, oy = w2c(0, 0)
        self.canvas.create_line(MARGIN, oy, MARGIN + GRID_PX, oy,
                                fill=C_AXIS0, width=2, dash=(6, 3))
        self.canvas.create_line(ox, MARGIN, ox, MARGIN + GRID_PX,
                                fill=C_AXIS0, width=2, dash=(6, 3))
        self.canvas.create_text(ox + 18, oy - 10, text="(0,0)",
                                fill=C_AXIS0, font=("Arial", 8, "bold"))

    def _draw_tick_labels(self):
        for i in range(GRID_N + 1):
            wx = -HALF + i * CELL_M
            wy =  HALF - i * CELL_M
            cx = MARGIN + i * CELL_PX
            cy = MARGIN + i * CELL_PX
            # X axis ticks (bottom)
            self.canvas.create_text(cx, MARGIN + GRID_PX + 10,
                                    text=f"{wx:.1f}", font=("Arial", 7), fill="#555")
            # Y axis ticks (left)
            self.canvas.create_text(MARGIN - 22, cy,
                                    text=f"{wy:.1f}", font=("Arial", 7), fill="#555")
        # Axis name labels
        self.canvas.create_text(MARGIN + GRID_PX / 2, MARGIN - 18,
                                text="X  (m)", font=("Arial", 9, "bold"), fill="#333")
        self.canvas.create_text(MARGIN - 42, MARGIN + GRID_PX / 2,
                                text="Y\n(m)", font=("Arial", 9, "bold"), fill="#333")

    def _draw_path(self):
        if len(self.waypoints) < 2:
            return
        for i in range(len(self.waypoints) - 1):
            x0, y0 = w2c(*self.waypoints[i])
            x1, y1 = w2c(*self.waypoints[i + 1])
            self.canvas.create_line(x0, y0, x1, y1,
                                    fill=C_PATH, width=2,
                                    arrow=tk.LAST, arrowshape=(10, 12, 4),
                                    tags="path")

    def _draw_waypoints(self):
        for idx, (wx, wy) in enumerate(self.waypoints):
            cx, cy = w2c(wx, wy)
            r = 7
            self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                                    fill=C_WAYPT, outline="white", width=2,
                                    tags="waypt")
            self.canvas.create_text(cx, cy - r - 7, text=str(idx + 1),
                                    font=("Arial", 8, "bold"),
                                    fill=C_WAYPT, tags="waypt")
        # Refresh list box
        self.listbox.delete(0, tk.END)
        for idx, (wx, wy) in enumerate(self.waypoints):
            self.listbox.insert(tk.END, f"  {idx+1:>2}.  x={wx:+.3f}  y={wy:+.3f}")

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------
    def _on_click(self, event):
        cx, cy = event.x, event.y
        if not px_in_grid(cx, cy):
            return

        if self.phase == "obstacles":
            col, row = px_to_cell(cx, cy)
            if (col, row) in self.occupied:
                self.occupied.discard((col, row))
            else:
                self.occupied.add((col, row))
            self._redraw()

        else:  # waypoints
            col, row = px_to_cell(cx, cy)
            if (col, row) in self.occupied:
                self.status_var.set("⚠  That cell is marked as occupied — pick another spot.")
                return
            wx, wy = c2w(cx, cy)
            self.waypoints.append((round(wx, 4), round(wy, 4)))
            self._redraw()
            self.status_var.set(
                f"Added waypoint {len(self.waypoints)}:  x={wx:+.3f} m   y={wy:+.3f} m"
            )

    def _on_hover(self, event):
        cx, cy = event.x, event.y
        if px_in_grid(cx, cy):
            wx, wy = c2w(cx, cy)
            col, row = px_to_cell(cx, cy)
            occ = "  [OCCUPIED]" if (col, row) in self.occupied else ""
            self.status_var.set(
                f"x = {wx:+.3f} m     y = {wy:+.3f} m     cell ({col}, {row}){occ}"
            )
            if self.phase == "obstacles":
                prev = self._hover_cell
                self._hover_cell = (col, row)
                if prev != self._hover_cell:
                    self._redraw()
        else:
            self.status_var.set("Move mouse over the grid")
            self._hover_cell = None

    def _on_leave(self, event):
        self._hover_cell = None
        self.status_var.set("Move mouse over the grid")
        self._redraw()

    # ------------------------------------------------------------------
    # Button callbacks
    # ------------------------------------------------------------------
    def _toggle_phase(self):
        if self.phase == "obstacles":
            self.phase = "waypoints"
            self.phase_lbl.config(
                text="Phase 2  ·  Click anywhere on the grid to place waypoints  "
                     "(right-click = undo)")
            self.phase_btn.config(text="←  Back to Obstacle Marking", bg="#e67e22")
            self.canvas.config(cursor="crosshair")
            self.canvas.bind("<Button-3>", lambda e: self._undo())
        else:
            self.phase = "obstacles"
            self.phase_lbl.config(text="Phase 1  ·  Click cells to mark obstacles")
            self.phase_btn.config(text="→  Start Placing Waypoints", bg="#27ae60")
            self.canvas.unbind("<Button-3>")
        self._hover_cell = None
        self._redraw()

    def _undo(self):
        if self.waypoints:
            self.waypoints.pop()
            self._redraw()

    def _clear_waypoints(self):
        if self.waypoints and messagebox.askyesno("Confirm", "Clear all waypoints?"):
            self.waypoints.clear()
            self._redraw()

    def _clear_obstacles(self):
        if self.occupied and messagebox.askyesno("Confirm", "Clear all obstacle markings?"):
            self.occupied.clear()
            self._redraw()

    def _save(self):
        if not self.waypoints:
            messagebox.showerror("Nothing to save", "No waypoints have been placed yet.")
            return

        path = filedialog.asksaveasfilename(
            title="Save waypoints as…",
            initialdir=os.path.dirname(OUTPUT_FILE),
            initialfile="waypoints.yaml",
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")],
        )
        if not path:
            return  # user cancelled

        lines = [
            "# Waypoints for the PID waypoint follower.",
            "# Each entry is [x, y, z] in the Vicon global frame (meters).",
            "waypoints:",
        ]
        for wx, wy in self.waypoints:
            lines.append(f"  - [{wx:.4f}, {wy:.4f}, 0.0]")
        lines.append("")  # trailing newline

        with open(path, "w") as f:
            f.write("\n".join(lines))

        self.root.title("Waypoint Grid Editor  —  " + path)
        messagebox.showinfo(
            "Saved",
            f"Saved {len(self.waypoints)} waypoint(s) to:\n{path}"
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    root = tk.Tk()
    app = WaypointEditor(root)
    root.mainloop()


if __name__ == "__main__":
    main()
