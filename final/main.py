import json
import tkinter as tk
from tkinter import ttk
from typing import Dict, Optional, Tuple
from geometry import Point
from viewport import Viewport
from editor import PolygonEditor
from utils import format_stats

class ArtGalleryApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Art Gallery Problem - Guards via Triangulation and 3-Coloring")
        self.root.minsize(900, 600)

        # --- Sidebar with scrollbar ---
        SIDEBAR_WIDTH = 270
        
        sidebar_frame = ttk.Frame(root, width=SIDEBAR_WIDTH)
        sidebar_frame.pack(side=tk.LEFT, fill=tk.Y)
        sidebar_frame.pack_propagate(False)

        sidebar_canvas = tk.Canvas(sidebar_frame, borderwidth=0, background="#f8f8f8", width=SIDEBAR_WIDTH - 20)
        sidebar_canvas.pack(side=tk.LEFT, fill=tk.Y, expand=True)

        scrollbar = ttk.Scrollbar(sidebar_frame, orient="vertical", command=sidebar_canvas.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        sidebar_canvas.configure(yscrollcommand=scrollbar.set)

        self.sidebar = ttk.Frame(sidebar_canvas, width=SIDEBAR_WIDTH - 20)
        self.sidebar.bind(
            "<Configure>",
            lambda e: sidebar_canvas.configure(scrollregion=sidebar_canvas.bbox("all"))
        )
        
        sidebar_canvas.create_window((0, 0), window=self.sidebar, anchor="nw", width=SIDEBAR_WIDTH - 20)

        def _on_mousewheel(event):
            sidebar_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        
        def _bind_mousewheel(event):
            sidebar_canvas.bind_all("<MouseWheel>", _on_mousewheel)
        
        def _unbind_mousewheel(event):
            sidebar_canvas.unbind_all("<MouseWheel>")

        sidebar_canvas.bind("<Enter>", _bind_mousewheel)
        sidebar_canvas.bind("<Leave>", _unbind_mousewheel)

        # --- Main canvas ---
        self.canvas = tk.Canvas(root, bg="#ffffff", highlightthickness=0)
        self.canvas.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.editor = PolygonEditor(self.canvas)

        # Initialize the UI
        self._init_controls()
        self._init_cost_inputs()
        self._init_stats_panel()
        self._init_class_stats()
        
        # Set up interactions
        self.mode = "add"
        self._setup_canvas_bindings()

    def _init_controls(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TButton", padding=6)
        style.configure("TLabel", padding=2)

        # Create and arrange control buttons
        self.btn_add_mode = ttk.Button(self.sidebar, text="Add Mode (Click)", command=self._set_add_mode)
        self.btn_move_mode = ttk.Button(self.sidebar, text="Move Mode (Drag)", command=self._set_move_mode)
        self.btn_delete_mode = ttk.Button(self.sidebar, text="Delete Mode (Click)", command=self._set_delete_mode)
        self.btn_socket_mode = ttk.Button(self.sidebar, text="Socket Mode (Click Edge)", command=self._set_socket_mode)
        self.btn_show_optimal_only = ttk.Button(self.sidebar, text="Show Optimal Only", command=self._toggle_opt)
        self.btn_close = ttk.Button(self.sidebar, text="Close Polygon", command=self._close_poly)
        self.btn_compute = ttk.Button(self.sidebar, text="Compute Guards", command=self._compute)
        self.btn_reset = ttk.Button(self.sidebar, text="Reset", command=self._reset)
        self.btn_save = ttk.Button(self.sidebar, text="Save JSON", command=self._save)
        self.btn_load = ttk.Button(self.sidebar, text="Load JSON", command=self._load)
        self.btn_export = ttk.Button(self.sidebar, text="Export PNG", command=self._export_png)

        for btn in [
            self.btn_add_mode, self.btn_move_mode, self.btn_delete_mode, 
            self.btn_socket_mode, self.btn_show_optimal_only, self.btn_close,
            self.btn_compute, self.btn_reset, self.btn_save, self.btn_load,
            self.btn_export
        ]:
            btn.pack(fill=tk.X, padx=10, pady=6)

    def _init_cost_inputs(self):
        cost_frame = ttk.LabelFrame(self.sidebar, text="Costs")
        cost_frame.pack(fill=tk.X, padx=10, pady=6)
        
        ttk.Label(cost_frame, text="Wire per unit:").grid(row=0, column=0, sticky="w")
        ttk.Label(cost_frame, text="Camera cost:").grid(row=1, column=0, sticky="w")
        
        self.var_wire = tk.StringVar(value="1.0")
        self.var_cam = tk.StringVar(value="1.0")
        
        ttk.Entry(cost_frame, textvariable=self.var_wire, width=10).grid(row=0, column=1, sticky="e")
        ttk.Entry(cost_frame, textvariable=self.var_cam, width=10).grid(row=1, column=1, sticky="e")

    def _init_stats_panel(self):
        stats_frame = ttk.LabelFrame(self.sidebar, text="Statistics")
        stats_frame.pack(fill=tk.X, padx=10, pady=6)
        
        self.stats_vars = {
            "n": tk.StringVar(value="n: -"),
            "tri": tk.StringVar(value="triangles: -"),
            "guards": tk.StringVar(value="guards: -"),
            "class": tk.StringVar(value="class: -"),
            "wire_len": tk.StringVar(value="wire length: -"),
            "wire_cost": tk.StringVar(value="wire cost: -"),
            "cam_cost": tk.StringVar(value="camera cost: -"),
            "total_cost": tk.StringVar(value="total cost: -"),
        }
        
        for i, key in enumerate(["n", "tri", "guards", "class", "wire_len", "wire_cost", "cam_cost", "total_cost"]):
            ttk.Label(stats_frame, textvariable=self.stats_vars[key]).grid(row=i, column=0, sticky="w")

    def _init_class_stats(self):
        self.class_frame = ttk.LabelFrame(self.sidebar, text="Class-wise costs (0/1/2)")
        self.class_frame.pack(fill=tk.X, padx=10, pady=6)
        
        self.class_vars = {
            "c0": tk.StringVar(value="0: size -, total -, wire -, cam -"),
            "c1": tk.StringVar(value="1: size -, total -, wire -, cam -"),
            "c2": tk.StringVar(value="2: size -, total -, wire -, cam -"),
        }
        
        for var in self.class_vars.values():
            ttk.Label(self.class_frame, textvariable=var).pack(anchor="w")

    def _setup_canvas_bindings(self):
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Configure>", lambda e: self.editor._refresh_view())
        
        # Initialize status variable
        self.status_var = tk.StringVar(value="Add vertices → Close → Socket → Costs → Compute")

    def _set_add_mode(self):
        self.mode = "add"
        self.status_var.set("Add Mode: click canvas to append vertices.")

    def _set_move_mode(self):
        self.mode = "move"
        self.status_var.set("Move Mode: click and drag a vertex to move it.")

    def _set_delete_mode(self):
        self.mode = "delete"
        self.status_var.set("Delete Mode: click a vertex to remove it.")

    def _set_socket_mode(self):
        self.mode = "socket"
        self.status_var.set("Socket Mode: click near an edge to place the socket.")

    def _toggle_opt(self):
        self.editor.show_opt_only = not self.editor.show_opt_only
        txt = "Show All" if self.editor.show_opt_only else "Show Optimal Only"
        self.btn_show_optimal_only.config(text=txt)
        self.editor._refresh_view()

    def _close_poly(self):
        self.editor.close_polygon()
        self.status_var.set("Polygon closed.")

    def _compute(self):
        try:
            self.editor.cost_wire_per_unit = float(self.var_wire.get())
            self.editor.cost_camera = float(self.var_cam.get())
        except ValueError:
            self.status_var.set("Invalid cost inputs.")
            return
        ok, msg = self.editor.compute()
        self.status_var.set(msg)
        self._refresh_stats()

    def _reset(self):
        self.editor.reset()
        self.status_var.set("Reset. Click to add vertices.")
        self._refresh_stats()

    def _save(self):
        try:
            self.editor.save_json("polygon.json")
            self.status_var.set("Saved to polygon.json")
        except Exception as e:
            self.status_var.set(f"Save failed: {e}")

    def _load(self):
        try:
            self.editor.load_json("polygon.json")
            self.status_var.set("Loaded polygon.json")
            self._refresh_stats()
        except Exception as e:
            self.status_var.set(f"Load failed: {e}")

    def _export_png(self):
        try:
            ps_path = "canvas.ps"
            png_path = "output.png"
            self.canvas.postscript(file=ps_path, colormode='color')
            try:
                from PIL import Image
                img = Image.open(ps_path)
                img.save(png_path, "png")
                self.status_var.set(f"Exported {png_path}")
            except ImportError:
                self.status_var.set(f"Saved PostScript {ps_path} (install Pillow for PNG)")
        except Exception as e:
            self.status_var.set(f"Export failed: {e}")

    def _screen_to_world(self, x: int, y: int) -> Point:
        w = int(self.canvas.winfo_width())
        h = int(self.canvas.winfo_height())
        return self.editor.viewport.screen_to_world(x, y, w, h)

    def _hit_vertex(self, x: int, y: int, radius_px: int = 10) -> Optional[int]:
        w = int(self.canvas.winfo_width())
        h = int(self.canvas.winfo_height())
        best = None
        best_d2 = (radius_px + 1) ** 2
        for i, p in enumerate(self.editor.points):
            sx, sy = self.editor.viewport.world_to_screen(p, w, h)
            dx = sx - x
            dy = sy - y
            d2 = dx * dx + dy * dy
            if d2 <= best_d2:
                best_d2 = d2
                best = i
        return best

    def _refresh_stats(self):
        """Update all statistics display variables."""
        st = getattr(self.editor, "stats", {}) or {}
        
        def fmt(k, fmt_spec="{:.3f}"):
            v = st.get(k)
            if v is None:
                return "-"
            try:
                if k in ("n", "tri", "guards", "class"):
                    return str(int(v)) if v >= 0 else "-"
                return fmt_spec.format(float(v))
            except Exception:
                return str(v)
                
        if hasattr(self, "stats_vars"):
            self.stats_vars["n"].set(f"n: {fmt('n','{:.0f}')} ")
            self.stats_vars["tri"].set(f"triangles: {fmt('tri','{:.0f}')} ")
            cls = fmt('class','{:.0f}')
            self.stats_vars["class"].set(f"class: {cls}")
            self.stats_vars["guards"].set(f"guards: {fmt('guards','{:.0f}')} ")
            self.stats_vars["wire_len"].set(f"wire length: {fmt('wire_len')} ")
            self.stats_vars["wire_cost"].set(f"wire cost: {fmt('wire_cost')} ")
            self.stats_vars["cam_cost"].set(f"camera cost: {fmt('cam_cost')} ")
            self.stats_vars["total_cost"].set(f"total cost: {fmt('total_cost')} ")

        # Update class-wise lines
        cs = getattr(self.editor, "class_stats", {}) or {}
        def cf(cid: int, key: str, spec="{:.3f}"):
            row = cs.get(cid)
            if not row:
                return "-"
            if key == "size":
                return str(int(row.get(key, 0)))
            try:
                return spec.format(float(row.get(key, 0.0)))
            except Exception:
                return str(row.get(key, "-"))
                
        if hasattr(self, "class_vars"):
            self.class_vars["c0"].set(f"0: size {cf(0,'size','{:.0f}')}, total {cf(0,'total_cost')}, wire {cf(0,'wire_cost')}, cam {cf(0,'cam_cost')}")
            self.class_vars["c1"].set(f"1: size {cf(1,'size','{:.0f}')}, total {cf(1,'total_cost')}, wire {cf(1,'wire_cost')}, cam {cf(1,'cam_cost')}")
            self.class_vars["c2"].set(f"2: size {cf(2,'size','{:.0f}')}, total {cf(2,'total_cost')}, wire {cf(2,'wire_cost')}, cam {cf(2,'cam_cost')}")

    def _on_click(self, event):
        if self.mode == "add" and not self.editor.closed:
            vx, vy = self._screen_to_world(event.x, event.y)
            self.editor.add_point((vx, vy))
            return
        if self.mode == "move":
            idx = self._hit_vertex(event.x, event.y)
            self.editor.selected_index = idx
            self.editor.drag_index = idx
            self.editor._refresh_view()
            return
        if self.mode == "delete":
            idx = self._hit_vertex(event.x, event.y)
            if idx is not None and not self.editor.closed:
                del self.editor.points[idx]
                # Clear computed data
                self.editor.triangles.clear()
                self.editor.diagonals.clear()
                self.editor.colors.clear()
                self.editor.guards.clear()
                self.editor.selected_index = None
                self.editor._refresh_view()
                return
        if self.mode == "socket" and self.editor.closed:
            # Pick nearest edge and parameter t
            n = len(self.editor.points)
            w = int(self.canvas.winfo_width())
            h = int(self.canvas.winfo_height())
            best = None
            best_d2 = 1e18
            best_t = 0.0
            for k in range(n):
                a = self.editor.points[k]
                b = self.editor.points[(k + 1) % n]
                # Project click to segment in screen space for more intuitive picking
                ax, ay = self.editor.viewport.world_to_screen(a, w, h)
                bx, by = self.editor.viewport.world_to_screen(b, w, h)
                vx = bx - ax
                vy = by - ay
                seg_len2 = vx * vx + vy * vy
                t = 0.0 if seg_len2 == 0 else max(0.0, min(1.0, ((event.x - ax) * vx + (event.y - ay) * vy) / seg_len2))
                px = ax + t * vx
                py = ay + t * vy
                dx = px - event.x
                dy = py - event.y
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2 = d2
                    best = k
                    best_t = t
            if best is not None:
                self.editor.socket_edge = best
                self.editor.socket_t = best_t
                # Auto-recompute with current costs
                try:
                    self.editor.cost_wire_per_unit = float(self.var_wire.get())
                    self.editor.cost_camera = float(self.var_cam.get())
                except ValueError:
                    pass
                ok, msg = self.editor.compute()
                self.status_var.set(msg if ok else "")
                self._refresh_stats()

    def _on_drag(self, event):
        if self.mode == "move" and self.editor.drag_index is not None and not self.editor.closed:
            vx, vy = self._screen_to_world(event.x, event.y)
            self.editor.points[self.editor.drag_index] = (vx, vy)
            # Invalidate computed artifacts while moving
            self.editor.triangles.clear()
            self.editor.diagonals.clear()
            self.editor.colors.clear()
            self.editor.guards.clear()
            self.editor._refresh_view()
            self._refresh_stats()

    def _on_release(self, event):
        if self.mode == "move":
            self.editor.drag_index = None

    # ... Rest of the methods (event handlers, etc.) stay the same ...

def main():
    root = tk.Tk()
    app = ArtGalleryApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
