import tkinter as tk
from tkinter import messagebox, filedialog

CANVAS_SIZE = 500         # Canvas size in pixels (500x500)
GRID_SIZE_METERS = 5.0    # Grid size in meters
PIXELS_PER_METER = CANVAS_SIZE / GRID_SIZE_METERS  # Conversion factor

class GridApp:
    def __init__(self, master):
        self.master = master
        master.title("Waypoint Mapper")
        # base_point will hold the absolute coordinates (in meters) of the first click
        self.base_point = None
        # points will store the relative waypoints, where the first point is (0, 0)
        self.points = []
        
        # Create and pack the canvas
        self.canvas = tk.Canvas(master, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white")
        self.canvas.pack()
        
        self.draw_grid()
        
        # Bind mouse click event to the canvas
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        
        # Create control buttons for exporting and clearing waypoints
        self.export_button = tk.Button(master, text="Export Waypoints", command=self.export_waypoints)
        self.export_button.pack(side=tk.LEFT, padx=10, pady=10)
        self.clear_button = tk.Button(master, text="Clear Waypoints", command=self.clear_waypoints)
        self.clear_button.pack(side=tk.LEFT, padx=10, pady=10)
        
    def draw_grid(self):
        # Draw grid lines every 0.2m for clarity
        spacing = PIXELS_PER_METER * 0.2
        num_lines = int(CANVAS_SIZE // spacing) + 1
        for i in range(num_lines):
            pos = i * spacing
            # Vertical and horizontal grid lines
            self.canvas.create_line(pos, 0, pos, CANVAS_SIZE, fill="lightgrey")
            self.canvas.create_line(0, pos, CANVAS_SIZE, pos, fill="lightgrey")
        # Draw the outer border
        self.canvas.create_rectangle(0, 0, CANVAS_SIZE, CANVAS_SIZE, outline="black")
        
    def on_canvas_click(self, event):
        # Convert pixel coordinates to meters, assuming the canvas origin is at the bottom-left
        x_m = event.x / PIXELS_PER_METER
        y_m = (CANVAS_SIZE - event.y) / PIXELS_PER_METER
        
        if self.base_point is None:
            # First click sets the origin; store it and record relative coordinate (0,0)
            self.base_point = (x_m, y_m)
            relative_point = (0, 0)
        else:
            # Subsequent clicks are stored relative to the base point
            relative_point = (x_m - self.base_point[0], y_m - self.base_point[1])
        
        self.points.append(relative_point)
        
        # Draw a marker at the actual (absolute) clicked position
        r = 3
        self.canvas.create_oval(event.x - r, event.y - r, event.x + r, event.y + r, fill="red")
        
        # Label the point with its order and the relative coordinate
        point_number = len(self.points)
        label_text = f"{point_number}: ({relative_point[0]:.2f}, {relative_point[1]:.2f})"
        self.canvas.create_text(event.x, event.y - 10, text=label_text, fill="blue", font=("Arial", 8))
        
    def export_waypoints(self):
        if not self.points:
            messagebox.showinfo("Info", "No waypoints to export.")
            return
        
        # Export the relative waypoints
        file_path = filedialog.asksaveasfilename(defaultextension=".txt",
                                                 filetypes=[("Text Files", "*.txt")])
        if file_path:
            with open(file_path, "w") as f:
                for point in self.points:
                    f.write(f"{point[0]:.2f}, {point[1]:.2f}\n")
            messagebox.showinfo("Success", f"Waypoints exported to {file_path}")
        
    def clear_waypoints(self):
        # Reset the base point and points list, then redraw the grid
        self.base_point = None
        self.points.clear()
        self.canvas.delete("all")
        self.draw_grid()

if __name__ == "__main__":
    root = tk.Tk()
    app = GridApp(root)
    root.mainloop()