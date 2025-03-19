file_path = "/home/pi/Downloads/waypoints.txt"

x_vals, y_vals = [], []

with open(file_path, "r") as file:
    for line in file:
        x, y = map(int, line.split(",")) # Split by space
        x_vals.append(x)
        y_vals.append(y)
        
print(x_vals, y_vals) 