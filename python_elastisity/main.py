from managers.managers import *

x_corner = 2
y_corner = -2
radius = 1
time = 1
step = 0.1
n = 100
grid_axis = 5

body = init_material_body(x_corner, y_corner, radius, n)
trajectory = move_material_body(time, step, body)

plot_trajectory(body, trajectory)

velocity_fields = move_through_space(time, step, grid_axis)
plot_velocity_fields(velocity_fields, grid_axis)
