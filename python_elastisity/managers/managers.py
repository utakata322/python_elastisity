import math
import matplotlib.pyplot as plt
import numpy as np

from models.MaterialPoints import MaterialPoint
from models.MaterialBody import MaterialBody
from models.PointTrajectory import PointTrajectory
from models.BodyTrajectory import BodyTrajectory
from models.SpacePoint import SpacePoint
from models.SpaceGrid import SpaceGrid


def f_x(t, x):
    return -math.log(3 * t + 1) * x


def f_y(t, y):
    return t * y


def int_runge_x(x, t, h):
    f_1 = f_x(t, x)
    f_2 = f_x(t + h * 1/2, x + h * f_1 * 1/2)
    f_3 = f_x(t + h * 1, x - h * f_1 * 1 + h * f_2 * 2)
    return x + h/6 * (f_1 + 4 * f_2 + f_3)


def int_runge_y(y, t, h):
    f_1 = f_y(t, y)
    f_2 = f_y(t + h * 1/2, y + h * f_1 * 1/2)
    f_3 = f_y(t + h * 1, y - h * f_1 * 1 + h * f_2 * 2)
    return y + h/6 * (f_1 + 4 * f_2 + f_3)


def init_material_body(x_c, y_c, radius, n):
    t = 0
    points = []
    for i in range(n):
        x = x_c + math.cos(i * radius * 2 * math.pi / n)
        y = y_c + math.sin(i * radius * 2 * math.pi / n)
        points.append(MaterialPoint(i, x, y, f_x(t, x), f_y(t, y), x, y, t))
    return MaterialBody(points)


def move_material_body(time, step, material_body):
    point_trajectories = []

    points_count = len(material_body.material_points)
    for i in range(points_count):
        current_time = 0
        x_0 = material_body.material_points[i].x_0
        y_0 = material_body.material_points[i].y_0
        x_t = [x_0]
        y_t = [y_0]
        steps_count = int(time / step)

        for n in range(steps_count + 1):
            prev_x = x_t[n]
            prev_y = y_t[n]
            x_t.append(int_runge_x(prev_x, current_time, step))
            y_t.append(int_runge_y(prev_y, current_time, step))
            current_time += step
        point_trajectories.append(PointTrajectory(material_body.material_points[i], x_t, y_t))
    return BodyTrajectory(point_trajectories, material_body)


def plot_trajectory(material_body, body_trajectory):
    for i in range(len(material_body.material_points)):
        plt.plot(material_body.material_points[i].coord_x, material_body.material_points[i].coord_y,'r.')
        plt.plot(body_trajectory.point_trajectories[i].x, body_trajectory.point_trajectories[i].y, 'black',
                 linewidth=0.5)
        time = len(body_trajectory.point_trajectories[i].x) - 1
        plt.plot(body_trajectory.point_trajectories[i].x[time], body_trajectory.point_trajectories[i].y[time],'g.')
    plt.axis('equal')
    plt.grid()
    plt.show()
    # plt.savefig('траектория.png', format='png', dpi=1200)


def move_through_space(time, step, grid_axis):
    current_time = step
    point_id = 0

    grid_length = grid_axis * 2 + 1
    a = np.linspace(-grid_axis, grid_axis,  grid_length)
    x_s, y_s = np.meshgrid(a, a)
    velocity_fields = []

    steps_count = int(time / step)
    for n in range(steps_count + 1):
        space_points = []
        for i in range( grid_length):
            for j in range( grid_length):
                x = x_s[i, j]
                y = y_s[i, j]
                space_points.append(SpacePoint(point_id, x, y, f_x(current_time, x), f_y(current_time, y), current_time))
                point_id += 1
        velocity_fields.append(SpaceGrid(space_points))
        current_time += step
    return velocity_fields

def plot_velocity_fields(velocity_fields, grid_axis):
    step = velocity_fields[0].space_points[0].t
    current_time = step

    grid_length = grid_axis * 2 + 1

    fields_counter = len(velocity_fields)
    for n in range(fields_counter):
        plt.figure(n)
        plt.suptitle('current time: ' + str(current_time))
        point_id = 0
        coord_x = []
        coord_y = []
        v_x = []
        v_y = []
        for i in range(grid_length):
            for j in range(grid_length):
                coord_x.append(velocity_fields[n].space_points[point_id].coord_x)
                coord_y.append(velocity_fields[n].space_points[point_id].coord_y)
                v_x.append(velocity_fields[n].space_points[point_id].velocity_x)
                v_y.append(velocity_fields[n].space_points[point_id].velocity_y)
                point_id += 1
        plt.subplot(1, 2, 1)
        plt.quiver(coord_x, coord_y, v_x, v_y)

        for i in range(0, 20):
                x1 = np.linspace(-grid_axis, -0.1, 100)
                x2 = np.linspace(0.1, grid_axis, 100)
                power = f_y(current_time, 1) / f_x(current_time, 1)
                diff_const = -i
                y1 = diff_const * pow(-x1, power)
                y2 = diff_const * pow(x2, power)

                plt.subplot(1, 2, 2)
                plt.axis([-grid_axis, grid_axis, -grid_axis, grid_axis])
                plt.plot(x1, y1)
                plt.plot(x2, y2)
        current_time += step
        plt.show()
        # plt.savefig('поле_скоростей' + str(n) + '.png', format='png', dpi=1200)
