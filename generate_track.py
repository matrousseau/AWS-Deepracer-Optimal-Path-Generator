import matplotlib.pyplot as plt
import seaborn as sns;

sns.set()
import numpy as np
import pandas as pd
import inquirer
import math
from CubicSpline.a_star import AStarPlanner
from CubicSpline.rear_wheel_feedback import main

from all_waypoints import reinvent_waypoints, kumo_torraku, AWS_track, Shanghai, Toronto, reinvent_waypoints2019, cumulo, oval, bowtie

import shapely.geometry as shp

from scipy.interpolate import interp1d
import time

show_animation = True

def getdistance(x1, x2, y1, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def getAngle(a, b, c):
    ang = math.degrees(math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(a[1] - b[1], a[0] - b[0]))
    return ang + 360 if ang < 0 else ang


class generate_Track():

    def __init__(self):

        self.all_track = [reinvent_waypoints, kumo_torraku, AWS_track, Shanghai, Toronto, reinvent_waypoints2019, cumulo, oval, bowtie]
        self.all_responses = ['reinvent_waypoints', 'kumo_torraku', 'AWS_track', 'Shanghai', 'Toronto','reinvent_waypoints2019', 'cumulo','oval','bowtie']

        questions = [
            inquirer.List('track',
                          message="Which track do you want ?",
                          choices=self.all_responses,
                          ), ]

        self.tracks = inquirer.prompt(questions)
        self.waypoints = self.all_track[self.all_responses.index(self.tracks["track"])]
        self.SCALE_COEFF = int(input(" Enlargement coefficient : "))
        self.INTERPOL_COEFF = int(input(" Interpolation coefficient : "))
        self.df_waypoints = pd.DataFrame(self.waypoints, columns=['X', 'Y'])
        self.x_origin = self.df_waypoints.X
        self.y_origin = self.df_waypoints.Y
        self.filename = str(
            'data/_' + str(self.tracks["track"]) + "_" + str(self.INTERPOL_COEFF) + '__' + str(
                self.SCALE_COEFF) + '__' + time.strftime("%Y%m%d-%H%M%S"))

    def interpolate_waypoints(self, waypoints):
        points = np.array(waypoints)
        # Linear length along the line:
        distance = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1)))
        distance = np.insert(distance, 0, 0) / distance[-1]

        # interpolations_methods = ['cubic','next','previous' ...]
        alpha = np.linspace(0, 1, 1000)

        interpolated_points = {}
        interpolator = interp1d(distance, points, kind='next', axis=0)
        interpolated_points = interpolator(alpha)

        return interpolated_points

    def generate_both_shapes(self):

        self.central_waypoints = self.interpolate_waypoints(np.array(list(zip(self.x_origin, self.y_origin))))

        afpoly = shp.Polygon(self.central_waypoints)
        # Create offset airfoils, both inward and outward


        # Change the width of the track
        poffafpoly = afpoly.buffer(1.0666124816191023/2)  # Outward offset
        noffafpoly = afpoly.buffer(-1.0666124816191023/2)  # Inward offset

        # Turn polygon points into numpy arrays for plotting
        self.poffafpolypts = np.array(poffafpoly.exterior)
        self.noffafpolypts = np.array(noffafpoly.exterior)

        self.poffafpolypts = self.interpolate_waypoints(self.poffafpolypts)
        self.noffafpolypts = self.interpolate_waypoints(self.noffafpolypts)

        if show_animation:
            fig, axs = plt.subplots(2, 2, figsize=(10, 8), constrained_layout=True)
            axs[0, 0].plot(*self.central_waypoints.T)
            axs[0, 0].set_title('Central Line')
            axs[0, 0].set_xlabel('X')
            axs[0, 0].set_ylabel('Y')
            axs[0, 0].plot(self.x_origin, self.y_origin)
            fig.suptitle('Track generated', fontsize=16)
            axs[0, 1].plot(*self.noffafpolypts.T)
            axs[0, 1].set_xlabel('X')
            axs[0, 1].set_title('Outward offset')
            axs[0, 1].set_ylabel('Y')

            axs[1, 0].plot(*self.poffafpolypts.T)
            axs[1, 0].set_xlabel('X')
            axs[1, 0].set_title('Inward offset')
            axs[1, 0].set_ylabel('Y')

            axs[1, 1].plot(*self.poffafpolypts.T)
            axs[1, 1].plot(*self.noffafpolypts.T)
            axs[1, 1].plot(*self.central_waypoints.T)
            axs[1, 1].plot(self.x_origin, self.y_origin)
            axs[1, 1].set_xlabel('X')
            axs[1, 1].set_title('Total track')
            axs[1, 1].set_ylabel('Y')
            plt.show()

        self.all_coordinates = pd.concat([pd.DataFrame(self.poffafpolypts, columns=["X_ext", 'Y_ext']),
                                          pd.DataFrame(self.noffafpolypts, columns=["X_int", 'Y_int']),
                                          pd.DataFrame(self.central_waypoints, columns=["X_central", 'Y_central'])],
                                         axis=1)

        self.mindf = abs(min(self.all_coordinates.min()))
        self.all_coordinates = round((self.all_coordinates + self.mindf) * self.SCALE_COEFF)
        self.all_coordinates.to_csv('all_coordinates_scaled.csv', index=False)

    def generate_shortest_path(self):

        print(" Computing the shortest path !!")

        all_coordinates = self.all_coordinates

        # start and goal position
        sx = int(all_coordinates.X_central.iloc[10])  # [m]
        sy = int(all_coordinates.Y_central.iloc[10])  # [m]
        gx = int(all_coordinates.X_central.iloc[len(all_coordinates) - 20])  # [m]
        gy = int(all_coordinates.Y_central.iloc[len(all_coordinates) - 20])  # [m]
        grid_size = 1  # [m]
        robot_radius = 1  # [m]

        ox = [int(round(x)) for x in all_coordinates.X_ext[1:]] + [int(round(x)) for x in all_coordinates.X_int[:-1]]
        oy = [int(round(x)) for x in all_coordinates.Y_ext[1:]] + [int(round(x)) for x in all_coordinates.Y_int[:-1]]

        print("start enhanced")

        for i in range(0, self.INTERPOL_COEFF):
            ox_enhanced = [x + (ox[i + 1] - ox[i]) / 2 for i, (x, y) in enumerate(zip(ox, oy)) if i + 1 < len(ox)] + ox
            oy_enhanced = [y + (oy[i + 1] - oy[i]) / 2 for i, (x, y) in enumerate(zip(ox, oy)) if i + 1 < len(oy)] + oy
            ox = ox_enhanced
            oy = oy_enhanced




        # Add barrieres add the beginning of the track (use it for Reinvent 2018)
        #ox = ox + [int(all_coordinates.X_central.iloc[0])] * (int(self.SCALE_COEFF / 2) + 5)
        #oy = oy + [x + int(all_coordinates.Y_central.iloc[0]) for x in range(-5, int(self.SCALE_COEFF / 2))]

        print("enhanced finished")
        if show_animation:
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")

        a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
        rx, ry = a_star.planning(sx, sy, gx, gy)

        if show_animation:  # pragma: no cover
            plt.plot(rx, ry, "-r")
            plt.show()

        print("Export shortest path")
        shortest_path_df = pd.DataFrame(list(zip(rx, ry)),
                                        columns=["xr", 'yr'])
        shortest_path_df = shortest_path_df.drop_duplicates()
        shortest_path_df.to_csv('shortest_path_points.csv', index=True)

        return rx, ry

    def improve_shorted_path(self):

        cx, cy = main()

        path_generated = pd.DataFrame(list(zip(cx, cy)), columns=["xr", 'yr'])
        path_generated = path_generated.drop_duplicates()


        ox = path_generated.xr
        oy = path_generated.yr

        self.downsampl = [(x, y) for i, (x, y) in enumerate(zip(ox, oy))]
        self.downsampl = (pd.DataFrame(self.downsampl, columns=['ox', 'oy'])) / (
            self.SCALE_COEFF) - self.mindf  # <---- CHANGE THIS
        self.downsampl = self.downsampl.iloc[::int(len(self.downsampl) / 179), :].reset_index()

        # Windows version
        # filename = str('data/_' + str(reinventTrack.INTERPOL_COEFF) + '__' + str(reinventTrack.SCALE_COEFF) + '__' + timestr)

        exportfile = open(self.filename + "WP" + str('.txt'), 'w')

        for element in zip(self.downsampl.ox, self.downsampl.oy):
            exportfile.write(str(element) + ",")
            exportfile.write('\n')
        exportfile.close()

    def generated_speedpoints(self):

        downsampl = self.downsampl
        speedpoints = []

        SEUIL_ANGLE_MEDIUM = 20
        SEUIL_ANGLE_SMALL = 5
        STEP = 3

        for i in range(0, len(downsampl)):
            if i + STEP + 1 < len(downsampl) and i > STEP:

                angle = abs(180 - getAngle([downsampl.ox.iloc[i - STEP], downsampl.oy.iloc[i - STEP]],
                                           [downsampl.ox.iloc[i], downsampl.oy.iloc[i]],
                                           [downsampl.ox.iloc[i + STEP], downsampl.oy.iloc[i + STEP]]))

                if angle < SEUIL_ANGLE_SMALL:
                    speedpoints.append(1)
                elif SEUIL_ANGLE_SMALL < angle < SEUIL_ANGLE_MEDIUM:
                    speedpoints.append(2)
                elif angle > SEUIL_ANGLE_MEDIUM:
                    speedpoints.append(3)
            else:
                speedpoints.append(1)

        self.speedpoints = speedpoints

        return speedpoints

    def export_result(self):

        labels = self.speedpoints

        exportfile = open(self.filename + "SP" + str('.txt'), 'w')

        for element in self.speedpoints:
            exportfile.write(str(element) + ",")
            exportfile.write('\n')
        exportfile.close()

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(35, 35))  # create figure & 1 axis
        plt.scatter(self.downsampl.ox, self.downsampl.oy, c='red', s=0.5)

        for i, txt in enumerate(labels):
            ax.annotate(txt, (self.downsampl.ox[i], self.downsampl.oy[i]))
        fig.savefig(self.filename + '.png')  # save the figure to file
        plt.close(fig)


reinventTrack = generate_Track()

reinventTrack.generate_both_shapes()

rx, ry = reinventTrack.generate_shortest_path()

reinventTrack.improve_shorted_path()

reinventTrack.generated_speedpoints()

reinventTrack.export_result()
