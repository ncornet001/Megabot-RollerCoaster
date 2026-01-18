####################################################
#                                                  #
#       MOVE THIS FILE TO THE MAIN FOLDER          #
#                NEXT TO FULL.PY                   #
#                                                  #
####################################################

import time
import sys
import numpy as np
import pinocchio as pin
import placo
from placo_utils.visualization import robot_viz, frame_viz, point_viz, robot_frame_viz, line_viz, get_viewer
from placo_utils.tf import tf
import math
import megabot
import com
from threading import Thread, Lock


from geo import isCoplanar, inTriangle2DXY,X,Y,Z
from js import Joystick
from ground import putRobotOnTheGround
from task import Task
from classic_walk import ClassicWalkTask


import PyQt6
from PyQt6.QtCore import QObject, QThread
from PyQt6.QtWidgets import QApplication,QMainWindow, QVBoxLayout,QWidget, QHBoxLayout, QPushButton, QDialog,QLabel, QStatusBar, QTabWidget, QSlider
from PyQt6.QtWebChannel import QWebChannel
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl, QSize
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QPixmap,QIcon
import pyqtgraph as pg
import time
import os

import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize
import csv
import json
from pathlib import Path
from queue import Queue

os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "0"
np.set_printoptions(precision=3,suppress=True)


robot = megabot.load()
virtualRobot=megabot.load()

viz = robot_viz(robot)

# put robot on the world center by default
robot.set_T_world_frame("base", np.eye(4))
robot.update_kinematics()

robotLock=Lock()
virtualRobotLock=Lock()

# open joystick using evdev:
joystick=Joystick()


com=com.ControlerHandler()
com.send_info(0.05) # ask for position update every 50ms
#TODO: change log file name to include date and time
com.startlog(time.strftime("logs/log__%Y_%m_%d@%H:%M.txt"))

class UpdateRobotAccordingToReal(Task):
    def __init__(self) -> None:
        super().__init__()
        self.solver = megabot.make_solver(robot)
        # update robot to actual actuator positions
        print("updating robot, com is fake:",com.fakeMode)

        self.init_joints_task = self.solver.add_joints_task()
        for l in range(4):
            for a in range(3):
                self.init_joints_task.configure(f"l{l+1}_c{a+1}", "soft", 1.0)
                self.init_joints_task.set_joint(f"l{l+1}_c{a+1}", com.legs[l][a]['position'])
                #print(com.legs[l][a])
                #print("leg ",l+1," actuator ",a+1," is at ",com.legs[l][a]['position'])
        self.regularization_task = self.solver.add_regularization_task(1e-2) # this will "slow down" the resolution...
        self.ncall=0
    def tick(self):
        self.ncall+=1
        self.solver.solve(True)
        robotLock.acquire()
        robot.update_kinematics()
        robotLock.release()
        max_error=0
        for l in range(4):
            for a in range(3):
                max_error=max(max_error,abs(robot.get_joint(f"l{l+1}_c{a+1}")-com.legs[l][a]['position']))
        if max_error<0.0001 or self.ncall>20:
            return True
        return False
    def name(self):
        return "UpdateRobotAccordingToReal"

class MoveLegsStaticPositions(Task):
    def __init__(self,targets) -> None:
        super().__init__()
        self.solver = megabot.make_solver(robot)
        self.targets=targets
        self.legs_task = []
        for l in range(4):
            current_position=robot.get_T_world_frame(f"leg_{l+1}")[:,3][:3]
            task = self.solver.add_position_task(f"leg_{l+1}",current_position)
            task.configure(f"leg_{l+1}", "soft", 10.0)
            self.legs_task.append(task)
        self.base_task = self.solver.add_frame_task("base", robot.get_T_world_frame("base"))
        self.base_task.configure("base", "hard", 1.0,1.0)
        self.start_time=time.time()
        self.current_leg=0
        self.current_leg_height=robot.get_T_world_frame(f"leg_{self.current_leg+1}")[2,3]
        self.state="up" # move / down
        self.in_state=time.time()
    def tick(self):
        print("move leg ",self.current_leg+1," state is ",self.state)
        if self.state=="up":
            target=robot.get_T_world_frame(f"leg_{self.current_leg+1}")[:,3][:3].copy()
            target[Z]+=0.1
            print("moving leg ",self.current_leg+1," from ",self.current_leg_height," to ",target[Z])
            self.legs_task[self.current_leg].target_world=target
            if abs(self.current_leg_height+0.05-robot.get_T_world_frame(f"leg_{self.current_leg+1}")[2,3])<0.01 or time.time()-self.in_state>5:
                self.state="move"
                self.in_state=time.time()
        elif self.state=="move":
            target=robot.get_T_world_frame(f"leg_{self.current_leg+1}")[:,3][:3].copy()
            target[X]=self.targets[self.current_leg][0]
            target[Y]=self.targets[self.current_leg][1]
            self.legs_task[self.current_leg].target_world=target
            if (abs(target[X]-robot.get_T_world_frame(f"leg_{self.current_leg+1}")[0,3])<0.01 and abs(target[Y]-robot.get_T_world_frame(f"leg_{self.current_leg+1}")[1,3])<0.01) or time.time()-self.in_state>5:
                self.state="down"
                self.in_state=time.time()
        elif self.state=="down":
            target=robot.get_T_world_frame(f"leg_{self.current_leg+1}")[:,3][:3].copy()
            target[Z]=self.current_leg_height
            self.legs_task[self.current_leg].target_world=target
            if abs(target[Z]-robot.get_T_world_frame(f"leg_{self.current_leg+1}")[2,3])<0.01 or time.time()-self.in_state>5:
                self.current_leg+=1
                self.in_state=time.time()
                self.current_leg_height=robot.get_T_world_frame(f"leg_{self.current_leg+1}")[2,3] if self.current_leg<4 else 0
                if self.current_leg==4:
                    return True
                self.state="up"
        self.solver.solve(True)
        robotLock.acquire()
        robot.update_kinematics()
        robotLock.release()
        for l in [1,2,3,4]:
            for c in [1,2,3]:
                com.send_move(l,c,robot.get_joint(f"l{l}_c{c}"),0.3)
        return False

class MoveActuatorsStaticPositions(Task):
    def __init__(self,targets) -> None:
        super().__init__()
        self.solver = megabot.make_solver(robot)
        self.targets=targets
        self.init_joints_task = self.solver.add_joints_task()
        for l in range(4):
            for a in range(3):
                self.init_joints_task.configure(f"l{l+1}_c{a+1}", "soft", 1.0)
                self.init_joints_task.set_joint(f"l{l+1}_c{a+1}", com.legs[l][a]['position'])
        self.regularization_task = self.solver.add_regularization_task(1e-2)
        self.step=0
        self.actuator=1 # move actuator 1, then 2, then 0
        self.power=0.5

    def tick(self):
        if self.step==0:
            if self.actuator<0:
                com.send_stop()
                return True
            for l in [1,2,3,4]:
                com.send_move(l,self.actuator+1,self.targets[l-1][self.actuator],self.power)
            self.step=1
        elif self.step==1:
            error=0
            for l in [1,2,3,4]:
                error=max(error,abs(com.legs[l-1][self.actuator]['position']-self.targets[l-1][self.actuator]))
            if error<0.01:
                print("for actuator ",self.actuator," error is ",error)
                self.step=0
                self.power=0.5
                if self.actuator==1:
                    self.actuator=2
                elif self.actuator==2:
                    self.actuator=0
                else:
                    self.actuator-=1
            else:
                self.power+=0.01
        # update robots to actual actuator positions
        for l in range(4):
            for a in range(3):
                self.init_joints_task.set_joint(f"l{l+1}_c{a+1}", com.legs[l][a]['position'])
        self.solver.solve(True)
        robotLock.acquire()
        robot.update_kinematics()
        robotLock.release()
        return False
    def name(self):
        return "MoveLegsStaticPositions"+str(self.targets)


class TestPosition(Task):
    def __init__(self) -> None:
        super().__init__(0.005)
        self.solver = megabot.make_solver(robot)
        T_world_base=robot.get_T_world_frame("base")
        self.base_height=T_world_base[2,3]
        self.height_dir=1
        self.speed=1.2
        self.base_task = self.solver.add_position_task("base",T_world_base[:,3][:3])
        self.base_task.configure("base", "soft", 1.0)
        base_orientation_task = self.solver.add_orientation_task("base",T_world_base[:3,:3])
        base_orientation_task.configure("base", "hard", 1.0)
        for l in [1,2,3,4]:
            current_position=robot.get_T_world_frame(f"leg_{l}")[:,3][:3]
            task = self.solver.add_position_task(f"leg_{l}",current_position)
            task.configure(f"leg_{l}", "hard", 1.0) # keep legs at their position
        self.state="start"

        self.max_trans = 0.35
        self.point_count = 0
        self.solve_count = 0
        self.x = 0  # en metres
        self.y = 0  # en metres
        self.filename = 'position_mapping.csv'
        self.data = []
        # Création d'une colormap personnalisée: vert -> bleu -> gris foncé
        colors = ['#00FF00', '#0080FF', '#8A8A8A']  # Vert, Bleu, Gris
        n_bins = 256
        self.custom_cmap = LinearSegmentedColormap.from_list('custom', colors, N=n_bins)
        self.grid = 50

    def dtTick(self):
        base_position=robot.get_T_world_frame("base")[:,3][:3].copy()
        if self.state=="start":
            print("danse:start : ",base_position[X])
            # move slowly base to startup position
            base_position[X]+=self.dt/4.0
            self.base_task.target_world=base_position
            if abs(base_position[X]-0.1)<0.02:
                print("running")
                self.state="running"
                self.x = 0
                self.y = 0
                self.solver.solve(True)
                self.start_time=time.time()

        elif self.state=="end": # back to zero
            print("danse:end")
            com.send_stop()
            self.plot_capability_map(grid_size=self.grid, show_plot=False, max_trans=self.max_trans)
            # self.plot_capability_map_old()
            return True

        else: # running
            # print("danse:run")
            t=time.time()-self.start_time

            if self.point_count >= self.grid*self.grid:
                print("end")
                self.state="end"
                self.start_time=time.time()

            if self.solve_count >= 4:
                self.solve_count = 0
                print("Count : ", self.point_count, "  X : ", round(self.x,2), "  Y : ", round(self.y,2), "  error:",self.base_task.error_norm())
                self.data.append({
                'x': self.x,
                'y': self.y,
                'error': self.base_task.error_norm()
                })

                if self.base_task.error_norm() > 0.02 and self.point_count%(2*self.grid) < self.grid:
                    self.point_count += 2*(self.grid-self.point_count%self.grid) + 1

                if self.point_count%(2*self.grid) < self.grid:
                    self.x = self.max_trans*(self.point_count%self.grid)/self.grid
                else:
                    self.x = self.max_trans*(self.grid-1-self.point_count%self.grid)/self.grid

                self.y = self.max_trans*int(self.point_count/self.grid)/self.grid

                self.point_count+=1
                self.start_time=time.time()

            base_position[X]=self.x
            base_position[Y]=self.y
            base_position[Z]=self.base_height
            self.base_task.target_world=base_position


        self.solver.solve(True)
        self.solve_count+=1
        # print("error:",self.base_task.error_norm())
        robotLock.acquire()
        robot.update_kinematics()
        robotLock.release()
        for l in [1,2,3,4]:
            for c in [1,2,3]:
                com.send_move(l,c,robot.get_joint(f"l{l}_c{c}"))
        return False

    def save_to_csv(self):
        """Sauvegarde les données dans un fichier CSV"""
        with open(self.filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['x', 'y', 'error'])
            writer.writeheader()
            writer.writerows(self.data)
        print(f"Données sauvegardées dans {self.filename}")

    def plot_capability_map(self, save_figure=True, figname='position_capability_map.png', show_plot=False, grid_size=60, max_trans=0.30):
        """
        Affiche la carte des capacités en 2D avec une grille de pixels colorés
        Gradient: Vert (≤0.001) -> Bleu -> Gris foncé (≥0.02)

        Args:
            save_figure: si True, sauvegarde la figure
            figname: nom du fichier de sortie
            show_plot: si True, affiche le graphique (à mettre False dans un thread)
            grid_size: nombre de pixels par axe (60x60 = 3600 pixels)
        """
        if not self.data:
            print("Aucune donnée à afficher. Collectez ou chargez des données d'abord.")
            return

        # Extraction des données
        xs = np.array([d['x'] for d in self.data])
        ys = np.array([d['y'] for d in self.data])
        errors = np.array([d['error'] for d in self.data])

        # Déterminer les limites
        x_min, x_max = xs.min(), xs.max()
        y_min, y_max = ys.min(), ys.max()

        # Créer une grille 2D pour les erreurs
        error_grid = np.full((grid_size, grid_size), np.nan)

        # Remplir la grille avec les données
        # Calcul des indices de grille pour chaque point avec un epsilon pour éviter les erreurs d'arrondi
        x_indices = np.clip(
            np.round(xs / max_trans * (grid_size - 1)).astype(int),
            0, grid_size - 1
        )
        y_indices = np.clip(
            np.round(ys / max_trans * (grid_size - 1)).astype(int),
            0, grid_size - 1
        )

        # Assigner les valeurs d'erreur à la grille
        for i in range(len(errors)):
            error_grid[y_indices[i], x_indices[i]] = errors[i]

        # Vérifier s'il y a des valeurs manquantes
        missing_count = np.isnan(error_grid).sum()
        if missing_count > 0:
            print(f"⚠️ Attention: {missing_count} pixels manquants sur {grid_size*grid_size}")
            print(f"   Pixels remplis: {len(errors)} / {grid_size*grid_size}")

        # Création de la figure
        fig, ax = plt.subplots(figsize=(10, 8))

        # Normalisation des couleurs entre 0.001 et 0.02
        norm = Normalize(vmin=0.001, vmax=0.02)

        # Affichage de la grille de pixels
        im = ax.imshow(error_grid,
                      cmap=self.custom_cmap,
                      norm=norm,
                      origin='lower',
                      extent=[0, max_trans, 0, max_trans],
                      aspect='auto',
                      interpolation='nearest')

        # Colorbar
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Erreur du simulateur', rotation=270, labelpad=20)

        # Labels et titre
        ax.set_xlabel('x (degrés)', fontsize=12)
        ax.set_ylabel('y (degrés)', fontsize=12)
        ax.set_title('Cartographie des capacités du megabot\n(x/y vs Erreur)',
                    fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.2, color='white', linewidth=0.5)

        # Statistiques
        valid_errors = errors[~np.isnan(errors)]
        error_min = valid_errors.min()
        error_max = valid_errors.max()
        error_mean = valid_errors.mean()

        # Ajout des statistiques sur le graphique
        stats_text = f'Statistiques:\n'
        stats_text += f'Erreur min: {error_min:.4f}\n'
        stats_text += f'Erreur max: {error_max:.4f}\n'
        stats_text += f'Grille: {grid_size}x{grid_size}\n'
        stats_text += f'Échantillons: {len(self.data)}'

        ax.text(0.98, 0.98, stats_text, transform=ax.transAxes,
               fontsize=10, verticalalignment='top', horizontalalignment='right',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        plt.tight_layout()

        if save_figure:
            plt.savefig(figname, dpi=300, bbox_inches='tight')
            print(f"Figure sauvegardée dans {figname}")

        if show_plot:
            plt.show()
        else:
            plt.close()


    def plot_capability_map_old(self, save_figure=True, figname='position_capability_map.png'):
        """
        Affiche la carte des capacités en 2D avec couleur selon l'erreur
        Gradient: Vert (≤0.001) -> Bleu -> Gris foncé (≥0.02)

        Args:
            save_figure: si True, sauvegarde la figure
            figname: nom du fichier de sortie
        """
        if not self.data:
            print("Aucune donnée à afficher. Collectez ou chargez des données d'abord.")
            return

        # Extraction des données
        xs = [d['x'] for d in self.data]
        ys = [d['y'] for d in self.data]
        errors = [d['error'] for d in self.data]

        # Création de la figure
        fig, ax = plt.subplots(figsize=(10, 8))

        # Normalisation des couleurs entre 0.001 et 0.02
        norm = Normalize(vmin=0.001, vmax=0.02)

        # Nuage de points avec couleur selon l'erreur
        scatter = ax.scatter(xs, ys, c=errors,
                           cmap=self.custom_cmap,  # Colormap personnalisée
                           norm=norm,
                           s=50, alpha=0.7, edgecolors='black', linewidth=0.5)

        # Colorbar
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Erreur du simulateur', rotation=270, labelpad=20)

        # Labels et titre
        ax.set_xlabel('x (degrés)', fontsize=12)
        ax.set_ylabel('y (degrés)', fontsize=12)
        ax.set_title('Cartographie des capacités du robot\n(x/y vs Erreur)',
                    fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        # Statistiques
        error_min = min(errors)
        error_max = max(errors)
        error_mean = np.mean(errors)

        # Ajout des statistiques sur le graphique
        stats_text = f'Statistiques:\n'
        stats_text += f'Erreur min: {error_min:.4f}\n'
        stats_text += f'Erreur max: {error_max:.4f}\n'
        stats_text += f'Erreur moy: {error_mean:.4f}\n'
        stats_text += f'Échantillons: {len(self.data)}'

        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
               fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        plt.tight_layout()

        if save_figure:
            plt.savefig(figname, dpi=300, bbox_inches='tight')
            print(f"Figure sauvegardée dans {figname}")

        plt.show()


class TestRotation(Task):
    def __init__(self) -> None:
        super().__init__(0.005)
        self.solver = megabot.make_solver(robot)
        T_world_base=robot.get_T_world_frame("base")
        self.base_height=T_world_base[2,3]
        self.height_dir=1
        self.speed=1.2
        self.base_task = self.solver.add_position_task("base",T_world_base[:,3][:3])
        self.base_task.configure("base", "hard", 1.0)
        self.base_orientation_task = self.solver.add_orientation_task("base",T_world_base[:3,:3])
        self.base_orientation_task.configure("base", "soft", 100.0)
        for l in [1,2,3,4]:
            current_position=robot.get_T_world_frame(f"leg_{l}")[:,3][:3]
            task = self.solver.add_position_task(f"leg_{l}",current_position)
            task.configure(f"leg_{l}", "hard", 1.0) # keep legs at their position
        self.state="start"

        self.initial_rotation=T_world_base[:3,:3].copy()
        print("initial rotation is ",self.initial_rotation)
        self.max_angle=math.pi*7.0/180.0
        self.point_count = 0
        self.solve_count = 0
        self.roll = 0
        self.pitch = 0
        self.filename = 'rotation_mapping.csv'
        self.data = []
        # Création d'une colormap personnalisée: vert -> bleu -> gris foncé
        colors = ['#00FF00', '#0080FF', '#8A8A8A']  # Vert, Bleu, Gris
        n_bins = 256
        self.custom_cmap = LinearSegmentedColormap.from_list('custom', colors, N=n_bins)
        self.grid = 20

        self.start_time=time.time()

    def dtTick(self):
        base_position=robot.get_T_world_frame("base")[:,3][:3].copy()

        if self.state=="start":
            # set start rotation to : (0,self.magnitude,0)
            t=time.time()-self.start_time
            print("TestRotation start ",t)
            self.base_orientation_task.R_world_frame=pin.rpy.rpyToMatrix(0,0,0)
            if t>1.0:
                print("running")
                self.state="running"
                self.roll = 0
                self.pitch = 0
                self.solver.solve(True)
                self.start_time=time.time()
            #print("danse:start : ",base_position[X])
            # move slowly base to startup position
            #base_position[X]+=self.dt/4.0
            #self.base_task.target_world=base_position
            #if abs(base_position[X]-0.1)<0.02:
            #    print("running")
        elif self.state=="end": # back to zero
            t=time.time()-self.start_time
            self.base_orientation_task.R_world_frame=pin.rpy.rpyToMatrix(0,0,0)@self.initial_rotation
            if  t>2.0:
                com.send_stop()
                # self.save_to_csv()
                # Affichage de la carte
                # self.plot_capability_map(grid_size=self.grid, show_plot=False)
                self.plot_capability_map_old()
                # Arrêter l'affichage
                # self.plotting_active = False
                # plot_thread.join()
                return True
        else: # running
            # print("TestRotation:run")
            t=time.time()-self.start_time
            self.base_orientation_task.R_world_frame=pin.rpy.rpyToMatrix(self.roll,self.pitch,0)@self.initial_rotation

            if self.solve_count >= 4:
                self.solve_count = 0
                print("Count : ", self.point_count, "  Roll : ", round(self.roll*180/np.pi,2), "  Pitch : ", round(self.pitch*180/np.pi,2), "  error:",self.base_orientation_task.error_norm())
                self.data.append({
                'roll': self.roll*180/np.pi,
                'pitch': self.pitch*180/np.pi,
                'error': self.base_orientation_task.error_norm()
                })
                # self.data_queue.put(self.data[-1])
                # print("next point")
                # self.roll = 2*self.max_angle*np.random.rand() - self.max_angle
                # self.pitch = 2*self.max_angle*np.random.rand() - self.max_angle

                # self.roll = self.max_angle*np.random.rand()
                # self.pitch = self.max_angle*np.random.rand()

                if self.point_count < self.grid*self.grid:

                    if self.base_orientation_task.error_norm() > 0.02 and self.point_count%(2*self.grid) < self.grid:
                        self.point_count += 2*(self.grid-self.point_count%self.grid) + 1

                    if self.point_count%(2*self.grid) < self.grid:
                        self.roll = self.max_angle*(self.point_count%self.grid)/self.grid
                    else:
                        self.roll = self.max_angle*(self.grid-1-self.point_count%self.grid)/self.grid

                    self.pitch = self.max_angle*int(self.point_count/self.grid)/self.grid

                elif self.point_count < self.grid*self.grid*2:

                    if self.base_orientation_task.error_norm() > 0.02 and self.point_count%(2*self.grid) < self.grid:
                        self.point_count += 2*(self.grid-self.point_count%self.grid) + 1

                    if self.point_count%(2*self.grid) < self.grid:
                        self.roll = -self.max_angle*(self.point_count%self.grid)/self.grid
                    else:
                        self.roll = -self.max_angle*(self.grid-1-self.point_count%self.grid)/self.grid

                    self.pitch = self.max_angle*int((self.point_count%(self.grid*self.grid))/self.grid)/self.grid

                elif self.point_count < self.grid*self.grid*3:

                    if self.base_orientation_task.error_norm() > 0.02 and self.point_count%(2*self.grid) < self.grid:
                        self.point_count += 2*(self.grid-self.point_count%self.grid) + 1

                    if self.point_count%(2*self.grid) < self.grid:
                        self.roll = self.max_angle*(self.point_count%self.grid)/self.grid
                    else:
                        self.roll = self.max_angle*(self.grid-1-self.point_count%self.grid)/self.grid

                    self.pitch = -self.max_angle*int((self.point_count%(self.grid*self.grid))/self.grid)/self.grid

                else:

                    if self.base_orientation_task.error_norm() > 0.02 and self.point_count%(2*self.grid) < self.grid:
                        self.point_count += 2*(self.grid-self.point_count%self.grid) + 1

                    if self.point_count%(2*self.grid) < self.grid:
                        self.roll = -self.max_angle*(self.point_count%self.grid)/self.grid
                    else:
                        self.roll = -self.max_angle*(self.grid-1-self.point_count%self.grid)/self.grid

                    self.pitch = -self.max_angle*int((self.point_count%(self.grid*self.grid))/self.grid)/self.grid

                self.point_count+=1
                self.start_time=time.time()

            if self.point_count >= self.grid*self.grid*4:
                print("end")
                self.state="end"
                self.last = (self.roll,self.pitch,0)
                self.start_time=time.time()

        self.solver.solve(True)
        self.solve_count+=1
        # print("Count : ", self.point_count, "  Roll : ", round(self.roll,3), "  Pitch : ", self.pitch, "  error:",self.base_orientation_task.error_norm())
        # print("error:",self.base_orientation_task.error_norm())
        robotLock.acquire()
        robot.update_kinematics()
        robotLock.release()
        for l in [1,2,3,4]:
            for c in [1,2,3]:
                com.send_move(l,c,robot.get_joint(f"l{l}_c{c}"))
        return False

    def save_to_csv(self):
        """Sauvegarde les données dans un fichier CSV"""
        with open(self.filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['roll', 'pitch', 'error'])
            writer.writeheader()
            writer.writerows(self.data)
        print(f"Données sauvegardées dans {self.filename}")

    def plot_capability_map(self, save_figure=True, figname='rotation_capability_map.png', show_plot=False, grid_size=60, max_angle_deg=7.0):
        """
        Affiche la carte des capacités en 2D avec une grille de pixels colorés
        Gradient: Vert (≤0.001) -> Bleu -> Gris foncé (≥0.02)

        Args:
            save_figure: si True, sauvegarde la figure
            figname: nom du fichier de sortie
            show_plot: si True, affiche le graphique (à mettre False dans un thread)
            grid_size: nombre de pixels par axe (60x60 = 3600 pixels)
        """
        if not self.data:
            print("Aucune donnée à afficher. Collectez ou chargez des données d'abord.")
            return

        # Extraction des données
        rolls = np.array([d['roll'] for d in self.data])
        pitches = np.array([d['pitch'] for d in self.data])
        errors = np.array([d['error'] for d in self.data])

        # Déterminer les limites
        roll_min, roll_max = rolls.min(), rolls.max()
        pitch_min, pitch_max = pitches.min(), pitches.max()

        # Créer une grille 2D pour les erreurs
        error_grid = np.full((grid_size, grid_size), np.nan)

        # Remplir la grille avec les données
        # Calcul des indices de grille pour chaque point avec un epsilon pour éviter les erreurs d'arrondi
        roll_indices = np.clip(
            np.round(rolls / max_angle_deg * (grid_size - 1)).astype(int),
            0, grid_size - 1
        )
        pitch_indices = np.clip(
            np.round(pitches / max_angle_deg * (grid_size - 1)).astype(int),
            0, grid_size - 1
        )

        # Assigner les valeurs d'erreur à la grille
        for i in range(len(errors)):
            error_grid[pitch_indices[i], roll_indices[i]] = errors[i]

        # Vérifier s'il y a des valeurs manquantes
        missing_count = np.isnan(error_grid).sum()
        if missing_count > 0:
            print(f"⚠️ Attention: {missing_count} pixels manquants sur {grid_size*grid_size}")
            print(f"   Pixels remplis: {len(errors)} / {grid_size*grid_size}")

        # Création de la figure
        fig, ax = plt.subplots(figsize=(10, 8))

        # Normalisation des couleurs entre 0.001 et 0.02
        norm = Normalize(vmin=0.001, vmax=0.02)

        # Affichage de la grille de pixels
        im = ax.imshow(error_grid,
                      cmap=self.custom_cmap,
                      norm=norm,
                      origin='lower',
                      extent=[0, max_angle_deg, 0, max_angle_deg],
                      aspect='auto',
                      interpolation='nearest')

        # Colorbar
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Erreur du simulateur', rotation=270, labelpad=20)

        # Labels et titre
        ax.set_xlabel('Roll (degrés)', fontsize=12)
        ax.set_ylabel('Pitch (degrés)', fontsize=12)
        ax.set_title('Cartographie des capacités du megabot\n(Roll/Pitch vs Erreur)',
                    fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.2, color='white', linewidth=0.5)

        # Statistiques
        valid_errors = errors[~np.isnan(errors)]
        error_min = valid_errors.min()
        error_max = valid_errors.max()
        error_mean = valid_errors.mean()

        # Ajout des statistiques sur le graphique
        stats_text = f'Statistiques:\n'
        stats_text += f'Erreur min: {error_min:.4f}\n'
        stats_text += f'Erreur max: {error_max:.4f}\n'
        stats_text += f'Grille: {grid_size}x{grid_size}\n'
        stats_text += f'Échantillons: {len(self.data)}'

        ax.text(0.98, 0.98, stats_text, transform=ax.transAxes,
               fontsize=10, verticalalignment='top', horizontalalignment='right',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        plt.tight_layout()

        if save_figure:
            plt.savefig(figname, dpi=300, bbox_inches='tight')
            print(f"Figure sauvegardée dans {figname}")

        if show_plot:
            plt.show()
        else:
            plt.close()


    def plot_capability_map_old(self, save_figure=True, figname='rotation_capability_map.png'):
        """
        Affiche la carte des capacités en 2D avec couleur selon l'erreur
        Gradient: Vert (≤0.001) -> Bleu -> Gris foncé (≥0.02)

        Args:
            save_figure: si True, sauvegarde la figure
            figname: nom du fichier de sortie
        """
        if not self.data:
            print("Aucune donnée à afficher. Collectez ou chargez des données d'abord.")
            return

        # Extraction des données
        rolls = [d['roll'] for d in self.data]
        pitches = [d['pitch'] for d in self.data]
        errors = [d['error'] for d in self.data]

        # Création de la figure
        fig, ax = plt.subplots(figsize=(10, 8))

        # Normalisation des couleurs entre 0.001 et 0.02
        norm = Normalize(vmin=0.001, vmax=0.02)

        # Nuage de points avec couleur selon l'erreur
        scatter = ax.scatter(rolls, pitches, c=errors,
                           cmap=self.custom_cmap,  # Colormap personnalisée
                           norm=norm,
                           s=50, alpha=0.7, edgecolors='black', linewidth=0.5)

        # Colorbar
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Erreur du simulateur', rotation=270, labelpad=20)

        # Labels et titre
        ax.set_xlabel('Roll (degrés)', fontsize=12)
        ax.set_ylabel('Pitch (degrés)', fontsize=12)
        ax.set_title('Cartographie des capacités du robot\n(Roll/Pitch vs Erreur)',
                    fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        # Statistiques
        error_min = min(errors)
        error_max = max(errors)
        error_mean = np.mean(errors)

        # Ajout des statistiques sur le graphique
        stats_text = f'Statistiques:\n'
        stats_text += f'Erreur min: {error_min:.4f}\n'
        stats_text += f'Erreur max: {error_max:.4f}\n'
        stats_text += f'Erreur moy: {error_mean:.4f}\n'
        stats_text += f'Échantillons: {len(self.data)}'

        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
               fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        plt.tight_layout()

        if save_figure:
            plt.savefig(figname, dpi=300, bbox_inches='tight')
            print(f"Figure sauvegardée dans {figname}")

        plt.show()


class VisLegAccessibility:
    def __init__(self):
        virtualRobot.state.q=robot.state.q.copy()
        virtualRobot.update_kinematics()
        self.solver = megabot.make_solver(virtualRobot)
        self.legs_tasks=[]
        self.legs_minmax=[]
        for l in [1,2,3,4]:
            current_position=virtualRobot.get_T_world_frame(f"leg_{l}")[:,3][:3]
            task = self.solver.add_position_task(f"leg_{l}",current_position)
            task.configure(f"leg_{l}", "soft", 1.0)
            self.legs_tasks.append(task)
            self.legs_minmax.append([current_position.copy(),current_position.copy()])
        T_world_base = virtualRobot.get_T_world_frame("base")
        self.base_task = self.solver.add_frame_task("base", T_world_base)
        #self.base_task.configure("base", "soft", 1.0,1.0)
        #for i in range(10):
        #    self.solver.solve(True)
        #    robotLock.acquire()
        #    robot.update_kinematics()
        #    robotLock.release()
        # lock the base:
        self.base_task.configure("base", "hard", 1.0,1.0)
        self.dstep=0.01 # i.e. 1cm
        self.direction=[1.0,1.0] # X,Y normed vector to have a length of 1cm
        self.direction/=(100.0*np.linalg.norm(self.direction))
    def setDirection(self,direction):
        self.direction=direction
        print("new direction is ",self.direction)
        self.direction/=(100.0*np.linalg.norm(self.direction))
        print("new direction is ",self.direction)
    def getMinMax(self,axe):
        return self.legs_minmax[axe]
    def getAccessibility(self,leg):
        return self.legs_minmax[leg-1]
    def getMaxMovement(self,leg):
        # get leg position
        current_position=robot.get_T_world_frame(f"leg_{leg}")[:,3][:3]
        # compute the max movement in the direction of the vector
        position = current_position.copy()
        position[Z]=0 # leg is supposed to be on the ground
        # return distance between current position and max position
        return np.linalg.norm(position-self.legs_minmax[leg-1][1])
    def updateSolver(self):
        for l in [1,2,3,4]:
            current_position=virtualRobot.get_T_world_frame(f"leg_{l}")[:,3][:3]
            self.legs_tasks[l-1].target_world=current_position
            self.legs_tasks[l-1].configure(f"leg_{l}", "soft", 1.0)
        self.base_task.configure("base", "hard", 1.0,1.0)
        self.base_task.T_world_frame=virtualRobot.get_T_world_frame("base")
        for i in range(5):
            self.solver.solve(True)
            virtualRobot.update_kinematics()
    def computeMinMax(self):
        robotLock.acquire() # need to lock robot to avoid conflict with meshcat thread
        virtualRobot.state.q=robot.state.q.copy()
        robotLock.release()
        virtualRobot.update_kinematics()
        self.updateSolver()
        first_run=False
        for l in [1,2,3,4]:
            for k in [1,2,3,4]:
                if k!=l:
                    self.legs_tasks[k-1].configure(f"leg_{k}", "soft", 1.0)
            self.legs_tasks[l-1].configure(f"leg_{l}", "soft", 100.0)
            current_position=virtualRobot.get_T_world_frame(f"leg_{l}")[:,3][:3]
            position = current_position.copy()
            error=0
            first_run=True
            while error<0.005: #5mm from target
                position[X]+=self.direction[X]
                position[Y]+=self.direction[Y]
                nbIter= 10 if first_run else 2
                try:
                    for i in range(nbIter):
                        self.legs_tasks[l-1].target_world=position
                        self.solver.solve(True)
                        self.solver.robot.update_kinematics()
                    error=self.legs_tasks[l-1].error_norm()
                except Exception as e:
                    print(e)
                    error=10
                first_run=False
            posmax=[position[X],position[Y]]
            error=0
            position=current_position.copy()
            first_run=True
            while error<0.001:
                position[X]-=self.direction[X]
                position[Y]-=self.direction[Y]
                nbIter= 10 if first_run else 2
                try:
                    for i in range(nbIter):
                        self.legs_tasks[l-1].target_world=position
                        self.solver.solve(True)
                        self.solver.robot.update_kinematics()
                    error=self.legs_tasks[l-1].error_norm()
                except:
                    error=10
                first_run=False
            posmin=[position[X],position[Y]]
            self.legs_minmax[l-1]=[[posmin[X],posmin[Y],0],[posmax[X],posmax[Y],0]]
             # move leg back to its original position
            self.legs_tasks[l-1].target_world=current_position
            first_run=True
        # put back robot in its original position : useless with virtualRobot
        #for l in [1,2,3,4]:
        #    self.legs_tasks[l-1].configure(f"leg_{l}", "soft", 1.0)
        #for i in range(10):
        #    self.solver.solve(True)
        #    robot.update_kinematics()
        #robotLock.release()
    def onVis(self,mc):
        self.computeMinMax()
        for l in [1,2,3,4]:
            point_viz(f"p{l}_x_min",self.legs_minmax[l-1][0],radius=0.02,color=0xFF0000)
            point_viz(f"p{l}_x_max",self.legs_minmax[l-1][1],radius=0.02,color=0xFF0000)
        print("max movement for leg 1:",self.getMaxMovement(1))
        print(self.legs_minmax)
    def clearVis(self):
        print("clearing vis legs accessibility")
        v=get_viewer()
        try:
            print("points:",v["point"])
            for l in [1,2,3,4]:
                print(v,l)
                v["point"][f"p{l}_x_min"].delete()
                v["point"][f"p{l}_x_max"].delete()
        except Exception as e:
            print("clear vis in leg access exception:",e)

def robotSaveState(robot):
    return robot.state.q.copy()

def robotRestoreState(robot,state):
    robot.state.q=state
    robot.update_kinematics()

class GroundRobotTask(Task):
    def __init__(self) -> None:
        super().__init__(0.05) # ask for 20Hz ticking
    def dtTick(self):
        putRobotOnTheGround(robot,robotLock)
        return True # leave the task

class TaskThread(Thread):
    def __init__(self,mw):
        Thread.__init__(self)
        self.currentTask=None
        self.shouldStop=False
        self.lock = Lock()
        self.pause=False
        self.mainWindow=mw
    def stop(self):
        self.shouldStop=True
    def setPause(self,p):
        self.pause=p
    def setCurrentTask(self,task):
        if task !=None:
            print("setting current task to ",task.name())
        else:
            print("setting current task to None")
        self.lock.acquire()
        if self.currentTask!=None:
            print("calling leaving on ",self.currentTask.name())
            self.mainWindow.meshcat.taskLeaving(self.currentTask)
        self.currentTask=task
        if task==None:
            com.send_stop()
        self.lock.release()
    def run(self):
        while self.shouldStop==False:
            if self.pause==False:
                if self.currentTask!=None:
                    self.lock.acquire()
                    if self.currentTask.tick():
                        print("task ",self.currentTask.name()," is done: leaving")
                        self.mainWindow.meshcat.taskLeaving(self.currentTask)
                        self.currentTask=None
                        com.send_stop()
                    self.lock.release()
                    try:
                        if self.currentTask!=None:
                            self.mainWindow.updateStatus(self.currentTask.status())
                        else:
                            self.mainWindow.updateStatus("No task")
                    except Exception as e:
                        print("error in task thread:",e)
                else:
                    self.mainWindow.updateStatus("No task")
            time.sleep(0.0001)


class MeshCatThread(Thread):

    def __init__(self, viz,robot,taskThread):
        Thread.__init__(self)
        self.last_viz=time.time()
        self.viz=viz
        self.robot=robot
        self.shouldStop=False
        self.taskThread=taskThread
        self.additionnalVis=[]
        self.toRemove=[]
        self.toLeaving=[]
    def stop(self):
        self.shouldStop=True
    def addVis(self,vis):
        print("meshcat: add vis ",vis)
        if vis not in self.additionnalVis:
            self.additionnalVis.append(vis)
    def removeVis(self,vis):
        self.toRemove.append(vis)
    def clearVis(self): # TODO: should be merge with removeVis
        for v in self.additionnalVis:
            self.toRemove.append(v)
    def taskLeaving(self,task):  # TODO: Task and Vis should be merged
        self.toLeaving.append(task)

    def run(self):
        ncalls=0
        while self.shouldStop==False:
            if time.time()-self.last_viz>0.02:
                ncalls+=1
                #print("meshcat:",ncalls," in last call was ",time.time()-self.last_viz," ago")
                #print("maeshcat waiting for task lock")
                #self.taskThread.lock.acquire()
                try:
                    if self.taskThread.currentTask!=None:
                        # ask the task to update the visualization
                        self.taskThread.currentTask.onVis(self.viz)
                    for v in self.toRemove:
                        if v in self.additionnalVis:
                            v.clearVis()
                            self.additionnalVis.remove(v)
                    for v in self.toLeaving:
                        v.leaving()
                    self.toLeaving=[]
                    self.toRemove=[]
                    for v in self.additionnalVis:
                        v.onVis(self.viz)
    #                self.taskThread.lock.release()
                    #print("maeshcat waiting for robot lock")
                    robotLock.acquire()
                    self.viz.display(self.robot.state.q)
                    robot_frame_viz(robot, "base")
                    robot_frame_viz(robot, "leg_1")
                    robot_frame_viz(robot, "leg_2")
                    robot_frame_viz(robot, "leg_3")
                    robot_frame_viz(robot, "leg_4")
                    robotLock.release()
                    self.last_viz=time.time()
                except:
                    print("Exception raised in meshcat thread : ",sys.exc_info()[0])
                    pass
            time.sleep(0.001)
        print("meshcat thread stopping...")
        #self.viz.viewer.window.server_proc.kill()
        print("meshcat thread stopped")


class MainWindow(QMainWindow):
    def __init__(self, viz,robot,*args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.taskThread=TaskThread(self)
        self.taskThread.start()
        self.taskThread.setCurrentTask(UpdateRobotAccordingToReal())
        self.meshcat=MeshCatThread(viz,robot, self.taskThread)
        self.meshcat.start()
        self.currentStatusMsg=""
        #self.resize(1400,1000)
        self.browser = QWebEngineView()
        self.browser.load(QUrl(self.meshcat.viz.viewer.url()))
        #self.browser.page().
        demoLeftColumn=QVBoxLayout()
        leftColumn=QVBoxLayout()
        b=QPushButton("close")
        f=b.font()
        f.setPointSize(20)
        b.setFont(f)
        b.clicked.connect(self.close)
        leftColumn.addWidget(b)

        b=QPushButton("update",font=f)
        b.clicked.connect(lambda: self.taskThread.setCurrentTask(UpdateRobotAccordingToReal()))
        leftColumn.addWidget(b)
        b=QPushButton("support (32cm)",font=f)
        b.clicked.connect(lambda: self.taskThread.setCurrentTask(MoveActuatorsStaticPositions([[0.1,0.18,0.1]]*4)))
        leftColumn.addWidget(b)
        b=QPushButton("leave support (46cm)",font=f)
        b.clicked.connect(lambda: self.taskThread.setCurrentTask(MoveActuatorsStaticPositions([[0.1,0.067,0.1]]*4)))
        leftColumn.addWidget(b)
        b=QPushButton("setup danse (40cm)",font=f)
        b.clicked.connect(lambda: self.taskThread.setCurrentTask(MoveActuatorsStaticPositions([[0.1,0.125,0.095]]*4)))
        leftColumn.addWidget(b)
        b=QPushButton("set on ground",font=f)
        b.clicked.connect(lambda: self.taskThread.setCurrentTask(GroundRobotTask()))
        leftColumn.addWidget(b)

        b=QPushButton("Rotation mapping",font=f)
        b.clicked.connect(lambda: self.taskThread.setCurrentTask(TestRotation()))
        leftColumn.addWidget(b)

        b=QPushButton("Position mapping",font=f)
        b.clicked.connect(lambda: self.taskThread.setCurrentTask(TestPosition()))
        leftColumn.addWidget(b)

        b=QPushButton("vis leg accessibility",font=f)
        b.setCheckable(True)
        self.visLegAccessibility=VisLegAccessibility()
        b.clicked.connect(self.toggleLegsVisibility)
        self.visLegAccessibilityButton=b
        leftColumn.addWidget(b)
        leftColumn.setAlignment(PyQt6.QtCore.Qt.AlignmentFlag.AlignTop)
        directionGroupLayout = QHBoxLayout()

        x=QWidget()
        x.setLayout(directionGroupLayout)
        leftColumn.addWidget(x)
        b=QPushButton("")
        b.setIcon(QIcon(QPixmap("stop.png")))
        b.setIconSize(QSize(50,50))
        b.clicked.connect(lambda: self.taskThread.setCurrentTask(None))
        leftColumn.addWidget(b)
        b=QPushButton("")
        b.setIcon(QIcon(QPixmap("stop.png")))
        b.setIconSize(QSize(50,50))
        b.clicked.connect(lambda: self.taskThread.setCurrentTask(None))

        demoLeftColumn.addWidget(b)
        #leftColumn.setSizeConstraint(QHBoxLayout.SizeConstraint.SetFixedSize)

        manualControlLayout = QVBoxLayout()
        self.actuatorsSliders=[]
        for l in range(4):
            legSliders=[]
            for c in range(3):
                bl=QVBoxLayout()
                label=QLabel(f"leg {l+1} / {c+1}")
                bl.addWidget(label)
                sb = QSlider()
                sb.setOrientation(PyQt6.QtCore.Qt.Orientation.Horizontal)
                sb.setRange(0,200)
                sb.setSingleStep(1)
                sb.setValue(int(1000*robot.get_joint(f"l{l+1}_c{c+1}")))
                def onChange(lb,v,l=l,c=c):
                    lb.setText(f"leg {l+1} / {c+1} : {float(v)/1000.0}")
                    print(f"leg {l+1} / {c+1} changed to {v} => {float(v)/1000.0}")
                    com.send_move(l+1,c+1,float(v)/1000.0)
                    #self.taskThread.setCurrentTask(UpdateRobotAccordingToReal())
                sb.valueChanged.connect(lambda v,l=l,c=c,lb=label: onChange(lb,v,l,c))
                legSliders.append((sb,label))
                bl.addWidget(sb)
                manualControlLayout.addLayout(bl)
            self.actuatorsSliders.append(legSliders)
        manualControl=QWidget()
        manualControl.setLayout(manualControlLayout)

        leftColumnTab = QTabWidget()
        w=QWidget()
        w.setLayout(leftColumn)
        leftColumnTab.addTab(w,"Actions")
        leftColumnTab.addTab(manualControl,"Manual control")

        d=QWidget()
        d.setLayout(demoLeftColumn)
        leftColumnTab.addTab(d,"Demo")

        hbox = QHBoxLayout()
        hbox.addWidget(leftColumnTab,stretch=0)
        hbox.addWidget(self.browser,stretch=1)


        buttonAndRobot=QVBoxLayout()
        buttonAndRobot.addLayout(hbox,stretch=4)
        plotsLayout=QHBoxLayout()

        self.plotData={}
        whitePen = pg.mkPen(color=(255, 255, 255))
        redPen = pg.mkPen(color=(255, 0, 0))
        bluePen = pg.mkPen(color=(0, 0, 255))
        for l in range(4):
            for a in range(3):
                # for each actuator, we plot position (white), target (blue) and pwm (red)
                plot=pg.PlotWidget()
                plot.setBackground('#000000')
                positions=plot.plot([], [],pen=whitePen)
                targets=plot.plot([], [],pen=bluePen)
                pwm=plot.plot([], [],pen=redPen)
                plot.setTitle(f"leg {l+1}/{a+1}")
                plot.hideAxis('bottom')
                plot.hideAxis('left')
                plot.setYRange(0,1)
                self.plotData[(l,a)]={"positionsPlot":positions,"targetsPlot":targets,"pwmPlot":pwm,"plot":plot,
                                      "positions":[],"targets":[],"pwm":[],"time":[]}
                plotsLayout.addWidget(plot)
        self.plotTimer=QTimer()
        self.plotTimer.setInterval(500)
        self.plotTimer.timeout.connect(self.updatePlots)
        self.plotTimer.start()
        buttonAndRobot.addLayout(plotsLayout,stretch=1)



        mainWidget = QWidget()
        mainWidget.setLayout(buttonAndRobot)
        self.setCentralWidget(mainWidget)

        self.statusBarW = QStatusBar()
        self.statusBarW.showMessage("js: "+("connected" if joystick.is_available() else "missing") + " | com: "+"fake mode" if com.fakeMode else "ok")
        self.setStatusBar(self.statusBarW)
        self.updateStatus("init")

        self.checkComTimer=QTimer()
        self.checkComTimer.setInterval(500)
        self.checkComTimer.timeout.connect(self.checkCom)
        self.checkComTimer.start()
        ### Add a startup dialog:
        #self.qdialog = QDialog(self)
        #self.qdialog.setWindowTitle("startup")
        #self.qdialog.exec()
    def checkCom(self):
        error=False
        for l in com.legs:
            for a in l:
                if (a['status'] & 0x8) == 0x8 or (a['status'] & 0x80)==0x80:
                    error=True
        if error:
            self.taskThread.setPause(True)
            self.updateStatus("*** ERROR: ACTUATOR MISSING ***")
        else:
            self.taskThread.setPause(False)
    def updatePlots(self):
        for l in range(4):
            for a in range(3):
                d=self.plotData[(l,a)]
                d["positions"].append(com.legs[l][a]['position']*5)
                d["targets"].append(com.legs[l][a]['target']*5)
                d["pwm"].append((com.legs[l][a]['pwm']+1.0)/2.0)
                d["time"].append(time.time())
                d["positions"]=d["positions"][-40:]
                d["targets"]=d["targets"][-40:]
                d["pwm"]=d["pwm"][-40:]
                d["time"]=d["time"][-40:]
                d["positionsPlot"].setData(d["time"],d["positions"])
                d["targetsPlot"].setData(d["time"],d["targets"])
                d["pwmPlot"].setData(d["time"],d["pwm"])
                if (com.legs[l][a]['status'] & 0x8) == 0x8 or (com.legs[l][a]['status'] & 0x80)==0x80:
                    print("actuator status error : ",com.legs[l][a])
                    d["plot"].setBackground((255,0,0))
                else:
                    d["plot"].setBackground((0,0,0))
                self.actuatorsSliders[l][a][0].blockSignals(True)
                self.actuatorsSliders[l][a][0].setValue(int(1000*com.legs[l][a]['position']))
                self.actuatorsSliders[l][a][0].blockSignals(False)
                self.actuatorsSliders[l][a][1].setText(f"leg {l+1} / {a+1} : {com.legs[l][a]['position']}")

    def updateStatus(self,message):
        try:
            msg="js: "+ ("connected" if joystick.is_available() else "missing") + " | com: "+ ("fake mode" if com.fakeMode else "ok")
            msg+= ' | Height: '+str(int(robot.get_T_world_frame("base")[Z,3]*100))+'cm  '
            msg+='| 1:(%.1f;%.1f)' % (robot.get_T_world_frame("leg_1")[X,3],robot.get_T_world_frame("leg_1")[Y,3])
            msg+=' 2:(%.1f;%.1f)' % (robot.get_T_world_frame("leg_2")[X,3],robot.get_T_world_frame("leg_2")[Y,3])
            msg+=' 3:(%.1f;%.1f)' % (robot.get_T_world_frame("leg_3")[X,3],robot.get_T_world_frame("leg_3")[Y,3])
            msg+=' 4:(%.1f;%.1f)' % (robot.get_T_world_frame("leg_4")[X,3],robot.get_T_world_frame("leg_4")[Y,3])
            msg+= ' | ' + message
            #print("height:",robot.get_T_world_frame("base")[Z,3],)
            if msg!=self.currentStatusMsg:
                self.currentStatusMsg=msg
                #self.statusBar.clearMessage()
                self.statusBar().showMessage(msg)
        except Exception as e:
            print("status bar error : ",e)

    def toggleLegsVisibility(self):
        if self.visLegAccessibilityButton.isChecked():
            self.meshcat.addVis(self.visLegAccessibility)
        else:
            self.meshcat.removeVis(self.visLegAccessibility)
    def closeEvent(self, *args, **kwargs):
        print("close event...")
        self.meshcat.stop()
        self.meshcat.join()
        self.taskThread.stop()
        self.taskThread.join()
        print("mesh thread is killed")
        global com
        com.stop()
        com.join()
        super(QMainWindow, self).closeEvent(*args, **kwargs)



if __name__ == '__main__':
    #QApplication.setHighDpiScaleFactorRoundingPolicy(PyQt6.QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QApplication(sys.argv)
    mainWindow = MainWindow(viz,robot)

    import signal
    def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        mainWindow.meshcat.stop()
        mainWindow.meshcat.join()
        mainWindow.taskThread.stop()
        mainWindow.taskThread.join()
        com.stop()
        com.join()
        mainWindow.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    if len(sys.argv)>1 and sys.argv[1]=='-F':
        mainWindow.showFullScreen()
    else:
        mainWindow.show()
    app.exec()
