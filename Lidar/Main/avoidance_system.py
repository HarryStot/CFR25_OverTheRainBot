import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import threading


class PotentialFieldNavigation:
    def __init__(self):
        # Paramètres de configuration
        self.K_att = 7  # Coefficient d'attraction vers l'objectif
        self.K_rep = 10  # Coefficient de répulsion des obstacles
        self.K_v = 25  # Gain de vitesse
        self.K_w = 30  # Gain angulaire
        self.K_theta = 8  # Gain de correction d'orientation

        self.d_safe = 1.0  # Distance de sécurité pour les obstacles
        self.d_eps = 0.05  # Tolérance de distance pour atteindre l'objectif

        # Limites de la visualisation
        self.xmin, self.xmax = 0, 10
        self.ymin, self.ymax = 0, 10

        # Paramètres de simulation
        self.dt = 0.1
        self.visualize = False
        self.fig = None
        self.ax = None
        self.visualization_lock = threading.Lock()

    def init_visualization(self):
        """Initialise la fenêtre de visualisation"""
        if self.visualize:
            self.fig, self.ax = plt.subplots()
            self.ax.set_aspect('equal')
            self.ax.set_xlim(self.xmin, self.xmax)
            self.ax.set_ylim(self.ymin, self.ymax)
            plt.ion()  # Mode interactif

    def close_visualization(self):
        """Ferme la fenêtre de visualisation"""
        if self.visualize and plt.fignum_exists(self.fig.number):
            plt.close(self.fig)

    def compute_control(self, robot_pos, robot_heading, target_pos, obstacles):
        """
        Calcule les commandes de contrôle basées sur le champ potentiel

        Args :
            robot_pos (tuple) : Position actuelle du robot (x, y)
            robot_heading (float) : Orientation du robot en radians
            target_pos (tuple) : Position cible (x, y)
            obstacles (list) : Liste des positions des obstacles [(x1, y1), (x2, y2), ...]

        Returns :
            tuple : (vitesse linéaire, vitesse angulaire)
        """
        # Conversion en numpy arrays
        robot_pos = np.array(robot_pos)
        target_pos = np.array(target_pos)

        # Distance au but
        distance_goal = np.linalg.norm(robot_pos - target_pos)

        # Force d'attraction vers le but
        F_att = -self.K_att * (robot_pos - target_pos) / max(distance_goal, 1e-3)

        # Force de répulsion des obstacles
        F_rep = np.array([0.0, 0.0])
        for obs_pos in obstacles:
            obs_pos = np.array(obs_pos)
            nq = robot_pos - obs_pos
            norm_nq = max(np.linalg.norm(nq), 1e-3)
            if norm_nq < self.d_safe:
                F_rep += self.K_rep * nq / (norm_nq ** 2)
            else:
                F_rep += self.K_rep * nq / norm_nq ** 3

        # Force résultante
        w = F_att + F_rep

        # Calcul des commandes de vitesse et d'orientation
        vbar = np.linalg.norm(w)
        thetabar = np.arctan2(w[1], w[0])

        # Calcul de l'erreur d'orientation
        delta_theta = (thetabar - robot_heading + np.pi) % (2 * np.pi) - np.pi
        erreur_theta = np.clip(delta_theta ** 2, 0, 5)

        # Commandes de contrôle
        v = self.K_v * (vbar - 0)  # Pas de prise en compte de la vitesse actuelle ici
        omega = 2 * self.K_w * np.arctan(np.tan(0.5 * delta_theta)) + self.K_theta * erreur_theta

        # Visualisation si activée
        if self.visualize and self.ax:
            with self.visualization_lock:
                self._visualize(robot_pos, robot_heading, target_pos, obstacles, w)

        return v, omega

    def _visualize(self, robot_pos, robot_heading, target_pos, obstacles, force_vector):
        """Visualise l'état actuel de la navigation"""
        self.ax.clear()
        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)

        # Dessiner les obstacles
        for obs_pos in obstacles:
            self._draw_disk(obs_pos, 0.3, "magenta")

        # Dessiner l'objectif
        self._draw_disk(target_pos, 0.2, "green")

        # Dessiner le robot
        self._draw_robot(robot_pos, robot_heading)

        # Dessiner le vecteur de force
        if np.linalg.norm(force_vector) > 0:
            self.ax.arrow(robot_pos[0], robot_pos[1],
                          force_vector[0] / 5, force_vector[1] / 5,
                          head_width=0.2, head_length=0.3, fc='blue', ec='blue')

        plt.pause(0.001)

    def _draw_disk(self, center, radius, color, alpha=0.7):
        """Dessine un disque"""
        e = Ellipse(xy=center, width=2 * radius, height=2 * radius, angle=0)
        self.ax.add_patch(e)
        e.set_alpha(alpha)
        e.set_facecolor(color)

    def _draw_robot(self, pos, theta, size=0.5, color='red'):
        """Dessine une représentation simple du robot"""
        # Rectangle représentant le corps du robot
        corners = np.array([
            [-size, -size / 2],
            [size, -size / 2],
            [size, size / 2],
            [-size, size / 2]
        ])

        # Rotation
        rot_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])

        rotated = np.dot(corners, rot_matrix.T)

        # Translation
        rotated[:, 0] += pos[0]
        rotated[:, 1] += pos[1]

        # Dessiner le rectangle
        self.ax.fill(rotated[:, 0], rotated[:, 1], color=color, alpha=0.7)

        # Dessiner une flèche pour indiquer l'orientation
        arrow_start = pos
        arrow_end = pos + 0.8 * size * np.array([np.cos(theta), np.sin(theta)])
        self.ax.arrow(arrow_start[0], arrow_start[1],
                      arrow_end[0] - arrow_start[0], arrow_end[1] - arrow_start[1],
                      head_width=0.1, head_length=0.1, fc='black', ec='black')