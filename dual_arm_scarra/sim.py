"""
Dual-Arm SCARA Robot Simulation
================================
Simulates a 5-bar parallel SCARA mechanism with:
  - Distance between two driver joints: 7.5 cm
  - Upper arm length: 5 cm
  - Lower arm length: 12.5 cm

Architecture
------------
  1. kinematics.py  - Forward & Inverse Kinematics
  2. controller.py  - Trajectory planning & joint interpolation
  3. display.py     - Matplotlib visualization with click interaction
  4. main()         - Entry point wiring everything together
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from matplotlib.patches import FancyArrowPatch, Circle, FancyBboxPatch
from matplotlib.lines import Line2D
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import warnings
warnings.filterwarnings("ignore")

# ─────────────────────────────────────────────
#  SECTION 1 ─ ROBOT CONSTANTS
# ─────────────────────────────────────────────

@dataclass(frozen=True)
class SCARAParams:
    """Physical parameters of the 5-bar SCARA mechanism (all in cm)."""
    d:  float = 7.5    # distance between the two fixed base joints (O1 and O2)
    l1: float = 5.0    # upper arm (proximal) length – same for both arms
    l2: float = 12.5   # lower arm (distal)  length – same for both arms

    # Base joint positions (O1 on left, O2 on right, centred at origin)
    @property
    def O1(self) -> np.ndarray:
        return np.array([-self.d / 2, 0.0])

    @property
    def O2(self) -> np.ndarray:
        return np.array([ self.d / 2, 0.0])

PARAMS = SCARAParams()


# ─────────────────────────────────────────────
#  SECTION 2 ─ INVERSE KINEMATICS
# ─────────────────────────────────────────────

class InverseKinematics:
    """
    5-bar parallel mechanism inverse kinematics.

    Joint angles θ1 (left arm) and θ2 (right arm) are solved so that
    both distal links meet at end-effector point P.

    For each arm the elbow can point "up" or "down"; we choose the
    configuration that keeps the arms above the base line (elbow-up).
    """

    def __init__(self, params: SCARAParams = PARAMS):
        self.p = params

    def solve(self, target: np.ndarray, elbow_sign: int = 1
              ) -> Optional[Tuple[float, float]]:
        """
        Compute (θ1, θ2) in radians for a given end-effector position.

        Parameters
        ----------
        target      : (x, y) end-effector position in cm
        elbow_sign  : +1 → elbow-up (preferred), -1 → elbow-down

        Returns
        -------
        (θ1, θ2) tuple or None if target is unreachable
        """
        px, py = target
        O1, O2 = self.p.O1, self.p.O2
        l1, l2 = self.p.l1, self.p.l2

        theta1 = self._solve_single_arm(O1, px, py, l1, l2,  elbow_sign)
        theta2 = self._solve_single_arm(O2, px, py, l1, l2, -elbow_sign)

        if theta1 is None or theta2 is None:
            return None
        return theta1, theta2

    @staticmethod
    def _solve_single_arm(origin: np.ndarray,
                          px: float, py: float,
                          l1: float, l2: float,
                          elbow_sign: int) -> Optional[float]:
        """
        2-DOF serial arm IK from 'origin' to point (px, py).
        Returns shoulder angle θ or None when out of reach.
        """
        dx, dy = px - origin[0], py - origin[1]
        dist2 = dx**2 + dy**2
        dist  = np.sqrt(dist2)

        # Reachability check
        if dist < abs(l1 - l2) + 1e-9 or dist > l1 + l2 - 1e-9:
            return None

        # Law of cosines: angle at the shoulder
        cos_alpha = (dist2 + l1**2 - l2**2) / (2.0 * dist * l1)
        cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
        alpha = np.arccos(cos_alpha)

        # Angle from origin to target
        phi = np.arctan2(dy, dx)

        theta = phi + elbow_sign * alpha
        return theta

    def forward_kinematics(self, theta1: float, theta2: float
                           ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Given joint angles, return the four link endpoints:
            elbow1 (E1), elbow2 (E2), and end-effector (P).
        Also returns P computed from both arms (should agree).

        Returns (E1, E2, P)
        """
        p = self.p
        O1, O2 = p.O1, p.O2

        E1 = O1 + p.l1 * np.array([np.cos(theta1), np.sin(theta1)])
        E2 = O2 + p.l1 * np.array([np.cos(theta2), np.sin(theta2)])

        # End-effector: intersection of two circles centred at E1, E2 with radius l2
        P = self._distal_intersection(E1, E2, p.l2)
        return E1, E2, P

    @staticmethod
    def _distal_intersection(E1: np.ndarray, E2: np.ndarray,
                             l2: float) -> Optional[np.ndarray]:
        """Find the upper intersection of two circles of radius l2."""
        d = np.linalg.norm(E2 - E1)
        if d < 1e-9 or d > 2 * l2:
            return None
        a = d / 2
        h = np.sqrt(max(l2**2 - a**2, 0.0))
        mid = (E1 + E2) / 2
        perp = np.array([-(E2[1] - E1[1]), E2[0] - E1[0]]) / d
        # Choose the intersection point that is above the base (positive y side)
        P_up   = mid + h * perp
        P_down = mid - h * perp
        return P_up if P_up[1] >= P_down[1] else P_down


# ─────────────────────────────────────────────
#  SECTION 3 ─ TRAJECTORY CONTROLLER
# ─────────────────────────────────────────────

class TrajectoryController:
    """
    Generates smooth joint-space trajectories between two configurations
    using a fifth-order polynomial (zero velocity & acceleration at endpoints).
    """

    def __init__(self, params: SCARAParams = PARAMS,
                 fps: int = 60, move_time: float = 0.8):
        self.ik  = InverseKinematics(params)
        self.fps = fps
        self.T   = move_time           # seconds per move

    def plan(self, theta_start: Tuple[float, float],
             theta_goal:  Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Return a list of (θ1, θ2) waypoints along a quintic polynomial path.
        """
        n_steps = max(int(self.T * self.fps), 2)
        waypoints = []
        for k in range(n_steps + 1):
            s = k / n_steps
            # Quintic ease: 6s^5 - 15s^4 + 10s^3
            blend = 6*s**5 - 15*s**4 + 10*s**3
            t1 = theta_start[0] + blend * (theta_goal[0] - theta_start[0])
            t2 = theta_start[1] + blend * (theta_goal[1] - theta_start[1])
            waypoints.append((t1, t2))
        return waypoints

    def angles_for_target(self, target: np.ndarray
                          ) -> Optional[Tuple[float, float]]:
        return self.ik.solve(target)


# ─────────────────────────────────────────────
#  SECTION 4 ─ ROBOT STATE
# ─────────────────────────────────────────────

@dataclass
class RobotState:
    """Mutable live state of the robot."""
    theta1: float = np.radians(60)
    theta2: float = np.radians(120)
    trajectory: List[Tuple[float, float]] = field(default_factory=list)
    traj_index: int = 0
    animating: bool = False
    end_effector: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    target_history: List[np.ndarray] = field(default_factory=list)


# ─────────────────────────────────────────────
#  SECTION 5 ─ DISPLAY / VISUALISATION
# ─────────────────────────────────────────────

class SCARADisplay:
    """
    Matplotlib-based real-time display with click-to-move interaction.

    Layout
    ------
    - Dark industrial colour scheme with neon accent highlights
    - Robot arms drawn as thick rounded links
    - Workspace boundary shown as a shaded reachable region
    - Click anywhere in the workspace to command a smooth move
    - Status bar shows joint angles and end-effector position
    """

    # ── Colour palette ──────────────────────────────────────────────
    BG          = "#0d0f14"
    GRID        = "#1c2030"
    AXIS        = "#2a3050"
    LINK_L      = "#00c8ff"   # left arm – cyan
    LINK_R      = "#ff6b35"   # right arm – orange
    EE_COL      = "#f0e130"   # end-effector – yellow
    JOINT_COL   = "#ffffff"
    TARGET_COL  = "#44ff88"
    SHADOW      = "#0a0c10"
    TEXT_COL    = "#c8d0e8"

    def __init__(self, params: SCARAParams = PARAMS):
        self.p      = params
        self.ik     = InverseKinematics(params)
        self.ctrl   = TrajectoryController(params)
        self.state  = RobotState()

        # Compute initial FK
        self._update_fk()

        self._build_figure()
        self._draw_static()
        self._init_artists()
        self._draw_frame()

        # Connect events
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self._anim = animation.FuncAnimation(
            self.fig, self._animation_step,
            interval=1000 // self.ctrl.fps,
            blit=False, cache_frame_data=False
        )

    # ── Figure setup ────────────────────────────────────────────────

    def _build_figure(self):
        plt.rcParams.update({
            "figure.facecolor": self.BG,
            "axes.facecolor":   self.BG,
            "text.color":       self.TEXT_COL,
            "font.family":      "monospace",
        })
        self.fig = plt.figure(figsize=(10, 9), facecolor=self.BG)
        self.fig.canvas.manager.set_window_title("Dual-Arm SCARA Simulation")

        # Main robot axes
        self.ax = self.fig.add_axes([0.07, 0.13, 0.86, 0.82])
        self.ax.set_facecolor(self.BG)
        lim = self.p.l1 + self.p.l2 + 1.0
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim * 0.4, lim * 1.1)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("X  (cm)", color=self.TEXT_COL, fontsize=9)
        self.ax.set_ylabel("Y  (cm)", color=self.TEXT_COL, fontsize=9)
        self.ax.tick_params(colors=self.TEXT_COL, labelsize=8)
        for spine in self.ax.spines.values():
            spine.set_edgecolor(self.AXIS)

        # Status bar axes
        self.ax_status = self.fig.add_axes([0.07, 0.01, 0.86, 0.09])
        self.ax_status.set_facecolor("#111520")
        self.ax_status.axis("off")

    def _draw_static(self):
        ax = self.ax
        p  = self.p

        # Grid
        ax.grid(True, color=self.GRID, linewidth=0.5, linestyle="--", alpha=0.6)
        ax.axhline(0, color=self.AXIS, linewidth=0.8)
        ax.axvline(0, color=self.AXIS, linewidth=0.8)

        # Workspace boundary – annulus swept by end-effector (approximate)
        reach_max = p.l1 + p.l2
        reach_min = abs(p.l2 - p.l1)
        theta_arc = np.linspace(0, np.pi, 300)
        for sign, label in [(1, "Max reach"), (-1, "")]:
            r = reach_max if sign == 1 else reach_min
            xs = [o[0] + r * np.cos(a) for o in [p.O1, p.O2] for a in theta_arc]
            # Shaded reachable region (very subtle)
        ws_outer = plt.Circle((0, (p.O1[1]+p.O2[1])/2 + 0.5),
                               reach_max * 0.95, color="#1a2540",
                               alpha=0.25, linewidth=0, zorder=0)
        ax.add_patch(ws_outer)

        # Base platform rectangle
        bw, bh = p.d + 1.8, 0.7
        base = FancyBboxPatch((-bw/2, -bh), bw, bh,
                               boxstyle="round,pad=0.15",
                               facecolor="#1a2030", edgecolor=self.AXIS,
                               linewidth=1.5, zorder=2)
        ax.add_patch(base)
        ax.text(0, -bh/2, "BASE", ha="center", va="center",
                fontsize=7, color=self.AXIS, fontweight="bold")

        # Dimension labels
        ax.annotate("", xy=p.O2, xytext=p.O1,
                    arrowprops=dict(arrowstyle="<->", color=self.AXIS, lw=1.2))
        ax.text(0, -0.9, f"d = {p.d} cm", ha="center", va="top",
                fontsize=7.5, color=self.AXIS)

        self.ax.set_title("Dual-Arm SCARA  —  click to move",
                          color=self.TEXT_COL, fontsize=11, pad=8,
                          fontweight="bold", fontfamily="monospace")

    def _init_artists(self):
        """Create all mutable matplotlib artists (lines, circles, text)."""
        ax = self.ax
        lw_upper = 5
        lw_lower = 4
        joint_r  = 0.35

        # ── Left arm ──────────────────────────────────────────────
        self.line_l1,  = ax.plot([], [], color=self.LINK_L, lw=lw_upper,
                                  solid_capstyle="round", zorder=5)
        self.shadow_l1,= ax.plot([], [], color=self.SHADOW, lw=lw_upper+3,
                                  solid_capstyle="round", zorder=4)
        self.line_l2,  = ax.plot([], [], color=self.LINK_L, lw=lw_lower,
                                  solid_capstyle="round", zorder=5, alpha=0.75)
        self.shadow_l2,= ax.plot([], [], color=self.SHADOW, lw=lw_lower+3,
                                  solid_capstyle="round", zorder=4)

        # ── Right arm ─────────────────────────────────────────────
        self.line_r1,  = ax.plot([], [], color=self.LINK_R, lw=lw_upper,
                                  solid_capstyle="round", zorder=5)
        self.shadow_r1,= ax.plot([], [], color=self.SHADOW, lw=lw_upper+3,
                                  solid_capstyle="round", zorder=4)
        self.line_r2,  = ax.plot([], [], color=self.LINK_R, lw=lw_lower,
                                  solid_capstyle="round", zorder=5, alpha=0.75)
        self.shadow_r2,= ax.plot([], [], color=self.SHADOW, lw=lw_lower+3,
                                  solid_capstyle="round", zorder=4)

        # ── Joints ────────────────────────────────────────────────
        self.joint_O1 = Circle(self.p.O1, joint_r, color=self.LINK_L,
                                zorder=8, lw=1.5, ec="white")
        self.joint_O2 = Circle(self.p.O2, joint_r, color=self.LINK_R,
                                zorder=8, lw=1.5, ec="white")
        self.joint_E1 = Circle((0,0), joint_r*0.85, color=self.LINK_L,
                                zorder=8, lw=1.5, ec="white", alpha=0.9)
        self.joint_E2 = Circle((0,0), joint_r*0.85, color=self.LINK_R,
                                zorder=8, lw=1.5, ec="white", alpha=0.9)
        self.joint_EE = Circle((0,0), joint_r*1.2, color=self.EE_COL,
                                zorder=9, lw=2, ec="white")
        # Inner dot
        self.joint_EE_inner = Circle((0,0), joint_r*0.5, color="white",
                                      zorder=10)
        for jt in [self.joint_O1, self.joint_O2, self.joint_E1,
                   self.joint_E2, self.joint_EE, self.joint_EE_inner]:
            ax.add_patch(jt)

        # ── End-effector crosshair ────────────────────────────────
        self.ee_h, = ax.plot([], [], color=self.EE_COL, lw=1, alpha=0.6,
                              zorder=6)
        self.ee_v, = ax.plot([], [], color=self.EE_COL, lw=1, alpha=0.6,
                              zorder=6)

        # ── Target marker ─────────────────────────────────────────
        self.target_dot, = ax.plot([], [], marker="+", ms=12,
                                    color=self.TARGET_COL, zorder=11,
                                    mew=2, linestyle="none")
        self.target_ring = Circle((0,0), 0.5, fill=False,
                                   ec=self.TARGET_COL, lw=1.2,
                                   zorder=10, alpha=0.6)
        ax.add_patch(self.target_ring)

        # ── Trail ────────────────────────────────────────────────
        self.trail_xs: List[float] = []
        self.trail_ys: List[float] = []
        self.trail_line, = ax.plot([], [], color=self.EE_COL,
                                   lw=0.8, alpha=0.35, zorder=3)

        # ── Status text ───────────────────────────────────────────
        ax_s = self.ax_status
        self.txt_theta = ax_s.text(0.01, 0.6, "", fontsize=9,
                                    color=self.LINK_L, va="center",
                                    transform=ax_s.transAxes)
        self.txt_ee    = ax_s.text(0.01, 0.2, "", fontsize=9,
                                    color=self.EE_COL, va="center",
                                    transform=ax_s.transAxes)
        self.txt_state = ax_s.text(0.75, 0.5, "IDLE", fontsize=10,
                                    color=self.TARGET_COL, va="center",
                                    ha="center", fontweight="bold",
                                    transform=ax_s.transAxes)
        # Legend
        legend_elements = [
            Line2D([0],[0], color=self.LINK_L, lw=3, label="Left arm"),
            Line2D([0],[0], color=self.LINK_R, lw=3, label="Right arm"),
            Line2D([0],[0], marker="o", color=self.EE_COL, lw=0,
                   ms=7, label="End-effector"),
        ]
        self.ax.legend(handles=legend_elements, loc="lower right",
                       facecolor="#111520", edgecolor=self.AXIS,
                       labelcolor=self.TEXT_COL, fontsize=8)

    # ── Drawing helpers ─────────────────────────────────────────────

    def _update_fk(self):
        """Run FK and store results in state."""
        E1, E2, P = self.ik.forward_kinematics(self.state.theta1,
                                                 self.state.theta2)
        self.E1 = E1
        self.E2 = E2
        self.P  = P if P is not None else np.array([0.0, 4.0])
        self.state.end_effector = self.P

    def _draw_frame(self):
        """Update all mutable artists to reflect current state."""
        O1, O2 = self.p.O1, self.p.O2
        E1, E2, P = self.E1, self.E2, self.P

        def _set(line, shadow, x0, y0, x1, y1):
            line.set_data([x0, x1], [y0, y1])
            shadow.set_data([x0, x1], [y0, y1])

        _set(self.shadow_l1, self.line_l1, O1[0], O1[1], E1[0], E1[1])
        _set(self.shadow_l2, self.line_l2, E1[0], E1[1], P[0],  P[1])
        _set(self.shadow_r1, self.line_r1, O2[0], O2[1], E2[0], E2[1])
        _set(self.shadow_r2, self.line_r2, E2[0], E2[1], P[0],  P[1])

        self.joint_E1.center = E1
        self.joint_E2.center = E2
        self.joint_EE.center = P
        self.joint_EE_inner.center = P

        ch = 0.8
        self.ee_h.set_data([P[0]-ch, P[0]+ch], [P[1], P[1]])
        self.ee_v.set_data([P[0], P[0]], [P[1]-ch, P[1]+ch])

        # Trail
        self.trail_xs.append(P[0])
        self.trail_ys.append(P[1])
        if len(self.trail_xs) > 300:
            self.trail_xs = self.trail_xs[-300:]
            self.trail_ys = self.trail_ys[-300:]
        self.trail_line.set_data(self.trail_xs, self.trail_ys)

        # Status
        t1d = np.degrees(self.state.theta1)
        t2d = np.degrees(self.state.theta2)
        self.txt_theta.set_text(
            f"θ₁ = {t1d:+7.2f}°   θ₂ = {t2d:+7.2f}°"
            f"   |  l₁ = {self.p.l1} cm   l₂ = {self.p.l2} cm")
        self.txt_ee.set_text(
            f"End-effector:  X = {P[0]:+7.3f} cm   Y = {P[1]:+7.3f} cm")
        state_str = "MOVING" if self.state.animating else "IDLE"
        self.txt_state.set_text(state_str)
        self.txt_state.set_color(self.LINK_L if self.state.animating
                                  else self.TARGET_COL)

        self.fig.canvas.draw_idle()

    # ── Animation step ──────────────────────────────────────────────

    def _animation_step(self, frame):
        if not self.state.animating:
            return
        s = self.state
        if s.traj_index < len(s.trajectory):
            s.theta1, s.theta2 = s.trajectory[s.traj_index]
            s.traj_index += 1
            self._update_fk()
            self._draw_frame()
        else:
            s.animating = False
            self._draw_frame()

    # ── Click handler ───────────────────────────────────────────────

    def _on_click(self, event):
        if event.inaxes is not self.ax:
            return
        if event.button != 1:
            return
        target = np.array([event.xdata, event.ydata])
        angles = self.ctrl.angles_for_target(target)
        if angles is None:
            # Flash status to indicate out-of-reach
            self.txt_state.set_text("OUT OF REACH")
            self.txt_state.set_color("#ff4444")
            self.fig.canvas.draw_idle()
            return

        # Show target marker
        self.target_dot.set_data([target[0]], [target[1]])
        self.target_ring.center = target
        self.target_ring.set_visible(True)

        # Plan trajectory
        traj = self.ctrl.plan(
            (self.state.theta1, self.state.theta2), angles)
        self.state.trajectory = traj
        self.state.traj_index = 0
        self.state.animating  = True
        self.state.target_history.append(target)

    # ── Public entry ────────────────────────────────────────────────

    def show(self):
        plt.show()


# ─────────────────────────────────────────────
#  SECTION 6 ─ MAIN
# ─────────────────────────────────────────────

def print_banner():
    banner = r"""
  ╔══════════════════════════════════════════════════════╗
  ║        Dual-Arm SCARA Simulation                     ║
  ║  ─────────────────────────────────────────────────   ║
  ║  Base distance  d  = 7.5  cm                         ║
  ║  Upper arm      l1 = 5.0  cm                         ║
  ║  Lower arm      l2 = 12.5 cm                         ║
  ║  ─────────────────────────────────────────────────   ║
  ║  ► Click inside the workspace to move the robot      ║
  ║  ► Quintic polynomial trajectory interpolation       ║
  ║  ► 5-bar parallel mechanism inverse kinematics       ║
  ╚══════════════════════════════════════════════════════╝
    """
    print(banner)


def main():
    print_banner()
    display = SCARADisplay()
    display.show()


if __name__ == "__main__":
    main()