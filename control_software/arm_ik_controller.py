import sys
import time
import math
import logging
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Union

# --- THIRD PARTY IMPORTS ---
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("CRITICAL: 'pyserial' not found. Install via: pip install pyserial")
    sys.exit(1)

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


# --- LOGGING SETUP ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger(__name__)


# --- CONFIGURATION ---
@dataclass
class RobotConfig:
    """Holds physical parameters and constraints of the robot."""
    port: str = "COM3"  # Default, will be overridden by auto-discovery
    baud_rate: int = 250000
    
    # Link Lengths (mm)
    l0_z: float = 65.0
    l1: float = 80.0
    l2: float = 80.0
    
    # Speed / Safety
    command_delay: float = 0.03
    home_angles: List[int] = field(default_factory=lambda: [0, 90, 40, 0])
    
    @property
    def max_reach(self) -> float:
        return self.l1 + self.l2


# --- KINEMATICS ENGINE ---
class KinematicsEngine:
    """Pure mathematical engine for Inverse (IK) and Forward (FK) Kinematics.
    This class is stateless regarding the actual robot position."""
    
    def __init__(self, config: RobotConfig):
        self.cfg = config

    @staticmethod
    def clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))

    def solve_ik(self, x: float, y: float, z: float, grip_angle: float = 90) -> Optional[List[int]]:
        """Calculates servo angles for target coordinates."""
        # --- J1: Base (Yaw) ---
        r = math.sqrt(x**2 + y**2)
        angle_base = 0.0 if r == 0 else math.degrees(math.atan2(y, x))
        servo_base = int(self.clamp(angle_base, 0, 180))

        # --- J2 & J3: Shoulder & Elbow ---
        z_eff = z - self.cfg.l0_z
        d_sq = r**2 + z_eff**2
        d = math.sqrt(d_sq)

        # Reachability Check
        if d > self.cfg.max_reach or d == 0:
            logger.warning(f"Target out of reach: dist={d:.1f}, max={self.cfg.max_reach}")
            return None

        try:
            # Elbow (Law of Cosines)
            cos_alpha_l2 = (self.cfg.l1**2 + self.cfg.l2**2 - d_sq) / (2 * self.cfg.l1 * self.cfg.l2)
            alpha_l2_rad = math.acos(self.clamp(cos_alpha_l2, -1.0, 1.0))
            
            # Map internal angle to servo angle
            angle_elbow_deg = math.degrees(math.pi - alpha_l2_rad)
            servo_elbow = int(self.clamp(angle_elbow_deg, 0, 180))

            # Shoulder
            gamma_rad = math.atan2(z_eff, r)
            cos_alpha_l1 = (d_sq + self.cfg.l1**2 - self.cfg.l2**2) / (2 * d * self.cfg.l1)
            alpha_l1_rad = math.acos(self.clamp(cos_alpha_l1, -1.0, 1.0))
            
            angle_shoulder_deg = math.degrees(gamma_rad + alpha_l1_rad)
            servo_shoulder = int(self.clamp(180.0 - angle_shoulder_deg, 0, 180))

        except ValueError:
            logger.error("Math domain error in IK calculation")
            return None

        # Gripper
        servo_gripper = int(self.clamp(grip_angle, 0, 180))

        return [servo_base, servo_shoulder, servo_elbow, servo_gripper]

    def forward_kinematics(self, angles: List[int]) -> List[List[float]]:
        """Calculates 3D coordinates of all joints based on angles."""
        b_deg, s_deg, e_deg, _ = angles
        
        theta1 = math.radians(b_deg)
        theta2 = -math.radians(s_deg - 180)
        thetaq = -math.radians(e_deg)
        theta3 = math.radians(90) - theta2 - thetaq

        # P0: Base, P1: Shoulder, P2: Elbow, P3: End Effector
        p0 = [0.0, 0.0, 0.0]
        p1 = [0.0, 0.0, self.cfg.l0_z]

        r1 = self.cfg.l1 * math.cos(theta2)
        z1 = self.cfg.l1 * math.sin(theta2)
        p2 = [r1 * math.cos(theta1), r1 * math.sin(theta1), self.cfg.l0_z + z1]

        phi = theta2 + (theta3 - math.pi)
        r2 = self.cfg.l2 * math.cos(phi)
        z2 = self.cfg.l2 * math.sin(phi)
        p3 = [p2[0] + r2 * math.cos(theta1), p2[1] + r2 * math.sin(theta1), p2[2] + z2]

        return [p0, p1, p2, p3]


# --- VISUALIZATION ---
class RobotVisualizer:
    """Handles matplotlib 3D plotting. Fails gracefully if matplotlib is missing."""
    def __init__(self, config: RobotConfig):
        self.cfg = config
        self.enabled = MATPLOTLIB_AVAILABLE
        self.fig = None
        self.ax = None
        
        if self.enabled:
            self.init_plot()

    def init_plot(self):
        plt.ion()
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title("Digital Twin: Kinematic Model")
        self._set_limits()

    def _set_limits(self):
        lim = self.cfg.max_reach + 50
        self.ax.set_xlim([-lim, lim])
        self.ax.set_ylim([-lim, lim])
        self.ax.set_zlim([0, lim])
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')

    def update(self, joint_coords: List[List[float]], target: Optional[Tuple[float, float, float]] = None):
        if not self.enabled: return
        
        self.ax.clear()
        self._set_limits()
        
        # Unpack
        p0, p1, p2, p3 = joint_coords
        xs = [p[0] for p in joint_coords]
        ys = [p[1] for p in joint_coords]
        zs = [p[2] for p in joint_coords]

        # Draw Arm
        self.ax.plot(xs[1:], ys[1:], zs[1:], 'b-', linewidth=4, label='Links')
        self.ax.scatter(xs[1:], ys[1:], zs[1:], c='r', s=80, label='Joints')

        # Draw Target
        if target:
            self.ax.scatter([target[0]], [target[1]], [target[2]], c='k', marker='x', s=100)

        plt.draw()
        plt.pause(0.001)
        
    def close(self):
        if self.enabled and self.fig:
            plt.close(self.fig)


# --- HARDWARE ABSTRACTION LAYER ---
class RobotController:
    """Manages state, serial communication, and trajectory execution."""
    
    def __init__(self, config: RobotConfig):
        self.cfg = config
        self.ser: Optional[serial.Serial] = None
        self.current_angles = list(self.cfg.home_angles)
        self.ik = KinematicsEngine(config)
        self.viz = RobotVisualizer(config)

    def connect(self) -> bool:
        """Handles serial port discovery and connection."""
        port = self.cfg.port
        
        # Auto-discovery if port looks like a placeholder
        if port == "AUTO" or port is None:
            available = list(serial.tools.list_ports.comports())
            if not available:
                logger.error("No serial ports found.")
                return False
            
            print("\n--- Available Ports ---")
            for i, p in enumerate(available):
                print(f"[{i}]: {p.device} - {p.description}")
            
            try:
                idx = int(input("Select Port Index: "))
                port = available[idx].device
            except (ValueError, IndexError):
                logger.error("Invalid selection.")
                return False

        try:
            self.ser = serial.Serial(port, self.cfg.baud_rate, timeout=1.0)
            time.sleep(2) # Allow Arduino reset
            logger.info(f"Connected to {port} @ {self.cfg.baud_rate}")
            return True
        except serial.SerialException as e:
            logger.error(f"Serial connection failed: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.write(b'D\n') # Disable/Park command if supported
            self.ser.close()
            logger.info("Serial port closed.")
        self.viz.close()

    def _send_raw(self, angles: List[int]):
        """Formats and transmits the protocol string."""
        if not self.ser: return
        
        cmd = f"P{angles[0]} {angles[1]} {angles[2]} {angles[3]}\n"
        try:
            self.ser.write(cmd.encode())
            time.sleep(self.cfg.command_delay)
        except serial.SerialException:
            logger.error("Communication lost during transmission.")

    def move_to(self, target_angles: List[int]):
        """Moves to target angles, optionally interpolating for smoothness."""
        
        self._send_raw(target_angles)
        
        # Update Visualization
        coords = self.ik.forward_kinematics(target_angles)
        self.viz.update(coords)
        

    def run_pick_and_place_job(self):
        """Executes the specific Pick and Place routine."""
        logger.info("Starting Pick and Place Routine...")

        # Sequence Definition: (Angles, Description)
        # Note: Negative inputs from user logic are clamped to 0 here.
        sequence = [
            ([0, 90, 90, 20],    "Home / Station 1 Approach"),
            ([90, 90, 90, 0],  "Pre-Grip Position (Open)"),
            ([90, 140, 0, 0],  "Descend to Pick"),
            ([90, 140, 0, 20],   "Actuate Gripper (Close)"),
            ([90, 90, 90, 20],   "Retract to Safe Height"),
            ([0, 90, 90, 20],    "Transport to Station 2"),
            ([0, 140, 0, 20],    "Descend to Place"),
            ([0, 140, 0, 0],   "Release Object"),
            ([0, 90, 90, 0],   "Retract to Safe Height")
        ]

        for angles, desc in sequence:
            logger.info(f"Step: {desc}")
            
            # Sanitize inputs (0-180 constraint)
            safe_angles = [max(0, min(180, a)) for a in angles]
            
            if safe_angles != self.current_angles:
                self.move_to(safe_angles)
            
            # Dwell time for mechanics to settle
            time.sleep(1.0)

        logger.info("Routine Complete.")


# --- MAIN APPLICATION ---
def main():
    config = RobotConfig(port="AUTO") # Set to specific COM port if known
    bot = RobotController(config)

    if not bot.connect():
        return

    try:
        # Move to Home on startup
        logger.info("Homing...")
        bot.move_to(config.home_angles)

        while True:
            print("\n=== ARDUINO ROBOT CONTROLLER ===")
            print(" [P] Run Pick & Place Routine")
            print(" [I] Interactive Coordinate Mode")
            print(" [M] Manual Joint Control")
            print(" [Q] Quit")
            
            cmd = input("Select Operation: ").strip().lower()

            if cmd == 'q':
                break
            
            elif cmd == 'p':
                bot.run_pick_and_place_job()
            
            elif cmd == 'i':
                try:
                    inp = input("Enter Target (X Y Z [Grip]): ")
                    parts = list(map(float, inp.split()))
                    if len(parts) < 3: raise ValueError
                    
                    grip = parts[3] if len(parts) > 3 else 90
                    
                    sol = bot.ik.solve_ik(parts[0], parts[1], parts[2], grip)
                    if sol:
                        logger.info(f"Moving to: {sol}")
                        bot.move_to(sol)
                    else:
                        logger.warning("Target Unreachable")
                except ValueError:
                    logger.error("Invalid Input Format")

            elif cmd == 'm':
                # Simplified manual control for brevity
                try:
                    j_idx = int(input("Joint Index (0-3): "))
                    angle = int(input("Angle (0-180): "))
                    
                    new_angles = list(bot.current_angles)
                    new_angles[j_idx] = max(0, min(180, angle))
                    bot.move_to(new_angles)
                except (ValueError, IndexError):
                    logger.error("Invalid Joint/Angle")

    except KeyboardInterrupt:
        logger.warning("Emergency Stop Triggered by User")
    finally:
        bot.disconnect()

if __name__ == '__main__':
    main()