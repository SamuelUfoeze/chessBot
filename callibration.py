import math
import serial
import time

# ==================== CONFIGURATION (Match your main script) ====================
SERIAL_PORT = "/dev/ttyUSB0"                         # Adjust to your Arduino Port
BAUD_RATE = 115200

# Arm Link Lengths (in mm)
L1 = 150.0  
L2 = 170.0  
STEPS_PER_DEGREE = 8.88  

# Board Position (Center of Square A1 relative to Robot Base in mm)
BOARD_ORIGIN_X = 100.0  
BOARD_ORIGIN_Y = 100.0
SQUARE_SIZE = 30.0      # Size of each chess square in mm

# Safe Vertical Constraints
Z_HOVER = 95.0          
Z_GRAB = 25.0           # The height where the peg should meet the board surface
# ================================================================================

# Initialize Serial Connection
print("[Calibration] Connecting to Arduino...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5)
time.sleep(2) # Wait for bootloader

def inverse_kinematics(x, y, z):
    """ Calculates motor step counts for 3-DOF parallel arm linkage """
    theta_base = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2)
    d = math.sqrt(r**2 + z**2)
    alpha = math.atan2(z, r)
    
    beta = math.acos(max(-1.0, min(1.0, (L1**2 + d**2 - L2**2) / (2 * L1 * d))))
    gamma = math.acos(max(-1.0, min(1.0, (L1**2 + L2**2 - d**2) / (2 * L1 * L2))))
    
    theta1 = math.degrees(alpha + beta)
    theta2 = math.degrees(gamma)
    theta_base_deg = math.degrees(theta_base)

    step_base = int(theta_base_deg * STEPS_PER_DEGREE)
    step_arm1 = int(theta1 * STEPS_PER_DEGREE)
    step_arm2 = int(theta2 * STEPS_PER_DEGREE)
    
    return step_base, step_arm1, step_arm2

def send_robot_command(x, y, z, grab):
    """ Converts coordinates to steps and transmits to Arduino """
    b, a1, a2 = inverse_kinematics(x, y, z)
    cmd = f"M {b} {a1} {a2} {1 if grab else 0}\n"
    ser.write(cmd.encode())
    
    # Wait for the physical movement to finish
    while True:
        line = ser.readline().decode().strip()
        if line == "OK":
            break

def algebraic_to_cartesian(square_name):
    """ Translates algebraic notation to physical (X,Y) millimeters """
    file_idx = ord(square_name[0]) - ord('a')  # a=0, b=1 ... h=7
    rank_idx = int(square_name[1]) - 1         # 1=0, 2=1 ... 5=4
    
    x = BOARD_ORIGIN_X + (file_idx * SQUARE_SIZE)
    y = BOARD_ORIGIN_Y + (rank_idx * SQUARE_SIZE)
    return x, y

def test_square(square_name):
    """ Moves to a square at hover height, drops down to verify alignment, and lifts back up """
    x, y = algebraic_to_cartesian(square_name)
    print(f"\n---> Testing square {square_name.upper()} (Physical X: {x}mm, Y: {y}mm)")
    input("Press Enter to move the arm over the square...")
    
    # 1. Hover directly above target
    send_robot_command(x, y, Z_HOVER, grab=False)
    
    # 2. Precision descent to verification height
    print(f"Descending to Z={Z_GRAB}mm. Check if the peg hits the center of the square.")
    send_robot_command(x, y, Z_GRAB, grab=False)
    
    input("Alignment check complete. Press Enter to ascend safely...")
    
    # 3. Safe vertical accent before next movement phase
    send_robot_command(x, y, Z_HOVER, grab=False)

def run_calibration():
    # The custom boundaries requested (Corners of the a1-h5 bounding rectangle)
    calibration_points = ["a1", "a5", "h5", "h1"]
    
    print("\n================ CHESS BOT CALIBRATION ================")
    print("This script will guide you through testing the outer bounds")
    print("of your demo environment setup (a1, a5, h1, h5).")
    print("Ensure your robot is powered up and in its home coordinate state.")
    print("=========================================================")
    
    for point in calibration_points:
        test_square(point)
        
    print("\n[Calibration Completed] All points tested. The arm is currently at safe hover altitude.")

if __name__ == "__main__":
    run_calibration()

