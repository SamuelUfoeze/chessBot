import serial
import time
import math

# ==================== CONFIGURATION ====================
SERIAL_PORT = "/dev/ttyUSB0"                         
BAUD_RATE = 115200

# Arm Link Lengths (in mm)
L1 = 150.0  
L2 = 170.0  
STEPS_PER_DEGREE = 8.88  

# Physical Chessboard Geometry
BOARD_TOTAL_SIZE = 160.0 # Your board size in mm (A1 to H8 bounding span)
SQUARE_SIZE = BOARD_TOTAL_SIZE / 7.0 # Spacing between square centers (7 transitions from A to H)

# Vertical Heights relative to your starting touch-point
Z_HOVER = 80.0   # Safe altitude to fly over pieces
Z_GRAB = 0.0     # 0.0 means the exact depth where you manually placed the arm at start
# =======================================================

# Establish Connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5)
time.sleep(2) 

# --- CRITICAL STEP: Reverse Engineering your Start Position ---
# Since you placed the arm manually on A1, we calculate what its 
# real physical coordinate must be relative to the robot's pivot base.
# Assuming a standard geometric setup where A1 sits roughly at X=100, Y=100:
START_A1_X = 100.0  
START_A1_Y = 100.0

def inverse_kinematics(x, y, z):
    """ Converts physical board coordinates to raw motor microsteps """
    theta_base = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2)
    d = math.sqrt(r**2 + z**2)
    alpha = math.atan2(z, r)
    
    beta = math.acos(max(-1.0, min(1.0, (L1**2 + d**2 - L2**2) / (2 * L1 * d))))
    gamma = math.acos(max(-1.0, min(1.0, (L1**2 + L2**2 - d**2) / (2 * L1 * L2))))
    
    theta1 = math.degrees(alpha + beta)
    theta2 = math.degrees(gamma)
    theta_base_deg = math.degrees(theta_base)

    return int(theta_base_deg * STEPS_PER_DEGREE), int(theta1 * STEPS_PER_DEGREE), int(theta2 * STEPS_PER_DEGREE)

# Since we aren't using limit switches, we need to know what the motor angles 
# were at our manual A1 starting position so we can use relative offsets.
START_B, START_A1, START_A2 = inverse_kinematics(START_A1_X, START_A1_Y, Z_GRAB)

def send_coordinate_command(target_x, target_y, target_z, grab=0):
    """ Calculates step differences from your manual starting point and moves """
    # 1. Calculate absolute steps needed for the new target coordinate
    raw_b, raw_a1, raw_a2 = inverse_kinematics(target_x, target_y, target_z)
    
    # 2. Subtract our manual A1 reference so the step counts start at 0 relative to power-on
    relative_b = raw_b - START_B
    relative_a1 = raw_a1 - START_A1
    relative_a2 = raw_a2 - START_A2
    
    cmd = f"M {relative_b} {relative_a1} {relative_a2} {grab}\n"
    ser.write(cmd.encode())
    while True:
        line = ser.readline().decode().strip()
        if line == "OK":
            break

def test_auto_point(square_label, target_x, target_y):
    print(f"\n[Auto-Probe] Moving to {square_label.upper()}...")
    
    # Fly over the square safely
    send_coordinate_command(target_x, target_y, Z_HOVER)
    time.sleep(0.5)
    
    # Drop down vertically to check if it lands exactly on the square center
    print(f"-> Dropping down to check alignment on {square_label.upper()}")
    send_coordinate_command(target_x, target_y, Z_GRAB)
    
    input("Verify alignment visually. Press ENTER to fly to next point...")
    
    # Lift back up to safety
    send_coordinate_command(target_x, target_y, Z_HOVER)

def main():
    print("=====================================================")
    print("AUTOMATED BOUNDARY PROBE VIA FIXED 160MM MATRIX")
    print("Make sure you manually placed the arm on A1 before power-on!")
    print("=====================================================")
    input("Press ENTER to begin automated sequence...")

    # Calculate coordinates relative to our A1 origin
    # A1 = Origin
    # A5 = 4 squares up on Y axis
    # H5 = 7 squares right on X, 4 squares up on Y
    # H1 = 7 squares right on X axis
    
    a5_x = START_A1_X
    a5_y = START_A1_Y + (4 * SQUARE_SIZE)
    
    h5_x = START_A1_X + (7 * SQUARE_SIZE)
    h5_y = START_A1_Y + (4 * SQUARE_SIZE)
    
    h1_x = START_A1_X + (7 * SQUARE_SIZE)
    h1_y = START_A1_Y

    # Run auto routine checks
    test_auto_point("a5", a5_x, a5_y)
    test_auto_point("h5", h5_x, h5_y)
    test_auto_point("h1", h1_x, h1_y)
    
    # Return home back over original manual placement spot
    print("\n[Complete] Returning back to sit over A1 safely.")
    send_coordinate_command(START_A1_X, START_A1_Y, Z_HOVER)

if __name__ == "__main__":
    main()

