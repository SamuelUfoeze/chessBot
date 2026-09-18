import math
import serial
import time
import sys

# ==================== CONFIGURATION ====================
SERIAL_PORT = "/dev/ttyUSB0"                         
BAUD_RATE = 115200

# Arm Link Lengths (in mm)
L1 = 150.0  
L2 = 170.0  
STEPS_PER_DEGREE = 8.88  

# Safe Heights
Z_HOVER = 95.0  # High safe cruising height to clear all pieces
# =======================================================

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5)
time.sleep(2) 

# Current physical position estimates during manual jogging
current_x = 120.0
current_y = 120.0

def inverse_kinematics(x, y, z):
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

def send_move(x, y, z):
    b, a1, a2 = inverse_kinematics(x, y, z)
    cmd = f"M {b} {a1} {a2} 0\n"
    ser.write(cmd.encode())
    while True:
        line = ser.readline().decode().strip()
        if line == "OK": break

def jog_to_square(square_label):
    global current_x, current_y
    print(f"\n--- Jogging to center of square: {square_label.upper()} ---")
    print("Controls: W/S (Forward/Back Y), A/D (Left/Right X), Spacebar to lock coordinate.")
    
    # Bring arm down to a low level so you can see where the peg is pointing
    send_move(current_x, current_y, 40.0) 
    
    while True:
        # Simple non-blocking terminal input reader alternative
        move_input = input(f"Current [X={current_x:.1f}, Y={current_y:.1f}]. Enter Command (w/a/s/d or space): ").strip().lower()
        
        if move_input == 'w': current_y += 5.0
        elif move_input == 's': current_y -= 5.0
        elif move_input == 'd': current_x += 5.0
        elif move_input == 'a': current_x -= 5.0
        elif move_input == ' ':
            # User locked it in. Let's find the exact surface height of the board
            z_surface = 40.0
            print("\nNow calibrate depth (Z). Press W to lift up, S to press DOWN into the piece.")
            while True:
                z_input = input(f"Current Z={z_surface:.1f}. Enter (w=up, s=down, space=lock): ").strip().lower()
                if z_input == 'w': z_surface += 2.0
                elif z_input == 's': z_surface -= 2.0
                elif z_input == ' ':
                    send_move(current_x, current_y, Z_HOVER) # Ascend safely immediately
                    return current_x, current_y, z_surface
                send_move(current_x, current_y, z_surface)
        
        send_move(current_x, current_y, 40.0)

def main():
    print("=====================================================")
    print("CHESSBOT BOUNDARY & SCALE CALIBRATION")
    print("Make sure your Arduino has homed against its limit switches!")
    print("=====================================================")
    
    # 1. Capture A1 coordinates
    a1_x, a1_y, a1_z = jog_to_square("a1")
    
    # 2. Capture A5 coordinates (Moving up the rank)
    a5_x, a5_y, _ = jog_to_square("a5")
    
    # 3. Capture H1 coordinates (Moving across the file)
    h1_x, h1_y, _ = jog_to_square("h1")
    
    # --- MATH ENGINE: Auto-calculate board parameters ---
    # Total distance spanning 4 square transitions (from rank 1 to rank 5)
    total_y_dist = a5_y - a1_y
    calculated_square_size_y = total_y_dist / 4.0
    
    # Total distance spanning 7 square transitions (from file A to file H)
    total_x_dist = h1_x - a1_x
    calculated_square_size_x = total_x_dist / 7.0
    
    # Average them out to absorb small physical imperfections
    final_square_size = (calculated_square_size_x + calculated_square_size_y) / 2.0
    
    # Your calibrated piece-grab compression calculations
    # Drop down an extra 7mm past the clean touch surface to ensure full engagement
    final_z_grab = a1_z - 7.0 
    
    print("\n================ CALIBRATION RESULTS ================")
    print("Copy and paste these directly into your main game script configuration block:")
    print("-----------------------------------------------------")
    print(f"BOARD_ORIGIN_X = {a1_x:.2f}")
    print(f"BOARD_ORIGIN_Y = {a1_y:.2f}")
    print(f"SQUARE_SIZE = {final_square_size:.2f}")
    print(f"Z_HOVER = {Z_HOVER:.2f}")
    print(f"Z_GRAB = {final_z_grab:.2f}  # (Auto-compressed below surface level)")
    print("=====================================================")

if __name__ == "__main__":
    main()

