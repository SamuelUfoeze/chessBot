import serial
import time

# ==================== CONFIGURATION ====================
SERIAL_PORT = "/dev/ttyUSB0"                         
BAUD_RATE = 115200

# Base microstep calibration unit mappings
STEPS_PER_10MM_BASE = 90   
STEPS_PER_10MM_ARM1 = 90  
STEPS_PER_10MM_ARM2 = 90  

# Automatically compute large scale offsets (5x multiplier)
STEPS_PER_50MM_BASE = STEPS_PER_10MM_BASE * 5   
STEPS_PER_50MM_ARM1 = STEPS_PER_10MM_ARM1 * 5  
STEPS_PER_50MM_ARM2 = STEPS_PER_10MM_ARM2 * 5  
# =======================================================

print("[System] Connecting to robot shield...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5)
time.sleep(2) # Allow system homing setup to complete completely

current_b = 0
current_a1 = 0
current_a2 = 0

def send_step_command(b, a1, a2, grab=0):
    """ Transmits position targets to Arduino and holds script processing until verified """
    cmd = f"M {b} {a1} {a2} {grab}\n"
    ser.write(cmd.encode())
    while True:
        line = ser.readline().decode().strip()
        if line == "OK":
            break

def jog_axis_menu(point_label):
    global current_b, current_a1, current_a2  
    print(f"\n================================================")
    print(f" JOGGING TO PORTION: {point_label.upper()}")
    print(f"================================================")
    print("Controls Matrix:")
    print("  Fine Tuning (10mm):            Coarse Adjustments (50mm):")
    print("    [q / a] -> Base (+ / -)        [Q / A] -> Base (+ / -)")
    print("    [w / s] -> Arm 1 (+ / -)       [W / S] -> Arm 1 (+ / -)")
    print("    [e / d] -> Arm 2 (+ / -)       [E / D] -> Arm 2 (+ / -)")
    print("\n  [ SPACE ] -> Lock this physical point and save steps")
    print("------------------------------------------------")

    # Send an initial position command to snap-lock holding currents instantly
    send_step_command(current_b, current_a1, current_a2)

    while True:
        print(f"Current Target Positions -> Base: {current_b} steps | Arm1: {current_a1} steps | Arm2: {current_a2} steps")
        user_key = input("Enter command key: ").strip() # Removed default lower() to distinguish case sensitivity

        # --- FINE TUNING INCREMENTS (10mm) ---
        if user_key == 'q':    current_b += STEPS_PER_10MM_BASE
        elif user_key == 'a':  current_b -= STEPS_PER_10MM_BASE
        elif user_key == 'w':  current_a1 += STEPS_PER_10MM_ARM1
        elif user_key == 's':  current_a1 -= STEPS_PER_10MM_ARM1
        elif user_key == 'e':  current_a2 += STEPS_PER_10MM_ARM2
        elif user_key == 'd':  current_a2 -= STEPS_PER_10MM_ARM2

        # --- COARSE JOGGING INCREMENTS (50mm via Shift Key combinations) ---
        elif user_key == 'Q':  current_b += STEPS_PER_50MM_BASE
        elif user_key == 'A':  current_b -= STEPS_PER_50MM_BASE
        elif user_key == 'W':  current_a1 += STEPS_PER_50MM_ARM1
        elif user_key == 'S':  current_a1 -= STEPS_PER_50MM_ARM1
        elif user_key == 'E':  current_a2 += STEPS_PER_50MM_ARM2
        elif user_key == 'D':  current_a2 -= STEPS_PER_50MM_ARM2
        
        elif user_key == ' ':
            print(f"[SAVED] Stored steps for {point_label.upper()}!")
            return current_b, current_a1, current_a2
        else:
            print("Invalid key! Shift characters (Q,W,E,A,S,D) move 50mm. Lowers move 10mm.")
            continue

        # Keep drivers fully energized by assigning the updated step count targets immediately
        send_step_command(current_b, current_a1, current_a2)

def main():
    print("=====================================================")
    print("CHESSBOT STEP-BASED MATRIX CALIBRATION")
    print("=====================================================")
    
    # 1. Map A1 position tracking configurations
    a1_b, a1_a1, a1_a2 = jog_axis_menu("Square A1 (Head down resting on piece)")
    
    print("\nLifting head to safe traversal height...")
    send_step_command(a1_b, a1_a1, a1_a2 + (STEPS_PER_10MM_ARM2 * 5))
    
    # 2. Map A5 position tracking configurations
    a5_b, a5_a1, a5_a2 = jog_axis_menu("Square A5 (Head down resting on piece)")
    send_step_command(a5_b, a5_a1, a5_a2 + (STEPS_PER_10MM_ARM2 * 5))
    
    # 3. Map H1 position tracking configurations
    h1_b, h1_a1, h1_a2 = jog_axis_menu("Square H1 (Head down resting on piece)")
    send_step_command(h1_b, h1_a1, h1_a2 + (STEPS_PER_10MM_ARM2 * 5))

    print("\n================= STEP MATRICES REGISTERED =================")
    print("Your arm can now scale the board by interpolation!")
    print(f"A1 Steps  -> Base: {a1_b}, Arm1: {a1_a1}, Arm2: {a1_a2}")
    print(f"A5 Steps  -> Base: {a5_b}, Arm1: {a5_a1}, Arm2: {a5_a2}")
    print(f"H1 Steps  -> Base: {h1_b}, Arm1: {h1_a1}, Arm2: {h1_a2}")
    print("============================================================")

if __name__ == "__main__":
    main()

