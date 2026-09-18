import cv2
import numpy as np
import math
import serial
import time
import chess
import chess.engine

# ==================== CONFIGURATION ====================
# Global installation path for Stockfish 19
STOCKFISH_PATH = "/usr/local/bin/stockfish"  
SERIAL_PORT = "/dev/ttyUSB0"                         # Adjust to your Arduino Port
BAUD_RATE = 115200

# Arm Link Lengths (in mm)
L1 = 150.0  # Lower arm length
L2 = 170.0  # Upper arm length
STEPS_PER_DEGREE = 8.88  # Microstep multiplier for 1.8° steppers

# Board Position (Center of Square A1 relative to Robot Base in mm)
BOARD_ORIGIN_X = 100.0  
BOARD_ORIGIN_Y = 100.0
SQUARE_SIZE = 30.0      # Size of each chess square in mm

# Safe Vertical Constraints
Z_HOVER = 95.0          # High safe height to clear all pieces during joint rotation
Z_GRAB = 25.0           # Precision drop-down height to interface with piece

# Graveyard Zone Configurations
GRAVEYARD_X = 60.0      # Safe physical X coordinates outside the board boundaries
GRAVEYARD_Y = 40.0      # Safe physical Y coordinates outside the board boundaries
GRAVEYARD_Z_DROP = 35.0 # Release height for dead pieces

# Stagger offset counter so dead pieces do not stack on top of each other
captured_count = 0 
# =======================================================

# Establish Hardware Connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5)
time.sleep(2) # Wait for Arduino bootloader to initialize

def inverse_kinematics(x, y, z):
    """ Calculates motor step counts for 3-DOF parallel arm linkage """
    # Base rotation angle (theta_base)
    theta_base = math.atan2(y, x)
    
    # 2D reach along the horizontal plane
    r = math.sqrt(x**2 + y**2)
    
    # Trigonometry for 2-link parallel linkage angles
    d = math.sqrt(r**2 + z**2)
    alpha = math.atan2(z, r)
    
    # Avoid domain errors by clamping boundaries securely between -1.0 and 1.0
    beta = math.acos(max(-1.0, min(1.0, (L1**2 + d**2 - L2**2) / (2 * L1 * d))))
    gamma = math.acos(max(-1.0, min(1.0, (L1**2 + L2**2 - d**2) / (2 * L1 * L2))))
    
    theta1 = math.degrees(alpha + beta)
    theta2 = math.degrees(gamma)
    theta_base_deg = math.degrees(theta_base)

    # Convert computed joint angles to microsteps
    step_base = int(theta_base_deg * STEPS_PER_DEGREE)
    step_arm1 = int(theta1 * STEPS_PER_DEGREE)
    step_arm2 = int(theta2 * STEPS_PER_DEGREE)
    
    return step_base, step_arm1, step_arm2

def send_robot_command(x, y, z, grab):
    """ Converts coordinates to steps and transmits to GRBL/Arduino via Serial """
    b, a1, a2 = inverse_kinematics(x, y, z)
    cmd = f"M {b} {a1} {a2} {1 if grab else 0}\n"
    ser.write(cmd.encode())
    
    # Blocking wait until Arduino completes execution and acknowledges with 'OK'
    while True:
        line = ser.readline().decode().strip()
        if line == "OK":
            break

def algebraic_to_cartesian(square_name):
    """ Translates board notation (e.g., 'e2') to physical space (X,Y) coordinates """
    file_idx = ord(square_name[0]) - ord('a')  # Column mapping: a=0, b=1 ... h=7
    rank_idx = int(square_name[1]) - 1         # Row mapping: 1=0, 2=1 ... 8=7
    
    x = BOARD_ORIGIN_X + (file_idx * SQUARE_SIZE)
    y = BOARD_ORIGIN_Y + (rank_idx * SQUARE_SIZE)
    return x, y

def execute_physical_capture(target_square):
    """ Moves the arm to remove an enemy piece before the attacking move occurs """
    global captured_count
    cx, cy = algebraic_to_cartesian(target_square)
    
    # Grid math to stagger dead pieces cleanly in rows of 4 inside the graveyard
    drop_x = GRAVEYARD_X + ((captured_count % 4) * 20.0)
    drop_y = GRAVEYARD_Y + ((captured_count // 4) * 20.0)
    
    print(f"[Hardware Action] Clearing captured piece from {target_square} to Graveyard...")
    
    # 1. Hover perfectly over the target piece
    send_robot_command(cx, cy, Z_HOVER, grab=False)
    # 2. Descend vertically to engage
    send_robot_command(cx, cy, Z_GRAB,  grab=False)
    # 3. Activate grab mechanism
    send_robot_command(cx, cy, Z_GRAB,  grab=True)
    # 4. Lift straight up vertically to avoid knocking surrounding pieces
    send_robot_command(cx, cy, Z_HOVER, grab=True)
    
    # 5. Travel horizontally at hover altitude to graveyard area
    send_robot_command(drop_x, drop_y, Z_HOVER, grab=True)
    # 6. Lower piece into graveyard
    send_robot_command(drop_x, drop_y, GRAVEYARD_Z_DROP, grab=True)
    # 7. Release grab mechanism
    send_robot_command(drop_x, drop_y, GRAVEYARD_Z_DROP, grab=False)
    # 8. Reset arm back to high hover altitude
    send_robot_command(drop_x, drop_y, Z_HOVER, grab=False)
    
    captured_count += 1

def execute_physical_move(board, move):
    """ Executes a clean standard move, automatically triggering capture clears first """
    move_str = move.uci()
    src, dst = move_str[:2], move_str[2:4]
    
    # Step A: Evaluate if the square needs to be cleared first
    if board.is_capture(move):
        execute_physical_capture(dst)
        
    x1, y1 = algebraic_to_cartesian(src)
    x2, y2 = algebraic_to_cartesian(dst)
    
    print(f"[Hardware Action] Relocating piece: {src} -> {dst}")
    
    # Step B: Pick up moving piece
    send_robot_command(x1, y1, Z_HOVER, grab=False) # Hover over source
    send_robot_command(x1, y1, Z_GRAB,  grab=False) # Lower down
    send_robot_command(x1, y1, Z_GRAB,  grab=True)  # Secure piece
    send_robot_command(x1, y1, Z_HOVER, grab=True)  # Lift vertically clear
    
    # Step C: Deposit piece on destination square
    send_robot_command(x2, y2, Z_HOVER, grab=True)  # Horizontal travel to destination
    send_robot_command(x2, y2, Z_GRAB,  grab=True)  # Lower into square
    send_robot_command(x2, y2, Z_GRAB,  grab=False) # Release piece
    send_robot_command(x2, y2, Z_HOVER, grab=False) # Ascend back to safe hover

def play_game():
    board = chess.Board()
    engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
    
    print("Game Ready! Enter your moves in SAN (e4) or UCI (e2e4):")
    
    while not board.is_game_over():
        print("\n" + str(board))
        human_move = input("\nYour move: ")
        
        # Validates user inputs dynamically across SAN and UCI formats
        try:
            try:
                move = board.parse_san(human_move)
            except ValueError:
                move = board.parse_uci(human_move)
            
            if move in board.legal_moves:
                board.push(move)
            else:
                print("Illegal move for this board configuration. Try again.")
                continue
        except Exception:
            print("Invalid format. Use clear notation like 'd2d4' or 'Nf3'.")
            continue
            
        if board.is_game_over(): 
            break
        
        # Query engine calculation state
        print("Stockfish is thinking...")
        result = engine.play(board, chess.engine.Limit(time=1.0))
        bot_move = result.move.uci()
        print(f"Robot plays: {bot_move}")
        
        # Intercept and physically translate move across hardware
        execute_physical_move(board, result.move)
        
        # Sync physical hardware move to virtual engine tracker
        board.push(result.move)
        
    print("\nGame Over! Final Match Result: " + board.result())
    engine.quit()

if __name__ == "__main__":
    play_game()

