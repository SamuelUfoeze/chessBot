import cv2
import numpy as np
import math
import serial
import time
import chess
import chess.engine

# ==================== CONFIGURATION ====================
STOCKFISH_PATH = "/home/ai-analyst/Documents/projects/Hardware/chessBot/stockfish-linux-x86-64-universal/stockfish/stockfish-linux-x86-64-universal"  # Adjust to your downloaded binary path
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
Z_HOVER = 80.0          # Safe height to traverse across board
Z_GRAB = 25.0           # Height to drop down and grab piece
# =======================================================

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5)
time.sleep(2) # Wait for serial ready

def inverse_kinematics(x, y, z):
    """ Calculates motor step counts for 3-DOF parallel arm """
    # Base rotation angle (theta_base)
    theta_base = math.atan2(y, x)
    
    # 2D reach along the plane
    r = math.sqrt(x**2 + y**2)
    
    # Trigonometry for 2-link parallel linkage angles
    d = math.sqrt(r**2 + z**2)
    alpha = math.atan2(z, r)
    beta = math.acos(max(-1.0, min(1.0, (L1**2 + d**2 - L2**2) / (2 * L1 * d))))
    gamma = math.acos(max(-1.0, min(1.0, (L1**2 + L2**2 - d**2) / (2 * L1 * L2))))
    
    theta1 = math.degrees(alpha + beta)
    theta2 = math.degrees(gamma)
    theta_base_deg = math.degrees(theta_base)

    # Convert angles to microsteps
    step_base = int(theta_base_deg * STEPS_PER_DEGREE)
    step_arm1 = int(theta1 * STEPS_PER_DEGREE)
    step_arm2 = int(theta2 * STEPS_PER_DEGREE)
    
    return step_base, step_arm1, step_arm2

def send_robot_command(x, y, z, grab):
    b, a1, a2 = inverse_kinematics(x, y, z)
    cmd = f"M {b} {a1} {a2} {1 if grab else 0}\n"
    ser.write(cmd.encode())
    while True:
        line = ser.readline().decode().strip()
        if line == "OK":
            break

def algebraic_to_cartesian(square_name):
    """ Translates board notation (e.g., 'e2') to physical (X,Y) coordinate """
    file_idx = ord(square_name[0]) - ord('a') # 0-7
    rank_idx = int(square_name[1]) - 1        # 0-7
    
    x = BOARD_ORIGIN_X + (file_idx * SQUARE_SIZE)
    y = BOARD_ORIGIN_Y + (rank_idx * SQUARE_SIZE)
    return x, y

def execute_physical_move(move_str):
    src, dst = move_str[:2], move_str[2:4]
    
    x1, y1 = algebraic_to_cartesian(src)
    x2, y2 = algebraic_to_cartesian(dst)
    
    # Pick up piece
    send_robot_command(x1, y1, Z_HOVER, grab=False)
    send_robot_command(x1, y1, Z_GRAB,  grab=False)
    send_robot_command(x1, y1, Z_GRAB,  grab=True)
    send_robot_command(x1, y1, Z_HOVER, grab=True)
    
    # Place piece
    send_robot_command(x2, y2, Z_HOVER, grab=True)
    send_robot_command(x2, y2, Z_GRAB,  grab=True)
    send_robot_command(x2, y2, Z_GRAB,  grab=False)
    send_robot_command(x2, y2, Z_HOVER, grab=False)

def play_game():
    board = chess.Board()
    engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
    
    print("Game Ready! Enter your moves in SAN or UCI (e.g. e2e4):")
    
    while not board.is_game_over():
        print("\n" + str(board))
        human_move = input("\nYour move: ")
        
        try:
            move = board.parse_san(human_move)
            board.push(move)
        except Exception:
            print("Invalid move. Try again.")
            continue
            
        if board.is_game_over(): break
        
        # Calculate Stockfish Move
        print("Stockfish is thinking...")
        result = engine.play(board, chess.engine.Limit(time=1.0))
        bot_move = result.move.uci()
        print(f"Robot plays: {bot_move}")
        
        # Execute physically
        execute_physical_move(bot_move)
        board.push(result.move)
        
    engine.quit()

if __name__ == "__main__":
    play_game()
