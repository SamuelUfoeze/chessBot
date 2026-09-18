import cv2
import numpy as np
import math
import serial
import time
import chess
import chess.engine

# ==================== CONFIGURATION ====================
STOCKFISH_PATH = "/usr/local/bin/stockfish"  
SERIAL_PORT = "/dev/ttyUSB0"                         
BAUD_RATE = 115200

# Camera Configuration
CAMERA_INDEX = 0        # 0 = Default webcam / overhead USB camera
BOARD_CROP_POINTS = []  # Can be populated to crop precisely to the board corners

# Arm Link Lengths (in mm)
L1 = 150.0  
L2 = 170.0  
STEPS_PER_DEGREE = 8.88  

# Board Position (Center of Square A1 relative to Robot Base in mm)
BOARD_ORIGIN_X = 100.0  
BOARD_ORIGIN_Y = 100.0
SQUARE_SIZE = 30.0      
Z_HOVER = 95.0          
Z_GRAB = 25.0           

# Graveyard Zone Configurations
GRAVEYARD_X = 60.0      
GRAVEYARD_Y = 40.0      
GRAVEYARD_Z_DROP = 35.0 

captured_count = 0 
# =======================================================

# Establish Hardware Connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5)
time.sleep(2) 

# Initialize Video Capture Stream
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("[Error] Could not open overhead camera source.")

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
    file_idx = ord(square_name[0]) - ord('a')  
    rank_idx = int(square_name[1]) - 1         
    x = BOARD_ORIGIN_X + (file_idx * SQUARE_SIZE)
    y = BOARD_ORIGIN_Y + (rank_idx * SQUARE_SIZE)
    return x, y

def get_board_image():
    """ Captures a clean, stable frame from the overhead camera """
    for _ in range(5):  # Flush buffer for real-time accuracy
        cap.read()
    ret, frame = cap.read()
    if not ret:
        return None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Apply a light blur to reduce pixel flickering noise
    return cv2.GaussianBlur(gray, (5, 5), 0)

def detect_human_move(before_img, after_img, board):
    """ Compares board frames to isolate squares where a piece changed position """
    # Calculate absolute difference between frames
    diff = cv2.absdiff(before_img, after_img)
    _, thresh = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)
    
    # We segment the image into an 8x8 virtual matrix grid
    h, w = thresh.shape
    sq_h, sq_w = h // 8, w // 8
    
    square_changes = {}
    
    # Analyze changes square by square
    for file_idx in range(8):
        for rank_idx in range(8):
            # Chess array maps rank 1 at bottom, image indexing maps rank 1 at top
            # Adjust mapping based on your orientation. Default assuming White is bottom:
            img_rank = 7 - rank_idx 
            
            x_start = file_idx * sq_w
            y_start = img_rank * sq_h
            
            square_roi = thresh[y_start:y_start+sq_h, x_start:x_start+sq_w]
            change_pixel_count = cv2.countNonZero(square_roi)
            
            square_name = chr(ord('a') + file_idx) + str(rank_idx + 1)
            square_changes[square_name] = change_pixel_count

    # Sort squares by highest amount of pixel movements
    sorted_squares = sorted(square_changes.items(), key=lambda item: item[1], reverse=True)
    
    # Extract the top candidate squares
    top_candidates = [sq[0] for sq in sorted_squares[:4] if sq[1] > 100]
    
    # Cross-reference candidate combinations against legal moves on the digital board
    for src in top_candidates:
        for dst in top_candidates:
            if src != dst:
                try:
                    move = chess.Move.from_uci(src + dst)
                    # Support pawn promotions visually defaulting to Queen
                    promo_move = chess.Move.from_uci(src + dst + 'q')
                    
                    if move in board.legal_moves:
                        return move
                    elif promo_move in board.legal_moves:
                        return promo_move
                except Exception:
                    continue
    return None

def execute_physical_capture(target_square):
    global captured_count
    cx, cy = algebraic_to_cartesian(target_square)
    drop_x = GRAVEYARD_X + ((captured_count % 4) * 20.0)
    drop_y = GRAVEYARD_Y + ((captured_count // 4) * 20.0)
    
    send_robot_command(cx, cy, Z_HOVER, grab=False)
    send_robot_command(cx, cy, Z_GRAB,  grab=False)
    send_robot_command(cx, cy, Z_GRAB,  grab=True)
    send_robot_command(cx, cy, Z_HOVER, grab=True)
    send_robot_command(drop_x, drop_y, Z_HOVER, grab=True)
    send_robot_command(drop_x, drop_y, GRAVEYARD_Z_DROP, grab=True)
    send_robot_command(drop_x, drop_y, GRAVEYARD_Z_DROP, grab=False)
    send_robot_command(drop_x, drop_y, Z_HOVER, grab=False)
    captured_count += 1

def execute_physical_move(board, move):
    if board.is_capture(move):
        execute_physical_capture(move.uci()[2:4])
        
    x1, y1 = algebraic_to_cartesian(move.uci()[:2])
    x2, y2 = algebraic_to_cartesian(move.uci()[2:4])
    
    send_robot_command(x1, y1, Z_HOVER, grab=False) 
    send_robot_command(x1, y1, Z_GRAB,  grab=False) 
    send_robot_command(x1, y1, Z_GRAB,  grab=True)  
    send_robot_command(x1, y1, Z_HOVER, grab=True)  
    send_robot_command(x2, y2, Z_HOVER, grab=True)  
    send_robot_command(x2, y2, Z_GRAB,  grab=True)  
    send_robot_command(x2, y2, Z_GRAB,  grab=False) 
    send_robot_command(x2, y2, Z_HOVER, grab=False) 

def play_game():
    board = chess.Board()
    engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
    
    print("\n=== SYSTEM ONLINE ===")
    print("1. Line up camera frame cleanly over the boundaries of the board.")
    print("2. Make your physical move on the board.")
    print("3. Press [SPACEBAR] in the camera view window to confirm your move.\n")
    
    # Get initial baseline frame of the starting position
    before_frame = get_board_image()
    
    while not board.is_game_over():
        print(board)
        print("\n[Your Turn] Physically move a piece, click the video window, and press [SPACEBAR].")
        
        while True:
            ret, frame = cap.read()
            if ret:
                cv2.putText(frame, "YOUR TURN - Press SPACE when done", (15, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("Robot Eye", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):  # Spacebar pressed
                after_frame = get_board_image()
                break
                
        # Run differential computer vision analysis
        detected_move = detect_human_move(before_frame, after_frame, board)
        
        if detected_move is None:
            print("[Vision Error] Could not detect a valid legal move. Please re-adjust and try again.")
            # Reset before_frame to current state so user can retry safely
            before_frame = get_board_image()
            continue
            
        print(f"[Vision Confirmed] You played: {detected_move}")
        board.push(detected_move)
        
        if board.is_game_over(): 
            break
            
        # Calculate Stockfish Counter-Move
        print("Stockfish is thinking...")
        result = engine.play(board, chess.engine.Limit(time=1.0))
        print(f"Robot Response: {result.move.uci()}")
        
        # Move physical arm hardware
        execute_physical_move(board, result.move)
        board.push(result.move)
        
        # Take a brand new baseline image because the robot just altered the board pieces
        print("Syncing board vision matrix...")
        time.sleep(0.5)
        before_frame = get_board_image()
        
    print("\nGame Over! Final Result: " + board.result())
    cap.release()
    cv2.destroyAllWindows()
    engine.quit()

if __name__ == "__main__":
    play_game()

