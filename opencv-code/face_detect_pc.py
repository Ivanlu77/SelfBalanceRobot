import cv2 as cv
import time
import numpy as np
import os

def face_detect(img):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    #face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_alt2.xml')
    face_cascade = cv.CascadeClassifier(os.path.join(os.path.dirname(__file__), 'haarcascade_frontalface_alt2.xml'))
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)
    
    # Find the most significant face (largest area)
    most_sig_face = None
    max_area = 0
    
    for (x, y, w, h) in faces:
        area = w * h
        if area > max_area:
            max_area = area
            most_sig_face = (x, y, w, h)
    
    # Draw rectangle only for the most significant face
    if most_sig_face:
        x, y, w, h = most_sig_face
        cv.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
        
        # Add text showing the center position of the face
        center_x, center_y = x+w//2, y+h//2
        cv.putText(img, f"Center: ({center_x},{center_y})", (x, y-10), 
                  cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return img, most_sig_face

def main():
    # Target FPS
    target_fps = 30
    frame_time = 1/target_fps
    res_x = 640
    res_y = 480
    
    # FPS tracking
    fps_start_time = time.time()
    fps = 0
    frame_count = 0

    # Initialize webcam with V4L2 backend (more reliable on Raspberry Pi)
    cap = cv.VideoCapture(0, cv.CAP_V4L2)
    
    if not cap.isOpened():
        print("Error: Could not open camera. Trying fallback method...")
        cap = cv.VideoCapture(0)
        if not cap.isOpened():
            print("Error: Failed to open camera with either method.")
            return
    
    # Lower resolution for better performance
    cap.set(cv.CAP_PROP_FRAME_WIDTH, res_x)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, res_y)
    
    while True:
        # Time at start of processing this frame
        start_time = time.time()
        
        try:
            # Capture frame
            ret, frame = cap.read()
            
            if not ret or frame is None:
                print("Failed to grab frame")
                break
                
            # Process the frame and get face position
            processed_frame, face_position = face_detect(frame)
            
            # Print face position if a face is detected
            if face_position:
                x, y, w, h = face_position
                print(f"Face position: center=({x+w//2},{y+h//2})")
            
            # Calculate and display FPS
            frame_count += 1
            if (time.time() - fps_start_time) > 1:
                fps = frame_count / (time.time() - fps_start_time)
                frame_count = 0
                fps_start_time = time.time()
            
            # Display FPS on frame
            cv.putText(processed_frame, f"FPS: {fps:.1f}", (10, 30), 
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Show the processed frame
            cv.imshow('Face Detection', processed_frame)
        except Exception as e:
            print(f"Error processing frame: {e}")
        
        # Wait for 'q' key to exit
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
            
        # Calculate processing time and add delay if needed to maintain target FPS
        processing_time = time.time() - start_time
        if processing_time < frame_time:
            time.sleep(frame_time - processing_time)
    
    # Release resources
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main() 