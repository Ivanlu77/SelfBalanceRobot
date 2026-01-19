import cv2 as cv
import time
import numpy as np
from picamera2 import Picamera2

def face_detect(img):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    #face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_alt2.xml')
    face_cascade = cv.CascadeClassifier('./haarcascade_frontalface_alt2.xml')
    
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
    # Target FPS - reduced for Raspberry Pi
    target_fps = 5
    frame_time = 1/target_fps
    
    # FPS tracking
    fps_start_time = time.time()
    fps = 0
    frame_count = 0
    
    # Desired output resolution
    res_x = 320
    res_y = 240
    
    try:
        # Initialize the Pi camera
        picam2 = Picamera2()
        
        # Configure the camera with scaling instead of cropping
        config = picam2.create_preview_configuration(
            main={"size": (res_x, res_y)},
            raw ={"size": picam2.sensor_modes[-1]["size"]}
        )
        picam2.configure(config)
        
        # Start the camera
        picam2.start()
        print("Pi Camera initialized successfully")
        
    except ImportError:
        print("Error: picamera2 module not found. Please install it using:")
        print("sudo apt install -y python3-picamera2")
        return
    except Exception as e:
        print(f"Error initializing Pi Camera: {e}")
        return
    
    while True:
        # Time at start of processing this frame
        start_time = time.time()
        
        try:
            # Capture frame
            frame = picam2.capture_array()
            
            # Convert from RGB to BGR if necessary
            if frame.shape[2] == 3:
                frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
            
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
    picam2.stop()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main() 
