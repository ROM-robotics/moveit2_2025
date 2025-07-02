import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import time
import random

class GesturePublisher(Node):
    def __init__(self):
        super().__init__('gesture_publisher')
        self.publisher_ = self.create_publisher(String, 'gesture_command', 10)

        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)

        self.hand_colors = {}
        self.last_gesture = ""
        self.last_speech_time = 0

        self.pTime = 0
        self.timer = self.create_timer(0.05, self.process_frame)

    def speak(self, text):
        def run_espeak():
            subprocess.run(['espeak', text])
        threading.Thread(target=run_espeak, daemon=True).start()

    def get_hand_color(self, hand_id):
        if hand_id not in self.hand_colors:
            self.hand_colors[hand_id] = (
                random.randint(50, 255),
                random.randint(50, 255),
                random.randint(50, 255)
            )
        return self.hand_colors[hand_id]

    def count_fingers(self, hand_landmarks, frame, hand_id):
        finger_tips = [8, 12, 16, 20]
        thumb_tip = 4
        fingers_up = 0
        color = self.get_hand_color(hand_id)

        wrist_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].x
        thumb_base_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_CMC].x
        hand_label = "Right" if wrist_x < thumb_base_x else "Left"

        if (hand_label == "Right" and hand_landmarks.landmark[thumb_tip].x > hand_landmarks.landmark[2].x) or \
           (hand_label == "Left" and hand_landmarks.landmark[thumb_tip].x < hand_landmarks.landmark[2].x):
            fingers_up += 1

        for tip in finger_tips:
            if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
                fingers_up += 1

        return fingers_up, color

    def classify_gesture(self, fingers_up):
        if fingers_up == 1:
            return "Pick Object 1"
        elif fingers_up == 2:
            return "Pick Object 2"
        elif fingers_up == 3:
            return "Pick Object 3"
        elif fingers_up == 4:
            return "Pick Object 4"
        elif fingers_up == 5:
            return "Stop"
        else:
            return "No Command Found"

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        if results.multi_hand_landmarks:
            for hand_id, hand_landmarks in enumerate(results.multi_hand_landmarks):
                self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                fingers_up, color = self.count_fingers(hand_landmarks, frame, hand_id)
                gesture = self.classify_gesture(fingers_up)

                cv2.putText(frame, f"Gesture: {gesture}", (50, 50 + hand_id * 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)

                current_time = time.time()
                if gesture != self.last_gesture or (current_time - self.last_speech_time) > 2.0:
                    self.publisher_.publish(String(data=gesture))
                    self.speak(gesture)
                    self.last_gesture = gesture
                    self.last_speech_time = current_time

        self.pTime = time.time()
        cv2.putText(frame, f"{int(1/(time.time()-self.pTime+1e-5))} FPS", (10, 70),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)
        cv2.imshow("ROS2 Gesture Publisher with Speech", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GesturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


