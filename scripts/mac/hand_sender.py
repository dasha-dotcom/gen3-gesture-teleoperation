"""Normal Mac camera sender for gen3_hand_bridge_v3.py.

Sends palm coordinates, not video, to Ubuntu over UDP. No deliberate pauses.
Restart the Ubuntu bridge whenever this sender restarts, because its sequence
counter begins at zero. This sender does not publish ROS or robot commands.
"""
import json
import socket
import time

import cv2
import mediapipe as mp

def main():
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)
    sequence = 0
    last_sent = 0.0
    hands_api = mp.solutions.hands
    drawing = mp.solutions.drawing_utils
    try:
        if not camera.isOpened():
            raise RuntimeError("Could not open the camera. Close other previews first.")
        with hands_api.Hands(
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6,
        ) as detector:
            while True:
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("Could not read a camera frame.")
                frame = cv2.flip(frame, 1)
                result = detector.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                message = {"seq": sequence, "hand": False}
                label = "No hand detected"
                if result.multi_hand_landmarks:
                    hand = result.multi_hand_landmarks[0]
                    palm = [hand.landmark[i] for i in (0, 5, 9, 13, 17)]
                    u = sum(p.x for p in palm) / len(palm)
                    v = sum(p.y for p in palm) / len(palm)
                    if 0.0 <= u <= 1.0 and 0.0 <= v <= 1.0:
                        message.update(hand=True, u=u, v=v)
                        label = f"Hand visible: u={u:.3f}, v={v:.3f}"
                    drawing.draw_landmarks(frame, hand, hands_api.HAND_CONNECTIONS)

                now = time.monotonic()
                if now - last_sent >= 0.05:
                    sender.sendto(json.dumps(message).encode(), ("192.168.64.2", 5005))
                    sequence += 1
                    last_sent = now

                banner = "Sending hand data to Ubuntu - Q to close"
                for y, text in ((30, banner), (65, label)):
                    cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (0, 255, 0), 2)
                cv2.imshow("Gen3 hand sender", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        camera.release()
        cv2.destroyAllWindows()
        sender.close()


if __name__ == "__main__":
    main()
