"""Mac camera sender: pause UDP output once, then resume.

Compatible with gen3_hand_bridge_v3.py. Restart that bridge before this
sender because the sequence counter begins at zero. No robot commands here.
"""
import json
import socket
import time

import cv2
import mediapipe as mp

PAUSE_AT = 20.0
PAUSE_DURATION = 3.0


def phase(elapsed):
    if elapsed < PAUSE_AT:
        return "countdown"
    if elapsed < PAUSE_AT + PAUSE_DURATION:
        return "paused"
    return "resumed"


def main():
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)
    sequence = 0
    last_sent = 0.0
    start = None
    last_phase = None
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
                if start is None:
                    start = now
                elapsed = now - start
                current_phase = phase(elapsed)
                if current_phase != last_phase:
                    print(f"[{time.time():.3f}] Sender phase: {current_phase}", flush=True)
                    last_phase = current_phase

                if current_phase != "paused" and now - last_sent >= 0.05:
                    sender.sendto(json.dumps(message).encode(), ("192.168.64.2", 5005))
                    sequence += 1
                    last_sent = now

                if current_phase == "countdown":
                    banner = f"Sending. Test pause in {PAUSE_AT - elapsed:.1f}s"
                elif current_phase == "paused":
                    banner = "MESSAGES PAUSED - camera still running"
                else:
                    banner = "Messages resumed - Q to close"
                for y, text in ((30, banner), (65, label)):
                    cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (0, 255, 0), 2)
                cv2.imshow("Hand sender - automatic gap test", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        camera.release()
        cv2.destroyAllWindows()
        sender.close()


if __name__ == "__main__":
    main()
