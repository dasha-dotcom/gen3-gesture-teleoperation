"""Combined palm/gesture sender for MediaPipe 0.10.21.

Experimental four-finger heuristic, not a trained gesture recognizer.
Thumb position is not checked. Start with your palm facing the camera.
Default is preview only. --send transmits labels to Ubuntu port 5007.
"""
import argparse
import json
import math
import socket
import time
import uuid

import cv2
import mediapipe as mp


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def angle(a, b, c):
    u = [x - y for x, y in zip(a, b)]
    v = [x - y for x, y in zip(c, b)]
    denom = math.sqrt(sum(x*x for x in u) * sum(x*x for x in v))
    if denom < 1e-10:
        return None
    cosine = sum(x*y for x, y in zip(u, v)) / denom
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def classify(points):
    states = []
    for base in (5, 9, 13, 17):
        mcp, pip, dip, tip = points[base:base+4]
        bend = angle(mcp, pip, dip)
        end_bend = angle(pip, dip, tip)
        length = distance(mcp, pip)
        if bend is None or end_bend is None or length < 1e-6:
            states.append("?")
        elif bend > 140 and end_bend > 140:
            states.append("straight")
        elif bend < 115 and distance(mcp, tip) < 1.6 * length:
            states.append("curled")
        else:
            states.append("?")
    if all(s == "straight" for s in states):
        return "OPEN", states
    if all(s == "curled" for s in states):
        return "CLOSED", states
    return "UNCERTAIN", states


class StableLabel:
    def __init__(self):
        self.candidate = None
        self.since = 0.0
        self.last = None
        self.count = 0

    def update(self, raw, now):
        if raw != self.candidate or self.last is None or now - self.last > 0.3:
            self.candidate, self.since, self.count = raw, now, 0
        self.last = now
        self.count += 1
        if raw not in ("OPEN", "CLOSED"):
            return raw
        if now - self.since >= 0.3 and self.count >= 3:
            return raw
        return "UNCERTAIN"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--send", action="store_true", help="Send palm and gesture data to Ubuntu")
    args = parser.parse_args()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.send else None
    session = uuid.uuid4().hex
    sequence = 0
    last_sent = None
    camera = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)
    hands_api = mp.solutions.hands
    drawing = mp.solutions.drawing_utils
    stable = StableLabel()
    try:
        if not camera.isOpened():
            raise RuntimeError("Cannot open camera. Close other camera previews first.")
        with hands_api.Hands(max_num_hands=1, min_detection_confidence=0.6,
                             min_tracking_confidence=0.6) as detector:
            while True:
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("Camera frame unavailable.")
                frame = cv2.flip(frame, 1)
                result = detector.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                raw, states = "NO HAND", []
                visible = False
                u = v = 0.5
                measurements = []
                if result.multi_hand_landmarks:
                    drawing.draw_landmarks(frame, result.multi_hand_landmarks[0],
                                           hands_api.HAND_CONNECTIONS)
                    raw = "UNCERTAIN"
                    palm = [result.multi_hand_landmarks[0].landmark[i]
                            for i in (0, 5, 9, 13, 17)]
                    u = sum(p.x for p in palm) / len(palm)
                    v = sum(p.y for p in palm) / len(palm)
                    visible = all(math.isfinite(x) and 0 <= x <= 1 for x in (u, v))
                    if result.multi_hand_world_landmarks:
                        points = [(p.x, p.y, p.z) for p in
                                  result.multi_hand_world_landmarks[0].landmark]
                        if all(math.isfinite(v) for p in points for v in p):
                            raw, states = classify(points)
                            for name, base in zip(
                                ("Index", "Middle", "Ring", "Little"), (5, 9, 13, 17)
                            ):
                                mcp, pip, dip, tip = points[base:base+4]
                                first = angle(mcp, pip, dip)
                                second = angle(pip, dip, tip)
                                if first is None or second is None:
                                    measurements.append(f"{name}: invalid geometry")
                                else:
                                    measurements.append(
                                        f"{name}: middle={first:.1f}, tip={second:.1f} degrees"
                                    )
                now = time.monotonic()
                label = stable.update(raw if visible else "NO HAND", now)
                if sender is not None and (last_sent is None or now - last_sent >= 0.05):
                    message = {"session": session, "seq": sequence, "gesture": label,
                               "hand": visible, "u": u, "v": v}
                    sender.sendto(json.dumps(message).encode(), ("192.168.64.2", 5007))
                    sequence += 1
                    last_sent = now
                banner = "Sending palm + gesture to Ubuntu:5007" if args.send else "PREVIEW ONLY"
                lines = [banner,
                         f"Gesture: {label}   Palm: u={u:.3f}, v={v:.3f}",
                         "Index / middle / ring / little: " + " / ".join(states),
                         "Palm toward camera. P: print angles. Q: quit."]
                lines.extend(measurements)
                for i, line in enumerate(lines):
                    cv2.putText(frame, line, (10, 28 + i*28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.imshow("Gen3 combined hand sender", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("p"):
                    print(f"\nGesture: {label}; raw: {raw}", flush=True)
                    print("OPEN requires middle > 140 and tip > 140 for all four fingers.",
                          flush=True)
                    print("\n".join(measurements) or "No valid 3D hand measurements.",
                          flush=True)
                if key == ord("q"):
                    break
    finally:
        camera.release()
        cv2.destroyAllWindows()
        if sender is not None:
            sender.close()


if __name__ == "__main__":
    main()
