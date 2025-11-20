import cv2
import time
import numpy as np
from pathlib import Path
import threading


class DisplayManager:
    """
    Pure display module: window, penguin filler, layout, overlays.
    Does NOT know about ROS or decoding.
    """

    def __init__(self, get_frames_fn, get_stats_fn, logger):
        self.get_frames = get_frames_fn
        self.get_stats = get_stats_fn
        self.log = logger
        self.running = True

        self.penguin = self._load_penguin_image()

        self.thread = threading.Thread(target=self._display_loop, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------
    def stop(self):
        self.running = False

    # ------------------------------------------------------------------
    def _load_penguin_image(self):
        path = Path(__file__).with_name("decoder_cfg.dat")
        if not path.exists():
            self.log.warn("🐧 No penguin data — using blank placeholder")
            return np.zeros((480, 640, 3), np.uint8)

        try:
            hex_data = path.read_text().strip()
            img = cv2.imdecode(np.frombuffer(bytes.fromhex(hex_data), np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError("Decode failed")
            return cv2.resize(img, (640, 480))
        except Exception as e:
            self.log.error(f"Failed loading penguin: {e}")
            return np.zeros((480, 640, 3), np.uint8)

    # ------------------------------------------------------------------
    def _display_loop(self):
        while self.running:
            frames = self.get_frames()
            stats = self.get_stats()

            if not frames:
                time.sleep(0.05)
                continue

            topics = list(frames.keys())
            imgs = [frames[t] for t in topics]

            # normalize height
            h = min(img.shape[0] for img in imgs)
            imgs = [
                cv2.resize(img, (int(img.shape[1] * h / img.shape[0]), h))
                for img in imgs
            ]

            num = len(imgs)

            if num == 1:
                ph = cv2.resize(self.penguin, (imgs[0].shape[1], imgs[0].shape[0]))
                combined = np.hstack([imgs[0], ph])

            elif num in (2, 3):
                while len(imgs) < 4:
                    ph = cv2.resize(self.penguin, (imgs[0].shape[1], imgs[0].shape[0]))
                    imgs.append(ph)
                combined = self._grid(imgs)

            else:
                combined = self._grid(imgs)

            # overlays
            y = 30
            for t in topics:
                s = stats.get(t, {})
                txt = f"{t}: {s.get('mbit_s', 0):.2f} Mbps  {s.get('fps', 0):.1f} FPS"
                cv2.putText(combined, txt, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                y += 30

            cv2.imshow("FPV MultiView", combined)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.running = False

        cv2.destroyAllWindows()

    # ------------------------------------------------------------------
    def _grid(self, imgs, cols=2):
        rows = []
        for i in range(0, len(imgs), cols):
            row = np.hstack(imgs[i:i+cols])
            rows.append(row)

        max_w = max(r.shape[1] for r in rows)
        for i, r in enumerate(rows):
            if r.shape[1] < max_w:
                pad = np.zeros((r.shape[0], max_w - r.shape[1], 3), np.uint8)
                rows[i] = np.hstack([r, pad])

        return np.vstack(rows)
