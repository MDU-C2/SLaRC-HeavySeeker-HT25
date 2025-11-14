import subprocess

class FFmpegProbe:

    def supports(self, encoder_name: str) -> bool:
        """Check if ffmpeg lists the encoder."""
        try:
            out = subprocess.getoutput("ffmpeg -hide_banner -encoders")
        except Exception:
            return False
        return encoder_name in out

    def test(self, encoder_name: str) -> bool:

        try:
            size = "128x128" if "hevc" in encoder_name else "64x64"

            cmd = [
                "ffmpeg", "-hide_banner", "-loglevel", "error",
                "-f", "lavfi", "-i", f"testsrc=size={size}:rate=1",
                "-frames:v", "1",
                "-c:v", encoder_name,
                "-f", "null", "-"
            ]

            proc = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5
            )

            return proc.returncode == 0

        except Exception:
            return False

    def is_usable(self, encoder_name: str) -> bool:
        return self.supports(encoder_name) and self.test(encoder_name)
