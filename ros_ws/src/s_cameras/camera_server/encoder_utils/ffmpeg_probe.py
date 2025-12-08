import subprocess

# This class is used to probe the system for existing ffmpeg. If we havent the correct gpu hardware drivers we will fallback to CPU instead.
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
            # So, some attemptemps didnt support 128x128 for probing and needed 64x64 instead. 
            # hevc works with 128x128, if not using hevc, the "else" might need to be changed to 64x64.
            size = "128x128" if "hevc" in encoder_name else "128x128" 
            if "vaapi" in encoder_name:
                # Special handling for VAAPI one Ubuntu server (it requiers a display usually so we test headless instead)
                cmd = [
                    "ffmpeg", "-hide_banner", "-loglevel", "error",
                    "-hwaccel", "vaapi",
                    "-vaapi_device", "/dev/dri/renderD128",
                    "-f", "lavfi", "-i", f"testsrc=size={size}:rate=1",
                    "-vf", "format=nv12,hwupload",
                    "-frames:v", "1",
                    "-c:v", encoder_name,
                    "-f", "null", "-"
                ]
            else:
                # Default for NVENC, QSV, CPU, etc. This worked untill tested on the Ubuntu server, hence the if statements.
                cmd = [
                    "ffmpeg", "-hide_banner", "-loglevel", "error",
                    "-f", "lavfi", "-i", f"testsrc=size={size}:rate=1",
                    "-frames:v", "1",
                    "-c:v", encoder_name,
                    "-f", "null", "-",
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
