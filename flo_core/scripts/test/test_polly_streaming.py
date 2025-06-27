#!/usr/bin/env python3
"""
test_tts_streaming.py

Simple smoke‐test for PollyTTSStream:
- Streams “Testing Amazon Polly streaming playback” via raw PCM
- Uses pyaudio for low-latency output
"""

import time
from polly_tts_streaming import PollyTTSStream

def main():
    # 1) Instantiate the streamer (choose your voice and sample rate)
    tts = PollyTTSStream(voice_id="Joanna", sample_rate=16000)

    # 2) Your test prompt
    prompt = "Testing Amazon Polly streaming playback with low latency."

    print(f"[TEST] Speaking: \"{prompt}\"")
    tts.speak(prompt)

    # 3) Optional: Give the stream a moment to finish
    time.sleep(0.5)
    print("[TEST] Done.")

if __name__ == "__main__":
    main()
