import boto3
import os
import tempfile

class PollyTTS:
    def __init__(self, voice_id="Salli", output_format="mp3", region_name=None):
        kwargs = {}
        if region_name:
            kwargs["region_name"] = region_name
        self.client = boto3.client("polly", **kwargs)
        self.voice_id = voice_id
        self.output_format = output_format

    def speak(self, text):
        # 1) Call Polly
        response = self.client.synthesize_speech(
            Text=text,
            VoiceId=self.voice_id,
            OutputFormat=self.output_format
        )
        # 2) Write to a temp file
        tmp_fd, tmp_path = tempfile.mkstemp(suffix="." + self.output_format)
        with os.fdopen(tmp_fd, "wb") as f:
            f.write(response["AudioStream"].read())
        # 3) Play it (using system player)
        os.system(f"mpg123 {tmp_path}")  # ensure mpg123 is installed
        # 4) Clean up
        os.remove(tmp_path)
