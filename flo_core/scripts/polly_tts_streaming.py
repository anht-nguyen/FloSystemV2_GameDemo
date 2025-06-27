import boto3
import pyaudio

class PollyTTSStream:
    """
    Amazon Polly TTS streaming playback using PyAudio (raw PCM).
    """
    def __init__(self, voice_id: str = "Salli", region_name: str = "us-east-1", sample_rate: int = 16000):
        # Initialize Polly client
        client_kwargs = {}
        if region_name:
            client_kwargs['region_name'] = region_name
        self.polly = boto3.client('polly', **client_kwargs)

        # Store voice and sample rate
        self.voice_id = voice_id
        self.sample_rate = sample_rate

        # Initialize PyAudio stream for 16-bit PCM mono
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            output=True
        )

    def speak(self, text: str):
        """
        Stream synthesized speech for `text` directly to audio output.
        """
        # Request raw PCM from AWS Polly
        response = self.polly.synthesize_speech(
            Text=text,
            VoiceId=self.voice_id,
            OutputFormat='pcm',
            SampleRate=str(self.sample_rate)
        )
        # Read and play in chunks
        if 'AudioStream' in response:
            stream_body = response['AudioStream']
            chunk_size = 1024  # bytes
            while True:
                data = stream_body.read(chunk_size)
                if not data:
                    break
                self.stream.write(data)
        else:
            raise RuntimeError("No AudioStream in Polly response")

    def __del__(self):
        # Clean up PyAudio resources
        try:
            self.stream.stop_stream()
            self.stream.close()
            self.audio.terminate()
        except Exception:
            pass
