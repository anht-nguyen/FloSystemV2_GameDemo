import boto3

def test_polly():
    """
    Simple test for Amazon Polly:
    - Synthesizes "Hello from Amazon Polly"
    - Saves output as test.mp3 in the current directory
    """
    try:
        polly = boto3.client('polly', region_name='us-east-1')
        response = polly.synthesize_speech(
            Text='Hello from Amazon Polly',
            VoiceId='Salli',
            OutputFormat='mp3'
        )
        audio_stream = response.get('AudioStream')
        if not audio_stream:
            raise RuntimeError("No AudioStream in response from Polly.")
        
        # Write out to MP3 file
        with open('test.mp3', 'wb') as f:
            f.write(audio_stream.read())
        
        print("✅ Success! 'test.mp3' has been saved. Play it to verify the voice output.")
    except Exception as e:
        print(f"❌ Polly test failed: {e}")

if __name__ == '__main__':
    test_polly()

