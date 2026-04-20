import os
import json
import subprocess
import tempfile
import threading
import base64
import audioop
import wave
from typing import Dict, List, Optional, Tuple
from urllib import error, request

import rospy
import rospkg
from std_msgs.msg import String
from flo_core_defs.msg import Emotion

from flo_core.polly_tts_streaming import PollyTTSStream


DEFAULT_FACE_LABELS = [
    "sleep",
    "very_happy",
    "sad",
    "very_sad",
    "content",
    "laughing",
    "mischievous",
    "neutral",
    "happy",
    "christmas",
    "very_very_sad",
    "surprised",
    "chortling",
    "so_sad",
    "dead",
    "embarrassed",
]

def _read_secret_file(path: str) -> str:
    if not path or not os.path.exists(path):
        return ""

    with open(path, "r", encoding="utf-8") as handle:
        raw = handle.read().strip()

    if not raw:
        return ""

    if "=" in raw:
        key, value = raw.split("=", 1)
        normalized_key = key.strip().upper()
        if normalized_key == "GEMINI_API_KEY":
            return value.strip().strip("\"'")

    return raw.strip().strip("\"'")


def _default_api_key_file() -> str:
    candidates = [
        os.getenv("GEMINI_API_KEY_FILE", ""),
        "certs/gemini-api-key",
    ]
    for path in candidates:
        if path and os.path.exists(path):
            return path
    return "certs/gemini-api-key"


def _default_record_device() -> str:
    return (
        os.getenv("FLO_RECORD_DEVICE", "")
        or os.getenv("AUDIODEV", "")
        or "pulse"
    )


def _load_face_labels(default_labels: List[str], faces_json_path: str = "") -> List[str]:
    faces_json = faces_json_path
    if not faces_json:
        try:
            rospack = rospkg.RosPack()
            faces_json = os.path.join(rospack.get_path("flo_face"), "data", "faces.json")
        except rospkg.ResourceNotFound:
            rospy.logwarn("Could not locate flo_face package; using fallback face labels.")
            return list(default_labels)

    try:
        with open(faces_json, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except (OSError, ValueError) as exc:
        rospy.logwarn("Could not load face labels from %s: %s. Using fallback labels.", faces_json, exc)
        return list(default_labels)

    labels = sorted(
        name for name in payload.get("mouths", {})
        if isinstance(name, str) and name and not name.startswith("_")
    )
    if not labels:
        rospy.logwarn("No usable face labels found in %s. Using fallback labels.", faces_json)
        return list(default_labels)
    return labels


class GeminiClient:
    def __init__(
        self,
        api_key: str,
        chat_model: str,
        transcription_model: str,
        system_prompt: str,
        timeout_s: float,
    ):
        if not api_key:
            raise ValueError("GEMINI_API_KEY (or ~api_key) is required")
        self._chat_model = chat_model
        self._transcription_model = transcription_model
        self._system_prompt = system_prompt
        self._timeout_s = timeout_s
        self._api_key = api_key

    @staticmethod
    def _normalize_model_name(model: str) -> str:
        normalized = (model or "").strip()
        if normalized.startswith("models/"):
            return normalized[len("models/") :]
        return normalized

    @staticmethod
    def _payload_for_logging(payload: Dict) -> Dict:
        def _sanitize(value):
            if isinstance(value, dict):
                sanitized = {}
                for key, item in value.items():
                    if key in {"data", "inline_data", "inlineData"}:
                        if key == "data" and isinstance(item, str):
                            sanitized[key] = f"<base64:{len(item)} chars>"
                        else:
                            sanitized[key] = _sanitize(item)
                    else:
                        sanitized[key] = _sanitize(item)
                return sanitized
            if isinstance(value, list):
                return [_sanitize(item) for item in value]
            return value

        return _sanitize(payload)

    def _post_generate_content(self, model: str, payload: Dict) -> Dict:
        normalized_model = self._normalize_model_name(model)
        url = (
            "https://generativelanguage.googleapis.com/v1beta/models/"
            f"{normalized_model}:generateContent?key={self._api_key}"
        )
        rospy.loginfo(
            "Gemini generateContent request model=%s payload=%s",
            normalized_model,
            json.dumps(self._payload_for_logging(payload), sort_keys=True),
        )
        req = request.Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with request.urlopen(req, timeout=self._timeout_s) as response:
                return json.loads(response.read().decode("utf-8"))
        except error.HTTPError as exc:
            details = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(
                f"Gemini API request failed for model '{normalized_model}' "
                f"({exc.code} {exc.reason}): {details}"
            )
        except error.URLError as exc:
            raise RuntimeError(
                f"Gemini API network error for model '{normalized_model}': {exc.reason}"
            )

    @staticmethod
    def _extract_text(response: Dict) -> str:
        candidates = response.get("candidates") or []
        for candidate in candidates:
            content = candidate.get("content") or {}
            for part in content.get("parts") or []:
                text = part.get("text")
                if text:
                    return text.strip()
        return ""

    def transcribe(self, audio_path: str, language: str = "") -> str:
        instruction = "Transcribe the user's speech from this audio. Return only the transcript."
        if language:
            instruction += f" The spoken language is likely {language}."

        with open(audio_path, "rb") as handle:
            encoded_audio = base64.b64encode(handle.read()).decode("ascii")

        payload = {
            "contents": [
                {
                    "role": "user",
                    "parts": [
                        {"text": instruction},
                        {
                            "inline_data": {
                                "mime_type": "audio/wav",
                                "data": encoded_audio,
                            }
                        },
                    ],
                }
            ],
            "generationConfig": {
                "temperature": 0.0,
            },
        }
        response = self._post_generate_content(self._transcription_model, payload)
        return self._extract_text(response)

    @staticmethod
    def _build_contents(messages: List[Dict[str, str]]) -> List[Dict]:
        contents = []
        for message in messages:
            role = "model" if message["role"] == "assistant" else "user"
            if message["role"] == "system":
                continue
            contents.append(
                {
                    "role": role,
                    "parts": [{"text": message["content"]}],
                }
            )
        return contents

    def audio_turn(
        self,
        messages: List[Dict[str, str]],
        audio_path: str,
        language: str,
        temperature: float,
        allowed_faces: List[str],
    ) -> Tuple[str, str, str]:
        instruction = (
            "Listen to the user's audio and do two things. "
            "First, transcribe exactly what the user said. "
            "Second, reply as Flo based on the conversation so far. "
            "Return compact JSON with exactly three keys: "
            '"transcript" for the user transcript, "text" for Flo\'s spoken reply, '
            '"face_name" for Flo\'s facial expression. '
            f'The "face_name" value must be exactly one of: {", ".join(allowed_faces)}. '
            "Do not wrap the JSON in markdown fences or add extra commentary."
        )
        if language:
            instruction += f" The spoken language is likely {language}."

        with open(audio_path, "rb") as handle:
            encoded_audio = base64.b64encode(handle.read()).decode("ascii")

        payload = {
            "contents": self._build_contents(messages)
            + [
                {
                    "role": "user",
                    "parts": [
                        {"text": instruction},
                        {
                            "inline_data": {
                                "mime_type": "audio/wav",
                                "data": encoded_audio,
                            }
                        },
                    ],
                }
            ],
            "system_instruction": {
                "parts": [
                    {
                        "text": (
                            self._system_prompt
                            + " For this specific audio request, override the usual response schema and "
                            + 'return exactly three JSON keys: "transcript", "text", and "face_name".'
                        )
                    }
                ],
            },
            "generationConfig": {
                "temperature": temperature,
            },
        }
        response = self._post_generate_content(self._chat_model, payload)
        return _parse_audio_turn_reply(self._extract_text(response), allowed_faces)

    def chat(self, messages: List[Dict[str, str]], temperature: float) -> str:
        contents = self._build_contents(messages)
        payload = {
            "contents": contents,
            "system_instruction": {
                "parts": [{"text": self._system_prompt}],
            },
            "generationConfig": {
                "temperature": temperature,
            },
        }
        response = self._post_generate_content(self._chat_model, payload)
        return self._extract_text(response)


class AudioRecorder:
    def __init__(self, sample_rate: int, channels: int, duration_s: int, device: str = ""):
        self._sample_rate = sample_rate
        self._channels = channels
        self._duration_s = duration_s
        self._device = device

    def record_to_wav(self) -> str:
        tmp = tempfile.NamedTemporaryFile(prefix="flo_user_", suffix=".wav", delete=False)
        tmp.close()

        cmd = [
            "arecord",
            "-q",
            "-f",
            "S16_LE",
            "-r",
            str(self._sample_rate),
            "-c",
            str(self._channels),
            "-d",
            str(self._duration_s),
            tmp.name,
        ]
        if self._device:
            cmd[1:1] = ["-D", self._device]

        rospy.loginfo("Recording user audio to %s", tmp.name)
        subprocess.run(cmd, check=True)
        return tmp.name


def _audio_has_speech(audio_path: str, threshold: int) -> bool:
    if threshold <= 0:
        return True

    with wave.open(audio_path, "rb") as wav_file:
        sample_width = wav_file.getsampwidth()
        while True:
            frames = wav_file.readframes(2048)
            if not frames:
                return False
            if audioop.rms(frames, sample_width) >= threshold:
                return True


def _infer_emotion(text: str) -> int:
    normalized = (text or "").strip().lower()
    if not normalized:
        return Emotion.NEUTRAL

    happy_keywords = (
        "great",
        "glad",
        "awesome",
        "fun",
        "excited",
        "happy",
        "wonderful",
        "nice",
        "cool",
        "play",
        "simon says",
    )
    sad_keywords = (
        "sorry",
        "trouble",
        "cannot",
        "can't",
        "unable",
        "error",
        "failed",
        "goodbye",
        "bye",
    )

    if any(keyword in normalized for keyword in sad_keywords):
        return Emotion.SAD
    if any(keyword in normalized for keyword in happy_keywords):
        return Emotion.HAPPY
    return Emotion.NEUTRAL


def _infer_face_name(text: str) -> str:
    normalized = (text or "").strip().lower()
    if not normalized:
        return "neutral"

    if any(keyword in normalized for keyword in ("wow", "whoa", "surprised", "amazing", "incredible")):
        return "surprised"
    if any(keyword in normalized for keyword in ("embarrassed", "awkward", "oops")):
        return "embarrassed"
    if any(keyword in normalized for keyword in ("goodbye", "bye", "cannot", "can't", "unable", "failed", "error")):
        return "very_sad"
    if any(keyword in normalized for keyword in ("sorry", "trouble", "sad", "unfortunately")):
        return "sad"
    if any(keyword in normalized for keyword in ("haha", "laugh", "funny", "hilarious")):
        return "laughing"
    if any(keyword in normalized for keyword in ("awesome", "fantastic", "wonderful", "excited")):
        return "very_happy"
    if any(keyword in normalized for keyword in ("great", "glad", "happy", "play", "simon says")):
        return "happy"
    if "!" in normalized:
        return "content"
    return "neutral"


def _extract_json_object(raw_text: str) -> str:
    text = (raw_text or "").strip()
    if not text:
        return ""

    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1 or end < start:
        return ""
    return text[start : end + 1]


def _parse_structured_reply(raw_text: str, allowed_faces: List[str]) -> Tuple[str, str]:
    normalized_allowed = set(allowed_faces)
    json_blob = _extract_json_object(raw_text)
    if json_blob:
        try:
            payload = json.loads(json_blob)
            reply_text = str(payload.get("text", "")).strip()
            face_name = str(payload.get("face_name", "")).strip()
            if reply_text and face_name in normalized_allowed:
                return reply_text, face_name
        except (TypeError, ValueError):
            pass

    fallback_text = (raw_text or "").strip()
    return fallback_text, _infer_face_name(fallback_text)


def _parse_audio_turn_reply(raw_text: str, allowed_faces: List[str]) -> Tuple[str, str, str]:
    normalized_allowed = set(allowed_faces)
    json_blob = _extract_json_object(raw_text)
    if json_blob:
        try:
            payload = json.loads(json_blob)
            transcript = str(payload.get("transcript", "")).strip()
            reply_text = str(payload.get("text", "")).strip()
            face_name = str(payload.get("face_name", "")).strip()
            if transcript and reply_text and face_name in normalized_allowed:
                return transcript, reply_text, face_name
        except (TypeError, ValueError):
            pass

    fallback_text = (raw_text or "").strip()
    return "", fallback_text, _infer_face_name(fallback_text)


class ConversationAgentNode:
    def __init__(self):
        self._api_key_file = rospy.get_param("~api_key_file", _default_api_key_file())
        self._api_key = (
            rospy.get_param("~api_key", "")
            or os.getenv("GEMINI_API_KEY", "")
            or _read_secret_file(self._api_key_file)
        )
        self._chat_model = rospy.get_param(
            "~chat_model",
            os.getenv("GEMINI_CHAT_MODEL", "gemini-2.5-flash"),
        )
        self._transcription_model = rospy.get_param(
            "~transcription_model",
            os.getenv("GEMINI_TRANSCRIPTION_MODEL", "gemini-2.5-flash"),
        )
        self._api_timeout_s = float(rospy.get_param("~api_timeout_s", 60.0))
        self._temperature = float(rospy.get_param("~temperature", 0.7))
        self._language = rospy.get_param("~language", "")
        self._listen_duration_s = int(rospy.get_param("~listen_duration_s", 6))
        self._sample_rate = int(rospy.get_param("~sample_rate", 16000))
        self._channels = int(rospy.get_param("~channels", 1))
        self._record_device = rospy.get_param("~record_device", _default_record_device())
        self._silence_rms_threshold = int(rospy.get_param("~silence_rms_threshold", 0))
        self._voice_id = rospy.get_param("~voice_id", "Salli")
        self._aws_region = rospy.get_param("~aws_region", "us-east-1")
        self._opening_prompt = rospy.get_param(
            "~opening_prompt",
            "Hi, I am Flo, a socially assistive robot. How are you doing today?",
        )
        self._reprompt = rospy.get_param(
            "~reprompt",
            "I didn't quite catch that. Please try again.",
        )
        self._closing_prompt = rospy.get_param(
            "~closing_prompt",
            "Okay, talk to you later.",
        )
        configured_face_labels = list(rospy.get_param("~face_labels", DEFAULT_FACE_LABELS))
        faces_json_path = rospy.get_param("~faces_json", "")
        self._face_labels = _load_face_labels(configured_face_labels, faces_json_path)
        self._system_prompt = rospy.get_param(
            "~system_prompt",
            (
                "You are Flo, a friendly humanoid social robot speaking with a person in real time "
                "during a live demo about social robots. "
                "Your audience may be children, adults, teachers, researchers, or other visitors who "
                "are simply curious about what social robots can do. "
                "Speak like a warm, approachable robot guide. "
                "Keep every response short, natural, and easy to say aloud, usually one to three sentences. "
                "Use simple, conversational language and avoid long explanations, bullet points, "
                "special formatting, or sounding like a technical manual. "
                "Focus on friendly interaction, curiosity, and helping the person feel comfortable talking with a robot. "
                "When relevant, briefly mention social robots in a way that is easy for a general audience to understand. "
                "Ask at most one natural follow-up question in a turn, and only when it helps keep the conversation going. "
                "Do not ask multiple questions at once. "
                "This is a brief interaction, so after a few exchanges, or sooner if the conversation naturally pauses, "
                "gently invite the person to play a game of Simon Says with you. "
                "Make that invitation feel like a smooth, upbeat transition from the conversation."
            ),
        )
        self._system_prompt += (
            " Return every answer as compact JSON with exactly two keys: "
            '"text" for the spoken reply and "face_name" for the matching facial expression. '
            f'The "face_name" value must be exactly one of: {", ".join(self._face_labels)}. '
            'Do not wrap the JSON in markdown fences or add extra commentary.'
        )
        self._stop_phrases = {
            phrase.strip().lower()
            for phrase in rospy.get_param(
                "~stop_phrases",
                ["goodbye", "bye", "stop", "quit", "exit"],
            )
            if phrase.strip()
        }
        self._loop_enabled = bool(rospy.get_param("~loop", True))

        self._tts = PollyTTSStream(voice_id=self._voice_id, region_name=self._aws_region)
        self._client = GeminiClient(
            api_key=self._api_key,
            chat_model=self._chat_model,
            transcription_model=self._transcription_model,
            system_prompt=self._system_prompt,
            timeout_s=self._api_timeout_s,
        )
        self._recorder = AudioRecorder(
            sample_rate=self._sample_rate,
            channels=self._channels,
            duration_s=self._listen_duration_s,
            device=self._record_device,
        )
        self._messages: List[Dict[str, str]] = []
        self._speak_lock = threading.Lock()

        self._user_text_pub = rospy.Publisher("/conversation/user_text", String, queue_size=10)
        self._assistant_text_pub = rospy.Publisher("/conversation/assistant_text", String, queue_size=10)
        self._assistant_emotion_pub = rospy.Publisher("/conversation/emotion", Emotion, queue_size=10)
        self._assistant_face_pub = rospy.Publisher("/conversation/face", String, queue_size=10)
        self._state_pub = rospy.Publisher("/conversation/state", String, queue_size=10)
        self._manual_input_sub = rospy.Subscriber(
            "/conversation/input_text", String, self._handle_manual_input, queue_size=10
        )

    @staticmethod
    def _language_service_error_message(exc: Exception) -> Tuple[str, str]:
        message = str(exc).lower()
        if isinstance(exc, ValueError) and "gemini_api_key" in message:
            return (
                "I am missing my Gemini API key right now, so I cannot answer yet.",
                "very_sad",
            )
        if any(token in message for token in ("401", "403", "api key", "permission denied", "unauthorized")):
            return (
                "I am not authenticated with the language service right now.",
                "very_sad",
            )
        if "(429 " in message or " 429 " in message or "resource_exhausted" in message or "quota exceeded" in message:
            return (
                "I have hit my language service usage limit right now.",
                "sad",
            )
        if "(404 " in message or " 404 " in message or "model not found" in message:
            return (
                "My language model configuration does not look right right now.",
                "sad",
            )
        if "(400 " in message or " 400 " in message or "invalid argument" in message or "bad request" in message:
            return (
                "My language request format was rejected just now.",
                "sad",
            )
        if any(token in message for token in ("timed out", "timeout", "temporarily unavailable", "network error")):
            return (
                "I am having trouble reaching the language service right now.",
                "sad",
            )
        return (
            "I ran into a language service error just now.",
            "very_sad",
        )

    def _publish_state(self, state: str):
        self._state_pub.publish(String(data=state))

    def _speak(self, text: str, face_name: str = "neutral"):
        with self._speak_lock:
            self._assistant_text_pub.publish(String(data=text))
            self._assistant_emotion_pub.publish(Emotion(state=_infer_emotion(text)))
            chosen_face = face_name if face_name in self._face_labels else _infer_face_name(text)
            self._assistant_face_pub.publish(String(data=chosen_face))
            self._publish_state("speaking")
            self._tts.speak(text)
            self._publish_state("idle")

    def _generate_reply(self, user_text: str) -> Tuple[str, str]:
        self._messages.append({"role": "user", "content": user_text})
        raw_reply = self._client.chat(self._messages, temperature=self._temperature)
        reply_text, face_name = _parse_structured_reply(raw_reply, self._face_labels)
        self._messages.append({"role": "assistant", "content": reply_text})
        return reply_text, face_name

    def _generate_audio_turn(self, audio_path: str) -> Tuple[str, str, str]:
        transcript, reply_text, face_name = self._client.audio_turn(
            messages=self._messages,
            audio_path=audio_path,
            language=self._language,
            temperature=self._temperature,
            allowed_faces=self._face_labels,
        )
        if transcript:
            self._messages.append({"role": "user", "content": transcript})
        if reply_text:
            self._messages.append({"role": "assistant", "content": reply_text})
        return transcript, reply_text, face_name

    def _handle_manual_input(self, msg: String):
        user_text = msg.data.strip()
        if not user_text:
            return
        self._user_text_pub.publish(String(data=user_text))
        if user_text.lower() in self._stop_phrases:
            self._speak(self._closing_prompt, "neutral")
            rospy.signal_shutdown("Manual stop phrase received")
            return
        try:
            reply_text, face_name = self._generate_reply(user_text)
            self._speak(reply_text, face_name)
        except Exception as exc:
            rospy.logerr("Manual conversation turn failed: %r", exc)
            error_text, face_name = self._language_service_error_message(exc)
            self._speak(error_text, face_name)

    def _listen_once(self) -> Optional[Tuple[str, str, str]]:
        audio_path = ""
        try:
            self._publish_state("listening")
            audio_path = self._recorder.record_to_wav()
            if not _audio_has_speech(audio_path, self._silence_rms_threshold):
                rospy.loginfo(
                    "Skipping transcription because audio stayed below RMS threshold %s",
                    self._silence_rms_threshold,
                )
                return None
            self._publish_state("transcribing")
            transcript, reply_text, face_name = self._generate_audio_turn(audio_path)
            if not transcript or not reply_text:
                return None
            return transcript, reply_text, face_name
        finally:
            self._publish_state("idle")
            if audio_path and os.path.exists(audio_path):
                os.unlink(audio_path)

    def run(self):
        self._speak(self._opening_prompt)
        while not rospy.is_shutdown():
            try:
                turn = self._listen_once()
                if not turn:
                    self._speak(self._reprompt)
                    if not self._loop_enabled:
                        break
                    continue
                transcript, reply_text, face_name = turn

                rospy.loginfo("User said: %s", transcript)
                self._user_text_pub.publish(String(data=transcript))

                if transcript.strip().lower() in self._stop_phrases:
                    self._speak(self._closing_prompt, "neutral")
                    break

                rospy.loginfo("Assistant replied: %s [face=%s]", reply_text, face_name)
                self._speak(reply_text, face_name)
            except subprocess.CalledProcessError as exc:
                rospy.logerr("Audio recording failed: %s", exc)
                self._speak("I am having trouble using the microphone right now.", "sad")
                break
            except Exception as exc:
                rospy.logerr("Gemini API request failed: %r", exc)
                error_text, face_name = self._language_service_error_message(exc)
                self._speak(error_text, face_name)
                break

            if not self._loop_enabled:
                break


def main():
    rospy.init_node("conversation_agent")
    node = ConversationAgentNode()
    node.run()
