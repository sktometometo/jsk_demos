#!/usr/bin/env python

import os
import tempfile
import wave
from typing import Optional

import pyaudio
import requests
import rospy
import webrtcvad
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

# OpenAIのAPIキーを設定
global API_KEY
API_KEY = "your-api-key-here"  # ここにあなたのAPIキーを入力してください

# 録音設定
FORMAT = pyaudio.paInt16
CHANNELS = 1  # モノラル
RATE = 16000  # サンプリングレート（Whisperとwebrtcvadは16kHzを推奨）
FRAME_DURATION = 30  # フレームの長さ（ミリ秒）：10, 20, または30のみ
FRAME_SIZE = int(RATE * FRAME_DURATION / 1000)  # フレームサイズ（サンプル数）
SILENCE_LIMIT = 1  # 無音が続く時間（秒）
WAVE_OUTPUT_FILENAME = "output.wav"

global pub, pub_led


def record_audio() -> Optional[str]:
    vad = webrtcvad.Vad(2)  # 0から3までの攻撃性レベル（0が最も低く、3が最も高い）
    p = pyaudio.PyAudio()

    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=FRAME_SIZE,
    )

    rospy.loginfo("* 音声を録音しています...（無音が続くと自動的に停止します）")

    frames = []
    num_silent_frames = 0
    started = False

    try:
        while True:
            data = stream.read(FRAME_SIZE)
            is_speech = vad.is_speech(data, RATE)

            if not started:
                if is_speech:
                    started = True
                    rospy.loginfo("== 音声検出開始 ==")
                    frames.append(data)
            else:
                frames.append(data)
                if not is_speech:
                    num_silent_frames += 1
                else:
                    num_silent_frames = 0

                if num_silent_frames > (SILENCE_LIMIT * 1000 / FRAME_DURATION):
                    rospy.loginfo("== 無音が続いたため録音を停止します ==")
                    break

    except Exception as e:
        rospy.logerr(f"録音中にエラーが発生しました: {e}")
        return None
    finally:
        rospy.loginfo("* 録音が完了しました")
        stream.stop_stream()
        stream.close()
        p.terminate()

    # wave を 一時ファイルに保存
    filename = tempfile.mktemp(suffix=".wav")
    wf = wave.open(filename, "wb")
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b"".join(frames))
    wf.close()
    return filename


def send_to_whisper(filename: str, api_key: str) -> Optional[str]:
    url = "https://api.openai.com/v1/audio/transcriptions"

    headers = {
        "Authorization": f"Bearer {api_key}",
    }

    with open(filename, "rb") as f:
        files = {
            "file": (filename, f, "audio/wav"),
        }
        data = {
            "model": "whisper-1",
            "language": "en",
        }
        response = requests.post(url, headers=headers, files=files, data=data)

    if response.status_code == 200:
        result = response.json()
        rospy.loginfo("* 文字起こし結果: {}".format(result))
        return result.get("text", None)
    else:
        rospy.logerr(
            f"リクエストが失敗しました。ステータスコード: {response.status_code}"
        )
        return None


def service_callback(req: TriggerRequest) -> TriggerResponse:
    global pub, API_KEY
    pub_led.publish(ColorRGBA(r=1.0, g=1.0, b=1.0))
    filename = record_audio()
    if not filename:
        pub_led.publish(ColorRGBA(r=1.0, g=0.0, b=0.0))
        rospy.logerr("録音に失敗しました")
        res = TriggerResponse(success=False, message="録音に失敗しました")
        return res
    text = send_to_whisper(filename=filename, api_key=API_KEY)
    # delete temporary file
    if filename:
        rospy.loginfo(f"* 一時ファイル {filename} を削除しました")
        os.remove(filename)
    if text:
        pub_led.publish(ColorRGBA(r=0.0, g=1.0, b=0.0))
        candidates = SpeechRecognitionCandidates()
        candidates.transcript.append(text)
        pub.publish(candidates)
        res = TriggerResponse(success=True, message="文字起こしに成功しました")
        return res
    else:
        pub_led.publish(ColorRGBA(r=1.0, g=0.0, b=0.0))
        rospy.logerr("文字起こしに失敗しました")
        res = TriggerResponse(success=False, message="文字起こしに失敗しました")
        return res


if __name__ == "__main__":
    rospy.init_node("lightweight_speech_recognition", anonymous=True)

    API_KEY = rospy.get_param("~api_key")

    pub = rospy.Publisher("/speech_to_text", SpeechRecognitionCandidates, queue_size=1)
    pub_led = rospy.Publisher(
        "/smart_device_protocol/led_color", ColorRGBA, queue_size=1
    )
    srv = rospy.Service(
        "/run_lightweight_speech_recognition", Trigger, service_callback
    )
    pub_led.publish(ColorRGBA(r=0.0, g=1.0, b=0.0))
    rospy.spin()
