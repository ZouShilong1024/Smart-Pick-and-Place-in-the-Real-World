import speech_recognition as sr
import pyaudio
import os
import tempfile
import sys
import numpy as np
from contextlib import contextmanager
from faster_whisper import WhisperModel

@contextmanager
def ignore_stderr():
    try:
        devnull = os.open(os.devnull, os.O_WRONLY)
        old_stderr = os.dup(2)
        sys.stderr.flush()
        os.dup2(devnull, 2)
        os.close(devnull)
        yield
    finally:
        os.dup2(old_stderr, 2)
        os.close(old_stderr)

class VoiceListener:
    def __init__(self, model_path_or_size="small", device="auto"):
        print(f"🔄 正在加载 Whisper 模型: {model_path_or_size} (Device: {device})...")
        try:
            with ignore_stderr():
                self.model = WhisperModel(model_path_or_size, device=device, compute_type="int8")
        except Exception as e:
            print(f"❌ 模型加载失败: {e}")
            print("⚠️ 尝试加载默认 'base' 模型...")
            self.model = WhisperModel("base", device=device, compute_type="int8")
        
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 300
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.8
        
        self.mic_index = self._find_ugreen_device_index()
        self._calibrate_noise()

    def _find_ugreen_device_index(self):
        with ignore_stderr():
            p = pyaudio.PyAudio()
            target_index = None
            keywords = ["UGREEN", "CM379", "USB Audio", "USB PnP"]
            print("🔍 正在扫描音频设备...")
            try:
                info = p.get_host_api_info_by_index(0)
                numdevices = info.get('deviceCount')
                for i in range(0, numdevices):
                    device_info = p.get_device_info_by_host_api_device_index(0, i)
                    if device_info.get('maxInputChannels') > 0:
                        raw_name = device_info.get('name')
                        try:
                            name = raw_name.encode('latin-1').decode('gbk')
                        except:
                            name = raw_name
                        for k in keywords:
                            if k.lower() in name.lower():
                                target_index = i
                                print(f"✅ 锁定设备: [{i}] {name}")
                                return target_index
            except Exception:
                pass
            finally:
                p.terminate()
        
        if target_index is None:
            print("⚠️ 未找到 UGREEN 设备，使用系统默认麦克风")
        return target_index

    def _calibrate_noise(self):
        if self.mic_index is not None:
            print("🔇 正在校准环境噪音 (请保持安静 1 秒)...")
            try:
                with ignore_stderr():
                    with sr.Microphone(device_index=self.mic_index) as source:
                        self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
                print("✅ 校准完成")
            except Exception as e:
                print(f"⚠️ 校准失败: {e}")

    def listen_and_transcribe(self):
        try:
            with ignore_stderr():
                with sr.Microphone(device_index=self.mic_index) as source:
                    print("\n👂 聆听中...", end="\r")
                    audio_data = self.recognizer.listen(source, timeout=1, phrase_time_limit=10)
                    print("⚡ 处理中...   ", end="\r")

        except sr.WaitTimeoutError:
            return ""
        except Exception as e:
            print(f"\n❌ 录音错误: {e}")
            return ""

        temp_wav_path = None
        text_output = ""
        try:
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
                temp_wav.write(audio_data.get_wav_data())
                temp_wav_path = temp_wav.name

            segments, info = self.model.transcribe(temp_wav_path, beam_size=5, language="zh")
            for segment in segments:
                text_output += segment.text

            if text_output:
                print(f"📝 识别结果: {text_output}")
            return text_output

        except Exception as e:
            print(f"\n❌ 识别错误: {e}")
            return ""
        finally:
            if temp_wav_path and os.path.exists(temp_wav_path):
                os.remove(temp_wav_path)

if __name__ == "__main__":
    model_path = "/home/zz/faster-whisper-large-v3"
    
    listener = VoiceListener(model_path_or_size=model_path, device="cuda")

    print("=======================================")
    print("   语音控制系统已启动 (极速优化版)")
    print("=======================================")
    
    while True:
        try:
            result_text = listener.listen_and_transcribe()
            if result_text and len(result_text.strip()) > 0:
                pass
        except KeyboardInterrupt:
            print("\n🛑 程序停止")
            break