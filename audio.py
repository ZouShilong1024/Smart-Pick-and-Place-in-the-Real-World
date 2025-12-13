import speech_recognition as sr
import pyaudio
import os
import tempfile
import sys
from contextlib import contextmanager
from faster_whisper import WhisperModel

@contextmanager
def ignore_stderr():
    """
    一个上下文管理器，用于暂时屏蔽 C 语言底层的 stderr 输出。
    主要用于解决 PyAudio 加载时 ALSA 打印大量警告的问题。
    """
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
        """
        初始化监听模块
        :param model_size: 模型大小，推荐 'tiny', 'base', 'small' (small 效果最好，base 速度最快且够用)
        :param device: 'auto', 'cuda' (N卡), or 'cpu'
        """
        # print(f"正在加载 Whisper 模型: {model_path_or_size} ...")
        try:
            with ignore_stderr():
                self.model = WhisperModel(model_path_or_size, device=device, compute_type="int8")
        except Exception as e:
            print(f"模型加载失败: {e}")
            print("尝试加载默认 'base' 模型...")
            self.model = WhisperModel("base", device=device, compute_type="int8")
        
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 300
        self.recognizer.dynamic_energy_threshold = True
        
        self.mic_index = self._find_ugreen_device_index()

    def _find_ugreen_device_index(self):
        """增强版查找 UGREEN 设备 (解决乱码问题)"""
        with ignore_stderr():
            p = pyaudio.PyAudio()
            target_index = None
            # 关键词列表
            keywords = ["UGREEN", "CM379", "USB Audio", "USB PnP"]
            print("正在扫描音频设备...")
            try:
                info = p.get_host_api_info_by_index(0)
                numdevices = info.get('deviceCount')
                for i in range(0, numdevices):
                    device_info = p.get_device_info_by_host_api_device_index(0, i)
                    if device_info.get('maxInputChannels') > 0:
                        raw_name = device_info.get('name')
                        # 尝试修复中文乱码
                        try:
                            name = raw_name.encode('latin-1').decode('gbk')
                        except:
                            name = raw_name
                        # 关键词匹配
                        for k in keywords:
                            if k.lower() in name.lower():
                                target_index = i
                                print(f"✅ 成功锁定设备: [{i}] {name}")
                                return target_index
            except Exception as e:
                print(f"设备扫描出错: {e}")
            finally:
                p.terminate()
                
        if target_index is not None:
            print(f"\n✅ 成功锁定设备 ID: [{target_index}]")
        else:
            print("\n⚠️ 未找到 UGREEN 设备，将使用系统默认麦克风")
            
        return target_index

    def listen_and_transcribe(self):
        """
        监听并进行语音转文字
        :return: 识别出的文本字符串 (如果没有识别到则返回 None 或空字符串)
        """
        try:
            with ignore_stderr():
                with sr.Microphone(device_index=self.mic_index) as source:
                    print("\n👂 正在监听... (请下达指令)")
                    self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                    
                    audio_data = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)
                    print("⏹️ 检测到语音，正在转录...")

        except sr.WaitTimeoutError:
            return ""
        except Exception as e:
            print(f"❌ 录音错误: {e}")
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
            print(f"❌ 识别错误: {e}")
            return ""
        finally:
            if temp_wav_path and os.path.exists(temp_wav_path):
                os.remove(temp_wav_path)

if __name__ == "__main__":
    model_path = "/media/zsl/zsl_disk/faster-whisper-large-v3"
    
    listener = VoiceListener(model_path_or_size=model_path, device="cpu")

    print("=======================================")
    print("   语音控制系统已启动")
    print("   流程: 监听 -> 识别 -> 机器人执行 -> 监听")
    print("=======================================")
    while True:
        try:
            result_text = listener.listen_and_transcribe()
            if result_text and len(result_text.strip()) > 0:
                # C. 执行机器人操作
                # D. 操作完成后，循环回到开头，再次调用 listen_and_transcribe
                print("🔄 准备下一轮监听...")
            else:
                # 如果只是超时没听到声音，或者识别为空，直接通过，继续监听
                pass
        except KeyboardInterrupt:
            print("\n程序已手动停止。")
            break