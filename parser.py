import re

class CommandParser:
    def __init__(self):
        self.objects = {
            "香蕉": "banana", "苹果": "apple", "橘子": "orange", "盒子": "black box",
            "水杯": "cup", "遥控器": "remote", "瓶子": "bottle", "水果": "fruit,orange,lemon"
        }
        self.containers = {
            "蓝色盘子": "blue plate", "粉色盘子": "pink plate", "粉色的盘子": "pink plate", "篮子": "basket", "箱子": "box", "绿色的碗": "green bowl",
            "绿色碗": "green bowl", "桌子": "desk", "碗": "bowl"
        }
        self.directions = {
            "左": "left", "左边": "left", "左侧": "left",
            "右": "right", "右边": "right", "右侧": "right",
            "中": "middle", "中间": "middle",
            "前": "front", "前面": "front",
            "后": "back", "后面": "back"
        }

    def parse(self, text):
        if not text:
            return None
        result = {
            "object": None,      # 抓什么
            "container": None,   # 放哪里
            "direction": None,   # 哪一边的容器
            "original_text": text
        }
        clean_text = re.sub(r"[，。！？,.]", " ", text)

        for obj, code in self.objects.items():
            if obj in clean_text:
                result["object"] = code
                break

        for container, code in self.containers.items():
            if container in clean_text:
                result["container"] = code
                break

        found_direction_key = None
        for direction_word, direction_code in self.directions.items():
            if direction_word in clean_text:
                found_direction_key = direction_word
                result["direction"] = direction_code
                break

        if result["object"] and result["container"] and found_direction_key:
            obj_idx = clean_text.find(result["object"])
            cont_idx = clean_text.find(result["container"])
            dir_idx = clean_text.find(found_direction_key)

            # 场景 A: "把左边的香蕉放到盘子里" (方位修饰物体)
            # 场景 B: "把香蕉放到右边的盘子里" (方位修饰容器)
            # 简单的距离判定：看方位词离哪个名词更近
            dist_to_obj = abs(dir_idx - obj_idx)
            dist_to_cont = abs(dir_idx - cont_idx)

            if dist_to_obj < dist_to_cont:
                print(f"⚠️ 提示: 识别到的方位 '{found_direction_key}' 似乎是修饰物体 '{result['object']}' 的")
            else:
                pass
        return result
    
if __name__ == "__main__":
    parser = CommandParser()
    test_sentences = [
        "帮我将香蕉放到右边的盘子里面",           # 标准语序
        "把那个苹果拿去放在左侧的篮子里",         # 不同的动词和方位词
        "把手机放到桌子上，右边的那个",           # 倒装/补充说明
        "帮我把橘子扔到箱子里",                  # 无方位词
        "请将水杯放到中间的盘子",                 # 中间
        "这里有个香蕉，把它放到右边的盘子",       # 废话较多
    ]
    print(f"{'指令文本':<30} | {'物体':<6} | {'方位':<6} | {'容器':<6}")
    print("-" * 60)
    for text in test_sentences:
        res = parser.parse(text)
        obj = res['object'] if res['object'] else "❌"
        direction = res['direction'] if res['direction'] else "--"
        container = res['container'] if res['container'] else "❌"
        print(f"{text:<30} | {obj:<6} | {direction:<6} | {container:<6}")