"""STS 单圈位置调试：通信、动作文件和定时播放。界面不在这里操作串口。"""
import copy
import json
import math
import time
from pathlib import Path


# 来自 SundayRobot/Src/robot_motion.c；单位为绝对位置计数，不写入舵机 EEPROM。
LIMITS = dict(enumerate([
    (48, 4048), (48, 4048), (48, 4048), (48, 4048),
    (1929, 4048), (48, 2158), (1308, 2962), (1360, 2786),
    (1140, 2768), (1355, 2600), (1881, 3080), (977, 2195),
    (1124, 2408), (1688, 2972),
], 1))


def number(value, low, high, name):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} 必须是 {low}～{high} 的整数")
    return value


def servo_id(value):
    if not str(value).isdigit() or str(int(value)) != str(value):
        raise ValueError(f"无效的舵机 ID：{value}")
    return number(int(value), 0, 253, "舵机 ID")


def new_group():
    return {"version": 1, "name": "我的动作组", "period_ms": 10,
            "limits": {str(i): list(v) for i, v in LIMITS.items()}, "frames": []}


def frame_limits(group, frame):
    limits = dict(group.get("limits", {}))
    limits.update(frame.get("limits", {}))
    return {int(i): tuple(v) for i, v in limits.items()}


def change_limits(group, limits, unify=False):
    """生成校验后的副本：修改的 ID 全帧生效；unify 显式清除所有逐帧覆盖。"""
    if not isinstance(limits, dict):
        raise ValueError("软件限位必须是 ID: [最小值, 最大值] 字典")
    result = copy.deepcopy(group)
    result["limits"] = copy.deepcopy(limits)
    changed = {i for i in set(group["limits"]) | set(limits)
               if group["limits"].get(i) != limits.get(i)}
    for frame in result["frames"]:
        if "limits" in frame:
            if unify:
                del frame["limits"]
            else:
                for i in changed:
                    frame["limits"].pop(i, None)
    # 不裁剪目标、不自动扩大范围；任一目标或采样越界时，整次修改不提交。
    return validate(result, True)


def frame_targets(frame):
    """本帧实际下发的目标。None 与省略的 ID 都沿用舵机之前的目标。"""
    return {int(i): value for i, value in frame["positions"].items() if value is not None}


def servo_timing(frame, i):
    """启动延迟从本帧开始计时；未单独设置的 ID 沿用整帧默认运动时间。"""
    timing = frame.get("timing", {}).get(str(i), {})
    return timing.get("delay_ms", 0), timing.get("duration_ms", frame["duration_ms"])


def frame_duration(frame):
    return max((sum(servo_timing(frame, i)) for i in frame_targets(frame)),
               default=frame["duration_ms"])


def check_positions(positions, limits, label="目标"):
    if not isinstance(positions, dict) or not 1 <= len(positions) <= 30:
        raise ValueError("每帧应包含 1～30 个舵机；SDK 的同步包上限为 250 字节")
    for i, value in positions.items():
        i = servo_id(i)
        number(value, 0, 4095, f"ID{i} {label}")
        if i not in limits:
            raise ValueError(f"请先在动作文件 limits 中设置 ID{i} 的机械安全范围")
        if not limits[i][0] <= value <= limits[i][1]:
            raise ValueError(f"ID{i} {label} {value} 超出软件限位 {limits[i]}")


def validate(group, allow_empty=False):
    if not isinstance(group, dict) or type(group.get("version")) is not int or group["version"] not in (1, 2):
        raise ValueError("动作文件 version 必须为 1 或 2")
    if not isinstance(group.get("name"), str) or not group["name"].strip():
        raise ValueError("动作组名称不能为空")
    number(group.get("period_ms"), 10, 100, "刷新周期 ms")
    if not isinstance(group.get("limits"), dict):
        raise ValueError("动作文件必须包含 limits 软件限位字典")
    if not isinstance(group.get("frames"), list) or len(group["frames"]) > 2000:
        raise ValueError("frames 必须为列表，最多 2000 帧")
    if not group["frames"] and not allow_empty:
        raise ValueError("动作组还没有动作")
    for item in [group] + group["frames"]:
        if not isinstance(item, dict) or not isinstance(item.get("limits", {}), dict):
            raise ValueError("动作和 limits 格式不正确")
        for i, bounds in item.get("limits", {}).items():
            servo_id(i)
            if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
                raise ValueError("limits 的每个值应为 [最小位置, 最大位置]")
            number(bounds[0], 0, 4095, "限位下界")
            number(bounds[1], bounds[0], 4095, "限位上界")
    for index, frame in enumerate(group["frames"]):
        if not isinstance(frame.get("name"), str) or not frame["name"].strip():
            raise ValueError("动作名称不能为空")
        number(frame.get("duration_ms"), 0, 600000, "运动时间 ms")
        number(frame.get("hold_ms", 0), 0, 600000, "停留时间 ms")
        number(frame.get("speed"), 1, 32766, "速度原始值（不使用含义不确定的 0）")
        number(frame.get("acc"), 0, 254, "ACC")
        if type(frame.get("wait_reached", False)) is not bool:
            raise ValueError("wait_reached 必须为 true 或 false")
        if type(frame.get("checked", False)) is not bool:
            raise ValueError("checked 勾选状态必须为 true 或 false")
        limits = frame_limits(group, frame)
        if not isinstance(frame.get("positions"), dict):
            raise ValueError("positions 必须是目标字典；空字典表示本帧全部保持")
        for i in frame["positions"]:
            servo_id(i)  # None 也不能掩盖写错的 ID。
        targets = frame_targets(frame)
        if targets:
            try:
                check_positions(targets, limits)
            except ValueError as exc:
                raise ValueError(f"第{index + 1}帧「{frame['name']}」：{exc}") from exc
        timing = frame.get("timing", {})
        if not isinstance(timing, dict):
            raise ValueError("timing 必须是各 ID 的时序字典")
        if timing and group["version"] != 2:
            raise ValueError("独立舵机时序需要 version=2，请使用新版软件保存")
        if timing and "samples" in frame:
            raise ValueError("固件采样帧不能设置独立时序；请先复制并转为普通帧")
        for i, item in timing.items():
            if not isinstance(i, str) or servo_id(i) not in targets:
                raise ValueError(f"时序 ID{i} 必须在本帧有数值目标，不能是 None 或未填写")
            if not isinstance(item, dict) or set(item) - {"delay_ms", "duration_ms"}:
                raise ValueError(f"ID{i} 时序只支持 delay_ms、duration_ms")
            delay, duration = servo_timing(frame, i)
            number(delay, 0, 600000, f"ID{i} 启动延迟 ms")
            number(duration, 0, 600000, f"ID{i} 运动时间 ms")
            number(delay + duration, 0, 600000, f"ID{i} 计划结束时间 ms")
        if "samples" in frame:
            if not targets or any(value is None for value in frame["positions"].values()):
                raise ValueError("固件采样帧不能使用 None 或空目标；请先转为普通帧再修改")
            samples = frame["samples"]
            ids = sorted(frame["positions"], key=int)
            if not isinstance(samples, list) or not 2 <= len(samples) <= 60001:
                raise ValueError("采样轨迹必须包含 2～60001 个采样点")
            previous = -1
            for sample in samples:
                if not isinstance(sample, list) or len(sample) != 2:
                    raise ValueError("采样格式为 [毫秒, [按 ID 升序排列的位置]]")
                number(sample[0], previous + 1, frame["duration_ms"], "采样时间")
                if not isinstance(sample[1], list) or len(sample[1]) != len(ids):
                    raise ValueError("采样点舵机数量不匹配")
                try:
                    check_positions(dict(zip(ids, sample[1])), limits)
                except ValueError as exc:
                    raise ValueError(f"第{index + 1}帧「{frame['name']}」采样 {sample[0]} ms：{exc}") from exc
                previous = sample[0]
            if samples[0][0] != 0 or samples[-1][0] != frame["duration_ms"]:
                raise ValueError("采样轨迹必须从 0 ms 开始，到动作时间结束")
            if samples[-1][1] != [frame["positions"][i] for i in ids]:
                raise ValueError("采样轨迹终点与目标位置不一致")
    return group


def load_group(path):
    if Path(path).stat().st_size > 20_000_000:
        raise ValueError("动作文件超过 20 MB")
    return validate(json.loads(Path(path).read_text(encoding="utf-8-sig")), True)


def save_group(path, group):
    validate(group, True)
    # 临时文件替换，避免保存中断破坏原动作文件。
    path = Path(path)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(group, ensure_ascii=False, indent=2), encoding="utf-8")
    temp.replace(path)


class ServoBus:
    """只支持 STS 位置模式；没有修改 ID、波特率、中位等 EEPROM 的接口。"""

    def __init__(self, port, baud):
        try:
            from scservo_sdk import PortHandler, sms_sts
        except ImportError as exc:
            raise RuntimeError("缺少 SDK：运行 python -m pip install -r requirements.txt") from exc
        self.port = PortHandler(port)
        self.sdk = sms_sts(self.port)
        self.limits = {}
        try:
            if not self.port.setBaudRate(baud):
                raise RuntimeError(f"无法打开 {port}；请关闭 FD，并检查串口和驱动")
            # 防止拔线/驱动异常使软件停止长时间卡在系统写操作。
            self.port.ser.write_timeout = 0.2
        except Exception:
            if self.port.ser is not None:
                self.port.closePort()
            raise

    def close(self):
        self.port.closePort()

    def check(self, result, error=0):
        if result != 0:
            raise IOError(self.sdk.getTxRxResult(result))
        if error:
            raise IOError(f"舵机报警 0x{error:02X}：{self.sdk.getRxPacketError(error)}")

    def read(self, i, address, size=1):
        value, result, error = (self.sdk.read1ByteTxRx(i, address) if size == 1
                                else self.sdk.read2ByteTxRx(i, address))
        self.check(result, error)
        return value

    def ping(self, i):
        model, result, error = self.sdk.ping(i)
        self.check(result, error)
        return model

    def position(self, i):
        value, result, error = self.sdk.ReadPos(i)
        self.check(result, error)
        return value

    def feedback(self, i):
        data, result, error = self.sdk.readTxRx(i, 56, 15)
        self.check(result, error)
        if len(data) != 15:
            raise IOError(f"ID{i} 反馈长度不正确")
        word = lambda index: data[index] | (data[index + 1] << 8)
        return {"position": self.sdk.scs_tohost(word(0), 15),
                "speed": self.sdk.scs_tohost(word(2), 15),
                "voltage": data[6] / 10, "temperature": data[7],
                "moving": data[10], "torque": self.read(i, 40),
                "mode": self.read(i, 33), "phase": self.read(i, 18)}

    def prepare(self, ids, stop, require_torque=True):
        positions = {}
        for i in ids:
            if stop.is_set():
                raise InterruptedError("已取消")
            if self.read(i, 33) != 0:
                raise ValueError(f"ID{i} 不是位置模式；本程序不会自动修改模式")
            low, high = self.read(i, 9, 2), self.read(i, 11, 2)
            if not 0 <= low < high <= 4095:
                raise ValueError(f"ID{i} 不是有效的单圈角度限制；请先在 FD 中核对")
            self.limits[i] = (low, high)
            if self.read(i, 64):
                raise ValueError(f"ID{i} 存在异步缓存；请在 FD 中处理并重新检查")
            if require_torque and self.read(i, 40) != 1:
                raise ValueError(f"ID{i} 未加载扭矩，请先支撑机器人并点击加载")
            positions[i] = self.position(i)
        return positions

    def write(self, positions, speed, acc, ordinary=False):
        check_positions(positions, {int(i): (0, 4095) for i in positions})
        number(speed, 1, 32766, "速度")
        number(acc, 0, 254, "ACC")
        if ordinary:
            if len(positions) != 1:
                raise ValueError("普通写只发送一只舵机；多舵机请选择同步写")
            i, pos = next(iter(positions.items()))
            self.check(*self.sdk.WritePosEx(i, pos, speed, acc))
            return
        try:
            for i, pos in positions.items():
                if not self.sdk.SyncWritePosEx(i, pos, speed, acc):
                    raise IOError(f"ID{i} 添加到同步包失败")
            self.check(self.sdk.groupSyncWrite.txPacket())
        finally:
            self.sdk.groupSyncWrite.clearParam()
            self.port.is_using = False

    def torque(self, ids, enable, stop):
        if enable:
            positions = self.prepare(ids, stop, False)
            for i, pos in positions.items():
                low, high = self.limits.get(i, (0, 4095))
                if not low <= pos <= high:
                    raise ValueError(f"ID{i} 当前位置超出舵机角度范围，拒绝加载")
            # 先把目标改为当前位置，再加载，防止跳到上次遗留目标。
            for i, pos in positions.items():
                if stop.is_set():
                    raise InterruptedError("已取消加载")
                self.write({i: pos}, 100, 12)
                self.check(*self.sdk.write1ByteTxRx(i, 40, 1))
        else:
            from scservo_sdk import GroupSyncWrite
            # 卸力可用于整张搜索表，分包避免超过 SDK 的包长上限。
            for offset in range(0, len(ids), 31):
                packet = GroupSyncWrite(self.sdk, 40, 1)
                try:
                    for i in ids[offset:offset + 31]:
                        if not packet.addParam(i, [0]):
                            raise IOError(f"ID{i} 卸力包构造失败")
                    self.check(packet.txPacket())
                finally:
                    packet.clearParam()


class DemoBus:
    """离线演示：目标即时跟随，不模拟负载、惯性或行走稳定性。"""

    def __init__(self):
        self.positions = {i: 2048 for i in LIMITS}
        self.enabled = set()
        self.writes = []

    def close(self):
        pass

    def ping(self, i):
        if i not in self.positions:
            raise IOError("无响应")
        return 777

    def position(self, i):
        self.ping(i)
        return self.positions[i]

    def feedback(self, i):
        return {"position": self.position(i), "speed": 0, "voltage": 12.0,
                "temperature": 25, "moving": 0, "torque": int(i in self.enabled),
                "mode": 0, "phase": 4}

    def prepare(self, ids, stop, require_torque=True):
        for i in ids:
            self.ping(i)
            if stop.is_set():
                raise InterruptedError("已取消")
            if require_torque and i not in self.enabled:
                raise ValueError(f"ID{i} 未加载扭矩")
        return {i: self.positions[i] for i in ids}

    def write(self, positions, speed, acc, ordinary=False):
        if ordinary and len(positions) != 1:
            raise ValueError("普通写只能有一个 ID")
        self.writes.append((dict(positions), speed, acc))
        if len(self.writes) > 2000:
            del self.writes[:1000]
        for i, pos in positions.items():
            if i in self.enabled:
                self.positions[i] = pos

    def torque(self, ids, enable, stop):
        for i in ids:
            if enable:
                if stop.is_set():
                    raise InterruptedError("已取消")
                self.enabled.add(i)
            else:
                self.enabled.discard(i)


def stop_motion(bus, ids, unload, log):
    """尽力停止。断线情况下软件无法保证停住；不向未参与运动的 ID 发广播。"""
    if not ids:
        return
    if unload:
        try:
            bus.torque(ids, False, None)
            log("已发送卸力；同步包无回执，请确认实物状态")
        except Exception as exc:
            log(f"卸力失败：{exc}；请使用外部断电措施")
        return
    for i in ids:
        try:
            pos = bus.position(i)
            if not 0 <= pos <= 4095:
                raise ValueError("反馈不在单圈范围")
            bus.write({i: pos}, 100, 12)
        except Exception as exc:
            log(f"ID{i} 保持当前位置失败：{exc}；请检查实物")
    log("已停止后续指令，并尝试保持当前位置；这不是硬件急停")


def recovery_plan(bus, group, stop):
    """只读检查。仅针对单个普通帧，最多接受8计数的反馈越界，绝不放宽目标限位。"""
    validate(group)
    if len(group["frames"]) != 1 or "samples" in group["frames"][0]:
        raise ValueError("起点恢复只用于选中的一个普通关键帧，不用于固件采样或整组动作")
    frame = group["frames"][0]
    targets = frame_targets(frame)
    if not targets:
        return {}  # 保持帧不读取、不恢复任何舵机。
    positions = bus.prepare(sorted(targets), stop)
    limits = frame_limits(group, frame)
    # 软件与舵机自身角度限制取交集，恢复目标必须同时满足二者。
    for i in positions:
        low, high = getattr(bus, "limits", {}).get(i, (0, 4095))
        limits[i] = (max(low, limits[i][0]), min(high, limits[i][1]))
    check_positions(targets, limits)
    plan = {}
    for i, pos in positions.items():
        if stop.is_set():
            raise InterruptedError("已取消恢复检查")
        low, high = limits[i]
        if low <= pos <= high:
            continue
        data = bus.feedback(i)
        pos = number(data["position"], 0, 4095, f"ID{i} 当前反馈")
        if low <= pos <= high:
            continue
        distance = low - pos if pos < low else pos - high
        if distance > 8:
            raise ValueError(f"ID{i} 当前反馈 {pos} 超出范围 {limits[i]} 达 {distance} 计数；"
                             "超过8计数的小幅恢复范围，请停止并检查机构、中位和动作文件")
        if data["moving"] or data["speed"] != 0:
            raise ValueError(f"ID{i} 仍在运动，请等停稳后重新检查，不在运动中切换恢复目标")
        if data["torque"] != 1 or data["mode"] != 0:
            raise ValueError(f"ID{i} 不是已加载的位置模式，不能执行恢复")
        # 不停在边界上：向内留8计数余量，减少跟随误差再次落到范围外的概率。
        target = min(high, low + 8) if pos < low else max(low, high - 8)
        plan[i] = {"position": pos, "target": target, "limits": limits[i],
                   "speed": min(frame["speed"], 50 if data["phase"] & 4 else 1),
                   "acc": min(frame["acc"] or 8, 8)}
    return plan


def recover_start(bus, group, approved, stop, unload, log, clock=time.monotonic):
    """用户确认之后重新检查，只移动获准的小幅越界 ID；结束后不自动播放原动作。"""
    sent = []
    try:
        plan = recovery_plan(bus, group, stop)
        # 弹窗期间可能有外力、参数或反馈变化，不能继续使用过期的恢复许可。
        for i, item in plan.items():
            if (i not in approved or abs(item["position"] - approved[i]["position"]) > 2
                    or any(item[key] != approved[i][key] for key in ("target", "limits", "speed", "acc"))):
                raise ValueError("确认后反馈或限制发生变化，未发送恢复指令；请重新点击恢复起点")
        if not plan:
            log("反馈已经回到范围内，无需恢复；请重新运行选中的普通帧")
            return
        for i, item in plan.items():
            if stop.is_set():
                raise InterruptedError("已停止恢复")
            # 发送前再核对这一个 ID；不允许在确认后扩大恢复距离。
            if abs(bus.position(i) - item["position"]) > 2:
                raise ValueError(f"ID{i} 在发送前位置又发生变化，已中止恢复")
            log(f"起点恢复 ID{i}：反馈 {item['position']} → 范围内目标 {item['target']}，"
                f"速度原值 {item['speed']}，ACC {item['acc']}；原限位不变")
            sent.append(i)  # 写入异常时仍可能已有硬件收到。
            bus.write({i: item["target"]}, item["speed"], item["acc"])
            deadline, stable = clock() + 3, 0
            while True:
                if stop.is_set():
                    raise InterruptedError("已停止恢复")
                data = bus.feedback(i)
                pos = data["position"]
                low, high = item["limits"]
                if data["torque"] != 1 or data["mode"] != 0:
                    raise IOError(f"ID{i} 恢复期间扭矩或模式发生变化，已中止")
                if pos < min(item["position"], item["target"]) - 2 or pos > max(item["position"], item["target"]) + 2:
                    raise IOError(f"ID{i} 反馈超出本次小幅恢复区间，请检查实物，必要时使用卸力停止")
                settled = (low <= pos <= high and abs(pos - item["target"]) <= 4
                           and data["moving"] == 0 and data["speed"] == 0)
                stable = stable + 1 if settled else 0
                if stable >= 3:
                    log(f"ID{i} 恢复确认：反馈 {pos}，已连续3次位于允许范围内")
                    break
                if clock() >= deadline:
                    raise IOError(f"ID{i} 恢复3秒超时，当前反馈 {pos}；未扩大限位，请检查负载和机构")
                stop.wait(0.05)
        log("起点恢复结束；没有播放原动作。请确认姿态后再次点击“仅运行选中普通帧”")
    except Exception:
        if unload.is_set():
            stop_motion(bus, list(approved), True, log)
        else:
            for i in sent:
                try:
                    pos = bus.position(i)
                    if approved[i]["limits"][0] <= pos <= approved[i]["limits"][1]:
                        bus.write({i: pos}, approved[i]["speed"], approved[i]["acc"])
                    else:
                        log(f"ID{i} 仍在范围外，不补发越界保持目标；已发恢复目标仍有效。"
                            "请支撑机构，必要时卸力或外部断电")
                except Exception as exc:
                    log(f"ID{i} 恢复停止确认失败：{exc}；请检查实物")
        raise


def curve(frame, start, elapsed):
    """普通动作五次平滑插值；导出的固件轨迹按相邻采样点插值。"""
    ids = sorted(frame["positions"], key=int)
    if "samples" in frame:
        import bisect
        samples = frame["samples"]
        index = min(len(samples) - 1, bisect.bisect_left(samples, elapsed, key=lambda s: s[0]))
        if index == 0:
            return {int(i): pos for i, pos in zip(ids, samples[0][1])}
        a, b = samples[index - 1], samples[index]
        q = min(1, max(0, (elapsed - a[0]) / (b[0] - a[0])))
        return {int(i): round(x + (y - x) * q) for i, x, y in zip(ids, a[1], b[1])}
    positions = {}
    for i, pos in frame_targets(frame).items():
        delay, duration = servo_timing(frame, i)
        if elapsed < delay:
            continue  # 尚未到启动时刻，连起点也不重发，保留舵机原目标。
        q = min(1, (elapsed - delay) / duration) if duration else 1
        q = q * q * q * (10 + q * (-15 + 6 * q))
        positions[i] = round(start[i] + (pos - start[i]) * q)
    return positions


def play(bus, group, stop, unload, log, repeats=1, ordinary=False,
         clock=time.monotonic, previous_ids=()):
    """单工作线程播放。stop/unload 是 Event，界面只设置它们，不并发使用串口。"""
    group = copy.deepcopy(validate(group))
    number(repeats, 1, 100, "重复次数")
    if repeats != 1 and any("samples" in f for f in group["frames"]):
        raise ValueError("含固件采样的动作组只允许播放一遍；末姿态不能直接循环接回起点")
    ids = sorted({i for f in group["frames"] for i in frame_targets(f)})
    previous_ids = {servo_id(i) for i in previous_ids}
    if len(ids) > 30:
        raise ValueError("一个动作组最多涉及 30 只舵机")
    active = False
    try:
        start = bus.prepare(ids, stop) if ids else {}
        # 在第一包运动前检查整组衔接，防止调整顺序后走到一半才发现采样起点不符。
        predicted = dict(start)
        for _ in range(min(repeats, 2)):
            for frame in group["frames"]:
                if stop.is_set():
                    raise InterruptedError("已停止")
                targets = frame_targets(frame)
                current = {i: predicted[i] for i in targets}
                if current:
                    check_positions(current, frame_limits(group, frame), "起点位置")
                if "samples" in frame:
                    expected = curve(frame, current, 0)
                    if any(abs(current[i] - expected[i]) > 2 for i in current):
                        raise ValueError(f"{frame['name']} 的起点不匹配；请保留原采样顺序及配套预备站姿")
                predicted.update(targets)
        if hasattr(bus, "limits"):
            for frame in group["frames"]:
                if frame_targets(frame):
                    check_positions(frame_targets(frame), bus.limits)
                for _, sample in frame.get("samples", []):
                    check_positions(dict(zip(sorted(frame["positions"], key=int), sample)), bus.limits)
        for repeat in range(repeats):
            for index, frame in enumerate(group["frames"]):
                if stop.is_set():
                    raise InterruptedError("已停止")
                targets = frame_targets(frame)
                current = {i: start[i] for i in targets}
                timing = {i: servo_timing(frame, i) for i in targets}
                duration = frame_duration(frame)
                milestones = sorted({point for delay, length in timing.values() for point in (delay, delay + length)})
                pending = set(targets)
                log(f"第 {repeat + 1}/{repeats} 遍 · {index + 1}/{len(group['frames'])} {frame['name']}")
                if frame.get("timing"):
                    for i in sorted(timing, key=lambda i: (timing[i][0], i)):
                        delay, length = timing[i]
                        log(f"  计划 ID{i}：本帧 +{delay} ms 启动，运动 {length} ms，+{delay + length} ms 结束")
                if not targets:
                    log("本帧全部 None/未填写：保留已有目标与扭矩状态，仅等待，不发送新目标")
                begin = clock()
                previous = begin
                if "samples" in frame:
                    # t=0 是衔接检查点。与原固件一样，到首个正时间采样点才换 ACC 并发包。
                    if stop.wait(min(group["period_ms"], frame["samples"][1][0]) / 1000):
                        raise InterruptedError("已停止")
                while True:
                    if stop.is_set():
                        raise InterruptedError("已停止")
                    now = clock()
                    if now - previous > 0.25:
                        raise IOError("调度延迟超过 250 ms，已中止，避免轨迹突然跳跃")
                    elapsed = min(duration, (now - begin) * 1000)
                    # 浮点误差不能使端点被跳过或在极短等待中空转。
                    for point in milestones + [duration]:
                        if abs(point - elapsed) < 0.000001:
                            elapsed = point
                            break
                    positions = {i: pos for i, pos in curve(frame, current, elapsed).items() if i in pending}
                    if positions:
                        active = True  # 即使发送失败，也可能有部分硬件已经收到。
                        bus.write(positions, frame["speed"], frame["acc"], ordinary)
                        pending.difference_update(i for i in positions if elapsed >= sum(timing[i]))
                    previous = now
                    if elapsed >= duration:
                        break
                    # 按绝对时钟调度，落后则跳过过期点，不积压成一串突发写。
                    passed = (clock() - begin) * 1000
                    due = (math.floor((passed + 0.000001) / group["period_ms"]) + 1) * group["period_ms"]
                    # 在常规定时点之间，额外唤醒到启动/结束边界，不把 15 ms 延迟改成 20 ms。
                    due = min([due, duration] + [point for point in milestones if point > passed + 0.000001])
                    if any(elapsed < point <= passed + 0.000001 for point in milestones):
                        due = passed  # 串口调用跨过了边界，立即补端点，不额外等一个周期。
                    stop.wait(max(0, (due / 1000 + begin) - clock()))
                start.update(targets)
                if frame.get("wait_reached") and targets:
                    deadline = clock() + 5
                    while True:
                        if stop.is_set():
                            raise InterruptedError("已停止")
                        measured = {}
                        for i in targets:
                            if stop.is_set():
                                raise InterruptedError("已停止")
                            measured[i] = bus.position(i)
                        if all(abs(measured[i] - targets[i]) <= 20 for i in targets):
                            break
                        if clock() >= deadline:
                            raise IOError("等待到位超时（误差需 ≤20 计数）；检查速度、供电和负载")
                        stop.wait(0.05)
                if stop.wait(frame.get("hold_ms", 0) / 1000):
                    raise InterruptedError("已停止")
        log("动作组发送完成；最后目标保持，不自动卸力。发送完成不等于实物已到位")
    except Exception:
        if active or (stop.is_set() and (unload.is_set() or previous_ids)):
            # None 帧不会清掉上一项动作的停止范围；正常播放绝不重发这些旧目标。
            stop_motion(bus, sorted(set(ids) | previous_ids), unload.is_set(), log)
        raise
