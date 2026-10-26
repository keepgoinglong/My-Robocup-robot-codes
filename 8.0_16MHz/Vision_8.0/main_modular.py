"""DNK230D lane following, based on the user's local experiment 7 main.py.

Platform initialization/display/exit flow adapted from ALIENTEK V1.0,
2024-09-12, Copyright (c) 2020-2032 Guangzhou Xingyi Electronic Technology.
Use canmv_line_follow_single.py in the IDE for a self-contained program.
"""
import gc
import os
import time
import sys
import image
import machine
from machine import UART, FPIOA
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager
from boundary_tracker import BoundaryTracker as LaneTracker, resolve_control
from motion_feedback import MotionFeedback
from lane_protocol import encode, encode_angle, encode_obstacle
from angle_tracker import AngleTracker
from shape_tracker import CardRecognizer, ShapeEvents, board_candidates
from obstacle_tracker import ObstacleTracker

WIDTH, HEIGHT = 160, 120
# 与用户提供的实验7一致；不额外申请未经板上确认的摄像头模式。
BLACK_MAX = 70
ROTATE_180 = True
PREVIEW = True           # IDE虚拟显示，保留调参画面。
DISPLAY_FPS = 30         # 与已稳定运行的官方摄像头例程一致。
PREVIEW_PERIOD_MS = 200  # 5Hz调参画面，降低USB/IDE图传负载。
IDE_QUALITY = 50         # 降低虚拟显示压缩流量，减少调试口掉线概率。
SENSOR_MODE_WIDTH = 1920 # GC2093使用低帧率模式，避免1280x960@90挤满4个VB缓冲。
SENSOR_MODE_HEIGHT = 1080
BINARY_VISIBLE = False # Grayscale preview; no full-frame binary conversion.
BINARY_DETECT = False  # Raw grayscale thresholding avoids full VGA binary cost.
BINARY_INVERT = True
COUNTERCLOCKWISE = True
RIGHT_RECOVER_ENTER = 0.30  # Lane 24px right of center in 160px image.
RIGHT_RECOVER_EXIT = 0.14
PRINT_STATS = True       # One line/sec; includes capture + processing + UART.
UART_TX_PIN = 40      # Set actual K230D UART1 TX GPIO to enable output.
UART_RX_PIN = 41      # PORT1 UART1 RX，启用主板READY/BUSY/DONE反馈。
UART_BAUD = 115200
UART_PERIOD_MS = 50     # 20 Hz independent of camera frame rate.
CAPTURE_RETRIES = 30    # 约3秒为一组；不因瞬时抖动退出。
CAPTURE_RETRY_MS = 100  # 失败期间降低重试频率，给CSI2/缓冲池恢复时间。
CAMERA_REBUILD = False  # 此固件原地重建会卡在sensor.run；失败后安全退出。
CAMERA_RESTART_LIMIT = 3 # 仅供明确开启CAMERA_REBUILD后的实验使用。
AUTO_REBOOT_ON_CAMERA_FAILURE = True
CAMERA_REBOOT_FAILURES = 50 # 连续50次失败后复位；snapshot自身等待使实际时间可能超过5秒。
CAMERA_WARMUP_FRAMES = 3 # run后立即取帧排空VB；不能空等到4个缓冲被占满。
CAMERA_WARMUP_RETRIES = 10 # 每个预热帧最多尝试10次；瞬时failed(3)不直接退出。
CAMERA_PREPARE_MS = 500 # MediaManager建立缓冲池后再启动SENSOR2。
LANE_PERIOD_MS = 40 # 巡线最高25Hz；取帧不降速，减少重复扫描。
ANGLE_ENTER_DEG = 8.0
ANGLE_EXIT_DEG = 4.0
ANGLE_CONFIRM_SECONDS = .20
ANGLE_ZERO_DEG = 0.0 # 未标定，不用直走录像中位数自动归零。
SHAPE_PERIOD_MS = 100   # 最高10Hz；完整卡片可见时间短，实际频率以板端日志为准。
SHAPE_DETECT = True     # 开启六种图形识别；巡线仍独立高频运行。
OBSTACLE_PERIOD_MS = 100 # 10Hz独立横杆检测；绝不改写图形识别结果。
BUILD_TAG = "VISION_7.0_OBSTACLE"


def native_runs(img, y):
    blobs = img.find_blobs([(0, BLACK_MAX)], roi=(0, y, WIDTH, 1),
                           x_stride=1, y_stride=1, pixels_threshold=2,
                           area_threshold=2, merge=False)
    return sorted(b.x() + (b.w()-1)*0.5 for b in blobs
                  if max(2, WIDTH // 100) <= b.w() <= WIDTH*0.12)


def is_capture_error(exc):
    return isinstance(exc,RuntimeError) and "snapshot" in str(exc) and "failed(3)" in str(exc)


def capture_warmup(sensor):
    for frame in range(CAMERA_WARMUP_FRAMES):
        for attempt in range(CAMERA_WARMUP_RETRIES):
            os.exitpoint()
            try:
                warmup=sensor.snapshot()
                # 不保留上一个VB帧再请求下一个，及时释放采集缓冲。
                del warmup
                gc.collect()
                break
            except RuntimeError as exc:
                if not is_capture_error(exc):raise
                if attempt==0:print("CAMERA WARMUP retry frame",frame+1,exc)
                if attempt+1==CAMERA_WARMUP_RETRIES:raise
                gc.collect()
                time.sleep_ms(CAPTURE_RETRY_MS)


def reset_failed_camera():
    print("CAMERA recovery: persistent capture failure; resetting board")
    time.sleep_ms(100)
    machine.reset()
    # reset成功不返回；异常返回也不陷入无提示的无限等待。
    raise RuntimeError("Camera board reset unexpectedly returned")


def open_camera():
    """创建全新采集链路；部分初始化失败也按顺序释放。"""
    sensor = None
    sensor_running = display_inited = media_inited = False
    try:
        sensor = Sensor(width=SENSOR_MODE_WIDTH, height=SENSOR_MODE_HEIGHT)
        sensor.reset()
        sensor.set_framesize(Sensor.VGA)
        sensor.set_pixformat(Sensor.GRAYSCALE)
        # One capture only: sample every fourth pixel for the 160x120 grid.
        # Both flips together = 180 degrees, not a left/right-only mirror.
        sensor.set_hmirror(ROTATE_180)
        sensor.set_vflip(ROTATE_180)
        if PREVIEW:
            # Follow the board's one-minute-stable official camera example.
            # On firmware 0.4.0, VIRT may retain shared VB capture buffers.
            Display.init(Display.ST7701, width=640, height=480,
                         fps=DISPLAY_FPS, to_ide=True, quality=IDE_QUALITY)
            display_inited = True
        MediaManager.init()
        media_inited = True
        gc.collect()
        time.sleep_ms(CAMERA_PREPARE_MS)
        sensor_running = True
        sensor.run()
        # run后立即取帧，不让VB缓冲在第一个消费者前积压。
        capture_warmup(sensor)
        print("CAMERA WARMUP OK", CAMERA_WARMUP_FRAMES)
        return sensor
    except BaseException as exc:
        close_camera(sensor, sensor_running, display_inited, media_inited)
        if is_capture_error(exc) and AUTO_REBOOT_ON_CAMERA_FAILURE:
            reset_failed_camera()
        raise


def close_camera(sensor, sensor_running=True, display_inited=None, media_inited=True):
    # stop 在 MediaManager.deinit 之前；不能只对已 stop 的旧对象调用 run。
    if sensor_running and sensor is not None:
        try:
            sensor.stop()
        except Exception as exc:
            print("Camera stop:", exc)
    if display_inited is None:display_inited=PREVIEW
    if display_inited:
        try:
            Display.deinit()
        except Exception as exc:
            print("Camera display cleanup:", exc)
    gc.collect()
    time.sleep_ms(100)
    if media_inited:
        try:MediaManager.deinit()
        except Exception as exc:print("Camera media cleanup:",exc)
    gc.collect()
    time.sleep_ms(200)


def main():
    sensor = uart = None
    display_ready = media_ready = False
    print("VISION BUILD",BUILD_TAG,"UART RX",UART_RX_PIN)
    tracker = LaneTracker(threshold=0 if BINARY_DETECT else BLACK_MAX,
                          counterclockwise=COUNTERCLOCKWISE,
                          right_recover_enter=RIGHT_RECOVER_ENTER,
                          right_recover_exit=RIGHT_RECOVER_EXIT)
    angle_tracker = AngleTracker(enter=ANGLE_ENTER_DEG, leave=ANGLE_EXIT_DEG,
                                 confirm=ANGLE_CONFIRM_SECONDS, zero=ANGLE_ZERO_DEG)
    feedback = MotionFeedback(enabled=UART_RX_PIN is not None, continuous_control=True)
    result = dict(valid=False,stop=True,state="LOST",confidence=0,degraded=True,offset=0,heading=0,curvature=0,steer=0,mode="LOST",angle_source="NONE",position_direction=0)
    angle_result = dict(valid=False,active=False,direction=0,angle=0.0,status=0,reason="NO_EDGE")
    raw_angle_result = dict(angle_result)
    vision_result = dict(result)
    vision_angle_result = dict(angle_result)
    sequence = 0
    uart_tx_count = 0
    uart_last_tx_seq = None
    recognizer = CardRecognizer()
    shape_events = ShapeEvents()
    obstacle_tracker = ObstacleTracker(confirm=3,clear=3)
    obstacle_result = dict(present=False,angle=0.0,detection=None)
    shape_detection = None
    pending_shapes = []
    sent_shapes = {}
    try:
        # Start SENSOR2 first. UART/FPIOA setup is intentionally delayed until
        # the CSI2 pipeline is running, avoiding startup resource/pinmux races.
        sensor = open_camera()
        display_ready = PREVIEW
        media_ready = True
        if UART_TX_PIN is not None:
            mux = FPIOA()
            mux.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
            if UART_RX_PIN is not None:
                mux.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
            uart = UART(UART.UART1, baudrate=UART_BAUD, bits=UART.EIGHTBITS,
                        parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        else:
            print("UART disabled: configure UART_TX_PIN for your board.")
        clock = time.clock()
        last_tick = time.ticks_ms()
        last_send = last_preview = last_stats = last_tick
        last_shape = last_obstacle = last_tick
        last_lane = None
        timestamp = 0.0
        frames = 0
        processing_us = 0
        lane_us = shape_us = shape_calls = shape_max_us = 0
        shape_max_id = 0
        display_fps = 0.0
        preview_hold = None  # Display may consume this copy asynchronously.
        capture_failures = 0
        consecutive_capture_failures = 0
        camera_restarts = stable_frames = 0
        while True:
            os.exitpoint()
            clock.tick()
            # 丢掉上一帧采集引用再取新帧；显示保留的是独立preview副本。
            img=None
            try:
                # Match the one-minute-stable official example exactly. The
                # default is channel 0; avoid the explicit chn path on 0.4.0.
                img = sensor.snapshot()
            except RuntimeError as exc:
                if not is_capture_error(exc):
                    raise
                angle_tracker.reset()
                last_lane = None
                stable_frames = 0
                capture_failures += 1
                consecutive_capture_failures += 1
                # Do not flood the IDE serial link while CSI2 is unavailable.
                if capture_failures in (1,CAPTURE_RETRIES):
                    print("CAPTURE retry %d/%d: %s" % (capture_failures, CAPTURE_RETRIES, exc))
                tracker = LaneTracker(threshold=0 if BINARY_DETECT else BLACK_MAX,
                                      counterclockwise=COUNTERCLOCKWISE,
                                      right_recover_enter=RIGHT_RECOVER_ENTER,
                                      right_recover_exit=RIGHT_RECOVER_EXIT)
                if uart is not None:
                    uart.write(encode(dict(valid=False, stop=True, state="LOST"), sequence))
                    uart.write(encode_angle(dict(valid=False, active=False, angle=0), sequence))
                    sequence = (sequence+1) & 255
                # Do not allocate/show a warning frame here: Display may still
                # retain VB buffers and make capture recovery harder. Keep the
                # last valid IDE preview and report progress in the terminal.
                gc.collect()
                if (AUTO_REBOOT_ON_CAMERA_FAILURE and
                        consecutive_capture_failures >= CAMERA_REBOOT_FAILURES):
                    reset_failed_camera()
                if capture_failures >= CAPTURE_RETRIES:
                    if not CAMERA_REBUILD:
                        print("CAMERA still unavailable; keep retrying without rebuild")
                        capture_failures = 0
                        time.sleep_ms(CAPTURE_RETRY_MS)
                        continue
                    if camera_restarts >= CAMERA_RESTART_LIMIT:
                        print("CAMERA recovery exhausted; check power/cable/firmware")
                        raise
                    camera_restarts += 1
                    print("CAMERA rebuild %d/%d" % (camera_restarts, CAMERA_RESTART_LIMIT))
                    close_camera(sensor)
                    sensor = None
                    display_ready = media_ready = False
                    sensor = open_camera()
                    display_ready = PREVIEW
                    media_ready = True
                    capture_failures = 0
                    # 保留图形冷却、待发送队列与串口序号。
                time.sleep_ms(CAPTURE_RETRY_MS)
                continue
            capture_failures = 0
            consecutive_capture_failures = 0
            stable_frames += 1
            if stable_frames >= 30:
                camera_restarts = 0
            now = time.ticks_ms()
            delta = time.ticks_diff(now, last_tick)
            if delta <= 0:
                continue
            timestamp += delta / 1000.0  # Accumulation survives ticks wraparound.
            last_tick = now
            before = time.ticks_us()
            if BINARY_DETECT:
                # Selected dark pixels stay 0, background becomes 255.
                img.binary([(0, BLACK_MAX)], invert=True)
            if uart is not None and UART_RX_PIN is not None:
                if uart.any():
                    feedback.feed(uart.read(min(uart.any(),64)),timestamp)
                if feedback.reset_heading:
                    angle_tracker.reset()
                    feedback.reset_heading=False
                    last_lane=None
            lane_before = time.ticks_us()
            if last_lane is None or time.ticks_diff(now,last_lane)>=LANE_PERIOD_MS:
                last_lane=now
                lane_result=tracker.update(lambda x,y:img.get_pixel(x*4,y*4),WIDTH,HEIGHT,timestamp)
                raw_angle_result=angle_tracker.update(tracker.raw_image_angle,timestamp)
                result,angle_result=resolve_control(lane_result,raw_angle_result)
                if angle_result['active'] and angle_result['reason']=='CURVE':
                    tracker.remember_turn(angle_result['direction'],timestamp,
                                          angle_result['angle'])
                # Position recovery suppresses stale opposite heading evidence.
                if angle_result['reason']=='POSITION':angle_tracker.reset()
                vision_result,vision_angle_result=result,angle_result
            # Keep the raw visual decision separate from the UART feedback gate.
            result,angle_result=feedback.gate(vision_result,vision_angle_result,timestamp)
            lane_us += time.ticks_diff(time.ticks_us(), lane_before)
            # 使用原始灰度帧，独立多阈值检测，不再跟随巡线阈值。
            if SHAPE_DETECT and time.ticks_diff(now, last_shape) >= SHAPE_PERIOD_MS:
                last_shape = now
                shape_before = time.ticks_us()
                boxes = board_candidates(img)
                shape_detection = recognizer.detect(img.get_pixel, boxes)
                event = shape_events.update(shape_detection, timestamp)
                if event and event not in pending_shapes:
                    pending_shapes.append(event)
                    sent_shapes.pop(event, None)
                    print("SHAPE %d CONFIRMED score %.3f" % (event, shape_detection['score']))
                one_shape_us = time.ticks_diff(time.ticks_us(), shape_before)
                shape_us += one_shape_us
                shape_calls += 1
                if one_shape_us > shape_max_us:
                    shape_max_us = one_shape_us
                    shape_max_id = 0
                del boxes
            # 障碍检测读取同一张原始灰度图，但与图形分类没有共享阈值、候选或状态。
            if time.ticks_diff(now,last_obstacle)>=OBSTACLE_PERIOD_MS:
                last_obstacle=now
                obstacle_result=obstacle_tracker.update(obstacle_tracker.detect(img))
            processing_us += time.ticks_diff(time.ticks_us(), before)
            frames += 1
            if uart is not None and time.ticks_diff(now, last_send) >= UART_PERIOD_MS:
                shape_event = pending_shapes[0] if pending_shapes else 0
                packet = encode(result, sequence, shape_id=shape_event)
                if uart.write(packet) != len(packet):
                    raise OSError("UART incomplete write")
                # 旧图形包成功后立即记账，附加角度包异常也不会重发图形。
                sequence = (sequence + 1) & 255
                last_send = now
                if shape_event:
                    sent_shapes[shape_event] = True
                    shape_events.mark_sent(shape_event, timestamp + time.ticks_diff(time.ticks_ms(), now)/1000.0)
                    pending_shapes.pop(0)  # Each queued event occupies one packet.
                angle_packet = encode_angle(angle_result, (sequence-1)&255)
                if uart.write(angle_packet) != len(angle_packet):
                    raise OSError("UART angle incomplete write")
                obstacle_packet=encode_obstacle(obstacle_result,(sequence-1)&255)
                if uart.write(obstacle_packet)!=len(obstacle_packet):
                    raise OSError("UART obstacle incomplete write")
                uart_tx_count += 3
                uart_last_tx_seq = (sequence-1)&255
                feedback.sent(angle_result,(sequence-1)&255,timestamp)
            # Future shape recognition goes here, BEFORE drawing debug overlays.
            # Schedule it at a lower rate; do not run full-frame contours every frame.
            if PREVIEW and time.ticks_diff(now, last_preview) >= PREVIEW_PERIOD_MS:
                # Never hand a Sensor/VB capture frame to asynchronous Display.
                # Retain only the newest heap copy until the next preview.
                preview_img = img.copy()
                if BINARY_VISIBLE and not BINARY_DETECT:
                    preview_img.binary([(0, BLACK_MAX)], invert=BINARY_INVERT)
                for y, left, right in tracker.rows:
                    preview_img.draw_cross(int(left*4), y*4, color=127, size=4)
                    preview_img.draw_cross(int(right*4), y*4, color=127, size=4)
                    preview_img.draw_cross(int((left+right)*2), y*4, color=127, size=6)
                state = vision_result["state"] if vision_result["valid"] else "LOST"
                labels = {"STRAIGHT": "直走", "LEFT": "向左修正",
                          "RIGHT": "向右修正", "LOST": "边界不确定"}
                preview_img.draw_rectangle(0, 0, 640, 124, color=0, fill=True)
                preview_img.draw_string_advanced(8, 2, 28, "视觉判定：" + labels[state] + "  " + BUILD_TAG,
                                                 color=(255, 255, 255))
                mode = {"BOTH":"双边","SINGLE_LEFT":"左单边","SINGLE_RIGHT":"右单边","UNCERTAIN":"边界不确定","LOST":"未找到边界"}.get(vision_result["mode"],"巡线")
                if shape_detection is not None:
                    sid = shape_detection['shape']
                    mode = "图形%d 分数%.2f / %s" % (sid, shape_detection['score'],
                              "已发送" if sent_shapes.get(sid) else "连续确认中")
                    bx,by,bw,bh = shape_detection['box']
                    preview_img.draw_rectangle(bx,by,bw,bh,color=255,thickness=2)
                elif pending_shapes:
                    mode = "图形%d已确认 / 等待串口发送" % pending_shapes[0]
                if obstacle_result['present']:
                    mode += " / 障碍角%+.1f度" % obstacle_result['angle']
                road = "左弯" if vision_result.get('road_shape')=='LEFT_CURVE' else "直线"
                mode += " / %s C:%+.3f H:%+.3f" % (road,vision_result.get('curvature',0),vision_result.get('heading',0))
                preview_img.draw_string_advanced(8, 38, 21, mode,
                                                 color=(255, 255, 255))
                if angle_result['reason']=='POSITION':
                    angle_text='下发：位置避边 / '+('向右' if angle_result['direction']>0 else '向左')
                elif angle_result['reason']=='TURN_MEMORY':
                    angle_text='下发：保持已确认左转'
                elif angle_result['reason']=='LOSS_HOLD':
                    angle_text='下发：短时丢线 / 保持左转'
                elif angle_result['reason']=='WAIT_BOARD':
                    angle_text='下发：等待主板完成'
                elif raw_angle_result['valid']:
                    angle_text='下发：%s / 偏角%+.1f度' % (labels[result['state']],raw_angle_result['angle'])
                else:
                    angle_text='下发：单边位置判断' if result['valid'] else '下发：边界证据不足'
                angle_text += "  FPS:%.1f" % display_fps
                preview_img.draw_string_advanced(8,68,20,angle_text,color=(255,255,255))
                preview_img.draw_rectangle(0,442,640,38,color=0,fill=True)
                preview_img.draw_string_advanced(8,446,21,"图形：独立灰度 / 外框校正 / 连续确认",color=(255,255,255))
                link=feedback.link_status(timestamp)
                rx_age='--' if feedback.last_link_rx is None else '%.1fs'%(timestamp-feedback.last_link_rx)
                tx_seq='--' if uart_last_tx_seq is None else str(uart_last_tx_seq)
                rx_seq='--' if feedback.last_seq is None else str(feedback.last_seq)
                rx_state='--' if feedback.last_state is None else str(feedback.last_state)
                pending='--' if feedback.pending is None else str(feedback.pending)
                uart_text='LINK:%s TX:%d Q%s RX:%d Q%s ST:%s P:%s %s B:%d' % (
                    link,uart_tx_count,tx_seq,feedback.valid_packets,rx_seq,
                    rx_state,pending,rx_age,feedback.bad_packets)
                preview_img.draw_string_advanced(8,94,17,uart_text,color=(255,255,255))
                Display.show_image(preview_img, x=round((640-sensor.width())/2),
                                   y=round((480-sensor.height())/2))
                old_preview = preview_hold
                preview_hold = preview_img
                del old_preview
                last_preview = now
                # Do not delete preview_hold: Display may still be reading it.
            elapsed = time.ticks_diff(now, last_stats)
            if elapsed >= 1000:
                display_fps = frames*1000/elapsed
                if PRINT_STATS:
                    perf_phase = "target" if shape_detection is not None else "search"
                    print("FPS %.1f algo %.2fms lane %.2fms shape %.2fms max %.2fms "
                          "phase=%s %s valid=%d conf=%.2f steer=%.2f" %
                          (frames*1000/elapsed, processing_us/frames/1000,
                           lane_us/frames/1000,
                           shape_us/max(1, shape_calls)/1000, shape_max_us/1000,
                           perf_phase,
                           result["state"], result["valid"], result["confidence"], result["steer"]))
                frames = processing_us = lane_us = shape_us = shape_calls = 0
                shape_max_us = shape_max_id = 0
                last_stats = now
                gc.collect()
    except KeyboardInterrupt as exc:
        print("user stop:", exc)
    except BaseException as exc:
        sys.print_exception(exc)
    finally:
        # 退出时发送无效视觉数据；主板沿用现有丢线控制策略。
        if uart is not None:
            try:
                uart.write(encode(dict(valid=False, stop=True, state="LOST",
                                       offset=0, heading=0, steer=0, confidence=0), sequence))
                uart.write(encode_angle(dict(valid=False, active=False, angle=0), sequence))
            except Exception as exc:
                print("UART stop packet:", exc)
            try:
                uart.deinit()
            except Exception as exc:
                print("UART cleanup:", exc)
        if sensor is not None:
            try:
                sensor.stop()
            except Exception as exc:
                print("Sensor cleanup:", exc)
        if display_ready:
            try:
                Display.deinit()
            except Exception as exc:
                print("Display cleanup:", exc)
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        if media_ready:
            try:MediaManager.deinit()
            except Exception as exc:print("Media cleanup:",exc)


main()
