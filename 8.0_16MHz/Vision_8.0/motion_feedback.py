"""Optional UART feedback. No RX pin => open-loop, no fabricated step completion."""
from lane_protocol import crc16

class MotionFeedback:
    def __init__(self, enabled=False, continuous_control=False):
        self.enabled=enabled
        self.continuous_control=continuous_control
        self.buffer=bytearray()
        self.pending=None
        self.ready=not enabled
        self.sent_at=None
        self.settle_until=0
        self.reset_heading=False
        self.fault=False
        self.link_established=not enabled
        self.start_time=0.0
        self.action_timeout=12.0
        # Any CRC-valid AA57 proves the UART link is alive.  Keep this
        # separate from last_rx, which is intentionally stricter and only
        # advances the motion-feedback state machine on an applicable reply.
        self.last_link_rx=None
        self.last_rx=None
        self.last_state=None
        self.last_seq=None
        self.valid_packets=0
        self.bad_packets=0
        self.rx_bytes=0

    def feed(self,data,now):
        if not data:return
        self.rx_bytes+=len(data)
        self.buffer.extend(data)
        # K230 CanMV MicroPython does not support deleting bytearray items.
        if len(self.buffer)>128:self.buffer=bytearray(self.buffer[-128:])
        while len(self.buffer)>=6:
            if self.buffer[0]!=0xaa or self.buffer[1]!=0x57:
                self.bad_packets+=1
                self.buffer=bytearray(self.buffer[1:]);continue
            packet=self.buffer[:6]
            if packet[3]>3 or crc16(packet[:4])!=(packet[4]|packet[5]<<8):
                self.bad_packets+=1
                self.buffer=bytearray(self.buffer[1:]);continue
            self.buffer=bytearray(self.buffer[6:])
            seq,state=packet[2],packet[3]
            self.valid_packets+=1
            self.last_link_rx=now
            self.last_seq=seq
            self.last_state=state
            if state==0:
                # READY is a one-shot startup handshake, not a heartbeat.
                self.link_established=True
                self.pending=None;self.fault=False;self.ready=True;self.last_rx=now
            elif self.pending==seq:
                self.last_rx=now
                if state==1:self.ready=False
                elif state==2:
                    self.pending=None;self.fault=False;self.ready=True;self.settle_until=now+.15
                    self.reset_heading=True
                elif state==3:self.fault=True;self.ready=False
            elif state==3:
                self.fault=True;self.ready=False

    def link_status(self,now):
        if not self.enabled:return 'OFF'
        if self.fault:return 'FAULT'
        if not self.link_established:return 'WAIT' if now-self.start_time<=3.0 else 'FAULT'
        return 'OK'

    def gate(self,lane,control,now):
        if not self.enabled:return lane,control
        # 主板已按完整步锁存方向，反馈仅作诊断。丢反馈不能把新左转换成等待，
        # 否则主板会一直沿用旧右转。真实主板故障仍由主板运动层处理。
        if self.continuous_control:return lane,control
        if self.pending is not None and self.sent_at is not None and now-self.sent_at>self.action_timeout:
            self.fault=True;self.ready=False
        waiting=(self.fault or not self.link_established or not self.ready or
                 self.pending is not None or now<self.settle_until)
        if waiting:
            lane=dict(lane);lane['state']='STRAIGHT' if lane['valid'] else 'LOST'
            control=dict(control);control.update(active=False,direction=0,angle=0.0,status=5,reason='WAIT_BOARD')
        return lane,control

    def sent(self,control,sequence,now):
        if self.continuous_control:return
        if self.enabled and control['active'] and self.pending is None:
            self.pending=sequence&255;self.sent_at=now;self.ready=False
