"""Protocol and event integration tests executing the actual Keil ARM image.
Mocked UART/servo hardware: this is not a real-board or stability test.
"""
import binascii
import hashlib
import json
import random
import struct
from test_firmware import Firmware, ROOT, IMAGE, TARGETS, FRAMES
from unicorn import UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2

RESULTS=[]
def packet(seq,action=1,shape=0):
    data=bytes([0xaa,0x55,seq,action,shape])
    return data+struct.pack('<H',binascii.crc_hqx(data,0xffff))

class System(Firmware):
    def __init__(self,bus_ms=0):
        super().__init__(bus_ms)
        self.control=False
        self.remote_action=None
        self.remote_shape=0
        self.remote_seq=0
        self.next_packet=0
        self.rearms=[]
        self.rx_fail=False
        self.aborts=0
        self.uc.mem_map(0x40000000,0x20000)
        for name,hook in [('HAL_UART_Receive_IT',self.receive_it),('HAL_UART_AbortReceive',self.abort_rx)]:
            address=self.symbols[name]&~1
            self.uc.hook_add(UC_HOOK_CODE,hook,begin=address,end=address)
        self.call('Vision_Init',0)
        self.call('VisionControl_Init')

    def write(self,*args):
        super().write(*args)
        self.trace[-1].update(phase=self.value('s_shape_phase'),shape=self.value('s_active_shape'))

    def receive_it(self,*args):
        self.rearms.append((self.uc.reg_read(UC_ARM_REG_R0),self.uc.reg_read(UC_ARM_REG_R1),self.uc.reg_read(UC_ARM_REG_R2)))
        self.result(int(self.rx_fail))

    def abort_rx(self,*args):
        self.aborts+=1
        self.result(0)

    def feed(self,data):
        for byte in data:self.call('Vision_FeedByte',byte,self.tick&0xffffffff)

    def status(self):
        address=0x20018000
        self.call('Vision_GetStatus',address)
        words=struct.unpack('<9I',self.uc.mem_read(address,36))
        tail=list(self.uc.mem_read(address+36,6))
        return dict(zip(['valid','crc','fields','duplicates','lost','jumps','uart','overflow','last_ms',
                         'sequence','action','shape','online','queued','latched'],[*words,*tail]))

    def pop(self):
        address=0x20018000
        if not self.call('Vision_PopShape',address):return None
        raw=self.uc.mem_read(address,8)
        return raw[0],raw[1],int.from_bytes(raw[4:8],'little')

    def step(self,ms=10):
        self.tick+=ms
        if self.remote_action is not None and self.tick>=self.next_packet:
            self.feed(packet(self.remote_seq,self.remote_action,self.remote_shape))
            # Unified protocol: normal walking is requested by AA56 status1, not AA55.
            if self.remote_action == 1:
                body=bytes([0xaa,0x56,self.remote_seq,1,0,0])
                self.feed(body+struct.pack('<H',binascii.crc_hqx(body,0xffff)))
            self.remote_shape=0
            self.remote_seq=(self.remote_seq+1)&255
            self.next_packet=self.tick+50
        if self.control:self.call('VisionControl_Task')
        self.call('Robot_Task')

def record(name):
    RESULTS.append(name);print('PASS:',name)

def run():
    # Actual shared g/r command implementation, including s -> g re-enable.
    for shape in range(1,6):
        fw=System();fw.stand();fw.control=True
        fw.call('VisionControl_SetEnabled',0)
        assert fw.call('VisionControl_StartWalk')==1
        assert fw.call('VisionControl_IsEnabled')==1
        fw.feed(packet(1,0,shape))
        fw.until(lambda:fw.value('s_completed_shapes',4)==1,30000)
        rows=[r for r in fw.trace if r['shape']==shape and r['phase'] in (1,3)]
        assert rows
        next_rows=[r for r in fw.trace if r['t']>rows[-1]['t']]
        assert next_rows[0]['t']-rows[-1]['t']==3000
        for _ in range(500):fw.step()
        assert fw.value('s_state')==3
        fw.call('VisionControl_SetEnabled',0)
        fw.until(lambda:fw.value('s_state')==5,10000)
        fw.remote_action=1
        for _ in range(300):fw.step()
        assert fw.value('s_state')==5
    record('g/r shared entry re-enables all five shapes; action0 and silence preserve 3000ms holds and walking; manual stop stays stopped')
    # Exercise the real vision entry point, not just an explicitly selected profile.
    fw=System();fw.stand();fw.control=True;fw.remote_action=1
    fw.until(lambda: len([r for r in fw.trace if r['frame']==10 and r['t']==r['begin']])>=3,40000)
    frames=FRAMES
    starts=[r for r in fw.trace if r['frame'] and r['t']==r['begin']]
    assert starts and all(r['duration']==frames[r['frame']]['duration_ms'] for r in starts)
    seams=[r['t'] for r in starts if r['frame']==10]
    assert seams[1]-seams[0]==2800 and seams[2]-seams[1]==2800
    assert [r['pos'] for r in starts if r['frame']==10]==[TARGETS[9],TARGETS[17],TARGETS[17]]
    record('Vision entry uses exact new JSON timing; repeated two-step cycles are 2800ms and startup/loop each use the correct preceding endpoint')
    fw=System();assert fw.call('Vision_GetAction')==0
    fw.feed(b'\x00\x55\xaa\xaa'+packet(254,2,6))
    assert fw.status()['valid']==1 and fw.call('Vision_GetAction')==2
    assert fw.pop()[:2]==(254,6)
    fw.feed(packet(255,1,3)[:3]);assert fw.status()['valid']==1
    fw.tick+=50;fw.feed(packet(255,1,3)[3:]+packet(0,3,0)+packet(1,1,4))
    assert fw.status()['valid']==4 and fw.status()['lost']==0
    assert fw.pop()[:2]==(255,3) and fw.pop()[:2]==(1,4) and fw.pop() is None
    record('Noise, AA-AA55 overlap, split packets, concatenated packets, independent action/shape, 255->0')

    fw=System()
    bad=bytearray(packet(10,1,5));bad[-1]^=1
    fw.feed(bad+packet(11,4,1)+packet(12,1,7))
    assert fw.status()['valid']==0 and fw.status()['fields']==2 and fw.status()['crc']>=1
    # A bad frame contains the beginning of the following good frame.
    fw.feed(b'\xaa\x55\x33'+packet(13,1,5))
    assert fw.status()['valid']==1 and fw.pop()[:2]==(13,5)
    fw.tick+=251;assert fw.call('Vision_GetAction')==1 and not fw.status()['online']
    fw.feed(bad);assert fw.call('Vision_GetAction')==1 and not fw.status()['online']
    record('CRC/field rejection, embedded-header resync; invalid packets do not renew online time or change last valid action')

    fw=System();fw.feed(packet(255,1,1)*3+packet(0,1,0)+packet(1,1,1))
    assert fw.status()['duplicates']==2
    assert fw.pop()[:2]==(255,1) and fw.pop()[:2]==(1,1) and fw.pop() is None
    for seq in range(2,256):fw.feed(packet(seq))
    fw.feed(packet(0,1,1));assert fw.pop()[:2]==(0,1)
    record('Repeated sequence emits once; shape0 does not cancel; new sequence and full sequence wrap permit new event')

    fw=System();fw.tick=0xfffffff0;fw.feed(packet(0))
    fw.tick+=250;assert fw.call('Vision_GetAction')==1 and fw.status()['online']
    fw.tick+=1;assert fw.call('Vision_GetAction')==1 and not fw.status()['online']
    for i in range(64):fw.feed(packet(i+1,1,1))
    assert fw.status()['queued']==63 and fw.status()['overflow']==1
    assert fw.call('Vision_GetAction')==1 and fw.status()['latched']==1
    assert fw.pop()[:2]==(1,1)
    fw.call('Vision_ClearOverflow');assert fw.call('Vision_GetAction')==1
    record('Exact >250ms offline diagnostic including clock wrap; queue overflow retains previous events and never changes movement action')

    fw=System();rng=random.Random(923)
    for seq in range(256):
        fw.feed(bytes(rng.randrange(0,0xaa) for _ in range(rng.randrange(0,12))))
        action=rng.randrange(4);shape=rng.randrange(7)
        fw.feed(packet(seq,action,shape))
        assert fw.call('Vision_GetAction')==action
        event=fw.pop();assert (event is not None)==(shape!=0)
        if event:assert event[:2]==(seq,shape)
    record('256 independent Python crc_hqx vectors accepted with randomized valid fields and interference')

    fw=System();uart=fw.symbols['huart4']
    fw.uc.mem_write(uart,struct.pack('<I',0x40004c00))
    fw.call('Vision_Init',uart);assert fw.rearms[-1][2]==1
    for byte in packet(4,3,6):
        fw.uc.mem_write(fw.symbols['s_rx_byte'],bytes([byte]))
        fw.call('HAL_UART_RxCpltCallback',uart)
    assert fw.call('Vision_GetAction')==3 and fw.pop()[:2]==(4,6)
    assert len(fw.rearms)==8
    fw.feed(b'\xaa\x55\x03');fw.call('HAL_UART_ErrorCallback',uart)
    assert fw.aborts==1 and fw.status()['uart']==1
    fw.feed(packet(5,1,2));assert fw.pop()[:2]==(5,2)
    fw.rx_fail=True;fw.call('HAL_UART_ErrorCallback',uart)
    fw.rx_fail=False;fw.call('Vision_Service')
    assert fw.value('s_restart_rx')==0
    count=len(fw.rearms);fw.call('HAL_UART_RxCpltCallback',uart+72)
    assert len(fw.rearms)==count
    record('UART4 callback feeds bytes and rearms 1-byte IT; error/rearm retry preserves recovery; other UART ignored')

    for profile in (0,1):
        for shape in range(1,6):
            fw=System();fw.stand();assert fw.call('Robot_RequestShape',shape)==1
            fw.call('Robot_StartActions',profile,0)
            fw.until(lambda:fw.value('s_completed_shapes',4)==1,30000)
            target=TARGETS[{1:11,2:11,3:14,4:10,5:11}[shape]].copy()
            source=json.loads((ROOT/f'Actions/{shape}.json').read_text(encoding='utf-8-sig'))['frames'][0]
            anchor={1:11,2:11,3:14,4:10,5:11}[shape]
            target=TARGETS[anchor].copy()
            if shape in (1,2,5): target[:4]=[source['positions'][str(i)] for i in range(1,5)]
            else: source=FRAMES[anchor]
            rows=[r for r in fw.trace if r['shape']==shape and r['phase'] in (1,3)]
            assert rows and rows[-1]['pos']==target
            if profile==0: assert all(r['duration']==source['duration_ms'] for r in rows)
            for r in rows:assert r['speed']==source['speed'] and r['acc']==source['acc']
            end=rows[-1]['t']
            if shape in (1,2,5):
                assert all(r['frame']==11 and r['pos'][4:]==TARGETS[11][4:] for r in rows)
                returns=[r for r in fw.trace if r['shape']==shape and r['phase']==2]
                assert returns[0]['t']-end==3000
                assert returns[-1]['pos']==TARGETS[11]
                assert fw.value('s_frame')==13
            else:
                anchor=14 if shape==3 else 10
                assert rows[0]['frame']==anchor
                expected_hold=3000
                following=[r for r in fw.trace if r['frame']==anchor+1]
                assert following[-1]['t']-end==expected_hold
                # Entire leg curve matches existing gait interpolation, only arms/hold are added.
                origin=rows[0]['pos'];duration=rows[0]['duration']
                for r in rows:
                    expected=[]
                    for i,(a,b) in enumerate(zip(origin[4:],TARGETS[anchor][4:]),5):
                        timing=FRAMES[anchor].get('timing',{}).get(str(i),{})
                        delay=timing.get('delay_ms',0)*duration//FRAMES[anchor]['duration_ms']
                        span=timing.get('duration_ms',FRAMES[anchor]['duration_ms'])*duration//FRAMES[anchor]['duration_ms']
                        u=max(0,min(1,(r['t']-r['begin']-delay)/span));q=u**3*(10+u*(-15+6*u))
                        expected.append(round(a+(b-a)*q))
                    assert max(abs(a-b) for a,b in zip(expected,r['pos'][4:]))<=1
                assert fw.value('s_frame')==anchor+1
    record('All 5 gestures on new gait anchors in both gait profiles: stationary arm raise/3000ms/return, leg holds integrated without replaying swing')

    for shape in (3,4):
        fw=System();fw.stand();fw.call('Robot_RequestShape',shape);fw.call('Robot_StartActions',0,0)
        fw.until(lambda:fw.value('s_shape_phase')==3 and fw.value('s_stage')==2)
        fw.step(100);start=fw.tick;fw.call('Robot_StopMarch')
        fw.until(lambda:fw.value('s_state')==5)
        assert fw.tick-start<600 and fw.value('s_frame')==(15 if shape==3 else 11)
        assert fw.value('s_interrupted_shapes',4)==1
        assert fw.measured==TARGETS[fw.value('s_frame')]
    record('Explicit manual stop during either foot hold still lands with original safe-stop state')

    fw=System();fw.stand();fw.control=True;fw.remote_action=1
    fw.feed(packet(90,1,6));fw.call('VisionControl_Task')
    assert fw.value('s_pending_shape')==6 and fw.value('g_vision_shape6_received',4)==1
    fw.until(lambda:fw.value('s_shape_phase')==5,15000)
    shake_begin=fw.value('s_shake_begin',4)
    landing_frame=fw.value('s_frame')
    assert landing_frame==11 and fw.value('s_active_shape')==6
    fw.until(lambda:fw.tick==shake_begin+2990,3000)
    assert fw.value('s_shape_phase')==5 and fw.value('s_active')==1
    fw.step(10)
    assert fw.tick==shake_begin+3000 and fw.value('s_shape_phase')==5
    assert fw.value('s_active')==2
    fw.until(lambda:fw.tick==shake_begin+3400,500)
    assert fw.value('s_shape_phase')==0
    assert fw.value('s_active_shape')==0 and fw.value('s_active')==0
    assert fw.value('s_completed_shapes',4)==1 and fw.value('s_state')==3
    assert fw.value('s_frame') != landing_frame
    record('Shape6 waits specifically for right-foot-forward landing frame11, shakes for 3000ms, holds center 400ms, disables idle PWM, then resumes walking')

    fw=System();fw.stand();fw.call('Robot_StartActions',0,1)
    fw.until(lambda:fw.value('s_state')==5)
    fw.call('Robot_RequestShape',5)
    fw.until(lambda:fw.value('s_completed_shapes',4)==1)
    assert fw.value('s_state')==5 and fw.value('s_frame')==15
    fw.call('Robot_RequestShape',1);fw.measured[6]+=100;fw.feedback_follows=False
    fw.until(lambda:fw.value('s_state')==4,6000)
    assert not any(r['shape']==1 for r in fw.trace)
    record('Arm event while already stopped at left-front landing stays stopped; bad leg feedback prevents raising arms')

    fw=System();fw.control=True;fw.remote_action=1
    for _ in range(50):fw.step()
    assert fw.value('s_state')==0 and not fw.trace
    fw.stand();fw.until(lambda:fw.value('s_state')==3)
    fw.remote_shape=4
    fw.until(lambda:fw.value('s_active_shape')==4 and fw.value('s_stage')==2,25000)
    fw.remote_action=None;start=fw.tick
    hold_begin=fw.value('s_stage_begin',4)
    fw.until(lambda:fw.value('s_completed_shapes',4)==1,4000)
    assert fw.tick-hold_begin==3000 and fw.value('s_frame')==11 and fw.value('s_state')==3
    assert fw.value('s_interrupted_shapes',4)==0
    # Extended loss, action=0 and turn requests never call the stop interface.
    fw.until(lambda:fw.tick>=start+10000,15000)
    assert fw.value('s_state')==3 and fw.value('s_stop')==0
    fw.feed(packet((fw.remote_seq+1)&255,0,0));fw.call('VisionControl_Task')
    assert fw.value('s_state')==3 and fw.value('s_stop')==0
    fw.feed(packet((fw.remote_seq+2)&255,2,6));fw.call('VisionControl_Task')
    assert fw.call('VisionControl_TurnUnavailable')==0 and fw.value('s_state')==3 and fw.value('s_stop')==0
    assert fw.value('g_vision_shape6_received',4)==1
    fw.feed(packet((fw.remote_seq+3)&255,3,0));fw.call('VisionControl_Task')
    assert fw.call('VisionControl_TurnUnavailable')==0 and fw.value('s_state')==3 and fw.value('s_stop')==0
    fw.call('VisionControl_SetEnabled',0)
    fw.until(lambda:fw.value('s_state')==5,15000)
    fw.feed(packet((fw.remote_seq+4)&255,1,0));fw.call('VisionControl_Task')
    assert fw.value('s_state')==5
    fw.call('VisionControl_SetEnabled',1);fw.call('VisionControl_Task')
    assert fw.value('s_state')==3
    record('Full integration: foot hold remains 3000ms then walks; prolonged loss/action0/turn requests never stop; shape6 queued; manual stop lock retained')

    for shape in (3,4):
        for loss in (None,0):
            fw=System();fw.control=True;fw.remote_action=1;fw.stand()
            fw.call('Robot_RequestShape',shape)
            fw.until(lambda:fw.value('s_shape_phase')==3 and fw.value('s_stage')==2,25000)
            begin=fw.value('s_stage_begin',4)
            fw.remote_action=loss;fw.next_packet=fw.tick
            fw.until(lambda:fw.value('s_completed_shapes',4)==1,4000)
            assert fw.tick-begin==3000
            assert fw.value('s_state')==3 and fw.value('s_stop')==0
            assert fw.value('s_interrupted_shapes',4)==0
            end=fw.tick+7000
            fw.until(lambda:fw.tick>=end,8000)
            assert fw.value('s_state')==3 and fw.value('s_stop')==0
    record('Both left/right foot displays preserve full 3000ms under silence or continuous action0 and keep walking afterwards')

    fw=System();fw.control=True;fw.remote_action=1;fw.stand()
    fw.feed(packet(200,1,1)+packet(201,1,0)+packet(202,1,1)+packet(202,1,1))
    fw.until(lambda:fw.value('s_completed_shapes',4)==2,30000)
    assert fw.status()['queued']==0 and fw.value('s_pending_shape')==0
    for _ in range(100):fw.step()
    assert fw.value('s_completed_shapes',4)==2
    record('Two different-sequence arm events execute twice in FIFO order; repeated packet never adds a third event')

    (ROOT/'Validation/vision_test_results.json').write_text(json.dumps({
        'image_sha256':hashlib.sha256(IMAGE.read_bytes()).hexdigest(),
        'passed':RESULTS,'scope':'Actual ARM code; mocked UART and servo feedback; no physical contact/dynamics validation.'
    },indent=2,ensure_ascii=False),encoding='utf-8')

if __name__=='__main__':run()
