"""Execute the actual Keil ARM image with mocked clock/servo bus (not robot dynamics).

python -m pip install --target Validation/.deps unicorn pyelftools
python Validation/test_firmware.py
"""
from pathlib import Path
import hashlib
import json
import math
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'Validation/.deps'))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile

IMAGE = ROOT / 'MDK-ARM/HumanoidRobot_6_0/HumanoidRobot_6_0.axf'
from action_model import DATA, LOOP
FRAMES = DATA['frames']
TARGETS = [[f['positions'][str(i)] for i in range(1,15)] for f in FRAMES]
RESULTS = []

class Firmware:
    def __init__(self, bus_ms=0):
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.mem_map(0x08000000, 0x80000)
        self.uc.mem_map(0x20000000, 0x20000)
        self.uc.mem_map(0xe000e000, 0x2000)
        with IMAGE.open('rb') as stream:
            elf = ELFFile(stream)
            self.symbols = {s.name:int(s['st_value']) for s in elf.get_section_by_name('.symtab').iter_symbols()}
            for section in elf.iter_sections():
                if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS' and section['sh_size']:
                    self.uc.mem_write(section['sh_addr'], section.data())
        self.uc.reg_write(UC_ARM_REG_C1_C0_2, 0xf00000)
        self.uc.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
        self.uc.mem_write(0xe000ed88, struct.pack('<I', 0xf00000))
        self.tick = 0
        self.bus_ms = bus_ms
        self.measured = [2048]*14
        self.feedback_follows = True
        self.read_fail = self.write_fail = self.torque_fail = False
        self.trace = []
        self.torque = []
        for name, hook in [('HAL_GetTick',self.clock), ('STS_ReadPosition',self.read),
                           ('STS_SyncWritePosition',self.write), ('STS_TorqueEnable',self.enable)]:
            address = self.symbols[name] & ~1
            self.uc.hook_add(UC_HOOK_CODE, hook, begin=address, end=address)
        self.call('Robot_Init')

    def value(self, name, size=1):
        return int.from_bytes(self.uc.mem_read(self.symbols[name],size),'little')

    def result(self, value):
        self.uc.reg_write(UC_ARM_REG_R0,value)
        self.uc.reg_write(UC_ARM_REG_PC,self.uc.reg_read(UC_ARM_REG_LR))

    def clock(self,*args): self.result(self.tick & 0xffffffff)

    def read(self,*args):
        i=self.uc.reg_read(UC_ARM_REG_R0)-1
        ptr=self.uc.reg_read(UC_ARM_REG_R1)
        if not self.read_fail: self.uc.mem_write(ptr,struct.pack('<H',self.measured[i]))
        self.tick += self.bus_ms
        self.result(int(self.read_fail))

    def enable(self,*args):
        self.torque.append((self.tick,self.uc.reg_read(UC_ARM_REG_R1)))
        self.tick += self.bus_ms
        self.result(int(self.torque_fail))

    def write(self,*args):
        ptr=self.uc.reg_read(UC_ARM_REG_R1)
        count=self.uc.reg_read(UC_ARM_REG_R2)
        speed=self.uc.reg_read(UC_ARM_REG_R3)
        acc=int.from_bytes(self.uc.mem_read(self.uc.reg_read(UC_ARM_REG_SP),4),'little')
        assert 1 <= count <= 14
        ids=list(self.uc.mem_read(self.uc.reg_read(UC_ARM_REG_R0),count))
        sent=list(struct.unpack('<'+str(count)+'H',self.uc.mem_read(ptr,count*2)))
        pos=list(struct.unpack('<14H',self.uc.mem_read(self.symbols['s_now'],28)))
        for i,p in enumerate(pos,1):
            lo,hi=DATA['limits'][str(i)]
            assert lo <= p <= hi,(i,p)
        self.trace.append(dict(t=self.tick, frame=self.value('s_frame'), begin=self.value('s_begin',4),
                               duration=self.value('s_duration',4), pos=pos, ids=ids, sent=sent, speed=speed, acc=acc))
        if self.feedback_follows and not self.write_fail:
            for sid,p in zip(ids,sent):self.measured[sid-1]=p
        self.tick += self.bus_ms
        self.result(int(self.write_fail))

    def call(self,name,*args):
        self.uc.reg_write(UC_ARM_REG_SP,0x2001f000)
        self.uc.reg_write(UC_ARM_REG_LR,0x0807f001)
        for reg,value in zip([UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3],args):
            self.uc.reg_write(reg,value)
        self.uc.emu_start(self.symbols[name] | 1,0x0807f000,count=100000)
        assert self.uc.reg_read(UC_ARM_REG_PC) == 0x0807f000, name
        return self.uc.reg_read(UC_ARM_REG_R0)

    def step(self,ms=10):
        self.tick += ms
        self.call('Robot_Task')

    def until(self,condition,limit=30000):
        end=self.tick+limit
        while not condition():
            assert self.tick < end, ('timeout',self.value('s_state'),self.value('s_frame'))
            self.step()

    def stand(self):
        assert self.call('Robot_EnterStand') == 1
        self.until(lambda:self.value('s_state') == 2)


def record(name):
    RESULTS.append(name)
    print('PASS:',name)

def run():
    fw=Firmware(); assert fw.value('s_state')==0 and not fw.trace
    assert fw.call('Robot_StartActions',1,0)==0
    fw.stand(); assert fw.measured == TARGETS[0]
    assert fw.call('Robot_StartActions',0,0)==1
    fw.until(lambda:sum(r['frame']==10 and r['t']==r['begin'] for r in fw.trace)>=4,40000)
    previous=None; begin=None; starts=[]; start_pos=None
    for row in fw.trace:
        key=(row['frame'],row['begin']); f=FRAMES[row['frame']]
        if key!=begin:
            if previous is not None: assert previous['pos']==TARGETS[previous['frame']]
            start_pos=row['pos']; starts.append(row); begin=key
        assert row['speed']==f['speed'] and row['acc']==f['acc']
        if row['frame']:
            assert row['duration']==f['duration_ms']
        expected=[]
        for i,(a,b) in enumerate(zip(start_pos,TARGETS[row['frame']]),1):
            timing=f.get('timing',{}).get(str(i),{})
            delay=timing.get('delay_ms',0);duration=timing.get('duration_ms',row['duration'])
            x=max(0,min(1,(row['t']-row['begin']-delay)/duration))
            q=x*x*x*(10+x*(-15+6*x));expected.append(round(a+(b-a)*q))
        assert max(abs(a-b) for a,b in zip(row['pos'],expected))<=1
        previous=row
    walk=[r for r in starts if r['frame']]
    expected_sequence=list(range(1,10))+LOOP*3+[10]
    assert [r['frame'] for r in walk]==expected_sequence
    for a,b in zip(walk,walk[1:]):
        assert b['t']-a['t']==FRAMES[a['frame']]['duration_ms']+FRAMES[a['frame']]['hold_ms']
    record('Actual ARM image: original positions, speed, ACC, holds, quintic curve <=1 count, startup once and 3 loop seams')

    fw=Firmware();fw.stand();assert fw.call('Robot_StartActions',1,0)==1
    fw.until(lambda:sum(r['frame']==10 and r['t']==r['begin'] for r in fw.trace)>=3,40000)
    starts=[r for r in fw.trace if r['frame'] and r['t']==r['begin']]
    for r in starts:
        f=FRAMES[r['frame']];required=f['duration_ms']
        for i,(a,b) in enumerate(zip(r['pos'],TARGETS[r['frame']]),1):
            ms=math.ceil(187500*abs(a-b)/(r['speed']*90))
            timing=f.get('timing',{}).get(str(i),{})
            if timing:ms=math.ceil(ms*f['duration_ms']/timing['duration_ms'])
            required=max(required,math.ceil(ms/10)*10)
        assert r['duration']==required,(r,required)
    record('Optional speed budget includes per-servo timing; normal JSON profile remains exact')

    for stop_frame in list(range(1,10))+LOOP:
        fw=Firmware();fw.stand();fw.call('Robot_StartActions',0,0)
        fw.until(lambda:fw.value('s_frame')==stop_frame)
        fw.call('Robot_StopMarch');fw.until(lambda:fw.value('s_state')==5)
        landing=next((i for i in (3,7,11,15) if i>=stop_frame),11)
        assert fw.value('s_frame')==landing,(stop_frame,fw.value('s_frame'))
        held=fw.measured[:];count=len(fw.trace)
        for _ in range(50):fw.step()
        assert len(fw.trace)==count
        assert fw.call('Robot_StartActions',1,0)==1
        assert fw.value('s_frame')==({11:13,15:17}.get(landing,landing+1)) and fw.trace[-1]['pos']==held
    record('Stop from every walking frame lands at 3/7/11/15; hold emits no packets; resume retains pose/sequence')

    fw=Firmware();fw.stand();fw.call('Robot_StartActions',1,1)
    fw.until(lambda:fw.value('s_state')==5,30000)
    assert fw.value('s_frame')==15
    fw.call('Robot_StartActions',1,1);fw.until(lambda:fw.value('s_state')==5,20000)
    assert fw.value('s_frame')==15
    record('Finite trial stops at fourth-step landing, including resume through a complete loop')

    fw=Firmware(bus_ms=2);fw.stand();fw.call('Robot_StartActions',1,1)
    fw.until(lambda:fw.value('s_state')==5,40000)
    assert fw.value('s_frame')==15
    # Original frame1 hold starts after endpoint packet completion, not before it.
    endpoints=[r for r in fw.trace if r['frame']==1 and r['pos']==TARGETS[1]]
    first2=next(r for r in fw.trace if r['frame']==2)
    assert first2['t']-endpoints[-1]['t']>=FRAMES[1]['hold_ms']+2
    record('2ms mocked bus latency: no timestamp underflow, full holds, stand feedback remains responsive')

    fw=Firmware();fw.stand();fw.call('Robot_StartActions',1,0)
    count=len(fw.trace);assert fw.call('Robot_Scan',0)==0
    assert fw.call('Robot_EnterStand')==0
    fw.step(251);assert fw.value('s_state')==4 and len(fw.trace)==count
    assert fw.torque[-1][1]==1
    fw.call('Robot_EmergencyOff');assert fw.value('s_state')==0 and fw.torque[-1][1]==0
    record('Moving scan/stand rejected; scheduling stall holds last command; emergency stops future motion')

    fw=Firmware();fw.stand();fw.measured[6]+=100
    assert fw.call('Robot_StartActions',1,0)==0
    fw=Firmware();fw.measured[11]=3000
    assert fw.call('Robot_EnterStand')==0 and not fw.trace
    fw=Firmware();fw.read_fail=True
    assert fw.call('Robot_EnterStand')==0 and not fw.trace
    fw=Firmware();fw.stand();fw.write_fail=True
    assert fw.call('Robot_StartActions',1,0)==0 and fw.value('s_state')==4
    fw=Firmware();fw.torque_fail=True;fw.call('Robot_EmergencyOff')
    assert fw.value('s_state')==4
    record('Changed pose, out-of-range start, read failure, write failure, emergency-write failure handled')

    fw=Firmware();fw.measured[0]=2100;fw.feedback_follows=False
    assert fw.call('Robot_EnterStand')==1
    fw.until(lambda:fw.value('s_state')==4,15000)
    fw=Firmware();fw.tick=0xfffff000;fw.stand();fw.call('Robot_StartActions',1,1)
    fw.until(lambda:fw.value('s_state')==5,40000)
    record('Stand feedback timeout and 32-bit tick wrap')
    report={'image_sha256':hashlib.sha256(IMAGE.read_bytes()).hexdigest(),
            'passed':RESULTS,'scope':'Actual Keil ARM machine code with mocked servo/clock; not mechanics or board timing.'}
    (ROOT/'Validation/firmware_test_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')

if __name__=='__main__':run()
