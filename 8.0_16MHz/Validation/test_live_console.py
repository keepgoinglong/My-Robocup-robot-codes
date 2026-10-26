from test_feedback import Board,control
import struct
b=Board();b.stand();b.call('HandleCommand',ord('g'));assert b.call('VisionControl_IsEnabled')==1
b=Board();b.stand();b.call('HandleCommand',ord('t'));assert b.call('VisionControl_IsEnabled')==0
b.control=True;b.step()
frame=b.value('s_frame');begin=b.value('s_begin',4)
b.call('HandleCommand',ord('g'));assert b.call('VisionControl_IsEnabled')==1
assert (b.value('s_frame'),b.value('s_begin',4))==(frame,begin)
b.step();b.feed(control(51,3));b.step();assert b.value('s_control_phase')==1;phase=b.value('s_control_phase');b.call('HandleCommand',ord('g'));assert b.value('s_control_phase')==phase
b.call('HandleCommand',ord('i'));n=b.value('s_console_head',2);assert n>0
raw=bytes(b.uc.mem_read(b.symbols['s_console_tx'],n));assert b'VISION enabled=1' in raw and b'AA56 status=3' in raw
# Emulate USART1 TXE low/high: no progress when busy; exactly one byte when ready.
b.uc.mem_write(b.symbols['huart1'],struct.pack('<I',0x40011000))
b.uc.mem_write(0x40011000,struct.pack('<I',0));b.call('ConsoleService');assert b.value('s_console_tail',2)==0
b.uc.mem_write(0x40011000,struct.pack('<I',128));b.call('ConsoleService');assert b.value('s_console_tail',2)==1
assert b.uc.mem_read(0x40011004,1)==raw[:1]
b.done(51)
b.call('HandleCommand',ord('s'));assert b.call('VisionControl_IsEnabled')==0
print('PASS actual ARM: g starts/enables; moving g enables without restarting; repeated g preserves transaction; moving i queues status; TXE service nonblocking; s still disables.')
