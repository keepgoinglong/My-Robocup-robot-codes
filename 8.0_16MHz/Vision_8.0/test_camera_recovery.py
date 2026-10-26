"""PC mocks exercise recovery branches; no board camera/FPS claim."""
import ast
from pathlib import Path
from unittest import TestCase,main
from unittest.mock import Mock

def helpers():
    tree=ast.parse(Path(__file__).with_name('main_modular.py').read_text(encoding='utf-8'))
    names={'is_capture_error','capture_warmup','reset_failed_camera','open_camera','close_camera'}
    tree.body=[node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name in names]
    sensor=Mock()
    ns=dict(RuntimeError=RuntimeError,Sensor=Mock(return_value=sensor),Display=Mock(),MediaManager=Mock(),
            os=Mock(),gc=Mock(),time=Mock(),machine=Mock(),PREVIEW=True,ROTATE_180=False,
            DISPLAY_FPS=30,IDE_QUALITY=50,CAMERA_PREPARE_MS=500,
            SENSOR_MODE_WIDTH=1920,SENSOR_MODE_HEIGHT=1080,
            CAMERA_WARMUP_FRAMES=3,CAMERA_WARMUP_RETRIES=10,CAPTURE_RETRY_MS=100,
            AUTO_REBOOT_ON_CAMERA_FAILURE=True)
    exec(compile(tree,'camera recovery helpers','exec'),ns)
    return ns,sensor

class CameraRecoveryTest(TestCase):
    def test_transient_warmup_failure_recovers(self):
        ns,sensor=helpers()
        sensor.snapshot.side_effect=[RuntimeError('sensor(0) snapshot chn(0) failed(3)'),object(),object(),object()]
        ns['open_camera']()
        self.assertEqual(sensor.snapshot.call_count,4)
        ns['machine'].reset.assert_not_called()

    def test_persistent_warmup_failure_cleans_then_resets(self):
        ns,sensor=helpers();events=[]
        sensor.snapshot.side_effect=RuntimeError('sensor(0) snapshot chn(0) failed(3)')
        sensor.stop.side_effect=lambda:events.append('stop')
        ns['Display'].deinit.side_effect=lambda:events.append('display')
        ns['MediaManager'].deinit.side_effect=lambda:events.append('media')
        ns['machine'].reset.side_effect=lambda:events.append('reset')
        with self.assertRaisesRegex(RuntimeError,'unexpectedly returned'):ns['open_camera']()
        self.assertEqual(sensor.snapshot.call_count,10)
        self.assertEqual(events,['stop','display','media','reset'])

    def test_cleanup_failure_does_not_hide_reset(self):
        ns,sensor=helpers()
        sensor.snapshot.side_effect=RuntimeError('sensor(0) snapshot chn(0) failed(3)')
        ns['MediaManager'].deinit.side_effect=RuntimeError('VB cleanup failed')
        with self.assertRaisesRegex(RuntimeError,'unexpectedly returned'):ns['open_camera']()
        ns['machine'].reset.assert_called_once()

    def test_other_error_is_not_retried_or_rebooted(self):
        ns,sensor=helpers();sensor.snapshot.side_effect=RuntimeError('unsupported format')
        with self.assertRaisesRegex(RuntimeError,'unsupported format'):ns['open_camera']()
        self.assertEqual(sensor.snapshot.call_count,1)
        ns['machine'].reset.assert_not_called()

if __name__=='__main__':main()
