"""Host-only execution of actual main.c control/telemetry code; never opens hardware."""
from pathlib import Path
import subprocess
import tempfile
import sys
import os

root=Path(sys.argv[1] if len(sys.argv)>1 else r'E:\HAL32\F4\three_motor')
gcc=sys.argv[2] if len(sys.argv)>2 else r'E:\CLion 2026.1.1\bin\mingw\bin\gcc.exe'
os.environ['PATH']=str(Path(gcc).parent)+os.pathsep+os.environ.get('SystemRoot',r'C:\Windows')+'\\System32'+os.pathsep+os.environ.get('SystemRoot',r'C:\Windows')
tests=Path(__file__).resolve().parent
source=(root/'Core/Src/main.c').read_text(encoding='utf-8')
def section(name):
    return source.split('/* USER CODE BEGIN '+name+' */',1)[1].split('/* USER CODE END '+name+' */',1)[0]
assert all(x not in source for x in ['MotorAngleLimitReached','angle_limit_latched','MOTOR_MIN_ANGLE_DEG','motor_zero_limit_armed'])
prefix=(tests/'firmware_stubs.c').read_text(encoding='utf-8')
telemetry=source.split('    if ((uint32_t)(now - last_send_ms)',1)[1].split('/* USER CODE END 3 */',1)[0]
telemetry='    if ((uint32_t)(now - last_send_ms)'+telemetry
text=prefix+'\n'+section('PD')+section('PV')+section('0')+section('4')
startup=section('2').split('  for (int i = 0; i < 3; ++i)',1)[1].split('  if (HAL_TIM_Base_Start_IT',1)[0]
text+='\nstatic void InitializePids(void){for (int i = 0; i < 3; ++i)'+startup+'}\n'
text+='\nstatic void send_telemetry(uint32_t now){uint32_t last_send_ms=0;static char usb_tx_buffer[1280];\n'+telemetry
text+=(tests/'firmware_checks.c').read_text(encoding='utf-8')
with tempfile.TemporaryDirectory(prefix='rrs-firmware-') as tmp:
    work=Path(tmp);(work/'main.h').write_text('#include <stddef.h>\n',encoding='utf-8')
    (work/'check.c').write_text(text,encoding='utf-8')
    includes=root/'Core/Algorithm/Inc'
    subprocess.run([gcc,'-std=c11','-O0','-UNDEBUG','-I'+str(work),'-I'+str(includes),str(work/'check.c'),str(root/'Core/Algorithm/Src/pid.c'),str(root/'Core/Algorithm/Src/butterworth.c'),'-lm','-o',str(work/'check.exe')],check=True)
    subprocess.run([str(work/'check.exe')],cwd=work,check=True)
