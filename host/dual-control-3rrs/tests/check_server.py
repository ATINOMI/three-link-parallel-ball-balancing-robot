"""Smoke test the packaged local server in demo mode: no controller or motor opened."""
from pathlib import Path
import json
import subprocess
import time
import urllib.request
import socket

root=Path(__file__).resolve().parents[1]
with socket.socket() as sock:
    sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
exe=root/'build-input-v2/dual_control_3rrs.exe'
process=subprocess.Popen([str(exe),'--demo','--no-browser','--seconds','4','--port',str(port)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,creationflags=subprocess.CREATE_NO_WINDOW)
url=f'http://127.0.0.1:{port}'
try:
    for _ in range(40):
        try:
            data=json.load(urllib.request.urlopen(url+'/api/state',timeout=.2))
            if data.get('ready'):break
        except OSError:pass
        time.sleep(.05)
    assert data['app']=='dual-control-3rrs' and data['demo'] and data['ready']
    assert data['input_version']==2 and data['dpad_up_seq']==0 and data['dpad_down_seq']==0 and len(data['angles'])==3 and data['circle_seq']==0
    for name in ['','app.js','style.css','core.js','serial.js','control.js','kinematics.js','scene.js']:
        with urllib.request.urlopen(url+'/'+name,timeout=1) as response:
            assert response.status==200 and len(response.read())>100
    for state in ['normal','invalid','idle']:
        response=urllib.request.urlopen(urllib.request.Request(url+'/api/feedback/'+state,method='POST'),timeout=1)
        assert json.load(response)['ok']
    stdout,stderr=process.communicate(timeout=6)
    assert process.returncode==0,(stdout,stderr)
    print('PASS: packaged local HTTP server, demo IMU JSON, all WebUI resources, feedback routes and timed clean exit; no hardware opened.')
finally:
    if process.poll() is None:process.terminate();process.wait(timeout=3)
