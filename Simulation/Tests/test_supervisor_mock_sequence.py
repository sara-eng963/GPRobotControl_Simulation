import subprocess,socket,struct,time
log=__import__('tempfile').TemporaryFile(mode='w+')
p=subprocess.Popen([str(__import__('pathlib').Path(__import__('sys').argv[1]).resolve())],stdout=log,stderr=subprocess.STDOUT)
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('127.0.0.1',5012));s.settimeout(.2)
c=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);seq=0

def send(cmd,*arg):
 global seq
 seq+=1;a=[struct.unpack('!I',struct.pack('!f',x))[0] if isinstance(x,float) else x for x in arg]
 c.sendto(struct.pack('!8I',0x484d4933,3,cmd,seq,*(a+[0]*4)[:4]),('127.0.0.1',5010))
def wait(fn,secs=10):
 end=time.time()+secs;last=None
 while time.time()<end:
  if p.poll()!=None:raise Exception('process died')
  try:
   w=struct.unpack('!57I',s.recv(300));last=w
   if fn(w):return w
  except socket.timeout:pass
 raise Exception(('timeout',last and (last[3],last[14:26],last[26:34])))
def fl(w):return struct.unpack('!f',struct.pack('!I',w))[0]
try:
 w=wait(lambda w:w[3]==2);print('IDLE', [fl(v) for v in w[34:37]],flush=True)
 send(1);w=wait(lambda w:w[3]==3 and w[20]);print('TEACH',flush=True)
 send(4);w=wait(lambda w:w[45]==1);print('REC1',flush=True)
 xyz=[fl(v) for v in w[34:37]];xyz[0]+=.01;send(0x80000001,*xyz)
 w=wait(lambda w:abs(fl(w[34])-xyz[0])<.0003);print('GUIDED',flush=True)
 send(4);w=wait(lambda w:w[21]);print('REC2',flush=True)
 send(5);w=wait(lambda w:w[3]==2,15);print('VALIDATION',w[23:25],flush=True)
 if w[23]!=2:raise Exception('invalid path')
 send(5);w=wait(lambda w:w[31]==1 and w[3]==2,15);print('PREVIEW DONE',flush=True)
 send(9);w=wait(lambda w:w[3]==6,5);print('PRODUCTION ENTER',flush=True)
 send(10);wait(lambda w:w[3]==9);print('PAUSE',flush=True)
 send(10);time.sleep(.1);wait(lambda w:w[3]==9);print('DUPLICATE PAUSE REMAINS PAUSED',flush=True)
 send(11);wait(lambda w:w[3]==6);print('RESUME',flush=True)
 w=wait(lambda w:w[3]==2,20);print('PRODUCTION DONE',flush=True)
 send(0x80000002);wait(lambda w:w[3]==12);print('ESTOP',flush=True)
 send(0x80000002);time.sleep(.1);wait(lambda w:w[3]==12);print('RELEASE REMAINS LATCHED',flush=True)
 send(12);time.sleep(.1);wait(lambda w:w[3]==12);print('RESET WAITS FOR HOME',flush=True)
 send(13);wait(lambda w:w[3]==2,10);print('HOME RECOVERY DONE',flush=True)
 send(0x80000003,1);wait(lambda w:w[3]==11);print('EXTERNAL FAULT LATCHED',flush=True)
 send(0x80000003,0);time.sleep(.1);wait(lambda w:w[3]==11);print('CLEAR ALONE DOES NOT RECOVER',flush=True)
 send(12);time.sleep(.1);wait(lambda w:w[3]==11)
 send(13);wait(lambda w:w[3]==2,10);print('FAULT RESET/HOME RECOVERY DONE',flush=True)
 print('PASS: real-state sequence through HMI task, no forced state completions')
except Exception as e:
 print('FAIL:',e);raise SystemExit(1)
finally:
 p.terminate();p.wait(timeout=5);log.seek(0);text=log.read();print(text[-4000:]);log.close()
 # Relay commands are observable at the simulated output, not just state flags.
 if 'wire_feed=ON' not in text:raise SystemExit('FAIL: production never enabled wire feed')
