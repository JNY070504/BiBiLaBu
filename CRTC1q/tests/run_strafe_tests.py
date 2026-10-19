"""Host regression tests: mocks peripherals, compiles actual control/driver C."""
from pathlib import Path
import subprocess, tempfile, zipfile, hashlib, shutil

p=Path(__file__).resolve().parents[1]
gcc=shutil.which('gcc')
if not gcc: raise SystemExit('Host gcc is required')
def run(args):
    result=subprocess.run([str(a) for a in args],capture_output=True)
    if result.returncode:
        raise SystemExit(result.stdout.decode(errors='replace')+result.stderr.decode(errors='replace'))
    return result.stdout
with tempfile.TemporaryDirectory(prefix='crtc_strafe_') as temp:
    t=Path(temp)
    # Use the actual axis mapping function, without MCU SPI implementation.
    axis=(p/'Core/Src/ps2.c').read_text(encoding='utf-8-sig').split('int16_t PS2_AxisMap(',1)[1]
    (t/'axis.c').write_text('#include "ps2.h"\nint16_t PS2_AxisMap('+axis,encoding='utf-8')
    def build(name,control,defines=()):
        exe=t/(name+'.exe')
        run([gcc,'-std=c99','-Wall','-Wextra','-Werror',
             '-include',p/'tests/host_hal.h','-I'+str(p/'Core/Inc'),
             *defines,p/'tests/test_strafe.c',control,p/'Core/Src/drv8833.c',
             t/'axis.c','-o',exe])
        return exe
    current=build('current',p/'Core/Src/ps2_ctrl.c')
    print(run([current]).decode().strip())
    for wheel in ['LF','LR','RF','RR']:
        exe=build('reverse_'+wheel,p/'Core/Src/ps2_ctrl.c',['-DDRV8833_DIR_'+wheel+'=-1'])
        run([exe])
    print('PASS: reverse each of LF/LR/RF/RR independently, other wheels unchanged')
    reference=p.parent/'CRTC2z/Core/Src/ps2_ctrl.c'
    if reference.is_file():
        exe=build('reference',reference)
        print(run([exe,'reference']).decode().strip())
    backups=sorted(p.parent.glob('CRTC1q_平移修改前_*.zip'))
    if backups:
        with zipfile.ZipFile(backups[-1]) as z:
            name=next(n for n in z.namelist() if n.replace('\\','/').endswith('/Core/Src/ps2_ctrl.c'))
            (t/'before.c').write_bytes(z.read(name))
        before=build('before',t/'before.c')
        old=run([before,'regression']);new=run([current,'regression'])
        assert old==new,'Forward/turn behavior changed unexpectedly'
        print('PASS: 12000 forward/turn samples match pre-edit CRTC1q exactly; SHA256='+hashlib.sha256(new).hexdigest())
    else:
        print('NOTE: pre-edit backup missing; baseline comparison skipped')
