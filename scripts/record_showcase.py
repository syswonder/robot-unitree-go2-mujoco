#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Record an operator-arranged X11 viewer/terminal while executing real RPCs."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--display', default=':1+80,90')
    parser.add_argument('--size', default='2440x1372')
    parser.add_argument('--output', type=Path, default=ROOT/'.runtime/videos')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%dT%H%M%S')
    video = args.output / ('Go2-Robonix-Courtyard-'+stamp+'.mp4')
    print('\033[H\033[2JROBONIX x GO2\n六区域完整演示录制\n\n'
          '平地机动 / 跨栏 / 绕桩\n楼梯 / 坡道 / 节律舞蹈\n'
          '后空翻 / 倒立行走\n\n'
          'Atlas -> Primitive RPC\n-> 关节力矩 -> MuJoCo\n\n'
          '请确认取景范围只有仿真和演示终端。\n'
          '按 Enter 开始录制和完整流程。\nCtrl+C 停止。', flush=True)
    input()
    capture = None
    code = None
    started = datetime.now().isoformat()
    try:
        with video.with_suffix('.capture.log').open('x') as log:
            capture = subprocess.Popen([
                'ffmpeg','-n','-hide_banner','-loglevel','warning',
                '-f','x11grab','-framerate','30','-video_size',args.size,'-i',args.display,
                '-vf','scale=1920:1080:force_original_aspect_ratio=decrease:flags=lanczos,'
                      'pad=1920:1080:(ow-iw)/2:(oh-ih)/2,setsar=1',
                '-c:v','libx264','-preset','veryfast','-crf','18','-threads','4',
                '-pix_fmt','yuv420p','-profile:v','high','-level:v','4.1',
                '-movflags','+faststart','-an',str(video)],
                stdin=subprocess.PIPE, stdout=log, stderr=log)
            time.sleep(1)
            if capture.poll() is not None:
                raise RuntimeError('Recorder could not start; inspect capture.log')
            code = subprocess.call(['bash',str(ROOT/'scripts/showcase.sh'),'--auto','--live'])
            time.sleep(3)
    finally:
        if capture and capture.poll() is None:
            capture.communicate(b'q', timeout=30)
        report = {'started':started,'video':str(video),'showcase_exit_code':code,
                  'capture_exit_code':capture.returncode if capture else None,
                  'complete':code == 0 and capture is not None and capture.returncode == 0}
        video.with_suffix('.recording.json').write_text(json.dumps(report, indent=2))
        print('\n录制文件：',video,flush=True)
    return 0 if report['complete'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
