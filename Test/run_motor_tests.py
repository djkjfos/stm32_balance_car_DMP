"""Compile motor.c against native TIM/GPIO mocks and run its unit tests."""
from pathlib import Path
import argparse
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='gcc')
parser.add_argument('--output', default='test_motor.exe')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
cmd = [args.cc, '-std=c99', '-Wall', '-Wextra',
       '-I'+str(root/'Test/host'), '-I'+str(root/'Core/user'),
       str(root/'Core/user/motor.c'), str(root/'Test/test_motor.c'),
       '-o', str(Path(args.output).resolve())]
if sys.platform != 'win32':
    cmd += ['-lm']
subprocess.run(cmd, check=True)
subprocess.run([str(Path(args.output).resolve())], check=True)
