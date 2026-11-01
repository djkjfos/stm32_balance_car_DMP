"""Test DMP controller integration using the existing mock HAL fixture."""
from pathlib import Path
import argparse
import subprocess
import sys

parser=argparse.ArgumentParser()
parser.add_argument('--cc',default='gcc')
parser.add_argument('--output',default='test_balance.exe')
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
files=['mpu6050.c','encoder.c','motor.c','pid.c','uart_cmd.c','balance.c']
cmd=[args.cc,'-std=c99','-I'+str(root/'Test/host'),'-I'+str(root/'Core/user')]
cmd += [str(root/'Core/user'/f) for f in files]
cmd += [str(root/'Test/test_dmp_controller.c'),'-o',str(Path(args.output).resolve())]
if sys.platform != 'win32':
    cmd += ['-lm']
subprocess.run(cmd,check=True)
subprocess.run([str(Path(args.output).resolve())],check=True)
