"""Compile the real DMP driver, port and attitude adapter against mock I2C."""
from pathlib import Path
import argparse
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='gcc')
parser.add_argument('--output', default='test_dmp.exe')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
user = root / 'Core/user'
files = ['inv_mpu.c', 'inv_mpu_dmp_motion_driver.c', 'dmp_platform.c', 'dmp_attitude.c']
cmd = [args.cc, '-std=c11', '-DSTM32_HAL', '-DMPU6050', '-I'+str(root/'Test/host'), '-I'+str(user)]
cmd += [str(user / f) for f in files]
cmd += [str(root/'Test/test_dmp.c'), '-o', str(Path(args.output).resolve())]
if sys.platform != 'win32': cmd += ['-lm']
subprocess.run(cmd, check=True)
subprocess.run([str(Path(args.output).resolve())], check=True)
