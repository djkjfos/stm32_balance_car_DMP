"""Run actual ST7789 driver against an asynchronous DMA/LCD mock."""
from pathlib import Path
import argparse
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='gcc')
parser.add_argument('--output', default='test_st7789.exe')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
output = Path(args.output).resolve()
subprocess.run([args.cc, '-std=c99', '-I'+str(root/'Test/lcd_host'), '-I'+str(root/'Core/user'),
                str(root/'Core/user/st7789.c'), str(root/'Test/test_st7789.c'), '-o', str(output)], check=True)
subprocess.run([str(output)], check=True)
