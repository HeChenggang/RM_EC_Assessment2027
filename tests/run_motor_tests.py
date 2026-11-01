"""Run Cortex-M4 driver tests without hardware (pip install unicorn==2.1.4).

Requires arm-none-eabi-gcc, objcopy and nm on PATH. HAL doubles verify software
behavior only; they cannot validate physical CAN timing, wiring or ACKs.
"""
import pathlib
import subprocess
import tempfile
import sys

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_SP, UC_ARM_REG_PC

ROOT = pathlib.Path(__file__).resolve().parents[1]
FW = ROOT / "firmware/chassis"
REMOTE = "--remote" in sys.argv
PID = "--pid" in sys.argv
MODULE = "control/pid" if PID else ("remote/dr16" if REMOTE else "motor/m3508_can")
TEST = "test_pid.c" if PID else ("test_dr16.c" if REMOTE else "test_m3508_can.c")


def run(*args):
    return subprocess.check_output([str(a) for a in args], text=True)


with tempfile.TemporaryDirectory(prefix="m3508-test-") as directory:
    elf = pathlib.Path(directory) / "test.elf"
    binary = pathlib.Path(directory) / "test.bin"
    includes = [ROOT / "modules/motor", ROOT / "modules/remote", ROOT / "modules/control", FW / "Core/Inc",
                FW / "Drivers/STM32F4xx_HAL_Driver/Inc",
                FW / "Drivers/CMSIS/Include",
                FW / "Drivers/CMSIS/Device/ST/STM32F4xx/Include"]
    run("arm-none-eabi-gcc", "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=soft",
        "-O1", "-g", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-DUSE_HAL_DRIVER", "-DSTM32F407xx", "-nostdlib",
        "-Wl,-Ttext=0x10000", "-Wl,-e,main",
        *[f"-I{p}" for p in includes], ROOT / f"modules/{MODULE}.c",
        ROOT / f"tests/{TEST}", "-lc", "-lgcc", "-o", elf)
    run("arm-none-eabi-objcopy", "-O", "binary", elf, binary)
    symbols = run("arm-none-eabi-nm", elf)
    entry = next(int(line.split()[0], 16) for line in symbols.splitlines()
                 if line.split()[-1] == "main")
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.mem_map(0x10000, 0x200000)  # Code, data and zero-initialized BSS.
    cpu.mem_write(0x10000, binary.read_bytes())
    cpu.mem_map(0x20000000, 0x10000)
    cpu.mem_map(0x40000000, 0x40000)  # UART status/data register reads during recovery.
    cpu.reg_write(UC_ARM_REG_SP, 0x20010000)
    stop = 0x1f0000
    cpu.reg_write(UC_ARM_REG_LR, stop | 1)
    cpu.emu_start(entry | 1, stop, count=10000000 if PID else 1000000)
    if cpu.reg_read(UC_ARM_REG_PC) != stop:
        raise SystemExit("FAIL: test did not return within instruction budget")
    result = cpu.reg_read(UC_ARM_REG_R0)
    if result:
        raise SystemExit(f"FAIL: {TEST} line {result}")
    print(f"PASS: {TEST}")
