#   Copyright (c) 2021, Xilinx, Inc.
#   All rights reserved.
# 
#   Redistribution and use in source and binary forms, with or without 
#   modification, are permitted provided that the following conditions are met:
#
#   1.  Redistributions of source code must retain the above copyright notice, 
#       this list of conditions and the following disclaimer.
#
#   2.  Redistributions in binary form must reproduce the above copyright 
#       notice, this list of conditions and the following disclaimer in the 
#       documentation and/or other materials provided with the distribution.
#
#   3.  Neither the name of the copyright holder nor the names of its 
#       contributors may be used to endorse or promote products derived from 
#       this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
#   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
#   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import os
import glob
import re
import cffi
import struct
import time
from pathlib import Path
from collections import defaultdict


__author__ = "Yun Rock Qu, Lewis Brown"
__copyright__ = "Copyright 2021, Xilinx"
__email__ = "pynq_support@xilinx.com"


board = os.environ['BOARD']

_lmx2594Config = defaultdict(list)
_lmk04208Config= defaultdict(list)
_lmk04832Config = defaultdict(list)


def write_LMK_regs(reg_vals, spi_address):
    """Write values to the LMK registers.

    This is an internal function.

    Parameters
    ----------
    reg_vals: list
        A list of 32-bit register values (LMK clock dependant number of values).
        LMK04208 = 32 registers
        LMK04832 = 125 registers

    """
    path = os.path.join('/dev/', spi_address)
    
    with open(path, 'wb', buffering=0) as f:
        for v in reg_vals:
            data = struct.pack('>I', v)
            f.write(data[1:])
    
def write_LMX_regs(reg_vals, spi_address):
    """Write values to the LMX registers.

    This is an internal function.

    Parameters
    ----------
    reg_vals: list
        A list of 113 32-bit register values.

    """
    
    path = os.path.join('/dev/', spi_address)
    
    with open(path, 'wb', buffering=0) as f:
        # Program RESET = 1 to reset registers.
        reset = struct.pack('>I', 0x020000)
        f.write(reset[1:])
        
        # Program RESET = 0 to remove reset.
        remove_reset = struct.pack('>I', 0)
        f.write(remove_reset[1:])
        
        # Program registers as shown in the register map in REVERSE order from highest to lowest
        for v in reg_vals:
            data = struct.pack('>I', v)
            f.write(data[1:])
            
        # Program register R0 one additional time with FCAL_EN = 1 
        # to ensure that the VCO calibration runs from a stable state.
        stable = struct.pack('>I', reg_vals[112])
        f.write(stable[1:])
        
        
def set_LMX_clks(LMX_freq, spi_address):
    """Set LMX chip frequency.

    Parameters
    ----------
    lmx_freq: float
        The frequency for the LMX PLL chip.

    """
    if LMX_freq not in _lmx2594Config:
        raise RuntimeError("Frequency {} MHz is not valid.".format(LMX_freq))
    else:
        write_LMX_regs(_lmx2594Config[LMX_freq], spi_address)
        
def set_LMK_clks(LMK_freq, spi_address):
    """Set LMK chip frequency.

    Parameters
    ----------
    lmx_freq: float
        The frequency for the LMX PLL chip.

    """
    if board == "ZCU111":
        if LMK_freq not in _lmk04208Config:
            raise RuntimeError("Frequency {} MHz is not valid.".format(LMK_freq))
        else:
            write_LMK_regs(_lmk04208Config[LMK_freq], spi_address)
    elif board == "RFSoC2x2":
        if LMK_freq not in _lmk04832Config:
            raise RuntimeError("Frequency {} MHz is not valid.".format(LMK_freq))
        else:
            write_LMK_regs(_lmk04832Config[LMK_freq], spi_address)
    else:
        raise ValueError("Board {} is not supported.".format(board))
        
def find_spi_address():

    spidevs = list(Path('/sys/bus/spi/devices').glob('*'))
    clock_name = list(Path('/sys/bus/spi/devices').glob('*'))

    for i in range(3):
        
        name_path = os.path.join(spidevs[i], 'of_node', 'name')
        clock_name[i] = Path(name_path).read_text()
        clock_name[i] = clock_name[i][:-1]
        
        Path('/sys/bus/spi/drivers/spidev/unbind').write_text(spidevs[i].name)
        (spidevs[i] / 'driver_override').write_text('spidev')
        Path('/sys/bus/spi/drivers/spidev/bind').write_text(spidevs[i].name)
        
        if clock_name[i] == 'lmxadc':
            lmxadc_address = spidevs[i].name
        elif clock_name[i] == 'lmxdac':
            lmxdac_address = spidevs[i].name
        elif clock_name[i] == 'lmk':
            lmk_address = spidevs[i].name
        else:
            raise ValueError("Clock {} is not supported.".format(clock_name[i]))
            
    spi_address = [lmk_address, lmxadc_address, lmxdac_address]
            
    return spi_address


def set_ref_clks(lmk_freq=122.88, lmx_freq=409.6):
    """Set all RF data converter tile reference clocks to a given frequency.

    LMX chips are downstream so make sure LMK chips are enabled first.

    Parameters
    ----------
    lmk_freq: float
        The frequency for the LMK clock generation chip.
    lmx_freq: float
        The frequency for the LMX PLL chip.

    """
    spi_address = find_spi_address()
    
    read_tics_output()
    set_LMK_clks(lmk_freq, spi_address[0])
    set_LMX_clks(lmx_freq, spi_address[1])
    set_LMX_clks(lmx_freq, spi_address[2])


def read_tics_output():
    """Read all the TICS register values from all the txt files.

    Reading all the configurations from the current directory. We assume the
    file has a format `CHIPNAME_frequency.txt`.

    """
    dir_path = os.path.dirname(os.path.realpath(__file__))
    all_txt = glob.glob(os.path.join(dir_path, '*.txt'))
    for s in all_txt:
        chip, freq = s.lower().split('/')[-1].strip('.txt').split('_')
        config = eval('_{}Config'.format(chip))
        with open(s, 'r') as f:
            lines = [l.rstrip("\n") for l in f]
            for i in lines:
                m = re.search('[\t]*(0x[0-9A-F]*)', i)
                config[float(freq)] += int(m.group(1), 16),

