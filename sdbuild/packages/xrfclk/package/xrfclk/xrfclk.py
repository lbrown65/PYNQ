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
import struct
from pathlib import Path
from collections import defaultdict

__author__ = "Yun Rock Qu, Lewis Brown"
__copyright__ = "Copyright 2021, Xilinx"
__email__ = "pynq_support@xilinx.com"


board = os.environ['BOARD']

_Config = defaultdict(dict)

lmk_address = 'None'
lmx_address = 'None'
num_bytes = 'None'
lmk_compatible = 'None'
lmx_compatible = 'None'

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
    
    global num_bytes
        
    with open(spi_address, 'rb+', buffering=0) as f:
        for v in reg_vals:
            data = struct.pack('>I', v)
            if num_bytes == 3:
                f.write(data[1:])
            else:
                f.write(data)
    
def write_LMX_regs(reg_vals, spi_address):
    """Write values to the LMX registers.

    This is an internal function.

    Parameters
    ----------
    reg_vals: list
        A list of 113 32-bit register values.

    """
    
    with open(spi_address, 'rb+', buffering=0) as f:
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
    LMX_freq: float
        The frequency for the LMX PLL chip.

    """
        
    if LMX_freq not in _Config[lmx_compatible]:
        raise RuntimeError("Frequency {} MHz is not valid.".format(LMX_freq))
    else:
        write_LMX_regs(_Config[lmx_compatible][LMX_freq], spi_address)
        
def set_LMK_clks(LMK_freq, spi_address):
    """Set LMK chip frequency.

    Parameters
    ----------
    LMK_freq: float
        The frequency for the LMK chip.

    """
    
    if LMK_freq not in _Config[lmk_compatible]:
        raise RuntimeError("Frequency {} MHz is not valid.".format(LMK_freq))
    else:
        write_LMK_regs(_Config[lmk_compatible][LMK_freq], spi_address)
        
def _get_spidev_path(dev):
    spidev = list(dev.glob('spidev/*'))[0]
    return Path('/dev') / spidev.name

def _spidev_bind(dev):
    (dev / 'driver_override').write_text('spidev')
    Path('/sys/bus/spi/drivers/spidev/bind').write_text(dev.name)
    
        
def _find_spi_address():
    global lmk_address, lmx_address, num_bytes, lmk_compatible, lmx_compatible
    
    lmk = []
    lmx = []
    
    for dev in Path('/sys/bus/spi/devices').glob('*'):
        compatible = (dev / 'of_node' / 'compatible').read_text()[3:-1]
        
        if compatible[:3] != 'lmk' and compatible[:3] != 'lmx':
            continue
        else:
            if (dev / 'driver').exists():
                (dev / 'driver' / 'unbind').write_text(dev.name)
            
            _spidev_bind(dev)

            if compatible[:3] == 'lmk':
                lmk.append(_get_spidev_path(dev))
                num_bytes = struct.unpack('>I', (dev / 'of_node' / 'num_bytes').read_bytes())[0]
                lmk_compatible = compatible
            else:
                # if not an LMK, is an LMX
                lmx.append(_get_spidev_path(dev))
                lmx_compatible = compatible
                       
    lmk_address = lmk
    lmx_address = lmx
    
    # Need error statement if lmk/lmx still 'None'

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
    global lmk_address, lmx_address
    
    if lmk_address == 'None' and lmx_address == 'None':
        _find_spi_address()
        read_tics_output()
        
    for lmk in lmk_address:
        set_LMK_clks(lmk_freq, lmk)
    
    for lmx in lmx_address:
        set_LMX_clks(lmx_freq, lmx)


def read_tics_output():
    """Read all the TICS register values from all the txt files.

    Reading all the configurations from the current directory. We assume the
    file has a format `CHIPNAME_frequency.txt`.

    """
    dir_path = os.path.dirname(os.path.realpath(__file__))
    all_txt = glob.glob(os.path.join(dir_path, '*.txt'))
    
    for s in all_txt:
        chip, freq = s.lower().split('/')[-1].strip('.txt').split('_')
        
        if chip == lmk_compatible or chip == lmx_compatible: 
            with open(s, 'r') as f:
                lines = [l.rstrip("\n") for l in f]
                
                registers = []
                for i in lines:
                    m = re.search('[\t]*(0x[0-9A-F]*)', i)
                    registers.append(int(m.group(1), 16),)
                    
            _Config[chip][float(freq)] = registers