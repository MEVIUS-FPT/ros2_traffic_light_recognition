import fnmatch
import functools
import json
import logging
import re
import struct
import time

import numpy as np
import pynq.lib.dma
from ament_index_python.packages import get_package_share_directory
from pynq import MMIO, Overlay, allocate
from pynq.pl_server import hwh_parser


class FpgaDriver:
    def __init__(self, user_ip):
        self.logger = logging.getLogger("PYNQ-Driver")
        logger_handler = logging.StreamHandler()
        logger_handler.setFormatter(
            logging.Formatter("[%(levelname)s] [%(name)s]: %(message)s")
        )
        self.logger.addHandler(logger_handler)

        # FPGA bit file
        self.bit_file = "/home/ubuntu/dev_ws/traffic_light_recognition.bit"
        self.ovl = Overlay(self.bit_file, download=False)
        self.ovl.is_loaded = self.is_loaded
        # User IP
        self.user_ip = user_ip

    def is_loaded(self):
        return True

    def update_map(self, user_ip):
        if not user_ip:
            return
        io_maps_json = open(
            get_package_share_directory("traffic_light_recognition_fpga_node") + "/io_maps.json",
            "r",
        )
        io_maps = json.load(io_maps_json)
        io_maps_json.close()
        self.map_num_str = str(io_maps["map_num"][user_ip])

        # Define the arrays that hold the information about the input and output
        # signals. If the value corresponds to a double or float variable, it is
        # in IEEE-754 representation
        self.in_map = io_maps["maps"][self.map_num_str]["input"]
        self.out_map = io_maps["maps"][self.map_num_str]["output"]
        self.has_axis_in = self.has_axis(self.in_map)
        self.has_axis_out = self.has_axis(self.out_map)
        if self.has_axis_in or self.has_axis_out:
            self.dma_name = self.find_dma_ip_name()

    def find_dma_ip_name(self):
        hwh_file = hwh_parser.get_hwh_name(self.bit_file)
        hwh = hwh_parser.HWH(hwh_file)
        pins = fnmatch.filter(hwh.pins.keys(), "{}/*TDATA".format(self.user_ip))
        for pin in pins:
            dma_match = re.search(r"axi_dma_\d+", hwh.pins[pin])
            if dma_match is not None:
                return dma_match.group()
                

    @property
    def user_ip(self):
        return self.__user_ip

    @user_ip.setter
    def user_ip(self, ip_name):
        self.__user_ip = ip_name
        self.update_map(ip_name)

    def program_fpga(self):
        # Program the FPGA and set the Overlay
        print(f"Downloading bit file {self.bit_file}...")
        self.ovl.download()
        print("Bit file downloaded")

    def setup_fpga(self):
        # Set the MMIO and DMA classes
        regs = self.ovl.ip_dict[self.user_ip]
        phys_addr = regs["phys_addr"]
        addr_range = regs["addr_range"]
        print("User IP phys_addr = 0x{:X}".format(phys_addr))
        print("User IP addr_range = {}".format(addr_range))
        self.mmio = MMIO(phys_addr, addr_range)
        if self.has_axis_in or self.has_axis_out:
            self.dma = getattr(self.ovl, self.dma_name)
        if self.has_axis_in:
            self.input_buffer = self.allocate_dma_buffer(self.in_map)
        if self.has_axis_out:
            self.output_buffer = self.allocate_dma_buffer(self.out_map)

    def has_axis(self, io_map):
        has_axis = False
        for signal_name in io_map.keys():
            has_axis |= io_map[signal_name]["protocol"] == "stream"
        return has_axis

    def allocate_dma_buffer(self, io_map):
        signal_type = ""
        n_bits = 0
        arr_size = 0
        for signal_name in io_map.keys():
            if io_map[signal_name]["protocol"] == "stream":
                signal_type = io_map[signal_name]["type"]
                n_bits = io_map[signal_name]["n_bits"]
                arr_size = io_map[signal_name]["n_elem"]
                break

        # case 1 - int32
        if signal_type == "int" and n_bits == 32:
            allocated_buffer = allocate(shape=(arr_size,), dtype=np.int32)
        # case 2 - uint8
        if signal_type == "int" and n_bits == 8:
            allocated_buffer = allocate(shape=(arr_size,), dtype=np.uint8)
        # case 3 - uint64
        if signal_type == "int" and n_bits == 64:
            allocated_buffer = allocate(shape=(arr_size,), dtype=np.uint64)
        # case 4 - float32
        if signal_type == "float" and n_bits == 32:
            allocated_buffer = allocate(shape=(arr_size,), dtype=np.float32)
        return allocated_buffer

    def process_input(self, signal_name, msg):
        # Receive the value from the msg and send it to the FPGA
        # Need to figure out if the input is an array, and the input type
        signal_type = self.in_map[signal_name]["type"]
        is_arr = self.in_map[signal_name]["arr"]
        n_bits = self.in_map[signal_name]["n_bits"]
        protocol = self.in_map[signal_name]["protocol"]

        if protocol == "lite":

            # case 1 - Signal represents an array
            if is_arr:
                arr_head = self.in_map[signal_name]["addr"]
                arr_size = self.in_map[signal_name]["n_elem"]
                if signal_type == "int":
                    if n_bits == 32:  # int32
                        # 1 array element per address
                        for i in range(arr_size):
                            self.mmio.write(arr_head + 4 * i, int(msg[i]))
                    elif n_bits == 8:  # uint8
                        # 4 array elements per address
                        byte_arr = bytes(msg)
                        lower_lim = 0
                        upper_lim = 4
                        n_exact = arr_size // 4
                        n_last = arr_size % 4
                        padding = 4 - n_last
                        addr = arr_head
                        for i in range(n_exact):
                            partition = byte_arr[lower_lim:upper_lim]
                            self.mmio.write(addr, partition)
                            lower_lim += 4
                            upper_lim += 4
                            addr += 4
                        if n_last != 0:
                            partition = byte_arr[arr_size - n_last : arr_size]
                            pad = bytes(padding)
                            partition += pad
                            self.mmio.write(addr, partition)
                elif signal_type == "float":
                    if n_bits == 32:  # Float
                        # 1 array element per address
                        for i in range(arr_size):
                            ieee_rep = self.dec_to_ieee(msg[i], "float")
                            self.mmio.write(arr_head + 4 * i, ieee_rep)
                    elif n_bits == 64:  # Double
                        # 1 array element per 2 addresses
                        for i in range(arr_size):
                            ieee_rep_hi, ieee_rep_lo = self.dec_to_ieee(msg[i], "double")
                            self.mmio.write(arr_head + 8 * i, ieee_rep_lo)
                            self.mmio.write(arr_head + 8 * i + 4, ieee_rep_hi)

            # case 2 - Signal does not represent an array
            else:
                addr = self.in_map[signal_name]["addr"]
                if signal_type == "int":
                    self.mmio.write(addr, int(msg))
                elif signal_type == "float":
                    if n_bits == 32:  # Float
                        ieee_rep = self.dec_to_ieee(msg, "float")
                        self.mmio.write(addr, ieee_rep)
                    elif n_bits == 64:  # Double
                        ieee_rep_hi, ieee_rep_lo = self.dec_to_ieee(msg, "double")
                        self.mmio.write(addr, ieee_rep_lo)
                        self.mmio.write(addr + 4, ieee_rep_hi)

        elif protocol == "stream":
            self.input_buffer[:] = msg[:]

    def process_output(self, signal_name):
        # Read the value from the FPGA and return it in a format that can be assigned to a msg
        # Need to figure out if the output is an array, and the output type
        signal_type = self.out_map[signal_name]["type"]
        is_arr = self.out_map[signal_name]["arr"]
        n_bits = self.out_map[signal_name]["n_bits"]
        protocol = self.out_map[signal_name]["protocol"]

        if protocol == "lite":

            if is_arr:
                arr_head = self.out_map[signal_name]["addr"]
                arr_size = self.out_map[signal_name]["n_elem"]
                val = [0] * arr_size
                if signal_type == "int":
                    if n_bits == 32:  # int32
                        for i in range(arr_size):
                            val[i] = self.mmio.read(arr_head + 4 * i)
                        return val
                    elif n_bits == 8:  # uint8
                        # 4 array elements per address
                        n_exact = arr_size // 4
                        n_last = arr_size % 4
                        padding = 4 - n_last
                        addr = arr_head
                        for i in range(n_exact):
                            result = self.mmio.read(addr).to_bytes(4, "little")
                            for j in range(4):
                                val[j + i] = int(result[j])
                            addr += 4
                        if n_last != 0:
                            result = self.mmio.read(addr).to_bytes(n_last, "little")
                            for j in range(n_last):
                                val[arr_size - n_last + j] = int(result[j])
                        return val
                elif signal_type == "float":
                    if n_bits == 32:  # Float
                        for i in range(arr_size):
                            val_ieee = self.mmio.read(arr_head + 4 * i)
                            val_ieee = hex(val)[2:]
                            val[i] = self.ieee_to_dec(val_ieee, "float")
                        return val
                    elif n_bits == 64:  # Double
                        for i in range(arr_size):
                            val_ieee_lo = self.mmio.read(arr_head + 8 * i)
                            val_ieee_hi = self.mmio.read(arr_head + 8 * i + 4)
                            val_ieee = hex((val_ieee_hi << 32) | val_ieee_lo)[2:]
                            val[i] = self.ieee_to_dec(val_ieee, "double")
                        return val

        elif protocol == "stream":
            data_size_out = self.out_map[signal_name]["n_elem"]
            if self.out_map[signal_name]["n_bits"] == 64:
                val = np.zeros((data_size_out,), dtype=np.uint64)
            elif signal_type == "float":
                val = np.zeros((data_size_out,), dtype=np.float32)
            else:
                val = [0] * data_size_out
            val[:] = self.output_buffer[:]
            return val

    def setup_dma_buffers(self):
        # Setup the input and output AXI_Stream buffers for DMA transfer

        if self.has_axis_in:
            self.dma.sendchannel.transfer(self.input_buffer)
        if self.has_axis_out:
            self.dma.recvchannel.transfer(self.output_buffer)

        return

    def do_calc(self):
        # Send the AP_START flag
        self.mmio.write(0x00, 1)
        # Wait for streams, if any
        if self.has_axis_in and self.dma.sendchannel.running:
            self.dma.sendchannel.wait()
        if self.has_axis_out and self.dma.recvchannel.running:
            self.dma.recvchannel.wait()

        # Wait for the computation to be finished (AP_DONE)
        self.wait_result()

    def wait_result(self):
        # Wait for AP_DONE
        while not ((self.mmio.read(0x00) >> 1) & 0x0001):
            continue

    def dec_to_ieee(self, num, data_type):
        # Converts a decimal number to IEEE-754 representation
        if data_type == "float":
            num_ieee_hex = struct.pack(">f", num).hex()
            num_ieee = int(num_ieee_hex, 16)
            return num_ieee
        elif data_type == "double":
            num_ieee = struct.pack(">d", num).hex()
            num_hi = int(num_ieee[:8], 16)
            num_lo = int(num_ieee[8:], 16)
            return num_hi, num_lo

    def ieee_to_dec(self, num_ieee, data_type):
        # Converts a number in IEEE-754 representation to its decimal form
        if num_ieee != "0":
            num_in_bytes = bytes.fromhex(num_ieee)
            if data_type == "float":
                num_dec = struct.unpack(">f", num_in_bytes)[0]
            elif data_type == "double":
                num_dec = struct.unpack(">d", num_in_bytes)[0]
        else:
            num_dec = 0
        return num_dec
