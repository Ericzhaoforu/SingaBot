#!/usr/bin/env python3

import os
import pathlib
import shutil
import sys

# modify our search path:
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_ChibiOS/hwdef/scripts'))
import chibios_hwdef  # noqa


class GenerateBoardPage():
    '''an object that will create many html files into a directory, one
    for each hwdef board'''

    def __init__(self):
        self.hwdef_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..", "..", "libraries", "AP_HAL_ChibiOS", "hwdef")
        self.outdir = "/tmp/some_html"

    def generate_UART_mapping(self, hwdef):
        uart_order = hwdef.get_config('SERIAL_ORDER', required=False, aslist=True)
        if uart_order is None:
            return None
        serial_purpose = hwdef.get_serial_purpose()
        count = 0
        ret = ""
        for uart in uart_order:
            purpose = serial_purpose.get(count, "")
            if purpose != "":
                purpose = f" ({purpose})"
            rtsctsnote = ""
            if hwdef.serial_has_cts_rts(count):
                rtsctsnote = " (RTS/CTS pins)"
            ret += f"SERIAL{count} -> {uart}{purpose}{rtsctsnote}\n"
            count += 1

        return ret

    def generate_Specifications(self, hwdef):
        ret = ""

        # baro_list contains all possible baros, not the onnes you
        # might actually find on a board...
        return None

        dev_counts = {}
        for baro in hwdef.baro_list:
            dev = baro[0]
            if dev not in dev_counts.keys():
                dev_counts[dev] = 0
            dev_counts[dev] += 1
        for dev in dev_counts.keys():
            ret += f" - {dev}"
            if dev_counts[dev] > 1:
                ret += f" * {dev_counts[dev]}"
            ret += "\n"

        return ret

    def content_section(self, section_name, content):
        if content is None:
            return "bob"
        return f"""
{"=" * len(section_name)}
{section_name}
{"=" * len(section_name)}

{content}
"""

    def generate_content_for_hwdef(self, filepath):
        hwdef = chibios_hwdef.ChibiOSHWDef(quiet=True, hwdef=None)
        hwdef.process_file(filepath)

        content = ""

        content += self.content_section("UART Mapping", self.generate_UART_mapping(hwdef))
        content += self.content_section("Specifications", self.generate_Specifications(hwdef))

        return content

    def run(self):
        try:
            shutil.rmtree(self.outdir)
        except FileNotFoundError:
            pass
        pathlib.Path(self.outdir).mkdir(parents=True, exist_ok=True)

        for adir in os.listdir(self.hwdef_dir):
            if adir is None:
                continue
            if adir != "Pixhawk6X":
                continue
            board_dirpath = os.path.join(self.hwdef_dir, adir)
            filepath = os.path.join(board_dirpath, "hwdef.dat")
            if not os.path.exists(filepath):
                continue

            output = self.generate_content_for_hwdef(filepath)

            outfile = os.path.join(self.outdir, f"{adir}.rst")
            pathlib.Path(outfile).write_text(output)


gbp = GenerateBoardPage()
gbp.run()
