#!/usr/bin/env python
#
# File: $Id$
#
"""
Read from an XGPS 160 (Dualav SkyPro GPS).. just trying to read
anything from it to see if I have the protocol correct.
"""

# system imports
#
import asyncio
import sys
import time
import packbits
import pathlib
from typing import Optional
from enum import IntEnum

# 3rd party imports
#
import serial_asyncio
from rich.console import Console

DEVICES_DIR = "/dev/"
XGPS_SERIAL_PATERN = "tty.XGPS160*"


####################################################################
#
def xgps_serialport():
    """
    Find the XGPS serial port. We only expect one so we are going to
    search and return the first tty that matches the pattern.
    """
    serial_ports = sorted(pathlib.Path(DEVICES_DIR).glob(XGPS_SERIAL_PATERN))
    if serial_ports:
        return serial_ports[0]
    raise FileNotFoundError(f"Found no XGPS serialport in {DEVICES_DIR}")


########################################################################
########################################################################
#
class Cmd160(IntEnum):
    """
    The possible commands for the XGPS160.
    """
    ack = 0
    nack = 1
    response = 2
    fwRsp = 3
    fwData = 4
    fwDataR = 5
    fwErase = 6
    fwUpdate = 7
    fwBDADDR = 8
    fwCancel = 9
    streamStop = 10
    streamResume = 11
    logDisable = 12
    logEnable = 13
    logOneshot = 14
    logPause = 15
    logResume = 16
    logInterval = 17
    logOWEnable = 18
    logOWDisable = 19
    getSettings = 20
    logReadBulk = 21
    logList = 22
    logListItem = 23
    logRead = 24
    logDelBlock = 25
    resetSettings = 26
    fwVersion = 27
    recentList = 28
    recentDel = 29


########################################################################
########################################################################
#
class XGPS160(asyncio.Protocol):
    """
    Mimicing the example XGPS160 interface project we have a class
    that is the interface to the XGPS160.

    A number of values were copied straight from the objective C
    example project without thinking too much about whether we need
    them or not. If we do not we can always delete them later.
    """

    ####################################################################
    #
    def __init__(self, serial_port: str, console: Optional[Console]):
        self.serial_port = serial_port
        self.console = console

        self.rx_sync = False
        self.rx_bin_sync = False
        self.rx_idx = 0
        self.rx_bin_len = 0
        self.rx_bytes_count = 0
        self.rx_bytes_total = 0
        self.rx_messages_total = 0

        self.pkt_buffer = bytearray(10)  # Length should be.. 4?

        self.rsp160_cmd = 0
        self.rsp160_buf = bytearray(256)
        self.rsp160_len = 0

        self.is_connected = False

        self.transport = None
        self.firmware_rev = ""
        self.serial_number = ""
        self.battery_voltage = 0
        self.is_charging = False
        self.streaming_mode = True

        self.log_list_entries = []
        self.log_data_samples = []

        self.log_list_item_timer_started = False
        self.new_log_list_item_received = False

        self.device_settings_have_been_read = False

        self.total_GPS_samples_in_log_entry = 0

        # This is an event that we can wait on for data to be received. If we
        # have not received enough data for a full command we just go back to
        # waiting.
        #
        self.awaiting_data = asyncio.Event()

        self.incoming_data = []

    ####################################################################
    #
    @classmethod
    async def connect(
        cls,
        serial_port: str,
        loop: Optional[asyncio.AbstractEventLoop],
        console: Optional[Console],
    ):
        """
        Creates the XGPS160 object and connects to the XGPS160.
        """
        if loop is None:
            loop = asyncio.get_event_loop()

        _, xgps_160 = await serial_asyncio.create_serial_connection(
            loop,
            lambda: cls(serial_port, console=console),
            serial_port,
            baudrate=115200,
        )
        return xgps_160

    ####################################################################
    #
    def connection_made(self, transport):
        print("Connected!")
        self.transport = transport
        self.is_connected = True

    ####################################################################
    #
    def connection_lost(self, exc):
        print(f"Connetion lost: {exc}")
        self.transport = None
        self.is_connected = False

    ####################################################################
    #
    def data_received(self, data):
        print("data received", repr(data))

        for x in data:
            if self.rx_bin_sync:
                self.pkt_buffer[self.rx_index] = x
                self.rx_index += 1
                match self.rx_index:
                    case 2:  # Second marker
                        if x != 0xEE:
                            self.rx_bin_sync = False
                    case 3:  # Length of packet
                        self.rx_bin_len = x

                if self.rx_index == (self.rx_bin_len + 4):
                    self.parse_command_response()
                    self.rx_bin_sync = False

                continue

            if x == 0x88:
                self.rx_bin_sync = True
                self.rx_bin_len = 0
                self.rx_idx = 1
                self.pkt_buffer[0] = x
                continue

            if not self.rx_sync:
                if x in [b"P", b"N", b"L", b"@"]:
                    self.rx_sync = True
                    self.rx_idx = 0
                    self.pkt_buffer[0] = x
            else:
                self.rx_idx += 1
                self.pkt_buffer[self.rx_idx] = x

                if x == b"\n":
                    self.rx_messages_total += 1
                    self.rx_sync = False
                    # self.pkt_buffer[self.rx_idx + 1] = 0
                    nmea_packet = str(self.pkt_buffer[: self.rx_idx], encoding="tuf-8")
                    self.parse_nmea(nmea_packet)

    ####################################################################
    #
    def pause_writing(self):
        """
        The underlying connection's buffer is filling up and it is telling
        us to stop writing for awhile.
        """
        print("Please pause writing!")

    ####################################################################
    #
    def resume_writing(self):
        """
        The underlying connection's buffer once again has room for more
        data, so we can resume writing data.
        """
        print("Please resume writing!")

    ####################################################################
    #
    def eof_received(self):
        print("EOF Received!")
        self.is_connected = False
        self.transport = None

    ####################################################################
    #
    def parse_command_response(self):
        """
        """
        size = self.rx_bin_len + 3
        cksum = sum(self.pkt_buffer[:size])

        if cksum != self.pkt_buffer[self.rx_bin_len + 3]:
            print(f"Checksum error on {self.pkt_buffer[:size]} ({self.pkt_buffer[size]})")
            return

        match self.pkt_buffer[3]:
            case Cmd160.ack | Cmd160.nack:
                print("Command: ACK or NACK")
                self.rsp160_cmd = self.pkt_buffer[3]
                self.rsp160_len = 0

            case Cmd160.fwRsp:
                print("Command fw response")
                self.rsp160_cmd = self.pkt_buffer[3]
                self.rsp160_buf[0:3] = self.pkt_buffer[4:7]
                self.rsp160_len = self.rx_bin_len

                if self.pkt_buffer[4] == Cmd160.getSettings:
                    print("XGPS160 sending settings.")
                    blk = self.pkt_buffer[8]
                    blk <<= 8
                    blk |= self.pkt_buffer[7]

    ####################################################################
    #
    def parse_nmea(self, nmea_packet: str):
        """
        Keyword Arguments:
        packet: bytes --
        """
        print(f"NMEA Packet: {nmea_packet}")


#############################################################################
#
async def main():
    """
    Find the XGPS 160 serial port. Listen for messages from it.
    """
    loop = asyncio.get_event_loop()
    xgps = XGPS160(xgps_serialport())
    await xgps.connect()

    status = await xgps.device_settings()

    return


############################################################################
############################################################################
#
# Here is where it all starts
#
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()

#
############################################################################
############################################################################
