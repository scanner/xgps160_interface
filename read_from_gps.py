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
import struct
import pathlib
from typing import Optional
from enum import IntEnum
from datetime import datetime, UTC
from collections import namedtuple

# 3rd party imports
#
import serial_asyncio
from rich.console import Console

DEVICES_DIR = "/dev/"
XGPS_SERIAL_PATERN = "tty.XGPS160*"


####################################################################
#
def datetime_str(date_bin: int, time_of_day: int) -> datetime:
        year = 2012 + date_bin / 372
        month = 1 + (date_bin % 372) / 31
        day = 1 + (date_bin % 31)

        hour = time_of_day / 3600
        minute = (time_of_day % 3600) / 60
        second = time_of_day % 60

        return datetime(year=year, month=month, second=second, hour=hour, minute=minute, second=second, tzinfo=UTC)

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

LogEntryStruct = struct.Struct("!BB")
LogEntry = namedtuple("LogEntry", "seq type" )

DataEntryStruct = struct.Struct("!HHB3s3s3sHBBBB")
# NOTE: latitude, longitude, altitude are 3 byte long `bytes` type. Need to do int.from_bytes(bytes,byteorder="big",signed=True)
DataEntry = namedtuple("DataEntry","date tod tod2 latitude longitude altitude speed heading satnum satsig dop")
# NOTE: Longitutde, latitude, altitude are MSB order so need to be interprete separately with int.from_bytes()
Data2EntryStruct = struct.Struct("!HHB4s4s3sHBB")
Data2Entry = namedtuple("Data2Entry", "date tod tod2 latitutde longitude altitude speed heading satsig")

########################################################################
########################################################################
#
LogListItemStruct = struct.Struct("!BBHIHHH")
LogListItem = namedtuple("LogListItem", "sig interval start_date start_tod start_block count_entry count_block")

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

        self.cfg_gps_settings = 0
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
                    nmea_packet = str(
                        self.pkt_buffer[: self.rx_idx],
                        encoding="tuf-8",
                    )
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
        """ """
        size = self.rx_bin_len + 3
        cksum = sum(self.pkt_buffer[:size])

        if cksum != self.pkt_buffer[self.rx_bin_len + 3]:
            print(
                f"Checksum error on {self.pkt_buffer[:size]} "
                f"({self.pkt_buffer[size]})"
            )
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
                match self.pkt_buffer[4]:
                    case Cmd160.getSettings:
                        print("XGPS160 sending settings")
                        blk = self.pkt_buffer[8]
                        blk <<= 8
                        blk |= self.pkt_buffer[7]

                        offset = self.pkt_buffer[10]
                        offset <<= 8
                        offset |= self.pkt_bufferBuf[9]

                        self.cfg_gps_settings = self.pkt_buffer[5]
                        self.cfg_log_interval = self.pkt_buffer[6]
                        self.log_update_rate = self.pkt_buffer[6]
                        print(f"Log update rate is: {self.log_update_rate}")

                        self.cfg_blk = blk
                        self.cfg_log_offste = offset

                        if self.cfg_gps_settings & 0x40:
                            print("Datalog enabled")
                            self.always_record_when_device_is_on = True
                        else:
                            print("Datalog disabled")
                            self.always_record_when_device_is_on = False

                            if self.cfg_gps_settings & 0x80:
                                print("Datalog overwrite")
                                self.stop_recording_when_mem_full = False
                            else:
                                print("Datalog no overwrite")
                                self.stop_recording_when_mem_full = True
                                self.device_settings_have_been_read = True
                    case Cmd160.logListItem:
                        logs = {}
                        # XXX These two ops are just converting two
                        #     bytes in to a 16bit int.. use struct module
                        list_idx, list_total = struct.unpack_from("!HH", self.pkt_buffer, 6)
                        # There is bug in firmware v. 1.3.0. The
                        # cmd160_logList command will append a
                        # duplicate of the last long entry. For
                        # example, if there are 3 recorded logs, the
                        # command will repond that there are four: log
                        # 0, log 1, log 2 and log 2 again.
                        if list_idx == list_total:
                            list_idx = 0
                            list_total = 0
                            logs = None
                        else:
                            log_list_item = LogListItem._make(LogListItemStruct.unpack_from(pkt_buffer, 10))
                            log_start_time = datetime_str(log_list_item.start_date, log_list_item.start_tod)

                            logs["interval"] = log_list_item.interval
                            logs["count_entry"] = log_list_item.count_entry
                            sample_interval = float(log_list_item.interval)
                            if log_list_item.interval == 255:
                                sample_interval = 10.0
                                recording_length_in_sec = log_list_item.country_entry * (sample_interval / 10.0)
                                duration = timedelta(seconds=recording_length_in_sec)
                                logs["human_friendly_duration"]=str(duration)
                                logs["sig"]=log_list_item.sig
                                logs["start_time"]=log_start_time
                                logs["duration"]=duration
                                logs["start_block"] = log_list_item.start_block
                                logs["count_block"] = log_list_item.count_block
                                self.log_list_entries.append(logs)
                                self.processing_log_list_entries_after_delay()
                                self.new_log_list_item_received = True
                    case Cmd160.logReadBulk:
                        addr, data_size = struct.unpack_from("!HB", 6)
                        log_entry_size = LogEntryStruct.size + max(DataEntryStruct.size, Data2EntryStruct.size)
                        self.log_read_bulk_count = data_size / log_entry_size
                        if addr == 0 and data_size == 0:
                            # End-of-data
                            #
                            self.log_read_bulk_count |= 0x1000000
                            self.decode_log_bulk()
                            self.log_read_bulk_count = 0
                            self.log_bulk_recode_cnt = 0
                            self.log_bulk_by_cnt = 0
                            self.log_records = []
                        else:
                            # Looks like we get 5 log entries at a time.
                            for loop_idx in range(5):
                                idx = 10 + (loop_idx * log_entry_size)
                                log_entry = LogEntry._make(LogEntryStruct.unpack_from(self.pkt_buffer,idx))
                                if LogEntry.type == 1:
                                    de = DataEntry._make(DataEntryStruct.unpack_from(self.pkt_buffer,idx+LogEntryStruct.size))
                                else:
                                    de = Data2Entry._make(Data2EntryStruct.unpack_from(self.pkt_buffer,idx + LogEntryStruct.size))
                                    self.log_records.append((log_entry, de))
                    case Cmd160.logDelBlock:
                    if self.pkt_buffer[5] != 0x01:
                        print("Error deleting block data")
                    case Cmd160.response:
                        self.rsp160_cmd = self.pkt_buffer[3]
                        self.rsp160_len = 0
                    case _default:
                    print("huh.. unknown response code")

    ####################################################################
    #
    def get_used_storage_percent(self):
            count_block = 0
            for log_list in self.log_list_entries:
                    count_block += log_list["count_block"]
            percent = (count_block * 1000 / 520)
            if percent > 0 and percent < 10:
                    percent = 10

            return percent / 10

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
    xgps = await XGPS160.connect(xgps_serialport(), loop)

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
