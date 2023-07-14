#!/usr/bin/env python
#
"""
Class for reading from the SkyPro XGP160 via bluetooth
"""
# system imports
#
import asyncio
import struct
import pathlib
import traceback
from typing import Optional
from enum import IntEnum, Enum
from datetime import datetime, timedelta, UTC
from collections import namedtuple, deque

# 3rd party imports
#
import pynmea2
import serial_asyncio
from rich.console import Console
from rich import print as rprint
from rich.traceback import install as install_tb


install_tb(show_locals=True)

DEVICES_DIR = "/dev/"
XGPS_SERIAL_PATERN = "tty.XGPS160*"

# Battery level conversion constants. See `parse_nmea()` for how these are
# used.
#
K_VOLT_415 = 644
K_VOLT_350 = 543


# Keep up to 10,000 NMEA messages in memory. If we receive more messages than
# that the oldest messages are tossed.
#
DEFAULT_MAX_NMEA_MESSAGES = 10_000

LogEntryStruct = struct.Struct("!BB")
LogEntry = namedtuple("LogEntry", "seq type")

DataEntryStruct = struct.Struct("!HHB3s3s3sHBBBB")

# NOTE: latitude, longitude, altitude are 3 byte long `bytes`
# type. Need to do int.from_bytes(bytes,byteorder="big",signed=True)
#
DataEntry = namedtuple(
    "DataEntry",
    "date tod tod2 latitude longitude altitude speed heading satnum satsig dop",
)
# NOTE: Longitutde, latitude, altitude are MSB order so need to be
#       interprete separately with int.from_bytes()
#
Data2EntryStruct = struct.Struct("!HHB4s4s3sHBB")
Data2Entry = namedtuple(
    "Data2Entry",
    "date tod tod2 latitutde longitude altitude speed heading satsig",
)

LAT_LON_RESOLUTION = 2.1457672e-5


####################################################################
#
def get_lat_lon_24bit(buf: bytes) -> float:
    """
    Keyword Arguments:
    buf: bytearray --
    """
    r = int.from_bytes(buf, byteorder="big")
    d = float(r) * LAT_LON_RESOLUTION

    if r & 0x800000:
        d = -d

    return d


####################################################################
#
def get_lat_lon_32bit(buf: bytes) -> float:
    """
    Keyword Arguments:
    buf: bytes --
    """
    r = int.from_bytes(buf, byteorder="big")
    d = float(r) * 0.000001
    return d


####################################################################
#
def datetime_str(date_bin: int, time_of_day: int) -> datetime:
    year = 2012 + (date_bin // 372)
    month = 1 + (date_bin % 372) // 31
    day = 1 + (date_bin % 31)

    hour = time_of_day // 3600
    minute = (time_of_day % 3600) // 60
    second = time_of_day % 60

    return datetime(
        year=year,
        month=month,
        day=day,
        hour=hour,
        minute=minute,
        second=second,
        tzinfo=UTC,
    )


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
LogListItemStruct = struct.Struct("<BBHIHHH")
LogListItem = namedtuple(
    "LogListItem",
    "sig interval start_date start_tod start_block count_entry count_block",
)


########################################################################
########################################################################
#
class LoggingRate(IntEnum):
    """
    Logging rate can only be one of the following enumerations
    """
    HZ_10 = 1      # 10hz
    HZ_05 = 2      # 5hz
    HZ_02 = 5      # 2hz
    HZ_01 = 10     # 1hz -- once every second
    SEC_2 = 20     # once every 2 seconds
    SEC_3 = 30     # once every 3 seconds
    SEC_4 = 40     # once every 4 seconds
    SEC_5 = 50     # once every 5 seconds
    SEC_10 = 100   # once every 10 seconds
    SEC_12 = 120   # once every 12 seconds
    SEC_15 = 150   # once every 15 seconds
    SEC_20 = 200   # once every 20 seconds


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

    TODO: Add a callback that does a call back to user code for every NMEA
          message received..
    """

    ####################################################################
    #
    def __init__(
        self,
        serial_port: str,
        max_nmea_msgs: int = DEFAULT_MAX_NMEA_MESSAGES,
        console: Optional[Console] = None,
    ):
        self.serial_port = serial_port
        self.console = console

        self.nmea_messages = deque(maxlen=max_nmea_msgs)

        self.rcvd_buff = None

        self.rsp160_cmd = 0

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

        self.datalog_enabled = False
        self.datalog_overwrite = False

        self.device_settings_have_been_read = False

        self.total_GPS_samples_in_log_entry = 0

    ####################################################################
    #
    @classmethod
    async def connect(
        cls,
        serial_port: str,
        loop: Optional[asyncio.AbstractEventLoop] = None,
        console: Optional[Console] = None,
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

        rprint("Connecting...")
        while not xgps_160.is_connected:
            await asyncio.sleep(0.1)

        # Before we return control to the user fetch various data from the
        # device.
        #
        await xgps_160.read_device_settings()
        return xgps_160

    ####################################################################
    #
    def connection_made(self, transport):
        rprint("Connected!")
        self.transport = transport
        self.is_connected = True

    ####################################################################
    #
    def connection_lost(self, exc):
        rprint(f"Connetion lost: {exc}")
        self.transport = None
        self.is_connected = False

    ####################################################################
    #
    def data_received(self, data):
        """
        Parse data received from the XGPS160

        NOTE: For the first implementation I am going to rely upon how packets
              are sent and passed to us: ie: Going to assume that we get at
              least one (and sometimes more than one) packet in each message we
              are passed.

              This is basically the case in all my early testing so instead of
              writing more complicated code that buffers partial packets until
              we have a complete message just going to assume we get the whole
              message.
        """
        # If there is data in the received buffer then append the data we just
        # received to that buffer.
        #
        if self.rcvd_buff:
            self.rcvd_buff += data
            rprint(f"data_received (appended): {repr(self.rcvd_buff)}")
        else:
            self.rcvd_buff = data

        # Parse packets out of the data we have received until there is no more
        # data left to parse.
        #
        while self.rcvd_buff:
            match self.rcvd_buff[0]:
                case 0x88:
                    # 0x88 is a command response message from the XGPS160.
                    #
                    # If the buffer is too short then do not clear
                    # `self.rcvd_buff`. The next time this method is called
                    # new data will be appended on to the end of the existing
                    # received buffer.
                    #
                    if len(self.rcvd_buff) < 3:
                        rprint(
                            f"[red]Binary packet too short (length {len(self.rcvd_buff)})[/red]"
                        )
                        return

                    # If the second byte is not `0xEE` then this is not a valid
                    # command response. Truncate this byte off of the buffer
                    # and try to match it again.
                    #
                    if self.rcvd_buff[1] != 0xEE:
                        rprint("[red]Expected binary packet, 0xEE missing")
                        self.rcvd_buff = self.rcvd_buff[1:]
                        return

                    # The 3rd byte in the buffer is the length of the command
                    # response message from the XGPS160. If the buffer does not
                    # have enough data in it, return letting more accumulate in
                    # self.rcvd_buff.
                    #
                    cmd_resp_length = self.rcvd_buff[2]
                    if (len(self.rcvd_buff) - 3) < cmd_resp_length:
                        rprint(
                            "[red]Binary packet too short. Must be at "
                            f"least {cmd_resp_length} (length: "
                            f"{len(self.rcvd_buff)-3})[/red]"
                        )
                        return

                    # The command response is everything after the first three
                    # bytes up to the length of the command response.
                    #
                    checksum = self.rcvd_buff[cmd_resp_length + 3]
                    cs = sum(self.rcvd_buff[: cmd_resp_length + 3]) & 255
                    if checksum == cs:
                        try:
                            cmd_response = self.rcvd_buff[3 : cmd_resp_length + 3]
                            rprint(
                                f"[bold][red]**[/red][/bold] command response length: {cmd_resp_length}"
                            )
                            rprint(f"   received buffer: {repr(self.rcvd_buff)}")
                            rprint(f"   command response: {repr(cmd_response)}")
                            self.parse_command_response(cmd_response)
                        except Exception as exc:
                            traceback.print_exc()
                            rprint(
                                f"Unable to parse command response {repr(cmd_response)}: {exc}"
                            )
                    else:
                        rprint(f"Checksum error: {cs} != {checksum}")

                    # If there is self.rcvd_buff left after the command
                    # response attempt to parse it. Otherwise we are done with
                    # this packet.
                    #
                    self.rcvd_buff = self.rcvd_buff[cmd_resp_length + 4 :]
                    if self.rcvd_buff:
                        continue
                    return

                case 0x24:
                    # 0x24 == $ .. all NMEA commands begin with "$" up to `\r\n`
                    #
                    # Search for end of NEMA packet.
                    #
                    if b"\r\n" not in self.rcvd_buff:
                        return

                    nmea_packet = self.rcvd_buff[: self.rcvd_buff.find(b"\r\n")].decode(
                        "utf-8"
                    )
                    try:
                        self.parse_nmea(nmea_packet)
                    except pynmea2.nmea.SentenceTypeError as exc:
                        rprint(f"[red][bold]NMEA Parse error:[/bold] {exc}[/red]")
                    except pynmea2.nmea.ChecksumError as exc:
                        rprint(f"[red][bold]NMEA checksum error:[/bold] {exc}[/red]")

                    # If there is any self.rcvd_buff left to parse, try parsing
                    # it, otherwise we are done with this packet.
                    #
                    self.rcvd_buff = self.rcvd_buff[self.rcvd_buff.find(b"\r\n") + 2 :]
                    if len(self.rcvd_buff):
                        continue
                    return

                case _:
                    # If neither b'$' nor 0x88 is in the buffer then it has no
                    # known commands and we can toss it and wait until data
                    # arrives that has a valid command character in it.
                    #
                    if not any(x in self.rcvd_buff for x in (b"$", 0x88)):
                        rprint(
                            f"No command characters in packet: {repr(self.rcvd_buff)}"
                        )
                        self.rcvd_buff = None
                        return

                    # Truncate to the first command byte we have.
                    #
                    bin_pos = self.rcvd_buff.find(0x88)
                    nmea_pos = self.rcvd_buff.find(b"$")

                    if bin_pos == -1:
                        self.rcvd_buff = self.rcvd_buff[nmea_pos:]
                        rprint("forwarding to $")
                        continue
                    if nmea_pos == -1:
                        self.rcvd_buff = self.rcvd_buff[bin_pos:]
                        rprint("forwarding to 0x88")
                        continue
                    self.rcvd_buff = self.rcvd_buff[min(bin_pos, nmea_pos) :]
                    continue

    ####################################################################
    #
    def pause_writing(self):
        """
        The underlying connection's buffer is filling up and it is telling
        us to stop writing for awhile.
        """
        rprint("Please pause writing!")

    ####################################################################
    #
    def resume_writing(self):
        """
        The underlying connection's buffer once again has room for more
        data, so we can resume writing data.
        """
        rprint("Please resume writing!")

    ####################################################################
    #
    def eof_received(self):
        rprint("EOF Received!")
        self.is_connected = False
        self.transport = None

    ####################################################################
    #
    def parse_command_response(self, cmd_response: bytes):
        """ """
        rprint(f"parse_command_response: {repr(cmd_response)}")

        cmd = cmd_response[0]
        match cmd:
            case Cmd160.ack | Cmd160.nack:
                rprint("Command: ACK or NACK")
                return

            case Cmd160.fwRsp:
                rprint("Command fw response")
                sub_cmd = cmd_response[1]
                match sub_cmd:
                    case Cmd160.getSettings:
                        self.parse_get_settings_resp(cmd_response[1:])
                    case Cmd160.logListItem:
                        self.parse_log_list_item(cmd_response[2:])
                    case Cmd160.logReadBulk:
                        self.parse_log_read_bulk(cmd_response[2:])
                    case Cmd160.logDelBlock:
                        if cmd_response[2] == 0x01:
                            rprint("Block data deleted")
                            # self.get_list_of_recorded_logs()
                        else:
                            rprint("Error deleting block data")

                    case Cmd160.response:
                        self.rsp160_cmd = cmd
                        self.rsp160_len = 0
                    case _:
                        rprint("huh.. unknown response code")

    ####################################################################
    #
    def parse_get_settings_resp(self, settings):
        """ """
        rprint("XGPS160 settings")
        self.cfg_gps_settings = settings[0]
        rprint(f"GPS Settings: {self.cfg_gps_settings:>08b}")

        # copy from objc .. we only need one of these two:
        #
        self.cfg_log_interval = settings[1]
        self.log_update_rate = settings[1]
        rprint(f"Log update rate is: {self.log_update_rate}")

        # block and offset are little endian unsigned shorts
        #
        self.cfg_block, self.cfg_log_offset = struct.unpack_from(
            "<HH",
            settings,
            2,
        )

        self.datalog_enabled = True if self.cfg_gps_settings & 0x40 else False
        rprint(f"Datalog enabled: {self.datalog_enabled}")
        self.datalog_overwrite = True if self.cfg_gps_settings & 0x80 else False
        rprint(f"Datalog overwrite: {self.datalog_overwrite}")
        self.device_settings_have_been_read = True

    ####################################################################
    #
    def parse_log_list_item(self, log_list_item_data):
        """
        Keyword Arguments:
        log_list_item_data --
        """
        rprint(f"XGPS160 log list item: {repr(log_list_item_data)}")

        list_idx, list_total = struct.unpack_from(
            "<HH",
            log_list_item_data,
            0,
        )
        rprint(f"  list index: {list_idx}, list total: {list_total}")

        # XXX We have firmware v3.7.3 ... maybe we can skip this check?
        #
        # There is bug in firmware v. 1.3.0. The cmd160_logList command will
        # append a duplicate of the last long entry. For example, if there are
        # 3 recorded logs, the command will repond that there are four: log 0,
        # log 1, log 2 and log 2 again.
        #
        if list_idx == list_total:
            list_idx = 0
            list_total = 0
            logs = None
            return

        logs = {}

        log_list_item = LogListItem._make(
            LogListItemStruct.unpack_from(
                log_list_item_data,
                4,
            )
        )
        rprint(f"  log list item: {log_list_item}")

        log_start_time = datetime_str(
            log_list_item.start_date,
            log_list_item.start_tod,
        )
        rprint(f"  log start time: {log_start_time}")

        logs["interval"] = log_list_item.interval
        logs["count_entry"] = log_list_item.count_entry
        sample_interval = float(log_list_item.interval)
        if log_list_item.interval == 255:
            sample_interval = 10.0
        recording_length_in_sec = log_list_item.country_entry * (sample_interval / 10.0)
        duration = timedelta(seconds=recording_length_in_sec)

        # XXX why do we not just add some more fields to the LogListItemStruct
        #     and add it to the array instead of making a dict and adding that.
        #
        # XXX Is the .sig unique? Can we use it to search for specific entries
        #     in the log_list_entries list?
        #
        logs["human_friendly_duration"] = str(duration)
        logs["sig"] = log_list_item.sig
        logs["start_time"] = log_start_time
        logs["duration"] = duration
        logs["start_block"] = log_list_item.start_block
        logs["count_block"] = log_list_item.count_block
        rprint(f"Adding log list entry: {logs}")
        self.log_list_entries.append(logs)
        # self.processing_log_list_entries_after_delay()
        self.new_log_list_item_received = True

    ####################################################################
    #
    def parse_logs_read_bulk(self, bulk_logs_response):
        """
        Keyword Arguments:
        bulk_logs_response --
        """
        # The address is an unsigned int, but it only uses the first 3 bytes
        # from the start of the `bulk_logs_response`.. so we need to copy it
        # adding that missing byte.
        #
        initial_data = "\x00" + bulk_logs_response[:4]
        addr, data_size = struct.unpack_from("<IB", 0, initial_data)

        log_entry_size = LogEntryStruct.size + max(
            DataEntryStruct.size, Data2EntryStruct.size
        )
        self.log_read_bulk_count = data_size / log_entry_size
        if addr == 0 and data_size == 0:
            # End-of-data
            #
            self.log_read_bulk_count |= 0x1000000
            # self.decode_log_bulk()
            self.log_read_bulk_count = 0
            self.log_bulk_recode_cnt = 0
            self.log_bulk_by_cnt = 0
            self.log_records = []
        else:
            # Looks like we get 5 log entries at a time.
            for loop_idx in range(5):
                # XXX can we do this with a `step` on the range?
                idx = 10 + (loop_idx * log_entry_size)
                log_entry = LogEntry._make(
                    LogEntryStruct.unpack_from(bulk_logs_response, idx)
                )
                if LogEntry.type == 1:
                    de = DataEntry._make(
                        DataEntryStruct.unpack_from(
                            bulk_logs_response, idx + LogEntryStruct.size
                        )
                    )
                else:
                    de = Data2Entry._make(
                        Data2EntryStruct.unpack_from(
                            bulk_logs_response, idx + LogEntryStruct.size
                        )
                    )
                    self.log_records.append((log_entry, de))

    ####################################################################
    #
    def get_used_storage_percent(self):
        count_block = 0
        for log_list in self.log_list_entries:
            count_block += log_list["count_block"]
        percent = count_block * 1000 / 520
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
        # If the message type is "PPWR" then this is proprietyary NMEA message
        # indicating the battery current supply and whether we are charging or
        # not. Battery voltage is not valid while charging.
        #
        if "GPPWR" not in nmea_packet:
            self.nmea_messages.append(pynmea2.parse(nmea_packet))
            return

        # Parse the custom power settings
        #
        nmea_packet = nmea_packet.split(",")
        self.is_charging = True if nmea_packet[5] == "1" else False

        # ??? the objC code looks for NMEA sentences with '@' in them, and we
        # are looking at GPPWR ones. The translation of the battery voltage
        # does not correlated.
        #
        # NOTE: 1280 is 96%
        self.battery_voltage = int(nmea_packet[1], 16)

    ####################################################################
    #
    def decode_log_bulk(self):
        """ """
        self.log_data_samples = []
        for idx, (log_entry, data_entry) in enumerate(self.log_records):
            rprint(f"{idx:02d}: {log_entry}")
            # if log_entry.type == 0:
            #     data_entry = log_entry.data  # Union.. dataentry vs data2entry
            #     # ....
            # else:
            #     data_2_entry = log_entry.data
            #     # ....

    ####################################################################
    #
    def send_command(self, command: Cmd160, payload: Optional[bytes] = None):
        """
        Keyword Arguments:
        command --
        payload --
        """
        payload = b"" if payload is None else payload
        message_header = struct.pack(
            "<BBBB",
            0x88,
            0xEE,
            len(payload) + 1,
            command,
        )
        message = message_header + payload
        checksum = struct.pack("<B", sum(message) & 255)

        self.transport.write(message + checksum)

    ####################################################################
    #
    async def read_device_settings(self):
        """ """
        self.device_settings_have_been_read = False
        self.send_command(Cmd160.getSettings, b"")
        while not self.device_settings_have_been_read:
            await asyncio.sleep(0.1)

        return self.cfg_gps_settings

    ####################################################################
    #
    def start_logging(self):
        """ """
        pass

    ####################################################################
    #
    def stop_logging(self):
        """ """
        pass

    ####################################################################
    #
    async def get_list_of_recorded_logs(self):
        """
        Gets the list of recorded log records from the device (this is not
        the actual logged data, it is the list of fixed size items that contain
        the log data.)
        """
        self.log_list_entries = []
        self.new_log_list_item_received = False
        self.send_command(Cmd160.logList)

        while not self.new_log_list_item_received:
            await asyncio.sleep(0.1)
        return self.log_list_entries

    ####################################################################
    #
    def get_gps_sample_data_for_log_list_item(self, log_list_item):
        """ """
        self.log_data_samples = []
        start_block = log_list_item["start_block"]
        count_block = log_list_item["count_block"]

        payload = struct.pack("<HH", start_block, count_block)
        self.send_command(Cmd160.log_read_bulk, payload)

    ####################################################################
    #
    def delete_gps_sample_data_for_log_list_item(self, log_list_item):
        """ """
        start_block = log_list_item["start_block"]
        count_block = log_list_item["count_block"]

        payload = struct.pack("<HH", start_block, count_block)
        self.send_command(Cmd160.log_del_block, payload)

        # find log_list_item in self.log_list_entries and delete it.
        #
        self.log_list_entries = [
            x
            for x in self.log_list_entries
            if not (x["start_block"] == start_block and
                    x["count_block"] == count_block)
        ]

    ####################################################################
    #
    def enter_log_access_mode(self):
        """
        It's much simpler to deal with log data information while the
        device is not streaming GPS data. So the recommended practice is to
        pause the NMEA stream output during the time that logs are being
        accessed and manipulated.

        XXX That may have been true in the objective c code but not sure if it
            is true when running in python on a computer
        """
        self.send_command(Cmd160.streamStop)
        self.streaming_mode = False
        self.get_list_of_recorded_logs()

    ####################################################################
    #
    def exit_log_access_mode(self):
        """
        Remember to tell the XGPS160 to resume sending NMEA data once you
        are finished with the log data.
        """
        self.send_command(Cmd160.streamResume)
        self.streaming_mode = True

    ####################################################################
    #
    def datalog_overwrite(self, overwrite: bool):
        """
        If overwrite is True then when storage fills up the oldest log
        records will be overwritten as new data comes in.

        If overwrite is False then the device will stop writing data when the
        storage is full.
        """
        if overwrite:
            self.send_command(Cmd160.logOWEnable)
        else:
            self.send_command(Cmd160.logOWDisable)

    ####################################################################
    #
    def datalog_enable(self, enable: bool):
        """
        This enables or disables automatic data logging when the device
        powers on.

        Keyword Arguments:
        enable: bool --
        """
        if enable:
            self.send_command(Cmd160.logEnable)
        else:
            self.send_command(Cmd160.logDisable)

    ####################################################################
    #
    def check_for_adjustable_rate_logging(self) -> bool:
        """
        The firmware version of my device is 3.7.3.. since I am using a
        serial interface and nothing else I have not used bluetooth to probe
        the device's firmware so just going to hard code this.. for my device
        this will always be True.

        XXX Add support to probe bluetooth device for firmware version.
        """
        return True

    ####################################################################
    #
    # XXX Instead of an int it should be an enum since only certain values are
    #     allowed.
    def set_logging_update_rate(self, rate: LoggingRate):
        """
        Keyword Arguments:
        rate: int --
        """
        # Must be at least firmware version 1.3.5 or greather.
        #
        if not self.check_for_adjustable_rate_logging:
            return False

        payload = struct.pack("<B", rate)

        self.send_command(Cmd160.logInterval, payload)
        return True
