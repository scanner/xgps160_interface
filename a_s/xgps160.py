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
import enum
from typing import Optional, List
from datetime import datetime, timedelta, UTC
from collections import namedtuple, deque

# 3rd party imports
#
import pynmea2
import serial_asyncio
from rich.console import Console
from rich import print as rprint
from rich.pretty import pprint
from rich.traceback import install as install_tb


install_tb(show_locals=True)

DEVICES_DIR = "/dev/"
XGPS_SERIAL_PATERN = "tty.XGPS160*"

# Keep up to 10,000 NMEA messages in memory. If we receive more messages than
# that the oldest messages are tossed.
#
DEFAULT_MAX_NMEA_MESSAGES = 10_000

# seq - sequence number of the record (wrap after 255)
# type - 0 = dataentry_t, 2=dataentry2_t,
#
LogEntryStruct = struct.Struct("<BB")
LogEntry = namedtuple("LogEntry", "seq type")

DataEntryStruct = struct.Struct("<HHB3s3s3sHBBBB")

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
Data2EntryStruct = struct.Struct("<HHB4s4s3sHBB")
Data2Entry = namedtuple(
    "Data2Entry",
    "date tod tod2 latitude longitude altitude speed heading satsig",
)

LAT_LON_RESOLUTION = 2.1457672e-5


####################################################################
#
def get_lat_lon_24bit(buf: bytes) -> float:
    """
    Convert a 24bit lat/lon value in to its float equivalent.
    """
    r = struct.unpack(">i", b"\0" + buf)[0]
    d = float(r) * LAT_LON_RESOLUTION

    if r & 0x800000:
        d = -d

    return d


####################################################################
#
def get_lat_lon_32bit(buf: bytes) -> float:
    """
    Convert a 32bit lat/lon value in to its float equivalent.
    """
    r = struct.unpack(">i", buf)[0]
    return float(r) * 0.000001


####################################################################
#
def datetime_str(
    date_bin: int,
    time_of_day: int,
    tenths_of_sec: int = 0,
) -> datetime:
    """
    The formula for converting the XGPS160 binary data for a date, time of
    day, tenths of a sec to a python datetime object.
    """
    year = 2012 + (date_bin // 372)
    month = 1 + (date_bin % 372) // 31
    day = 1 + (date_bin % 31)

    hour = time_of_day // 3600
    minute = (time_of_day % 3600) // 60
    second = time_of_day % 60

    # XXX This is not correct. ObjC code:
    #        tod = (d->tod2 & 0x10);
    #        tod <<= 12;
    #        tod |= d->tod;
    #        tod10th = d->tod2 & 0x0F;
    #
    usec = tenths_of_sec * 100_000

    return datetime(
        year=year,
        month=month,
        day=day,
        hour=hour,
        minute=minute,
        second=second,
        microsecond=usec,
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
# A struct and named tuple to make it slightly easier to parse the binary data
# stream for the meta data associated with a recorded GPS track.
#
LogListItemStruct = struct.Struct("<BBHIHHH")
LogListItem = namedtuple(
    "LogListItem",
    "sig interval start_date start_tod start_block count_entry count_block",
)

########################################################################
########################################################################
#
# A struct for parsing the settings information.
#
SettingsStruct = struct.Struct("<BBHH")
Settings = namedtuple(
    "Settings",
    "settings log_interval block offset",
)


########################################################################
########################################################################
#
class LoggingRate(enum.IntEnum):
    """
    Logging rate can only be one of the following enumerations
    """

    SEC_00_1 = 1  # 10hz
    SEC_00_2 = 2  # 5hz
    SEC_00_5 = 5  # 2hz
    SEC_01_0 = 10  # 1hz -- once every second
    SEC_02_0 = 20  # once every 2 seconds
    SEC_03_0 = 30  # once every 3 seconds
    SEC_04_0 = 40  # once every 4 seconds
    SEC_05_0 = 50  # once every 5 seconds
    SEC_10_0 = 100  # once every 10 seconds
    SEC_12_0 = 120  # once every 12 seconds
    SEC_15_0 = 150  # once every 15 seconds
    SEC_20_0 = 200  # once every 20 seconds


########################################################################
########################################################################
#
class Cmd160(enum.IntEnum):
    """
    The possible commands and command responses for the XGPS160.
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

        # This buffer pools together messages received from the device for
        # parsing. If we do not have enough data for a complete command, we
        # wait until more data is retrieved (and we can find a command in that
        # data.)
        self.rcvd_buff = b""

        self.transport = None
        self.is_connected = False

        # XXX These are not set. Currently we are only talking to the device
        #     via a bluetooth serial port. We will need to probe the bluetooth
        #     device itself to get this data. My unit is running fw 3.7.3 so
        #     most of the code is operating with that assumption.
        #
        self.firmware_rev = ""
        self.serial_number = ""

        self.cfg_gps_settings = 0
        self.datalog_enabled = False
        self.datalog_overwrite = False

        # XXX We are getting the battery voltage via the GPPWR proprietary
        #     message. We are not parsing it correctly so the value is only
        #     advisory.

        self.battery_voltage = 0
        self.is_charging = False
        self.streaming_mode = True

        # A list of the log entries in the device. Refreshed when
        # `get_recorded_logs()` is called and completes.
        #
        self.log_list_entries = []

        # An array used to build up GPS sample data retrieved for
        # get_gps_sample_data_for_log_list_item(). Only valid during the
        # duration of that call.
        #
        self.log_data_samples = []

        # XXX These booleans are used as "signals" to note when the results
        #     from a command have been processed. This is mostly an artifact of
        #     how I copied the code from the ObjC project. Need to properly
        #     make these signals and if you have multiple calls open at the
        #     same time we only process the data once (and return it multiple
        #     times)
        #
        # For now my simplistic usage this is good enough (just to get all the
        # recorded data and store it)
        #
        self.new_log_list_item_received = False
        self.device_settings_have_been_read = False
        self.bulk_data_has_been_read = False

        # Hm.. maybe we should enforce that you can only send one command at a
        # time and no other until the one currently sent has finished.. and
        # then use a single asyncio.Event to notify this.
        #
        self.delete_event = asyncio.Event()
        self.settings_event = asyncio.Event()
        self.bulk_data_event = asyncio.Event()
        self.log_list_event = asyncio.Event()

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

        # XXX This is going to wait forever. We should have a timeout and if
        #     does not connect in that time raise an exception or return False.
        #
        while not xgps_160.is_connected:
            await asyncio.sleep(0.1)

        # Before we return control to the user fetch various data from the
        # device.
        #
        await xgps_160.read_device_settings()
        return xgps_160

    ####################################################################
    #
    def close(self):
        """
        Close our connection to the device.
        """
        self.transport.close()

    ####################################################################
    #
    def connection_made(self, transport):
        self.transport = transport
        self.is_connected = True

    ####################################################################
    #
    def connection_lost(self, exc):
        if exc:
            rprint(f"Connetion lost: {exc}")
        else:
            rprint("Connection closed")
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
        # All new data gets concatenated to our existing rcvd_buff. Not the
        # most efficient way to handle bytes of data comming in, but it is
        # representationally simpler code.
        #
        self.rcvd_buff += data

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
                            "[red]Binary packet too short (length "
                            f"{len(self.rcvd_buff)}[/red])"
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
                    cmd_response = self.rcvd_buff[3 : cmd_resp_length + 3]

                    if checksum == cs:
                        try:
                            self.parse_command_response(cmd_response)
                        except Exception as exc:
                            traceback.print_exc()
                            rprint(
                                "Unable to parse command response "
                                f"{repr(cmd_response.hex(':'))}: {exc}"
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
                    crlf_pos = self.rcvd_buff.find(b"\r\n")
                    nmea_packet = self.rcvd_buff[:crlf_pos].decode("utf-8")

                    try:
                        self.parse_nmea(nmea_packet)
                    except pynmea2.nmea.SentenceTypeError as exc:
                        rprint(f"[red][bold]NMEA Parse error:[/bold] {exc}[/red]")
                    except pynmea2.nmea.ChecksumError as exc:
                        rprint(f"[red][bold]NMEA checksum error:[/bold] {exc}[/red]")

                    # If there is any self.rcvd_buff left to parse, try parsing
                    # it, otherwise we are done with this packet.
                    #
                    self.rcvd_buff = self.rcvd_buff[crlf_pos + 2 :]
                    if len(self.rcvd_buff):
                        continue
                    return

                case _:
                    # If neither b'$' nor 0x88 is in the buffer then it has no
                    # known commands and we can toss it and wait until data
                    # arrives that has a valid command character in it.
                    #
                    if not any(x in self.rcvd_buff for x in (b"$", 0x88)):
                        self.rcvd_buff = b""
                        return

                    # Truncate to the first command byte we have.
                    #
                    bin_pos = self.rcvd_buff.find(0x88)
                    nmea_pos = self.rcvd_buff.find(b"$")

                    if bin_pos == -1:
                        self.rcvd_buff = self.rcvd_buff[nmea_pos:]
                        rprint("truncating up to first $")
                        continue
                    if nmea_pos == -1:
                        self.rcvd_buff = self.rcvd_buff[bin_pos:]
                        rprint("truncating up to first 0x88")
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
        cmd = cmd_response[0]
        match cmd:
            case Cmd160.ack | Cmd160.nack:
                rprint("Command: ACK or NACK")
                return

            case Cmd160.fwRsp:
                sub_cmd = cmd_response[1]
                # rprint(f"* [blue][bold]{Cmd160(sub_cmd).name}[/bold][/blue]")
                match sub_cmd:
                    case Cmd160.getSettings:
                        self.parse_get_settings_resp(cmd_response[1:])
                    case Cmd160.logListItem:
                        self.parse_log_list_item(cmd_response[2:])
                    case Cmd160.logReadBulk:
                        self.parse_log_read_bulk(cmd_response[2:])
                    case Cmd160.logDelBlock:
                        if cmd_response[2] != 0x01:
                            rprint(f"Error deleting block data: {cmd_response.hex(':')}")
                        self.delete_event.set()

                    case Cmd160.response:
                        self.rsp160_cmd = cmd
                        self.rsp160_len = 0
                        rprint(f"Command response {Cmd160(response).name}")
                    case _:
                        rprint("huh.. unknown response code")
            case _:
                rprint("[red]Unknown command {cmd}[/red]")

    ####################################################################
    #
    def parse_get_settings_resp(self, settings_data):
        """ """
        settings = Settings._make(SettingsStruct.unpack_from(settings_data, 1))

        self.cfg_gps_settings = settings.settings
        self.cfg_log_interval = settings.log_interval
        self.log_update_rate = settings.log_interval
        self.cfg_block = settings.block
        self.cfg_log_offset = settings.offset

        self.datalog_enabled = True if self.cfg_gps_settings & 0x40 else False
        self.datalog_overwrite = True if self.cfg_gps_settings & 0x80 else False
        self.device_settings_have_been_read = True

    ####################################################################
    #
    def parse_log_list_item(self, log_item_data):
        """
        Keyword Arguments:
        log_item_data --
        """
        list_idx, list_total = struct.unpack_from(
            ">HH",
            log_item_data,
            1,
        )

        # If the length of the data we got back is less than 18 bytes long then
        # we do not have enough data to unpack the log list item. Return.
        # NOTE: This seems to consistently happen as the last packet after all
        # of the other log entries have been received. So we are going to use
        # this as our "end packet"
        #
        if len(log_item_data) < 18:
            self.new_log_list_item_received = True
            return

        log = {}

        log_item = LogListItem._make(
            LogListItemStruct.unpack_from(
                log_item_data,
                # 4,
                5,
            )
        )
        log_start_time = datetime_str(
            log_item.start_date,
            log_item.start_tod,
        )
        sample_interval = float(log_item.interval)
        if log_item.interval == 255:
            sample_interval = 10.0

        recording_length_in_sec = log_item.count_entry * (sample_interval / 10.0)
        duration = timedelta(seconds=recording_length_in_sec)

        # NOTE: We combine "start date" and "start time"'s from the objC code
        #       in to the obvious python "datetime" class.
        #
        # LogDic items from objC code:
        # - DeviceStartDate
        # - DeviceStartTime
        # - DevicerecordingStart
        # - humanFriendlyStartDate
        # - humanFriendlyStartTime
        # - recordingState
        # - interval
        # - countEntry
        # - humanFriendlyDuration
        # - sig
        # - startDate
        # - startTod
        # - startBlock
        # - countBlock
        log["interval"] = log_item.interval
        log["data_points"] = log_item.count_entry
        log["human_friendly_duration"] = str(duration)
        log["sig"] = log_item.sig
        log["start_date"] = log_item.start_date
        log["start_tod"] = log_item.start_tod
        log["start_time"] = log_start_time
        log["duration"] = duration
        log["start_block"] = log_item.start_block
        log["count_block"] = log_item.count_block
        self.log_list_entries.append(log)
        # pprint(log)

    ####################################################################
    #
    def parse_log_read_bulk(self, bulk_log_data):
        """
        This is called when we have received a GPS sample data packet. When
        doing a sample download the device will send a whole bunch of packets,
        each with an "addr" and "data_size" element. When these two elements
        are both 0 then that means that the device has sent us all of the data
        for the required data log.
        """
        addr, data_size = struct.unpack_from(">IB", bulk_log_data, 0)

        # If addr & data size are 0 then we are done collecting bulk sample
        # data. We can now decode and tell our caller that the sample data has
        # been retrieved.
        #
        if addr == 0 and data_size == 0:
            self.bulk_data_has_been_read = True
            return

        # To make code easier to read we cut off the int & byte we just read
        # from the data. Everything here should be the bulk gps sample data we
        # care about.
        #
        bulk_log_data = bulk_log_data[5:]

        # Loop through the data determining what kind of data entry type it is
        # (1 or 2.. but my device is always 2 since it is a later version)
        #
        while bulk_log_data:
            log_entry = LogEntry._make(LogEntryStruct.unpack_from(bulk_log_data, 0))

            # Any version besides 1 or 2 indicates end of data entry packets in
            # our buffer (and likely end of all gps sample data for this entry)
            #
            if log_entry.type == 2:
                data_entry = Data2Entry._make(
                    Data2EntryStruct.unpack_from(bulk_log_data, 2)
                )
            elif log_entry.type == 1:
                data_entry = DataEntry._make(
                    DataEntryStruct.unpack_from(bulk_log_data, 2)
                )
            else:
                break

            data = self.convert_bulk_data_entry(log_entry, data_entry)
            self.log_data_samples.append((log_entry, data))
            # The ObjC source code used a union struct which takes the maximal
            # size of the sub-structs and that is the Data2Entry. So chop off
            # from our bulk log data bytes that amount of data to get to the
            # next element.
            #
            data_size = LogEntryStruct.size + Data2EntryStruct.size
            bulk_log_data = bulk_log_data[data_size:]

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
    def convert_bulk_data_entry(self, entry, sample):
        """
        Convert the DataEntry to a dict with the values converted to more
        appropriate types (datetimes, floats, feet, etc.)
        """
        converted_data = []
        match entry.type:
            case 2:
                # when = datetime_str(sample.date, sample.tod, tenths_of_sec=sample.tod2, )
                when = datetime_str(sample.date, sample.tod)
                lat = get_lat_lon_32bit(sample.latitude)
                lon = get_lat_lon_32bit(sample.longitude)
                # Convert 24bit altitude from centimeters to feet.
                alt = struct.unpack(">I", b"\0" + sample.altitude)[0] / 100.0 / 0.3048
                heading = sample.heading * 360.0 / 256.0
            case 1:
                when = datetime_str(sample.date, sample.tod)
                lat = get_lat_lon_24bit(sample.latitude)
                lon = get_lat_lon_24bit(sample.longitude)
                # Convert 24bit altitude from units of 5ft
                alt = struct.unpack(">I", b"\0" + sample.altitude) * 5.0
                heading = sample.heading * 360.0 / 256.0
        d = {
            "datetime": when,
            "latitude": lat,
            "longitude": lon,
            "altitude": alt,
            "speed": sample.speed,
            "heading": heading,
        }
        return d

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
    async def get_recorded_logs(self) -> List:
        """
        Gets the list of recorded log records from the device (this is not
        the actual logged data, it is the list of fixed size items that contain
        the log data.)

        XXX Needs better name.
        """
        self.log_list_entries = []
        self.new_log_list_item_received = False
        self.send_command(Cmd160.logList)

        while not self.new_log_list_item_received:
            await asyncio.sleep(0.1)
        return self.log_list_entries

    ####################################################################
    #
    async def get_gps_sample_data_for_log_list_item(
        self,
        log_list_item,
    ) -> List:
        """
        Download the GPS sample data for the specific log list item.

        It will return a list of tuples. Each tuple represents one GPS
        point. The first element in the tuple is the "log entry" information
        for the datapoint indicating the sequence number of the data type (it
        mostly can be just ignored). The second element will be a dict with the
        data for the GPS point including things such as datetime, longitude,
        latitude, and altitude.

        This function will await until the complete results have been received
        from the device.

        """
        self.log_data_samples = []
        # XXX We should start using signals for stuff like this.
        self.bulk_data_has_been_read = False
        start_block = log_list_item["start_block"]
        count_block = log_list_item["count_block"]

        payload = struct.pack(">HH", start_block, count_block)
        self.send_command(Cmd160.logReadBulk, payload)

        while not self.bulk_data_has_been_read:
            await asyncio.sleep(0.1)
        return self.log_data_samples

    ####################################################################
    #
    async def delete_gps_data(self, log_list_item) -> List:
        """
        Given a specific log list item tell the device to delete the
        associated GPS sample data.

        This will also fetch the log list from the device and return it.
        """
        self.delete_event.clear()
        start_block = log_list_item["start_block"]
        count_block = log_list_item["count_block"]

        payload = struct.pack(">HH", start_block, count_block)
        self.send_command(Cmd160.logDelBlock, payload)
        await self.delete_event.wait()

        self.log_list_entries = [
            x
            for x in self.log_list_entries
            if not (x["start_block"] == start_block and x["count_block"] == count_block)
        ]
        return self.log_list_entries

    ####################################################################
    #
    async def enter_log_access_mode(self):
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
        return await self.get_recorded_logs()

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
    async def datalog_overwrite(self, overwrite: bool):
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
        await self.read_device_settings()

    ####################################################################
    #
    async def datalog_enable(self, enable: bool):
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
        await self.read_device_settings()

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
