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
import time

# 3rd party imports
#
from rich import print as rprint
from rich.traceback import install as install_tb

# Project imports
#
from a_s.xgps160 import XGPS160, xgps_serialport

install_tb(show_locals=True)


#############################################################################
#
async def main(loop):
    """
    Find the XGPS 160 serial port. Listen for messages from it.
    """
    ser_port = xgps_serialport()
    rprint(f"XGPS serial port: {ser_port}")
    xgps = await XGPS160.connect(str(ser_port), loop=loop)

    settings = await xgps.read_device_settings()
    rprint(f"Device settings: {settings}")

    await xgps.get_list_of_recorded_logs()
    rprint(f"number of log list entries: {len(xgps.log_list_entries)}")
    rprint(f"percent storage used: {xgps.get_used_storage_percent()}")

    # Print out difference between "start_date"'s
    #
    # first_start = None
    # for idx, log_item in enumerate(xgps.log_list_entries):
    #     if first_start is not None:
    #         rprint(f"{idx:02}: {log_item['start_date']-first_start}")
    #     first_start = log_item["start_date"]

    # print out differences in
    # For now loop forever..
    #
    voltage = xgps.battery_voltage
    charging = xgps.is_charging
    last_check = time.time()

    while True:
        if xgps.is_charging != charging:
            rprint(f"Charging: {xgps.is_charging}")
            charging = xgps.is_charging

        if not charging and voltage != xgps.battery_voltage:
            rprint(
                f"Battery voltage: {xgps.battery_voltage}, num nmea messages: "
                f"{len(xgps.nmea_messages)}"
            )
            voltage = xgps.battery_voltage

        # Once a minute check the device settings. Print if they have changed.
        #
        now = time.time()
        if now - last_check > 60:
            await xgps.read_device_settings()
            rprint(f"Device settings: {xgps.cfg_gps_settings:>08b}")
            rprint(f"Datalog enabled: {xgps.datalog_enabled}")
            rprint(f"Datalog overwrite: {xgps.datalog_overwrite}")
            rprint(f"Config block: {xgps.cfg_block}, log offset: {xgps.cfg_log_offset}")
            rprint(f"Log update rate: {xgps.log_update_rate}")
            last_check = now

        await asyncio.sleep(1)


############################################################################
############################################################################
#
# Here is where it all starts
#
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(main(loop))
    except KeyboardInterrupt:
        rprint("[red]Caught keyboard interrupt, exiting..[/red]")
    finally:
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()
#
############################################################################
############################################################################
