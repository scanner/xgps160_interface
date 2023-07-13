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

    # settings = await xgps.device_settings()
    xgps.read_device_settings()
    # rprint(f"Device settings: {settings}")

    # For now loop until the loop exits.
    #
    voltage = xgps.battery_voltage
    charging = xgps.is_charging
    while True:
        if xgps.is_charging != charging:
            rprint(f"Charging: {xgps.is_charging}")
            charging = xgps.is_charging

        if not charging and voltage != xgps.battery_voltage:
            rprint(
                f"Battery voltage: {xgps.battery_voltage}, num nmea messages: {len(xgps.nmea_messages)}"
            )
            voltage = xgps.battery_voltage
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
