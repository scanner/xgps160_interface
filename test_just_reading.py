#!/usr/bin/env python
#
# File: $Id$
#
"""
Just connect to the XGPS160 and print out all the data we receive from it.
"""
# system imports
#
import asyncio
import serial_asyncio

# 3rd party imports
#
from rich.traceback import install as install_tb
from rich import print as rprint


install_tb(show_locals=True)


########################################################################
########################################################################
#
class InputChunkProtocol(asyncio.Protocol):
    def connection_made(self, transport):
        rprint("[bold][green]Connected![/green][/bold]")
        self.transport = transport

    def data_received(self, data):
        rprint("data received", repr(data))

        # stop callbacks again immediately
        # self.pause_reading()

    def pause_reading(self):
        # This will stop the callbacks to data_received
        self.transport.pause_reading()

    def resume_reading(self):
        # This will start the callbacks to data_received again with all data
        # that has been received in the meantime.
        self.transport.resume_reading()


########################################################################
#
async def reader(loop):
    rprint("[blue]Connecting..[/blue]")
    transport, protocol = await serial_asyncio.create_serial_connection(
        loop,
        InputChunkProtocol,
        "/dev/tty.XGPS160-770173",
        baudrate=115200,
    )
    rprint("[green]Entering reading loop.[/green]")

    while True:
        await asyncio.sleep(0.3)
        # protocol.resume_reading()


#############################################################################
#
def main():
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(reader(loop))
    except KeyboardInterrupt:
        rprint("[red]Caught keyboard interrupt, exiting..[/red]")
    finally:
        loop.close()


############################################################################
############################################################################
#
# Here is where it all starts
#
if __name__ == '__main__':
    main()
#
############################################################################
############################################################################
