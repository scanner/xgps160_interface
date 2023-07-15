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
from datetime import datetime, timedelta

# 3rd party imports
#
from gpxpy.gpx import GPX, GPXTrack, GPXTrackSegment, GPXTrackPoint
from rich import print as rprint
from rich.pretty import pprint
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

    await xgps.get_recorded_logs()
    rprint(f"number of log list entries: {len(xgps.log_list_entries)}")
    rprint(f"percent storage used: {xgps.get_used_storage_percent()}")

    # Get the gps data for the last log entry
    #
    gps_tracks = []
    for log_list in xgps.log_list_entries:
        rprint(
            f"Loading GPS data for log {log_list['start_block']}-{log_list['count_block']}"
        )
        gps_data = await xgps.get_gps_sample_data_for_log_list_item(log_list)
        gps_tracks.append(gps_data)

    # We no longer need a connection to the device since we have downloaded all
    # the tracks.
    #
    xgps.close()

    # The GPS160 basically stores a fixed number of points in a data log. If
    # youare recording data continuously for longer than one data log can
    # store, it will create multiple datalogs.
    #
    # If you just translate one datalog to one track then you get a lot of very
    # short tracks. What we do here is if the start of the next log is within
    # 30 minutes of the end of the previous log then we treat the next log as
    # part of the same track (maybe we should break this up in to track
    # segments?)
    #
    gpx = GPX()
    gpx_track = GPXTrack()
    gpx.tracks.append(gpx_track)
    gpx_segment = GPXTrackSegment()
    gpx_track.segments.append(gpx_segment)

    # make the start & end time the timestamp of the first entry in the first
    # track we got from the GPS.
    end_time = gps_tracks[0][0][1]["datetime"]
    start_time = gps_tracks[0][0][1]["datetime"]
    min_timedelta = timedelta(minutes=45)
    dt_format = "%Y-%m-%d_%H-%M-%S"

    for gps_track in gps_tracks:
        # Starting a new GPS track.. see if the timestamp of the first entry is
        # more than min timedelta from the timestamp of the last entry in the
        # previous track.
        #
        rprint(
            f"Starting track {gps_track[0][1]['datetime']}",
            end="",
            flush=True,
        )

        if gps_track[0][1]["datetime"] - end_time > min_timedelta:
            start = start_time.strftime(dt_format)
            end = end_time.strftime(dt_format)
            filename = f"{start}--{end}.gpx"
            with open(filename, "w") as f:
                f.write(gpx.to_xml())
            rprint(f"\nWrote gps track `{filename}`")
            gpx = GPX()
            gpx_track = GPXTrack()
            gpx.tracks.append(gpx_track)
            gpx_segment = GPXTrackSegment()
            gpx_track.segments.append(gpx_segment)

            # We also can now set the start_time and end_time
            # track.
            #
            start_time = gps_track[0][1]["datetime"]
            end_time = gps_track[-1][1]["datetime"]

        for entry, data in gps_track:
            rprint(".", end="", flush=True)
            gpx_segment.points.append(
                GPXTrackPoint(
                    longitude=data["longitude"],
                    latitude=data["latitude"],
                    elevation=data["altitude"],
                    time=data["datetime"],
                )
            )
        rprint("")

    # and at the end write the final collected track out.
    #
    start = start_time.strftime(dt_format)
    end = end_time.strftime(dt_format)
    filename = f"{start}--{end}.gpx"
    with open(filename, "w") as f:
        f.write(gpx.to_xml())
    rprint(f"Wrote gps track `{filename}`")


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
