#!/usr/bin/env python
#
# File: $Id$
#
"""
Read the stored tracks in the XGPS160 and download them in to GPX files.

Usage:
    read_from_gps.py [--delete] [--no-write] [--no-download]

Options:
  --version
  -h, --help        Show this text and exit
  --delete          Delete all the gps tracks in the device
                    (after download step)
  --no-write        Do not write the tracks (handy for testing)
  --no-download     Do not download the tracks (handy if you have already
                    downloaded them and you just want to delete the tracks
                    in the device.)
"""
# system imports
#
import asyncio
from datetime import timedelta
from typing import List

# 3rd party imports
#
from docopt import docopt
from gpxpy.gpx import GPX, GPXTrack, GPXTrackPoint, GPXTrackSegment
from rich import print as rprint
from rich.progress import Progress
from rich.traceback import install as install_tb

# Project imports
#
from a_s.xgps160 import XGPS160, xgps_serialport

install_tb(show_locals=True)

VERSION = "0.2"


####################################################################
#
async def download_gps_tracks(xgps: XGPS160, log_list: List, write: bool):
    """
    Download all of the GPS tracks from the device and write the data out
    in to GPS formatted files. The names of the files are from the start and
    end times of the collated tracks being written.
    """
    # Get the gps data for the last log entry
    #
    gps_tracks = []
    with Progress(transient=True) as progress:
        task = progress.add_task("Downloading", total=len(log_list))
        for log in log_list:
            progress.console.print(
                f"Downloading GPS data for log {log['start_block']}-"
                f"{log['count_block']}",
                end="",
            )
            gps_data = await xgps.get_gps_sample_data_for_log_list_item(log)
            start_time = gps_data[0][1]["datetime"]
            end_time = gps_data[-1][1]["datetime"]
            progress.console.print(
                f"{log['start_block']}, start: {start_time.strftime('%Y-%m-%d %H:%M:%S')}, end: {end_time.strftime('%Y-%m-%d %H:%M:%S')}"
            )
            progress.update(task, advance=1)
            gps_tracks.append(gps_data)

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
    end_time = gps_tracks[0][-1][1]["datetime"]
    start_time = gps_tracks[0][0][1]["datetime"]

    # If it has been more than 45 minutes between one data record ending and
    # the next data record starting, then start a whole new GPX file.
    #
    min_timedelta = timedelta(minutes=45)

    # If it has been more than 10 minutes between a data record stopping and
    # the next data record starting, start a new GPXTrackSegment.
    #
    # seg_timedelta = timedelta(minutes=10)  XXX currently unused
    dt_format = "%Y-%m-%d_%H-%M-%S"

    with Progress() as progress:
        task_tracks = progress.add_task("Processing", total=len(gps_tracks))
        for gps_track in gps_tracks:
            # Starting a new GPS track.. see if the timestamp of the first
            # entry is more than min timedelta from the timestamp of the last
            # entry in the previous track.
            #
            td = gps_track[0][1]["datetime"] - end_time
            if td > min_timedelta:
                start = start_time.strftime(dt_format)
                end = end_time.strftime(dt_format)
                filename = f"{start}--{end}.gpx"
                if write:
                    with open(filename, "w") as f:
                        f.write(gpx.to_xml())
                progress.console.print(
                    f"Wrote gps track [blue]`{filename}`[/blue]"
                )
                gpx = GPX()
                gpx_track = GPXTrack()
                gpx.tracks.append(gpx_track)
                gpx_segment = GPXTrackSegment()
                gpx_track.segments.append(gpx_segment)

                # We also can now set the start_time and end_time
                # track.
                #
                start_time = gps_track[0][1]["datetime"]
            # elif gps_track[0][1]["datetime"] - end_time > seg_timedelta:
            #     gpx_segment = GPXTrackSegment()
            #     gpx_track.segments.append(gpx_segment)

            # task_points = progress.add_task(
            #     f"{gps_track[0][1]['datetime']}", total=len(gps_track)
            # )
            for entry, data in gps_track:
                gpx_segment.points.append(
                    GPXTrackPoint(
                        longitude=data["longitude"],
                        latitude=data["latitude"],
                        elevation=data["altitude"],
                        time=data["datetime"],
                    )
                )
                # progress.update(task_points, advance=1)
            progress.update(task_tracks, advance=1)
            end_time = gps_track[-1][1]["datetime"]

    # and at the end write the final collected track out.
    #
    start = start_time.strftime(dt_format)
    end = end_time.strftime(dt_format)
    filename = f"{start}--{end}.gpx"
    with open(filename, "w") as f:
        f.write(gpx.to_xml())
    rprint(f"Wrote gps track `{filename}`")


#############################################################################
#
async def main(loop):
    """
    Connect to the XGPS160. Get its settings, download all of its tracks,
    and delete them.
    """
    args = docopt(__doc__, version=VERSION)
    delete = args["--delete"]
    write = not args["--no-write"]
    download = not args["--no-download"]

    ser_port = xgps_serialport()
    rprint(f"XGPS serial port: {ser_port}")
    xgps = await XGPS160.connect(str(ser_port), loop=loop)

    await xgps.read_device_settings()
    rprint(f"Datalog enabled: {xgps.datalog_enabled}")
    rprint(f"Datalog overwrite: {xgps.datalog_overwrite}")
    rprint(f"Log interval: {xgps.cfg_log_interval}")

    log_list = await xgps.enter_log_access_mode()
    rprint(f"number of log list entries: {len(log_list)}")
    rprint(f"percent storage used: {xgps.get_used_storage_percent()}")

    if download:
        rprint("Downloading GPS data..")
        await download_gps_tracks(xgps, log_list, write)

    # Now that we have downloaded and saved the gps data we can delete it from
    # the device.
    #
    if delete:
        rprint("Deleting GPS data from device..")
        rprint(f"Log list len: {len(log_list)}")
        for log in log_list:
            await xgps.delete_gps_data(log)
        log_list = await xgps.enter_log_access_mode()
        rprint(f"After delete, log list len: {len(log_list)}")
    xgps.exit_log_access_mode()


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
