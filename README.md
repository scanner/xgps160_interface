# xgps160_interface
![](https://img.shields.io/badge/language-python-orange.svg?style=flat)
![Platform](https://img.shields.io/badge/platform-macos%20%7C%20linux-lightgrey?style=flat)
![](https://img.shields.io/badge/version-0.0.1-blue.svg?style=flat)

A python module for interfacing with the DualSky XGPS160. This project was primarily motivated because I wanted an easy way to download all the tracks record in the GPS without going through the painful iOS app the vendor provided.

Luckily the vendor also provided ObjC code that has information to implement an interface to retrieve this data.

The script `read_from_gps.py` will accomplish this. It was written on a Mac and thus the format of the tty for the bluetooth serial connection is hardcoded to that but it should be easy enough to get it to run on other platforms provided you can get a bluetooth serial connection up.

The ObjectC this was copied from is at: [https://github.com/dualav/XGPS160-SDK-iOS](dualav/XGPS160-SDK-iOS)

# Example code

This will connect to the GPS160 and get the recorded logs:

```python
    loop = asyncio.get_event_loop()
    ser_port = xgps_serialport()
    xgps = await XGPS160.connect(str(ser_port), loop=loop)

    await xgps.get_recorded_logs()
    print(f"number of log list entries: {len(xgps.log_list_entries)}")
    print(f"percent storage used: {xgps.get_used_storage_percent()}")
    gps_tracks = []
    for log_list in xgps.log_list_entries:
        gps_data = await xgps.get_gps_sample_data_for_log_list_item(log_list)
        gps_tracks.append(gps_data)
```
