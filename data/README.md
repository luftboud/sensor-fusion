# Local datasets

Reference trajectories and logs stay on this machine. Git tracks this file only. Everything else under `data/` is ignored.

Expected layout once the files are downloaded or recorded:

```text
data/
  kitti/     KITTI sequence used as the public reference trajectory
  track/     Our own recorded GNSS and IMU log
```

Do not create those directories until the files exist. Weeks 1–2 cover fetching KITTI. Week 9 covers the recorded track.

## KITTI

One drive: `2011_09_26`, `drive_0009`, synced archive, plus that day's calibration. The drive lasts about 46 seconds, long enough for a later 15-second GNSS gap with time before and after it. Run the commands from the repository root.

```bash
mkdir -p data/kitti && cd data/kitti
curl -fL -O https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0009/2011_09_26_drive_0009_sync.zip
curl -fL -O https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip
unzip 2011_09_26_drive_0009_sync.zip
unzip 2011_09_26_calib.zip
```

The synced zip is 1,791,264,044 bytes. That is the file size, not the rounded figure from `du`. From `data/kitti`, these paths should open:

```text
2011_09_26/2011_09_26_drive_0009_sync/oxts/data/*.txt
2011_09_26/2011_09_26_drive_0009_sync/oxts/timestamps.txt
2011_09_26/2011_09_26_drive_0009_sync/oxts/dataformat.txt
```
