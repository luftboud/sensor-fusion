# Local datasets

Reference trajectories and logs stay on this machine. Git tracks this file only. Everything else under `data/` is ignored.

Expected layout once the files are downloaded or recorded:

```text
data/
  kitti/     KITTI sequence used as the public reference trajectory
  track/     Our own recorded GNSS and IMU log
```

Do not create those directories until the files exist. Weeks 1–2 cover fetching KITTI. Week 9 covers the recorded track.
