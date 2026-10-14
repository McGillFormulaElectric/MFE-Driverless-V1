
## 2026-10-09 16:37:58 - run

bag `test_bag/accel_cuda_test`, node src md5 `e4a59ac5`, cmd `--rate 1.0`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 16:38:22 | run |  | all | (defaults) | 1.000 | 0.400 | 0.571 | 0.107 | 13.7 | 16.2 | **best** |

## 2026-10-09 16:38:23 - tune

bag `test_bag/accel_cuda_test`, node src md5 `e4a59ac5`, cmd `--tune 25 --rate 1.0`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 16:38:48 | tune | 0 | train | leaf_size=0.085 ground_threshold=0.1873 cluster_tolerance=0.5617 min_cluster_size=3 far_min_cluster_size=2 near_far_split_x=7.5213 height_min=0.0466 height_max=0.8459 | 0.000 | 0.000 | 0.000 | nan | 6.7 | 10.6 | **best** |
| 16:39:12 | tune | 1 | train | leaf_size=0.1867 ground_threshold=0.1144 cluster_tolerance=0.675 min_cluster_size=3 far_min_cluster_size=2 near_far_split_x=9.4792 height_min=0.0117 height_max=0.4436 | 0.000 | 0.000 | 0.000 | nan | 4.4 | 4.9 | **best** |
| 16:39:36 | tune | 2 | train | leaf_size=0.0312 ground_threshold=0.2132 cluster_tolerance=0.6669 min_cluster_size=4 far_min_cluster_size=3 near_far_split_x=8.5941 height_min=0.0488 height_max=0.7903 | 0.000 | 0.000 | 0.000 | nan | 36.0 | 41.3 |  |
| 16:40:00 | tune | 3 | train | leaf_size=0.0375 ground_threshold=0.1708 cluster_tolerance=0.286 min_cluster_size=4 far_min_cluster_size=2 near_far_split_x=5.9026 height_min=0.0301 height_max=0.7871 | 0.000 | 0.000 | 0.000 | nan | 23.8 | 26.9 |  |
| 16:40:24 | tune | 4 | train | leaf_size=0.0713 ground_threshold=0.1551 cluster_tolerance=0.2113 min_cluster_size=3 far_min_cluster_size=2 near_far_split_x=7.3185 height_min=0.0947 height_max=0.7409 | 0.000 | 0.000 | 0.000 | nan | 7.9 | 8.7 |  |
| 16:40:48 | tune | 5 | train | leaf_size=0.0593 ground_threshold=0.1261 cluster_tolerance=0.6186 min_cluster_size=1 far_min_cluster_size=3 near_far_split_x=7.6945 height_min=0.025 height_max=0.4645 | 0.333 | 0.200 | 0.250 | 0.152 | 9.5 | 11.4 | **best** |
| 16:41:12 | tune | 6 | train | leaf_size=0.0546 ground_threshold=0.11 cluster_tolerance=0.5421 min_cluster_size=2 far_min_cluster_size=3 near_far_split_x=3.7143 height_min=0.0248 height_max=0.4807 | 0.000 | 0.000 | 0.000 | nan | 11.3 | 13.1 |  |
| 16:41:36 | tune | 7 | train | leaf_size=0.1036 ground_threshold=0.0857 cluster_tolerance=0.4798 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=3.7726 height_min=0.0674 height_max=0.4691 | 0.600 | 0.600 | 0.600 | 0.121 | 5.6 | 6.2 | **best** |
| 16:42:00 | tune | 8 | train | leaf_size=0.0436 ground_threshold=0.1111 cluster_tolerance=0.6926 min_cluster_size=1 far_min_cluster_size=3 near_far_split_x=3.6727 height_min=0.0978 height_max=0.6343 | 0.000 | 0.000 | 0.000 | nan | 16.7 | 19.1 |  |
| 16:42:24 | tune | 9 | train | leaf_size=0.1914 ground_threshold=0.1631 cluster_tolerance=0.6436 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=3.8414 height_min=0.0331 height_max=0.4594 | 0.676 | 0.302 | 0.417 | 0.117 | 4.3 | 4.9 |  |
| 16:42:48 | tune | 10 | train | leaf_size=0.1995 ground_threshold=0.0579 cluster_tolerance=0.4508 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=4.8339 height_min=0.0622 height_max=0.5387 | 0.635 | 0.694 | 0.663 | 0.118 | 4.6 | 5.3 | **best** |
| 16:43:12 | tune | 11 | train | leaf_size=0.1689 ground_threshold=0.0702 cluster_tolerance=0.4764 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=5.7065 height_min=0.0569 height_max=0.5001 | 0.600 | 0.600 | 0.600 | 0.121 | 4.5 | 5.6 |  |
| 16:43:37 | tune | 12 | train | leaf_size=0.1582 ground_threshold=0.1165 cluster_tolerance=0.3795 min_cluster_size=1 far_min_cluster_size=2 near_far_split_x=5.5477 height_min=0.0631 height_max=0.5538 | 0.333 | 0.200 | 0.250 | 0.148 | 4.5 | 5.0 |  |
| 16:44:01 | tune | 13 | train | leaf_size=0.1962 ground_threshold=0.0339 cluster_tolerance=0.6112 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=3.9726 height_min=0.0665 height_max=0.6649 | 0.662 | 0.393 | 0.493 | 0.096 | 4.3 | 4.9 |  |
| 16:44:25 | tune | 14 | train | leaf_size=0.134 ground_threshold=0.0751 cluster_tolerance=0.6369 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=5.8709 height_min=0.0395 height_max=0.422 | 0.600 | 0.600 | 0.600 | 0.121 | 4.6 | 5.2 |  |
| 16:44:49 | tune | 15 | train | leaf_size=0.1067 ground_threshold=0.0408 cluster_tolerance=0.3441 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=6.496 height_min=0.0635 height_max=0.6072 | 1.000 | 0.600 | 0.750 | 0.120 | 5.1 | 5.7 | **best** |
| 16:45:13 | tune | 16 | train | leaf_size=0.1501 ground_threshold=0.0476 cluster_tolerance=0.3145 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=3.8228 height_min=0.0764 height_max=0.5182 | 0.750 | 0.600 | 0.667 | 0.122 | 4.6 | 6.6 |  |
| 16:45:37 | tune | 17 | train | leaf_size=0.1338 ground_threshold=0.0626 cluster_tolerance=0.351 min_cluster_size=3 far_min_cluster_size=1 near_far_split_x=3.0159 height_min=0.0524 height_max=0.7128 | 0.759 | 0.630 | 0.688 | 0.118 | 5.0 | 5.9 |  |
| 16:46:01 | tune | 18 | train | leaf_size=0.0853 ground_threshold=0.0759 cluster_tolerance=0.4465 min_cluster_size=3 far_min_cluster_size=1 near_far_split_x=3.5404 height_min=0.0439 height_max=0.6912 | 0.750 | 0.600 | 0.667 | 0.121 | 6.6 | 7.6 |  |
| 16:46:25 | tune | 19 | train | leaf_size=0.114 ground_threshold=0.0783 cluster_tolerance=0.3122 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=7.8036 height_min=0.0728 height_max=0.6965 | 1.000 | 0.552 | 0.711 | 0.119 | 5.1 | 6.5 |  |
| 16:46:50 | tune | 20 | train | leaf_size=0.133 ground_threshold=0.0663 cluster_tolerance=0.3452 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=7.4951 height_min=0.0605 height_max=0.6218 | 0.600 | 0.600 | 0.600 | 0.121 | 5.0 | 5.9 |  |
| 16:47:14 | tune | 21 | train | leaf_size=0.1334 ground_threshold=0.0836 cluster_tolerance=0.2166 min_cluster_size=3 far_min_cluster_size=1 near_far_split_x=7.1944 height_min=0.0655 height_max=0.7221 | 1.000 | 0.400 | 0.571 | 0.107 | 4.7 | 5.2 |  |
| 16:47:38 | tune | 22 | train | leaf_size=0.1404 ground_threshold=0.1224 cluster_tolerance=0.3694 min_cluster_size=3 far_min_cluster_size=1 near_far_split_x=7.83 height_min=0.0642 height_max=0.7015 | 1.000 | 0.400 | 0.571 | 0.107 | 4.9 | 6.2 |  |
| 16:48:02 | tune | 23 | train | leaf_size=0.0843 ground_threshold=0.0763 cluster_tolerance=0.3474 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=6.9699 height_min=0.0424 height_max=0.5634 | 1.000 | 0.522 | 0.686 | 0.121 | 6.4 | 7.3 |  |
| 16:48:27 | tune | 24 | train | leaf_size=0.1059 ground_threshold=0.1063 cluster_tolerance=0.3144 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=6.5897 height_min=0.0802 height_max=0.6354 | 1.000 | 0.400 | 0.571 | 0.107 | 5.3 | 6.2 |  |
| 16:48:51 | best |  | train | leaf_size=0.1067 ground_threshold=0.0408 cluster_tolerance=0.3441 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=6.496 height_min=0.0635 height_max=0.6072 | 1.000 | 0.600 | 0.750 | 0.120 | 5.4 | 6.2 |  |
| 16:48:51 | best | 15 | test | leaf_size=0.1067 ground_threshold=0.0408 cluster_tolerance=0.3441 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=6.496 height_min=0.0635 height_max=0.6072 | 1.000 | 0.600 | 0.750 | 0.120 | 5.4 | 6.2 |  |

### Tune summary (best trial 15 of 25, train reward 0.595)

| split | P | R | F1 | err m | lat ms |
|---|---|---|---|---|---|
| train | 1.000 | 0.600 | 0.750 | 0.120 | 5.4 |
| test | 1.000 | 0.600 | 0.750 | 0.120 | 5.4 |

params: `{'leaf_size': 0.10665227213483826, 'ground_threshold': 0.04083386017610074, 'cluster_tolerance': 0.34410604518517085, 'min_cluster_size': 2, 'far_min_cluster_size': 1, 'near_far_split_x': 6.496012815127685, 'height_min': 0.0635178197260607, 'height_max': 0.6072173707452804}`

```
--ros-args -p leaf_size:=0.1067 -p ground_threshold:=0.0408 -p cluster_tolerance:=0.3441 -p min_cluster_size:=2 -p far_min_cluster_size:=1 -p near_far_split_x:=6.496 -p height_min:=0.0635 -p height_max:=0.6072
```

## 2026-10-09 18:06:39 - run

bag `test_bag/accel_cuda_test`, node src md5 `014800d0`, cmd ``. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:07:49 | run |  | all | (defaults) | 1.000 | 0.400 | 0.571 | 0.107 | 13.1 | 14.7 | **best** |

## 2026-10-09 18:08:30 - run

bag `test_bag/accel_cuda_test`, node src md5 `014800d0`, cmd `-p leaf_size=0.05 -p ground_threshold=0.04 -p cluster_tolerance=0.4 -p min_cluster_size=1 -p far_min_cluster_size=1 -p cluster_2d=True -p height_min=0.02 -p rf_model_path=/home/neil/Develop/cudaPCL/eval/rf_model.txt --rate 1.0`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:08:54 | run |  | all | leaf_size=0.05 ground_threshold=0.04 cluster_tolerance=0.4 min_cluster_size=1 far_min_cluster_size=1 cluster_2d=True height_min=0.02 rf_model_path=/home/neil/Develop/cudaPCL/eval/rf_model.txt | 1.000 | 0.800 | 0.889 | 0.121 | 13.3 | 14.4 | **best** |

## 2026-10-09 18:51:32 - run

bag `test_bag/accel_moving`, node src md5 `014800d0`, cmd ``. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:52:43 | run |  | all | (defaults) | 0.997 | 0.615 | 0.761 | 0.171 | 13.3 | 15.2 | **best** |

## 2026-10-09 18:52:50 - run

bag `test_bag/accel_moving`, node src md5 `014800d0`, cmd `--rate 2.5`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:53:22 | run |  | all | (defaults) | 0.997 | 0.615 | 0.761 | 0.171 | 13.2 | 14.6 | **best** |

## 2026-10-09 18:53:23 - run

bag `test_bag/accel_moving`, node src md5 `014800d0`, cmd `--rate 5`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:53:41 | run |  | all | (defaults) | 0.997 | 0.615 | 0.761 | 0.171 | 13.2 | 15.2 | **best** |

## 2026-10-09 18:54:27 - run

bag `test_bag/accel_moving`, node src md5 `f425ced2`, cmd `--rate 2.5`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:54:58 | run |  | all | (defaults) | 0.997 | 0.615 | 0.761 | 0.171 | 11.6 | 13.1 | **best** |

## 2026-10-09 18:55:45 - run

bag `test_bag/accel_moving`, node src md5 `3d824dba`, cmd `--rate 2.5 -p gpu_voxel=true`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:56:16 | run |  | all | gpu_voxel=true | 0.997 | 0.615 | 0.761 | 0.171 | 11.6 | 13.2 | **best** |

## 2026-10-09 18:56:17 - run

bag `test_bag/accel_moving`, node src md5 `3d824dba`, cmd `--rate 2.5 -p gpu_voxel=false`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:56:48 | run |  | all | gpu_voxel=false | 0.997 | 0.615 | 0.761 | 0.171 | 2.3 | 2.5 | **best** |

## 2026-10-09 18:58:17 - run

bag `test_bag/accel_moving`, node src md5 `3d824dba`, cmd `--rate 2.5 -p gpu_voxel=False`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:58:48 | run |  | all | gpu_voxel=False | 0.997 | 0.615 | 0.761 | 0.171 | 2.3 | 2.6 | **best** |

## 2026-10-09 18:58:49 - run

bag `test_bag/accel_moving`, node src md5 `3d824dba`, cmd `--rate 2.5 -p gpu_voxel=False -p rf_model_path=/home/neil/Develop/cudaPCL/eval/rf_model_70.txt -p min_cluster_size=1 -p far_min_cluster_size=1 -p cluster_2d=True -p ground_threshold=0.04 -p height_min=0.02`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:59:20 | run |  | all | gpu_voxel=False rf_model_path=/home/neil/Develop/cudaPCL/eval/rf_model_70.txt min_cluster_size=1 far_min_cluster_size=1 cluster_2d=True ground_threshold=0.04 height_min=0.02 | 0.999 | 0.714 | 0.832 | 0.143 | 2.3 | 2.5 | **best** |

## 2026-10-09 18:59:28 - run

bag `test_bag/accel_moving`, node src md5 `3d824dba`, cmd `--rate 2.5 -p gpu_voxel=False --json /tmp/claude-1000/-home-neil-Develop-cudaPCL/47008f88-cb0a-4931-b9a9-77057b2eb944/scratchpad/base.json`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:59:59 | run |  | all | gpu_voxel=False | 0.997 | 0.615 | 0.761 | 0.171 | 2.3 | 2.2 | **best** |

## 2026-10-09 19:00:00 - run

bag `test_bag/accel_moving`, node src md5 `3d824dba`, cmd `--rate 2.5 -p gpu_voxel=False -p rf_model_path=/home/neil/Develop/cudaPCL/eval/rf_model_70.txt -p min_cluster_size=1 -p far_min_cluster_size=1 -p cluster_2d=True -p ground_threshold=0.04 -p height_min=0.02 --json /tmp/claude-1000/-home-neil-Develop-cudaPCL/47008f88-cb0a-4931-b9a9-77057b2eb944/scratchpad/rf.json`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 19:00:31 | run |  | all | gpu_voxel=False rf_model_path=/home/neil/Develop/cudaPCL/eval/rf_model_70.txt min_cluster_size=1 far_min_cluster_size=1 cluster_2d=True ground_threshold=0.04 height_min=0.02 | 0.999 | 0.714 | 0.832 | 0.143 | 2.4 | 2.8 | **best** |

## 2026-10-09 19:00:39 - tune

bag `test_bag/accel_moving`, node src md5 `3d824dba`, cmd `--tune 40 --rate 5 -p gpu_voxel=False --study eval/study_moving.db --json /tmp/claude-1000/-home-neil-Develop-cudaPCL/47008f88-cb0a-4931-b9a9-77057b2eb944/scratchpad/tune_moving.json`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 19:00:58 | tune | 0 | train | gpu_voxel=False leaf_size=0.085 ground_threshold=0.1873 cluster_tolerance=0.5617 min_cluster_size=3 far_min_cluster_size=2 near_far_split_x=7.5213 height_min=0.0466 height_max=0.8459 cluster_2d=True min_cluster_dz=0.1583 | 1.000 | 0.012 | 0.023 | 0.281 | 2.0 | 2.1 | **best** |
| 19:01:15 | tune | 1 | train | gpu_voxel=False leaf_size=0.0818 ground_threshold=0.155 cluster_tolerance=0.7554 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=3.1415 height_min=0.0841 height_max=0.7891 cluster_2d=False min_cluster_dz=0.1598 | 0.997 | 0.243 | 0.390 | 0.216 | 1.9 | 1.9 | **best** |
| 19:01:33 | tune | 2 | train | gpu_voxel=False leaf_size=0.072 ground_threshold=0.2017 cluster_tolerance=0.271 min_cluster_size=3 far_min_cluster_size=1 near_far_split_x=9.6127 height_min=0.0546 height_max=0.6073 cluster_2d=False min_cluster_dz=0.0912 | 1.000 | 0.251 | 0.402 | 0.145 | 2.0 | 2.1 | **best** |
| 19:01:51 | tune | 3 | train | gpu_voxel=False leaf_size=0.0882 ground_threshold=0.0341 cluster_tolerance=0.5706 min_cluster_size=3 far_min_cluster_size=2 near_far_split_x=9.6062 height_min=0.0698 height_max=0.5798 cluster_2d=False min_cluster_dz=0.012 | 0.991 | 0.162 | 0.278 | 0.239 | 2.0 | 2.0 |  |
| 19:02:09 | tune | 4 | train | gpu_voxel=False leaf_size=0.1063 ground_threshold=0.1775 cluster_tolerance=0.3262 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=5.546 height_min=0.0592 height_max=0.6193 cluster_2d=True min_cluster_dz=0.0418 | 1.000 | 0.454 | 0.625 | 0.184 | 1.9 | 1.9 | **best** |
| 19:02:27 | tune | 5 | train | gpu_voxel=False leaf_size=0.0407 ground_threshold=0.1737 cluster_tolerance=0.352 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=4.1128 height_min=0.0155 height_max=0.7282 cluster_2d=False min_cluster_dz=0.0737 | 1.000 | 0.435 | 0.606 | 0.179 | 2.1 | 2.1 |  |
| 19:02:44 | tune | 6 | train | gpu_voxel=False leaf_size=0.1424 ground_threshold=0.0514 cluster_tolerance=0.7028 min_cluster_size=1 far_min_cluster_size=3 near_far_split_x=6.2806 height_min=0.0978 height_max=0.7024 cluster_2d=True min_cluster_dz=0.0566 | 0.989 | 0.135 | 0.238 | 0.250 | 1.9 | 1.7 |  |
| 19:03:02 | tune | 7 | train | gpu_voxel=False leaf_size=0.0377 ground_threshold=0.0952 cluster_tolerance=0.2712 min_cluster_size=2 far_min_cluster_size=2 near_far_split_x=3.449 height_min=0.0708 height_max=0.6833 cluster_2d=False min_cluster_dz=0.0188 | 0.997 | 0.422 | 0.593 | 0.182 | 2.1 | 2.5 |  |
| 19:03:20 | tune | 8 | train | gpu_voxel=False leaf_size=0.0895 ground_threshold=0.2344 cluster_tolerance=0.3911 min_cluster_size=3 far_min_cluster_size=1 near_far_split_x=8.0143 height_min=0.0325 height_max=0.4916 cluster_2d=True min_cluster_dz=0.1658 | 1.000 | 0.246 | 0.395 | 0.147 | 1.9 | 1.9 |  |
| 19:03:39 | tune | 9 | train | gpu_voxel=False leaf_size=0.0303 ground_threshold=0.1791 cluster_tolerance=0.362 min_cluster_size=3 far_min_cluster_size=3 near_far_split_x=4.7413 height_min=0.0597 height_max=0.696 cluster_2d=True min_cluster_dz=0.1905 | 1.000 | 0.226 | 0.369 | 0.134 | 2.1 | 2.5 |  |
| 19:03:56 | tune | 10 | train | gpu_voxel=False leaf_size=0.1379 ground_threshold=0.1954 cluster_tolerance=0.3122 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=6.6634 height_min=0.0228 height_max=0.4113 cluster_2d=True min_cluster_dz=0.0105 | 0.997 | 0.450 | 0.620 | 0.182 | 1.9 | 1.9 |  |
| 19:04:15 | tune | 11 | train | gpu_voxel=False leaf_size=0.1632 ground_threshold=0.1496 cluster_tolerance=0.4157 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=7.3995 height_min=0.0308 height_max=0.4044 cluster_2d=True min_cluster_dz=0.0381 | 0.997 | 0.480 | 0.648 | 0.193 | 1.8 | 1.8 | **best** |
| 19:04:32 | tune | 12 | train | gpu_voxel=False leaf_size=0.0908 ground_threshold=0.1601 cluster_tolerance=0.5195 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=8.8093 height_min=0.0501 height_max=0.4315 cluster_2d=True min_cluster_dz=0.0518 | 1.000 | 0.286 | 0.445 | 0.235 | 2.0 | 2.0 |  |
| 19:04:50 | tune | 13 | train | gpu_voxel=False leaf_size=0.1122 ground_threshold=0.1415 cluster_tolerance=0.4196 min_cluster_size=1 far_min_cluster_size=2 near_far_split_x=6.0935 height_min=0.0155 height_max=0.5019 cluster_2d=True min_cluster_dz=0.024 | 0.996 | 0.372 | 0.541 | 0.183 | 1.8 | 1.8 |  |
| 19:05:08 | tune | 14 | train | gpu_voxel=False leaf_size=0.0577 ground_threshold=0.1881 cluster_tolerance=0.3407 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=7.1483 height_min=0.0791 height_max=0.6038 cluster_2d=True min_cluster_dz=0.0151 | 0.997 | 0.479 | 0.647 | 0.185 | 2.1 | 2.2 |  |
| 19:05:26 | tune | 15 | train | gpu_voxel=False leaf_size=0.0564 ground_threshold=0.1206 cluster_tolerance=0.2617 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=6.6645 height_min=0.0399 height_max=0.5724 cluster_2d=True min_cluster_dz=0.0684 | 0.995 | 0.535 | 0.695 | 0.187 | 2.0 | 2.2 | **best** |
| 19:05:43 | tune | 16 | train | gpu_voxel=False leaf_size=0.1431 ground_threshold=0.1862 cluster_tolerance=0.4401 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=8.4561 height_min=0.023 height_max=0.4423 cluster_2d=True min_cluster_dz=0.0586 | 0.997 | 0.445 | 0.616 | 0.185 | 1.8 | 1.9 |  |
| 19:06:01 | tune | 17 | train | gpu_voxel=False leaf_size=0.0787 ground_threshold=0.0964 cluster_tolerance=0.2183 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=7.109 height_min=0.0461 height_max=0.5287 cluster_2d=True min_cluster_dz=0.1062 | 0.998 | 0.601 | 0.750 | 0.172 | 2.1 | 2.1 | **best** |
| 19:06:19 | tune | 18 | train | gpu_voxel=False leaf_size=0.1056 ground_threshold=0.0492 cluster_tolerance=0.3068 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=5.3985 height_min=0.0427 height_max=0.5743 cluster_2d=True min_cluster_dz=0.1189 | 0.998 | 0.722 | 0.838 | 0.173 | 2.0 | 2.0 | **best** |
| 19:06:37 | tune | 19 | train | gpu_voxel=False leaf_size=0.0854 ground_threshold=0.0763 cluster_tolerance=0.3111 min_cluster_size=3 far_min_cluster_size=1 near_far_split_x=7.0408 height_min=0.0256 height_max=0.4789 cluster_2d=True min_cluster_dz=0.0999 | 0.998 | 0.625 | 0.768 | 0.179 | 2.1 | 2.3 |  |
| 19:06:55 | tune | 20 | train | gpu_voxel=False leaf_size=0.0825 ground_threshold=0.0573 cluster_tolerance=0.4393 min_cluster_size=3 far_min_cluster_size=1 near_far_split_x=6.865 height_min=0.0174 height_max=0.5402 cluster_2d=True min_cluster_dz=0.087 | 0.993 | 0.447 | 0.616 | 0.205 | 2.1 | 2.1 |  |
| 19:07:12 | tune | 21 | train | gpu_voxel=False leaf_size=0.061 ground_threshold=0.1109 cluster_tolerance=0.3111 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=5.8036 height_min=0.0398 height_max=0.5647 cluster_2d=True min_cluster_dz=0.117 | 0.997 | 0.459 | 0.629 | 0.179 | 2.1 | 2.2 |  |
| 19:07:30 | tune | 22 | train | gpu_voxel=False leaf_size=0.0789 ground_threshold=0.039 cluster_tolerance=0.2527 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=7.7838 height_min=0.0512 height_max=0.5697 cluster_2d=True min_cluster_dz=0.0852 | 0.998 | 0.760 | 0.863 | 0.176 | 2.1 | 2.1 | **best** |
| 19:07:48 | tune | 23 | train | gpu_voxel=False leaf_size=0.1047 ground_threshold=0.0582 cluster_tolerance=0.3995 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=6.4823 height_min=0.0354 height_max=0.5892 cluster_2d=True min_cluster_dz=0.1076 | 0.998 | 0.622 | 0.766 | 0.180 | 2.0 | 2.0 |  |
| 19:08:05 | tune | 24 | train | gpu_voxel=False leaf_size=0.1012 ground_threshold=0.0517 cluster_tolerance=0.2815 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=5.1881 height_min=0.0363 height_max=0.4954 cluster_2d=True min_cluster_dz=0.1255 | 0.997 | 0.653 | 0.789 | 0.174 | 2.0 | 2.0 |  |
| 19:08:23 | tune | 25 | train | gpu_voxel=False leaf_size=0.0984 ground_threshold=0.0378 cluster_tolerance=0.3462 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=5.7581 height_min=0.0477 height_max=0.5889 cluster_2d=True min_cluster_dz=0.1303 | 0.998 | 0.710 | 0.830 | 0.172 | 2.0 | 2.0 |  |
| 19:08:41 | tune | 26 | train | gpu_voxel=False leaf_size=0.0751 ground_threshold=0.0549 cluster_tolerance=0.2672 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=7.3432 height_min=0.0499 height_max=0.5431 cluster_2d=True min_cluster_dz=0.0449 | 0.998 | 0.681 | 0.810 | 0.183 | 2.0 | 2.2 |  |
| 19:08:59 | tune | 27 | train | gpu_voxel=False leaf_size=0.0915 ground_threshold=0.0537 cluster_tolerance=0.2917 min_cluster_size=1 far_min_cluster_size=1 near_far_split_x=5.7087 height_min=0.0315 height_max=0.6298 cluster_2d=True min_cluster_dz=0.0846 | 0.994 | 0.512 | 0.676 | 0.198 | 2.0 | 2.1 |  |
| 19:09:16 | tune | 28 | train | gpu_voxel=False leaf_size=0.0591 ground_threshold=0.0434 cluster_tolerance=0.215 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=6.3978 height_min=0.0408 height_max=0.5433 cluster_2d=True min_cluster_dz=0.0844 | 0.997 | 0.785 | 0.878 | 0.179 | 2.2 | 2.3 | **best** |
| 19:09:34 | tune | 29 | train | gpu_voxel=False leaf_size=0.1144 ground_threshold=0.0616 cluster_tolerance=0.2513 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=7.5865 height_min=0.0346 height_max=0.4561 cluster_2d=True min_cluster_dz=0.1211 | 0.997 | 0.580 | 0.733 | 0.172 | 2.0 | 2.0 |  |
| 19:09:52 | tune | 30 | train | gpu_voxel=False leaf_size=0.0852 ground_threshold=0.0401 cluster_tolerance=0.2449 min_cluster_size=2 far_min_cluster_size=2 near_far_split_x=7.3684 height_min=0.045 height_max=0.5674 cluster_2d=True min_cluster_dz=0.1184 | 0.996 | 0.388 | 0.558 | 0.177 | 2.1 | 2.2 |  |
| 19:10:10 | tune | 31 | train | gpu_voxel=False leaf_size=0.0537 ground_threshold=0.0437 cluster_tolerance=0.3216 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=7.1125 height_min=0.0481 height_max=0.5636 cluster_2d=True min_cluster_dz=0.1386 | 0.998 | 0.715 | 0.833 | 0.172 | 2.2 | 2.2 |  |
| 19:10:28 | tune | 32 | train | gpu_voxel=False leaf_size=0.0644 ground_threshold=0.0698 cluster_tolerance=0.2091 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=6.6321 height_min=0.0258 height_max=0.5263 cluster_2d=True min_cluster_dz=0.0722 | 0.998 | 0.670 | 0.802 | 0.181 | 2.0 | 2.1 |  |
| 19:10:45 | tune | 33 | train | gpu_voxel=False leaf_size=0.0803 ground_threshold=0.0414 cluster_tolerance=0.2514 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=5.5311 height_min=0.0549 height_max=0.6408 cluster_2d=True min_cluster_dz=0.1242 | 0.998 | 0.615 | 0.761 | 0.177 | 2.0 | 2.1 |  |
| 19:11:03 | tune | 34 | train | gpu_voxel=False leaf_size=0.054 ground_threshold=0.0367 cluster_tolerance=0.36 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=7.0411 height_min=0.0279 height_max=0.6346 cluster_2d=True min_cluster_dz=0.1423 | 0.998 | 0.712 | 0.831 | 0.172 | 2.1 | 2.3 |  |
| 19:11:21 | tune | 35 | train | gpu_voxel=False leaf_size=0.1211 ground_threshold=0.0727 cluster_tolerance=0.32 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=5.5394 height_min=0.0516 height_max=0.5154 cluster_2d=True min_cluster_dz=0.1281 | 0.997 | 0.569 | 0.724 | 0.177 | 1.9 | 2.1 |  |
| 19:11:39 | tune | 36 | train | gpu_voxel=False leaf_size=0.0603 ground_threshold=0.056 cluster_tolerance=0.2133 min_cluster_size=3 far_min_cluster_size=1 near_far_split_x=8.2895 height_min=0.0475 height_max=0.6513 cluster_2d=True min_cluster_dz=0.1231 | 0.997 | 0.505 | 0.671 | 0.183 | 2.1 | 2.1 |  |
| 19:11:56 | tune | 37 | train | gpu_voxel=False leaf_size=0.1019 ground_threshold=0.0696 cluster_tolerance=0.2864 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=5.8754 height_min=0.0417 height_max=0.6016 cluster_2d=True min_cluster_dz=0.0647 | 0.997 | 0.671 | 0.802 | 0.184 | 2.0 | 2.0 |  |
| 19:12:14 | tune | 38 | train | gpu_voxel=False leaf_size=0.0408 ground_threshold=0.0579 cluster_tolerance=0.3003 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=6.713 height_min=0.055 height_max=0.5693 cluster_2d=True min_cluster_dz=0.1579 | 0.997 | 0.574 | 0.729 | 0.168 | 2.2 | 2.3 |  |
| 19:12:33 | tune | 39 | train | gpu_voxel=False leaf_size=0.059 ground_threshold=0.0572 cluster_tolerance=0.2946 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=8.2083 height_min=0.0328 height_max=0.5755 cluster_2d=True min_cluster_dz=0.1487 | 0.997 | 0.468 | 0.637 | 0.177 | 2.2 | 2.1 |  |
| 19:12:50 | best |  | train | gpu_voxel=False leaf_size=0.0591 ground_threshold=0.0434 cluster_tolerance=0.215 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=6.3978 height_min=0.0408 height_max=0.5433 cluster_2d=True min_cluster_dz=0.0844 | 0.997 | 0.785 | 0.878 | 0.179 | 2.2 | 2.3 | **best** |
| 19:12:50 | best | 28 | test | gpu_voxel=False leaf_size=0.0591 ground_threshold=0.0434 cluster_tolerance=0.215 min_cluster_size=2 far_min_cluster_size=1 near_far_split_x=6.3978 height_min=0.0408 height_max=0.5433 cluster_2d=True min_cluster_dz=0.0844 | 1.000 | 0.500 | 0.667 | 0.133 | 2.2 | 2.3 |  |

### Tune summary (best trial 28 of 40, train reward 0.782)

| split | P | R | F1 | err m | lat ms |
|---|---|---|---|---|---|
| train | 0.997 | 0.785 | 0.878 | 0.179 | 2.2 |
| test | 1.000 | 0.500 | 0.667 | 0.133 | 2.2 |

params: `{'leaf_size': 0.05908672178170442, 'ground_threshold': 0.043427691824194835, 'cluster_tolerance': 0.21496251955315687, 'min_cluster_size': 2, 'far_min_cluster_size': 1, 'near_far_split_x': 6.3978135401517475, 'height_min': 0.04075738915430666, 'height_max': 0.5432692510455538, 'cluster_2d': True, 'min_cluster_dz': 0.0844134196069756}`

```
--ros-args -p gpu_voxel:=False -p leaf_size:=0.0591 -p ground_threshold:=0.0434 -p cluster_tolerance:=0.215 -p min_cluster_size:=2 -p far_min_cluster_size:=1 -p near_far_split_x:=6.3978 -p height_min:=0.0408 -p height_max:=0.5433 -p cluster_2d:=True -p min_cluster_dz:=0.0844
```

## 2026-10-09 22:50:19 - run

bag `test_bag/accel_moving`, node src md5 `3d824dba`, cmd `--rate 5 -p gpu_voxel=False -p min_cluster_size=1 -p far_min_cluster_size=1 -p cluster_2d=True -p ground_threshold=0.04 -p height_min=0.02 --json /tmp/claude-1000/-home-neil-Develop-cudaPCL/47008f88-cb0a-4931-b9a9-77057b2eb944/scratchpad/abl_box.json`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 22:50:37 | run |  | all | gpu_voxel=False min_cluster_size=1 far_min_cluster_size=1 cluster_2d=True ground_threshold=0.04 height_min=0.02 | 0.995 | 0.813 | 0.894 | 0.164 | 2.2 | 2.5 | **best** |

## 2026-10-09 22:50:38 - run

bag `test_bag/accel_moving`, node src md5 `3d824dba`, cmd `--rate 5 -p gpu_voxel=False -p min_cluster_size=1 -p far_min_cluster_size=1 -p cluster_2d=True -p ground_threshold=0.04 -p height_min=0.02 -p rf_model_path=/home/neil/Develop/cudaPCL/eval/rf_model_70.txt --json /tmp/claude-1000/-home-neil-Develop-cudaPCL/47008f88-cb0a-4931-b9a9-77057b2eb944/scratchpad/abl_rf.json`. P/R/F1/err on the train split (first 70% of frames) unless row says `all`; best = new best reward so far.

| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 22:50:55 | run |  | all | gpu_voxel=False min_cluster_size=1 far_min_cluster_size=1 cluster_2d=True ground_threshold=0.04 height_min=0.02 rf_model_path=/home/neil/Develop/cudaPCL/eval/rf_model_70.txt | 0.999 | 0.714 | 0.832 | 0.143 | 2.3 | 2.5 | **best** |
