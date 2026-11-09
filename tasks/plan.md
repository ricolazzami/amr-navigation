# Implementation Plan: Project 3 — AMR Navigation (DecodeLabs)

Status: final · 2026-11-09
Target: simulasi saja (tidak ada robot fisik). Penilaian: 3 deliverable + repo + laporan + video.

## Overview

Membangun sistem navigasi AMR di simulasi ROS 2: robot mengolah LiDAR + wheel odometry + IMU, membangun occupancy grid 2D lewat SLAM online, melakukan state estimation dengan EKF, merencanakan jalur dengan A* buatan sendiri (heuristik Manhattan, 4-arah), lalu mengeksekusi jalur lewat Nav2 controller sambil menghindari obstacle dinamis dengan deselerasi halus dan replan.

Mengacu pada slide DecodeLabs Project 3 (26 halaman): tiga masalah navigasi (Where am I? / Where am I going? / How do I avoid that wall?), TF tree `map → odom → base_link`, occupancy grid `0/100/-1`, `f(n)=g(n)+h(n)`, inflation layer, dan reflex override `v = v_max · tanh(d − d_safe)`.

## Keputusan arsitektur (final)

| Topik | Keputusan | Alasan |
|---|---|---|
| Starter asset | Bikin sendiri, TurtleBot3 hanya acuan angka | Starter repo tidak tersedia; URDF sendiri = paham TF |
| Robot | Diff-drive custom, chassis box 0.25 × 0.20 m, 2 wheel + caster | Ringan di Gazebo; mesh TB3 terlalu berat |
| Distro | ROS 2 **Jazzy** + Gazebo **Harmonic** | LTS sampai 2029, Nav2 & gz-sim masih dirawat |
| Environment | **Docker** `ros:jazzy-ros-base` di Arch (host) | ROS 2 Tier 3 di Arch; paket Nav2/Gazebo rapuh |
| GUI | XWayland (`DISPLAY=:1`) + `QT_QPA_PLATFORM=xcb` + `xhost` | Host pakai Hyprland/Wayland |
| Akselerasi | **Software (llvmpipe) dulu, lalu iGPU AMD `/dev/dri` bila perlu** | NVIDIA toolkit ditunda: ROI buruk, Gazebo headless |
| Arsitektur | **Hybrid**: Nav2 costmap + controller, A* & avoidance milik sendiri | Semua deliverable tetap kode sendiri, "local costmap" & Nav2 nyata |
| Eksekusi jalur | A* kirim `FollowPath` action; controller output `/cmd_vel_nav` | Tidak ada dua node rebutan aktuator |
| Mapping | SLAM online `slam_toolbox`; **AMCL tidak dinyalakan** | Menghindari rebutan `map→odom` |
| Perilaku obstacle | Rem tanh → tunggu 1 s → tandai obstacle → replan | Memenuhi opsi "stop" dan "re-route" |
| Overlay obstacle | Disimpan di A* node, bukan inject ke static layer Nav2 | Static layer tidak bisa ditulisi saat runtime |
| Ground truth | `/world/<world>/dynamic_pose/info` via `ros_gz_bridge` | Metrik RMSE butuh posisi sebenarnya |
| Waktu | `use_sim_time:=true` di **semua** node | Trap nomor satu di container |

## Data flow

```
LiDAR ──► slam_toolbox ──► /map ─┬─► global_costmap (static + inflation)
                                 └─► a_star_node (Manhattan, 4-way, overlay)
                                        │ FollowPath action
                                        ▼
IMU ────┐                          controller_server
Wheel ──┴─► ekf_node ──► TF odom→base_link ──► /cmd_vel_nav
                                                    │
LiDAR ──► local_costmap ──► avoidance_node (tanh) ──┴─► /cmd_vel ──► robot
```

TF: `map → odom → base_link → {laser_link, imu_link, wheel links}`

## Struktur workspace

```
Project-3/
├── tasks/                      # plan.md, todo.md
├── src/
│   ├── amr_description/        # xacro URDF, sensor config
│   ├── amr_bringup/            # launch, params, worlds, maps
│   ├── amr_localization/       # ekf.yaml, slam_toolbox.yaml
│   ├── amr_planning/           # a_star_node.py, grid_utils.py, test/
│   └── amr_control/            # avoidance_node.py, watchdog.py
├── docs/                       # laporan, diagram, metrik, bag
├── scripts/                    # run_container.sh, record.sh, eval_metrics.py
└── README.md
```

## Fase kerja

### Phase 0 — Environment (½ hari)
- `xorg-xhost`, `xhost +local:docker`, pull `ros:jazzy-ros-base`.
- Paket minimal: `xacro, robot-state-publisher, joint-state-publisher, rviz2, nav2-bringup, slam-toolbox, robot-localization, ros-gz, teleop-twist-keyboard, tf2-tools, nav2-simple-commander, python3-numpy, python3-scipy, mesa-utils`.
- Container: `--network host --ipc host`, mount `/tmp/.X11-unix` dan `~/Projects/Project-3`.
- Git init + `.gitignore` (`build/ install/ log/ *.bag *.db3`).

### Phase 1 — Simulasi & robot (1–2 hari)
- World maze 10×10 m (grid 0.05 m → 200×200), dinding + 3 obstacle statis + 1 kotak bergerak.
- xacro diff-drive: wheel r=0.033, separation=0.16, `max_vel_x` 0.26 m/s.
- Plugin Gazebo: diff-drive (odom + TF), ray (LiDAR 360 sampel, 10 Hz, 0.12–3.5 m), IMU 50 Hz.
- Verifikasi `/scan`, `/wheel/odom`, `/imu/data`, `/cmd_vel`.

### Phase 2 — Mapping & EKF (2 hari) — Deliverable 1
- `ekf.yaml`: `two_d_mode: true`, `odom0: /wheel/odom`, `imu0: /imu/data`, `frequency: 30`, `publish_tf: true`, `world_frame: odom`.
- `slam_toolbox` async → `/map` + `map→odom`.
- Metrik: RMSE posisi (odom mentah vs EKF) terhadap ground truth; target penurunan ≥ 30%.
- Simpan peta dengan `map_saver` untuk lampiran.

### Phase 3 — A* (2–3 hari) — Deliverable 2
- `grid_utils.py`: `OccupancyGrid` → numpy; `>=65` blocked, `-1` diperlakukan blocked (konservatif), `0` free.
- `a_star_node.py`: 4-arah (bukan diagonal, supaya Manhattan admissible), `h = |dx| + |dy|`, `heapq`, closed-set numpy.
- Overlay obstacle dinamis: dict sel → timestamp, diterapkan sebelum planning, expired setelah N detik.
- Visualisasi `nav_msgs/Path` + Marker.
- Target: 200×200 < 100 ms; 400×400 < 600 ms dengan weighted A* (`h×1.2`).
- Unit test: konsistensi heuristic, kasus tanpa jalur, perbandingan biaya vs Dijkstra pada 20 peta.

### Phase 4 — Costmap & controller (2 hari)
- Nav2 costmap 2D: `obstacle_layer` (scan), `inflation_layer` (radius ≥ radius robot + margin, scaling 3–5), `static_layer` (`/map`).
- controller_server remap output ke `/cmd_vel_nav`; progress checker aktif.
- A* mengirim `FollowPath` action ke `/follow_path` setelah path dihitung.
- Verifikasi: 10/10 maze run tanpa tabrakan, deviasi lateral < 1 sel.

### Phase 5 — Dynamic avoidance (2–3 hari) — Deliverable 3
- `avoidance_node.py`: subscribe `/local_costmap/costmap` (200×200 rolling), hitung jarak obstacle terdekat di koridor depan.
- `error = d − d_safe`; `v = v_max · tanh(k·error)`, clamp; hard stop bila `d < d_min`.
- Steering menjauh (repulsive vector opsional) supaya tidak sekadar berhenti.
- Timer 1 s: bila masih terhalang → tandai sel di overlay A* → replan → kirim `FollowPath` baru.
- Watchdog: `/cmd_vel_nav` timeout 0.5 s → publish nol.
- Verifikasi: 10/10 bebas tabrakan; kecepatan monoton turun; osilasi < 2 zero-crossing per approach.

### Phase 6 — Integrasi & evaluasi (2 hari)
- Satu launch: `ros2 launch amr_bringup demo.launch.py world:=maze_dynamic use_sim_time:=true`.
- 5 skenario: maze statis; obstacle statis tak terduga; obstacle bergerak; jalan buntu; goal tidak terjangkau.
- Metrik per skenario: success, waktu, panjang jalur vs optimal, node diekspansi, min clearance, max decel, RMSE, CPU/RAM.
- Rekam `ros2 bag` + tangkapan RViz (map, costmap, path, TF tree).

### Phase 7 — Delivery (1–2 hari)
- README (instalasi, run, parameter, hasil).
- Laporan: arsitektur, TF tree, penjelasan `f=g+h`, tabel RMSE, grafik `v` vs jarak, tabel 5 skenario.
- Video 3–5 menit (`wf-recorder`): intro world, mapping, A* path, avoidance, grafik kecepatan. Demo utama di RViz2; Gazebo GUI hanya untuk klip pembuka lalu ditutup.

## Acceptance criteria

| # | Kriteria | Target |
|---|---|---|
| 1 | TF tree | `map→odom→base_link` utuh, tidak ada TF putus |
| 2 | Occupancy grid | dinding 100, bebas 0, unknown -1, peta tersimpan |
| 3 | EKF | `/odometry/filtered` ≥ 30 Hz, RMSE turun ≥ 30% |
| 4 | A* | Manhattan, 4-arah, optimal vs Dijkstra, 200×200 < 100 ms |
| 5 | Path | 10/10 maze run tanpa tabrakan |
| 6 | Avoidance | 10/10 bebas tabrakan, deselerasi halus, hard stop benar |
| 7 | Replan | obstacle menetap → replan dalam ≤ 1.5 s |
| 8 | Observability | RViz menampilkan map, costmap, path, robot, scan |

## Risiko

| Risiko | Dampak | Mitigasi |
|---|---|---|
| Wayland/Hyprland, RViz tak muncul di container | Blokir total | `xhost +local:docker`, `QT_QPA_PLATFORM=xcb`, mount `/tmp/.X11-unix` |
| FastDDS shared-memory gagal di container | Node tak saling menemukan | `--ipc host`; fallback `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| RAM 3 GB headroom | OOM saat demo | Gazebo headless, tutup browser, world kecil, RViz saja untuk rekam |
| `use_sim_time` tertinggal `false` | TF melompat, EKF kacau | Set di launch & tiap node; cek `ros2 param get` |
| AMCL & slam_toolbox jalan bersamaan | `map→odom` dobel | AMCL hanya launch terpisah, tidak pernah bersamaan |
| A* Python lambat di peta besar | Deadline demo | Weighted A*, downsample, target 200×200 |
| Avoidance osilasi | Robot goyang | Hysteresis + low-pass pada jarak + deadband |
| GPU NVIDIA | Waktu terbuang | **Ditunda**; software dulu, iGPU AMD bila perlu |

## Catatan environment (Arch host)

- Sesi: Hyprland/Wayland, XWayland di `:1`, socket `/tmp/.X11-unix/X1`.
- Docker aktif, user di grup `docker`, cgroup v2.
- GPU: GTX 1650 Mobile 4 GB (Optimus) + Radeon Renoir. `nvidia-container-toolkit` **tidak** dipasang.
- Swap 16 GB, disk 51 GB free.
- Rekaman: `wf-recorder` (OBS tidak dipakai di Hyprland).

## Pertanyaan terbuka

- Format pengumpulan resmi (repo + video + PDF?) dan apakah ada template laporan dari DecodeLabs.
- Apakah penilai mensyaratkan demo Gazebo GUI (bukan hanya RViz).
