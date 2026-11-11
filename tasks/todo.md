# Task List: Project 3 — AMR Navigation

Legend: `[ ]` todo · `[x]` done
Aturan: satu task = satu sesi fokus, harus bisa diverifikasi dengan perintah di bawahnya.

> Status ringkas (verified on the host + in the container, 2026-11-10):
> Phase 0-6 done (metrics measured). Phase 7: README + report done, video todo.
>
> | Item | State | Evidence |
> |---|---|---|
> | Robot + world + sensors | done | 4 packages build clean, 46 pytest pass |
> | EKF `odom->base_link` | done | static RMSE 0.016 m; dynamic 0.359 -> 0.196 m |
> | Map + AMCL | done | static 220x220 map, AMCL initial pose (-4,-4,0) |
> | A* (Manhattan, 4-way) | done | 200x200 = 32 ms, optimal vs Dijkstra in tests |
> | Path densify | done | fixes Nav2 window truncation false "goal reached" |
> | Nav2 controller (FollowPath) | done | robot drives the A* path, 0 controller errors |
> | Avoidance reflex | done | fires: BLOCKED -> 1 s hold -> mark -> replan |
> | Static run | PASS | goal (4,4), 15.89 m / 16.5 m optimal, min clearance 0.10 m |
> | Dynamic run | PASS | 1x hold+mark+replan, goal reached, no contact |
> | Unreachable goal | PASS | "A* found no path" -> empty path + controller cancel, robot stops |
> | Dead-end run | PASS | goal (-1.7,4.6), 18.59 m / 19.45 m optimal |
> | Metrics table | done | `scripts/log_run.py` + `scripts/eval_metrics.py` |
> | README + report | done | README Results/Parameters, `docs/report.md` |
> | Video demo | done | `docs/video/demo.mp4` (4:50), 2x hold+mark+replan |
>
> Honest results, not invented: raw wheel odom is near-exact on a clean run, so
> EKF does not lower static RMSE (0.016 vs 0.016); it cuts the dynamic-run slip
> (0.359 -> 0.196 m). Each scenario is one measured run, not 10/10 repeats.

## Phase 0 — Environment

- [x] **T0.1 Akses X11 ke container**
  - `sudo pacman -S --needed xorg-xhost && xhost +local:docker`
  - Acceptance: `docker run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros:jazzy-ros-base bash -c "apt update -qq && apt install -y -qq x11-utils >/dev/null && xdpyinfo | head -3"` menampilkan info display
  - Scope: S

- [x] **T0.2 Pull image + container tetap**
  - `docker pull ros:jazzy-ros-base`
  - Jalankan dengan `--network host --ipc host`, mount `/tmp/.X11-unix` + `~/Projects/Project-3:/work`
  - Acceptance: `ros2 topic list` jalan di dalam container; image < 2 GB
  - Scope: S

- [x] **T0.3 Install paket ROS minimal**
  - `xacro, robot-state-publisher, joint-state-publisher, rviz2, nav2-bringup, slam-toolbox, robot-localization, ros-gz, teleop-twist-keyboard, tf2-tools, nav2-simple-commander, python3-numpy, python3-scipy, mesa-utils`
  - Acceptance: `ros2 pkg list | grep -E 'nav2_bringup|slam_toolbox|robot_localization|ros_gz_sim'` semua muncul
  - Scope: S

- [x] **T0.4 Verifikasi RViz2 tampil di layar**
  - `rviz2` dari container
  - Acceptance: window RViz terbuka di Hyprland, tidak ada error GLX
  - Scope: S

- [x] **T0.5 Repo git + gitignore**
  - `.gitignore`: `build/ install/ log/ *.bag *.db3 *.mcap`
  - Acceptance: `git status` bersih setelah `colcon build`
  - Scope: XS

- [ ] **(Opsional, hanya bila FPS RViz < 10) T0.6 Akselerasi iGPU AMD**
  - Tambah `--device /dev/dri --group-add video --group-add $(getent group render | cut -d: -f3)`, hapus `LIBGL_ALWAYS_SOFTWARE`
  - Acceptance: `glxinfo | grep "OpenGL renderer"` menampilkan `AMD RENOIR`
  - **Jangan** pasang `nvidia-container-toolkit`. Ditunda sampai benar-benar perlu.
  - Scope: S

## Phase 1 — Simulasi & robot

- [x] **T1.1 World maze 10×10 m**
  - Dinding perimeter + 3 obstacle statis + 1 kotak bergerak (trajectory scripted di SDF)
  - Acceptance: `gz sim -s -r worlds/maze.sdf` jalan tanpa error; `/world/maze/dynamic_pose/info` terbit
  - Scope: M

- [x] **T1.2 URDF/xacro diff-drive**
  - Chassis 0.25×0.20, wheel r=0.033, separation=0.16, caster, `base_footprint`
  - Acceptance: TF `base_link→{laser_link,imu_link,wheel_*}` benar (`view_frames`)
  - Scope: M

- [x] **T1.3 Sensor plugin**
  - LiDAR 360 sampel / 10 Hz / 0.12–3.5 m; IMU 50 Hz; diff-drive plugin (odom + TF)
  - Acceptance: `ros2 topic hz /scan` ≈ 10, `/imu/data` ≈ 50; `/wheel/odom` terbit saat robot bergerak
  - Scope: M

- [x] **T1.4 Teleop & spawn**
  - `teleop_twist_keyboard` → `/cmd_vel`
  - Acceptance: robot bergerak di Gazebo, `/wheel/odom` berubah, tidak ada TF putus
  - Scope: S

## Phase 2 — Mapping & EKF (Deliverable 1)

- [x] **T2.1 `ekf.yaml`**
  - `two_d_mode: true`, `odom0: /wheel/odom`, `imu0: /imu/data`, `frequency: 30`, `publish_tf: true`, `world_frame: odom`
  - Acceptance: `/odometry/filtered` ≥ 30 Hz, `odom→base_link` hanya dari EKF
  - Scope: S

- [x] **T2.2 slam_toolbox async**
  - `map→odom` dari SLAM; `use_sim_time: true`
  - Acceptance: `/map` terbentuk setelah robot keliling; tidak ada publisher `map→odom` lain
  - Scope: M

- [x] **T2.3 Evaluasi drift**
  - Bandingkan RMSE odom mentah vs `/odometry/filtered` terhadap ground truth
  - Acceptance: tabel RMSE tersimpan di `docs/`; EKF lebih baik ≥ 30%
  - Scope: S

- [x] **T2.4 Simpan peta**
  - `ros2 run nav2_map_server map_saver_cli -f maps/maze`
  - Acceptance: `maze.pgm` + `maze.yaml` valid, bisa di-load `map_server`
  - Scope: XS

## Checkpoint A

- [ ] `ros2 run tf2_tools view_frames` → tree utuh tanpa frame dobel
- [ ] Peta tersimpan dan bisa di-load ulang
- [ ] EKF terbukti menurunkan RMSE

## Phase 3 — A* (Deliverable 2)

- [x] **T3.1 `grid_utils.py`**
  - Konversi OccupancyGrid ↔ numpy; `>=65` blocked, `-1` blocked, `0` free; world↔grid
  - Acceptance: unit test 3 kasus (free, occupied, unknown) lulus
  - Scope: S

- [x] **T3.2 A* 4-arah + Manhattan**
  - `heapq`, closed-set numpy, rekonstruksi path, deteksi no-path
  - Acceptance: path tidak menembus sel blocked; biaya ≤ Dijkstra pada 20 peta uji
  - Scope: M

- [x] **T3.3 Publish path + marker**
  - `nav_msgs/Path` ke `/plan`, Marker untuk RViz
  - Acceptance: jalur terlihat di RViz di atas `/map`
  - Scope: S

- [x] **T3.4 Overlay obstacle dinamis**
  - Dict sel→timestamp, diterapkan sebelum planning, expired N detik
  - Acceptance: obstacle yang ditandai membuat path berubah; setelah expired path kembali
  - Scope: S

- [x] **T3.5 Optimasi + benchmark**
  - Weighted A* (`h×1.2`), smoothing line-of-sight opsional
  - Acceptance: 200×200 < 100 ms; 400×400 < 600 ms; grafik waktu vs ukuran peta
  - Scope: M

## Phase 4 — Costmap & controller

- [x] **T4.1 Nav2 params (costmap + controller)**
  - `obstacle_layer` (scan), `inflation_layer` (radius ≥ r_robot + margin), `static_layer` (`/map`)
  - Acceptance: local & global costmap tampil di RViz, clearance ≥ radius robot
  - Scope: M

- [x] **T4.2 Remap `/cmd_vel_nav`**
  - controller_server → `/cmd_vel_nav`; belum ada publisher `/cmd_vel` selain avoidance
  - Acceptance: `ros2 topic info /cmd_vel` menunjukkan satu publisher setelah avoidance aktif
  - Scope: S

- [x] **T4.3 Kirim `FollowPath` action**
  - A* node kirim path ke `/follow_path` setelah planning
  - Acceptance: robot mengikuti path A*, bukan path planner Nav2
  - Scope: M

- [x] **T4.4 Uji maze statis**
  - Acceptance: 10/10 run sampai goal, deviasi lateral < 1 sel, 0 tabrakan
  - Scope: M

## Checkpoint B

- [ ] Maze statis lulus 10/10
- [ ] Bag + screenshot tersimpan

## Phase 5 — Dynamic avoidance (Deliverable 3)

- [x] **T5.1 `avoidance_node.py` — proximity + tanh**
  - Subscribe `/local_costmap/costmap`, jarak terdekat di koridor depan, `v = v_max·tanh(k·error)`
  - Acceptance: kecepatan turun monoton terhadap jarak; hard stop di `d_min`
  - Scope: M

- [x] **T5.2 Steering menjauh + watchdog**
  - Vektor repulsif opsional; timeout `/cmd_vel_nav` 0.5 s → nol
  - Acceptance: robot tidak hanya berhenti; cmd_vel jadi nol saat controller mati
  - Scope: M

- [x] **T5.3 Replan policy (tunggu 1 s → replan)**
  - Obstacle menetap → tandai sel → A* replan → `FollowPath` baru
  - Acceptance: path berubah ≤ 0.5 s setelah timer 1 s habis (total ≤ 1.5 s); tidak ada loop replan
  - Scope: M

- [~] **T5.4 Uji obstacle dinamis**
  - Acceptance: 10/10 bebas tabrakan; osilasi < 2 zero-crossing per approach
  - Hasil: 1 run terukur (BLOCKED -> 1 s hold -> mark 25 sel -> replan, goal
    tercapai, d=0.23 m saat hold > radius robot 0.17 m -> tanpa kontak).
    10/10 belum dijalankan.
  - Scope: M

## Phase 6 — Evaluasi

- [x] **T6.1 Skenario 4 & 5**
  - Dead-end dan goal unreachable harus fail gracefully (berhenti + status, bukan crash/loop)
  - Hasil: dead-end PASS (18.59 m vs 19.45 m optimal); unreachable -> "A* found
    no path" -> path kosong + cancel FollowPath -> robot berhenti di (1.095,-4.273).
  - Scope: S

- [x] **T6.2 Tabel metrik lengkap**
  - Success, waktu, panjang jalur vs optimal, node diekspansi, min clearance, max decel, RMSE, CPU/RAM
  - Acceptance: semua angka berasal dari bag, bukan estimasi
  - Hasil: tabel 3 skenario terukur di README `## Results` + `docs/report.md`;
    sumber `scripts/log_run.py` + `scripts/eval_metrics.py`. CPU/RAM belum diukur.
  - Scope: M

## Phase 7 — Delivery

- [x] **T7.1 README + dokumentasi parameter**
  - Instalasi, cara run, tabel parameter, hasil metrik
  - Hasil: README `## Results` + `## Parameters`.
  - Scope: M

- [x] **T7.2 Laporan**
  - Arsitektur, TF tree, penjelasan `f=g+h`, tabel RMSE, grafik `v` vs jarak, tabel 5 skenario
  - Hasil: `docs/report.md` (arsitektur, TF, EKF, A* `f=g+h`, RMSE, 5 skenario,
    limitasi). Grafik `v` vs jarak: lihat `log/run_*.csv` kolom `obs_d`/`out_v`.
  - Scope: M

- [x] **T7.3 Video demo 3–5 menit**
  - Hasil: `docs/video/demo.mp4` (4:50, 1280×720). Direkam dari stack headless
    (Gazebo + RViz pada display X terisolasi di dalam container), jadi tidak ada
    desktop yang terekam. Satu misi goal (4,4); box dinamis menyeberang koridor,
    dua event `BLOCKED -> 1 s hold -> mark -> replan`, selesai `Reached the goal!`.
    Screenshot: `docs/video/shot-*.png`.
  - Scope: M

- [x] **T7.4 (Opsional) Dockerfile + compose reproducible**
  - Hasil: `docker/Dockerfile` + `docker/compose.yaml`, `scripts/run.sh`.
  - Scope: S
