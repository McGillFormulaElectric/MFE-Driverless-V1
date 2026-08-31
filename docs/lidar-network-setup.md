# LiDAR Network Setup & Troubleshooting

## LiDAR Network Config (from sensor web UI)

| Field       | Value               |
|-------------|---------------------|
| DHCP        | Off (static)        |
| IP Address  | `10.37.1.194`       |
| Subnet Mask | `255.255.255.192` (`/26`) |
| Gateway     | `10.37.1.193`       |
| MAC Address | `60-76-88-38-5e-fd` |

---

## Step 1: Check if LiDAR is physically connected

```bash
ip link show
```

Look for your Ethernet interface (e.g. `enP8p1s0`). It should show `UP,LOWER_UP` — not `NO-CARRIER` or `DOWN`.

```bash
ip addr show enP8p1s0
```

- `LOWER_UP` = physical cable detected (good)
- `NO-CARRIER` = nothing connected

---

## Step 2: Assign a static IP on the host machine

The host must be on the same `/26` subnet as the LiDAR (`10.37.1.192/26`).

### One-shot (not persistent across reboots)
```bash
sudo ip addr add 10.37.1.193/26 dev enP8p1s0
sudo ip link set enP8p1s0 up
```

### Persistent via NetworkManager (nmcli)
```bash
sudo nmcli con add type ethernet ifname enP8p1s0 con-name lidar-eth \
  ip4 10.37.1.193/26

sudo nmcli con up lidar-eth
```

> Note: `netplan` is not installed on this machine — use `nmcli` instead.

---

## Step 3: Verify LiDAR is reachable

```bash
ping 10.37.1.194
```

Once ping succeeds, the LiDAR driver should be able to connect.

---

## Troubleshooting: Disk Full (git fetch/pull fails)

If you see `fatal: write error: No space left on device`, the disk is full.

### Diagnose
```bash
df -h
du -sh /* 2>/dev/null | sort -rh | head -20
du -sh ~/Develop ~/.ros /var/log /tmp 2>/dev/null | sort -rh
```

### Quick Fixes

```bash
# ROS2 logs (common culprit)
rm -rf ~/.ros/log/*

# Docker cleanup
docker system prune -f

# APT cache
sudo apt clean

# ROS2 build artifacts
rm -rf ~/Develop/MFE-Driverless-V1/build/ ~/Develop/MFE-Driverless-V1/install/
```

After freeing space, retry `git pull`.

---

## Valid IP Range for This Subnet

| Address           | Role                |
|-------------------|---------------------|
| `10.37.1.192`     | Network address          |
| `10.37.1.193`     | Host (`mfe-driverless-2`) |
| `10.37.1.194`     | LiDAR sensor (VLP-16)    |
| `10.37.1.255`     | Broadcast                |

---

## ROS2 Remote Viewing over Tailscale VPN

### Domain IDs & Device Reference

| Device              | Tailscale IP    | ROS_DOMAIN_ID |
|---------------------|-----------------|---------------|
| `mfe-driverless-2`  | `100.119.35.6`  | `42`          |
| Neil's MacBook              | `100.101.148.114` | `42` |

> All devices must share the same `ROS_DOMAIN_ID` to discover each other.

### Required env vars (add to `~/.bashrc` on each machine)

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/cyclone_tailscale.xml
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

### CycloneDDS config (`~/cyclone_tailscale.xml`)

Update `<Peer Address>` entries to include all device Tailscale IPs:

```xml
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>tailscale0</NetworkInterfaceAddress>
    </General>
    <Discovery>
      <Peers>
        <Peer Address="100.119.35.6"/>
        <Peer Address="100.101.148.114"/>  <!-- Neil's MacBook -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

### Install CycloneDDS if missing

```bash
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
```

### Verify connection on remote device

```bash
ping 100.119.35.6
ros2 topic list                   # should show /velodyne_points
ros2 topic hz /velodyne_points    # check data is flowing
rviz2                             # visualise — add PointCloud2, topic: /velodyne_points, frame: velodyne
```
