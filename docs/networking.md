# Networking — Tailscale + CycloneDDS

How ROS 2 nodes communicate across machines (host PC ↔ Jetsons) over Tailscale VPN.

---

## Why Tailscale?

The Jetsons and the host PC live on different networks (venue WiFi, hotspot, lab LAN). Tailscale creates a persistent VPN mesh so every machine has a stable IP — no need to reconfigure for each deployment location.

---

## The DDS multicast problem

ROS 2's default DDS (Fast-DDS) uses UDP multicast for discovery. Tailscale is a **unicast-only** VPN — multicast packets are silently dropped. Without configuration, ros2 topic list on Machine A sees nothing from Machine B even if ping works.

**Fix**: Switch to CycloneDDS with explicit unicast peer IPs.

---

## CycloneDDS configuration

File: `ros2/src/mfe_bringup/config/cyclone_tailscale.xml`

```xml
<CycloneDDS>
  <Domain>
    <General>
      <!-- Bind DDS to the Tailscale virtual interface only -->
      <NetworkInterfaceAddress>tailscale0</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <!-- Host PC -->
        <Peer Address="100.98.63.80"/>
        <!-- Jetson 1 (perception) -->
        <Peer Address="100.84.7.80"/>
        <!-- Jetson 2 (compute) -->
        <Peer Address="100.119.35.6"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

**To add a new machine**: get its Tailscale IP (`tailscale ip -4`) and add a `<Peer>` entry. Rebuild is not required — the file is read at node startup.

---

## Required environment variables

Every machine (host + all Jetsons) must have these set before launching ROS 2:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclone_tailscale.xml
```

`setup_jetson.sh` appends these to `~/.bashrc` automatically. For the host PC, add them to your shell profile or to `docker_run.sh`.

The XML file is copied to `~/cyclone_tailscale.xml` by `setup_jetson.sh`. It is also available at the repo path above for updating peer IPs.

---

## Tailscale IP addresses

| Machine | Hostname | Tailscale IP | Role |
|---------|----------|-------------|------|
| Host PC | — | 100.98.63.80 | Sim + perception (dev mode) |
| Jetson 1 | mfe-driverless-1 | 100.84.7.80 | Perception (real car) |
| Jetson 2 | mfe-driverless-2 | 100.119.35.6 | Compute |

> **Note**: The Jetsons are in the `McGillFormulaEV@` Tailscale organisation. The host PC is in `neilgeorge03@`. MagicDNS short names (e.g. `mfe-driverless-2`) do **not** resolve cross-org — use IP addresses directly.

---

## Verifying connectivity

```bash
# Ping Tailscale peer
ping 100.119.35.6

# Check ROS 2 sees the remote machine's topics
ros2 topic list   # should show topics from all peers after ~10 s

# SSH to Jetson
ssh mfe@100.119.35.6 echo "connection OK"
```

If `ros2 topic list` is empty on a peer:
1. Check Tailscale is connected on both machines (`tailscale status`).
2. Verify `CYCLONEDDS_URI` points to the XML file and that file has the peer's IP.
3. Confirm `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` match on all machines.
4. Firewall: CycloneDDS uses UDP 7400–7500 for discovery + data. Allow these on the Tailscale interface.

---

## Bandwidth considerations

Raw sensor data is large. On a bandwidth-limited Tailscale link:

| Topic | Approx bandwidth |
|-------|-----------------|
| `/lidar/points_raw` (PointCloud2, VLP-16) | ~8 Mbps |
| `/camera/image_raw` (1080p, 30 fps) | ~600 Mbps uncompressed |
| `/planning/cones` (processed cones) | < 0.1 Mbps |
| `/cmd` (AckermannDrive) | < 0.01 Mbps |

**Keep raw data local**. In the two-machine architecture:
- Perception Jetson (or host PC in sim): runs all sensor processing, publishes only `/planning/cones` and `/ekf/output` across the network.
- Compute Jetson: subscribes to processed topics only.

This is enforced via `run_perception:=true/false` and `run_compute:=true/false` in `bringup.launch.py`.
