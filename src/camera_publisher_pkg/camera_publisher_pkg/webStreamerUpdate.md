# 🚀 Hybrid Dual Web Streamer for ROS2

### A lightweight, full-featured web dashboard that merges real-time video, AI detections, and system telemetry — built for Raspberry Pi and ROS2-powered rovers.

---

## 📘 Overview

This project **combines the best of two prior implementations**:

| Feature | Document 1 (Original Advanced) | Document 2 (Optimized) | Merged Hybrid |
|----------|--------------------------------|--------------------------|----------------|
| Real-time Logs (Socket.IO) | ✅ Yes | ❌ No | ✅ Yes |
| Camera + AI Detection Streams | ✅ Yes | ✅ Yes | ✅ Yes |
| Cache & CORS Headers | ❌ No | ✅ Yes | ✅ Yes |
| JavaScript Error Handling | ❌ Basic | ✅ Enhanced | ✅ Enhanced |
| Lightweight Design | ❌ Heavy | ✅ Light | ⚖️ Balanced |
| Health Endpoint | ❌ No | ✅ Yes | ✅ Yes |
| Deployment Simplicity | ⚠️ Moderate | ✅ Simple | ✅ Simple |
| Debug Console Logging | ❌ Minimal | ✅ Verbose | ✅ Verbose |

The merged script (`dual_web_streamer_merged.py`) delivers:

- **Live dual camera streams** (raw + AI detection)  
- **Real-time system logs** via Socket.IO  
- **Periodic status updates** and a `/status` API  
- **Improved HTTP caching and CORS headers**  
- **JavaScript-based error handling and live UI updates**  
- **Simple one-command deployment** for Flask + Socket.IO

---

## 🧩 Architecture

```text
+-------------------+     +----------------------+
| ROS2 Topics       | --> |  DualWebStreamer Node |
|  /camera/image_raw/compressed         |
|  /yolo/annotated/compressed           |
|  /cmd_vel, /motor/*, /status ...      |
+-------------------+     +----------------------+
                                 |
                                 v
                        Flask + Socket.IO Server
                                 |
                                 v
                          🌐 Web Dashboard UI
