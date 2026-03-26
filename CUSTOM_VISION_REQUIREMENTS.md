# Custom Vision System Requirements (2026 Season)
**Target: NetworkTables (NT4) Implementation for PhotonVision Replacement**

To successfully replace PhotonVision, your custom system (running on a Pi/Jetson) must publish the following data to NetworkTables at a rate of **30-60 FPS**.

---

## 1. Spatial Data (3D Geometry)
These values define where the target is located in space relative to the camera lens.
*   **`targetX` (double)**: Distance forward/backward from lens (**Inches**).
*   **`targetY` (double)**: Distance left/right from lens (**Inches**).
*   **`targetZ` (double)**: Distance up/down from lens (**Inches**).
*   **`targetYaw` (double)**: Horizontal rotation of the target (Radians).
*   **`targetPitch` (double)**: Vertical rotation of the target (Radians).
*   **`targetRoll` (double)**: Tilt rotation of the target (Radians).

> **Note:** The Java `VisionIO` implementation must convert these to **Meters** using `Units.inchesToMeters()` before logging to AdvantageKit.

## 2. Identification & Quality
Used by the robot to decide if it should trust the data.
*   **`targetId` (int)**: The Fiducial ID of the AprilTag (e.g., 1-16).
*   **`hasTarget` (boolean)**: True if at least one valid target is in view.
*   **`poseAmbiguity` (double)**: A score from 0.0 to 1.0. 
    *   *Low (0.1):* Very stable, high confidence.
    *   *High (0.8):* Unstable, likely a "ghost" or "flicker" image.
*   **`targetArea` (double)**: Percentage of the screen the target occupies (0-100).

## 3. Critical Timing (Lag Compensation)
The robot moves fast; it needs to know exactly **when** the picture was taken to calculate its position correctly.
*   **`publishTimestamp` (double)**: The FPGA timestamp (Microseconds) from the RIO when the data was sent.
*   **`processingLatency` (double)**: How many milliseconds it took the Pi to process the frame (ms).
*   **`totalLatency` (double)**: `processingLatency` + `networkLatency`.

## 4. NetworkTables Structure
Your Pi code should write to a dedicated table so the robot can find it easily.
*   **Table Name:** `CustomVision`
*   **Keys:**
    *   `/CustomVision/targetPose` (Double Array: [x, y, z, roll, pitch, yaw])
    *   `/CustomVision/targetId`
    *   `/CustomVision/latency`
    *   `/CustomVision/heartbeat` (An incrementing integer to prove the Pi hasn't crashed)

---

## 5. AdvantageKit `VisionIO` Integration
Your Java `Inputs` class must be structured like this to maintain your current project standards:

```java
@AutoLog
public class CustomVisionIOInputs {
    public boolean hasTarget = false;
    public int targetId = -1;
    public Pose3d cameraToTarget = new Pose3d(); // Unified 3D Pose
    public double latencyMillis = 0.0;
    public double timestampMicros = 0.0;
    public double ambiguity = 0.0;
}
```

## 6. Hardware & Network Setup
*   **Static IP:** The Pi must be set to `10.2.50.X` (matches your `Electrical.md`).
*   **Resolution:** 640x480 or 1280x720.
*   **Field Map:** Your Pi must have the `2026-field.json` file loaded to know where the AprilTags are located on the floor.

## 7. Mathematical Standard
*   **Coordinate System:** EDN (X: Forward, Y: Left, Z: Up).
*   **Angle Units:** Radians (Standard for WPILib geometry).
*   **Distance Units:** Meters.
