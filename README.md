# 🧭 TurtleBot4 Autonomous Navigation

### 🎥 Demo Video  
**Watch the demonstration here:**  
https://www.play.mdx.ac.uk/media/t/1_niml867m

---

## 📌 Overview  
This project demonstrates a TurtleBot4 robot navigating to multiple user-defined poses on a classroom map using Python and ROS2. The assessment builds upon the basic pass criteria and includes additional features for improved performance and usability.

---

## ✅ Basic Functionality

- The robot receives a user-specified destination and autonomously navigates to it.  
- Multiple destinations are supported.

---

## 🌟 Additional Feature

If a navigation attempt fails:

- The robot waits **10 seconds** for the user to input a new destination.  
- If no input is received within that time, the robot automatically returns to the predefined home location.

---

## 🗂️ Files and Descriptions

| File | Description |
|------|-------------|
| `NavigatorRobot.py` | A user-interactive Python script that lets the TurtleBot4 navigate to predefined goals and return home if a navigation attempt fails. |
| `navigate_with_fallback.py` | Navigation script with 10-second user prompt on failure; returns to home if no new input is received. |
| `navigation.yaml` | Parameters for TurtleBot4 path planning and navigation. |
| `localisation.yaml` | Parameters for AMCL-based localization on the map. |
| `Hatchcroft_Lab.pgm` & `Hatchcroft_Lab.yaml` | Map and associated YAML file used for localization and navigation during the demonstration. Describes resolution, origin, and occupancy thresholds for the Hatchcroft Lab layout. |

---
