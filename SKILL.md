---
name: smart-wheelchair-bap
description: Context and engineering reference for a Bachelor End Project (BAP) building an autonomous smart wheelchair demo for a university open day. The demo targets high school visitors and the general public, running in a pre-mapped indoor space. The primary user input modality is Voice Control (Acoustic Keyword Spotting) with touchscreen/tactile fallbacks. Use this skill whenever the user discusses the BAP, the voice control subsystem, noise reduction strategies, the offline Vosk engine, integration with the pathfinding team, ROS2 bridge, the GUI, or the touchscreen/button fallbacks.
---

# Smart Wheelchair BAP — Voice Control & User Input Subsystem

This skill captures the full engineering context for a Bachelor End Project focused on upgrading a smart wheelchair to autonomous destination-based navigation. The user owns the **User Input Subsystem** (Voice Control + Fallbacks) of a larger team project.

The target audience is high school students and the general public visiting a university open day. The system must be robust to noisy environments, easy to set up, hygienic, and scalable so that it can be further trained or upgraded in the future.

When this skill is active, assume the user is the student engineer and respond with the specificity of someone who has read the project reports and design decisions. The user is located in Delft, NL. Prices are in euros. The user codes in Cursor.

---

## 1. Project Goal

### 1.1 Technical Goal
Upgrade the wheelchair to **autonomous pathfinding**, where the user speaks a destination keyword, the system confirms intent, and the chair drives itself there autonomously.

### 1.2 Demo Context
The project must produce a **university open day demo** in a pre-mapped, known indoor space.
- **User-Directed Mode**: A visitor sits in the chair, clips on a microphone, and speaks a destination. The chair drives there autonomously.
- **Constraints**: Must work reliably in a crowded, noisy environment (cocktail party effect) for naive users with zero prior training. Setup time per visitor must be ≤ 3 minutes.
- **Future-Proofing**: The architecture must be modular so that it can be further trained with new machine learning acoustic models, or upgraded to advanced LLM intent-parsing in the future without rebuilding the core integration.

### 1.3 Selected Input System
The student is responsible for the User Input Subsystem, which consists of:
1. **Primary Interface:** Voice Control (Acoustic Keyword Spotting) using offline Vosk ASR, wake-word gated, with Groq LLM intent classification.
2. **Deterministic Fallback:** Mouse/Cursor click on a digital map displayed on the host laptop screen. **(GUI map overlay is built; click-to-navigate is not yet wired up — see Section 6.)**
3. **Ultimate Safety Override:** Emergency Stop button on the GUI + physical keyboard.

Mode switching is handled by a Mode Arbiter to ensure mutual exclusion before passing commands to the pathfinding team.

---

## 2. Hardware Stack

### 2.1 Voice Hardware — **DECIDED**
The Plantronics Voyager 5200 UC headset with the BT300M USB Bluetooth adapter has been selected.

| Component | Chosen Part | Role |
|---|---|---|
| **Wireless Transmitter/Receiver** | Plantronics BT300M USB adapter | Plug-and-play, bypasses unstable Bluetooth stacks. Appears as device index 24 on WASAPI (Windows). |
| **Directional Microphone** | Plantronics Voyager 5200 UC | 6-microphone array with hardware Acoustic Fence DSP — physically rejects off-axis crowd noise. |

The software noise gate (`NoiseGate` class in `voice_control.py`) is present but disabled by default because the Plantronics hardware DSP already does a better job. Only enable the gate if you switch to a plain omnidirectional mic.

*WASAPI is used as the audio API (lowest latency on Windows). Device index 24 is hardcoded in `VoiceConfig.input_device`.*

### 2.2 Fallback & Integration Hardware
| Component | Role |
|---|---|
| Laptop Mouse/Cursor | Visual map selection if voice fails (click-to-navigate not yet implemented) |
| GUI E-Stop Button | Emergency stop with toggle (release) — wired to ROS2 |
| Laptop Keyboard | F11 for fullscreen, ESC to exit or dismiss map |
| Cables & Mounts | USB hubs, cable management, mounting brackets |

---

## 3. Code Architecture & Files

All source files live in the project root (`C:\BAP V2\`).

### 3.1 File Map

| File | Status | Purpose |
|---|---|---|
| `main.py` | Done | Entry point — wires GUI + Voice + ROS2 together |
| `voice_control.py` | Done | Full voice pipeline (Vosk, wake gate, FastNavigator, LLM, TTS, noise gate, gender detection) |
| `jarvis_gui.py` | Done | PyQt6 full-screen Jarvis GUI |
| `ros2_bridge.py` | Done | ROS2 integration (native rclpy + rosbridge WebSocket) |
| `gender_detector.py` | Done | Pitch-based F0 autocorrelation gender classifier |
| `requirements.txt` | Done | Python dependencies |
| `models/vosk-model-en-us-0.22/` | Done | Vosk ASR model, downloaded and in place |
| `assets/eemcs_map.png` | Done | EEMCS walking-routes map for the GUI overlay |
| `green_light_assessment.tex` | Done | Green light milestone report |
| `.env` | Done | Holds `GROQ_API_KEY=gsk_...` |
| `Jarvis-Windows/` | Archive | Earlier prototype using Whisper STT + Kokoro TTS. Not used in the current system. |

### 3.2 Keyword Spotting (KWS) Pipeline

1. Audio captured from Plantronics BT300M (device 24, WASAPI) via `sounddevice.RawInputStream`, 16 kHz, 125 ms chunks.
2. Optional software `NoiseGate` filters chunks below an RMS energy threshold (disabled by default — see Section 2.1).
3. Every chunk is fed in parallel to `GenderDetector` (pitch analysis) to update the sir/madame address.
4. Chunks are fed to `VoskEngine` (offline, `vosk-model-en-us-0.22`). Partial results are displayed in the GUI in real time.
5. Final results are filtered against `_STT_NOISE_WORDS` (single common words like "the", "a") and a per-word confidence gate (`_STT_MIN_CONF = 0.45`).
6. `WakeWordGate` checks for "Jarvis". Once awake, the gate stays open for 30 seconds per utterance, refreshed on each new command.
7. On wake, if there is no trailing command in the same utterance, Jarvis says "Yes, sir?" and waits. If there is a trailing command it is processed immediately.
8. Every command goes to `FastNavigator` first (zero-LLM path): if it matches a motion verb + known destination, the chair is asked for confirmation and moves. No Groq call needed.
9. If `FastNavigator` does not match, the command goes to `IntentClassifier` (Groq `llama-3.1-8b-instant`, ~200 ms round trip) which classifies into one of six intents: `navigate`, `show_map`, `hide_map`, `question`, `chatter`, `goodbye`.
10. For `navigate`, `VoiceController._voice_confirm()` asks "Shall I take you to X, sir?" and listens for a spoken yes/no (8 s timeout, safe default: cancel).
11. On confirmation, `PayloadEmitter` builds the JSON payload and fires it via `VoiceObserver.on_navigate()` → `QtVoiceObserver` Qt signal → `ros2.publish_nav_goal()`.

### 3.3 Interface to Pathfinding Team — **DECIDED**

Communication is over ROS2. Two transport options are available via `create_bridge()` in `ros2_bridge.py`:

| Transport | When to use |
|---|---|
| `Ros2NativeNode` | GUI and robot on the same machine (development) |
| `Ros2WebsocketBridge` | GUI laptop and wheelchair PC are separate (demo, connects over Wi-Fi to rosbridge port 9090) |

Set `ROS2_TRANSPORT=native` or `ROS2_TRANSPORT=websocket` in the environment before launching. The WebSocket bridge auto-reconnects on drop.

**ROS2 Topics**

| Direction | Topic | Type | Content |
|---|---|---|---|
| Publish | `/wheelchair/nav_goal` | `std_msgs/String` | JSON nav goal payload |
| Publish | `/wheelchair/estop` | `std_msgs/Bool` | True = stop, False = release |
| Subscribe | `/wheelchair/status` | `std_msgs/String` | JSON status from pathfinding team |

**Navigation goal payload**
```json
{
  "mode":        "voice",
  "destination_id": "LOC_LAB_A",
  "destination_phrase": "lab a",
  "confidence":  0.980,
  "confirmed":   true,
  "timestamp":   1716200000.0
}
```

**Status payload (from pathfinding team)**
```json
{
  "state":       "NAVIGATING",
  "destination": "lab_a",
  "progress":    0.42,
  "message":     ""
}
```
States are `IDLE`, `NAVIGATING`, `ARRIVED`, `OBSTACLE`, `ESTOP`, `ERROR`. The GUI shows them colour-coded below the voice state label.

---

## 4. Subsystem Requirements (from PoR)

- **REQ-UI-FR01**: Robustly interpret user intent and map to a predefined spatial destination.
- **REQ-UI-FR03**: Setup/calibration ≤ 3.0 minutes per naive user. (Clipping on the mic takes < 30 seconds).
- **REQ-UI-FR04**: Deterministic Mode Arbiter to prevent simultaneous inputs (e.g., clicking the map while speaking).
- **REQ-UI-FR05**: Provide immediate unambiguous feedback with a 2-second window to cancel the action. *(Implemented as a spoken voice-confirmation loop, not a countdown timer — more natural for naive users.)*
- **REQ-UI-FR06**: Completely local/offline operation (absolute data privacy, no cloud APIs). *(Vosk, GenderDetector, and GUI are offline. Groq is cloud but optional — Ollama local model is supported as a drop-in by setting `llm_provider="ollama"`.)*
- **REQ-UI-TR02**: End-to-end processing latency ≤ 1.0 seconds. *(FastNavigator path is ~0 ms + TTS. LLM path is ~200–500 ms Groq round trip.)*
- **REQ-UI-TR03**: Intent recognition accuracy ≥ 95% in standard indoor demonstration noise.

---

## 5. GUI — Jarvis Console

`jarvis_gui.py` is a PyQt6 full-screen GUI themed after the Iron Man / Cortana orb aesthetic. It is I/O-free: all state is driven via Qt signals from the voice subsystem running in a daemon thread.

Key elements:
- **JarvisOrb** — animated glowing ring with rotating arcs. Speed and brightness react to `State` (IDLE / LISTENING / AWAITING COMMAND / THINKING / SPEAKING).
- **AddressChip** — top-right chip showing "SIR" (blue) or "MADAME" (pink), updated in real time by the gender detector.
- **Ros2StatusChip** — top-right connection indicator (green = LIVE, red = OFFLINE).
- **Robot state label** — coloured text below the voice state label showing NAVIGATING / ARRIVED / OBSTACLE / ESTOP / ERROR from the pathfinding team.
- **Live transcript** — partial (live) and final (confirmed) Vosk output shown in the centre.
- **Reply label** — Jarvis's spoken replies also shown on screen.
- **MapOverlay** — fade-in full-screen panel displaying `assets/eemcs_map.png`. Triggered by voice ("show me the map") or dismissed by voice ("close the map") or ESC key.
- **Emergency Stop button** — large red button at the bottom. Toggles between EMERGENCY STOP (activate) and RELEASE E-STOP, publishing to `/wheelchair/estop`.

Run `python jarvis_gui.py` directly to preview the GUI with a scripted demo loop (no mic required).

---

## 6. What Is Not Yet Built

The following items are identified but not yet implemented:

**Map click-to-navigate** — `MapOverlay` displays the EEMCS map and fades in/out correctly. Clicking a room on the map is *not yet wired* to send a navigation command. The `map_dismissed` signal from `MapOverlay` fires but no click handler on the map image converts pixel coordinates to a destination ID. This is the main remaining piece of the fallback input subsystem.

**Grammar-locked Vosk** — `VoiceConfig.grammar_locked` exists and passes a constrained word list to `KaldiRecognizer`. It is off by default because the full model handles natural speech better for the LLM question/chatter paths. May be worth testing for the FastNavigator-only mode.

**Ollama offline LLM** — code is fully in place (`_init_ollama`, `_call_ollama`). Set `llm_provider="ollama"` and `ollama_model="llama3.2:3b"` in `VoiceConfig` to run fully offline. Not tested end-to-end against the demo script yet.

---

## 7. Running the System

```
# Install dependencies (once):
pip install -r requirements.txt

# Add Groq key to .env:
echo GROQ_API_KEY=gsk_... > .env

# Native ROS2 (same machine as robot, ROS2 Jazzy sourced):
source /opt/ros/jazzy/setup.bash
python main.py

# WebSocket (robot on a separate PC at 192.168.1.100):
set ROSBRIDGE_HOST=192.168.1.100
set ROS2_TRANSPORT=websocket
python main.py

# GUI preview only (no mic, no ROS2):
python jarvis_gui.py

# Voice only (CLI, no GUI):
python voice_control.py
```

Press F11 for fullscreen, ESC to dismiss the map or quit.

---

## 8. Cursor Workflow for this Project
The user codes in Cursor. Best practices when providing code:
- Give complete, self-contained functions the user can paste directly.
- Focus on modular, class-based Python architecture so the Voice module can be easily swapped or extended later (e.g., upgrading from Vosk to a local LLM in the future).
- Ensure all serial/socket logic includes robust error handling and reconnect loops.
- Match the docstring and comment style of the existing files (module-level triple-quote docstrings with dashed underlines, inline comments explaining *why* not *what*).
