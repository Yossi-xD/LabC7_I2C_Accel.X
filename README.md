# Smart Watch – User Guide

A PIC24FJ256GA705-based smart watch with a 96×96 colour OLED display, ADXL345 accelerometer, and potentiometer input.

---

## Controls

| Control | Action |
|---|---|
| **S1** (short press) | Move menu cursor down |
| **S2** (short press) | Confirm selection / advance to next field |
| **S1** (long press, 2 sec) | Open the menu |
| **Potentiometer** | Change value when editing a field |
| **Shake device** | Dismiss a ringing alarm / Exit Menu |
| **Flip device face-down** | Dismiss a ringing alarm / Exit Menu |

Button presses provide visual feedback using LEDs.

---

## Clock Faces

The watch has two display modes switchable from the menu.

The display always shows:
- Current time (HH:MM:SS)
- Current date (day/month)
- Alarm status indicator (icon)

---

### Digital
- Shows time in large text (HH:MM:SS)
- AM/PM label shown in 12H format
- Date (day/month) and alarm icon displayed in the corners
- Only the time digits refresh every second; the rest redraws only when changed

---

### Analog
- Classic clock face with hour, minute, and second hands
- Hour hand is shorter and thicker than the others
- AM/PM label shown in the bottom corner in 12H format
- Small alarm icon in the top-left corner when alarm is enabled

#### Hand Movement
- Second hand updates every second (6° step)
- Minute hand updates every 60 seconds
- Hour hand moves continuously according to time

---

## Time and Date Behavior

- The system updates the time once per second using a hardware timer (ISR)
- The date automatically updates at midnight (00:00:00)
- Month transitions are handled correctly (including February with 28 days)

---

## Menu

Hold **S1 for 2 seconds** from the clock screen to open the menu.  
Press **S1** to scroll through items and **S2** to select.  
Shake or flip the device to exit the menu.

While inside the menu:
- The current time continues updating in real-time
- A small digital clock is displayed on screen

---

### Menu Options

| Item | What it does |
|---|---|
| **Display Mode** | Switch between Digital and Analog |
| **12H/24H Format** | Toggle 12-hour (AM/PM) and 24-hour time display |
| **Set Time** | Set hours, minutes, and seconds one field at a time |
| **Set Date** | Set day (1–30) and month (1–12) |
| **Set Alarm** | Set alarm hour and minute; enables the alarm automatically |
| **Alarm Off** | Immediately disables the alarm |

---

### Editing a field (Set Time / Set Date / Set Alarm)

1. The active field is highlighted (white box, black text)
2. Turn the **potentiometer** to change the value
3. Press **S2** to confirm and move to the next field
4. After the last field, the setting is saved and you return to the main menu

Arrows on the left side of the screen indicate the active field.

---

## Alarm

- Set an alarm time via **Set Alarm** in the menu
- When the alarm triggers:
  - The display flashes (visual / inverse effect)
  - LEDs blink to attract attention

### Dismissing the Alarm
- Press **S2**
- Shake the device
- Flip the device face-down

### Auto-stop
- The alarm automatically stops after 20 seconds if not dismissed

### Important
- Any dismissal method fully disables the alarm
- To re-enable → use **Set Alarm**
- You can also disable the alarm manually using **Alarm Off** in the menu
