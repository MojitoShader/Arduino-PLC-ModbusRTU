# Arduino Opta RS485 → MKS SERVO42D über Modbus RTU

Ansteuerung eines **MKS SERVO42D** (NEMA17 Closed-Loop-Schrittmotor, RS485-Version) mit einer **Arduino Opta RS485** (Finder) über **Modbus RTU**. In dieser ersten Version steuerst du den Motor über einen Text-Befehlssatz im seriellen Monitor (USB-C).

```
 PC (serieller Monitor)       Arduino Opta RS485                MKS SERVO42D
 ┌──────────────┐  USB-C   ┌──────────────────┐  RS485    ┌───────────────────┐
 │ > spd 120    │ ───────► │ Modbus-RTU-Master│ ────────► │ Modbus-Slave ID 1 │
 │ OK           │ ◄─────── │ (ArduinoModbus)  │ ◄──────── │ NEMA17 + Encoder  │
 └──────────────┘          └──────────────────┘ 38400 8N1 └───────────────────┘
```

---

## 1. Komponenten

### Arduino Opta RS485 (Arduino SKU AFX00001)

| Merkmal       | Wert                                                                 |
|---------------|----------------------------------------------------------------------|
| MCU           | STM32H747XI Dual-Core (Cortex-M7 480 MHz + Cortex-M4 240 MHz)        |
| Versorgung    | 12–24 V DC                                                           |
| Eingänge      | 8 × digital / analog 0–10 V                                          |
| Ausgänge      | 4 × Relais (Schließer), 10 A                                         |
| Schnittstellen| RS485 (Halbduplex), Ethernet, USB-C (Programmierung + seriell)       |
| Programmierung| Arduino-Framework (mbed-Core), hier über **PlatformIO** (`board = opta`) |
| RS485 intern  | `Serial2`, DE = PB_14, RE = PB_13 (bindet `ArduinoRS485` automatisch ein) |

Produktseite: <https://store.arduino.cc/products/opta-rs485>

### MKS SERVO42D (RS485-Version)

| Merkmal           | Wert                                                                |
|-------------------|---------------------------------------------------------------------|
| Motor             | NEMA17 (42 mm), Closed-Loop mit Magnet-Encoder                      |
| Versorgung        | 12–24 V DC (Netzteil mind. 24 V/1 A oder 12 V/2 A)                  |
| Arbeitsstrom      | 0–3000 mA (Default 1600 mA)                                         |
| Encoder-Auflösung | 16384 (0x4000) Einheiten pro Umdrehung                              |
| Mikroschritte     | 1–256 (Default 16 → 3200 Pulse pro Umdrehung)                       |
| Schnittstelle     | RS485: MKS-eigenes Protokoll **oder** Modbus RTU (Menü `Mb_RTU`)     |
| Default-Bus       | 38400 Baud, 8N1, Slave-Adresse 1                                    |

Arbeitsmodi (Menü `Mode`):

| Modus   | Steuerung              | max. Drehzahl | Bemerkung                     |
|---------|------------------------|---------------|-------------------------------|
| CR_OPEN / CR_CLOSE / CR_vFOC | STEP/DIR-Pulse | 400 / 1500 / 3000 U/min | **nicht** für Modbus |
| SR_OPEN | seriell (RS485)        | 400 U/min     | ohne Encoder, fester Strom     |
| SR_CLOSE| seriell (RS485)        | 1500 U/min    | mit Encoder, fester Strom      |
| **SR_vFOC** | seriell (RS485)    | 3000 U/min    | mit Encoder, adaptiver Strom (**empfohlen**) |

Handbücher und Schaltpläne: <https://github.com/makerbase-motor/MKS-SERVO42D-57D> (für Firmware V1.0.9 gibt es dort ein eigenes *Modbus RTU User Manual*). Die Firmware-Version steht im Servo-Menü unter `About`.

---

## 2. Verdrahtung

```
 Opta RS485 (Klemmen unten)           MKS SERVO42D (RS485-Stecker)
 ┌───────────┐                        ┌───────────┐
 │  A        ├────────────────────────┤  A        │
 │  B        ├────────────────────────┤  B        │
 │  GND      ├────────────────────────┤  GND      │
 └───────────┘                        └───────────┘

 24 V DC Netzteil ──┬── Opta  +/-  (12–24 V)
                    └── Servo V+/GND (12–24 V)
```

- **A ↔ A, B ↔ B, GND ↔ GND**, möglichst verdrilltes Kabel (z. B. Twisted Pair, 100–130 Ω).
- Die Bezeichnung von A/B ist zwischen Herstellern nicht einheitlich. **Antwortet der Servo nicht, A und B tauschen** (dabei geht nichts kaputt).
- Die gemeinsame Masse (GND) zwischen Opta und Servo verbessert die Störfestigkeit.
- Abschlusswiderstand 120 Ω: bei **einem** Servo und kurzer Leitung laut MKS nicht nötig. Bei langen Leitungen oder mehreren Slaves an beiden Busenden je 120 Ω zwischen A und B.
- Motorkabel und Datenkabel **nicht unter Spannung stecken**.

---

## 3. Einstellungen am Servo (Display-Menü)

Vor dem ersten Betrieb einmal **ohne Last kalibrieren** (`Cal`). Danach:

| Menüpunkt  | Wert       | Hinweis                                             |
|------------|------------|-----------------------------------------------------|
| `Mode`     | `SR_vFOC`  | oder `SR_CLOSE` / `SR_OPEN` – muss ein **SR_**-Modus sein |
| `UartBaud` | `38400`    | muss zu `MODBUS_BAUD` in `src/main.cpp` passen      |
| `UartAddr` | `01`       | muss zu `SERVO_SLAVE_ID` passen                     |
| `UartRSP`  | `Enable`   | sonst antwortet der Servo nicht auf Fahrbefehle → Timeout |
| `Mb_RTU`   | `Enable`   | Modbus RTU statt MKS-Protokoll                      |
| `Ma`       | z. B. `1600` | Arbeitsstrom in mA, passend zum Motor             |

---

## 4. Entwicklungsumgebung (PlatformIO)

Das Projekt ist fertig eingerichtet. Abhängigkeiten (`ststm32`-Plattform, Opta-Core, `ArduinoRS485`, `ArduinoModbus`) lädt PlatformIO beim ersten Build automatisch.

### In VS Code

1. Extension **PlatformIO IDE** installieren (wird beim Öffnen des Ordners vorgeschlagen).
2. Ordner öffnen → unten in der Statusleiste:
   - ✓ **Build**
   - → **Upload** (Opta per USB-C anschließen)
   - 🔌 **Serial Monitor**

### Auf der Kommandozeile

`pio` liegt bei einer PlatformIO-Installation unter `%USERPROFILE%\.platformio\penv\Scripts\pio.exe`, falls es nicht im PATH ist. Alternativ funktioniert es im **PlatformIO-Terminal** in VS Code direkt.

```powershell
pio run                  # kompilieren
pio run -t upload        # flashen (USB-C, DFU)
pio device monitor       # serieller Monitor (Strg+C zum Beenden)
```

### Upload-Modus und USB-Treiber (Windows)

Der Opta meldet sich je nach Zustand mit einer anderen USB-ID:

| USB-ID (VID:PID) | Zustand | Was Windows zeigt |
|------------------|---------|-------------------|
| `2341:0164` | Sketch läuft (Opta RS485; die PID hängt vom Opta-Modell ab) | nur ein COM-Port |
| `2341:0364` | **Bootloader = Upload-Modus** | COM-Port + DFU-Schnittstelle „Finder Bootloader“ |

Der COM-Port kann sich zwischen den beiden Zuständen ändern (z. B. COM8 im Bootloader, COM7 im Betrieb).

Prüfen mit `pio device list` (Zeile `Hardware ID: USB VID:PID=...`).

**Ablauf beim Upload:** PlatformIO öffnet den COM-Port kurz mit 1200 Baud. Daraufhin springt der Opta in den Bootloader, und `dfu-util` flasht das Programm. Normalerweise musst du dafür nichts drücken.

**Einmalig: DFU-Treiber installieren.** Ohne Treiber zeigt der Geräte-Manager die DFU-Schnittstelle mit Fehler (Code 28), und der Upload bricht mit `LIBUSB_ERROR_NOT_FOUND` ab. Der Treiber liegt dem PlatformIO-Paket bei. Installiere ihn in einer **PowerShell als Administrator**:

```powershell
pnputil /add-driver "$env:USERPROFILE\.platformio\packages\framework-arduino-mbed\drivers\mimsy.inf" /install
```

Danach den Opta kurz ab- und wieder anstecken. Zur Kontrolle muss `dfu-util -l` die Opta finden:

```powershell
& "$env:USERPROFILE\.platformio\packages\tool-dfuutil-arduino\dfu-util.exe" -l
# erwartet: Found DFU: [2341:0364] ... name="@Internal Flash ..."
```

(Alternativ richtet die **Arduino IDE 2** mit dem Boardpaket **„Arduino Mbed OS Opta Boards“** dieselben Treiber ein.)

**Upload-Probleme:**
- Kein COM-Port oder hängender Sketch: Reset-Taste am Opta **zweimal schnell drücken**. Der Opta bleibt dann im Bootloader (`2341:0364`), anschließend erneut hochladen.
- Der serielle Monitor blockiert den Upload: vor dem Upload schließen.

---

## 5. Bedienung im seriellen Monitor

Nach dem Start meldet der Opta, ob der Servo antwortet:

```
=== Opta RS485 -> MKS SERVO42D (Modbus RTU) ===
Bus: 38400 Baud, 8N1, Servo-ID 1
Servo antwortet: steht
'help' fuer alle Befehle. Erster Test: en -> spd 60 -> stop
>
```

Die Startmeldung erscheint nur, wenn das Terminal innerhalb von 3 s nach dem Start verbunden ist. Sonst einfach `status` eingeben.

### Terminal: PlatformIO-Monitor oder PuTTY

- **PlatformIO:** `pio device monitor` ist über `platformio.ini` bereits passend eingestellt (Echo an, Senden mit Enter).
- **PuTTY:**

  | Kategorie | Einstellung | Wert |
  |-----------|-------------|------|
  | Session | Connection type | `Serial` |
  | Session | Serial line | COM-Port des Opta, z. B. `COM7` (siehe `pio device list`) |
  | Session | Speed | `115200` (bei USB egal, aber **niemals 1200**: das versetzt den Opta in den Bootloader) |
  | Connection → Serial | Data bits / Stop bits / Parity | `8` / `1` / `None` |
  | Connection → Serial | Flow control | **`None`** (Standard ist XON/XOFF) |
  | Terminal | Local echo | **`Force on`** |
  | Terminal | Local line editing | **`Force on`** (Zeile wird erst mit Enter gesendet, Backspace funktioniert) |

  Die Einstellungen unter *Session* mit einem Namen (z. B. `Opta`) speichern (**Save**).

> Es kann immer nur **ein** Programm den COM-Port öffnen. Vor einem Upload PuTTY bzw. den Monitor schließen.

### Befehle

Optionale Parameter stehen in `[]`. Ohne `rpm`/`acc` werden die Standardwerte (`set`) verwendet (Start: 300 U/min, acc 200).

| Befehl | Funktion |
|--------|----------|
| `help` oder `?` | Befehlsübersicht |
| `status` | Zustand, Aktiviert, Blockierschutz, Drehzahl, Position, Pulse, Winkelfehler |
| `watch [ms]` | Drehzahl, Position und Winkelfehler zyklisch ausgeben (Default 500 ms), `watch 0` = aus |
| `en` / `dis` | Motor aktivieren (Haltemoment) / deaktivieren (Welle frei) |
| `spd <±rpm> [acc]` | Drehzahlmodus; Vorzeichen = Richtung, `spd 0` = Stopp mit Rampe |
| `rel <±pulse> [rpm] [acc]` | relativ um Pulse fahren (bei 16 Mikroschritten: 3200 = 1 Umdrehung) |
| `abs <pulse> [rpm] [acc]` | absolut auf Pulsposition fahren |
| `rot <±grad> [rpm] [acc]` | relativ um Winkel drehen (über Encoder-Achse) |
| `goto <grad> [rpm] [acc]` | absolut auf Winkel fahren (über Encoder-Achse) |
| `stop [acc]` | zuletzt gestartete Bewegung stoppen; `stop 0` = sofort |
| `estop` | Not-Halt (sofort) |
| `zero` | aktuelle Position als Nullpunkt setzen |
| `hcfg <dir> <rpm> <mA> [grad]` | Referenzfahrt gegen mechanischen Anschlag einstellen (siehe unten) |
| `home` | Referenzfahrt starten; das Ergebnis wird automatisch gemeldet |
| `unlock` | ausgelösten Blockierschutz zurücksetzen |
| `mode <0..5>` | Arbeitsmodus setzen (5 = SR_vFOC) |
| `cur <mA>` | Arbeitsstrom setzen (max. 3000) |
| `set [rpm\|acc\|id <wert>]` | Standardwerte anzeigen bzw. ändern (`id` = Ziel-Slave-Adresse) |
| `rd <reg> [anzahl]` | Input-Register roh lesen (FC 0x04), z. B. `rd 0x32` |
| `wr <reg> <wert>` | Holding-Register roh schreiben (FC 0x06), z. B. `wr 0xF3 1` |

Zahlen dürfen dezimal oder hexadezimal (`0x…`) eingegeben werden.

### Erster Test (Motor ohne Last)

```
> status            # Kommunikation prüfen
> en                # Motor aktivieren
> spd 60            # 60 U/min drehen
> spd -60           # Richtung wechseln
> stop              # sanft anhalten
> zero              # Position nullen
> rot 90            # 90 Grad weiter
> goto 0            # zurück auf 0
> rel 3200 120      # 1 Umdrehung (16 Mikroschritte) mit 120 U/min
> watch 250         # Live-Ausgabe, "watch 0" beendet
> dis               # Motor stromlos
```

### Einheiten und Parameter

- **Drehzahl `rpm`**: 0–3000 U/min, begrenzt durch den Modus (SR_OPEN 400, SR_CLOSE 1500). MKS kalibriert die Drehzahl auf 16/32/64 Mikroschritte; bei anderen Einstellungen skaliert sie (z. B. 8 Mikroschritte = doppelt so schnell).
- **Beschleunigung `acc`**: 0–255. Pro 1 U/min Drehzahländerung vergehen `(256 − acc) × 50 µs`.
  `acc = 200` → 0 auf 300 U/min in ca. 0,84 s · `acc = 236` → 1 ms pro U/min · `acc = 0` → ohne Rampe (sofort).
- **Richtung**: positives Vorzeichen → `dir = 0` (laut Handbuch CCW), negatives → `dir = 1` (CW). Beim Ablesen gilt: CCW = positive Drehzahl. Die tatsächliche Drehrichtung bitte einmal am Motor prüfen.
- **Pulse** (`rel`/`abs`): Mikroschritte, also `200 × MStep` pro Umdrehung.
- **Encoder-Achse** (`rot`/`goto`/Position): 16384 pro Umdrehung, unabhängig von den Mikroschritten. Die Genauigkeit liegt bei etwa ±15 Einheiten (≈ 0,3°); MKS empfiehlt dafür 64 Mikroschritte.
- Hohe Drehzahlen (> 1000 U/min) nicht mit `acc 0`/`estop` abrupt stoppen.

### Referenzfahrt gegen Anschlag (sensorlos)

Neuere Firmware-Versionen (eingeführt mit V1.0.5, hier umgesetzt nach dem Handbuch V1.0.9) können **selbst** gegen einen mechanischen Anschlag referenzieren. Er fährt mit begrenztem Strom in den Anschlag, erkennt den Stillstand und fährt dann um einen Rückzugswinkel zurück. Diese Position wird zum Nullpunkt. Einen Endschalter braucht es dafür nicht.

```
> en
> hcfg 0 60 500 10    # Richtung CW, 60 U/min, 500 mA, 10 Grad zurück
> home
Referenzfahrt gestartet (Abbruch: estop)
Referenzfahrt OK, Position 0.00 Grad
> goto 90             # Positionen beziehen sich jetzt auf den Anschlag
```

| Parameter | Bedeutung | Empfehlung |
|-----------|-----------|------------|
| `dir` | 0 = CW, 1 = CCW (Richtung zum Anschlag) | einmal ohne Mechanik prüfen |
| `rpm` | Suchdrehzahl | 30–120 U/min |
| `mA` | Strom beim Anfahren des Anschlags = **Kraft** | so klein wie möglich, nur so viel, dass die Last sicher bewegt wird (MKS-Beispiel: 600 mA) |
| `grad` | Rückzug vom Anschlag | z. B. 5–20°, Firmware-Default 180° |

- Die Werte entsprechen den Menüpunkten `HmDir`, `HmSpeed`, `Hm_Mode` und `Hm_Ma` und werden wie diese im Servo gespeichert. Nach einem Neustart reicht also `home`. Kontrollieren kannst du das im Menü.
- Der Strom wirkt wie eine Rutschkupplung: Er begrenzt die Kraft auf den Anschlag. Lieber mit wenig Strom anfangen und langsam erhöhen.
- Meldet `home` „Blockierschutz ausgelöst“, hat der Blockierschutz (`Protect`) zugeschlagen. Dann `unlock` senden und `Protect` während der Referenzfahrt deaktivieren (`wr 0x88 0`).
- **Drehmoment oder Strom lassen sich nicht auslesen**, auch nicht in Firmware V1.0.9. Live verfügbar sind Drehzahl, Position, **Winkelfehler** (`watch`) und das Blockier-Flag (`status`). Der Winkelfehler steigt deutlich an, wenn die Welle blockiert wird, und eignet sich deshalb für eine eigene Lasterkennung in der SPS.

---

## 6. Modbus-Registerübersicht (MKS SERVO42D)

Quellen: *MKS SERVO42D/57D_RS485 User Manual V1.0.4* (Teil 8) und *MKS SERVO42&57D Modbus RTU User Manual V1.0.9*. 32-/48-Bit-Werte im Big-Endian-Format (High-Word zuerst).

### Lesen – Input-Register (FC 0x04)

| Adresse | Anz. | Typ | Inhalt |
|---------|------|-----|--------|
| 0x0030 | 3 | int32 + uint16 | Encoder (Übertrag + Wert 0–0x3FFF) |
| 0x0031 | 3 | int48 | Encoder fortlaufend (0x4000 = 1 U) – **Position** |
| 0x0032 | 1 | int16 | Drehzahl in U/min (CCW > 0, CW < 0) |
| 0x0033 | 2 | int32 | empfangene Pulse |
| 0x0034 | 1 | uint8 | IO-Status (Bit0 IN_1, Bit1 IN_2, Bit2 OUT_1, Bit3 OUT_2) |
| 0x0039 | 2 | int32 | Winkelfehler (51200 = 360°) |
| 0x003A | 1 | uint8 | Aktiviert (1 = ja) |
| 0x003B | 1 | 2 × uint8 | **Hi: Referenzfahrt**, Lo: Single-Turn-Null (je 0 läuft, 1 ok, 2 nicht referenziert/Fehler) ¹ |
| 0x003E | 1 | uint8 | blockiert / Blockierschutz ausgelöst (1 = ja) |
| 0x0040 | 2 | uint32 | Firmware-Version, untere 3 Bytes (`0x01010009` = V1.0.9) |
| 0x00F1 | 1 | uint8 | Motorzustand (1 steht, 2 beschl., 3 bremst, 4 volle Drehzahl, 5 Referenzfahrt – auch während des Rückzugs, 6 Kalibrierung) |

¹ Am Gerät mit Firmware V1.0.9 gemessen: Während einer Anschlag-Referenzfahrt wechselt das High-Byte von 0 auf 1, das Low-Byte bleibt 2. Das Handbuch V1.0.9 beschreibt die beiden Bytes vertauscht.

### Schreiben – einzelne Register (FC 0x06)

| Adresse | Wert | Funktion |
|---------|------|----------|
| 0x003D | 1 | Blockierschutz zurücksetzen |
| 0x003F | 1 | Werkseinstellungen (danach neu kalibrieren!) |
| 0x0080 | 1 | Kalibrieren (ohne Last!) |
| 0x0082 | 0–5 | Arbeitsmodus (5 = SR_vFOC) |
| 0x0083 | mA | Arbeitsstrom |
| 0x0084 | n | Mikroschritte |
| 0x0088 | 0/1 | Blockierschutz aus/ein |
| 0x008A | 1–7 | Baudrate (1 9600 … 4 38400 … 7 256000) |
| 0x008B | n | Slave-Adresse |
| 0x008E | 0/1 | Modbus RTU aus/ein |
| 0x0091 | 1 / 2 | 1 = Referenzfahrt starten, 2 = direkt auf den Nullpunkt fahren |
| 0x0092 | 1 | aktuelle Position = 0 |
| 0x00F3 | 0/1 | Motor deaktivieren/aktivieren |
| 0x00F7 | 1 | Not-Halt |
| 0x00FF | 0xC8/0xCA | Drehzahlmodus-Parameter speichern/löschen (Autostart nach Power-on) |

### Fahrbefehle – mehrere Register (FC 0x10)

| Adresse | Anz. | Register-Inhalt | Funktion |
|---------|------|-----------------|----------|
| 0x00F6 | 2 | `[dir<<8 \| acc] [speed]` | Drehzahlmodus (speed 0 = Stopp) |
| 0x00FD | 4 | `[dir<<8 \| acc] [speed] [pulse hi] [pulse lo]` | relativ um Pulse |
| 0x00FE | 4 | `[acc] [speed] [pos hi] [pos lo]` (int32) | absolut auf Pulse |
| 0x00F4 | 4 | `[acc] [speed] [achse hi] [achse lo]` (int32) | relativ um Encoder-Achse |
| 0x00F5 | 4 | `[acc] [speed] [achse hi] [achse lo]` (int32) | absolut auf Encoder-Achse |

Gestoppt wird jeweils mit demselben Register und `speed = 0`: Mit `acc > 0` bremst der Motor über eine Rampe, mit `acc = 0` hält er sofort an.

### Konfiguration – mehrere Register (FC 0x10)

| Adresse | Anz. | Inhalt | Funktion |
|---------|------|--------|----------|
| 0x0090 | 3 | **nur 5 Datenbytes:** `trig:8 dir:8 speed:16 endLimit:8` | Referenzfahrt: Richtung, Drehzahl (MKS-Sonderformat, wird roh über RS485 gesendet) |
| 0x0094 | 4 | `[offset hi] [offset lo] [mode] [mA]` | Referenzfahrt: Rückzug (0x4000 = 360°), Art (0 Endschalter, 1 Anschlag, 2 Single-Turn), Strom |
| 0x0098 | 2 | uint32 ms | Heartbeat: Not-Halt, wenn so lange kein Telegramm kommt (0 = aus) |
| 0x009D | 3 | `[0 \| enable] [Zeit × 15 ms] [Fehler, 28000 = 360°]` | Schutz bei zu großem Positionsfehler |

---

## 7. Konfiguration im Code

Alle Einstellungen stehen oben in [src/main.cpp](src/main.cpp):

| Konstante | Default | Bedeutung |
|-----------|---------|-----------|
| `MODBUS_BAUD` | 38400 | Baudrate (= `UartBaud` am Servo) |
| `MODBUS_FORMAT` | `SERIAL_8N1` | Datenformat |
| `SERVO_SLAVE_ID` | 1 | Slave-Adresse (= `UartAddr`) |
| `MODBUS_TIMEOUT_MS` | 200 | Antwort-Timeout |
| `RS485_PRE_DELAY_US` | 3,5 Zeichen | Pause nach dem Einschalten des Senders, bevor das erste Byte gesendet wird |
| `RS485_POST_DELAY_US` | 1,5 Zeichen | Nachlauf des Senders nach dem letzten Byte |
| `gRpm` / `gAcc` | 300 / 200 | Standardwerte für Bewegungen |

> **Hinweis zum Post-Delay:** Der Opta schaltet erst nach `RS485_POST_DELAY_US` auf Empfang. Das offizielle Arduino-Beispiel verwendet dafür 3,5 Zeichenzeiten. Ein schnell antwortender Slave wie der MKS-Servo könnte dann schon senden, bevor der Opta zuhört. Deshalb ist hier 1,5 Zeichenzeiten eingestellt (≈ 390 µs bei 38400 Baud).

---

## 8. Fehlersuche

| Symptom | Mögliche Ursache / Lösung |
|---------|---------------------------|
| `FEHLER: Connection timed out` | Servo ohne Versorgung · A/B vertauscht · `Mb_RTU` nicht `Enable` · Baudrate/Adresse falsch · `UartRSP` = Disable |
| `Invalid CRC` / sporadische Timeouts | `RS485_POST_DELAY_US` anpassen (z. B. 300–900 µs testen) · Leitung verdrillen, gemeinsame Masse · Abschlusswiderstand bei langer Leitung |
| `OK`, aber Motor dreht nicht | Modus ist kein `SR_…` · Motor nicht aktiviert (`en`) · Blockierschutz aktiv (`status` → `unlock`) · nicht kalibriert |
| Motor ruckelt / „Phase Line Error!“ | Kalibrierung wiederholen · Netzteil zu schwach · Motorkabel prüfen |
| `Illegal data address` | Register wird von der Firmware nicht unterstützt – Firmware-Version am Servo (`About`) prüfen |
| Kein serieller Output | USB-C-Kabel mit Datenleitungen verwenden · Monitor nach dem Upload neu öffnen |

---

## 9. Projektstruktur

```
├── platformio.ini      # PlatformIO-Konfiguration (Board opta, Bibliotheken)
├── src/
│   ├── main.cpp        # serielles Befehls-Interface + Konfiguration
│   ├── MksServo.h      # Treiber: Registertabelle und API
│   └── MksServo.cpp    # Treiber: Modbus-Zugriffe
└── README.md
```

Die Klasse `mks::MksServo` ist unabhängig vom seriellen Interface und kann später direkt in der SPS-Logik verwendet werden, z. B. für Eingänge, Taster oder Ethernet/Modbus TCP:

```cpp
mks::MksServo servo(ModbusRTUClient, 1);
servo.enable(true);
servo.runSpeed(120, 200);          // 120 U/min, Rampe 200
servo.moveRelativeAxis(4096, 300, 200);  // Viertelumdrehung
```

---

## Quellen

- MKS SERVO42D/57D RS485 User Manual V1.0.4 (Teil 8: Modbus RTU) und **MKS SERVO42&57D Modbus RTU User Manual V1.0.9** – [makerbase-motor/MKS-SERVO42D-57D › User Manual](https://github.com/makerbase-motor/MKS-SERVO42D-57D/tree/master/User%20Manual)
- [Arduino Opta RS485 – Produktseite](https://store.arduino.cc/products/opta-rs485)
- [Getting Started with Modbus RTU on Opta](https://docs.arduino.cc/tutorials/opta/getting-started-with-modbus-rtu)
- [Getting Started with RS-485 on Opta](https://docs.arduino.cc/tutorials/opta/getting-started-with-rs485)
- [PlatformIO Board „Arduino Opta“](https://docs.platformio.org/page/boards/ststm32/opta.html)
