# Arduino Opta RS485 → MKS SERVO42D über Modbus RTU

Steuert einen **MKS SERVO42D** (NEMA17 Closed-Loop, RS485-Version, Firmware V1.0.9) von einer **Arduino Opta RS485** aus über Modbus RTU. Bedient wird er über Textbefehle im seriellen Monitor (USB-C).

## Hardware und Verdrahtung

| Opta RS485 | | MKS SERVO42D |
|---|---|---|
| A | ↔ | A |
| B | ↔ | B |
| GND | ↔ | GND |

- Opta und Servo laufen mit 12–24 V DC (Servo: mind. 24 V/1 A).
- **Antwortet der Servo nicht, A und B tauschen.** Die Benennung ist nicht bei allen Herstellern gleich.
- Bei einem Servo und kurzer Leitung ist kein 120-Ω-Abschluss nötig.
- Kabel nicht unter Spannung stecken.

## Einstellungen am Servo (Menü)

Einmal **ohne Last kalibrieren** (`Cal`), dann:

| Menü | Wert |
|---|---|
| `Mode` | `SR_vFOC` (muss ein SR_-Modus sein) |
| `UartBaud` / `UartAddr` | `38400` / `01` (= `MODBUS_BAUD` / `SERVO_SLAVE_ID` in `main.cpp`) |
| `UartRSP` | `Enable` |
| `Mb_RTU` | `Enable` |

## Build und Upload (PlatformIO)

In VS Code mit der Extension **PlatformIO IDE**: Build ✓, Upload →, Monitor 🔌. Oder im Terminal:

```powershell
pio run -t upload      # pio.exe liegt ggf. unter %USERPROFILE%\.platformio\penv\Scripts\
```

- **Einmalig unter Windows den DFU-Treiber installieren** (PowerShell als Administrator), danach den Opta neu anstecken:
  ```powershell
  pnputil /add-driver "$env:USERPROFILE\.platformio\packages\framework-arduino-mbed\drivers\mimsy.inf" /install
  ```
- Vor dem Upload das Terminal (PuTTY/Monitor) schließen.
- Upload klappt nicht: Reset-Taste am Opta **zweimal schnell** drücken (Bootloader, USB-ID `2341:0364`) und erneut hochladen.

## Terminal

- **PlatformIO:** `pio device monitor` ist schon passend eingestellt.
- **PuTTY:** Serial, COM-Port des Opta (`pio device list`), Speed 115200 (**niemals 1200**, das startet den Bootloader), Flow control **None**, Terminal → Local echo und Local line editing **Force on**.

Beim Verbinden nach dem Start fehlt die Begrüßung, dann einfach `status` eingeben.

## Befehle

`[ ]` = optional. Ohne `rpm`/`acc` gelten die Standardwerte (300 U/min, acc 200, änderbar mit `set`). Zahlen dezimal oder `0x…`.

| Befehl | Funktion |
|---|---|
| `help` | Befehlsübersicht |
| `status` | Zustand, Firmware, Referenz, Drehzahl, Position, Winkelfehler |
| `watch [ms]` | Drehzahl, Position und Winkelfehler zyklisch, `watch 0` = aus |
| `en` / `dis` | Motor aktivieren / deaktivieren (**`dis` nur im Stillstand**) |
| `spd <±rpm> [acc]` | Drehzahlmodus, Vorzeichen = Richtung |
| `rel <±pulse> [rpm] [acc]` | relativ um Pulse (16 Mikroschritte: 3200 = 1 U) |
| `abs <pulse> [rpm] [acc]` | absolut auf Pulsposition |
| `rot <±grad> [rpm] [acc]` | relativ um Winkel (Encoder) |
| `goto <grad> [rpm] [acc]` | absolut auf Winkel (Encoder) |
| `stop [acc]` / `estop` | letzte Bewegung stoppen (`stop 0` = sofort) / Not-Halt |
| `zero` | aktuelle Position = 0 |
| `hcfg <dir> <rpm> <mA> [grad]` / `home` | Referenzfahrt einstellen / starten (siehe unten) |
| `unlock` | Blockierschutz zurücksetzen |
| `mode <0..5>` | Arbeitsmodus (5 = SR_vFOC) |
| `cur <mA>` | Arbeitsstrom, max. 3000 (**nur im Stillstand**) |
| `set [rpm\|acc\|id <wert>]` | Standardwerte anzeigen/ändern |
| `rd <reg> [n]` / `wr <reg> <wert>` | Register roh lesen (FC 04) / schreiben (FC 06) |

Erster Test ohne Last: `en` → `spd 60` → `stop` → `rot 90` → `goto 0` → `dis`.

## Referenzfahrt gegen Anschlag (ohne Endschalter)

Der Servo fährt mit begrenztem Strom in den Anschlag, erkennt den Stillstand, fährt um den Rückzugswinkel zurück und setzt dort den Nullpunkt.

```
> hcfg 0 60 500 10    # dir 0 = CW / 1 = CCW, 60 U/min, 500 mA, 10° Rückzug
> home
Referenzfahrt OK, Position 0.20 Grad
```

- Der Strom bestimmt die Kraft auf den Anschlag: klein anfangen, erst bei Bedarf erhöhen.
- Die Werte bleiben im Servo gespeichert. Nach einem Neustart reicht `home`.
- Meldet `home` einen ausgelösten Blockierschutz: `unlock` senden und den Schutz abschalten (`wr 0x88 0`).

## Gut zu wissen

- **Der Servo lehnt manche Befehle ab, solange der Motor läuft** (beobachtet bei `dis` und `cur`). Das Programm meldet dann `vom Servo abgelehnt`, also vorher `stop` senden.
- **Strom und Drehmoment lassen sich nicht auslesen.** Als Hinweis auf die Last dient der Winkelfehler (`watch`).
- Einheiten: Encoder 16384 pro Umdrehung, Winkelfehler 51200 = 360°. Beschleunigung `acc`: `(256 − acc) × 50 µs` pro 1 U/min.
- Die Registertabelle mit allen Eigenheiten steht kommentiert in [src/MksServo.h](src/MksServo.h). Zwei Abweichungen vom Handbuch V1.0.9:
  - Im Statusregister 0x3B steht die Referenzfahrt im **High-Byte**.
  - Register 0x90 erwartet ein Sonderformat mit 5 Datenbytes und wird deshalb roh über RS485 gesendet.

## Fehlersuche

| Meldung | Ursache / Lösung |
|---|---|
| `Connection timed out` | Versorgung, A/B tauschen, `Mb_RTU`, Baudrate/Adresse, `UartRSP` prüfen |
| `Invalid CRC`, sporadische Timeouts | `RS485_POST_DELAY_US` in `main.cpp` anpassen (300–900 µs), Leitung verdrillen |
| `vom Servo abgelehnt` | Motor läuft noch, erst `stop` |
| `OK`, aber keine Bewegung | kein SR_-Modus, nicht aktiviert (`en`), Blockierschutz (`unlock`), nicht kalibriert |

## Projektstruktur

```
platformio.ini     Board opta, Bibliotheken ArduinoRS485 + ArduinoModbus
src/main.cpp       Konfiguration, serielles Befehls-Interface
src/MksServo.*     Treiber für den Servo (wiederverwendbar für die SPS-Logik)
```

## Quellen

- [MKS SERVO42D/57D Handbücher (u. a. Modbus RTU Manual V1.0.9)](https://github.com/makerbase-motor/MKS-SERVO42D-57D/tree/master/User%20Manual)
- [Arduino Opta RS485](https://store.arduino.cc/products/opta-rs485) · [Modbus RTU auf dem Opta](https://docs.arduino.cc/tutorials/opta/getting-started-with-modbus-rtu)
