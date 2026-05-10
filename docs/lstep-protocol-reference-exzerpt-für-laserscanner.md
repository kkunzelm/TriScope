Basierend auf der technischen Referenz für die **Lang LStep 23** Steuerung sind hier die für dein Laserscanner-Projekt relevanten Informationen zusammengefasst:

### 1. Verbindungsparameter (Seriell)

Die Kommunikation erfolgt über eine serielle Schnittstelle mit strikten Vorgaben:

* **Konfiguration:** Baudrate 19200, 8 Datenbits, keine Parität, **2 Stopbits** (essentiell für Stabilität).
* **Flowcontrol:** Nutze `HardwareControl` (RTS/CTS). Der Befehl `!cts 1` muss zwingend als allererstes Byte gesendet werden, um die Kommunikation zu starten.
* **Signale:** Nach dem Öffnen müssen DTR und RTS explizit auf `true` gesetzt werden.

### 2. Protokoll & Befehlsstruktur (MCL3)

* **Format:** Jeder Befehl folgt dem Schema: `0x55` (Präfix) + `Register-Byte` (binär) + `Wert` (ASCII-String) + `0x0D` (CR).
* **Zwei-Schritt-Bewegung:** Bewegungen werden nicht sofort ausgeführt. Zuerst wird das Sub-Kommando in Register 7 geschrieben (z. B. `U(7)r` für absolut), dann löst der Befehl `UP` die Fahrt aus.
* **Antwort-Handling:** Die Steuerung antwortet auf den `UP`-Befehl erst, wenn die Bewegung **abgeschlossen** ist. Der resultierende 5-Byte-String (z. B. `@@@-.\r`) gibt den Status der Endschalter an.

### 3. Einheiten und Koordinaten

* **Auflösung:** 1 Unit entspricht **1 µm** (0,001 mm).
* **Spindelsteigung (Pitch):** Muss für eine korrekte interne Berechnung gesetzt werden. Standard für den XYZ-Tisch: X/Y = 2,0 mm (`20000`), Z = 1,0 mm (`10000`).
* **Hardware-Nullpunkt:** Nach der Referenzfahrt (`Calibrate`) ist der Nullpunkt oben-rechts-hinten. Die Werte wachsen nach links (X), vorne (Y) und unten (Z).
* **Transformation:** Für ein rechtshändiges Koordinatensystem (Z=0 oben, Z negativ nach unten) müssen die Hardware-Werte in der Software gespiegelt/invertiert werden.

### 4. Relevante Register für den Scanner

| Funktion | Register (hex) | Beschreibung |
| --- | --- | --- |
| **X-Ziel** | `0x00` | Zielposition für die Scan-Linie in Units. |
| **ActiveAxes** | `0x0B` | Muss vor Bewegungen auf `7` (XYZ) gesetzt werden. |
| **Speed / Ramp** | `0x09` / `0x08` | Empfohlen: Speed 50, Ramp 500 (Z-Achse sanfter mit Ramp 200). |
| **Position** | `0x43` (UC) | Abfrage der aktuellen X-Position (Rohwert). |

### 5. Strategie für die Scanner-Anwendung

* **Triggerung:** Da die Steuerung auf `UP` erst nach Abschluss der Bewegung antwortet, ist das Empfangen der Status-Antwort (`statusUpdated`) der ideale Software-Trigger, um die Bildaufnahme der IDS-Kamera auszulösen.
* **Präzision:** Um das mechanische Umkehrspiel (Backlash) zu eliminieren, sollte die X-Achse (Scan-Richtung) während der Messfahrt immer nur in eine Richtung verfahren werden.
* **Initialisierung:** Die im Dokument unter Punkt 4 aufgeführte 20-stufige Sequenz ist zwingend erforderlich, da die Steuerung sonst nicht korrekt reagiert.

### 6. Software-Architektur (Qt6)

* **Thread-Trennung:** Der `LStepController` sollte in einem eigenen `QThread` laufen, um das GUI nicht zu blockieren, während auf die Antwort der Steuerung (bis zu 30s bei langen Wegen) gewartet wird.
* **Queue-System:** Implementiere eine FIFO-Warteschlange, da die Steuerung immer nur einen Befehl gleichzeitig verarbeiten kann.
