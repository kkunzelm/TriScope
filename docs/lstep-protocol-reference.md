# Lang LStep MCL3 – Technische Referenz für Softwareentwickler

Dieses Dokument fasst alles zusammen, was für die Programmierung der
**Lang LStep 23**-Steuerung mit dem **MCL3-Register-Befehlssatz** nötig ist.
Es entstand aus der Implementierung einer Qt6-Steuerungssoftware für den
**Walter Uhl MS4-WT02** xyz-Messtisch und ist so geschrieben, dass es
auch als Ausgangspunkt für neue Projekte (z. B. einen Triangulations-Laser-Scanner
oder eine automatisierte Messstation) dienen kann.

Alle Angaben sind empirisch verifiziert an:
- Messtisch: Walter Uhl MS4-WT02
- Steuerung: Lang LStep 23
- Firmware: LS31.00.011
- Protokoll: MCL3

---

## Inhaltsverzeichnis

1. [Serielle Schnittstelle](#1-serielle-schnittstelle)
2. [Protokollformat](#2-protokollformat)
3. [Registerübersicht](#3-registerübersicht)
4. [Initialisierungssequenz](#4-initialisierungssequenz)
5. [Bewegungsbefehle](#5-bewegungsbefehle)
6. [Positionsabfrage und Einheiten](#6-positionsabfrage-und-einheiten)
7. [Statusantworten und Endschalter](#7-statusantworten-und-endschalter)
8. [Koordinatensystem](#8-koordinatensystem)
9. [Geschwindigkeit und Beschleunigung](#9-geschwindigkeit-und-beschleunigung)
10. [Spindelsteigung (Pitch)](#10-spindelsteigung-pitch)
11. [Firmware-Version abfragen](#11-firmware-version-abfragen)
12. [Bekannte Fallstricke](#12-bekannte-fallstricke)
13. [Software-Architektur](#13-software-architektur)
14. [Erweiterung: Scanner-Anwendung](#14-erweiterung-scanner-anwendung)

---

## 1. Serielle Schnittstelle

### Exakte Konfiguration

```
Baudrate:      19200
Datenbits:     8
Parität:       None
Stopbits:      2   ← KRITISCH: Qt6-Default ist 1, MUSS explizit gesetzt werden
Flowcontrol:   HardwareControl (RTS/CTS, gekreuztes Nullmodem-Kabel)
DTR:           nach open() einschalten
```

### Typischer Qt6-Code zum Öffnen

```cpp
m_port.setBaudRate(QSerialPort::Baud19200);
m_port.setDataBits(QSerialPort::Data8);
m_port.setParity(QSerialPort::NoParity);
m_port.setStopBits(QSerialPort::TwoStop);           // 2 Stopbits!
m_port.setFlowControl(QSerialPort::HardwareControl); // RTS/CTS

m_port.open(QIODevice::ReadWrite);
m_port.setDataTerminalReady(true);
m_port.setRequestToSend(true);
m_port.clear(QSerialPort::AllDirections);            // Puffer leeren
```

### XON/XOFF – warum nicht über Qt

Die Steuerung erwartet beim Start sowohl RTS/CTS-Hardware-Flowcontrol als
auch die XON-Ankündigung (`!cts 1`). `QSerialPort` unterstützt Hardware- und
Software-Flowcontrol **nicht gleichzeitig**: Sobald `SoftwareControl` gesetzt
wird, deaktiviert Qt intern `HardwareControl`.

**Lösung:** `FlowControl` auf `HardwareControl` setzen. Den `!cts 1`-Befehl
als erstes Byte in der Initialisierungssequenz manuell senden.
Niemals `QSerialPort::SoftwareControl` verwenden.

### Pseudo-Terminals (Entwicklung / Simulator)

Socat-Pseudo-Terminals (`/dev/pts/x`, `/tmp/lstep-app`) haben keine echten
RTS/CTS-Leitungen. Qt wartet beim `HardwareControl`-Modus auf CTS, das nie
gesetzt wird → Schreib-Hänger.

```cpp
const bool isPty = portName.startsWith("/tmp/") || portName.startsWith("/dev/pts/");
m_port.setFlowControl(isPty ? QSerialPort::NoFlowControl
                            : QSerialPort::HardwareControl);
```

---

## 2. Protokollformat

### Befehlsaufbau

Jeder Befehl besteht aus genau drei Teilen:

```
0x55  +  <Register-Byte als binäre Zahl>  +  <Wert als ASCII-String>  +  0x0D (CR)
```

**Wichtig:** Das Register-Byte ist eine **binäre Zahl**, kein ASCII-Zeichen.
Register 7 wird als Byte `0x07` gesendet, nicht als ASCII-Zeichen `'7'`.

Kurzschreibweise in Kommentaren und Dokumentationen:
```
U(7)c  →  0x55 0x07 'c' 0x0D
```

### Lese- vs. Schreibbefehle

- **Schreiben:** `0x55` + `Schreibadresse` + `ASCII-Wert` + `0x0D`
- **Lesen:** `0x55` + `Leseadresse` + `0x0D` (kein Wert-Byte)

**Leseadresse = Schreibadresse + 64** (gilt für alle Register)

Ausnahme: Einige Register haben nur eine Lese- oder nur eine Schreibadresse.

### Antwortformat

- Alle Antworten werden mit `0x0D` (CR) abgeschlossen
- **Niemals auf eine feste Anzahl Bytes warten**, sondern Bytes puffern
  und bei CR auswerten
- Länge der Antwort ist befehlsabhängig (1 Byte bis mehrere Zeichen)

### Schreibbefehle ohne Antwort

Viele Befehle (Preselection setzen, Geschwindigkeit setzen, Achsmaske setzen)
liefern keine Antwort. Den nächsten Befehl erst senden, wenn die Bytes
physisch übertragen wurden:

```cpp
m_port.write(cmdBytes);
m_port.waitForBytesWritten(timeoutMs);
```

### Bewegungsablauf: immer zwei Schritte

Bewegungsbefehle bestehen stets aus zwei Teilen:
1. Sub-Kommando in Register 7 schreiben: `U(7)<buchstabe>`
2. Start-Befehl senden: `UP` (= `0x55 0x50 0x0D`)

Der `UP`-Befehl löst die Bewegung aus und **wartet auf die Status-Antwort**
(CR-terminiert). Erst nach Empfang der Antwort ist die Bewegung abgeschlossen.

---

## 3. Registerübersicht

### Schreib-/Lese-Register (Lese = Schreib + 64)

| Funktion              | Schreiben | Lesen | Hex (Schreib) | Anmerkung                          |
|-----------------------|-----------|-------|---------------|------------------------------------|
| Preselection X        | 0         | 64    | 0x00          | Zielposition oder Delta in Units   |
| Preselection Y        | 1         | 65    | 0x01          |                                    |
| Preselection Z        | 2         | 66    | 0x02          |                                    |
| Command               | 7         | 71    | 0x07          | Bewegungs-Sub-Kommando schreiben   |
| Ramp (Beschleunigung) | 8         | 72    | 0x08          | Wert 1–500+, empfohlen 500         |
| Speed (Geschwindigkeit)| 9        | 73    | 0x09          | Wert 1–100, empfohlen 50           |
| CurrentReduction      | 10        | 74    | 0x0A          | Wert 5 (Haltestrom-Reduktion)      |
| ActiveAxes            | 11        | 75    | 0x0B          | Bitmaske: 1=X, 2=Y, 4=Z, 7=alle   |
| Pitch X               | 21        | 85    | 0x15          | Steigung in 0,0001 mm (20000=2mm)  |
| Pitch Y               | 22        | 86    | 0x16          |                                    |
| Pitch Z               | 23        | 87    | 0x17          |                                    |
| Init-Setting U(24)    | 24        | 88    | 0x18          | Init: ASCII "000000000"            |
| Trigger               | 35        | 99    | 0x23          | 0=aus; für Kamera-Trigger nutzbar  |
| MotorCurrent X        | 48        | 112   | 0x30          | Wert 100 (= 100 %)                 |
| MotorCurrent Y        | 49        | 113   | 0x31          |                                    |
| MotorCurrent Z        | 50        | 114   | 0x32          |                                    |
| Init-Setting U(47)    | 47        | 111   | 0x2F          | Init: Wert 65087; Funktion unklar  |

### Nur-Lese-Register

| Funktion           | Lese-Byte | ASCII-Kürzel | Anmerkung                        |
|--------------------|-----------|--------------|----------------------------------|
| Absolut-Position X | 67        | 'C' → "UC"   | Rohwert in Units                 |
| Absolut-Position Y | 68        | 'D' → "UD"   |                                  |
| Absolut-Position Z | 69        | 'E' → "UE"   |                                  |
| Status             | 70        | 'F' → "UF"   | 5-Byte-Antwort, s. Abschnitt 7   |
| ActiveAxes lesen   | 75        | 'K' → "UK"   | Antwort z. B. "7\r"              |
| Start / Trigger    | 80        | 'P' → "UP"   | Löst Bewegung aus, liefert Status|

---

## 4. Initialisierungssequenz

Diese Sequenz **muss** nach dem Öffnen des Ports byte-identisch gesendet werden.
Sie stammt aus einem WinCommander-Sniffer-Protokoll und ist experimentell
verifiziert. **Reihenfolge und Werte nicht ändern.**

```
Schritt  Befehl                         Hex-Bytes                         Bedeutung
──────────────────────────────────────────────────────────────────────────────────────
 1       !cts 1\r                        21 63 74 73 20 31 0D             XON ankündigen
 2       U(11)7\r                        55 0B 37 0D                      ActiveAxes = XYZ (alle)
 3       U(9)50\r                        55 09 35 30 0D                   Speed = 50
 4       U(8)500\r                       55 08 35 30 30 0D                Ramp = 500
 5       U(21)20000\r                    55 15 32 30 30 30 30 0D          Pitch X = 2,0 mm
 6       U(22)20000\r                    55 16 32 30 30 30 30 0D          Pitch Y = 2,0 mm
 7       U(23)10000\r                    55 17 31 30 30 30 30 0D          Pitch Z = 1,0 mm
 8       U(48)100\r                      55 30 31 30 30 0D                MotorCurrent X = 100%
 9       U(49)100\r                      55 31 31 30 30 0D                MotorCurrent Y = 100%
10       U(50)100\r                      55 32 31 30 30 0D                MotorCurrent Z = 100%
11       U(10)5\r                        55 0A 35 0D                      CurrentReduction = 5
12       U(47)65087\r                    55 2F 36 35 30 38 37 0D          Unbekannt (Init-Pflicht)
13       U(24)000000000\r                55 18 30 ... 30 0D (12 Bytes)    Unbekannt (Init-Pflicht)
14       U(7)d0x\r                       55 07 64 30 78 0D                SetTVRMode 0 für X
15       U(7)d0y\r                       55 07 64 30 79 0D                SetTVRMode 0 für Y
16       U(7)d0z\r                       55 07 64 30 7A 0D                SetTVRMode 0 für Z
17       U(11)7\r                        55 0B 37 0D                      ActiveAxes = XYZ
18       U(35)0\r                        55 23 30 0D                      Trigger = false
19       U(8)500\r                       55 08 35 30 30 0D                Ramp = 500
20       U(9)50\r                        55 09 35 30 0D                   Speed = 50
```

**Schritte 12 und 13 (U(47) und U(24)):** Zweck dieser Register ist unbekannt,
aber ohne sie kommuniziert die Steuerung nach der Init-Sequenz nicht korrekt.
Werte stammen aus dem Sniffer-Protokoll und sind offensichtlich
konfigurationsspezifisch für den MS4-WT02.

**Keine Antworten:** Keiner dieser Befehle liefert eine Antwort. Sie können
nacheinander mit `waitForBytesWritten` gesendet werden.

### Spindelsteigung nach Init überschreiben

Die Schritte 5–7 setzen die Steigung auf die Defaults des MS4-WT02.
Für andere Maschinen muss danach `setPitch` aufgerufen werden:

```cpp
// Beispiel: X=3,0 mm, Y=3,0 mm, Z=2,0 mm
setPitch(30000, 30000, 20000);
```

---

## 5. Bewegungsbefehle

Alle Bewegungen folgen demselben Schema:
1. Optionale Parameter setzen (Preselection, Speed, Ramp, ActiveAxes)
2. Sub-Kommando in Register 7 schreiben
3. `UP` senden → Steuerung antwortet nach Abschluss der Bewegung mit Status-Byte

### Referenzfahrt (Calibrate)

Fährt alle aktivierten Achsen zu den Nullschaltern (Home). Setzt den
Hardware-Nullpunkt der Steuerung.

```
Vorbereitung:
  U(9)100\r   → Speed = 100  (schnell zur Referenzposition)
  U(8)50\r    → Ramp = 50    (sanfte Beschleunigung für Endschalter-Anfahrt)
  U(11)7\r    → ActiveAxes = alle

Bewegung:
  U(7)c\r     → Sub-Kommando Calibrate
  UP\r        → Start; Antwort nach ca. 10–15 s: "AAA-.\r"

Danach:
  U(9)50\r    → Speed zurücksetzen
  U(8)500\r   → Ramp zurücksetzen
```

**Erwartete Antwort:** `"AAA-.\r"` — alle drei Achsen am Nullschalter.

**Kritisch:** Vor jedem Calibrate muss `ActiveAxes` auf `7` (alle Achsen)
gesetzt sein. Fehlt das, wartet die Steuerung auf eine Rotationsachse
(die beim MS4-WT02 nicht vorhanden ist) und hängt dauerhaft.

**Timeout:** 60 Sekunden empfohlen.

### Tischhub messen (MeasureLength)

Fährt alle Achsen zu den **Endschaltern** (entgegengesetzt zu Calibrate).
Damit wird der maximale Verfahrweg gemessen.

```
U(11)7\r    → ActiveAxes = alle
U(7)l\r     → Sub-Kommando MeasureLength
UP\r        → Start; Antwort nach Abschluss: "DDD-.\r"
```

**Nach der Antwort:** Sofort die Absolutpositionen UC, UD, UE lesen. Die
dort gemeldeten Rohwerte sind die Hardware-Maxima (in Units). Mit
`rawValue / unitsPerMm` erhält man den Verfahrweg in mm.

**Reihenfolge:** Muss nach Calibrate aufgerufen werden. Ohne vorherigen
Nullpunkt sind die Positionswerte nicht sinnvoll interpretierbar.

**Timeout:** 60 Sekunden empfohlen.

### Absolute Bewegung (MoveAbsolute)

Fährt zu einer absoluten Hardware-Koordinate (in Units ab Hardware-Nullpunkt).

```
U(0)<X_units>\r   → Preselection X setzen
U(1)<Y_units>\r   → Preselection Y setzen
U(2)<Z_units>\r   → Preselection Z setzen
U(11)7\r          → ActiveAxes = alle (oder Teilmenge)
U(7)r\r           → Sub-Kommando MoveAbsolute
UP\r              → Start; Antwort nach Abschluss: Status-String
```

**Wenn nur einzelne Achsen bewegt werden sollen:** ActiveAxes entsprechend
setzen (z. B. `4` für nur Z). Nur die Preselection-Register der
aktivierten Achsen werden ausgewertet.

**Timeout:** 30 Sekunden (je nach Verfahrweg mehr).

### Relative Bewegung (MoveRelative)

Fährt einen Delta-Wert relativ zur aktuellen Position.

```
U(0)<delta_X_units>\r   → Delta X (positiv oder negativ)
U(11)1\r                → ActiveAxes = nur X (Bitmaske!)
U(7)v\r                 → Sub-Kommando MoveRelative
UP\r                    → Start; Antwort: Status-String
```

**Vorzeichen:** Der Delta-Wert kann negativ sein. Die Steuerung akzeptiert
negative ASCII-Zahlen (z. B. `-500`).

### NOT-AUS (Abort)

Unterbricht eine laufende Bewegung sofort. **Kein Enqueue, direkt schreiben:**

```
U(7)a\r   →  0x55 0x07 'a' 0x0D
```

Danach keine Antwort abwarten. Steuerung hält innerhalb der laufenden
Rampe an. Für neue Bewegungen danach empfiehlt sich eine Referenzfahrt,
weil die Position nicht mehr präzise bekannt ist.

---

## 6. Positionsabfrage und Einheiten

### Abfrage der Absolutpositionen

```
UC\r → Antwort: ASCII-Zahl + 0x0D, z. B. "165801\r"
UD\r → Position Y
UE\r → Position Z
```

### Einheiten

Empirisch ermittelt am Walter Uhl MS4-WT02:

```
1 Unit = 1 µm = 0,001 mm
1000 Units/mm für alle drei Achsen (X, Y, Z)
```

**Umrechnung:**
```cpp
double posMm = rawUnits / 1000.0;
long   units = std::llround(posMm * 1000.0);
```

**Vorsicht:** In einem älteren WinCommander-Protokollmitschnitt (von einer
anderen, größeren Maschine mit ~330 mm Verfahrweg X) wurde der Faktor 10000
beobachtet. Das ist für den MS4-WT02 **falsch**. Immer maschinenseitig
prüfen: nach Calibrate muss der Positionswert aller Achsen `0` sein, nach
MeasureLength muss der Wert mit dem Typenschildwert übereinstimmen.

**Verfahrwege MS4-WT02:**
```
X: 165801 Units = 165,801 mm
Y: 166459 Units = 166,459 mm
Z: 104631 Units = 104,631 mm
```

### Präzise Umrechnung

Bei anderen Maschinen ist der `unitsPerMm`-Faktor aus dem Datenblatt oder
durch Messung zu ermitteln. Nie direkt aus einem fremden Sniffer-Log übernehmen.

---

## 7. Statusantworten und Endschalter

### Format der Status-Antwort (nach Bewegungsbefehlen)

Die Antwort auf `UP` nach einem Bewegungsbefehl ist ein 5-Byte-String:

```
Byte 1: X-Achse Schalterstatus
Byte 2: Y-Achse Schalterstatus
Byte 3: Z-Achse Schalterstatus
Byte 4: '-'
Byte 5: '.'
```

**Schalterwerte:**

| Zeichen | Hex  | Bedeutung                        |
|---------|------|----------------------------------|
| `@`     | 0x40 | Kein Schalter aktiv (Freilauf)   |
| `A`     | 0x41 | Nullschalter (Home) aktiv        |
| `D`     | 0x44 | Endschalter (Limit) aktiv        |

**Beispiele:**
- `"AAA-."` → alle drei Achsen am Nullschalter (nach Calibrate erwartet)
- `"DDD-."` → alle drei Achsen am Endschalter (nach MeasureLength erwartet)
- `"@@@-."` → alle Achsen frei (normale Bewegung beendet)

### Status-Register (UF)

Das Status-Register (Lese-Byte 70, Befehl `UF\r`) liefert eine
freitext-ähnliche Statusmeldung der Steuerung. Inhalt hängt vom
Steuerungszustand ab (z. B. "OK" bei Bereitschaft).

---

## 8. Koordinatensystem

### Hardware-Koordinatensystem (wie die Steuerung zählt)

Nach Calibrate setzt die Steuerung alle Achsen auf 0. Von dort aus
**wachsen** die Positionswerte in Richtung Endschalter:

```
Hardware-Nullpunkt: X=rechts, Y=hinten, Z=oben (Nullschalter-Position)
Wachstumsrichtung:  X wächst nach links
                    Y wächst nach vorne
                    Z wächst nach unten (!)
```

### Software-Koordinatensystem (wie die Applikation zeigt)

Für die Bedienung durch den Anwender ist ein intuitives, rechtshändiges
CNC-Koordinatensystem (ISO 841) erwünscht:

```
Software-Nullpunkt: X=links, Y=vorne, Z=oben (Nullschalter = Ausgangspunkt)
Wachstumsrichtung:  X wächst nach rechts  (+X = Tisch nach rechts)
                    Y wächst nach hinten  (+Y = Tisch nach hinten)
                    Z wächst nach oben    (Z ist im Betrieb negativ:
                                           Z=0 oben, Z=-104mm unten)
```

Das rechtshändige Kreuzprodukt X×Y=Z zeigt nach oben — konform mit
G-Code-Koordinatensystemen und der üblichen Laborkonvention.

### Koordinatentransformation

```cpp
// Gegeben: m_hwMax = Verfahrwege in mm (nach MeasureLength gemessen)
//          Für MS4-WT02: m_hwMax.x = 165.80, m_hwMax.y = 166.46, m_hwMax.z = 104.63

// Hardware → Software (für Positionsanzeige):
double swX = m_hwMax.x - hwX;   // Spiegeln
double swY = m_hwMax.y - hwY;   // Spiegeln
double swZ = -hwZ;              // Invertieren (Z oben = 0)

// Software → Hardware (für Bewegungsbefehle):
double hwX = m_hwMax.x - swX;
double hwY = m_hwMax.y - swY;
double hwZ = -swZ;
```

### Relative Bewegungen

Bei relativen Bewegungen muss das Delta ebenfalls transformiert werden.
Da beide Achsen gespiegelt sind, gilt:

```cpp
double deltaHw = -deltaSw;   // für alle drei Achsen gleich
```

### Verfahrbereiche in Software-Koordinaten

Nach Calibrate und MeasureLength:
```
X: 0 mm (links) bis +165.80 mm (rechts)
Y: 0 mm (vorne) bis +166.46 mm (hinten)
Z: 0 mm (oben)  bis -104.63 mm (unten)
```

---

## 9. Geschwindigkeit und Beschleunigung

Die Steuerung kennt zwei Regelgrößen:

| Register | Name  | Empfohlene Werte           | Einheit                         |
|----------|-------|----------------------------|---------------------------------|
| 9        | Speed | 50 normal, 100 bei Calibrate | Interne Einheit (5 mm/s / Unit?) |
| 8        | Ramp  | 500 normal, 50 bei Calibrate, 200 bei Z | Interne Einheit  |

**Faustregel:**
- Schnelle normale Bewegungen: Speed=50, Ramp=500
- Referenzfahrt (sanftes Anfahren der Schalter): Speed=100, Ramp=50
- Z-Achse (mechanische Bremse): Ramp=200 statt 500

**Z-Achsen-Bremse:** Die elektromagnetische Haltebremse der Z-Achse ist
über das MCL3-Protokoll nicht ansteuerbar. `SetExtValue` (ein WinCommander-
Befehl) erzeugt keine seriellen Bytes — er wird nur intern verarbeitet.
Als Kompensation wird für Z-Bewegungen eine weichere Rampe (200) verwendet,
damit beim Anlaufen der Bremse keine harten Erschütterungen entstehen.

---

## 10. Spindelsteigung (Pitch)

### Bedeutung

Die Steigung gibt an, wie weit sich die Achse pro Motorumdrehung bewegt.
Die Steuerung benötigt diesen Wert für ihre interne Drehzahl- und
Beschleunigungsberechnung.

### Einheit im Register

```
Registereinheit: 0,0001 mm pro Umdrehung
Beispiel: Steigung 2,0 mm → Registerwert 20000
Formel:   Registerwert = Steigung_mm × 10000
```

### Werte für Walter Uhl MS4-WT02

| Achse | Steigung | Registerwert |
|-------|----------|--------------|
| X     | 2,0 mm   | 20000        |
| Y     | 2,0 mm   | 20000        |
| Z     | 1,0 mm   | 10000        |

### Befehlsformat

```
U(21)<wert>\r   → Pitch X
U(22)<wert>\r   → Pitch Y
U(23)<wert>\r   → Pitch Z
```

Pitch-Werte werden in der Initialisierungssequenz gesetzt (Schritte 5–7)
und können danach jederzeit überschrieben werden.

---

## 11. Firmware-Version abfragen

Der Befehl ist ungewöhnlich: er verwendet das Command-Register und den
Start-Befehl wie ein Bewegungskommando, liefert aber keine Bewegung.

```
# Versionsnummer:
U(7)b\r  →  Sub-Kommando 'b'
UP\r     →  Start; Antwort z. B. "LS31.00.011\r"

# Build-Datum (separater zweiter Abfruf):
U(7)b1\r →  Sub-Kommando 'b1'
UP\r     →  Start; Antwort z. B. "2018-03-12\r"
```

**Besonderheit: Erster Aufruf nach dem Öffnen liefert `ERR 2`.**
Das ist normales Verhalten: Die Steuerung ist nach dem Reset noch nicht
vollständig synchronisiert. Die Initialisierungssequenz muss vorher
abgeschlossen sein. `ERR 2` ignorieren, beim zweiten Versuch funktioniert
die Abfrage.

---

## 12. Bekannte Fallstricke

Diese Punkte haben in der Praxis zu schwer zu findenden Fehlern geführt.

### 1. Zwei Stopbits sind Pflicht

Qt6 öffnet Ports standardmäßig mit einem Stopbit. Die LStep-Steuerung
kommuniziert ohne explizites `setStopBits(QSerialPort::TwoStop)` **nicht
zuverlässig** — Befehle werden sporadisch ignoriert oder die Verbindung
hängt nach dem ersten Befehl.

### 2. SoftwareControl zerstört HardwareControl

Sobald `setFlowControl(QSerialPort::SoftwareControl)` aufgerufen wird,
deaktiviert Qt intern den RTS/CTS-Hardwarepfad. Ergebnis: Steuerung sendet
keine Daten mehr. Niemals beide Modi mischen.

### 3. DTR und RTS müssen gesetzt werden

Nach `open()` müssen `setDataTerminalReady(true)` und `setRequestToSend(true)`
explizit aufgerufen werden. Ohne DTR startet die Steuerung die Kommunikation
nicht.

### 4. ActiveAxes vor jedem Bewegungsbefehl setzen

Die Steuerung merkt sich den letzten `ActiveAxes`-Wert. Wenn beim Start
noch ein alter Wert gesetzt ist, der eine Rotationsachse einschließt
(die beim MS4-WT02 nicht vorhanden ist), wartet die Steuerung nach
`Calibrate` dauerhaft und reagiert auf nichts mehr.

**Immer explizit vor Calibrate `U(11)7\r` senden.**

### 5. Befehlssequenz: immer auf Antwort warten

Wenn eine Antwort erwartet wird (d. h. nach `UP`), darf der nächste Befehl
erst gesendet werden, wenn die CR-terminierte Antwort vollständig empfangen
wurde. Zu früh gesendete Befehle werden von der Steuerung ignoriert.

### 6. Empfangspuffer korrekt verwalten

`readyRead()` kann mehrfach ausgelöst werden, bevor ein vollständiges Paket
ankommt. Bytes müssen in einem Puffer akkumuliert werden; der Puffer wird
erst bei CR verarbeitet.

```cpp
void onReadyRead() {
    m_buffer.append(m_port.readAll());
    int crIdx = m_buffer.indexOf('\r');
    if (crIdx < 0) return;                       // Noch kein vollständiges Paket
    QByteArray response = m_buffer.left(crIdx);
    m_buffer.remove(0, crIdx + 1);
    processResponse(response);
}
```

### 7. Pseudo-Terminals: NoFlowControl

Bei Verwendung von `socat` oder `mkpty` zum Testen ohne Hardware:
HardwareControl deaktivieren, sonst hängt Qt beim ersten Schreibversuch.

### 8. Einheitenfaktor maschinenseitig prüfen

Der Faktor `unitsPerMm` ist **nicht** aus fremden Sniffer-Protokollen
ableitbar. Auf einer anderen Maschine derselben Serie kann er abweichen.
Eigene Messung: nach Calibrate 100 mm fahren, gemeldeten Rohwert / 100.0
ergibt den Faktor.

### 9. `!cts 1` muss das allererste Byte sein

Dieser Befehl ist kein MCL3-Register-Befehl (kein `0x55`-Präfix).
Er muss als allererste Sequenz gesendet werden, noch vor `SetActiveAxes`.

---

## 13. Software-Architektur

### Bewährte Struktur für Qt-Projekte

```
LStepController  ─ reines QObject, kein Widget-Code
    │
    ├── QSerialPort m_port         – serielle Verbindung
    ├── QQueue<Pending> m_queue    – FIFO-Befehlswarteschlange
    ├── Pending m_current          – aktuell laufender Befehl
    ├── bool m_busy                – true wenn auf Antwort gewartet wird
    ├── QByteArray m_rxBuffer      – Empfangspuffer
    └── QTimer m_timeout           – Timeout-Wächter pro Befehl

MainWindow / Controller
    │
    └── LStepController* m_ctrl   – in eigenem QThread
```

### Befehlswarteschlange

Die Steuerung verarbeitet exakt **einen Befehl gleichzeitig**. Alle
geplanten Befehle werden in eine FIFO-Queue eingereiht. Die Queue wird
durch `pumpQueue()` abgearbeitet, das sich nach jeder Antwort selbst
per `QMetaObject::invokeMethod(... Qt::QueuedConnection)` wieder aufruft.

```
Zwei Befehlstypen:
  enqueueWrite(bytes)              – kein Response; weiter nach waitForBytesWritten
  enqueueRead(bytes, handler, ms)  – wartet auf CR-Antwort, ruft handler auf
  enqueueMove(subCmd, handler, ms) – Kurzform für enqueueWrite + enqueueRead(UP)
```

### QThread-Auslagerung

`LStepController` wird ohne Parent angelegt und per `moveToThread` in einen
Worker-Thread verschoben. Alle GUI→Controller-Aufrufe müssen über
`QMetaObject::invokeMethod` erfolgen:

```cpp
// GUI-Thread:
QMetaObject::invokeMethod(m_ctrl, [ctrl = m_ctrl, x, y, z] {
    ctrl->moveAbsolute(x, y, z);
});
```

Controller→GUI-Signale werden von Qt automatisch als `QueuedConnection`
verbunden, wenn Sender und Empfänger in verschiedenen Threads leben.

### Timeout-Strategie

| Befehlstyp               | Empfohlener Timeout |
|--------------------------|---------------------|
| Schreibbefehle (kein Resp)| 5 s (waitForBytesWritten) |
| Positionsabfrage         | 5 s                 |
| Normale Bewegung         | 30 s                |
| Calibrate / MeasureLength| 60 s                |

### Polling

Für kontinuierliche Positionsanzeige: `QTimer` mit 500 ms Intervall,
der `readPositions()` aufruft. **Wichtig:** Polling darf die Queue nicht
aufstauen, wenn die Steuerung beschäftigt ist.

```cpp
void LStepController::readPositionsPoll() {
    if (!m_busy && m_queue.isEmpty())  // isIdle()-Check, threadsicher
        readPositions();
}
```

---

## 14. Erweiterung: Scanner-Anwendung

### Grundprinzip

Ein Triangulations-Laser-Scanner mit dem MS4-WT02 könnte so aufgebaut sein:
- Messtisch trägt das Messobjekt (oder den Laser)
- Kamera (z. B. IDS uEye/Peak) nimmt Laserebene auf
- Pro Scan-Zeile: X-Achse auf Zielposition → Bild aufnehmen → nächste X-Position

### Integration `LStepController` ↔ Kamera

Die Klasse `LStepController` liefert zwei relevante Signale:

```cpp
// Nach jeder Positionsänderung (Polling oder nach Bewegung):
void positionsUpdated(double xMm, double yMm, double zMm);

// Nach Abschluss einer Bewegung (Status-Antwort der Steuerung):
void statusUpdated(const QString& status);
```

Für präzises Triggern: `statusUpdated` verwenden, weil es erst nach dem
bestätigten Ende der Bewegung emittiert wird. `positionsUpdated` ist
für Polling gedacht und kann noch während der Bewegung kommen.

### Trigger-Register (Register 35)

Die Steuerung hat ein Trigger-Register (Reg 35). Im aktuellen Projekt
ist es auf `0` (aus) gesetzt. Es könnte für Hardware-Trigger an der
Kamera genutzt werden — Dokumentation dazu ist im verfügbaren
Protokollmaterial aber lückenhaft. Vorläufig: Software-Trigger über
das `positionsUpdated`-Signal.

### Scan-Ablauf (Beispiel: Zeilenraster)

```
1. Calibrate XYZ
2. MeasureLength (einmalig, um Verfahrbereich zu kennen)
3. Für jede Y-Zeile y = y_start, y_start + step_y, ..., y_end:
     a. moveAbsolute(x_start, y, z_scan)
     b. Auf statusUpdated warten
     c. Für jede X-Position x = x_start, x_start + step_x, ..., x_end:
          i.  moveAbsolute(x, y, z_scan)   oder moveRelative(Axis_X, step_x)
          ii. Auf statusUpdated warten
          iii. Kamera-Trigger → Bild aufnehmen
4. Zurück zur Ausgangsposition
```

### Genauigkeit und Auflösung

- **Schrittweite minimal:** 1 Unit = 1 µm → theoretische Auflösung 1 µm
- **Reproduzierbarkeit:** abhängig vom Umkehrspiel der Spindeln
  (nicht per Software kompensierbar ohne Encoder-Feedback)
- **Empfehlung für Scanner:** Y-Achse immer in derselben Richtung abfahren
  (Schlittenfahrt ohne Richtungsumkehr pro Zeile), um Umkehrspiel zu
  eliminieren

### Parallelisierung (Kamera + Messtisch)

`LStepController` im Worker-Thread ermöglicht es, Kamera-Capture-Operationen
im GUI-Thread (oder einem dritten Thread) parallel zur Tischbewegung
vorzubereiten. Typisches Muster:

```
Worker-Thread (LStepController):
  moveAbsolute(x, y, z) enqueueRun
  ← statusUpdated() emittiert

GUI-Thread / Scan-Thread:
  [verbunden mit statusUpdated]
  → Kamera-Trigger auslösen
  → Bild in separatem Thread speichern
  → nächste Position an LStepController übergeben
```

### Empfohlene Erweiterungen in LStepController für Scanner

```cpp
// Neue Slots:
void moveScanLine(double yMm, double zMm,
                  double xStart, double xEnd, double xStep);

// Neues Signal nach jeder Teilbewegung im Scan:
void scanPositionReached(double xMm, double yMm, double zMm, int index);
```

Die Queue-Architektur erlaubt es, eine komplette Scan-Zeile als Folge
von `enqueueMove`-Aufrufen in einem Aufruf einzureihen. Jeder Einzelschritt
emittiert `statusUpdated` (über den vorhandenen Response-Handler); der
Scanner-Layer abonniert dieses Signal und löst den Kamera-Trigger aus.

---

## Anhang: Befehlsreferenz (kompakt)

```
Befehl            Bytes (Hex)                    Bedeutung
─────────────────────────────────────────────────────────────────────
!cts 1\r          21 63 74 73 20 31 0D           XON-Ankündigung (immer zuerst)
U(11)7\r          55 0B 37 0D                    Alle 3 Achsen aktivieren
U(9)50\r          55 09 35 30 0D                 Speed = 50 (normal)
U(8)500\r         55 08 35 30 30 0D              Ramp = 500 (normal)
U(9)100\r         55 09 31 30 30 0D              Speed = 100 (Calibrate)
U(8)50\r          55 08 35 30 0D                 Ramp = 50 (Calibrate)
U(8)200\r         55 08 32 30 30 0D              Ramp = 200 (Z-Achse)
U(21)20000\r      55 15 32 30 30 30 30 0D        Pitch X = 2,0 mm
U(22)20000\r      55 16 32 30 30 30 30 0D        Pitch Y = 2,0 mm
U(23)10000\r      55 17 31 30 30 30 30 0D        Pitch Z = 1,0 mm
U(0)<val>\r       55 00 <ASCII> 0D               Preselection X setzen
U(1)<val>\r       55 01 <ASCII> 0D               Preselection Y setzen
U(2)<val>\r       55 02 <ASCII> 0D               Preselection Z setzen
U(7)c\r + UP\r    55 07 63 0D  55 50 0D          Calibrate starten
U(7)l\r + UP\r    55 07 6C 0D  55 50 0D          MeasureLength starten
U(7)r\r + UP\r    55 07 72 0D  55 50 0D          MoveAbsolute starten
U(7)v\r + UP\r    55 07 76 0D  55 50 0D          MoveRelative starten
U(7)a\r           55 07 61 0D                    NOT-AUS (sofort)
U(7)b\r + UP\r    55 07 62 0D  55 50 0D          Firmware-Version
U(7)b1\r + UP\r   55 07 62 31 0D  55 50 0D       Firmware-Build-Datum
UC\r              55 43 0D                        Position X lesen
UD\r              55 44 0D                        Position Y lesen
UE\r              55 45 0D                        Position Z lesen
UF\r              55 46 0D                        Status lesen
UP\r              55 50 0D                        Start (Bewegung auslösen)
UK\r              55 4B 0D                        ActiveAxes lesen
```
