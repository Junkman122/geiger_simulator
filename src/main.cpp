/*
  Geiger click simulator
  Target board: Arduino Pro Mini (ATmega328P, 5V/16MHz oder 3.3V/8MHz)
                läuft auch unverändert auf Uno/Nano

  Programmierung: externen USB-Serial-Adapter (FTDI) verwenden.

  Hardware:
    - Poti  -> A0
    - Buzzer-> D8     (bei 3.3V Variante: Transistor-Treiber empfohlen)
    - 10 LEDs Bar:    Pins {2,3,4,5,6,7,9,10,11,12}
                      Anode -> Vorwiderstand -> Pin, Kathoden -> GND
                      5V Variante:  220 Ω
                      3.3V Variante: ~100 Ω
                      Farben: 0..2 grün, 3..6 gelb, 7..9 rot
    - Onboard-LED auf D13 als Click-Indikator
                      (auf manchen Pro-Mini-Klonen nicht bestückt)
    - A1 als Entropy-Quelle für randomSeed

  Verhalten:
    - Click-Rate exponentiell mit Poti: MIN_CPS .. MAX_CPS
    - Drift: ±10 % sinusförmig, Periode 15 min (auf der Click-Rate)
    - Bar-Display 0..8: gefüllter Balken, Skala basiert auf baseRate (drift-frei)
    - LED 9: Vollausschlag-Anzeige bei baseRate = MAX_DISPLAY_CPS
             (= Poti ganz aufgedreht). Im Vollausschlag blinken alle LEDs.
    - Jitter: ganzer Balken endet bei (displayValue + offset), gecappt bei 8

  Hinweis: Bei displayValue = 0 leuchtet LED 0 als „Gerät aktiv"-Indikator.
*/

#include <Arduino.h>

// Vorwärts-Deklarationen (nötig in reinem C++ statt .ino-Sketch)
int  computeDisplayValue(float r);
void displayFilledBar(int d, int offset);
void playStartup();

// --- Pins ---
const int POT_PIN     = A0;
const int ENTROPY_PIN = A1;
const int BUZZER_PIN  = 8;
const int LED_ONBOARD = 13;

// 10 LEDs für Bar-Display (D8 und D13 sind belegt -> Lücke bei 8)
// Farbzuordnung: Index 0..2 grün, 3..6 gelb, 7..9 rot
const int LED_PINS[10] = { 2, 3, 4, 5, 6, 7, 9, 10, 11, 12 };

// --- Click-Rate ---
const float MAX_CPS = 100.0;
const float MIN_CPS = 0.5;

// --- Click-Form ---
const unsigned int CLICK_FREQ_HZ     = 2000;
const unsigned int CLICK_DURATION_MS = 3;
const unsigned int LED_ON_MS         = 8;

// --- Drift (langfristige Modulation) ---
const unsigned long DRIFT_PERIOD_MS = 15UL * 60UL * 1000UL;  // 15 min
const float         DRIFT_AMPLITUDE = 0.10;                  // ±10 %

// --- Display-Mapping ---
const float RED_THRESHOLD_CPS = 15.0;   // ab hier leuchtet LED 7 (erste rote)
const float MAX_DISPLAY_CPS   = 100.0;  // bei dieser Rate -> LED 9 (Vollausschlag)

// --- Jitter ---
const int           JITTER_THRESHOLD  = 1;
const unsigned long JITTER_UPDATE_MS  = 80;
const float         JITTER_PROB       = 0.6;
const int           JITTER_MAX_OFFSET = 1;

// --- Blink bei Vollausschlag ---
const unsigned long BLINK_INTERVAL_MS = 300;

// --- Vollausschlag-Erkennung am rohen ADC (Hysterese, robust gegen Rauschen) ---
const int POT_OVERLOAD_HIGH = 1015;  // ab hier in Blink-Modus wechseln
const int POT_OVERLOAD_LOW  = 990;   // erst hierunter wieder zurück

// --- Laufzeit-Zustand ---
unsigned long nextClickMillis  = 0;
unsigned long ledOffMillis     = 0;
bool          ledOnboardOn     = false;

unsigned long lastJitterMillis = 0;
int           lastJitterOffset = 0;

unsigned long lastBlinkMillis  = 0;
bool          blinkState       = false;


void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_ONBOARD, OUTPUT);
  pinMode(POT_PIN, INPUT);
  for (int i = 0; i < 10; ++i) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
  randomSeed(analogRead(ENTROPY_PIN));

  // Startup-Ritual: Sweep der LEDs synchron zur Hymne.
  // Läuft jedes Mal beim Power-On (also auch nach jedem Powerbank-Reset).
  playStartup();
}


// Aufwärm-Effekt für den Geigerzähler: Selbsttest mit kontinuierlicher
// Rampe der Klick-Rate von 0 bis MAX und wieder zurück. Die LED-Bar
// folgt der Rate synchron, sodass der Bediener sehen UND hören kann,
// dass der gesamte Anzeigebereich funktioniert.
//
// Verlauf:
//   Phase 1 (0..2 s):   Klick-Rate steigt 0 → 100 CPS, LEDs zünden 0 → 10
//   Phase 2 (2..4 s):   Klick-Rate sinkt 100 → 0 CPS, LEDs erlöschen 10 → 0
//   Phase 3 (4..4,6 s): kurzer Bestätigungs-Flash aller LEDs ("Ready")
//
// Im Gegensatz zum normalen Klick-Modus sind die Klicks hier regelmäßig
// (gleichmäßiger Abstand), nicht Poisson-verteilt. Das macht den
// "Self-Test"-Charakter eindeutig hörbar — eine zufällig streuende
// Rampe würde optisch und akustisch verschwimmen.
void playStartup() {
  const unsigned long SWEEP_MS     = 1000;   // Dauer auf bzw. ab
  const float         MAX_TEST_CPS = 60.0;  // Spitzenrate beim Wendepunkt
  const float         MIN_AUDIBLE_CPS = 0.5; // unterhalb -> gar keine Klicks

  // ===== Phase 1: Aufwärts =====
  unsigned long sweepStart  = millis();
  unsigned long nextClickMs = sweepStart;
  int           ledsOn      = 0;

  while (millis() - sweepStart < SWEEP_MS) {
    unsigned long elapsed = millis() - sweepStart;
    float fraction = (float)elapsed / SWEEP_MS;   // 0.0 .. 1.0

    // LED-Synchronisation: zünde alle LEDs, deren Index < fraction * 10
    int targetLedsOn = (int)(fraction * 10.0);
    while (ledsOn < targetLedsOn && ledsOn < 10) {
      digitalWrite(LED_PINS[ledsOn], HIGH);
      ++ledsOn;
    }

    // Klick-Erzeugung: Intervall ergibt sich aus aktueller Rate
    float rate = MAX_TEST_CPS * fraction;
    if (rate >= MIN_AUDIBLE_CPS) {
      unsigned long intervalMs = (unsigned long)(1000.0 / rate);
      if (intervalMs < 1) intervalMs = 1;

      // Cap: nextClickMs darf nie weiter in der Zukunft liegen als 2x
      // das aktuelle Intervall. Sonst kann die rampende Rate nicht
      // durchschlagen, wenn das letzte Intervall mit einer noch
      // niedrigeren Rate berechnet wurde.
      if (nextClickMs > millis() + 2 * intervalMs) {
        nextClickMs = millis() + intervalMs;
      }

      if (millis() >= nextClickMs) {
        tone(BUZZER_PIN, CLICK_FREQ_HZ, CLICK_DURATION_MS);
        nextClickMs = millis() + intervalMs;
      }
    }
  }
  // Sicherstellen, dass am Ende von Phase 1 wirklich alle LEDs leuchten
  for (int i = 0; i < 10; ++i) digitalWrite(LED_PINS[i], HIGH);

  // ===== Phase 2: Abwärts =====
  sweepStart  = millis();
  nextClickMs = sweepStart;
  ledsOn      = 10;

  while (millis() - sweepStart < SWEEP_MS) {
    unsigned long elapsed = millis() - sweepStart;
    float fraction = 1.0 - (float)elapsed / SWEEP_MS;  // 1.0 .. 0.0

    // LED-Synchronisation: schalte LEDs ab, deren Index >= fraction * 10
    int targetLedsOn = (int)(fraction * 10.0);
    while (ledsOn > targetLedsOn && ledsOn > 0) {
      --ledsOn;
      digitalWrite(LED_PINS[ledsOn], LOW);
    }

    // Klick-Erzeugung wie oben, aber mit fallender Rate
    float rate = MAX_TEST_CPS * fraction;
    if (rate >= MIN_AUDIBLE_CPS) {
      unsigned long intervalMs = (unsigned long)(1000.0 / rate);
      if (intervalMs < 1) intervalMs = 1;

      // Cap analog zur Aufwärts-Phase (Symmetrie der Logik).
      if (nextClickMs > millis() + 2 * intervalMs) {
        nextClickMs = millis() + intervalMs;
      }

      if (millis() >= nextClickMs) {
        tone(BUZZER_PIN, CLICK_FREQ_HZ, CLICK_DURATION_MS);
        nextClickMs = millis() + intervalMs;
      }
    }
  }
  // Sicherstellen, dass alle LEDs aus sind
  for (int i = 0; i < 10; ++i) digitalWrite(LED_PINS[i], LOW);
  noTone(BUZZER_PIN);

  // ===== Phase 3: Bestätigungs-Flash =====
  delay(200);
  for (int i = 0; i < 10; ++i) digitalWrite(LED_PINS[i], HIGH);
  delay(400);
  for (int i = 0; i < 10; ++i) digitalWrite(LED_PINS[i], LOW);
}


// Display-Wert 0..9 aus Click-Rate (drift-frei: baseRate).
// floor() sorgt dafür, dass v=9 erst bei r >= MAX_DISPLAY_CPS erreicht wird.
int computeDisplayValue(float r) {
  if (r >= MAX_DISPLAY_CPS) return 9;

  int v;
  if (r >= RED_THRESHOLD_CPS) {
    // [15..100) -> [7..9), also v in {7, 8}
    float f = 7.0 + ((r - RED_THRESHOLD_CPS)
                    / (MAX_DISPLAY_CPS - RED_THRESHOLD_CPS)) * 2.0;
    v = (int)floor(f);
  } else {
    // [0..15) -> [0..7)
    float f = r * (7.0 / RED_THRESHOLD_CPS);
    v = (int)floor(f);
  }
  if (v < 0) v = 0;
  if (v > 8) v = 8;  // 9 ist Vollausschlag, Bar-Bereich endet bei 8
  return v;
}


// Gefüllter Balken von 0 bis (d + offset). Top wird bei 8 gecappt,
// damit LED 9 (Vollausschlag) durch Jitter nicht ungewollt aktiviert wird.
void displayFilledBar(int d, int offset) {
  int top = d + offset;
  if (top < -1) top = -1;   // -1 erlaubt: alle aus
  if (top > 8)  top = 8;
  for (int i = 0; i < 10; ++i) {
    digitalWrite(LED_PINS[i], (i <= top) ? HIGH : LOW);
  }
}


void loop() {
  unsigned long now = millis();

  // --- Poti lesen, Rate berechnen ---
  int   pot     = analogRead(POT_PIN);             // 0..1023
  float potNorm = pot / 1023.0;
  // geometrische Interpolation: pot=0 -> MIN_CPS, pot=max -> MAX_CPS
  float baseRate = MIN_CPS * pow(MAX_CPS / MIN_CPS, potNorm);

  // Drift (sinusförmig um 1.0, ±DRIFT_AMPLITUDE) - wirkt nur auf Klick-Rate
  float phase       = (now % DRIFT_PERIOD_MS) / (float)DRIFT_PERIOD_MS;
  float driftFactor = 1.0 + DRIFT_AMPLITUDE * sin(2.0 * PI * phase);
  float rate        = baseRate * driftFactor;
  if (rate < MIN_CPS) rate = MIN_CPS;

  // Display basiert auf baseRate (drift-frei)
  int displayValue = computeDisplayValue(baseRate);

  // Vollausschlag-Erkennung mit Hysterese direkt am ADC-Wert
  static bool overload = false;
  if (overload) {
    if (pot < POT_OVERLOAD_LOW) overload = false;
  } else {
    if (pot >= POT_OVERLOAD_HIGH) overload = true;
  }

  // --- Anzeige ---
  if (overload) {
    // Vollausschlag: alle LEDs blinken im Takt
    if (now - lastBlinkMillis >= BLINK_INTERVAL_MS) {
      lastBlinkMillis = now;
      blinkState = !blinkState;
    }
    for (int i = 0; i < 10; ++i) {
      digitalWrite(LED_PINS[i], blinkState ? HIGH : LOW);
    }
  } else {
    // Normal: gefüllter Balken mit Jitter
    if (now - lastJitterMillis >= JITTER_UPDATE_MS) {
      lastJitterMillis = now;
      int newOffset = 0;
      if (displayValue >= JITTER_THRESHOLD &&
          random(0, 1000) < (int)(JITTER_PROB * 1000)) {
        newOffset = random(-JITTER_MAX_OFFSET, JITTER_MAX_OFFSET + 1);
      }
      lastJitterOffset = newOffset;
    }
    displayFilledBar(displayValue, lastJitterOffset);
  }

  // --- Click erzeugen (Poisson-Intervalle) ---
  // Bevor wir auf nextClickMillis warten: kürzen wir die Wartezeit, falls sie
  // deutlich länger ist, als die aktuelle Rate erwarten lässt. Das passiert,
  // wenn das Intervall mit einer alten (niedrigeren) Rate berechnet wurde und
  // der User dann das Poti hochgedreht hat. Ohne diesen Cap müsste man bis
  // zum Ende des alten, langen Intervalls warten, bevor der Klick-Rhythmus
  // sich an die neue Rate anpasst.
  // Der Faktor 3 ist ein Kompromiss: groß genug, dass die natürliche
  // Streuung der Poisson-Intervalle erhalten bleibt; klein genug, dass das
  // Aufdrehen sich gefühlt sofort auswirkt.
  unsigned long maxWaitMs = (unsigned long)(3000.0 / rate);
  if (nextClickMillis > now + maxWaitMs) {
    nextClickMillis = now + maxWaitMs;
  }

  if (now >= nextClickMillis) {
    long  rint        = random(1, 1000000);
    float u           = rint / 1000000.0;
    float interval_ms = (-log(u) / rate) * 1000.0;
    nextClickMillis   = now + (unsigned long)interval_ms;

    tone(BUZZER_PIN, CLICK_FREQ_HZ, CLICK_DURATION_MS);
    digitalWrite(LED_ONBOARD, HIGH);
    ledOnboardOn  = true;
    ledOffMillis  = now + LED_ON_MS;
  }

  // --- Onboard-LED non-blocking ausschalten ---
  if (ledOnboardOn && now >= ledOffMillis) {
    digitalWrite(LED_ONBOARD, LOW);
    ledOnboardOn = false;
  }
}
