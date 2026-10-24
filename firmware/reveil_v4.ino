// Réveil enfants — v4
// v3 : heure d'été/hiver automatique, réveil semaine / week-end, mode sieste.
// v4 : mesure de la batterie (D0, pont 220k/220k), alerte batterie faible le matin,
//      mise à jour sans fil par Bluetooth, publicité Bluetooth plus économe.
// Menu Outils > Partition Scheme : « Minimal SPIFFS (1.9MB APP with OTA) » (place pour deux programmes).
// Carte : Seeed XIAO ESP32-C3
// Câblage (LED à cathode commune) :
//   D3 (GPIO5) -- 330 ohm -- R
//   D2 (GPIO4) -- 100 ohm -- G
//   D1 (GPIO3) -- 100 ohm -- B
//   GND -------------------- cathode (patte la plus longue)
//
// Fonctionnement normal (le "jour du réveil" est demain si on a passé
// l'heure du coucher, sinon aujourd'hui ; samedi/dimanche = week-end) :
//   coucher -> (réveil - durée orange) : rouge
//   (réveil - durée orange) -> réveil  : orange
//   réveil -> (réveil + durée vert)     : vert
//   le reste de la journée              : éteint
// Sieste : rouge, orange les 10 dernières minutes, puis vert 30 min.
//          Bouton d'arrêt : pendant le rouge -> orange 5 min puis vert ;
//          pendant l'orange -> vert tout de suite ; pendant le vert -> fin.
// Heure inconnue (après coupure de courant) : flash bleu toutes les 5 s.
// Batterie faible (< 3,70 V, environ 15 %) : pendant le vert du matin, bref éclair orange toutes les 3 s.
// Mise à jour en cours : bleu doux.
//
// Protocole Bluetooth (une caractéristique, texte, une paire "cle=valeur" par écriture) :
//   t  = heure universelle (UTC), secondes depuis 1970
//   z  = fuseau, partie fixe POSIX (ex. "STD-1DST")
//   zs = règle de début d'heure d'été (ex. "M3.5.0"), vide si aucune
//   ze = règle de fin d'heure d'été (ex. "M10.5.0/3"), vide si aucune
//   c  = coucher (min depuis minuit)
//   rs = réveil semaine (min)        rw = réveil week-end (min)
//   o  = durée orange (min)          v  = durée vert (min)
//   lr / lo / lv = luminosité rouge / orange / vert (0..100)
//   to = teinte de l'orange (part de vert, 0..100)
//   n  = sieste : durée en minutes ; 0 = étape suivante (rouge -> orange 5 min -> vert -> fin)
//   p  = aperçu 5 s : r, o, v ou 0
//   s  = 1 : enregistre les réglages
//   u  = mise à jour : taille du fichier (début), "fin" (vérifie et redémarre), "annule"
//        les données du programme s'écrivent sur la 2e caractéristique (OTA_UUID)
// Lecture : "t,c,rs,rw,o,v,lr,lo,lv,etat,to,sieste,version,mv,bas,maj,recu"
//   (sieste = secondes restantes avant la fin, négatif pendant le vert, 0 = pas de sieste ;
//    mv = tension batterie en millivolts ; bas = 1 si batterie faible ;
//    maj = 0 rien, 1 en cours, 2 réussie, -1 erreur ; recu = octets reçus)

#include <BLEDevice.h>
#include <BLEServer.h>
#include <Preferences.h>
#include <Update.h>
#include <sys/time.h>
#include <time.h>

const int PIN_R = 5;  // D3
const int PIN_G = 4;  // D2
const int PIN_B = 3;  // D1
const int PIN_BAT = 2;  // D0 (A0) : milieu du pont 220k / 220k depuis BAT+
const int VERSION = 4;
const int BAT_BAS_MV = 3700, BAT_OK_MV = 3800, BAT_ABSENTE_MV = 3000;

const int PWM_BITS = 12;
const int PWM_MAX = 4095;

const int SIESTE_ORANGE_MIN = 10;
const int SIESTE_VERT_MIN = 30;
const int SIESTE_ARRET_ORANGE_MIN = 5;  // orange après un arrêt pendant le rouge

#define SERVICE_UUID "6f1c0001-3b7a-4c55-9a2e-7d1e5a0c0001"
#define CHAR_UUID    "6f1c0002-3b7a-4c55-9a2e-7d1e5a0c0001"
#define OTA_UUID     "6f1c0003-3b7a-4c55-9a2e-7d1e5a0c0001"

struct Config {
  uint16_t coucher = 19 * 60 + 30;
  uint16_t reveilSemaine = 7 * 60;
  uint16_t reveilWeekend = 7 * 60 + 45;
  uint16_t dureeOrange = 30;
  uint16_t dureeVert = 60;
  uint8_t lumRouge = 5;
  uint8_t lumOrange = 15;
  uint8_t lumVert = 30;
  uint8_t teinteOrange = 35;   // part de vert dans l'orange, en % du rouge
  char tzBase[24] = "CET-1CEST";   // Europe/Zurich par défaut
  char tzDebut[20] = "M3.5.0";
  char tzFin[20] = "M10.5.0/3";
} cfg;

enum Etat { ETEINT, ROUGE, ORANGE, VERT, INCONNU, MAJ };

Preferences prefs;
BLECharacteristic* carac = nullptr;
int apercu = -1;
unsigned long finApercu = 0;
time_t siesteFin = 0;  // 0 = pas de sieste
float actuel[3] = {0, 0, 0};

// batterie
float batMv = 0;          // tension filtrée
bool batBasse = false;
unsigned long prochaineMesure = 0;

// mise à jour sans fil
volatile int majEtat = 0;     // 0 rien, 1 en cours, 2 réussie, -1 erreur
volatile size_t majRecu = 0;
size_t majTaille = 0;
unsigned long redemarrage = 0;

// ---------- Heure et fuseau ----------

void appliquerFuseau() {
  String tz = cfg.tzBase;
  if (strlen(cfg.tzDebut) && strlen(cfg.tzFin)) {
    tz += ","; tz += cfg.tzDebut; tz += ","; tz += cfg.tzFin;
  }
  setenv("TZ", tz.c_str(), 1);
  tzset();
  Serial.println("Fuseau : " + tz);
}

bool heureConnue() { return time(nullptr) > 1700000000; }

int mod1440(int x) { return ((x % 1440) + 1440) % 1440; }

bool dans(int m, int a, int b) {  // m dans [a, b[ en gérant minuit
  a = mod1440(a);
  b = mod1440(b);
  if (a == b) return false;
  if (a < b) return m >= a && m < b;
  return m >= a || m < b;
}

Etat etatSieste() {
  long reste = (long)(siesteFin - time(nullptr));
  if (reste > SIESTE_ORANGE_MIN * 60L) return ROUGE;
  if (reste > 0) return ORANGE;
  if (reste > -SIESTE_VERT_MIN * 60L) return VERT;
  siesteFin = 0;  // sieste terminée
  return ETEINT;
}

Etat etatCourant() {
  if (majEtat == 1) return MAJ;
  if (apercu >= 0 && millis() < finApercu) return (Etat)apercu;
  apercu = -1;
  if (siesteFin) {
    Etat e = etatSieste();
    if (siesteFin) return e;
  }
  if (!heureConnue()) return INCONNU;

  time_t t = time(nullptr);
  struct tm tm;
  localtime_r(&t, &tm);  // heure locale, été/hiver compris
  int m = tm.tm_hour * 60 + tm.tm_min;

  // jour du prochain réveil : demain si on a dépassé le coucher
  int jour = (m >= cfg.coucher) ? (tm.tm_wday + 1) % 7 : tm.tm_wday;
  bool weekend = (jour == 0 || jour == 6);
  int w = weekend ? cfg.reveilWeekend : cfg.reveilSemaine;

  if (dans(m, w, w + cfg.dureeVert)) return VERT;
  if (dans(m, w - cfg.dureeOrange, w)) return ORANGE;
  if (dans(m, cfg.coucher, w - cfg.dureeOrange)) return ROUGE;
  return ETEINT;
}

// ---------- Batterie ----------

void mesurerBatterie() {
  uint32_t somme = 0;
  for (int i = 0; i < 16; i++) somme += analogReadMilliVolts(PIN_BAT);
  float mv = somme / 16.0f * 2.0f;                  // pont diviseur par 2
  batMv = (batMv == 0) ? mv : batMv + (mv - batMv) * 0.2f;
  if (batMv < BAT_ABSENTE_MV) batBasse = false;     // pas de batterie (ou mesure impossible)
  else if (batMv < BAT_BAS_MV) batBasse = true;
  else if (batMv > BAT_OK_MV) batBasse = false;     // hystérésis : il faut remonter à 3,80 V
}

// ---------- Mémoire ----------

void charger() {
  prefs.begin("reveil", false);
  if (prefs.getBytesLength("cfg3") == sizeof(cfg)) {
    prefs.getBytes("cfg3", &cfg, sizeof(cfg));
  }
}

void enregistrer() {
  prefs.putBytes("cfg3", &cfg, sizeof(cfg));
  Serial.println("Réglages enregistrés");
}

// ---------- Bluetooth ----------

String statut() {
  long sieste = siesteFin ? (long)(siesteFin - time(nullptr)) : 0;
  char buf[160];
  snprintf(buf, sizeof(buf), "%lu,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,%ld,%d,%d,%d,%d,%u",
           (unsigned long)time(nullptr), cfg.coucher, cfg.reveilSemaine,
           cfg.reveilWeekend, cfg.dureeOrange, cfg.dureeVert, cfg.lumRouge,
           cfg.lumOrange, cfg.lumVert, (int)etatCourant(), cfg.teinteOrange,
           sieste, VERSION, (int)batMv, batBasse ? 1 : 0, (int)majEtat, (unsigned)majRecu);
  return String(buf);
}

void copier(char* dest, size_t taille, const String& val) {
  strncpy(dest, val.c_str(), taille - 1);
  dest[taille - 1] = 0;
}

void appliquer(const String& cle, const String& val) {
  long n = val.toInt();
  if (cle == "t") {
    struct timeval tv = {(time_t)n, 0};
    settimeofday(&tv, nullptr);
    Serial.println("Heure synchronisée");
  }
  else if (cle == "z")  { copier(cfg.tzBase, sizeof(cfg.tzBase), val); appliquerFuseau(); }
  else if (cle == "zs") { copier(cfg.tzDebut, sizeof(cfg.tzDebut), val); appliquerFuseau(); }
  else if (cle == "ze") { copier(cfg.tzFin, sizeof(cfg.tzFin), val); appliquerFuseau(); }
  else if (cle == "c")  cfg.coucher = mod1440(n);
  else if (cle == "rs") cfg.reveilSemaine = mod1440(n);
  else if (cle == "rw") cfg.reveilWeekend = mod1440(n);
  else if (cle == "o")  cfg.dureeOrange = constrain(n, 0, 180);
  else if (cle == "v")  cfg.dureeVert = constrain(n, 0, 600);
  else if (cle == "lr") cfg.lumRouge = constrain(n, 0, 100);
  else if (cle == "lo") cfg.lumOrange = constrain(n, 0, 100);
  else if (cle == "lv") cfg.lumVert = constrain(n, 0, 100);
  else if (cle == "to") cfg.teinteOrange = constrain(n, 0, 100);
  else if (cle == "n") {
    time_t maintenant = time(nullptr);
    if (n > 0) {
      siesteFin = maintenant + constrain(n, 1, 300) * 60;
      Serial.println("Sieste démarrée");
    } else if (siesteFin && siesteFin - maintenant > SIESTE_ORANGE_MIN * 60L) {
      siesteFin = maintenant + SIESTE_ARRET_ORANGE_MIN * 60;  // pendant le rouge : orange 5 min
      Serial.println("Sieste arrêtée, orange 5 min");
    } else if (siesteFin && siesteFin > maintenant) {
      siesteFin = maintenant - 1;  // pendant l'orange : vert tout de suite
      Serial.println("Passage au vert");
    } else {
      siesteFin = 0;               // arrêt pendant le vert : fin de la sieste
      Serial.println("Sieste terminée");
    }
  }
  else if (cle == "s") enregistrer();
  else if (cle == "u") {
    if (val == "fin") {
      if (majEtat == 1 && majRecu == majTaille && Update.end(true)) {
        majEtat = 2;
        redemarrage = millis() + 1500;      // laisse le temps au téléphone de lire le résultat
        Serial.println("Mise à jour réussie, redémarrage");
      } else {
        Update.abort();
        majEtat = -1;
        Serial.printf("Mise à jour refusée (%u / %u octets) : %s\n", (unsigned)majRecu, (unsigned)majTaille, Update.errorString());
      }
    } else if (val == "annule") {
      Update.abort();
      majEtat = 0;
    } else if (n > 0) {
      if (Update.isRunning()) Update.abort();
      majRecu = 0;
      majTaille = n;
      majEtat = Update.begin(n) ? 1 : -1;
      Serial.printf("Mise à jour : %ld octets, %s\n", n, majEtat == 1 ? "début" : Update.errorString());
    }
  }
  else if (cle == "p") {
    apercu = (val == "r") ? ROUGE : (val == "o") ? ORANGE : (val == "v") ? VERT : -1;
    finApercu = millis() + 5000;
  }
}

class CaracCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    int start = 0;
    while (start < (int)v.length()) {  // accepte aussi "a=1;b=2"
      int fin = v.indexOf(';', start);
      if (fin < 0) fin = v.length();
      String paire = v.substring(start, fin);
      int eg = paire.indexOf('=');
      if (eg > 0) appliquer(paire.substring(0, eg), paire.substring(eg + 1));
      start = fin + 1;
    }
  }
  void onRead(BLECharacteristic* c) override { c->setValue(statut().c_str()); }
};

class MajCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (majEtat != 1) return;
    size_t n = c->getLength();
    if (majRecu + n > majTaille || Update.write(c->getData(), n) != n) {
      Update.abort();
      majEtat = -1;
      Serial.println("Mise à jour : erreur d'écriture");
      return;
    }
    majRecu += n;
  }
};

class ServeurCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override { Serial.println("Téléphone connecté"); }
  void onDisconnect(BLEServer* s) override {
    Serial.println("Téléphone déconnecté");
    if (majEtat == 1) { Update.abort(); majEtat = -1; }   // transfert interrompu : on garde l'ancien programme
    BLEDevice::startAdvertising();
  }
};

// ---------- LED ----------

void ecrireLed(float r, float g, float b) {
  float v[3] = {r, g, b};
  int pins[3] = {PIN_R, PIN_G, PIN_B};
  for (int i = 0; i < 3; i++) {
    int duty = (int)(v[i] * v[i] * PWM_MAX + 0.5f);  // courbe quadratique
    if (v[i] > 0.001f && duty < 1) duty = 1;
    ledcWrite(pins[i], duty);
  }
}

void cible(Etat e, float out[3]) {
  out[0] = out[1] = out[2] = 0;
  switch (e) {
    case ROUGE: out[0] = cfg.lumRouge / 100.0f; break;
    case ORANGE: {
      float l = cfg.lumOrange / 100.0f;
      out[0] = l;
      // racine carrée : après la courbe quadratique de ecrireLed, le vert vaut
      // exactement teinteOrange % du rouge (0 = rouge pur, 100 = jaune)
      out[1] = l * sqrtf(cfg.teinteOrange / 100.0f);
      break;
    }
    case VERT:
      if (batBasse && (millis() % 3000) < 300) {     // batterie faible : éclair orange
        float l = cfg.lumVert / 100.0f;
        out[0] = l; out[1] = l * sqrtf(cfg.teinteOrange / 100.0f);
      } else out[1] = cfg.lumVert / 100.0f;
      break;
    case MAJ: out[2] = 0.12f; break;
    case INCONNU: out[2] = ((millis() / 250) % 20 == 0) ? 0.15f : 0; break;
    default: break;
  }
}

// ---------- Programme ----------

void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  delay(500);
  Serial.println("Réveil enfants v4");

  ledcAttach(PIN_R, 5000, PWM_BITS);
  ledcAttach(PIN_G, 5000, PWM_BITS);
  ledcAttach(PIN_B, 5000, PWM_BITS);
  ecrireLed(0, 0, 0);

  analogSetPinAttenuation(PIN_BAT, ADC_11db);
  mesurerBatterie();

  charger();
  appliquerFuseau();

  BLEDevice::init("Reveil enfants");
  BLEDevice::setMTU(185);
  BLEServer* serveur = BLEDevice::createServer();
  serveur->setCallbacks(new ServeurCallbacks());
  BLEService* service = serveur->createService(SERVICE_UUID);
  carac = service->createCharacteristic(
      CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  carac->setCallbacks(new CaracCallbacks());
  carac->setValue(statut().c_str());
  BLECharacteristic* maj = service->createCharacteristic(
      OTA_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  maj->setCallbacks(new MajCallbacks());
  service->start();

  BLEAdvertising* pub = BLEDevice::getAdvertising();
  pub->addServiceUUID(SERVICE_UUID);
  pub->setScanResponse(true);
  pub->setMinInterval(400);   // publicité toutes les 250 à 500 ms (unités de 0,625 ms) : moins de courant
  pub->setMaxInterval(800);
  BLEDevice::startAdvertising();
  Serial.println("Bluetooth actif");
}

void loop() {
  if (redemarrage && millis() > redemarrage) ESP.restart();
  if (millis() > prochaineMesure) {
    mesurerBatterie();
    prochaineMesure = millis() + 10000;
  }
  float c[3];
  Etat e = etatCourant();
  cible(e, c);
  bool eclair = (e == INCONNU) || (e == VERT && batBasse);
  float k = eclair ? 1.0f : 0.03f;  // fondu doux (sauf clignotements)
  for (int i = 0; i < 3; i++) actuel[i] += (c[i] - actuel[i]) * k;
  ecrireLed(actuel[0], actuel[1], actuel[2]);
  delay(20);
}
