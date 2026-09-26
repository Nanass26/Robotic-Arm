# 11. Firmware et protocole

Deux firmwares pour **Teensy 4.1**, construits de la même façon :

- un **cœur portable** en C++, sans aucune dépendance Arduino, testé sur PC avec un faux matériel ;
- une **couche matérielle** mince (`main.cpp`).

| Firmware | Dossier | Rôle |
|---|---|---|
| `orion_fw` | `firmware/orion_fw` | contrôleur pas-à-pas d’ORION-6 MAKER : génération de pas, prise d’origine, trajectoires, flux temps réel |
| `mit_bridge` | `firmware/mit_bridge` | pont USB ↔ CAN d’ORION-6 PRO : trames mode MIT, lissage des consignes, surveillance des moteurs |

## Flasher la Teensy

**Méthode la plus simple : fichier .hex prêt à l’emploi.**

1. Installez **Teensy Loader** (PJRC) ou Teensyduino.
2. *File → Open HEX File* : `firmware/bin/orion_fw-teensy41.hex` (ou `mit_bridge-teensy41.hex`).
3. Appuyez sur le bouton de la Teensy : le chargement est automatique.

Ces fichiers contiennent la configuration par défaut. Vos réglages (pas/°, butées, vitesses, prise d’origine…) s’envoient
ensuite **sans recompiler**, par le bouton *Envoyer la config* d’ORION Studio. Il envoie des commandes `SET` puis
`SAVE`, qui enregistre en EEPROM.

**Autres méthodes :**

| Outil | Procédure |
|---|---|
| IDE Arduino + Teensyduino | ouvrir `firmware/orion_fw/orion_fw.ino`. Carte *Teensy 4.1*, USB Type *Serial*, CPU 600 MHz, puis *Téléverser* |
| PlatformIO | dans `firmware/orion_fw` : `pio run -e teensy41 -t upload` |
| Ligne de commande (GCC ARM) | `npm run build:firmware`, puis Teensy Loader sur `firmware/bin/*.hex` |

Pour recompiler avec **votre** configuration par défaut : *Paramètres → Firmware .h* dans ORION Studio. Remplacez
`firmware/orion_fw/src/orion_config.h` par le fichier obtenu, puis recompilez.

**Tests sur PC** (aucune carte nécessaire) : `npm run test:firmware`, soit 126 vérifications pour `orion_fw` et 74 pour
`mit_bridge`. Le faux matériel simule :

- les drivers, avec contrôle des temps DIR/STEP ;
- les capteurs d’origine et l’arrêt d’urgence ;
- l’EEPROM ;
- six moteurs Damiao sur un faux bus CAN.

Ces tests couvrent le protocole, MoveJ, la prise d’origine avec position inconnue, le flux à 60 Hz irrégulier, les
sauts de consigne, le chien de garde, les défauts et la sauvegarde.

## Protocole ORION-ASCII (commun aux deux firmwares)

- Liaison **USB série** native de la Teensy. La vitesse est ignorée : n’importe quelle valeur convient, 115 200 par
  exemple.
- **Une commande par ligne**, terminée par `\n`. La casse du mot-clé est libre.
- **Somme de contrôle optionnelle** `*HH` en fin de ligne : XOR de tous les caractères qui précèdent `*`, en
  hexadécimal.
  - Une somme fausse donne `ERR 1`.
  - ORION Studio l’ajoute toujours ; à la main, on peut l’omettre.
- **Toutes les réponses** portent une somme de contrôle :

| Réponse | Forme | Sens |
|---|---|---|
| `OK …` | `OK PONG ORION-6 fw1.0.0 axes=6*78` | commande acceptée (+ données éventuelles) |
| `ERR code message` | `ERR 5 J2 hors butees (150.00)*25` | commande refusée |
| `EVT nom …` | `EVT DONE*67` | événement asynchrone |
| `ST …` | `ST HOLD 18342 0.000 0.000 0.000 0.000 90.000 0.000 03 00*0E` | état périodique |

Exemple de calcul : pour `PING`, 'P' ⊕ 'I' ⊕ 'N' ⊕ 'G' = 0x50 ⊕ 0x49 ⊕ 0x4E ⊕ 0x47 = 0x10, soit `PING*10`.

### Codes d’erreur

| Code | Sens |
|---|---|
| 1 | somme de contrôle incorrecte |
| 2 | commande inconnue |
| 3 | syntaxe ou argument invalide (nombre d’arguments, valeur hors plage, ligne trop longue) |
| 4 | commande interdite dans l’état actuel (moteurs coupés, défaut, prise d’origine requise, robot en mouvement) |
| 5 | consigne hors butées, ou début de flux trop éloigné de la position actuelle |
| 6 | (pont MIT) moteur muet sur le bus CAN |
| 7 | arrêt d’urgence actif |
| 8 | EEPROM indisponible ou configuration invalide |

### Ligne d’état `ST`

```
ST <état> <t_ms> <q1> … <q6> <drapeaux hex> <entrées hex>
```

- Les positions `q` sont en **degrés** articulaires. Pour `orion_fw`, ce sont les pas réellement émis, convertis en
  degrés. Pour `mit_bridge`, ce sont les positions mesurées par les codeurs.
- La fréquence d’envoi se règle avec `STREAM hz` (0 = arrêt, 500 max). `STAT` demande une ligne ponctuelle.

| Drapeau | Bit | Sens |
|---|---|---|
| enabled | 0x01 | drivers / moteurs actifs |
| homed | 0x02 | robot référencé |
| moving | 0x04 | mouvement en cours |
| fault | 0x08 | défaut (voir `FLT`) |
| estopInput | 0x10 | entrée d’arrêt d’urgence active |
| watchdog | 0x20 | le chien de garde du flux a déclenché |
| limit | 0x40 | un capteur de fin de course est actif (pont MIT : axe hors butées) |
| underrun | 0x80 | le tampon de flux s’est vidé (consignes en retard) |
| limited | 0x100 | le limiteur vitesse/accélération a écrêté la consigne |

### États

| État | Sens | Pour en sortir |
|---|---|---|
| `IDLE` | drivers coupés | `EN 1` |
| `READY` | drivers actifs, **non référencé** | `HOME` ou `ZERO` (ou `JG` pour de petits déplacements) |
| `HOMING` | prise d’origine en cours | fin automatique → `HOLD`, ou `STOP` |
| `RUN` | mouvement (`MJ`, `JG`, flux `SP`) | fin → `HOLD`, `STOP` → arrêt contrôlé |
| `HOLD` | référencé, à l’arrêt, prêt | `MJ`, `SP`, `JG`, `HOME`… |
| `FAULT` | défaut (capteur atteint, prise d’origine échouée…) | `CLR` |
| `ESTOP` | arrêt d’urgence (entrée ou commande `ESTOP`) | relâcher l’AU, `CLR`, `EN 1`, `HOME` |

Couper les drivers (`EN 0`, arrêt d’urgence) **fait perdre la référence**. Moteurs libres, la position n’est plus
garantie.

## Commandes du firmware pas-à-pas (`orion_fw`)

| Commande | Effet | Réponse |
|---|---|---|
| `PING` | identification | `OK PONG ORION-6 fw1.0.0 axes=6` |
| `VER` | version | `OK VER 1.0.0` |
| `STAT` | une ligne d’état | `ST …` |
| `STREAM hz` | lignes `ST` périodiques (0 = arrêt) | `OK STREAM 50` |
| `FLT` | dernier message de défaut | `OK FLT fin de course J3` |
| `EN 1` / `EN 0` | active / coupe les drivers | `OK EN 1` |
| `HOME [masque]` | prise d’origine (masque optionnel, bit 0 = J1 : `HOME 0x3F` = tous) | `OK HOME 0x3F`, puis `EVT HOMED_AXIS n`…, `EVT HOMED`, `EVT DONE` |
| `ZERO` | déclare la position actuelle comme **pose Home** (sans capteurs) | `OK ZERO` |
| `JG axe delta` | déplacement relatif d’un axe (°, \|delta\| ≤ 30), possible sans référence | `OK MJ durée`, puis `EVT DONE` |
| `MJ T q1 … q6` | MoveJ vers q (°), durée T (s) ; T = 0 → au plus vite | `OK MJ durée`, puis `EVT DONE` |
| `SP seq t q1 … q6 [v1 … v6]` | consigne du **flux temps réel** (voir plus bas) | aucune (sauf erreur) |
| `STOP` | arrêt contrôlé (décélération max) | `OK STOP`, puis `EVT STOPPED` |
| `ESTOP` | arrêt d’urgence logiciel : drivers coupés | `OK ESTOP` |
| `CLR` | acquitte un défaut ou un arrêt d’urgence (entrée relâchée) | `OK CLR` |
| `GRIP pct` | pince : 0 = fermée, 100 = ouverte | `OK GRIP 50` |
| `DO n v` | sortie TOR n (0–7) à v (0/1) | `OK DO 3 1` |
| `DI` | lecture des 8 entrées (hex) | `OK DI 42` |
| `SET clé axe valeur` | règle un paramètre (axe 1–6 ou `*`) | `OK SET vmax 2 45` |
| `GET clé axe` | lit un paramètre | `OK 45` |
| `SAVE` / `LOAD` / `DEFAULTS` | EEPROM : enregistrer / recharger / valeurs d’usine | `OK SAVE` |

### Paramètres `SET` / `GET` (`orion_fw`)

| Clé | Unité | Rôle |
|---|---|---|
| `steps_per_deg` | pas/° | (360/angle de pas) × micro-pas × réduction / 360. Le changement conserve la position angulaire |
| `min`, `max` | ° | butées logicielles |
| `vmax`, `amax` | °/s, °/s² | limites de vitesse et d’accélération |
| `invert` | 0/1 | inversion du sens (câblage) |
| `home_dir` | −1/+1 | sens de recherche du capteur |
| `home_pos` | ° | angle articulaire au déclenchement du capteur (calibration) |
| `home_speed`, `home_slow` | °/s | vitesse de recherche rapide, puis d’approche fine |
| `home_backoff` | ° | dégagement après détection |
| `home_pose` | ° | pose atteinte après la prise d’origine, et pose déclarée par `ZERO` |
| `home_en` | 0/1 | axe équipé d’un capteur |
| `watchdog` (`*`) | ms | chien de garde du flux (défaut 250) |
| `stream_delay` (`*`) | ms | retard du tampon de lissage (défaut 40) |
| `grip_closed`, `grip_open` (`*`) | µs | impulsions servo fermée / ouverte |

### Mouvement

- **Génération de pas** : accumulateur de phase (DDA) à **100 kHz**, 45 000 pas/s max par axe. Impulsion STEP de
  10 µs. Le changement de DIR est appliqué entre deux impulsions : ≥ 10 µs après le dernier front, ≥ 20 µs avant le
  suivant.
- **Boucle 1 kHz** : calcule la consigne de chaque axe puis la vitesse de pas nécessaire pour l’atteindre en 1 ms.
- **MoveJ (`MJ`)** : profil trapézoïdal synchronisé (25 % accélération, 50 % vitesse constante, 25 % décélération).
  Durée = max(T, max_axes(Δ/(0,75·vmax), √(Δ/(0,1875·amax)))). Tous les axes arrivent ensemble.
- **Limiteur de sécurité** : toute consigne passe par un limiteur de vitesse (1,05·vmax) et d’accélération (1,5·amax).
  Le rattrapage est borné par la distance de freinage (v ≤ √(2·a·écart)). Un saut de consigne est donc rejoint **sans
  dépassement**, ce que vérifie un test avec un saut de 60°.
- **Butées** : les consignes sont écrêtées aux butées logicielles dès que le robot est référencé. `MJ` hors butées est
  refusé (`ERR 5`).

### Flux temps réel `SP` (jumeau numérique)

```
SP <seq> <t_ms> <q1> … <q6> [<v1> … <v6>]
```

- `seq` : numéro de séquence sur 16 bits, qui reboucle. Les doublons et les consignes périmées sont ignorés.
- `t_ms` : horodatage de l’**émetteur** en millisecondes, sur 16 bits, qui reboucle.
- `q` en degrés, `v` en °/s (facultatives).

Le firmware ne suppose **aucune cadence fixe** :

1. Les consignes sont rangées dans un tampon (32 places) avec leur horodatage d’origine.
2. La lecture démarre quand le tampon contient `stream_delay` ms d’avance (40 ms par défaut).
3. L’horloge de lecture avance de 1 ms par ms, avec une correction lente (±5 % au plus) pour garder cette avance
   constante. Elle absorbe la gigue de l’USB et du navigateur, ainsi que la dérive des horloges.
4. Entre deux consignes, l’interpolation est **cubique d’Hermite** si les vitesses sont fournies, linéaire sinon.
5. **Démarrage** : la première consigne doit être à moins de 5° de la consigne actuelle (`ERR 5` sinon). Utilisez
   *Synchroniser* dans ORION Studio, ou un `MJ` préalable.
6. **Chien de garde** : sans nouvelle consigne pendant `watchdog` ms, le firmware émet `EVT WATCHDOG` puis s’arrête en
   douceur (`EVT STOPPED`).

Sur le banc de test (émission à 55–67 Hz, gigue d’arrivée de 0 à 8 ms, sinusoïdes de 25° d’amplitude), l’erreur
maximale mesurée entre la position réelle et la trajectoire émise, retardée du tampon, est de **0,52°**.

### Prise d’origine (`HOME`)

Les axes sont référencés un par un, dans l’ordre configuré : par défaut **J5, J6, J4, J3, J2, J1**. On replie d’abord
le poignet pour réduire l’encombrement. Pour chaque axe :

1. S’il est déjà sur son capteur, il s’en dégage (`home_backoff`).
2. **Approche rapide** vers le capteur (`home_speed`, sens `home_dir`).
3. **Recul** de `home_backoff` jusqu’à libérer le capteur.
4. **Approche lente** (`home_slow`) : au front du capteur, la position de l’axe devient `home_pos`.
5. **Dégagement** de `home_backoff`, pour revenir dans les butées et libérer le capteur.

Quand tous les axes sont faits : `EVT HOMED`, puis MoveJ vers `home_pose` et `EVT DONE`.

Défauts possibles :

- capteur introuvable (course > débattement + 20°) ;
- capteur bloqué (> 45° sans libération) ;
- capteur perdu pendant l’approche lente.

La répétabilité dépend de l’approche lente : à 2°/s, avec un anti-rebond de 2 ms, l’incertitude est d’environ 0,004°
à la sortie.

### Sécurité

- **Entrée d’arrêt d’urgence** (broche 30) : lue à chaque milliseconde, avec 2 ms d’anti-rebond. Elle coupe les
  drivers, passe en `ESTOP` et émet `EVT ESTOP`. La référence est perdue.
- **Capteur de fin de course actif** pendant `RUN` ou `HOLD` : arrêt immédiat, `EVT FAULT fin de course Jn`.
- **Ligne trop longue** (> 199 caractères) : ignorée, avec `ERR 3`.
- **Configuration EEPROM** protégée par un CRC32 et un numéro de version. Une configuration corrompue est ignorée au
  démarrage, qui reprend alors les valeurs par défaut.
- **Écriture série non bloquante** : si l’hôte ne lit pas, les lignes sont abandonnées, sans jamais bloquer la boucle
  temps réel.

### Exemple de session

```text
> PING                         < OK PONG ORION-6 fw1.0.0 axes=6
> EN 1                         < OK EN 1
> HOME                         < OK HOME 0x3F
                               < EVT HOMED_AXIS 5 … EVT HOMED_AXIS 1
                               < EVT HOMED
                               < OK MJ 1.912
                               < EVT DONE
> MJ 2 30 -20 40 0 70 0        < OK MJ 2.000
                               < EVT DONE
> GRIP 0                       < OK GRIP 0
> STAT                         < ST HOLD 61234 30.000 -20.000 40.000 0.000 70.000 0.000 03 00
```

## Pont CAN mode MIT (`mit_bridge`)

Le pont relie ORION Studio, ou tout programme de pilotage, aux six moteurs QDD. Il envoie une trame MIT par moteur à
`ctrl_hz` (400 Hz par défaut) et reçoit les retours : position, vitesse, couple, températures, code d’erreur. Les
positions échangées avec l’hôte sont **articulaires, en degrés**. La conversion vers le repère de chaque moteur se fait
dans le pont : p_moteur = sens·(q − décalage), en radians.

| Commande | Effet |
|---|---|
| `PING`, `VER`, `STAT`, `FLT`, `STREAM hz` | comme `orion_fw` (les lignes `ST` contiennent les positions **mesurées**) |
| `FB` | retours détaillés : `OK FB v/τ/T_mos/T_rotor/err` pour chaque axe (°/s, N·m, °C) |
| `EN 1` | active les moteurs (trame 0xFC) et **maintient** la position mesurée (Kp, Kd, τ = 0). Refusé si un moteur n’a pas répondu (`ERR 6`) |
| `EN 0` | **amortissement** pendant 1,5 s (Kp = 0, Kd = `kd_damp`), puis coupure (0xFD) |
| `DAMP` | amortissement maintenu (le bras se laisse déplacer lentement) |
| `MC seq t q1…q6 [v1…v6 [τ1…τ6]]` | flux de consignes : positions (°), vitesses (°/s), couples d’anticipation (N·m) |
| `STOP` | fige la dernière consigne (maintien avec le dernier couple d’anticipation) |
| `ESTOP` | coupure immédiate des moteurs (0xFD) |
| `CLR` | efface les erreurs moteur (0xFB). Défaut → maintien de la position actuelle ; arrêt d’urgence → `IDLE` |
| `ZERO` | la position actuelle devient la **pose de référence** `ref_pose` (pose de repos par défaut). Calcule les décalages logiciels ; `SAVE` pour les garder |
| `MZERO masque` | écrit le zéro dans les moteurs (0xFE), moteurs coupés uniquement |
| `SET`/`GET`/`SAVE`/`LOAD`/`DEFAULTS` | paramètres (ci-dessous) |

**Comportement** :

- **Moteurs coupés** (`IDLE`) : le pont interroge les moteurs à 50 Hz par des trames à gains et couple nuls. Les
  positions restent donc lisibles, en *Miroir* dans ORION Studio par exemple.
- **Démarrage du flux** : la première consigne doit être à moins de 5° de la position mesurée. Un **raccord de 300 ms**
  fait passer en douceur la position et le couple du maintien au flux, sans à-coup de gravité.
- **Lissage** : même tampon horodaté et même interpolation d’Hermite que `orion_fw`. La vitesse envoyée au moteur est la
  dérivée de l’interpolant. Le couple d’anticipation est interpolé linéairement.
- **Chien de garde** (`watchdog`, 250 ms par défaut) : maintien sur la dernière consigne **avec le dernier couple de
  gravité**. Le bras reste en place au lieu de retomber.
- **Défauts → amortissement** (Kp = 0, Kd = `kd_damp`, dernier couple d’anticipation conservé). Ils sont déclenchés
  par :
  - un moteur muet depuis plus de `fb_timeout` ms ;
  - un code d’erreur moteur : surtension, sous-tension, surintensité, surchauffe MOSFET ou bobinage, perte de
    communication, surcharge ;
  - une température ≥ `max_temp` ;
  - une position hors butées de plus de 5°.
- **Bornes** : consignes écrêtées aux butées, vitesse bornée à `vmax`, couple d’anticipation borné à `tau_max`.

| Clé `SET` | Unité | Rôle |
|---|---|---|
| `kp`, `kd` | N·m/rad, N·m·s/rad | gains MIT, limités à `kp_max` / `kd_max` |
| `kd_damp` | N·m·s/rad | amortissement en défaut ou arrêt |
| `dir`, `offset` | ±1, ° | repère : q = dir·p_moteur + offset (moteurs coupés) |
| `min`, `max`, `vmax`, `tau_max` | °, °/s, N·m | bornes |
| `ref_pose` | ° | pose déclarée par `ZERO` |
| `can_id` | — | identifiant CAN du moteur (moteurs coupés) |
| `p_max`, `v_max`, `t_max`, `kp_max`, `kd_max` | rad, rad/s, N·m… | plages d’encodage des trames, **identiques aux réglages des moteurs** |
| `watchdog`, `stream_delay`, `ctrl_hz`, `fb_timeout`, `max_temp` (`*`) | ms, ms, Hz, ms, °C | temporisations et seuils |

## Piloter le robot depuis un script (Python)

ORION Studio n’est pas obligatoire. N’importe quel programme peut parler au firmware, par exemple une IA qui planifie
des gestes dans l’esprit des démonstrations robotiques de GPT-6 Astra :

```python
import time, math, serial   # pip install pyserial

def line(cmd):
    c = 0
    for ch in cmd.encode():
        c ^= ch
    return f"{cmd}*{c:02X}\n".encode()

s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.05)   # Windows : "COM5"
for cmd in ["PING", "EN 1", "HOME"]:
    s.write(line(cmd))
# … attendre « EVT DONE » (lire s.readline()) …

# Flux à 100 Hz : balancement de J1 de ±20° autour de la pose Home
t0, seq = time.monotonic(), 0
while time.monotonic() - t0 < 10:
    t = time.monotonic() - t0
    q = [20 * math.sin(0.5 * t) * min(t, 1.0), 0, 0, 0, 90, 0]
    ms = int((time.monotonic() * 1000)) & 0xFFFF
    s.write(line("SP %d %d %s" % (seq & 0xFFFF, ms, " ".join("%.3f" % v for v in q))))
    seq += 1
    time.sleep(0.01)
```

La rampe `min(t, 1.0)` fait démarrer le flux sur la position actuelle, condition du démarrage. Si le script s’arrête, le
chien de garde immobilise le robot en douceur.
