# 9. Électronique et câblage

> ⚠️ **Sécurité électrique.** L’alimentation de puissance (36 V continu, 10 A) peut provoquer brûlures et départs de feu
> en cas de court-circuit. Montez-la dans un boîtier fermé et protégez-la par un fusible. Ne branchez ni ne débranchez
> jamais un moteur pas-à-pas **sous tension** : la surtension détruit les drivers.

## Vue d’ensemble (ORION-6 MAKER)

```
                       ┌──────────────── ARRÊT D’URGENCE (2 contacts NF, à accrochage) ─────────────┐
 Secteur 230 V         │ NF1 : en série avec la bobine du contacteur        NF2 : entrée Teensy 30 │
    │                  │                                                                            │
 [Alim 36 V 10 A] ─ F10A ─┬─[Contacteur K1]──┬── +36 V puissance → 6 drivers (DM542T)       Teensy 30 ─┤
    │ 0 V                 │   (bobine via NF1 │                                                       GND ─┘
    │                     │    + bouton Marche)├── Buck 36→6 V 5 A → servo de pince (MG996R)
    │                     │                   │
    │                     └── Buck 36→5 V 3 A (non coupé par l’AU) → Teensy VIN*, capteurs Hall, 74HCT245
    │
    └── 0 V commun (drivers : entrées logiques ; Teensy : GND)          PC ── USB ── Teensy 4.1
```

\* Si la Teensy est alimentée à la fois par l’USB et par VIN, **coupez le pont VIN-VUSB** au dos de la carte (pastille
prévue par PJRC). Sinon le 5 V externe remonte dans l’USB du PC.

- L’**arrêt d’urgence coupe la puissance des moteurs** (catégorie 0) par le contacteur K1. La logique (Teensy) reste
  alimentée et **l’entrée 30 signale l’arrêt** au firmware : les drivers sont désactivés et le robot doit être
  référencé de nouveau.
- Le réarmement demande deux actions : déverrouiller le champignon, puis appuyer sur un bouton **Marche** (auto-maintien
  du contacteur). Le robot ne redémarre jamais seul.
- ⚠️ Moteurs coupés, le bras peut **retomber** sous son poids. Avant de manipuler, amenez-le en pose de repos (*Repos*,
  bras replié) ou soutenez-le.

## Brochage de la Teensy 4.1 (firmware `orion_fw`)

Le brochage par défaut est généré dans `orion_config.h`. Il est modifiable dans *Paramètres → Matériel & firmware*,
avant de recompiler.

| Fonction | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| STEP | 2 | 4 | 6 | 8 | 10 | 12 |
| DIR | 3 | 5 | 7 | 9 | 11 | 14 |
| Capteur d’origine | 24 | 25 | 26 | 27 | 28 | 29 |

| Fonction | Broche | Remarque |
|---|---|---|
| ENABLE (commun aux 6 drivers) | 15 | actif **bas** par défaut (drivers activés quand la broche est à 0) |
| Arrêt d’urgence (entrée) | 30 | contact NF vers GND ; niveau haut = arrêt (fil coupé compris) |
| Servo de pince (MLI 50 Hz) | 33 | signal seul ; le servo est alimenté par le buck 6 V |
| Entrées TOR DI0…DI7 | 16, 17, 18, 19, 20, 21, 22, 23 | actives bas, rappel interne au 3,3 V |
| Sorties TOR DO0…DO7 | 31, 32, 34, 35, 36, 37, 38, 39 | 3,3 V, 4 mA max : passez par un transistor ou un relais |
| LED d’état | 13 | lente = prêt, rapide = mouvement, fixe = défaut, éclair toutes les 2 s = moteurs coupés |

> ⚠️ Les broches de la Teensy 4.1 sont en **3,3 V et ne tolèrent pas le 5 V**. N’y reliez jamais directement une sortie
> 5 V ou 12 V.

## Drivers pas-à-pas

### DM542T (ou équivalent optocouplé : DM556, TB6600…)

Leurs entrées sont des optocoupleurs qui demandent 5 V environ 10 mA. Les 3,3 V de la Teensy sont souvent
insuffisants et donnent des pas manqués aléatoires. **Intercalez un tampon 74HCT245** alimenté en 5 V :

```
Teensy STEP/DIR/EN (3,3 V) → 74HCT245 (VCC = 5 V, DIR = 5 V, OE = GND) → PUL+ / DIR+ / ENA+ du driver
                                                                         PUL− / DIR− / ENA− → GND commun
```

Deux 74HCT245 suffisent pour les 13 signaux (6 STEP + 6 DIR + ENABLE).

Réglages par micro-interrupteurs :

| Axe | Moteur | Courant crête conseillé | Micro-pas |
|---|---|---|---|
| J1 | NEMA17 48 mm (2,0 A) | 1,8 A (≈ 90 %) | 16 (3 200 pas/tr) |
| J2 | NEMA23 76 mm (2,8 A) | 2,5 A | 16 |
| J3 | NEMA17 60 mm (2,1 A) | 1,9 A | 16 |
| J4, J5 | NEMA17 40 mm (1,7 A) | 1,5 A | 16 |
| J6 | NEMA17 34 mm (1,3 A) | 1,2 A | 16 |

- Activez la **réduction de courant au repos** (« half current ») : elle fait moins chauffer, et le maintien reste
  largement suffisant grâce à la réduction.
- Le signal ENABLE du DM542T **désactive** le driver quand l’optocoupleur conduit. Avec le câblage ci-dessus (ENA+ piloté,
  ENA− à la masse), un niveau haut désactive, d’où « ENABLE actif bas » dans les paramètres.
- Les temps imposés (DM542T : impulsion ≥ 2,5 µs, DIR ≥ 5 µs avant le front) sont garantis par le firmware : impulsion
  de 10 µs, DIR changé au moins 10 µs après le dernier front et 20 µs avant le suivant.

### TMC2209 / TMC5160 (modules « silencieux »)

Ils acceptent directement la logique 3,3 V : pas besoin de 74HCT245. Utilisez le mode STEP/DIR. Mettez le TMC5160 sur J2
(courant élevé) et des TMC2209 (2 A efficaces max) sur les autres axes. Le couple des TMC2209 à haute vitesse est un peu
plus faible : vérifiez les marges dans *Analyse → Dimensionnement*. Ne comptez pas sur le *StallGuard* pour la prise
d’origine, peu fiable avec la réduction et le frottement des cycloïdes.

### Moteurs

- Câble blindé 4 × 0,5 mm². Reliez le blindage à la masse **côté driver uniquement**, et éloignez ces câbles de ceux des
  capteurs.
- Si un axe tourne à l’envers, inversez **une seule** bobine (A+ ↔ A−), ou cochez *Inverser DIR* dans les paramètres
  (commande `SET invert`).

## Capteurs de prise d’origine

**Capteur à effet Hall A3144** et aimant néodyme Ø6 × 3 mm collé sur la partie mobile, face sud vers le capteur :

```
A3144 : VCC ← 5 V      GND ← GND      OUT → broche 24…29 de la Teensy (collecteur ouvert,
                                             rappel interne au 3,3 V activé par le firmware)
```

- Sortie à 0 quand l’aimant est présent, d’où *Capteur actif bas* coché par défaut.
- Positions des capteurs (angle articulaire au déclenchement), à régler ensuite par calibration :

  | J1 | J2 | J3 | J4 | J5 | J6 |
  |---|---|---|---|---|---|
  | −172° | −133° | +62° | −172° | −117° | −182° |

  Elles se trouvent 2° au-delà des butées logicielles : pendant la prise d’origine, l’axe va chercher le capteur, puis
  s’en dégage et revient dans ses butées.
- Le firmware filtre les rebonds (état stable 2 ms). Il met le robot **en défaut** si un capteur s’active pendant un
  mouvement normal, ou si la course de recherche est dépassée (capteur absent ou débranché).
- **Variante « sécurité » avec interrupteurs mécaniques à contact NF** : câblez l’interrupteur entre la broche et GND
  et décochez *Capteur actif bas*. Le contact est fermé (niveau bas) au repos. Il s’ouvre (niveau haut) quand
  l’interrupteur est actionné **ou si le fil est coupé**.
- ⚠️ Beaucoup de capteurs inductifs NPN (série NJK, LJ12A3…) ont une résistance de rappel interne vers leur tension
  d’alimentation (5–30 V). Ils ne doivent **pas** être reliés directement à la Teensy : utilisez un optocoupleur ou un
  pont diviseur.

## Pince

- Servo **MG996R** alimenté en **6 V** par un buck dédié : les appels de courant (jusqu’à 2,5 A au blocage)
  perturberaient sinon la logique. Masse commune avec la Teensy.
- Signal sur la broche 33 : MLI matérielle 50 Hz. Largeurs 900 µs (fermée) à 2 100 µs (ouverte), réglables
  (`SET grip_closed`, `SET grip_open`).

## Bus CAN (ORION-6 PRO, firmware `mit_bridge`)

```
Teensy 4.1           SN65HVD230 (3,3 V)                       moteurs DM4340 / DM4310
 22 (CTX1) ──────────► D
 23 (CRX1) ◄────────── R
 3,3 V ────────────── VCC        CAN_H ═══╤═══════╤═══════ … ═══╤══ [120 Ω]
 GND ──────────────── GND        CAN_L ═══╧═══════╧═══════ … ═══╧══
                                 [120 Ω]   J1      J2            J6
```

- **Transceiver 3,3 V** (SN65HVD230 ou TJA1051T/3). Le MCP2551, alimenté en 5 V, n’est pas compatible directement.
- Paire torsadée CAN_H / CAN_L en ligne (chaînage), **terminaison 120 Ω aux deux extrémités** (60 Ω mesurés entre
  CAN_H et CAN_L, alimentation coupée). Débit 1 Mbit/s.
- Identifiants moteurs 1 à 6 (commande), réglés avec l’outil du fabricant, qui fixe aussi l’identifiant maître et les
  plages P/V/T.
- Charge du bus : chaque trame de commande reçoit une réponse. 6 moteurs à 400 Hz occupent environ 60 % d’un bus à
  1 Mbit/s. Au-delà de 500 Hz, répartissez les moteurs sur deux bus (CAN1 et CAN2 de la Teensy).
- Puissance : 24 V, 15 A minimum pour 3 × DM4340 + 3 × DM4310, avec fusible. **L’arrêt d’urgence coupe le 24 V des
  moteurs** : sans alimentation, ils redeviennent réversibles.
- Arrêt d’urgence (entrée) : broche 30, comme pour la version pas-à-pas.

## Mise sous tension : ordre conseillé

1. Contrôle visuel : polarités, pas de fil nu, blindages, fusible.
2. Alimentation 36 V **sans** les moteurs : vérifiez 36 V, puis 5 V et 6 V en sortie des bucks.
3. Teensy sur USB seule : flashez, puis vérifiez `PING` et `STAT` depuis ORION Studio.
4. Testez l’arrêt d’urgence : `STAT` doit afficher `ESTOP` (drapeau 0x10) quand le champignon est enfoncé.
5. Actionnez chaque capteur à la main (aimant) : le drapeau 0x40 apparaît dans `STAT`.
6. Branchez les moteurs **hors tension**, réarmez, puis `EN 1` : les moteurs doivent se bloquer (couple de maintien).
7. Premiers mouvements à 10 % de vitesse avec `JG` (voir [12. Calibration](12-calibration.md)).
