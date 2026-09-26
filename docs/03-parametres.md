<!-- FICHIER GÉNÉRÉ par tools/gen-docs.mjs depuis studio/core/schema.js et studio/core/defaults.js — ne pas modifier à la main. -->


# 3. Référence de tous les paramètres

ORION Studio expose **179 paramètres** (soit **567 valeurs** en comptant chaque articulation). Ils sont tous modifiables en direct dans l’onglet **Paramètres** (champ de recherche, aide au survol de `?`), enregistrés dans le fichier de configuration `JSON` et, pour ceux qui concernent le matériel, exportés vers le firmware (`orion_config.h` ou commandes `SET`).

- Les valeurs sont stockées en unités SI (m, rad, kg, s) et **affichées** dans les unités du tableau (mm, °, g·cm²…).
- La colonne *Valeur* donne le réglage du préréglage **ORION-6 MAKER** (pas-à-pas + réducteurs cycloïdaux imprimés) ; lorsque le préréglage **ORION-6 PRO** (moteurs QDD en mode MIT) diffère, sa valeur est indiquée en italique.
- Pour les tableaux par articulation, les valeurs sont données dans l’ordre J1 · J2 · J3 · J4 · J5 · J6.
- La validation borne automatiquement toute valeur hors plage (import JSON compris) et signale la correction dans le journal.

## Sommaire

- [Général](#général)
- [Cinématique (Denavit-Hartenberg)](#cinématique-denavit-hartenberg)
- [Limites & performances](#limites--performances)
- [Dynamique (masses & inerties)](#dynamique-masses--inerties)
- [Actionneurs (moteurs & réducteurs)](#actionneurs-moteurs--réducteurs)
- [Commande (PID, mode MIT, couple calculé)](#commande-pid-mode-mit-couple-calculé)
- [Trajectoires & modèle inverse](#trajectoires--modèle-inverse)
- [Pince / outil](#pince--outil)
- [Sécurité](#sécurité)
- [Simulation](#simulation)
- [Matériel & firmware](#matériel--firmware)
- [Poses nommées](#poses-nommées)

## Général

Identité du robot et variante matérielle.

| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |
|---|---|---|---|---|---|
| Nom du robot | `meta.name` |  |  | « ORION-6 » | Nom affiché dans l’interface et utilisé pour les exports (URDF, MJCF, firmware). *PRO : « ORION-6 PRO »* |
| Variante matérielle | `meta.variant` |  | `maker`, `pro`, `custom` | MAKER — NEMA17/23 + réducteurs cycloïdaux imprimés | Change uniquement l’étiquette et les valeurs proposées par les préréglages. Tous les paramètres restent modifiables individuellement. *PRO : PRO — actionneurs brushless QDD en mode MIT (CAN)* |
| Description | `meta.description` |  |  | « Bras 6 axes imprimé en 3D — NEMA17/23 + réducteurs cycloïdaux imprimés, Teensy 4.1. » | Notes libres enregistrées avec la configuration. *PRO : « Variante à actionneurs brushless QDD pilotés en mode MIT sur bus CAN (type Damiao DM4340 / DM4310), PD + compensation de gravité. »* |

## Cinématique (Denavit-Hartenberg)

Géométrie du bras : table DH, repère de base et outil (TCP). Ces paramètres définissent le modèle géométrique direct et inverse.

| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |
|---|---|---|---|---|---|
| Convention DH | `kinematics.convention` |  | `DH`, `MDH` | DH standard (Denavit-Hartenberg classique) | Standard : Aᵢ = Rz(θᵢ)·Tz(dᵢ)·Tx(aᵢ)·Rx(αᵢ), l’axe de l’articulation i est zᵢ₋₁. Modifiée (Craig) : Aᵢ = Rx(αᵢ₋₁)·Tx(aᵢ₋₁)·Rz(θᵢ)·Tz(dᵢ), l’axe i est zᵢ. Le modèle inverse analytique n’est disponible qu’en DH standard ; en DH modifiée le solveur numérique est utilisé. |
| **Table DH** — tableau par articulation (J1 · J2 · … · J6) | `kinematics.joints[i]` | | | | Une ligne par articulation (J1 = base … J6 = poignet). Unités d’affichage : mm et degrés. |
| ↳ Nom | `.name` |  |  | « J1 Base » · « J2 Épaule » · « J3 Coude » · « J4 Avant-bras » · « J5 Poignet » · « J6 Bride » | Nom de l’axe (affiché dans l’interface). |
| ↳ Type | `.type` |  | `revolute`, `prismatic` | Rotoïde (tous) | Rotoïde : θ varie. Prismatique : d varie (glissière). |
| ↳ a — longueur | `.a` | mm | -2000 … 2000 | 0 · 240 · 0 · 0 · 0 · 0 | Distance entre les axes zᵢ₋₁ et zᵢ mesurée le long de xᵢ (DH standard). |
| ↳ α — torsion | `.alpha` | ° | -180 … 180 | -90 · 0 · -90 · 90 · -90 · 0 | Angle entre zᵢ₋₁ et zᵢ autour de xᵢ. |
| ↳ d — décalage | `.d` | mm | -2000 … 2000 | 160 · 0 · 0 · 220 · 0 · 33,2 | Distance entre xᵢ₋₁ et xᵢ le long de zᵢ₋₁. |
| ↳ θ₀ — décalage zéro | `.thetaOffset` | ° | -180 … 180 | 0 · -90 · 0 · 0 · 0 · 0 | Décalage entre la valeur articulaire q et l’angle DH : θ = sens·q + θ₀. Permet de choisir la pose « zéro » mécanique. |
| ↳ Sens (+1/−1) | `.direction` |  | -1 … 1 | 1 (tous) | Inverse le sens positif de l’axe sans changer la géométrie (θ = sens·q + θ₀). |
| Base — X | `kinematics.base.x` | mm | -5000 … 5000 | 0 | Position du repère de base du robot dans le monde (ex. table). |
| Base — Y | `kinematics.base.y` | mm | -5000 … 5000 | 0 |  |
| Base — Z | `kinematics.base.z` | mm | -5000 … 5000 | 0 | Hauteur de la base au-dessus du plan de travail. |
| Base — roulis (Rx) | `kinematics.base.roll` | ° | -180 … 180 | 0 | Orientation de la base (montage mural/plafond : ±90° / 180°). |
| Base — tangage (Ry) | `kinematics.base.pitch` | ° | -180 … 180 | 0 |  |
| Base — lacet (Rz) | `kinematics.base.yaw` | ° | -180 … 180 | 0 |  |
| Outil — nom | `kinematics.tool.name` |  |  | « Pince parallèle » | Nom de l’outil monté sur la bride (pince, ventouse, stylo…). |
| TCP — X | `kinematics.tool.x` | mm | -1000 … 1000 | 0 | Position du point outil (TCP) dans le repère de la bride (repère 6). |
| TCP — Y | `kinematics.tool.y` | mm | -1000 … 1000 | 0 |  |
| TCP — Z | `kinematics.tool.z` | mm | -1000 … 1000 | 100,3 | Longueur de l’outil le long de l’axe de la bride. |
| TCP — Rx | `kinematics.tool.roll` | ° | -180 … 180 | 0 |  |
| TCP — Ry | `kinematics.tool.pitch` | ° | -180 … 180 | 0 |  |
| TCP — Rz | `kinematics.tool.yaw` | ° | -180 … 180 | 0 |  |

## Limites & performances

Butées logicielles, vitesses, accélérations et jerks maximaux (articulaires et cartésiens).

| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |
|---|---|---|---|---|---|
| **Limites articulaires** — tableau par articulation (J1 · J2 · … · J6) | `limits.joints[i]` | | | |  |
| ↳ Butée min | `.min` | ° | -360 … 360 | -170 · -130 · -170 · -170 · -115 · -180 | Butée logicielle basse. Doit rester à l’intérieur de la butée mécanique. |
| ↳ Butée max | `.max` | ° | -360 … 360 | 170 · 130 · 60 · 170 · 115 · 180 | Butée logicielle haute. |
| ↳ Vitesse max | `.vmax` | °/s | 0,6 … 2864,8 | 120 · 90 · 110 · 170 · 170 · 200 | Vitesse articulaire maximale utilisée par le planificateur (100 % de vitesse). *PRO : 150 · 120 · 150 · 360 · 360 · 360* |
| ↳ Accélération max | `.amax` | °/s² | 1 … 28 648 | 300 · 200 · 300 · 600 · 600 · 700 | Accélération articulaire maximale. *PRO : 400 · 300 · 400 · 900 · 900 · 900* |
| ↳ Jerk max | `.jmax` | °/s³ | 6 … 2 864 789 | 3000 · 2000 · 3000 · 6000 · 6000 · 7000 | Dérivée de l’accélération (profil en S). Plus faible = mouvements plus doux, moins de vibrations. *PRO : 4000 · 3000 · 4000 · 9000 · 9000 · 9000* |
| Vitesse TCP max | `limits.cartesian.vmax` | mm/s | 1 … 5000 | 250 | Vitesse linéaire maximale du point outil pour les mouvements linéaires/circulaires (MoveL/MoveC). |
| Accélération TCP max | `limits.cartesian.amax` | mm/s² | 10 … 50 000 | 1000 |  |
| Jerk TCP max | `limits.cartesian.jmax` | mm/s³ | 100 … 1 000 000 | 10 000 |  |
| Vitesse angulaire outil max | `limits.cartesian.wmax` | °/s | 0,6 … 1145,9 | 90 |  |
| Accélération angulaire outil max | `limits.cartesian.alphamax` | °/s² | 1 … 11 459 | 360 |  |
| Boîte de sécurité active | `limits.workspace.enabled` |  | oui / non | non | Refuse tout mouvement dont le TCP sortirait de la boîte définie ci-dessous (repère monde). |
| Boîte — X min | `limits.workspace.xmin` | mm | -5000 … 5000 | -700 |  |
| Boîte — X max | `limits.workspace.xmax` | mm | -5000 … 5000 | 700 |  |
| Boîte — Y min | `limits.workspace.ymin` | mm | -5000 … 5000 | -700 |  |
| Boîte — Y max | `limits.workspace.ymax` | mm | -5000 … 5000 | 700 |  |
| Boîte — Z min | `limits.workspace.zmin` | mm | -5000 … 5000 | 5 | Mettre légèrement au-dessus de la table pour protéger l’outil. |
| Boîte — Z max | `limits.workspace.zmax` | mm | -5000 … 5000 | 900 |  |
| Hauteur du plan de travail | `limits.floorZ` | mm | -2000 … 2000 | 0 | Altitude de la table dans le repère monde, utilisée pour la détection de collision avec le sol. |

## Dynamique (masses & inerties)

Paramètres inertiels des segments, de l’outil et de la charge, frottements. Utilisés par la simulation, la compensation de gravité, le couple calculé et le dimensionnement. Valeurs par défaut calculées depuis la CAO (PLA, remplissage indiqué) + moteurs + visserie.

| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |
|---|---|---|---|---|---|
| Gravité | `dynamics.gravity` | m/s² |  | 0 / 0 / -9,81 | Vecteur gravité dans le repère monde (par défaut [0, 0, −9,81]). |
| **Segments** — tableau par articulation (J1 · J2 · … · J6) | `dynamics.links[i]` | | | |  |
| ↳ Masse | `.mass` | kg | 0 … 50 | 1,808 · 1,211 · 0,744 · 0,708 · 0,52 · 0,049 | Masse du segment i (pièces imprimées + moteur/stator porté + visserie). *PRO : 0,711 · 0,755 · 0,585 · 0,551 · 0,413 · 0,12* |
| ↳ CdG x | `.com.0` | mm | -1000 … 1000 | 0 · -58,21 · 0 · 0 · 0 · 0 | Centre de gravité exprimé dans le repère DH i du segment. *PRO : 0 · -98,95 · 0 · 0 · 0 · 0* |
| ↳ CdG y | `.com.1` | mm | -1000 … 1000 | 4,13 · -0 · -5,74 · -31,16 · -8,57 · 0 |  *PRO : 26,85 · -0 · -26,51 · -69,65 · -34,37 · 0* |
| ↳ CdG z | `.com.2` | mm | -1000 … 1000 | -11,32 · 83,66 · 22,7 · 72,99 · -3,93 · -9,88 |  *PRO : -18,62 · 67,25 · 12,71 · 58,19 · -8,79 · -49,15* |
| ↳ Ixx | `.inertia.0` | kg·cm² | 0 … 100 000 | 33,857 · 14,652 · 9,024 · 34,486 · 5,539 · 0,103 | Tenseur d’inertie au centre de gravité, axes du repère i. *PRO : 27,932 · 23,291 · 19,721 · 53,896 · 14,282 · 1,35* |
| ↳ Iyy | `.inertia.1` | kg·cm² | 0 … 100 000 | 29,588 · 117,43 · 7,223 · 15,127 · 3,379 · 0,103 |  *PRO : 11,8 · 103,32 · 7,371 · 18,384 · 4,729 · 1,35* |
| ↳ Izz | `.inertia.2` | kg·cm² | 0 … 100 000 | 22,779 · 110,71 · 7,534 · 23,151 · 5,045 · 0,167 |  *PRO : 27,297 · 87,965 · 19,733 · 40,81 · 14,09 · 1,35* |
| ↳ Ixy | `.inertia.3` | kg·cm² | -100 000 … 100 000 | -0 · -0 · -0 · 0 · -0 · 0 |  *PRO : 0 · -0 · -0 · 0 · -0 · 0* |
| ↳ Ixz | `.inertia.4` | kg·cm² | -100 000 … 100 000 | 0,005 · -18,409 · -0,001 · -0 · -0,001 · 0 |  *PRO : 0 · -29,71 · -0 · 0 · 0 · 0* |
| ↳ Iyz | `.inertia.5` | kg·cm² | -100 000 … 100 000 | -0,642 · -0,002 · -1,078 · -13,764 · 0,205 · -0 |  *PRO : -3,352 · 0 · -2,077 · -20,01 · 1,278 · 0* |
| Outil — masse | `dynamics.tool.mass` | kg | 0 … 20 | 0,156 | Masse de l’outil (pince + servo + bride). |
| Outil — CdG (repère bride) | `dynamics.tool.com` | mm |  | -1,96 / -0,01 / 35,26 |  |
| Outil — inertie (Ixx, Iyy, Izz) | `dynamics.tool.inertiaDiag` | kg·cm² |  | 1,258 / 1,756 / 0,834 |  |
| Charge — masse | `dynamics.payload.mass` | kg | 0 … 20 | 0,2 | Masse de l’objet manipulé. Utilisée quand la pince est fermée sur un objet (et pour le dimensionnement). |
| Charge — CdG (repère TCP) | `dynamics.payload.com` | mm |  | 0 / 0 / 0 |  |
| Charge nominale (dimensionnement) | `dynamics.payloadRated` | kg | 0 … 20 | 0,5 | Charge utile maximale annoncée, utilisée par l’analyse de dimensionnement. |
| **Frottements (côté articulation)** — tableau par articulation (J1 · J2 · … · J6) | `dynamics.friction[i]` | | | |  |
| ↳ Coulomb | `.coulomb` | N·m | 0 … 50 | 0,35 · 0,7 · 0,45 · 0,15 · 0,15 · 0,1 | Couple de frottement sec (réducteur + roulements), ramené à la sortie. *PRO : 0,25 · 0,25 · 0,25 · 0,08 · 0,08 · 0,08* |
| ↳ Visqueux | `.viscous` | N·m·s/rad | 0 … 50 | 0,3 · 0,6 · 0,4 · 0,1 · 0,1 · 0,08 | Coefficient de frottement visqueux (couple ∝ vitesse). *PRO : 0,08 · 0,08 · 0,08 · 0,03 · 0,03 · 0,03* |
| ↳ Adhérence | `.stiction` | N·m | 0 … 50 | 0,45 · 0,9 · 0,6 · 0,2 · 0,2 · 0,14 | Couple de décollement (≥ Coulomb). Modèle de Stribeck. *PRO : 0,3 · 0,3 · 0,3 · 0,1 · 0,1 · 0,1* |
| ↳ Vitesse de Stribeck | `.stribeckVel` | °/s | 0 … 57,3 | 1,1 (tous) | Vitesse caractéristique de la transition adhérence → glissement. |

## Actionneurs (moteurs & réducteurs)

Modèle de chaque actionneur : type, réduction, rendement, caractéristiques électriques et mécaniques. Les couples/vitesses « de sortie » s’entendent côté articulation.

| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |
|---|---|---|---|---|---|
| Tension d’alimentation | `actuators.supplyVoltage` | V | 5 … 80 | 36 | Tension du bus moteurs. Détermine la chute de couple des pas-à-pas à haute vitesse. *PRO : 24* |
| **Actionneurs** — tableau par articulation (J1 · J2 · … · J6) | `actuators.joints[i]` | | | |  |
| ↳ Type | `.type` |  | `stepper`, `bldc_mit`, `servo` | Pas-à-pas + réducteur (boucle ouverte) (tous) | Modèle physique utilisé par la simulation. *PRO : Brushless QDD « mode MIT » (CAN) (tous)* |
| ↳ Référence | `.model` |  |  | « NEMA17-48 + cycloïdal CY-M 25:1 » · « NEMA23-76 + cycloïdal CY-L 30:1 » · « NEMA17-60 + cycloïdal CY-M 25:1 » · « NEMA17-40 + cycloïdal CY-S 20:1 » · « NEMA17-40 + cycloïdal CY-S 20:1 » · « NEMA17-34 + cycloïdal CY-S 20:1 » | Référence du moteur / module (documentation, nomenclature). *PRO : « DM4340 (QDD, mode MIT) » · « DM4340 (QDD, mode MIT) » · « DM4340 (QDD, mode MIT) » · « DM4310 (QDD, mode MIT) » · « DM4310 (QDD, mode MIT) » · « DM4310 (QDD, mode MIT) »* |
| ↳ Réduction | `.gearRatio` | :1 | 1 … 500 | 25 · 30 · 25 · 20 · 20 · 20 | Rapport de réduction N (moteur/sortie). Réducteur cycloïdal : N = nombre de lobes = nb de galets − 1. *PRO : 40 · 40 · 40 · 10 · 10 · 10* |
| ↳ Rendement | `.efficiency` | % | 10 … 100 | 70 (tous) | Rendement du réducteur (couple sortie = N·η·couple moteur). *PRO : 90 (tous)* |
| ↳ Inertie rotor | `.rotorInertia` | g·cm² | 0 … 100 000 | 82 · 480 · 102 · 54 · 54 · 34 | Inertie du rotor moteur. Ramenée à la sortie : N²·J_rotor (souvent dominante !). *PRO : 150 · 150 · 150 · 60 · 60 · 60* |
| ↳ Jeu (backlash) | `.backlash` | ° | 0 … 5,73 | 0,25 · 0,2 · 0,25 · 0,35 · 0,35 · 0,4 | Jeu angulaire total en sortie de réducteur. *PRO : 0,15 (tous)* |
| ↳ Raideur transmission | `.stiffness` | N·m/rad | 10 … 1 000 000 | 2500 · 4000 · 2500 · 800 · 800 · 600 | Raideur torsionnelle du réducteur (sortie). Utilisée avec le jeu. *PRO : 6000 · 6000 · 6000 · 2000 · 2000 · 2000* |
| ↳ Couple de maintien | `.holdingTorque` | N·m | 0 … 20 | 0,59 · 1,89 · 0,65 · 0,42 · 0,42 · 0,28 | [Pas-à-pas] Couple de maintien du moteur (fiche technique) au courant nominal. *PRO : 0 (tous)* |
| ↳ Angle de pas | `.stepAngle` | ° | 0,06 … 11,46 | 1,8 (tous) | [Pas-à-pas] 1,8° (200 pas/tour) ou 0,9° (400 pas/tour). |
| ↳ Micro-pas | `.microsteps` |  | 1 … 256 | 16 (tous) | [Pas-à-pas] Subdivision réglée sur le driver (DIP/UART). *PRO : 1 (tous)* |
| ↳ Courant nominal | `.ratedCurrent` | A | 0 … 20 | 2 · 2,8 · 2,1 · 1,7 · 1,7 · 1,3 | [Pas-à-pas] Courant par phase de la fiche technique. *PRO : 0 (tous)* |
| ↳ Courant de travail | `.runCurrent` | A | 0 … 20 | 1,8 · 2,52 · 1,89 · 1,53 · 1,53 · 1,17 | [Pas-à-pas] Courant réglé sur le driver. Couple ≈ proportionnel. *PRO : 0 (tous)* |
| ↳ Résistance de phase | `.phaseResistance` | Ω | 0,01 … 100 | 1,4 · 1,13 · 1,6 · 1,5 · 1,5 · 2,4 |  *PRO : 1 (tous)* |
| ↳ Inductance de phase | `.phaseInductance` | mH | 0,01 … 100 | 3 · 3,6 · 3 · 2,8 · 2,8 · 2,8 | [Pas-à-pas] Limite la montée du courant à haute vitesse → chute de couple. *PRO : 1 (tous)* |
| ↳ Couple crête (sortie) | `.peakTorque` | N·m | 0 … 500 | 10,32 · 39,69 · 11,38 · 5,88 · 5,88 · 3,92 | [Brushless/servo] Couple maximal en sortie de réducteur. *PRO : 27 · 27 · 27 · 7 · 7 · 7* |
| ↳ Couple nominal (sortie) | `.ratedTorque` | N·m | 0 … 500 | 6,19 · 23,81 · 6,83 · 3,53 · 3,53 · 2,35 | [Brushless/servo] Couple continu admissible (échauffement). *PRO : 9 · 9 · 9 · 3 · 3 · 3* |
| ↳ Vitesse à vide (sortie) | `.maxSpeed` | tr/min | 1 … 9549,3 | 24 · 20 · 24 · 30 · 30 · 30 | [Brushless/servo] Vitesse à vide en sortie. Le couple disponible décroît linéairement jusqu’à cette vitesse. *PRO : 52 · 52 · 52 · 200 · 200 · 200* |
| ↳ Constante de couple (sortie) | `.torqueConstant` | N·m/A | 0 … 100 | 0 (tous) | [Brushless/servo] Kt ramené à la sortie (N·m/A). *PRO : 4,4 · 4,4 · 4,4 · 0,945 · 0,945 · 0,945* |
| ↳ Résolution codeur | `.encoderCpr` |  | 0 … 16 777 216 | 4096 (tous) | Points par tour (côté indiqué ci-dessous). 0 = pas de codeur. *PRO : 16 384 (tous)* |
| ↳ Codeur en sortie | `.encoderOnOutput` |  | oui / non | oui (tous) | Coché : codeur absolu sur l’arbre de sortie (ex. AS5600/MT6701). Sinon : codeur moteur. |
| ↳ ID CAN | `.canId` |  | 0 … 2047 | 1 · 2 · 3 · 4 · 5 · 6 | [Mode MIT] Identifiant CAN du moteur (esclave). |
| ↳ P_MAX | `.pMax` | ° | 5,73 … 5729,58 | 716,2 (tous) | [Mode MIT] Plage de position du protocole (±P_MAX rad) — doit correspondre à la configuration du moteur. |
| ↳ V_MAX | `.vMax` | °/s | 5,7 … 28 647,9 | 458,4 (tous) | [Mode MIT] Plage de vitesse du protocole (±V_MAX rad/s). *PRO : 458,4 · 458,4 · 458,4 · 1718,9 · 1718,9 · 1718,9* |
| ↳ T_MAX | `.tMax` | N·m | 0,1 … 500 | 28 (tous) | [Mode MIT] Plage de couple du protocole (±T_MAX N·m). *PRO : 28 · 28 · 28 · 10 · 10 · 10* |
| ↳ KP_MAX | `.kpMax` | N·m/rad | 1 … 5000 | 500 (tous) | [Mode MIT] Borne haute du gain Kp codé sur 12 bits (souvent 500). |
| ↳ KD_MAX | `.kdMax` | N·m·s/rad | 0,1 … 100 | 5 (tous) | [Mode MIT] Borne haute du gain Kd codé sur 12 bits (souvent 5). |

## Commande (PID, mode MIT, couple calculé)

Stratégie de commande et gains. Le mode « pas-à-pas » reproduit le comportement réel des moteurs pas-à-pas en boucle ouverte (y compris la perte de pas). Les autres modes supposent des actionneurs pilotés en couple.

| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |
|---|---|---|---|---|---|
| Mode de commande | `control.mode` |  | `stepper`, `pid`, `mit`, `computed_torque`, `impedance`, `gravity_comp` | Pas-à-pas boucle ouverte (consigne de position) | PID : τ = Kp·e + Ki·∫e + Kd·ė + anticipations. MIT : τ = Kp·(p*−p) + Kd·(v*−v) + τff (exécuté dans le driver du moteur). Couple calculé : τ = M(q)·(q̈* + Kp·e + Kd·ė) + C(q,q̇)·q̇ + g(q). Impédance : τ = Jᵀ·(Kx·Δx + Dx·Δẋ) + g(q). Gravité : τ = g(q) — le bras flotte et se déplace à la main. *PRO : Mode MIT : Kp·(p*−p) + Kd·(v*−v) + τff* |
| Fréquence de boucle | `control.loopHz` | Hz | 50 … 20 000 | 1000 | Fréquence d’échantillonnage du correcteur (bloqueur d’ordre 0 entre deux échantillons). Une fréquence trop basse rend les gains élevés instables — comme sur le vrai robot. |
| Anticipation gravité | `control.feedforward.gravity` |  | oui / non | oui | Ajoute g(q) calculé par le modèle dynamique (indispensable pour des gains faibles / compliance). |
| Anticipation frottements | `control.feedforward.friction` |  | oui / non | oui | Ajoute une estimation des frottements (Coulomb + visqueux) dans le sens de la vitesse désirée. |
| Anticipation inertie | `control.feedforward.inertia` |  | oui / non | oui | Ajoute M(q)·q̈* : améliore fortement le suivi de trajectoire. |
| Erreur de modèle (test) | `control.modelError` | % | 0 … 50 | 0 | Perturbe volontairement les masses du modèle utilisé par le correcteur pour tester la robustesse (0 % = modèle parfait). |
| **Gains articulaires** — tableau par articulation (J1 · J2 · … · J6) | `control.joints[i]` | | | |  |
| ↳ PID — Kp | `.kp` | N·m/rad | 0 … 100 000 | 700 · 1100 · 500 · 80 · 80 · 40 | Gain proportionnel (couple par radian d’erreur). |
| ↳ PID — Ki | `.ki` | N·m/(rad·s) | 0 … 1 000 000 | 3500 · 5500 · 2500 · 400 · 400 · 200 | Gain intégral : annule l’erreur statique. Trop élevé → dépassement/oscillations. |
| ↳ PID — Kd | `.kd` | N·m·s/rad | 0 … 10 000 | 22 · 35 · 15 · 2,5 · 2,5 · 1,2 | Gain dérivé (amortissement). Calculé sur la mesure (pas de coup de dérivée sur échelon). |
| ↳ PID — limite intégrale | `.iLimit` | N·m | 0 … 500 | 5 · 8 · 5 · 2 · 2 · 1 | Saturation du terme intégral (anti-emballement / anti-windup). |
| ↳ PID — filtre dérivée | `.dFilterHz` | Hz | 1 … 5000 | 150 · 150 · 150 · 200 · 200 · 200 | Fréquence de coupure du filtre passe-bas du terme dérivé (réduit le bruit de mesure). |
| ↳ Zone morte | `.deadband` | ° | 0 … 2,86 | 0 (tous) | Erreur en dessous de laquelle le correcteur n’agit pas (évite le « pompage » dû au jeu). |
| ↳ MIT — Kp | `.mitKp` | N·m/rad | 0 … 5000 | 90 · 130 · 90 · 30 · 30 · 15 | Raideur du mode MIT (dans le driver moteur). Typique 20–150 N·m/rad pour un bras de bureau. *PRO : 80 · 120 · 80 · 30 · 25 · 15* |
| ↳ MIT — Kd | `.mitKd` | N·m·s/rad | 0 … 100 | 3 · 4 · 2,5 · 1 · 1 · 0,5 | Amortissement du mode MIT. Typique 0,5–5 N·m·s/rad. *PRO : 2,5 · 3,5 · 2,5 · 1 · 0,8 · 0,5* |
| ↳ Limite de couple | `.torqueLimit` | % | 5 … 100 | 100 (tous) | Fraction du couple disponible autorisée (protection mécanique). |
| Impédance — raideur linéaire | `control.impedance.kTrans` | N/mm | 0 … 100 | 0,8 | Raideur cartésienne du TCP en translation. |
| Impédance — raideur angulaire | `control.impedance.kRot` | N·m/rad | 0 … 1000 | 15 | Raideur cartésienne en rotation. |
| Impédance — amortissement ζ | `control.impedance.zeta` |  | 0 … 3 | 1 | Taux d’amortissement critique visé (1 = critique). |
| Impédance — amortissement articulaire | `control.impedance.nullKd` | N·m·s/rad | 0 … 100 | 0,5 | Amortissement articulaire additionnel (espace nul, stabilité). |
| Auto-réglage — bande passante | `control.autotune.bandwidthHz` | Hz | 0,5 … 100 | 8 | Bande passante visée par le placement de pôles : Kp = J·ω², Kd = 2·ζ·J·ω, Ki = Kp·ω/10 avec J = inertie vue par l’axe. |
| Auto-réglage — amortissement ζ | `control.autotune.zeta` |  | 0,3 … 2 | 0,9 | 0,7 : réponse rapide avec léger dépassement ; 1 : critique, sans dépassement. |

## Trajectoires & modèle inverse

Génération de mouvements (profils de vitesse, lissage) et solveur de cinématique inverse.

| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |
|---|---|---|---|---|---|
| Profil de vitesse | `trajectory.profile` |  | `scurve`, `trapezoid`, `quintic`, `cycloidal` | Courbe en S (jerk limité, 7 phases) | Courbe en S : accélération continue (recommandé pour limiter les vibrations des réducteurs imprimés). |
| Vitesse globale | `trajectory.speedOverride` | % | 1 … 100 | 50 | Facteur appliqué à toutes les vitesses programmées (override). |
| Facteur d’accélération | `trajectory.accelScale` | % | 1 … 100 | 100 | Facteur appliqué aux accélérations et jerks maximaux. |
| Zone de lissage par défaut | `trajectory.blendRadius` | mm | 0 … 200 | 5 | Rayon de passage (« fly-by ») entre deux mouvements consécutifs. 0 = arrêt précis sur chaque point. |
| Échantillonnage cartésien | `trajectory.sampleHz` | Hz | 20 … 2000 | 250 | Fréquence de discrétisation des trajectoires linéaires/circulaires (IK à chaque échantillon). |
| Solveur IK | `trajectory.ik.method` |  | `auto`, `analytic`, `numeric` | Automatique (analytique si possible, sinon numérique) | L’analytique donne jusqu’à 8 solutions exactes ; le numérique fonctionne pour toute géométrie. |
| Configuration préférée | `trajectory.ik.configuration` |  | `closest`, `front-up-noflip`, `front-down-noflip`, `front-up-flip`, `back-up-noflip` | La plus proche de la position actuelle | Choix parmi les solutions analytiques multiples. |
| IK — amortissement λ | `trajectory.ik.damping` |  | 0 … 1 | 0,02 | Facteur d’amortissement des moindres carrés (robustesse près des singularités). |
| IK — itérations max | `trajectory.ik.maxIter` |  | 5 … 2000 | 150 | Nombre maximal d’itérations du solveur numérique. |
| IK — tolérance position | `trajectory.ik.tolPos` | mm | 0 … 10 | 0,01 |  |
| IK — tolérance orientation | `trajectory.ik.tolRot` | ° | 0 … 5,73 | 0,01 |  |
| IK — poids orientation | `trajectory.ik.orientationWeight` | m | 0,001 … 2 | 0,15 | Longueur caractéristique (m/rad) équilibrant erreurs de position et d’orientation. |
| Jog — pas linéaire | `trajectory.jog.linStep` | mm | 0,1 … 100 | 5 |  |
| Jog — pas angulaire | `trajectory.jog.angStep` | ° | 0,06 … 57,3 | 5 |  |
| Jog — vitesse linéaire | `trajectory.jog.linSpeed` | mm/s | 1 … 1000 | 50 | Vitesse du jog cartésien continu. |
| Jog — vitesse articulaire | `trajectory.jog.jointSpeed` | °/s | 0,6 … 286,5 | 30 |  |

## Pince / outil

Pince parallèle à crémaillère entraînée par servomoteur (MG996R par défaut).

| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |
|---|---|---|---|---|---|
| Type | `gripper.type` |  | `parallel`, `vacuum`, `none` | Pince parallèle (servo) |  |
| Ouverture max | `gripper.strokeMax` | mm | 1 … 300 | 50 | Écartement maximal des doigts. |
| Vitesse des doigts | `gripper.speed` | mm/s | 1 … 1000 | 40 |  |
| Force de serrage | `gripper.force` | N | 0 … 500 | 25 | Force nominale (servo en butée sur l’objet). |
| Longueur des doigts | `gripper.fingerLength` | mm | 5 … 300 | 55 |  |
| Servo — impulsion fermée | `gripper.servoMinUs` |  | 400 … 2600 | 900 | Largeur d’impulsion PWM (µs) pince fermée. |
| Servo — impulsion ouverte | `gripper.servoMaxUs` |  | 400 … 2600 | 2100 | Largeur d’impulsion PWM (µs) pince ouverte. |

## Sécurité

Surveillance des collisions, vitesses réduites et chien de garde de communication.

| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |
|---|---|---|---|---|---|
| Détection de collision (couple) | `safety.collisionDetection` |  | oui / non | oui | Compare le couple mesuré/commandé au couple prédit par le modèle ; au-delà du seuil → arrêt. |
| Seuil de collision | `safety.collisionThreshold` | N·m | 0,05 … 50 | 3 | Écart de couple (N·m, côté articulation) déclenchant l’arrêt. |
| Auto-collision | `safety.selfCollision` |  | oui / non | oui | Vérifie les collisions entre segments non adjacents (modèle en capsules). |
| Collision avec la table | `safety.floorCollision` |  | oui / non | oui | Vérifie que le bras et l’outil restent au-dessus du plan de travail. |
| Marge de collision | `safety.capsuleMargin` | mm | 0 … 50 | 4 | Distance minimale tolérée entre capsules. |
| Vitesse réduite (mode manuel) | `safety.reducedSpeed` | % | 1 … 100 | 25 | Limite de vitesse en mode apprentissage (≈ 250 mm/s en industrie). |
| Vitesse TCP collaborative | `safety.collabTcpSpeed` | mm/s | 10 … 2000 | 250 | Vitesse TCP maximale quand le mode collaboratif est actif. |
| Chien de garde (ms) | `safety.watchdogMs` |  | 20 … 5000 | 250 | Sans nouvelle consigne pendant ce délai, le robot freine et s’arrête (flux temps réel). |
| Marge butées logicielles | `safety.softLimitMargin` | ° | 0 … 11,46 | 1 | Marge de décélération avant les butées. |

## Simulation

Intégration numérique et effets physiques simulés.

| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |
|---|---|---|---|---|---|
| Pas physique | `simulation.dt` | ms | 0,05 … 5 | 0,25 | Pas d’intégration (intégrateur semi-implicite). 0,25 ms reproduit fidèlement la raideur magnétique des pas-à-pas. |
| Facteur temps réel | `simulation.realtimeFactor` |  | 0,05 … 10 | 1 | 1 = temps réel ; < 1 ralenti ; > 1 accéléré. |
| Frottements | `simulation.enableFriction` |  | oui / non | oui |  |
| Jeu des réducteurs | `simulation.enableBacklash` |  | oui / non | oui |  |
| Perte de pas (pas-à-pas) | `simulation.enableStepLoss` |  | oui / non | oui | Modèle physique couple-angle : si la charge dépasse le couple disponible, le rotor décroche. |
| Bruit de mesure | `simulation.sensorNoise` | ° | 0 … 0,57 | 0 | Écart-type du bruit ajouté aux positions mesurées. |
| Fréquence d’enregistrement | `simulation.recordHz` | Hz | 10 … 2000 | 200 | Échantillonnage des courbes et exports CSV. |
| Fenêtre des courbes | `simulation.plotWindow` | s | 1 … 120 | 10 |  |

## Matériel & firmware

Brochage, drivers, prise d’origine. Exporté dans firmware/orion_fw/src/orion_config.h.

| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |
|---|---|---|---|---|---|
| Contrôleur | `hardware.controller` |  | `teensy41`, `mit_can_bridge` | Teensy 4.1 (recommandé) |  *PRO : Teensy 4.1 + CAN (moteurs mode MIT)* |
| Largeur impulsion STEP | `hardware.stepPulse` | µs | 1 … 100 | 3 | DM542T : ≥ 2,5 µs. TMC2209 : ≥ 0,1 µs. |
| Délai DIR → STEP | `hardware.dirSetup` | µs | 0 … 100 | 6 | Temps d’établissement de la direction avant l’impulsion (DM542T : ≥ 5 µs). |
| ENABLE actif à l’état bas | `hardware.enableActiveLow` |  | oui / non | oui | Dépend du câblage des drivers (optocoupleurs DM542T : ENA+ au 5 V, ENA− piloté). |
| Fréquence de flux USB | `hardware.streamHz` | Hz | 10 … 1000 | 200 | Fréquence d’envoi des consignes articulaires en mode jumeau numérique (streaming). |
| Lissage du flux (retard) | `hardware.streamDelay` | ms | 0 … 400 | 40 | Avance du tampon d’interpolation du firmware : absorbe la gigue USB/navigateur. Plus grand = plus lisse mais plus de retard (40 ms conseillé). |
| Débit CAN | `hardware.canBitrate` | kbit/s | 125 … 1000 | 1000 | Débit du bus CAN (mode MIT) — 1 Mbit/s en standard. |
| Fréquence de commande CAN (MIT) | `hardware.mitCtrlHz` | Hz | 50 … 1000 | 400 | Trames MIT envoyées par moteur et par seconde. Chaque trame reçoit une réponse : 6 moteurs à 400 Hz ≈ 60 % d’un bus à 1 Mbit/s. |
| Délai max. sans retour CAN | `hardware.feedbackTimeout` | ms | 10 … 1000 | 50 | Au-delà, le pont passe en défaut et amortit tous les axes. |
| Température moteur max. | `hardware.maxMotorTemp` |  | 40 … 120 | 85 | °C (MOSFET ou bobinage) avant mise en défaut. |
| **Brochage Teensy** — tableau par articulation (J1 · J2 · … · J6) | `hardware.pins[i]` | | | |  |
| ↳ STEP | `.step` |  | 0 … 54 | 2 · 4 · 6 · 8 · 10 · 12 |  |
| ↳ DIR | `.dir` |  | 0 … 54 | 3 · 5 · 7 · 9 · 11 · 14 |  |
| ↳ Capteur d’origine | `.limit` |  | 0 … 54 | 24 · 25 · 26 · 27 · 28 · 29 |  |
| ↳ Inverser DIR | `.invertDir` |  | oui / non | non (tous) | Inverse le sens de rotation physique (câblage moteur). |
| ↳ Capteur actif bas | `.limitActiveLow` |  | oui / non | oui (tous) | Capteur NPN/interrupteur vers la masse avec pull-up : actif à l’état bas. |
| Broche ENABLE commune | `hardware.enablePin` |  | 0 … 54 | 15 |  |
| Broche arrêt d’urgence | `hardware.estopPin` |  | 0 … 54 | 30 |  |
| Broche servo pince | `hardware.gripperPin` |  | 0 … 54 | 33 |  |
| **Prise d’origine** — tableau par articulation (J1 · J2 · … · J6) | `hardware.homing[i]` | | | |  |
| ↳ Active | `.enabled` |  | oui / non | oui (tous) |  |
| ↳ Sens de recherche | `.direction` |  | -1 … 1 | -1 · -1 · 1 · -1 · -1 · -1 | −1 ou +1 : sens de déplacement vers le capteur. |
| ↳ Vitesse de recherche | `.speed` | °/s | 0,1 … 114,6 | 20 · 15 · 15 · 30 · 30 · 30 |  |
| ↳ Vitesse d’approche fine | `.slowSpeed` | °/s | 0,1 … 57,3 | 2 · 2 · 2 · 3 · 3 · 3 |  |
| ↳ Recul | `.backoff` | ° | 0 … 28,65 | 4 · 4 · 4 · 5 · 5 · 5 |  |
| ↳ Position du capteur | `.switchPosition` | ° | -360 … 360 | -172 · -133 · 62 · -172 · -117 · -182 | Valeur articulaire q au point de déclenchement du capteur (à calibrer). |
| ↳ Ordre | `.order` |  | 1 … 12 | 6 · 5 · 4 · 3 · 1 · 2 | Ordre de prise d’origine (poignet d’abord recommandé). |

## Poses nommées

Positions articulaires de référence utilisables dans les programmes (MoveJ Home…).

| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |
|---|---|---|---|---|---|
| Home (travail) | `poses.home` | ° |  | 0 / 0 / 0 / 0 / 90 / 0 | Pose de départ des programmes. |
| Repos (transport) | `poses.rest` | ° |  | 0 / -10 / 55 / 0 / 35 / 0 | Pose repliée, compacte et stable hors tension. |
| Zéro mécanique | `poses.zero` | ° |  | 0 / 0 / 0 / 0 / 0 / 0 | Toutes les articulations à 0 (vérification de la table DH). |
