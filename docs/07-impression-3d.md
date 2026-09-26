<!-- FICHIER GÉNÉRÉ par tools/gen-docs.mjs depuis hardware/parts.json — ne pas modifier à la main. -->

# 7. Impression 3D des pièces

Toutes les pièces sont générées par la CAO paramétrique (`npm run build:cad`) et se trouvent dans
[`hardware/stl/`](../hardware/stl/). Elles tiennent sur un plateau **220 × 220 × 250 mm** (vérifié automatiquement)
et sont déjà **orientées pour l’impression** (face d’appui en Z = 0, sans supports sauf mention).

**Total : 37 fichiers STL, 58 pièces, ≈ 2,2 kg de matière** (+ 25 % de marge pour les purges et
supports éventuels).

> ⚠️ **Imprimez d’abord `tolerance-test.stl`.** Il contient les alésages de goupilles Ø3/Ø4/Ø5, un logement de
> roulement 688, un logement d’insert M3 et une fente de méplat. Si les goupilles n’entrent pas à la main (ajustement
> glissant) ou flottent, corrigez le paramètre *Jeu d’impression* (CAO) ou la compensation XY de votre trancheur **avant**
> d’imprimer les réducteurs. Ce projet n’a pas encore été validé sur un exemplaire physique.

## Réglages conseillés

| Réglage | Pièces de structure | Réducteurs (disques, cames, flasques) | Patins de pince |
|---|---|---|---|
| Matériau | PETG (ou PLA+, ASA près des moteurs chauds) | PETG ou PA-CF si disponible | TPU 95A |
| Buse / couche | 0,4 mm / 0,2 mm | 0,4 mm / **0,15 mm** | 0,4 mm / 0,2 mm |
| Parois | 4 périmètres | **5 périmètres** | 3 |
| Remplissage | voir tableau (gyroïde) | voir tableau (100 % pour cames et disques) | 100 % |
| Couches pleines dessus/dessous | 5 / 5 | 6 / 6 | 4 / 4 |
| Vitesse | 60–100 mm/s | **≤ 50 mm/s** (précision des profils) | 25 mm/s |
| Compensation XY | selon le test de tolérances | idem (profils critiques) | — |
| Supports | non (sauf note) | non | non |

- **Retrait / dilatation** : laissez refroidir les pièces sur le plateau (PETG) pour limiter le gauchissement des carters.
- **Inserts filetés** : fer à 230–245 °C (PETG), poussez l’insert bien perpendiculairement ; laissez refroidir avant de
  visser. Les logements font Ø4,0 mm (M3) et Ø5,6 mm (M4), profondeur 6 mm.
- **Goupilles** : les alésages des couronnes sont prévus pour des goupilles acier trempé ISO 8734 enfoncées à la presse
  (étau). Une goutte de colle frein/époxy est inutile si l’ajustement est correct.
- **Disques cycloïdaux** : ébavurez l’arête inférieure (« pied d’éléphant ») au cutter ; c’est la cause n°1 de points
  durs. Les deux disques d’un même module (A et B) sont décalés d’un demi-lobe : ils ne sont **pas** interchangeables.
- **Cames** : 100 % de remplissage, méplat orienté vers le haut ; vérifiez l’excentricité au comparateur si possible.

## Liste des pièces

| Fichier | Pièce | Qté | Matière · remplissage | Encombrement (mm) | Masse (unité) | Remarques |
|---|---|---|---|---|---|---|
| **Module CY-M** | | | | | | |
| `CY-M_housing.stl` | Carter | 2 | PETG · 60 % | 100 × 100 × 34 | 61 g | Imprimer fond sur le plateau, sans supports (les gorges sont verticales). |
| `CY-M_retainer.stl` | Couvercle | 2 | PETG · 80 % | 100 × 100 × 3 | 12 g | Maintient la bague extérieure du roulement principal ; serré par les boulons de fixation du module. |
| `CY-M_cam.stl` | Came excentrique | 2 | PETG · 100 % | 19 × 17 × 25 | 5 g | Remplissage 100 %. Excentricité 0.9 mm. Emmanchée sur l’arbre moteur (méplat), goutte de colle frein. |
| `CY-M_discA.stl` | Disque cycloïdal A | 2 | PETG · 100 % | 62 × 62 × 7 | 18 g | 25 lobes. Remplissage 100 %, 4 périmètres, couche 0,12–0,16 mm pour le profil. |
| `CY-M_discB.stl` | Disque cycloïdal B | 2 | PETG · 100 % | 62 × 62 × 7 | 18 g | Identique au disque A mais décalé d’un demi-lobe (7.20°) : repérer « B ». |
| `CY-M_flange.stl` | Flasque de sortie | 2 | PETG · 90 % | 58 × 58 × 16 | 39 g | Imprimer face avant sur le plateau. 6 inserts M3 ; 6 doigts Ø4 emmanchés. |
| `CY-M_clamp.stl` | Bague de sortie | 2 | PETG · 90 % | 63 × 63 × 8 | 13 g | Face d’interface du segment suivant. Serre la bague intérieure du roulement. |
| **Module CY-L** | | | | | | |
| `CY-L_housing.stl` | Carter | 1 | PETG · 60 % | 118 × 118 × 39 | 95 g | Imprimer fond sur le plateau, sans supports (les gorges sont verticales). |
| `CY-L_retainer.stl` | Couvercle | 1 | PETG · 80 % | 118 × 118 × 3 | 15 g | Maintient la bague extérieure du roulement principal ; serré par les boulons de fixation du module. |
| `CY-L_cam.stl` | Came excentrique | 1 | PETG · 100 % | 22 × 20 × 29 | 8 g | Remplissage 100 %. Excentricité 1 mm. Emmanchée sur l’arbre moteur (méplat), goutte de colle frein. |
| `CY-L_discA.stl` | Disque cycloïdal A | 1 | PETG · 100 % | 77 × 77 × 8 | 32 g | 30 lobes. Remplissage 100 %, 4 périmètres, couche 0,12–0,16 mm pour le profil. |
| `CY-L_discB.stl` | Disque cycloïdal B | 1 | PETG · 100 % | 77 × 77 × 8 | 32 g | Identique au disque A mais décalé d’un demi-lobe (6.00°) : repérer « B ». |
| `CY-L_flange.stl` | Flasque de sortie | 1 | PETG · 90 % | 73 × 73 × 17 | 68 g | Imprimer face avant sur le plateau. 6 inserts M4 ; 8 doigts Ø5 emmanchés. |
| `CY-L_clamp.stl` | Bague de sortie | 1 | PETG · 90 % | 78 × 78 × 8 | 20 g | Face d’interface du segment suivant. Serre la bague intérieure du roulement. |
| **Module CY-S** | | | | | | |
| `CY-S_housing.stl` | Carter | 3 | PETG · 60 % | 91 × 91 × 31 | 52 g | Imprimer fond sur le plateau, sans supports (les gorges sont verticales). |
| `CY-S_retainer.stl` | Couvercle | 3 | PETG · 80 % | 91 × 91 × 3 | 10 g | Maintient la bague extérieure du roulement principal ; serré par les boulons de fixation du module. |
| `CY-S_cam.stl` | Came excentrique | 3 | PETG · 100 % | 17 × 15 × 24 | 4 g | Remplissage 100 %. Excentricité 0.8 mm. Emmanchée sur l’arbre moteur (méplat), goutte de colle frein. |
| `CY-S_discA.stl` | Disque cycloïdal A | 3 | PETG · 100 % | 53 × 53 × 6 | 12 g | 20 lobes. Remplissage 100 %, 4 périmètres, couche 0,12–0,16 mm pour le profil. |
| `CY-S_discB.stl` | Disque cycloïdal B | 3 | PETG · 100 % | 54 × 54 × 6 | 12 g | Identique au disque A mais décalé d’un demi-lobe (9.00°) : repérer « B ». |
| `CY-S_flange.stl` | Flasque de sortie | 3 | PETG · 90 % | 53 × 53 × 14 | 28 g | Imprimer face avant sur le plateau. 6 inserts M3 ; 6 doigts Ø3 emmanchés. |
| `CY-S_clamp.stl` | Bague de sortie | 3 | PETG · 90 % | 57 × 57 × 8 | 10 g | Face d’interface du segment suivant. Serre la bague intérieure du roulement. |
| **Socle** | | | | | | |
| `base-plate.stl` | Plaque de base | 1 | PETG · 50 % | 200 × 200 × 6 | 104 g | Fixation à la table par 4 vis M6 (lumières Ø6,8). Ouverture pour le passage des câbles. |
| `socle.stl` | Socle (fût) | 1 | PETG · 40 % | 116 × 116 × 80 | 85 g | Porte le module J1. Inserts M4 sur la face supérieure et le pied. |
| **Tourelle** | | | | | | |
| `turret.stl` | Tourelle (tambour d’épaule) | 1 | PETG · 45 % | 120 × 122 × 118 | 281 g | Tambour horizontal qui reçoit le module J2 (moteur NEMA23 dans l’alésage). Imprimer collerette sur le plateau (alésage vertical), supports sous le pied. |
| **Bras supérieur** | | | | | | |
| `upperarm-low.stl` | Raccord bas du bras | 1 | PETG · 60 % | 86 × 143 × 54 | 141 g | Vissé sur la sortie de J2 ; serre le tube Ø40 (vis M4 + écrou). |
| `upperarm-high.stl` | Raccord haut du bras (porte J3) | 1 | PETG · 60 % | 110 × 146 × 54 | 108 g | Reçoit le module J3 (moteur côté extérieur). |
| **Coude** | | | | | | |
| `elbow.stl` | Bloc coude (porte J4) | 1 | PETG · 45 % | 96 × 86 × 86 | 196 g | Vissé sur la sortie de J3 ; loge le moteur de J4. |
| **Avant-bras** | | | | | | |
| `forearm-adapter.stl` | Adaptateur de tube (sortie J4) | 1 | PETG · 60 % | 63 × 63 × 30 | 28 g | Tube collé (époxy) + vis M3 traversante. |
| `wrist.stl` | Poignet (porte J5) | 1 | PETG · 55 % | 138 × 101 × 102 | 102 g | Reçoit le module J5 ; le moteur de J5 dépasse côté +y. |
| **Poignet** | | | | | | |
| `yoke.stl` | Chape (J5 → J6) | 1 | PETG · 70 % | 93 × 102 × 65 | 43 g | Relie la sortie de J5 au carter de J6. Imprimer à plat, remplissage élevé. |
| **Pince** | | | | | | |
| `gripper-body.stl` | Corps de pince | 1 | PETG · 50 % | 72 × 57 × 59 | 68 g | Vissé sur la bride J6 (6 × M3). Servo MG996R glissé par le dessous, fixé par 4 vis M2. |
| `gripper-pinion.stl` | Pignon m1 z20 | 1 | PETG · 100 % | 22 × 22 × 8 | 3 g | Vissé sur le palonnier rond du servo (4 vis M2). PETG 100 %. |
| `gripper-finger-left.stl` | Doigt gauche (crémaillère m1) | 1 | PETG · 80 % | 60 × 69 × 25 | 14 g | Crémaillère intégrée. Imprimer à plat, 100 % sur la denture conseillé. |
| `gripper-pad-left.stl` | Patin gauche (TPU) | 1 | TPU · 100 % | 33 × 16 × 2 | 1 g | TPU 95A, collé sur le doigt (adhérence). |
| `gripper-finger-right.stl` | Doigt droit (crémaillère m1) | 1 | PETG · 80 % | 61 × 69 × 25 | 14 g | Crémaillère intégrée. Imprimer à plat, 100 % sur la denture conseillé. |
| `gripper-pad-right.stl` | Patin droit (TPU) | 1 | TPU · 100 % | 33 × 16 × 2 | 1 g | TPU 95A, collé sur le doigt (adhérence). |
| **Calibrage** | | | | | | |
| `tolerance-test.stl` | Test de tolérances | 1 | PETG · 100 % | 110 × 42 × 6 | 29 g | À imprimer en premier : alésages de roulements (6802, 6810, 688), goupilles Ø3/4/5, inserts M3/M4, jeu d’emboîtement du tube. |

## Ordre d’impression conseillé

1. `tolerance-test` → ajuster le trancheur.
2. Un module complet **CY-S** (le plus petit, J4–J6) → assembler et tester à la main (rotation fluide, pas de jeu
   radial perceptible, couple d’entraînement faible). Corriger si besoin avant de lancer les autres modules.
3. Modules **CY-M** (J1, J3) et **CY-L** (J2).
4. Structure : socle, plaque de base, tourelle, raccords de bras, coude, poignet, chape.
5. Pince (corps, pignon, doigts, patins TPU).

Les masses ci-dessus proviennent de la CAO (volume × densité × taux de remplissage estimé) ; elles alimentent
directement le modèle dynamique du simulateur (`studio/core/generated/massprops.js`).
