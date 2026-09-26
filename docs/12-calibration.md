# 12. Calibration et mise en service

La calibration fait coïncider le **modèle** (ORION Studio, firmware) avec **votre** robot : sens des axes, échelles,
zéros, longueurs, masses. Travaillez à **vitesse réduite** (10–25 %), l’arrêt d’urgence à portée de main.

Outils utiles :

- inclinomètre numérique (ou application de téléphone calibrée) ;
- pied à coulisse ;
- comparateur à cadran sur support magnétique ;
- balance de cuisine ;
- rapporteur.

## 1. Sens de rotation (axes non référencés)

Moteurs actifs (`EN 1`), sans prise d’origine, déplacez chaque axe de +5° :

```
JG 1 5      JG 2 5      JG 3 5      JG 4 5      JG 5 5      JG 6 5
```

| Axe | Sens positif attendu |
|---|---|
| J1 | rotation anti-horaire vue de dessus |
| J2 | le bras penche vers l’avant (+X) |
| J3 | l’avant-bras s’abaisse |
| J4 | rotation anti-horaire autour de l’avant-bras, vue depuis le poignet en regardant vers le coude |
| J5 | l’outil s’incline vers le bas (depuis la pose Zéro) |
| J6 | rotation anti-horaire autour de l’axe de l’outil, vue depuis le bout des doigts en regardant vers le poignet |

Un axe tourne à l’envers ? Deux corrections possibles :

- `SET invert n 1` puis `SAVE` ;
- cocher *Inverser DIR* dans ORION Studio, puis *Envoyer la config*.

En cas de doute, activez les *Repères DH* dans la vue 3D : l’axe z de chaque repère donne le sens positif (règle de la
main droite).

## 2. Échelle (pas par degré)

Pour chaque axe, repérez un trait sur la partie fixe et la partie mobile, puis commandez un grand déplacement, par
exemple `JG 1 30` trois fois (90°). Mesurez l’angle réel au rapporteur ou à l’inclinomètre.

steps_per_deg_nouveau = steps_per_deg_actuel × angle_commandé / angle_mesuré

Un écart d’un facteur 2 ou 4 signale un mauvais réglage des micro-pas du driver. Un écart de 4 % sur un CY-M (26/25)
signale une confusion entre nombre de galets et rapport de réduction. Corrigez la cause plutôt que la valeur.

## 3. Positions des capteurs d’origine

Après `HOME`, le firmware croit être en `home_pose`. L’écart réel vient de la position exacte du capteur. Pour chaque
axe :

1. `HOME`, puis `MJ 0 …` vers une pose où l’axe est mesurable :
   - J2 et J3 : bras vertical, avant-bras horizontal (pose *Zéro*), vérifiés à l’inclinomètre ;
   - J1 : bras dans l’axe X de la table (repère tracé) ;
   - J4, J5, J6 : faces planes du poignet et de la pince, à l’équerre.
2. Mesurez l’écart δ = angle réel − angle commandé, avec le signe du sens positif de l’axe.
3. Corrigez : `SET home_pos n (home_pos + δ)`, puis `SAVE`. Recommencez `HOME` et vérifiez.

Exemple : après `HOME` puis `MJ 0 0 0 0 0 90 0`, l’inclinomètre sur le tube du bras indique 1,2° vers l’avant
(J2 réel = +1,2°). Faites `SET home_pos 2 -131.8` (−133 + 1,2), `SAVE`, puis `HOME`.

**Répétabilité de la prise d’origine** : enchaînez 5 `HOME` et mesurez la pose Home au comparateur (TCP contre une
cale). La dispersion doit rester sous 0,1 mm. Sinon, réduisez `home_slow` et vérifiez la fixation de l’aimant et du
capteur.

## 4. Géométrie (table DH) et outil

- **Longueurs** : mesurez d’axe à axe a₂ (J2 → J3, nominal 240 mm) et d₄ (J3 → centre du poignet, nominal 220 mm).
  Reportez-les dans *Paramètres → Cinématique*.
  - ⚠️ Modifier a₂ ou d₄ **régénère aussi la CAO**, avec d’autres longueurs de tube. Pour une simple correction après
    montage, c’est sans conséquence : n’imprimez rien de nouveau.
- **Outil (TCP)** : mesurez la distance bride → bout des doigts fermés (nominal 100,3 mm), et reportez-la dans *TCP —
  Z*. Pour un autre outil (stylo, ventouse), renseignez aussi X/Y et l’orientation.
- **Vérification** : placez une pointe fixe sur la table. Amenez le TCP dessus dans trois configurations différentes
  (MoveL avec orientations différentes). Si la pointe « tourne » autour du TCP, la longueur d’outil ou les zéros J4–J6
  sont faux.

## 5. Masses (dynamique, compensation de gravité)

Pesez les sous-ensembles au montage : bras, coude avec avant-bras, poignet, pince. Corrigez les masses dans *Paramètres
→ Dynamique*. Les centres de masse calculés par la CAO restent de bonnes valeurs de départ.

- **Pas-à-pas** : les masses servent au dimensionnement (*Analyse*) et aux anticipations de la simulation.
- **PRO (mode MIT)** : elles déterminent directement la **compensation de gravité**. Vérifiez-les en mode *Compensation
  de gravité* :
  - le bras lâché dans une pose quelconque doit rester en place, ou dériver lentement ;
  - s’il retombe, augmentez la masse (ou le centre de masse) du segment concerné ;
  - s’il remonte, diminuez-la.

## 6. Raideur et jeu des réducteurs (pour la simulation)

Pour chaque axe, moteur actif et à l’arrêt :

1. Placez un comparateur sur le segment à une distance r de l’axe (par exemple 100 mm).
2. Appliquez à la main un effort alterné, faible puis plus fort, avec un peson à la distance r si possible.
3. **Jeu** : la zone où le comparateur bouge sans effort donne jeu (rad) = course / r.
4. **Raideur** : la pente au-delà du jeu donne k = (F·r) / (déplacement / r) en N·m/rad.

Reportez jeu et raideur dans *Paramètres → Actionneurs*. Le simulateur, l’auto-réglage des gains et le choix de la zone
morte les utilisent.

## 7. Pince

- `GRIP 0` : les doigts doivent se toucher **sans forcer**. Le servo ne doit ni grogner ni chauffer. Ajustez
  `SET grip_closed * 950` (valeur en µs), puis `SAVE`.
- `GRIP 100` : ouverture maximale sans butée dure (`SET grip_open * …`).

## 8. ORION-6 PRO : moteurs, zéros et gains

1. **Chaque moteur, seul sur le bus**, avec l’outil du fabricant :
   - mode **MIT** ;
   - identifiant CAN = numéro d’axe (1 à 6) ;
   - identifiant maître ;
   - plages **P_MAX = 12,5 rad**, **V_MAX** (8 rad/s pour DM4340, 30 rad/s pour DM4310), **T_MAX** (28 N·m pour
     DM4340, 10 N·m pour DM4310).

   Ces plages doivent être identiques à celles d’ORION Studio (*Actionneurs*).
2. **Bus complet** : `STREAM 10` puis `FB`. Les six moteurs doivent répondre (températures plausibles, erreur `1` =
   actif ou `0` = inactif).
3. **Sens** : moteurs coupés, tournez chaque axe à la main dans son sens positif (tableau §1). La valeur de `STAT` doit
   croître ; sinon, `SET dir n -1`.
4. **Zéro** : posez le bras dans sa **pose de repos** sur un support (J2 −10°, J3 55°, J5 35°, les autres à 0), puis
   `ZERO` et `SAVE`. Contrôlez ensuite `STAT` dans deux ou trois poses mesurées à l’inclinomètre.
   - Les moteurs à réducteur intégré ne connaissent leur position absolue qu’à l’intérieur d’un tour moteur. Après
     une coupure d’alimentation, vérifiez donc toujours `STAT` **avant** `EN 1`. Si la position est fausse, reposez le
     bras en pose de repos et refaites `ZERO`.
5. **Gains** : suivez [6. Commande et réglage](06-commande-et-reglage.md#6-réglage-spécifique-orion-6-pro-mode-mit).
   Commencez avec la moitié des gains par défaut, **compensation de gravité active**.

## 9. Validation finale

1. Programme **Test de répétabilité** (exemple d’ORION Studio) en *Jumeau numérique*, avec un comparateur au point
   d’arrivée. Relevez la dispersion sur 20 cycles.
2. Programme **Prise et dépose** avec un vrai cube de 30 mm aux positions indiquées.
3. Montée en vitesse progressive : 25, puis 50, puis 75 % en surveillant :
   - l’absence de pas perdus : la position après `HOME` doit rester identique ;
   - les températures des moteurs ;
   - le bruit des réducteurs.
4. Sauvegardez la configuration validée : *Paramètres → JSON* (fichier) et `SAVE` (EEPROM).
