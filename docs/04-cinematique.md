# 4. Cinématique

Code : `studio/core/kinematics.js` (modèles direct, inverse et jacobien), `studio/core/math3d.js`,
`studio/core/export.js` (URDF, MJCF). Tests : `tests/kinematics.test.mjs`.

## Repères et conventions

- **Repère de base** : Z vers le haut, X vers l’avant du robot, origine au centre de la plaque de base, au niveau de la
  table. Il peut être déplacé ou tourné (paramètres *Base — X…lacet*) pour un montage mural ou au plafond.
- **Unités** : le calcul utilise le mètre et le radian ; l’interface affiche le millimètre et le degré.
- **Pose Zéro** (tous les angles à 0) : bras supérieur **vertical**, avant-bras **horizontal** vers +X, outil dans l’axe
  de l’avant-bras.
- **Pose Home** `(0, 0, 0, 0, 90°, 0)` : même pose, poignet plié à 90°, outil vers le bas. Le TCP est alors en
  (220, 0, 266,5) mm.
- **Sens positifs** : J2+ penche le bras vers l’avant, J3+ abaisse l’avant-bras, J5+ incline l’outil vers le bas
  (vérifié par les tests).
- Orientation cartésienne en **roulis-tangage-lacet** (Rx, Ry, Rz, angles fixes XYZ). En interne, l’orientation est
  gérée par matrices et quaternions, sans blocage de cardan.

## Table de Denavit-Hartenberg (convention standard)

Aᵢ = Rz(θᵢ) · Tz(dᵢ) · Tx(aᵢ) · Rx(αᵢ), avec θᵢ = sensᵢ · qᵢ + θ₀ᵢ.

| i | aᵢ (mm) | αᵢ | dᵢ (mm) | θ₀ᵢ | Rôle |
|---|---|---|---|---|---|
| 1 | 0 | −90° | 160 | 0 | base → épaule |
| 2 | 240 | 0 | 0 | −90° | bras (épaule → coude) |
| 3 | 0 | −90° | 0 | 0 | coude |
| 4 | 0 | +90° | 220 | 0 | avant-bras (coude → poignet) |
| 5 | 0 | −90° | 0 | 0 | poignet |
| 6 | 0 | 0 | 33,2 | 0 | bride |
| outil | TCP à +100,3 mm sur z₆ | | | | pince fermée, bout des doigts |

La **convention modifiée** (Craig, MDH) est aussi prise en charge : Aᵢ = Rx(αᵢ₋₁)·Tx(aᵢ₋₁)·Rz(θᵢ)·Tz(dᵢ). Un test
vérifie qu’un même robot décrit dans les deux conventions donne le même TCP. Le paramètre *sens* (±1) inverse le sens
positif d’un axe sans toucher à la géométrie.

**Modèle direct** : ⁰T₆ = A₁·A₂·…·A₆, puis T_tcp = T_base · ⁰T₆ · T_outil. ORION Studio affiche les repères DH
intermédiaires (bouton *Repères DH* de la vue 3D) : c’est le meilleur moyen de valider une table DH saisie à la main.

## Modèle inverse analytique

Il s’applique dès que le robot a un **poignet sphérique**, c’est-à-dire des axes 4, 5 et 6 concourants : a₄ = a₅ = d₅ =
0 et α₄, α₅ = ±90°. Le logiciel vérifie ces conditions automatiquement (`analyticInfo`) et bascule sur le solveur
numérique sinon.

1. **Centre du poignet** : W = p_tcp − (d₆ + L_outil)·z_tcp (en tenant compte du repère outil et de la base).
2. **J1** : θ₁ = atan2(W_y, W_x) *(épaule avant)* ou θ₁ + π *(épaule arrière)*.
3. **J2, J3** : triangle épaule–coude–poignet de côtés a₂ et d₄ (loi des cosinus). Deux solutions, *coude haut* et
   *coude bas*.
4. **J4, J5, J6** : orientation relative ³R₆ = (⁰R₃)ᵀ · ⁰R₆, décomposée en angles d’Euler ZYZ. Deux solutions,
   *poignet normal* et *poignet retourné* (θ₅ ↔ −θ₅, avec θ₄ et θ₆ décalés de π).

On obtient **jusqu’à 8 solutions exactes**. Elles sont filtrées par les butées, puis la solution retenue est :

- soit la **plus proche** de la position actuelle (distance articulaire pondérée, par défaut) ;
- soit une configuration imposée (paramètre *Configuration préférée*).

Le panneau *Pilotage → Solutions du modèle inverse* les affiche toutes.

Précision : sur 300 poses aléatoires, l’aller-retour direct → inverse → direct donne une erreur **< 10⁻⁸ m** pour toutes
les solutions. La configuration d’origine est retrouvée dans ≥ 295 cas sur 300 ; les autres sont des cas singuliers.

### Cas singuliers

| Singularité | Condition | Effet | Indicateur (onglet *Analyse*) |
|---|---|---|---|
| **Poignet** | θ₅ = 0 : axes 4 et 6 alignés | seule la somme θ₄ + θ₆ est définie | \|sin θ₅\| → 0 |
| **Coude** | bras tendu (a₂ et d₄ alignés) | limite de portée, vitesses radiales impossibles | σmin → 0 |
| **Épaule** | centre du poignet sur l’axe J1 | θ₁ indéterminé | distance poignet ↔ axe J1 → 0 |

En singularité de poignet, le solveur conserve θ₄ à sa valeur courante et reporte la rotation sur θ₆, ce qui évite un
demi-tour inutile. En singularité d’épaule, il conserve de même θ₁. Les trajectoires MoveL/MoveC qui passent près d’une singularité sont automatiquement ralenties
(re-temporisation), au lieu de demander des vitesses articulaires infinies.

## Modèle inverse numérique

Il sert pour toute autre géométrie : poignet décalé de type UR (préréglage *Cobot à poignet décalé*), convention MDH,
axes prismatiques, nombre d’axes différent de 6. Méthode : **moindres carrés amortis** (Levenberg-Marquardt).

Δq = Jᵀ · (J·Jᵀ + λ²·I)⁻¹ · e, où e = [e_position ; L·e_orientation]

- λ (*IK — amortissement*) s’adapte : il est réduit si l’itération progresse et augmenté sinon.
- L (*poids orientation*, en m/rad) équilibre millimètres et radians.
- Les butées sont respectées à chaque itération. Faute de convergence, le solveur relance jusqu’à 12 départs.
- Arrêt quand les erreurs passent sous *tolérance position* et *tolérance orientation*, ou à *itérations max*.
- Validation : 38 convergences au moins sur 40 poses aléatoires du cobot à poignet décalé.

## Jacobien et manipulabilité

Jacobien géométrique (6 × n) : colonne i = [zᵢ₋₁ × (p_tcp − pᵢ₋₁) ; zᵢ₋₁] pour une liaison rotoïde et [zᵢ₋₁ ; 0] pour
une liaison prismatique. Un test le vérifie contre des différences finies du modèle direct.

- **Manipulabilité** (Yoshikawa) : w = |det J| pour 6 axes, √det(J·Jᵀ) sinon.
- **Conditionnement** : σmax/σmin (valeurs singulières).
- **Dextérité** (jauge de la vue 3D) : manipulabilité rapportée à celle de la pose Home. La jauge passe à l’orange puis
  au rouge quand σmin descend sous 0,03 puis 0,01, c’est-à-dire près d’une singularité.

Le jacobien sert :

- au jog cartésien ;
- à la commande en impédance (τ = Jᵀ·F) ;
- au calcul des efforts extérieurs (τ = −Jᵀ·F_ext, vérifié par test) ;
- au solveur numérique.

## Espace de travail

![Nuage de l’espace de travail](img/studio-analyse.png)

- Allonge horizontale maximale du TCP : **593 mm**. Au centre du poignet : a₂ + d₄ = 460 mm.
- Hauteur du TCP : de **−344 mm** (sous le plan de la table, donc inaccessible en pratique) à **+753 mm**.
- **Débattements mécaniques réels**, déterminés en balayant la CAO sur les capsules de collision des pièces :

  | Axe | Débattement libre | Pièces en contact | Avec la pince vers le bas, contact de la table à |
  |---|---|---|---|
  | J1 | ±180° | — | — |
  | J2 | ±140° | socle ↔ bras | −115° / +55° |
  | J3 | −180° … +65° | bras ↔ avant-bras | +35° |
  | J4 | ±180° | — | — |
  | J5 | ±120° | poignet ↔ chape | — |
  | J6 | ±180° | — | — |

  Les butées logicielles (J2 ±130°, J3 +60°, J5 ±115°…) laissent 5 à 10° de marge sur ces valeurs.
- La **détection de collision** en simulation utilise ces capsules, extraites de la CAO (53 capsules). Elle couvre
  l’auto-collision et la table. Les paires de capsules toujours en contact par construction sont ignorées
  automatiquement.

## Exports

- **URDF** (ROS, MoveIt) et **MJCF** (MuJoCo) : chaîne cinématique, butées, vitesses, masses, centres de masse et
  inerties. Les repères sont convertis depuis la table DH, et un test vérifie que les poses du TCP sont identiques à celles
  du modèle direct (avec base déplacée et axes inversés).
- Les inerties exportées viennent de la CAO : volumes × densité × remplissage des pièces imprimées, plus les masses
  réelles des moteurs, roulements, tubes et visserie (`studio/core/generated/massprops.js`).
