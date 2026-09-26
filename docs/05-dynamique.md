# 5. Dynamique et modèles d’actionneurs

Code : `studio/core/dynamics.js` (modèle rigide), `studio/core/actuators.js` (moteurs, réducteurs, frottements),
`studio/core/simulator.js` (intégration temps réel). Tests : `tests/dynamics.test.mjs`.

Le simulateur d’ORION Studio a un objectif précis : **reproduire les défauts qui comptent sur un bras imprimé**, pour
qu’un réglage validé à l’écran se comporte de la même façon sur le vrai robot. Ces défauts sont :

- la flexibilité et le jeu des réducteurs ;
- le frottement ;
- la chute de couple des pas-à-pas avec la vitesse ;
- la **perte de pas** ;
- la saturation de couple.

## Modèle rigide : τ = M(q)·q̈ + C(q, q̇)·q̇ + g(q)

- **Newton-Euler récursif (RNEA)** dans le repère monde, valable en DH standard comme en DH modifiée. Une passe
  aller propage les vitesses et accélérations des segments ; une passe retour propage les efforts. Le résultat donne les
  couples articulaires, mais aussi les **efforts dans chaque liaison** (force axiale et radiale, moment de basculement),
  affichés dans *Analyse* pour choisir les roulements.
- **Matrice d’inertie M(q)** : obtenue colonne par colonne avec le RNEA (q̇ = 0, g = 0, q̈ = eᵢ). Elle est symétrique
  définie positive, ce que vérifie une factorisation de Cholesky dans les tests.
- **Gravité g(q)** : RNEA avec q̇ = q̈ = 0. Un test compare le couple de J2 au moment des poids des segments 2 à 6.
- **Dynamique directe** : q̈ = M⁻¹·(τ − C·q̇ − g − τ_frottement + Jᵀ·F_ext). Sans frottement, l’énergie est conservée
  (dérive < 0,01 % après 1 000 pas RK4, vérifié par test).
- **Charge utile** : masse ponctuelle, décalage et inertie propre réglables. Elle s’ajoute au segment 6. Une pièce saisie
  par la pince en simulation modifie automatiquement la charge.

### Paramètres inertiels

Pour chaque segment : masse, centre de masse (repère DH du segment), tenseur d’inertie au centre de masse (Ixx, Iyy, Izz,
Ixy, Ixz, Iyz). Les valeurs par défaut sont **calculées par la CAO** (`npm run build:cad`) :

- pièces imprimées : volume × densité (1,24 g/cm³, valeur type PLA/PETG) × taux de remplissage estimé ;
- moteurs, roulements, tubes aluminium, goupilles et visserie : masses réelles.

Ces valeurs dépendent du matériau et des réglages d’impression réels. Pesez les segments imprimés et corrigez les masses
dans *Paramètres → Dynamique* : la compensation de gravité (modes MIT, PID, impédance) et le dimensionnement en dépendent
directement.

## Modèle d’actionneur : l’articulation flexible

Chaque axe est modélisé comme une **articulation flexible** (modèle de Spong) :

```
 moteur (inertie ramenée N²·J_rotor)  ──[ réducteur : raideur k, amortissement c, jeu b ]──  segment (dynamique rigide)
         θ_rotor (ramené en sortie)                                                          q
```

- **Inertie rotor ramenée** : N²·J_rotor. Elle domine souvent l’inertie vue par l’axe : pour J2 (NEMA23 76 mm, 30:1),
  30² × 480 g·cm² = 0,043 kg·m², soit une valeur **comparable à l’inertie du bras** lui-même. Elle entre dans
  l’auto-réglage des gains.
- **Transmission** : τ_t = k·dz(θ_rotor − q) + c·(ω_rotor − q̇)·contact. dz est une **zone morte lissée** de largeur
  égale au jeu b :
  dz(δ) = s·[softplus((δ − b/2)/s) − softplus((−δ − b/2)/s)], avec s = b/24.
  Ce lissage garde un modèle dérivable, donc stable à l’intégration implicite, tout en reproduisant le « flottement »
  dans le jeu et le choc de reprise.
- **Valeurs par défaut** (MAKER) :
  - raideur : 2 500 à 4 000 N·m/rad pour J1–J3, 600 à 800 N·m/rad pour le poignet ;
  - jeu : 0,2 à 0,4°.

  Ce sont des estimations pour des cycloïdes en PETG à goupilles acier. **Mesurez-les** sur vos modules (voir
  [12. Calibration](12-calibration.md)).

### Pas-à-pas : couple magnétique et perte de pas

Un moteur hybride 1,8° a **Nr = 50 dents** (4 pas entiers par période électrique). Son couple magnétique rappelle le
rotor vers la position commandée :

τ_moteur = T(ω)·sin(Nr·(θ_commande − θ_rotor))

- **T(ω)** est le couple de décrochage : environ 85 % du couple de maintien, au courant réglé du driver. Au-delà de la
  **vitesse de coin** ωc = V / (I·Nr·L + k_e), l’inductance empêche le courant de s’établir et le couple décroît en
  1/ω.
  - V : tension d’alimentation ;
  - I : courant réglé ;
  - L : inductance de phase ;
  - k_e ≈ couple de maintien / courant nominal.

  C’est pourquoi une alimentation **36 V ou plus** est recommandée.
- **Perte de pas** : si la charge écarte le rotor de plus d’une demi-période électrique (2 pas entiers) de sa consigne,
  il saute sur la dent suivante. Le simulateur détecte ce saut et compte les pas perdus (événement *perte de pas* dans le
  journal, compteur par axe). La position réelle reste décalée, exactement comme sur le vrai robot en boucle ouverte.
- **Micro-pas** : la consigne est quantifiée à la résolution micro-pas en sortie, par exemple 0,0045° pour J1 à 16
  micro-pas et 25:1.

### Brushless QDD (mode MIT) et servos

L’actionneur est une source de couple limitée par une **droite couple-vitesse** : couple crête à vitesse nulle, nul à
la vitesse maximale. En freinage, le couple crête reste disponible. Le couple commandé vient soit de la loi MIT
(Kp, Kd, τff), évaluée **en continu** comme dans le driver du moteur, soit du correcteur choisi.

## Frottements

Côté sortie de chaque axe, modèle de **Stribeck lissé** :

τ_f(v) = [F_c + (F_s − F_c)·exp(−(v/v_s)²)]·tanh(v/ε) + b·v

- F_c : frottement sec (Coulomb) ;
- F_s : adhérence (stiction) ;
- v_s : vitesse de Stribeck ;
- b : frottement visqueux ;
- ε = 2 mrad/s : lissage autour de v = 0.

Les correcteurs peuvent anticiper le frottement avec un estimateur sans le terme d’adhérence (option *anticipation
frottements*).

## Intégration numérique (temps réel)

L’état comprend, pour chaque axe, la position et la vitesse du segment (q, v) et celles du rotor ramené (θ, u).

Les raideurs magnétiques des pas-à-pas et les raideurs de transmission rendent le système **raide** : un Euler explicite
exigerait un pas inférieur à 10 µs. Le simulateur utilise donc un **Euler linéairement implicite** et élimine les
rotors par **complément de Schur** :

```
(Ja + α + β)·Δu − (α − γ)·Δv = dt·(τm − τt)
[M + diag(α + δ − α(α−γ)/s)]·Δv = dt·(τt − b − τf + τext) + diag(α/s)·r
```

- α = dt·(dt·k + c) : transmission ;
- β : raideur et amortissement propres du moteur ;
- γ : PD continu du mode MIT ;
- δ : frottement visqueux et butées.

Résultats :

- **pas de temps 0,25 ms** par défaut (réglable), stable même avec le jeu et la perte de pas ;
- matrice d’inertie recalculée à 1 kHz ;
- **6 à 9 fois plus rapide que le temps réel** dans un navigateur courant ;
- le paramètre *Facteur temps réel* ralentit ou accélère la simulation ;
- correcteurs échantillonnés à leur propre fréquence (*Commande → Fréquence de boucle*), avec bloqueur d’ordre zéro,
  comme sur un vrai contrôleur ;
- bruit de mesure optionnel (*Simulation → Bruit de mesure*) pour tester la robustesse des gains.

## Détection de collision (simulation)

- **Auto-collision et table** : les 53 capsules extraites de la CAO sont testées sur la **consigne**, à chaque cycle de
  commande. Si un contact est prévu pendant un mouvement, le simulateur déclenche un *arrêt de protection* signalé dans
  le journal. L’affichage des capsules en contact (vue 3D) est rafraîchi à 4 Hz.
- **Choc** (option *Détection de collision*) : le simulateur calcule un résidu r = τ_transmis − τ̂_frottement −
  (M̂·q̈ + b̂), qui estime le couple extérieur. Ce résidu est filtré sur environ 20 ms. S’il dépasse le *seuil de
  collision* (N·m), le mouvement s’arrête. Poussez le bras (`Maj` + glisser) pour l’essayer.
