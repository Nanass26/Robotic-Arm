# 6. Commande et réglage

Code : `studio/core/control.js` (lois de commande, auto-réglage), `studio/core/trajectory.js` (profils, MoveJ/L/C,
lissage), firmware `firmware/orion_fw` (pas-à-pas) et `firmware/mit_bridge` (mode MIT).

Tous les réglages se font dans **Paramètres → Commande** et **Paramètres → Limites / Trajectoires**. On les teste
immédiatement en simulation : onglets *Analyse → Réponse indicielle* et *Courbes → Erreur de suivi / Charge moteur*.

## Vue d’ensemble des modes

| Mode | Loi de commande | Quand l’utiliser |
|---|---|---|
| **Pas-à-pas** | position commandée micro-pas par micro-pas, sans retour | ORION-6 MAKER (le vrai robot) |
| **PID** | τ = Kp·e + Ki·∫e + Kd·(q̇* − q̇) + τ_ff | servos à codeur, étude des gains |
| **Mode MIT** | τ = Kp·(p* − p) + Kd·(v* − v) + τff | ORION-6 PRO, moteurs Damiao/CubeMars/MIT Cheetah |
| **Couple calculé** | τ = M̂(q)·(q̈* + ω²·e + 2ζω·ė) + b̂(q, q̇) | suivi précis à grande vitesse |
| **Impédance** | τ = Jᵀ·(K·Δx + D·Δẋ) + ĝ(q) − D_null·q̇ | contact, insertion, interaction humaine |
| **Compensation de gravité** | τ = ĝ(q) − D_null·q̇ | guidage à la main, apprentissage |

L’anticipation τ_ff peut contenir trois termes, cumulables et activables séparément :

- la **gravité** ĝ(q) ;
- l’**inertie** M̂(q)·q̈* ;
- le **frottement** (Coulomb + visqueux dans le sens de q̇*).

Les grandeurs « chapeau » (M̂, ĝ, b̂) viennent du modèle dynamique. Le paramètre *Erreur de modèle* le fausse
volontairement pour tester la robustesse.

## Le mode MIT en détail

Le mode MIT vient du contrôleur de moteur développé par Ben Katz pour le robot **Mini Cheetah** du MIT. De nombreux
moteurs QDD l’ont repris depuis (Damiao, CubeMars, MyActuator…). Le driver du moteur exécute à haute fréquence
(plusieurs kHz, dans sa boucle de courant) :

**τ = Kp·(p* − p) + Kd·(v* − v) + τff**

Le contrôleur maître envoie, à 200 Hz ou plus, une trame CAN de 8 octets avec les cinq consignes p*, v*, Kp, Kd et τff.
La boucle de raideur et d’amortissement est donc **fermée dans le moteur**, sans retard de bus. Le maître ne s’occupe
que de la trajectoire et du modèle (compensation de gravité, inertie).

Avec un seul format, on obtient ainsi plusieurs comportements :

| Réglage | Comportement |
|---|---|
| Kp élevé, Kd moyen, τff = ĝ(q) | asservissement de position raide, qui tient la charge sans erreur statique |
| Kp faible, Kd faible, τff = ĝ(q) | bras **compliant** : suit la trajectoire mais cède aux chocs |
| Kp = 0, Kd faible, τff = ĝ(q) | **gravité compensée** : le bras flotte, on le guide à la main |
| Kp = 0, Kd > 0, τff = 0 | **amortissement** : état de sécurité (défaut, perte de communication) |

C’est exactement l’architecture des bras de recherche de type **YAM** (i2rt) : commande PD + compensation de gravité,
moteurs DM sur bus CAN à 1 Mbit/s, passage en amortissement en cas de perte des commandes.

### Encodage des trames (identique dans ORION Studio et dans le firmware)

| Octets | Contenu | Bits | Plage |
|---|---|---|---|
| 0–1 | p* | 16 | ±P_MAX (12,5 rad) |
| 2 + ½ octet 3 | v* | 12 | ±V_MAX (8 rad/s pour DM4340, 30 rad/s pour DM4310) |
| ½ octet 3 + 4 | Kp | 12 | 0 … KP_MAX (500) |
| 5 + ½ octet 6 | Kd | 12 | 0 … KD_MAX (5) |
| ½ octet 6 + 7 | τff | 12 | ±T_MAX (28 N·m pour DM4340, 10 N·m pour DM4310) |

La conversion est linéaire : u = round((x − min)·(2ⁿ − 1)/(max − min)). **Les plages doivent correspondre aux réglages
du moteur**, faits avec l’outil du fabricant. Sinon les consignes sont fausses d’un facteur d’échelle. Le retour du
moteur (format Damiao) donne l’identifiant, le code d’erreur, p, v, τ et les températures du MOSFET et du bobinage.

### Gains MIT par défaut (ORION-6 PRO)

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| Kp (N·m/rad) | 80 | 120 | 80 | 30 | 25 | 15 |
| Kd (N·m·s/rad) | 2,5 | 3,5 | 2,5 | 1,0 | 0,8 | 0,5 |
| Kd amortissement (défaut) | 5 | 5 | 5 | 2 | 1,6 | 1 |

Ordre de grandeur : pour J2 avec Kp = 120 N·m/rad, une erreur de 1° produit 2,1 N·m. Sans compensation de gravité, les
10,7 N·m de gravité maximale de J2 (bras tendu, avec la charge nominale) feraient donc fléchir le bras d’environ 5°.
Avec τff = ĝ(q), l’erreur statique ne dépend plus que de l’erreur du modèle de masse. La marge de couple de la variante
PRO est de ×1,68 sur J2 et ×3,8 ou plus sur les autres axes (27 N·m crête pour les DM4340, 7 N·m pour les DM4310).

## Réglage : méthode pas à pas

### 1. Vérifier le modèle

- Pose *Zéro* : bras vertical, avant-bras horizontal. Activez les *Repères DH* pour contrôler les axes.
- Masses : pesez vos segments imprimés et corrigez *Dynamique → masses*. Une compensation de gravité juste en dépend.
- *Analyse → Dimensionnement* : toutes les marges doivent être au moins *Juste* (≥ ×1,1), idéalement *OK* (≥ ×1,5).
  Sinon, trois leviers :
  - réduire *Accélération max* et *Jerk max* ;
  - augmenter le courant du driver (sans dépasser le courant nominal du moteur) ;
  - changer de moteur ou de réduction.

### 2. Auto-réglage (placement de pôles)

*Analyse → Auto-régler les gains* calcule pour chaque axe, dans la pose courante :

- **J** : inertie vue par l’axe = M_ii(q) + N²·J_rotor ;
- **Kp = J·ω²**, **Kd = 2·ζ·J·ω**, **Ki = Kp·ω/10**, avec ω = 2π·f_bande ;
- les gains MIT : **Kp_MIT = min(Kp, KP_MAX)** et **Kd_MIT = min(Kd, KD_MAX)**.

Les valeurs par défaut sont une bande passante de **8 Hz** et **ζ = 0,9** : réponse rapide, dépassement quasi nul.

- Augmentez la bande passante pour plus de raideur et un meilleur suivi, jusqu’à l’apparition d’oscillations. Celles-ci
  viennent du jeu, de la flexibilité des réducteurs imprimés et du retard d’échantillonnage.
- Règle pratique : **f_bande < f_résonance/3**. La fréquence de résonance vaut f_rés = √(k/J)/2π, avec k la raideur du
  réducteur.
- Avec des cycloïdes imprimées (k ≈ 2 500 N·m/rad), on obtient typiquement f_rés ≈ 15–40 Hz selon la pose. D’où une
  bande passante de 5 à 10 Hz.

### 3. Réponse indicielle

*Analyse → Réponse indicielle* : un échelon de 2° est appliqué sur un axe dans un simulateur séparé. Le robot affiché
n’est pas perturbé. Lisez :

| Symptôme | Cause probable | Correction |
|---|---|---|
| dépassement > 20 %, oscillation amortie | Kd trop faible ou Kp trop fort | augmenter ζ (ou Kd) |
| oscillation entretenue (« pompage ») autour de la consigne | jeu du réducteur + Ki | réduire Ki, ajouter une *zone morte* ≈ jeu/2 |
| montée lente, erreur finale | Kp faible, gravité non compensée | activer l’*anticipation gravité*, augmenter Kp |
| vibration haute fréquence | Kd bruité, fréquence de boucle basse | baisser le *filtre dérivée*, augmenter la *fréquence de boucle* |
| saturation du couple (*Courbes → Couples*) | échelon trop grand pour les gains | normal sur grand échelon, sinon réduire les gains |

### 4. Suivi de trajectoire

Lancez l’exemple *Test de répétabilité* et observez *Courbes → Erreur de suivi*.

- Activez l’**anticipation d’inertie** (M̂·q̈*) : l’erreur de suivi chute d’un facteur 5 à 10 en mouvement rapide.
- En pas-à-pas, surveillez la **Charge moteur**. Au-delà de 70–80 % du couple disponible, le risque de perte de pas
  devient réel à cause des variations de frottement et de l’usure. Réduisez l’accélération ou le jerk, ou la vitesse si
  la charge monte avec elle (chute de couple au-delà de la vitesse de coin).

### 5. Réglage spécifique ORION-6 MAKER (pas-à-pas)

Il n’y a pas de gains à régler : la précision vient du **dimensionnement** et des **profils**.

| Paramètre | Conseil |
|---|---|
| Profil | **courbe en S** : jerk limité, beaucoup moins de vibrations et de pertes de pas qu’un trapèze |
| Micro-pas | 16 (bon compromis douceur / fréquence de pas ; 45 kpas/s max par axe dans le firmware) |
| Courant de marche | 80 à 100 % du nominal ; au-delà, le moteur chauffe sans gain de couple |
| Alimentation | 36 V ou plus pour garder du couple à vitesse élevée (vitesse de coin ∝ V) |
| Vitesse max | *Analyse → Dimensionnement* : survolez un axe pour voir la vitesse moteur et la vitesse de coin |
| Accélération / jerk | commencez à 50 % et augmentez tant que la charge moteur reste < 70 % |

### 6. Réglage spécifique ORION-6 PRO (mode MIT)

1. Réglez sur chaque moteur, avec l’outil du fabricant, le **mode MIT**, l’identifiant CAN (1–6), l’identifiant maître
   et les plages P_MAX, V_MAX et T_MAX. Reportez les mêmes plages dans *Actionneurs* (P_MAX…KD_MAX).
2. Commencez avec des gains **divisés par 2**, sans charge. Activez *Anticipation gravité* : le préréglage PRO la fournit
   dans τff.
3. Mode *Compensation de gravité* : le bras doit flotter. S’il tombe ou remonte, corrigez les masses et les centres de
   masse du segment concerné.
4. Remontez Kp jusqu’aux valeurs par défaut en contrôlant la *Réponse indicielle*. Kd doit donner ζ ≈ 0,7–1.
5. **Surveillez les températures** : la commande `FB` du pont affiche les températures MOSFET et bobinage. Un Kp élevé
   avec une erreur statique non compensée (masse fausse) fait chauffer le moteur en permanence.

## Trajectoires

### Profils de vitesse (MoveJ et re-temporisation)

| Profil | Continuité | Durée pour Δq, v, a donnés | Usage |
|---|---|---|---|
| Trapèze | accélération discontinue | la plus courte | robots rigides |
| **Courbe en S** (7 phases) | accélération continue, jerk borné | un peu plus longue (dépend du jerk) | **par défaut**, réducteurs imprimés |
| Quintique | jerk continu | plus longue | mouvements très doux |
| Cycloïdal | accélération sinusoïdale | plus longue | mouvements « organiques » |

- **MoveJ** synchronise tous les axes : chacun arrive en même temps, avec un profil homothétique. L’axe le plus
  contraint fixe la durée.
- **MoveL / MoveC** : trajectoire cartésienne (ligne ou arc par 3 points) avec interpolation d’orientation SLERP.
  Elle est discrétisée à *Échantillonnage cartésien* (250 Hz par défaut) par le modèle inverse, puis
  **re-temporisée** si une vitesse ou une accélération articulaire serait dépassée (près d’une singularité, par exemple).
- **Lissage (fly-by)** : avec une zone `z` > 0, le mouvement suivant commence avant la fin du précédent, par
  superposition des deux profils. Le robot ne s’arrête pas aux points intermédiaires.
- **Arrêt contrôlé** (touche `Espace`) : freinage à la décélération maximale le long de la trajectoire.
  **Arrêt d’urgence** (bouton rouge *STOP* ou `Échap`) : coupure immédiate du couple.

### Limites par défaut (ORION-6 MAKER)

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| vitesse (°/s) | 120 | 90 | 110 | 170 | 170 | 200 |
| accélération (°/s²) | 300 | 200 | 300 | 600 | 600 | 700 |
| jerk (°/s³) | 3 000 | 2 000 | 3 000 | 6 000 | 6 000 | 7 000 |

Côté cartésien : 250 mm/s et 1 m/s² au TCP, 90°/s en rotation. Le potentiomètre de *Vitesse* de la barre du haut
multiplie toutes les vitesses. Il vaut 50 % au démarrage.

## Côté firmware

- **Pas-à-pas** (`orion_fw`) :
  - `MJ` exécute un profil trapézoïdal synchronisé 25/50/25 % ;
  - `SP` reçoit le flux de consignes du jumeau numérique ;
  - un limiteur de vitesse et d’accélération ultime, qui ne dépasse jamais la cible, protège les moteurs contre toute
    consigne incohérente.
- **Pont MIT** (`mit_bridge`) :
  - envoi des trames à 400 Hz par moteur (réglable) ;
  - raccord progressif de 300 ms au démarrage d’un flux ;
  - maintien avec le dernier couple de gravité si les consignes s’interrompent (chien de garde) ;
  - amortissement (Kp = 0) en cas de défaut moteur.

Voir [11. Firmware et protocole](11-firmware-protocole.md).
