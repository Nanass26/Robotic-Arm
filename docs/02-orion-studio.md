# 2. Guide d’ORION Studio

ORION Studio est le logiciel de simulation, de réglage, de programmation et de pilotage d’ORION-6. Il tourne dans un
navigateur récent (Chrome, Edge, Firefox, Safari) et fonctionne entièrement **hors ligne** : aucune donnée ne quitte
votre ordinateur.

- **Lancer depuis les sources** : `npm install` puis `npm start`, et ouvrir http://localhost:8080/studio/.
- **Lancer sans installation** : ouvrir `dist/ORION-Studio.html`, fichier unique produit par `npm run build:app`.
- **Robot réel** : la liaison USB (Web Serial) demande **Chrome ou Edge sur ordinateur**, avec le logiciel ouvert en
  local (fichier ou `npm start`).

![Interface complète](img/studio-clair.png)

## Organisation de l’écran

| Zone | Contenu |
|---|---|
| **Barre du haut** | mode de commande, potentiomètre de **vitesse** (0–100 %), état des moteurs et du robot, affichage des panneaux, pause de la simulation, thème clair/sombre, bouton rouge **STOP** (arrêt d’urgence, touche `Échap`) |
| **Gauche** | *Pilotage*, *Programme*, *Assistant IA* |
| **Centre** | vue 3D : pose du TCP, configuration (épaule/coude/poignet), jauge de dextérité, barre d’outils de vue |
| **Droite** | *Paramètres*, *Analyse*, *Pièces 3D*, *Aide* |
| **Bas** | *Courbes*, *Robot réel*, *Journal* |

Sur téléphone (largeur ≤ 620 px), les onglets passent dans un tiroir sous la vue 3D. Les panneaux se masquent avec les
trois boutons de disposition de la barre du haut.

### Vue 3D

- **Souris** : clic gauche pour tourner, clic droit (ou deux doigts) pour déplacer, molette pour zoomer.
- **Gizmo du TCP** : faites glisser les flèches (translation, `T`) ou les anneaux (rotation, `R`). Le modèle inverse
  calcule les angles en direct et le robot suit, dans les limites de vitesse. `G` affiche ou masque le gizmo.
- **`Maj` + glisser sur le bras** : applique une force sur le robot. C’est utile en *compensation de gravité* et en
  *impédance*.
- **Barre d’outils** :
  - vues iso/face/côté/dessus (`1` à `4`) ;
  - trace du TCP ;
  - repères DH ;
  - **modèle de collision** (capsules, qui s’affichent en rouge en cas de contact) ;
  - ombres.
- Le modèle 3D affiché est **la CAO réelle des pièces**, générée dans le navigateur. Si la géométrie DH ne correspond
  plus à l’architecture ORION-6 (autre robot), un modèle simplifié la remplace.

## Pilotage

- **Poses** : *Home*, *Repos*, *Zéro* et *Définir Home ici*. Le bouton *Moteurs* met le robot sous tension ou hors
  tension (simulé).
- **Articulations** : maintenez `−`/`+` pour jogger, ou faites glisser le curseur. Sous chaque axe, deux lignes
  s’affichent :
  - la **charge** (couple demandé / couple disponible) ;
  - la **consigne** en cours.
- **Cartésien (TCP)** : boutons `X± Y± Z± Rx± Ry± Rz±`, dans le repère **Base** ou **Outil**. Vous pouvez aussi saisir
  une pose (mm, degrés) puis choisir *MoveJ* (articulaire) ou *MoveL* (ligne droite).
- **Solutions du modèle inverse** : liste les 8 configurations qui atteignent la pose courante :
  - épaule avant ou arrière ;
  - coude haut ou bas ;
  - poignet normal ou retourné.

  Un clic sur une solution y déplace le robot.
- **Pince** : *Fermer*, *Ouvrir* ou curseur de course. La simulation détecte la saisie d’un cube entre les doigts.
- **Entrées/sorties** : 8 sorties et 8 entrées TOR, simulées. Une fois le robot réel connecté, les sorties lui sont
  envoyées (`DO`) et les entrées reflètent les siennes.
- **Scène** : ajouter ou replacer les cubes de démonstration.

## Programmes (langage ORL)

L’onglet *Programme* contient un éditeur à lignes numérotées, des exemples (*Prise et dépose*, *Dessin : carré +
cercle*, *Test de répétabilité*), l’exécution pas à pas (ligne courante en surbrillance), la pause et l’arrêt. Il permet
aussi d’ouvrir et d’enregistrer des fichiers `.orl`. **Mémoriser le point** insère la position actuelle
(`POINT Pn = J(…)`) puis `MOVEJ Pn`. `Ctrl`+`Entrée` exécute le programme.

Les mots-clés existent en français **et** en anglais, et la casse est indifférente :

```text
# commentaire                         ; commentaire en fin de ligne
VITESSE 60                            | SPEED 60       vitesse globale en %
POINT A = J(0, -20, 45, 0, 65, 0)     pose articulaire (degrés)
POINT B = X(260, -120, 110, 180, 0, 0) pose cartésienne : x, y, z (mm), roulis, tangage, lacet (°)
MOVEJ A v=80 a=50 z=10                articulaire : v, a en % ; z = zone de lissage (mm)
MOVEL B v=100 z=5                     ligne droite : v en mm/s
MOVEL REL(0, 0, -50)                  déplacement relatif dans le repère de base (mm [, °])
MOVEL OUTIL(0, 0, 30)                 | TOOL(...)    relatif au repère outil
MOVEC Pvia B v=50                     arc de cercle passant par Pvia
PINCE OUVRIR | FERMER | 50            | GRIP OPEN | CLOSE | 50
ATTENDRE 0.5                          | WAIT 0.5 (secondes)
SORTIE 1 ON                           | SETDO 1 1
ATTENDRE_ENTREE 2 ON timeout=5        | WAITDI 2 1 timeout=5
REPETER 3 … FIN                       | REPEAT 3 … END
MESSAGE "texte"                       | PRINT "texte"
PAUSE                                 attend « Reprendre »
HOME | REPOS                          raccourcis vers les poses nommées
```

- **Zone de lissage `z`** : le mouvement suivant démarre avant la fin du précédent (*fly-by*). La trajectoire est plus
  fluide et plus rapide, mais elle ne passe pas exactement par le point.
- **MoveL/MoveC** : la trajectoire cartésienne est discrétisée par le modèle inverse puis re-temporisée si une limite
  articulaire (vitesse ou accélération) serait dépassée. Le mouvement n’échoue donc jamais sur une vitesse, mais peut
  ralentir près d’une singularité.
- Les **erreurs de syntaxe** sont signalées avec leur numéro de ligne avant toute exécution.

## Assistant IA (langage naturel)

L’assistant transforme une phrase en lignes ORL, **affichées avant exécution** : vous choisissez *Exécuter* ou
*Ajouter au programme*. Il fonctionne sans connexion ni service externe, par règles, en français et en anglais. La
**dictée vocale** (micro) utilise la reconnaissance vocale du navigateur, si elle est disponible.

Exemples :

- « monte de 5 cm puis ouvre la pince »
- « tourne la base de 30 degrés à gauche »
- « mets l’axe 3 à −20° »
- « va à la position repos »
- « prends le cube en 260 −120 »
- « dessine un cercle de 4 cm »
- « vitesse 30 % »
- « arrête-toi »
- « salue »

## Paramètres

Tous les paramètres du robot sont modifiables en direct, soit **179 réglages** et **567 valeurs** en comptant chaque
axe. La liste complète figure dans [3. Référence des paramètres](03-parametres.md).

- **Recherche** : tapez « Kp », « jeu », « courant », « vitesse »… Les sections filtrées s’ouvrent automatiquement.
- **Aide** : survolez ou touchez `?` pour obtenir l’explication, l’unité et la plage de chaque réglage.
- **Préréglages** :
  - *ORION-6 MAKER* : pas-à-pas et cycloïdes ;
  - *ORION-6 PRO* : QDD en mode MIT ;
  - *Cobot à poignet décalé* : une géométrie de type UR, qui montre le solveur inverse numérique.
- **Exports** :

  | Format | Usage |
  |---|---|
  | `JSON` | configuration complète, rechargeable avec *Importer* |
  | `URDF` | ROS, MoveIt, Isaac |
  | `MJCF` | MuJoCo |
  | `Firmware .h` | en-tête `orion_config.h` à recompiler |
  | `SET` | commandes série pour régler le firmware sans recompiler |

- La configuration courante est **sauvegardée automatiquement** dans le navigateur. Le bouton ⟲ (deux clics pour
  confirmer) rétablit les valeurs par défaut.
- Les modifications de géométrie (table DH) régénèrent la CAO. Les pièces suivent les longueurs `d1`, `a2` et `d4`.

## Analyse

![Onglet Analyse](img/studio-analyse.png)

- **Dimensionnement des moteurs** : pour chaque axe, compare le couple disponible (moteur × réduction × rendement) au pire
  cas trouvé sur 2 500 configurations. Ce pire cas additionne la gravité avec la charge nominale, l’inertie à
  l’accélération maximale et le frottement sec. La marge est jugée *OK* au-dessus de ×1,5, *Juste* entre ×1,1 et ×1,5,
  *Insuffisant* en dessous. L’outil calcule aussi la **charge utile maximale** qui garde une marge statique de 1,2.
  Survolez une ligne pour voir la vitesse moteur, la vitesse de coin et la réduction conseillée.
- **Réponse indicielle & auto-réglage** : applique un échelon (par exemple 2°) sur un axe dans un simulateur séparé,
  trace la réponse et mesure le dépassement et le temps d’établissement. *Auto-régler les gains* calcule Kp, Ki, Kd et
  les gains MIT par placement de pôles (voir [6. Commande](06-commande-et-reglage.md)).
- **Espace de travail** : nuage de 20 000 positions du TCP, coloré selon la **manipulabilité** (clair : proche d’une
  singularité). L’outil donne aussi l’allonge et les hauteurs extrêmes.
- **Efforts dans les articulations** : efforts axial et radial, moment de basculement et couple de chaque liaison
  (Newton-Euler), utiles pour vérifier les roulements.
- **Conditionnement & singularités** : dans la pose courante, manipulabilité, plus petite valeur singulière σmin et
  conditionnement du jacobien. Deux indicateurs de singularité s’y ajoutent :
  - poignet : |sin θ5| ;
  - épaule : distance entre le centre du poignet et l’axe J1.

## Courbes

Positions, vitesses, **erreur de suivi**, couples, **charge moteur** (en % du couple disponible), position et vitesse du
TCP. Les courbes s’affichent sur une fenêtre glissante réglable (10 s par défaut). Les consignes apparaissent en
pointillés, et la légende donne les valeurs en direct. *CSV* exporte toute la télémétrie enregistrée (fréquence réglable
dans *Simulation*).

## Pièces 3D

Liste des pièces imprimables générées par la CAO, avec leur encombrement, leur masse, leur matière et leur remplissage.
Un clic sur une pièce la met en évidence dans la vue 3D. Le panneau permet de télécharger chaque **STL** (déjà orienté
pour l’impression), **tout le jeu en .zip** et la **nomenclature (CSV)**.

## Robot réel

Connexion USB à la Teensy 4.1, avec le protocole ORION-ASCII détaillé dans
[11. Firmware et protocole](11-firmware-protocole.md) :

1. **Connecter (USB)** : choisissez le port de la Teensy. Le logiciel envoie `PING` puis `STREAM 50`, qui demande l’état
   du robot 50 fois par seconde.
2. **Envoyer la config** : transmet les paramètres matériels (`SET …`) puis `SAVE` (EEPROM).
3. **Activer** : alimente les drivers (`EN 1`).
4. **Référencer** :
   - pas-à-pas : prise d’origine sur capteurs (`HOME`) ;
   - pont MIT : `ZERO`, qui déclare la pose actuelle comme pose de repos.
5. **Synchroniser** : place le simulateur sur la position mesurée du robot.
6. **Jumeau numérique** : le robot réel suit en continu la consigne du simulateur. Les consignes sont horodatées et
   lissées par le firmware, et le pont MIT reçoit en plus la compensation de gravité. Tout ce que vous faites dans le
   logiciel (jog, gizmo, programmes, assistant) est alors exécuté par le bras.
7. **Miroir** : affiche la position mesurée du robot réel comme un « fantôme » superposé.

Le panneau affiche aussi les **trames CAN mode MIT** calculées pour chaque moteur (identifiant, 8 octets, p* et τff).
Une ligne de commande brute permet d’envoyer n’importe quelle commande, par exemple `STAT`, `GET vmax 2` ou `FLT`.

## Journal

Messages du simulateur, du programme et du robot réel : erreurs, avertissements (collision, butée, perte de pas),
événements du firmware (`EVT …`).

## Modes de commande (barre du haut)

| Mode | Principe | Usage |
|---|---|---|
| Pas-à-pas boucle ouverte | consigne de position aux moteurs, sans retour | ORION-6 MAKER |
| PID articulaire | Kp·e + Ki·∫e + Kd·ė + anticipations (gravité, inertie, frottements) | servos à codeur |
| Mode MIT | Kp·(p*−p) + Kd·(v*−v) + τff, exécuté dans le driver | ORION-6 PRO, bras de recherche |
| Couple calculé | linéarisation par le modèle dynamique | commande haute performance |
| Impédance cartésienne | le TCP se comporte comme un ressort amorti | contact, assemblage |
| Compensation de gravité | le bras « flotte » | apprentissage à la main |

Voir [6. Commande et réglage](06-commande-et-reglage.md) pour les équations et la méthode de réglage.
