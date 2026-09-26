# 13. Dépannage

## ORION Studio

| Symptôme | Cause probable | Remède |
|---|---|---|
| Page blanche avec `npm start` | modules non installés | `npm install`, puis `npm start`. Ouvrez bien http://localhost:8080/studio/ |
| Page blanche en ouvrant `studio/index.html` directement | les modules ES ne se chargent pas depuis `file://` | utilisez `npm start`, ou le fichier autonome `dist/ORION-Studio.html` |
| Le modèle 3D reste simplifié (sans pièces CAO) | CAO en cours de génération, ou géométrie DH incompatible avec l’architecture ORION-6 | patientez quelques secondes, ou rechargez le préréglage ORION-6 MAKER |
| *Connecter (USB)* grisé | navigateur sans Web Serial (Firefox, Safari, mobile) | Chrome ou Edge sur ordinateur |
| Le port n’apparaît pas | câble USB de charge seule, pilote, autre logiciel qui occupe le port | câble de données, fermer le moniteur série de l’IDE Arduino |
| « Jumeau numérique » : `ERR 5 flux …` | la simulation est loin de la position réelle | *Synchroniser*, puis relancer le jumeau numérique |
| Mouvements saccadés en jumeau numérique | onglet en arrière-plan (navigateur ralenti), PC chargé | garder l’onglet visible ; augmenter `stream_delay` (60–80 ms) |
| Le robot ne suit plus et `EVT WATCHDOG` apparaît | consignes interrompues (onglet masqué, câble) | normal : arrêt de sécurité. Relancer le jumeau numérique |
| Paramètres « bizarres » après import | valeurs hors plage bornées automatiquement | lire le *Journal* : chaque correction y est listée |
| Revenir aux réglages d’origine | configuration sauvegardée dans le navigateur | *Paramètres* → ⟲ (deux clics) ou choisir un préréglage |

## Simulation et réglage

| Symptôme | Cause probable | Remède |
|---|---|---|
| « Perte de pas » en simulation | couple insuffisant (accélération, charge, vitesse au-delà de la vitesse de coin) | *Analyse → Dimensionnement* ; réduire accélération, jerk ou vitesse ; augmenter la tension ou le courant |
| Oscillations autour de la consigne (PID) | jeu + intégrale, bande passante trop haute | réduire Ki, zone morte ≈ jeu/2, bande passante < f_résonance/3 |
| Le bras « tombe » en compensation de gravité | masses ou centres de masse sous-estimés | corriger *Dynamique*, puis tester de nouveau |
| Arrêt « auto-collision » inattendu | pose réellement en collision, ou capsules trop grosses | vue *Modèle de collision* ; ajuster la *Marge de collision* (*Sécurité*) |
| MoveL très lent par endroits | passage près d’une singularité (re-temporisation) | changer de configuration (solutions IK) ou passer par un MoveJ |
| « cible hors d’atteinte » ou « toutes les solutions sont hors butées » | pose hors de l’espace de travail, orientation impossible ou butées | *Analyse → Espace de travail* ; changer l’orientation ou la configuration demandée |

## Firmware et électronique (ORION-6 MAKER)

| Symptôme | Cause probable | Remède |
|---|---|---|
| Aucune réponse à `PING` | mauvais port, firmware non flashé, câble | LED 13 : un éclair toutes les 2 s = firmware en marche, moteurs coupés ; reflasher le .hex |
| `ERR 7 arret d'urgence actif` | entrée 30 ouverte (champignon enfoncé ou fil coupé) | réarmer ; vérifier le contact NF entre la broche 30 et GND |
| `ERR 4 prise d'origine requise` | robot non référencé (démarrage, `EN 0`, arrêt d’urgence) | `HOME` (ou `ZERO` si le robot est posé en pose Home) |
| `EVT FAULT prise d'origine Jn : capteur introuvable` | capteur débranché, aimant trop loin, mauvais sens de recherche | tester le capteur (drapeau 0x40 de `STAT`) ; `SET home_dir n …` |
| `EVT FAULT fin de course Jn` pendant un mouvement | butée logicielle au-delà du capteur, ou capteur parasité | vérifier `min`/`max` et la position du capteur ; blinder et éloigner le câble du capteur |
| Moteurs qui sifflent sans tourner | câblage de bobine croisé (A+/B+ inversés) | identifier les paires à l’ohmmètre (≈ quelques Ω) |
| Pas manqués aléatoires, même à vide | niveaux 3,3 V insuffisants pour des drivers optocouplés | tampon 74HCT245 en 5 V (voir [9. Électronique](09-electronique.md)) |
| Axe qui tourne à l’envers | câblage moteur | `SET invert n 1` puis `SAVE` |
| Angle réel ≠ angle commandé (échelle) | micro-pas du driver ≠ 16, réduction | vérifier les interrupteurs du driver ; `steps_per_deg` ([12. Calibration](12-calibration.md)) |
| Décalage constant après `HOME` | position du capteur non calibrée | calibrer `home_pos` |
| Drivers chauds, moteurs brûlants | courant trop élevé, pas de réduction au repos | courant ≤ 90 % du nominal ; activer « half current » |
| La Teensy redémarre quand les moteurs démarrent | chute de tension ou masse commune mal faite | alimentation logique séparée (buck 5 V), masse en étoile, condensateur 470 µF sur le 5 V |

## Pont CAN mode MIT (ORION-6 PRO)

| Symptôme | Cause probable | Remède |
|---|---|---|
| `ERR 6 moteur Jn muet` | identifiant faux, moteur non alimenté, bus sans terminaison, CAN_H/CAN_L inversés | mesurer 60 Ω entre CAN_H et CAN_L (hors tension) ; vérifier l’ID avec l’outil du fabricant |
| Positions qui « sautent » ou valeurs aberrantes | plages P/V/T du pont ≠ réglages du moteur | aligner `p_max`/`v_max`/`t_max` sur la configuration du moteur |
| `EVT FAULT CAN : Jn ne repond plus` en fonctionnement | bus saturé (fréquence trop haute), connectique | `SET ctrl_hz * 300`, contrôler les connecteurs, séparer sur deux bus |
| `EVT FAULT Jn : temperature …` | Kp élevé face à une erreur statique permanente (masse fausse), réducteur qui force | corriger la compensation de gravité ; réduire Kp ; laisser refroidir |
| Le bras tombe à l’activation | compensation de gravité absente avant le premier flux | normal sans τff : activer puis lancer rapidement le jumeau numérique ; ou soutenir le bras |
| Vibrations en maintien | Kd trop faible ou trop fort (bruit de vitesse), Kp trop haut | ζ ≈ 0,7–1 ; réduire Kp ; vérifier la rigidité mécanique |
| Pose fausse après remise sous tension | position absolue perdue (réducteur intégré) | poser en pose de repos, `ZERO`, `SAVE` ; toujours vérifier `STAT` avant `EN 1` |

## Mécanique

| Symptôme | Cause probable | Remède |
|---|---|---|
| Réducteur dur ou à points durs | pied d’éléphant sur un disque, galet mal enfoncé, disques A/B inversés | ébavurer, contrôler les galets, vérifier le repère « B » |
| Jeu excessif en sortie | compensation XY trop négative, trous de doigts trop grands | réimprimer les disques après le test de tolérances |
| Craquements sous charge | inserts arrachés, vis trop serrées | reposer les inserts (fer), serrer à la main |
| Le tube glisse dans son raccord | collier desserré | resserrer les vis M4 ; un tour de ruban aluminium en cale |
| Flexion visible du bras | pièces imprimées trop peu remplies | respecter les remplissages indiqués (≥ 45 % pour la structure) |
