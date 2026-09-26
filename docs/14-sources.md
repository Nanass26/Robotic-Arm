# 14. Sources et inspirations

## La démonstration robotique de GPT-6 Astra

La demande à l’origine d’ORION-6 faisait référence au logiciel présenté autour des **robots de GPT-6 Astra**.

- [GPT-6 Astra: A new generation of intelligence](https://openai.com/index/gpt-6-astra/) (OpenAI).
- [GPT-6 Astra: The next generation in intelligence for work](https://openai.com/index/gpt-6-astra-next-generation-work/)
  (OpenAI).

Selon les sources consultées, GPT-6 Astra pilotait une paire de **bras YAM**. Il y apportait le jugement et la
correction au niveau « sémantique », tandis qu’une politique motrice gérait l’interaction physique. Il a par exemple
réussi 19 fois sur 20 à déposer un bloc dans un bol.

- [awesome-gpt-6-astra](https://github.com/magiccreator-ai/awesome-gpt-6-astra) : liste des démonstrations publiées
  (jeux, Blender et 3D, mécanismes).

Idées reprises dans ORION-6 :

- un logiciel unique qui sert à la fois de **jumeau numérique**, d’outil de réglage et de programmation, et
  d’**interface en langage naturel** ;
- des bras à **moteurs brushless sur bus CAN, commandés en mode MIT avec compensation de gravité** : variante PRO ;
- un **protocole ouvert et scriptable** : ORION-ASCII, utilisable depuis n’importe quel programme, IA comprise (voir
  l’exemple Python de [11. Firmware](11-firmware-protocole.md)).

## Bras YAM (i2rt)

- [i2rt-robotics/i2rt](https://github.com/i2rt-robotics/i2rt) : bibliothèque Python des bras YAM. Elle documente :
  - des moteurs Damiao (DM3507, DM4310, DM4340) sur CAN à 1 Mbit/s ;
  - le mode MIT et la compensation de gravité (*zero-gravity mode*) ;
  - l’intégration MuJoCo et les modèles URDF/MJCF ;
  - un **délai de sécurité de 400 ms** au-delà duquel les moteurs passent en amortissement.
- Les bras YAM ont 6 axes. Leurs versions vont de 2 à 4 kg de charge, pour environ 780 mm de portée.

Ce qu’ORION-6 en reprend :

- la loi PD + compensation de gravité ;
- l’export MJCF (MuJoCo) ;
- le chien de garde du pont MIT. Celui-ci **maintient** la dernière consigne avec son couple de gravité plutôt que
  d’amortir, pour ne pas laisser retomber le bras. L’amortissement est réservé aux défauts moteur.

Actionneurs de référence de la variante PRO :

- [Damiao DM-J4340-2EC](https://aifitlab.com/products/damiao-dm-j4340-2ec-servo-motor) (fiche revendeur) ;
- Damiao DM-J4310 pour le poignet.

## Defend Intelligence (YouTube)

- [Chaîne Defend Intelligence](https://www.youtube.com/@DefendIntelligence) : vulgarisation de l’IA en français.
- [« Création bras Robot qui apprend »](https://www.youtube.com/shorts/8peRzwpX3AY) (#robot #ai #lerobot #blender) :
  construction d’un bras qui apprend par démonstration, avec l’écosystème LeRobot.

Je n’ai pas retrouvé de description publique détaillée d’un logiciel qu’il aurait créé pour la présentation de
GPT-6 Astra. ORION Studio reprend donc l’esprit général de ces démonstrations : bras imprimé, simulation 3D, pilotage
par l’IA et apprentissage. Il n’est pas une copie d’un logiciel existant.

## Bras open source imprimés (YouTube et GitHub)

- **SO-100 / SO-101** et **LeRobot** (Hugging Face) :
  [github.com/TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100),
  [github.com/huggingface/lerobot](https://github.com/huggingface/lerobot). Bras bon marché à servos, référence pour
  l’apprentissage par imitation.
- **AR4** (Annin Robotics), [anninrobotics.com](https://www.anninrobotics.com) : bras 6 axes à pas-à-pas et Teensy.
  Il a inspiré le choix Teensy + drivers pas-à-pas + capteurs d’origine d’ORION-6 MAKER.
- **BCN3D Moveo**, [github.com/BCN3D/BCN3D-Moveo](https://github.com/BCN3D/BCN3D-Moveo) : bras imprimé historique
  (pas-à-pas, courroies).
- **Thor**, [github.com/AngelLM/Thor](https://github.com/AngelLM/Thor) : bras 6 axes imprimé open source.
- **Dummy Robot** (Peng Zhihui), [github.com/peng-zhihui/Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot) :
  petit bras 6 axes à servos pas-à-pas en boucle fermée, très compact.

## Références techniques

- **Mode MIT** :
  - B. Katz, *A Low Cost Modular Actuator for Dynamic Robots*, mémoire de master, MIT, 2018 ;
  - B. Katz, J. Di Carlo, S. Kim, *Mini Cheetah: A Platform for Pushing the Limits of Dynamic Quadruped Control*,
    ICRA 2019.

  Ces travaux sont à l’origine du format de trame p/v/Kp/Kd/τ repris par les moteurs QDD.
- **Robotique générale** : B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, *Robotics: Modelling, Planning and
  Control*, Springer, 2009 (DH, modèle inverse, jacobien, commande).
- **Dynamique** : R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008 (RNEA, CRBA).
- **Articulation flexible** : M. W. Spong, *Modeling and Control of Elastic Joint Robots*, ASME J. Dyn. Sys. Meas.
  Control, 1987.
- **Trajectoires** : L. Biagiotti, C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots*, Springer,
  2008 (courbe en S à 7 phases).
- **Frottement** : H. Olsson, K. J. Åström et al., *Friction Models and Friction Compensation*, European Journal of
  Control, 1998 (effet Stribeck).
- **Manipulabilité** : T. Yoshikawa, *Manipulability of Robotic Mechanisms*, IJRR, 1985.

## Logiciels et bibliothèques utilisés

- [three.js](https://threejs.org) (licence MIT) : rendu 3D d’ORION Studio.
- [manifold-3d](https://github.com/elalish/manifold) (licence Apache-2.0) : géométrie de construction solide robuste,
  pour la CAO paramétrique et l’export STL.
- [esbuild](https://esbuild.github.io) : regroupement du fichier autonome.
- Cœur Teensy ([PaulStoffregen/cores](https://github.com/PaulStoffregen/cores)), EEPROM et
  [FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4) : compilation des firmwares.
