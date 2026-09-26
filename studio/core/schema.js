// ORION-6 — Schéma de TOUS les paramètres réglables.
// Source unique utilisée par : l'interface (formulaires générés), la validation,
// la documentation (docs/03-parametres.md générée) et l'export firmware.
//
// Types de descripteurs :
//   number  : valeur scalaire (SI) — unit, min, max, step (en SI)
//   int     : entier
//   enum    : liste d'options [valeur, libellé]
//   bool    : case à cocher
//   text    : texte libre
//   vec3    : vecteur [x, y, z] (SI)
//   jointvec: vecteur de N valeurs articulaires
//   table   : tableau d'objets par articulation (colonnes = champs)
//
// Chaque paramètre possède une aide (help) en français. Les tables sont
// affichées « transposées » (une ligne par paramètre, une colonne par axe).

const DEG = Math.PI / 180;

const num = (path, label, unit, opt = {}) => ({ kind: 'number', path, label, unit, ...opt });
const int = (path, label, opt = {}) => ({ kind: 'int', path, label, unit: 'count', ...opt });
const sel = (path, label, options, opt = {}) => ({ kind: 'enum', path, label, options, ...opt });
const bool = (path, label, opt = {}) => ({ kind: 'bool', path, label, ...opt });
const text = (path, label, opt = {}) => ({ kind: 'text', path, label, ...opt });
const vec3 = (path, label, unit, opt = {}) => ({ kind: 'vec3', path, label, unit, ...opt });
const jointvec = (path, label, unit, opt = {}) => ({ kind: 'jointvec', path, label, unit, ...opt });
const table = (path, label, columns, opt = {}) => ({ kind: 'table', path, label, columns, ...opt });

// Colonnes (champs par articulation)
const col = (key, label, unit, opt = {}) => ({ kind: 'number', key, label, unit, ...opt });
const colInt = (key, label, opt = {}) => ({ kind: 'int', key, label, unit: 'count', ...opt });
const colSel = (key, label, options, opt = {}) => ({ kind: 'enum', key, label, options, ...opt });
const colBool = (key, label, opt = {}) => ({ kind: 'bool', key, label, ...opt });
const colText = (key, label, opt = {}) => ({ kind: 'text', key, label, ...opt });

export const ACTUATOR_TYPES = [
  ['stepper', 'Pas-à-pas + réducteur (boucle ouverte)'],
  ['bldc_mit', 'Brushless QDD « mode MIT » (CAN)'],
  ['servo', 'Servo asservi (PID couple/position)'],
];

export const CONTROL_MODES = [
  ['stepper', 'Pas-à-pas boucle ouverte (consigne de position)'],
  ['pid', 'PID articulaire + anticipations'],
  ['mit', 'Mode MIT : Kp·(p*−p) + Kd·(v*−v) + τff'],
  ['computed_torque', 'Couple calculé (linéarisation dynamique)'],
  ['impedance', 'Impédance cartésienne (compliance)'],
  ['gravity_comp', 'Compensation de gravité (main libre)'],
];

export const PROFILES = [
  ['scurve', 'Courbe en S (jerk limité, 7 phases)'],
  ['trapezoid', 'Trapézoïdal (accélération limitée)'],
  ['quintic', 'Polynôme quintique (jerk minimal)'],
  ['cycloidal', 'Cycloïdal (sinusoïdal)'],
];

export const SCHEMA = [
  {
    id: 'general',
    title: 'Général',
    icon: 'robot',
    help: 'Identité du robot et variante matérielle.',
    params: [
      text('meta.name', 'Nom du robot', { help: 'Nom affiché dans l’interface et utilisé pour les exports (URDF, MJCF, firmware).' }),
      sel('meta.variant', 'Variante matérielle', [
        ['maker', 'MAKER — NEMA17/23 + réducteurs cycloïdaux imprimés'],
        ['pro', 'PRO — actionneurs brushless QDD en mode MIT (CAN)'],
        ['custom', 'Personnalisée'],
      ], { help: 'Change uniquement l’étiquette et les valeurs proposées par les préréglages. Tous les paramètres restent modifiables individuellement.' }),
      text('meta.description', 'Description', { help: 'Notes libres enregistrées avec la configuration.' }),
    ],
  },
  {
    id: 'kinematics',
    title: 'Cinématique (Denavit-Hartenberg)',
    icon: 'axes',
    help: 'Géométrie du bras : table DH, repère de base et outil (TCP). Ces paramètres définissent le modèle géométrique direct et inverse.',
    params: [
      sel('kinematics.convention', 'Convention DH', [
        ['DH', 'DH standard (Denavit-Hartenberg classique)'],
        ['MDH', 'DH modifiée (Craig)'],
      ], { help: 'Standard : Aᵢ = Rz(θᵢ)·Tz(dᵢ)·Tx(aᵢ)·Rx(αᵢ), l’axe de l’articulation i est zᵢ₋₁. Modifiée (Craig) : Aᵢ = Rx(αᵢ₋₁)·Tx(aᵢ₋₁)·Rz(θᵢ)·Tz(dᵢ), l’axe i est zᵢ. Le modèle inverse analytique n’est disponible qu’en DH standard ; en DH modifiée le solveur numérique est utilisé.' }),
      table('kinematics.joints', 'Table DH', [
        colText('name', 'Nom', { help: 'Nom de l’axe (affiché dans l’interface).' }),
        colSel('type', 'Type', [['revolute', 'Rotoïde'], ['prismatic', 'Prismatique']], { help: 'Rotoïde : θ varie. Prismatique : d varie (glissière).' }),
        col('a', 'a — longueur', 'length', { min: -2, max: 2, step: 0.0005, help: 'Distance entre les axes zᵢ₋₁ et zᵢ mesurée le long de xᵢ (DH standard).' }),
        col('alpha', 'α — torsion', 'angle', { min: -Math.PI, max: Math.PI, step: DEG, help: 'Angle entre zᵢ₋₁ et zᵢ autour de xᵢ.' }),
        col('d', 'd — décalage', 'length', { min: -2, max: 2, step: 0.0005, help: 'Distance entre xᵢ₋₁ et xᵢ le long de zᵢ₋₁.' }),
        col('thetaOffset', 'θ₀ — décalage zéro', 'angle', { min: -Math.PI, max: Math.PI, step: DEG, help: 'Décalage entre la valeur articulaire q et l’angle DH : θ = sens·q + θ₀. Permet de choisir la pose « zéro » mécanique.' }),
        col('direction', 'Sens (+1/−1)', 'none', { min: -1, max: 1, step: 2, help: 'Inverse le sens positif de l’axe sans changer la géométrie (θ = sens·q + θ₀).' }),
      ], { help: 'Une ligne par articulation (J1 = base … J6 = poignet). Unités d’affichage : mm et degrés.' }),
      num('kinematics.base.x', 'Base — X', 'length', { min: -5, max: 5, step: 0.001, help: 'Position du repère de base du robot dans le monde (ex. table).' }),
      num('kinematics.base.y', 'Base — Y', 'length', { min: -5, max: 5, step: 0.001 }),
      num('kinematics.base.z', 'Base — Z', 'length', { min: -5, max: 5, step: 0.001, help: 'Hauteur de la base au-dessus du plan de travail.' }),
      num('kinematics.base.roll', 'Base — roulis (Rx)', 'angle', { min: -Math.PI, max: Math.PI, step: DEG, help: 'Orientation de la base (montage mural/plafond : ±90° / 180°).' }),
      num('kinematics.base.pitch', 'Base — tangage (Ry)', 'angle', { min: -Math.PI, max: Math.PI, step: DEG }),
      num('kinematics.base.yaw', 'Base — lacet (Rz)', 'angle', { min: -Math.PI, max: Math.PI, step: DEG }),
      text('kinematics.tool.name', 'Outil — nom', { help: 'Nom de l’outil monté sur la bride (pince, ventouse, stylo…).' }),
      num('kinematics.tool.x', 'TCP — X', 'length', { min: -1, max: 1, step: 0.0005, help: 'Position du point outil (TCP) dans le repère de la bride (repère 6).' }),
      num('kinematics.tool.y', 'TCP — Y', 'length', { min: -1, max: 1, step: 0.0005 }),
      num('kinematics.tool.z', 'TCP — Z', 'length', { min: -1, max: 1, step: 0.0005, help: 'Longueur de l’outil le long de l’axe de la bride.' }),
      num('kinematics.tool.roll', 'TCP — Rx', 'angle', { min: -Math.PI, max: Math.PI, step: DEG }),
      num('kinematics.tool.pitch', 'TCP — Ry', 'angle', { min: -Math.PI, max: Math.PI, step: DEG }),
      num('kinematics.tool.yaw', 'TCP — Rz', 'angle', { min: -Math.PI, max: Math.PI, step: DEG }),
    ],
  },
  {
    id: 'limits',
    title: 'Limites & performances',
    icon: 'gauge',
    help: 'Butées logicielles, vitesses, accélérations et jerks maximaux (articulaires et cartésiens).',
    params: [
      table('limits.joints', 'Limites articulaires', [
        col('min', 'Butée min', 'angle', { min: -2 * Math.PI, max: 2 * Math.PI, step: DEG, help: 'Butée logicielle basse. Doit rester à l’intérieur de la butée mécanique.' }),
        col('max', 'Butée max', 'angle', { min: -2 * Math.PI, max: 2 * Math.PI, step: DEG, help: 'Butée logicielle haute.' }),
        col('vmax', 'Vitesse max', 'angvel', { min: 0.01, max: 50, step: DEG, help: 'Vitesse articulaire maximale utilisée par le planificateur (100 % de vitesse).' }),
        col('amax', 'Accélération max', 'angacc', { min: 0.01, max: 500, step: DEG, help: 'Accélération articulaire maximale.' }),
        col('jmax', 'Jerk max', 'angjerk', { min: 0.1, max: 50000, step: 10 * DEG, help: 'Dérivée de l’accélération (profil en S). Plus faible = mouvements plus doux, moins de vibrations.' }),
      ]),
      num('limits.cartesian.vmax', 'Vitesse TCP max', 'linvel', { min: 0.001, max: 5, step: 0.001, help: 'Vitesse linéaire maximale du point outil pour les mouvements linéaires/circulaires (MoveL/MoveC).' }),
      num('limits.cartesian.amax', 'Accélération TCP max', 'linacc', { min: 0.01, max: 50, step: 0.01 }),
      num('limits.cartesian.jmax', 'Jerk TCP max', 'linjerk', { min: 0.1, max: 1000, step: 0.1 }),
      num('limits.cartesian.wmax', 'Vitesse angulaire outil max', 'angvel', { min: 0.01, max: 20, step: DEG }),
      num('limits.cartesian.alphamax', 'Accélération angulaire outil max', 'angacc', { min: 0.01, max: 200, step: DEG }),
      bool('limits.workspace.enabled', 'Boîte de sécurité active', { help: 'Refuse tout mouvement dont le TCP sortirait de la boîte définie ci-dessous (repère monde).' }),
      num('limits.workspace.xmin', 'Boîte — X min', 'length', { min: -5, max: 5, step: 0.001 }),
      num('limits.workspace.xmax', 'Boîte — X max', 'length', { min: -5, max: 5, step: 0.001 }),
      num('limits.workspace.ymin', 'Boîte — Y min', 'length', { min: -5, max: 5, step: 0.001 }),
      num('limits.workspace.ymax', 'Boîte — Y max', 'length', { min: -5, max: 5, step: 0.001 }),
      num('limits.workspace.zmin', 'Boîte — Z min', 'length', { min: -5, max: 5, step: 0.001, help: 'Mettre légèrement au-dessus de la table pour protéger l’outil.' }),
      num('limits.workspace.zmax', 'Boîte — Z max', 'length', { min: -5, max: 5, step: 0.001 }),
      num('limits.floorZ', 'Hauteur du plan de travail', 'length', { min: -2, max: 2, step: 0.001, help: 'Altitude de la table dans le repère monde, utilisée pour la détection de collision avec le sol.' }),
    ],
  },
  {
    id: 'dynamics',
    title: 'Dynamique (masses & inerties)',
    icon: 'weight',
    help: 'Paramètres inertiels des segments, de l’outil et de la charge, frottements. Utilisés par la simulation, la compensation de gravité, le couple calculé et le dimensionnement. Valeurs par défaut calculées depuis la CAO (PLA, remplissage indiqué) + moteurs + visserie.',
    params: [
      vec3('dynamics.gravity', 'Gravité', 'accel', { help: 'Vecteur gravité dans le repère monde (par défaut [0, 0, −9,81]).' }),
      table('dynamics.links', 'Segments', [
        col('mass', 'Masse', 'mass', { min: 0, max: 50, step: 0.001, help: 'Masse du segment i (pièces imprimées + moteur/stator porté + visserie).' }),
        col('com.0', 'CdG x', 'length', { min: -1, max: 1, step: 0.0005, help: 'Centre de gravité exprimé dans le repère DH i du segment.' }),
        col('com.1', 'CdG y', 'length', { min: -1, max: 1, step: 0.0005 }),
        col('com.2', 'CdG z', 'length', { min: -1, max: 1, step: 0.0005 }),
        col('inertia.0', 'Ixx', 'inertia', { min: 0, max: 10, step: 1e-6, help: 'Tenseur d’inertie au centre de gravité, axes du repère i.' }),
        col('inertia.1', 'Iyy', 'inertia', { min: 0, max: 10, step: 1e-6 }),
        col('inertia.2', 'Izz', 'inertia', { min: 0, max: 10, step: 1e-6 }),
        col('inertia.3', 'Ixy', 'inertia', { min: -10, max: 10, step: 1e-6 }),
        col('inertia.4', 'Ixz', 'inertia', { min: -10, max: 10, step: 1e-6 }),
        col('inertia.5', 'Iyz', 'inertia', { min: -10, max: 10, step: 1e-6 }),
      ]),
      num('dynamics.tool.mass', 'Outil — masse', 'mass', { min: 0, max: 20, step: 0.001, help: 'Masse de l’outil (pince + servo + bride).' }),
      vec3('dynamics.tool.com', 'Outil — CdG (repère bride)', 'length', {}),
      vec3('dynamics.tool.inertiaDiag', 'Outil — inertie (Ixx, Iyy, Izz)', 'inertia', {}),
      num('dynamics.payload.mass', 'Charge — masse', 'mass', { min: 0, max: 20, step: 0.001, help: 'Masse de l’objet manipulé. Utilisée quand la pince est fermée sur un objet (et pour le dimensionnement).' }),
      vec3('dynamics.payload.com', 'Charge — CdG (repère TCP)', 'length', {}),
      num('dynamics.payloadRated', 'Charge nominale (dimensionnement)', 'mass', { min: 0, max: 20, step: 0.01, help: 'Charge utile maximale annoncée, utilisée par l’analyse de dimensionnement.' }),
      table('dynamics.friction', 'Frottements (côté articulation)', [
        col('coulomb', 'Coulomb', 'torque', { min: 0, max: 50, step: 0.001, help: 'Couple de frottement sec (réducteur + roulements), ramené à la sortie.' }),
        col('viscous', 'Visqueux', 'damping', { min: 0, max: 50, step: 0.001, help: 'Coefficient de frottement visqueux (couple ∝ vitesse).' }),
        col('stiction', 'Adhérence', 'torque', { min: 0, max: 50, step: 0.001, help: 'Couple de décollement (≥ Coulomb). Modèle de Stribeck.' }),
        col('stribeckVel', 'Vitesse de Stribeck', 'angvel', { min: 1e-4, max: 1, step: 0.001, help: 'Vitesse caractéristique de la transition adhérence → glissement.' }),
      ]),
    ],
  },
  {
    id: 'actuators',
    title: 'Actionneurs (moteurs & réducteurs)',
    icon: 'motor',
    help: 'Modèle de chaque actionneur : type, réduction, rendement, caractéristiques électriques et mécaniques. Les couples/vitesses « de sortie » s’entendent côté articulation.',
    params: [
      num('actuators.supplyVoltage', 'Tension d’alimentation', 'voltage', { min: 5, max: 80, step: 0.5, help: 'Tension du bus moteurs. Détermine la chute de couple des pas-à-pas à haute vitesse.' }),
      table('actuators.joints', 'Actionneurs', [
        colSel('type', 'Type', ACTUATOR_TYPES, { help: 'Modèle physique utilisé par la simulation.' }),
        colText('model', 'Référence', { help: 'Référence du moteur / module (documentation, nomenclature).' }),
        col('gearRatio', 'Réduction', 'ratio', { min: 1, max: 500, step: 0.5, help: 'Rapport de réduction N (moteur/sortie). Réducteur cycloïdal : N = nombre de lobes = nb de galets − 1.' }),
        col('efficiency', 'Rendement', 'percent', { min: 0.1, max: 1, step: 0.01, help: 'Rendement du réducteur (couple sortie = N·η·couple moteur).' }),
        col('rotorInertia', 'Inertie rotor', 'rotorInertia', { min: 0, max: 1e-2, step: 1e-7, help: 'Inertie du rotor moteur. Ramenée à la sortie : N²·J_rotor (souvent dominante !).' }),
        col('backlash', 'Jeu (backlash)', 'angle', { min: 0, max: 0.1, step: 0.0001, help: 'Jeu angulaire total en sortie de réducteur.' }),
        col('stiffness', 'Raideur transmission', 'stiffness', { min: 10, max: 1e6, step: 10, help: 'Raideur torsionnelle du réducteur (sortie). Utilisée avec le jeu.' }),
        col('holdingTorque', 'Couple de maintien', 'torque', { min: 0, max: 20, step: 0.01, group: 'stepper', help: '[Pas-à-pas] Couple de maintien du moteur (fiche technique) au courant nominal.' }),
        col('stepAngle', 'Angle de pas', 'angle', { min: 0.001, max: 0.2, step: 0.0001, group: 'stepper', help: '[Pas-à-pas] 1,8° (200 pas/tour) ou 0,9° (400 pas/tour).' }),
        colInt('microsteps', 'Micro-pas', { min: 1, max: 256, group: 'stepper', help: '[Pas-à-pas] Subdivision réglée sur le driver (DIP/UART).' }),
        col('ratedCurrent', 'Courant nominal', 'current', { min: 0, max: 20, step: 0.05, group: 'stepper', help: '[Pas-à-pas] Courant par phase de la fiche technique.' }),
        col('runCurrent', 'Courant de travail', 'current', { min: 0, max: 20, step: 0.05, group: 'stepper', help: '[Pas-à-pas] Courant réglé sur le driver. Couple ≈ proportionnel.' }),
        col('phaseResistance', 'Résistance de phase', 'resistance', { min: 0.01, max: 100, step: 0.01, group: 'stepper' }),
        col('phaseInductance', 'Inductance de phase', 'inductance', { min: 1e-5, max: 0.1, step: 1e-5, group: 'stepper', help: '[Pas-à-pas] Limite la montée du courant à haute vitesse → chute de couple.' }),
        col('peakTorque', 'Couple crête (sortie)', 'torque', { min: 0, max: 500, step: 0.1, group: 'servo', help: '[Brushless/servo] Couple maximal en sortie de réducteur.' }),
        col('ratedTorque', 'Couple nominal (sortie)', 'torque', { min: 0, max: 500, step: 0.1, group: 'servo', help: '[Brushless/servo] Couple continu admissible (échauffement).' }),
        col('maxSpeed', 'Vitesse à vide (sortie)', 'speedRpm', { min: 0.1, max: 1000, step: 0.1, group: 'servo', help: '[Brushless/servo] Vitesse à vide en sortie. Le couple disponible décroît linéairement jusqu’à cette vitesse.' }),
        col('torqueConstant', 'Constante de couple (sortie)', 'torqueConst', { min: 0, max: 100, step: 0.001, group: 'servo', help: '[Brushless/servo] Kt ramené à la sortie (N·m/A).' }),
        colInt('encoderCpr', 'Résolution codeur', { min: 0, max: 1 << 24, group: 'sensor', help: 'Points par tour (côté indiqué ci-dessous). 0 = pas de codeur.' }),
        colBool('encoderOnOutput', 'Codeur en sortie', { group: 'sensor', help: 'Coché : codeur absolu sur l’arbre de sortie (ex. AS5600/MT6701). Sinon : codeur moteur.' }),
        colInt('canId', 'ID CAN', { min: 0, max: 0x7ff, group: 'mit', help: '[Mode MIT] Identifiant CAN du moteur (esclave).' }),
        col('pMax', 'P_MAX', 'angle', { min: 0.1, max: 100, step: 0.1, group: 'mit', help: '[Mode MIT] Plage de position du protocole (±P_MAX rad) — doit correspondre à la configuration du moteur.' }),
        col('vMax', 'V_MAX', 'angvel', { min: 0.1, max: 500, step: 0.1, group: 'mit', help: '[Mode MIT] Plage de vitesse du protocole (±V_MAX rad/s).' }),
        col('tMax', 'T_MAX', 'torque', { min: 0.1, max: 500, step: 0.1, group: 'mit', help: '[Mode MIT] Plage de couple du protocole (±T_MAX N·m).' }),
        col('kpMax', 'KP_MAX', 'stiffness', { min: 1, max: 5000, step: 1, group: 'mit', help: '[Mode MIT] Borne haute du gain Kp codé sur 12 bits (souvent 500).' }),
        col('kdMax', 'KD_MAX', 'damping', { min: 0.1, max: 100, step: 0.1, group: 'mit', help: '[Mode MIT] Borne haute du gain Kd codé sur 12 bits (souvent 5).' }),
      ]),
    ],
  },
  {
    id: 'control',
    title: 'Commande (PID, mode MIT, couple calculé)',
    icon: 'sliders',
    help: 'Stratégie de commande et gains. Le mode « pas-à-pas » reproduit le comportement réel des moteurs pas-à-pas en boucle ouverte (y compris la perte de pas). Les autres modes supposent des actionneurs pilotés en couple.',
    params: [
      sel('control.mode', 'Mode de commande', CONTROL_MODES, { help: 'PID : τ = Kp·e + Ki·∫e + Kd·ė + anticipations. MIT : τ = Kp·(p*−p) + Kd·(v*−v) + τff (exécuté dans le driver du moteur). Couple calculé : τ = M(q)·(q̈* + Kp·e + Kd·ė) + C(q,q̇)·q̇ + g(q). Impédance : τ = Jᵀ·(Kx·Δx + Dx·Δẋ) + g(q). Gravité : τ = g(q) — le bras flotte et se déplace à la main.' }),
      num('control.loopHz', 'Fréquence de boucle', 'frequency', { min: 50, max: 20000, step: 50, help: 'Fréquence d’échantillonnage du correcteur (bloqueur d’ordre 0 entre deux échantillons). Une fréquence trop basse rend les gains élevés instables — comme sur le vrai robot.' }),
      bool('control.feedforward.gravity', 'Anticipation gravité', { help: 'Ajoute g(q) calculé par le modèle dynamique (indispensable pour des gains faibles / compliance).' }),
      bool('control.feedforward.friction', 'Anticipation frottements', { help: 'Ajoute une estimation des frottements (Coulomb + visqueux) dans le sens de la vitesse désirée.' }),
      bool('control.feedforward.inertia', 'Anticipation inertie', { help: 'Ajoute M(q)·q̈* : améliore fortement le suivi de trajectoire.' }),
      num('control.modelError', 'Erreur de modèle (test)', 'percent', { min: 0, max: 0.5, step: 0.01, help: 'Perturbe volontairement les masses du modèle utilisé par le correcteur pour tester la robustesse (0 % = modèle parfait).' }),
      table('control.joints', 'Gains articulaires', [
        col('kp', 'PID — Kp', 'stiffness', { min: 0, max: 1e5, step: 1, help: 'Gain proportionnel (couple par radian d’erreur).' }),
        col('ki', 'PID — Ki', 'integralGain', { min: 0, max: 1e6, step: 1, help: 'Gain intégral : annule l’erreur statique. Trop élevé → dépassement/oscillations.' }),
        col('kd', 'PID — Kd', 'damping', { min: 0, max: 1e4, step: 0.01, help: 'Gain dérivé (amortissement). Calculé sur la mesure (pas de coup de dérivée sur échelon).' }),
        col('iLimit', 'PID — limite intégrale', 'torque', { min: 0, max: 500, step: 0.1, help: 'Saturation du terme intégral (anti-emballement / anti-windup).' }),
        col('dFilterHz', 'PID — filtre dérivée', 'frequency', { min: 1, max: 5000, step: 1, help: 'Fréquence de coupure du filtre passe-bas du terme dérivé (réduit le bruit de mesure).' }),
        col('deadband', 'Zone morte', 'angle', { min: 0, max: 0.05, step: 0.0001, help: 'Erreur en dessous de laquelle le correcteur n’agit pas (évite le « pompage » dû au jeu).' }),
        col('mitKp', 'MIT — Kp', 'stiffness', { min: 0, max: 5000, step: 0.1, help: 'Raideur du mode MIT (dans le driver moteur). Typique 20–150 N·m/rad pour un bras de bureau.' }),
        col('mitKd', 'MIT — Kd', 'damping', { min: 0, max: 100, step: 0.01, help: 'Amortissement du mode MIT. Typique 0,5–5 N·m·s/rad.' }),
        col('torqueLimit', 'Limite de couple', 'percent', { min: 0.05, max: 1, step: 0.01, help: 'Fraction du couple disponible autorisée (protection mécanique).' }),
      ]),
      num('control.impedance.kTrans', 'Impédance — raideur linéaire', 'linStiffness', { min: 0, max: 1e5, step: 10, help: 'Raideur cartésienne du TCP en translation.' }),
      num('control.impedance.kRot', 'Impédance — raideur angulaire', 'stiffness', { min: 0, max: 1e3, step: 0.1, help: 'Raideur cartésienne en rotation.' }),
      num('control.impedance.zeta', 'Impédance — amortissement ζ', 'none', { min: 0, max: 3, step: 0.05, help: 'Taux d’amortissement critique visé (1 = critique).' }),
      num('control.impedance.nullKd', 'Impédance — amortissement articulaire', 'damping', { min: 0, max: 100, step: 0.01, help: 'Amortissement articulaire additionnel (espace nul, stabilité).' }),
      num('control.autotune.bandwidthHz', 'Auto-réglage — bande passante', 'frequency', { min: 0.5, max: 100, step: 0.5, help: 'Bande passante visée par le placement de pôles : Kp = J·ω², Kd = 2·ζ·J·ω, Ki = Kp·ω/10 avec J = inertie vue par l’axe.' }),
      num('control.autotune.zeta', 'Auto-réglage — amortissement ζ', 'none', { min: 0.3, max: 2, step: 0.05, help: '0,7 : réponse rapide avec léger dépassement ; 1 : critique, sans dépassement.' }),
    ],
  },
  {
    id: 'trajectory',
    title: 'Trajectoires & modèle inverse',
    icon: 'path',
    help: 'Génération de mouvements (profils de vitesse, lissage) et solveur de cinématique inverse.',
    params: [
      sel('trajectory.profile', 'Profil de vitesse', PROFILES, { help: 'Courbe en S : accélération continue (recommandé pour limiter les vibrations des réducteurs imprimés).' }),
      num('trajectory.speedOverride', 'Vitesse globale', 'percent', { min: 0.01, max: 1, step: 0.01, help: 'Facteur appliqué à toutes les vitesses programmées (override).' }),
      num('trajectory.accelScale', 'Facteur d’accélération', 'percent', { min: 0.01, max: 1, step: 0.01, help: 'Facteur appliqué aux accélérations et jerks maximaux.' }),
      num('trajectory.blendRadius', 'Zone de lissage par défaut', 'length', { min: 0, max: 0.2, step: 0.001, help: 'Rayon de passage (« fly-by ») entre deux mouvements consécutifs. 0 = arrêt précis sur chaque point.' }),
      num('trajectory.sampleHz', 'Échantillonnage cartésien', 'frequency', { min: 20, max: 2000, step: 10, help: 'Fréquence de discrétisation des trajectoires linéaires/circulaires (IK à chaque échantillon).' }),
      sel('trajectory.ik.method', 'Solveur IK', [
        ['auto', 'Automatique (analytique si possible, sinon numérique)'],
        ['analytic', 'Analytique (poignet sphérique)'],
        ['numeric', 'Numérique (moindres carrés amortis)'],
      ], { help: 'L’analytique donne jusqu’à 8 solutions exactes ; le numérique fonctionne pour toute géométrie.' }),
      sel('trajectory.ik.configuration', 'Configuration préférée', [
        ['closest', 'La plus proche de la position actuelle'],
        ['front-up-noflip', 'Épaule avant · coude haut · poignet normal'],
        ['front-down-noflip', 'Épaule avant · coude bas · poignet normal'],
        ['front-up-flip', 'Épaule avant · coude haut · poignet retourné'],
        ['back-up-noflip', 'Épaule arrière · coude haut · poignet normal'],
      ], { help: 'Choix parmi les solutions analytiques multiples.' }),
      num('trajectory.ik.damping', 'IK — amortissement λ', 'none', { min: 1e-5, max: 1, step: 0.001, help: 'Facteur d’amortissement des moindres carrés (robustesse près des singularités).' }),
      int('trajectory.ik.maxIter', 'IK — itérations max', { min: 5, max: 2000, help: 'Nombre maximal d’itérations du solveur numérique.' }),
      num('trajectory.ik.tolPos', 'IK — tolérance position', 'length', { min: 1e-8, max: 0.01, step: 1e-6 }),
      num('trajectory.ik.tolRot', 'IK — tolérance orientation', 'angle', { min: 1e-8, max: 0.1, step: 1e-5 }),
      num('trajectory.ik.orientationWeight', 'IK — poids orientation', 'lengthM', { min: 0.001, max: 2, step: 0.01, help: 'Longueur caractéristique (m/rad) équilibrant erreurs de position et d’orientation.' }),
      num('trajectory.jog.linStep', 'Jog — pas linéaire', 'length', { min: 0.0001, max: 0.1, step: 0.0001 }),
      num('trajectory.jog.angStep', 'Jog — pas angulaire', 'angle', { min: 0.001, max: 1, step: DEG / 10 }),
      num('trajectory.jog.linSpeed', 'Jog — vitesse linéaire', 'linvel', { min: 0.001, max: 1, step: 0.001, help: 'Vitesse du jog cartésien continu.' }),
      num('trajectory.jog.jointSpeed', 'Jog — vitesse articulaire', 'angvel', { min: 0.01, max: 5, step: DEG }),
    ],
  },
  {
    id: 'gripper',
    title: 'Pince / outil',
    icon: 'gripper',
    help: 'Pince parallèle à crémaillère entraînée par servomoteur (MG996R par défaut).',
    params: [
      sel('gripper.type', 'Type', [['parallel', 'Pince parallèle (servo)'], ['vacuum', 'Ventouse (pompe)'], ['none', 'Aucun']], {}),
      num('gripper.strokeMax', 'Ouverture max', 'length', { min: 0.001, max: 0.3, step: 0.0005, help: 'Écartement maximal des doigts.' }),
      num('gripper.speed', 'Vitesse des doigts', 'linvel', { min: 0.001, max: 1, step: 0.001 }),
      num('gripper.force', 'Force de serrage', 'force', { min: 0, max: 500, step: 0.5, help: 'Force nominale (servo en butée sur l’objet).' }),
      num('gripper.fingerLength', 'Longueur des doigts', 'length', { min: 0.005, max: 0.3, step: 0.0005 }),
      int('gripper.servoMinUs', 'Servo — impulsion fermée', { min: 400, max: 2600, help: 'Largeur d’impulsion PWM (µs) pince fermée.' }),
      int('gripper.servoMaxUs', 'Servo — impulsion ouverte', { min: 400, max: 2600, help: 'Largeur d’impulsion PWM (µs) pince ouverte.' }),
    ],
  },
  {
    id: 'safety',
    title: 'Sécurité',
    icon: 'shield',
    help: 'Surveillance des collisions, vitesses réduites et chien de garde de communication.',
    params: [
      bool('safety.collisionDetection', 'Détection de collision (couple)', { help: 'Compare le couple mesuré/commandé au couple prédit par le modèle ; au-delà du seuil → arrêt.' }),
      num('safety.collisionThreshold', 'Seuil de collision', 'torque', { min: 0.05, max: 50, step: 0.05, help: 'Écart de couple (N·m, côté articulation) déclenchant l’arrêt.' }),
      bool('safety.selfCollision', 'Auto-collision', { help: 'Vérifie les collisions entre segments non adjacents (modèle en capsules).' }),
      bool('safety.floorCollision', 'Collision avec la table', { help: 'Vérifie que le bras et l’outil restent au-dessus du plan de travail.' }),
      num('safety.capsuleMargin', 'Marge de collision', 'length', { min: 0, max: 0.05, step: 0.001, help: 'Distance minimale tolérée entre capsules.' }),
      num('safety.reducedSpeed', 'Vitesse réduite (mode manuel)', 'percent', { min: 0.01, max: 1, step: 0.01, help: 'Limite de vitesse en mode apprentissage (≈ 250 mm/s en industrie).' }),
      num('safety.collabTcpSpeed', 'Vitesse TCP collaborative', 'linvel', { min: 0.01, max: 2, step: 0.01, help: 'Vitesse TCP maximale quand le mode collaboratif est actif.' }),
      int('safety.watchdogMs', 'Chien de garde (ms)', { min: 20, max: 5000, help: 'Sans nouvelle consigne pendant ce délai, le robot freine et s’arrête (flux temps réel).' }),
      num('safety.softLimitMargin', 'Marge butées logicielles', 'angle', { min: 0, max: 0.2, step: 0.001, help: 'Marge de décélération avant les butées.' }),
    ],
  },
  {
    id: 'simulation',
    title: 'Simulation',
    icon: 'cpu',
    help: 'Intégration numérique et effets physiques simulés.',
    params: [
      num('simulation.dt', 'Pas physique', 'timeMs', { min: 5e-5, max: 0.005, step: 5e-5, help: 'Pas d’intégration (intégrateur semi-implicite). 0,25 ms reproduit fidèlement la raideur magnétique des pas-à-pas.' }),
      num('simulation.realtimeFactor', 'Facteur temps réel', 'none', { min: 0.05, max: 10, step: 0.05, help: '1 = temps réel ; < 1 ralenti ; > 1 accéléré.' }),
      bool('simulation.enableFriction', 'Frottements', {}),
      bool('simulation.enableBacklash', 'Jeu des réducteurs', {}),
      bool('simulation.enableStepLoss', 'Perte de pas (pas-à-pas)', { help: 'Modèle physique couple-angle : si la charge dépasse le couple disponible, le rotor décroche.' }),
      num('simulation.sensorNoise', 'Bruit de mesure', 'angle', { min: 0, max: 0.01, step: 1e-5, help: 'Écart-type du bruit ajouté aux positions mesurées.' }),
      num('simulation.recordHz', 'Fréquence d’enregistrement', 'frequency', { min: 10, max: 2000, step: 10, help: 'Échantillonnage des courbes et exports CSV.' }),
      num('simulation.plotWindow', 'Fenêtre des courbes', 'time', { min: 1, max: 120, step: 1 }),
    ],
  },
  {
    id: 'hardware',
    title: 'Matériel & firmware',
    icon: 'chip',
    help: 'Brochage, drivers, prise d’origine. Exporté dans firmware/orion_fw/src/orion_config.h.',
    params: [
      sel('hardware.controller', 'Contrôleur', [['teensy41', 'Teensy 4.1 (recommandé)'], ['mit_can_bridge', 'Teensy 4.1 + CAN (moteurs mode MIT)']], {}),
      num('hardware.stepPulse', 'Largeur impulsion STEP', 'timeUs', { min: 1e-6, max: 1e-4, step: 1e-7, help: 'DM542T : ≥ 2,5 µs. TMC2209 : ≥ 0,1 µs.' }),
      num('hardware.dirSetup', 'Délai DIR → STEP', 'timeUs', { min: 0, max: 1e-4, step: 1e-7, help: 'Temps d’établissement de la direction avant l’impulsion (DM542T : ≥ 5 µs).' }),
      bool('hardware.enableActiveLow', 'ENABLE actif à l’état bas', { help: 'Dépend du câblage des drivers (optocoupleurs DM542T : ENA+ au 5 V, ENA− piloté).' }),
      num('hardware.streamHz', 'Fréquence de flux USB', 'frequency', { min: 10, max: 1000, step: 10, help: 'Fréquence d’envoi des consignes articulaires en mode jumeau numérique (streaming).' }),
      num('hardware.streamDelay', 'Lissage du flux (retard)', 'timeMs', { min: 0, max: 0.4, step: 0.005, help: 'Avance du tampon d’interpolation du firmware : absorbe la gigue USB/navigateur. Plus grand = plus lisse mais plus de retard (40 ms conseillé).' }),
      num('hardware.canBitrate', 'Débit CAN', 'bitrate', { min: 125000, max: 1000000, step: 125000, help: 'Débit du bus CAN (mode MIT) — 1 Mbit/s en standard.' }),
      num('hardware.mitCtrlHz', 'Fréquence de commande CAN (MIT)', 'frequency', { min: 50, max: 1000, step: 50, help: 'Trames MIT envoyées par moteur et par seconde. Chaque trame reçoit une réponse : 6 moteurs à 400 Hz ≈ 60 % d’un bus à 1 Mbit/s.' }),
      num('hardware.feedbackTimeout', 'Délai max. sans retour CAN', 'timeMs', { min: 0.01, max: 1, step: 0.005, help: 'Au-delà, le pont passe en défaut et amortit tous les axes.' }),
      num('hardware.maxMotorTemp', 'Température moteur max.', 'none', { min: 40, max: 120, step: 1, help: '°C (MOSFET ou bobinage) avant mise en défaut.' }),
      table('hardware.pins', 'Brochage Teensy', [
        colInt('step', 'STEP', { min: 0, max: 54 }),
        colInt('dir', 'DIR', { min: 0, max: 54 }),
        colInt('limit', 'Capteur d’origine', { min: 0, max: 54 }),
        colBool('invertDir', 'Inverser DIR', { help: 'Inverse le sens de rotation physique (câblage moteur).' }),
        colBool('limitActiveLow', 'Capteur actif bas', { help: 'Capteur NPN/interrupteur vers la masse avec pull-up : actif à l’état bas.' }),
      ]),
      int('hardware.enablePin', 'Broche ENABLE commune', { min: 0, max: 54 }),
      int('hardware.estopPin', 'Broche arrêt d’urgence', { min: 0, max: 54 }),
      int('hardware.gripperPin', 'Broche servo pince', { min: 0, max: 54 }),
      table('hardware.homing', 'Prise d’origine', [
        colBool('enabled', 'Active'),
        col('direction', 'Sens de recherche', 'none', { min: -1, max: 1, step: 2, help: '−1 ou +1 : sens de déplacement vers le capteur.' }),
        col('speed', 'Vitesse de recherche', 'angvel', { min: 0.001, max: 2, step: DEG }),
        col('slowSpeed', 'Vitesse d’approche fine', 'angvel', { min: 0.001, max: 1, step: DEG / 10 }),
        col('backoff', 'Recul', 'angle', { min: 0, max: 0.5, step: DEG / 10 }),
        col('switchPosition', 'Position du capteur', 'angle', { min: -2 * Math.PI, max: 2 * Math.PI, step: DEG / 10, help: 'Valeur articulaire q au point de déclenchement du capteur (à calibrer).' }),
        colInt('order', 'Ordre', { min: 1, max: 12, help: 'Ordre de prise d’origine (poignet d’abord recommandé).' }),
      ]),
    ],
  },
  {
    id: 'poses',
    title: 'Poses nommées',
    icon: 'pin',
    help: 'Positions articulaires de référence utilisables dans les programmes (MoveJ Home…).',
    params: [
      jointvec('poses.home', 'Home (travail)', 'angle', { help: 'Pose de départ des programmes.' }),
      jointvec('poses.rest', 'Repos (transport)', 'angle', { help: 'Pose repliée, compacte et stable hors tension.' }),
      jointvec('poses.zero', 'Zéro mécanique', 'angle', { help: 'Toutes les articulations à 0 (vérification de la table DH).' }),
    ],
  },
];

// -------------------------------------------------------------- utilitaires

/** Lit une valeur par chemin « a.b.0.c ». */
export function getPath(obj, path) {
  let o = obj;
  for (const k of path.split('.')) {
    if (o == null) return undefined;
    o = o[k];
  }
  return o;
}

/** Écrit une valeur par chemin (crée les objets intermédiaires). */
export function setPath(obj, path, value) {
  const keys = path.split('.');
  let o = obj;
  for (let i = 0; i < keys.length - 1; i++) {
    const k = keys[i];
    if (o[k] == null) o[k] = /^\d+$/.test(keys[i + 1]) ? [] : {};
    o = o[k];
  }
  o[keys[keys.length - 1]] = value;
}

/** Liste à plat de tous les descripteurs (tables développées colonne par colonne). */
export function flattenSchema() {
  const out = [];
  for (const section of SCHEMA) {
    for (const p of section.params) {
      if (p.kind === 'table') {
        for (const c of p.columns) out.push({ ...c, section: section.id, table: p.path, tableLabel: p.label });
      } else {
        out.push({ ...p, section: section.id });
      }
    }
  }
  return out;
}

/** Valide et borne une configuration ; retourne la liste des avertissements. */
export function validateParams(params) {
  const warnings = [];
  const clampNum = (v, d, where) => {
    if (typeof v !== 'number' || Number.isNaN(v)) {
      warnings.push(`${where} : valeur non numérique`);
      return d.min ?? 0;
    }
    if (d.min !== undefined && v < d.min) { warnings.push(`${where} : ${v} < min ${d.min}`); return d.min; }
    if (d.max !== undefined && v > d.max) { warnings.push(`${where} : ${v} > max ${d.max}`); return d.max; }
    return v;
  };
  for (const section of SCHEMA) {
    for (const p of section.params) {
      if (p.kind === 'table') {
        const rows = getPath(params, p.path);
        if (!Array.isArray(rows)) { warnings.push(`${p.path} manquant`); continue; }
        rows.forEach((row, j) => {
          for (const c of p.columns) {
            if (c.kind !== 'number' && c.kind !== 'int') continue;
            const v = getPath(row, c.key);
            if (v === undefined) continue;
            const nv = clampNum(v, c, `${p.path}[${j}].${c.key}`);
            if (nv !== v) setPath(row, c.key, nv);
          }
        });
      } else if (p.kind === 'number' || p.kind === 'int') {
        const v = getPath(params, p.path);
        if (v === undefined) continue;
        const nv = clampNum(v, p, p.path);
        if (nv !== v) setPath(params, p.path, nv);
      }
    }
  }
  // Cohérence des butées
  const lim = params.limits?.joints || [];
  lim.forEach((l, i) => {
    if (l.min >= l.max) warnings.push(`Axe ${i + 1} : butée min ≥ butée max`);
  });
  return warnings;
}
