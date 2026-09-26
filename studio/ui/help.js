// ORION Studio — aide intégrée.

import { h } from './dom.js';

const HTML = `
<h3>Démarrage rapide</h3>
<ul>
  <li><b>Pilotage</b> : maintenez <code>−</code> / <code>+</code> pour déplacer un axe, ou faites glisser le curseur. Les boutons <code>X± Y± Z±</code> déplacent le point outil (TCP) en ligne droite.</li>
  <li><b>Vue 3D</b> : faites glisser les flèches du gizmo pour amener le TCP où vous voulez (le modèle inverse calcule les angles en direct). Clic droit / deux doigts : déplacer la vue.</li>
  <li><b>Programme</b> : chargez l’exemple « Prise et dépose », puis <kbd>Exécuter</kbd>. Le robot saisit le cube bleu et le dépose de l’autre côté.</li>
  <li><b>Assistant</b> : écrivez « monte de 5 cm puis ouvre la pince » ou dictez-le au micro.</li>
  <li><b>Paramètres</b> : chaque réglage (géométrie DH, masses, moteurs, réducteurs, gains PID et mode MIT, trajectoires, sécurité…) est modifiable en direct ; survolez <code>?</code> pour l’explication.</li>
</ul>
<h3>Raccourcis clavier</h3>
<ul>
  <li><kbd>Espace</kbd> : arrêt contrôlé du mouvement · <kbd>Échap</kbd> : arrêt d’urgence</li>
  <li><kbd>T</kbd> / <kbd>R</kbd> : gizmo en translation / rotation · <kbd>G</kbd> : afficher/masquer le gizmo</li>
  <li><kbd>1</kbd>–<kbd>4</kbd> : vues iso / face / côté / dessus</li>
  <li><kbd>Maj</kbd> + glisser sur le bras : pousser le robot (utile en modes <i>compensation de gravité</i> et <i>impédance</i>)</li>
  <li><kbd>Ctrl</kbd>+<kbd>Entrée</kbd> dans l’éditeur : exécuter le programme</li>
</ul>
<h3>Modes de commande</h3>
<ul>
  <li><b>Pas-à-pas</b> : comme le vrai ORION-6 MAKER. La simulation reproduit le couple magnétique, la chute de couple en vitesse, le jeu du réducteur et la <i>perte de pas</i> si le moteur est trop faible.</li>
  <li><b>PID</b> : régulateur par axe avec anticipations (gravité, inertie, frottements), anti-emballement et filtre de dérivée.</li>
  <li><b>Mode MIT</b> : τ = Kp·(p*−p) + Kd·(v*−v) + τff, calculé dans le driver du moteur (Damiao, CubeMars, MIT Mini Cheetah). C’est le mode des bras de recherche à moteurs brushless.</li>
  <li><b>Couple calculé</b> : linéarisation exacte par le modèle dynamique ; testez sa robustesse avec « Erreur de modèle ».</li>
  <li><b>Impédance</b> : le TCP se comporte comme un ressort amorti ; poussez-le avec <kbd>Maj</kbd>+glisser.</li>
  <li><b>Compensation de gravité</b> : le bras « flotte » ; idéal pour l’apprentissage à la main.</li>
</ul>
<h3>Réglage conseillé</h3>
<ol>
  <li>Vérifiez la table DH (pose <i>Zéro</i> : bras supérieur vertical, avant-bras horizontal).</li>
  <li>Analyse → <i>Dimensionnement</i> : toutes les marges doivent être « OK ».</li>
  <li>Analyse → <i>Réponse indicielle</i> puis <i>Auto-régler</i> ; ajustez la bande passante (section Commande).</li>
  <li>Lancez « Test de répétabilité » et surveillez les pas perdus et l’erreur de suivi (onglet Courbes).</li>
</ol>
<h3>Robot réel</h3>
<p>Onglet <b>Robot réel</b> (Chrome/Edge sur ordinateur) : <i>Connecter (USB)</i> → <i>Envoyer la config</i> → <i>Activer</i> → <i>Référencer</i> (prise d’origine sur capteurs, ou pose de repos pour le pont MIT) → <i>Synchroniser</i> → <i>Jumeau numérique</i> : le vrai bras suit la simulation (consignes horodatées, lissées par le firmware ; compensation de gravité transmise au pont MIT). Le dossier <code>docs/</code> du dépôt détaille l’impression, l’assemblage, le câblage, la calibration et le protocole.</p>
<h3>Fichiers exportables</h3>
<p>Paramètres → <code>JSON</code> (configuration complète), <code>URDF</code> (ROS / MoveIt), <code>MJCF</code> (MuJoCo), <code>Firmware .h</code> (compilation Teensy) et <code>SET</code> (réglage série sans recompiler). Courbes → <code>CSV</code>. Pièces → <code>STL</code>.</p>
`;

export function buildHelp(container) {
  container.replaceChildren(h('div.doc', { html: HTML }));
}
