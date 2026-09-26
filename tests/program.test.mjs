import { test } from 'node:test';
import assert from 'node:assert/strict';
import { parseProgram, ProgramRunner, DEMO_PROGRAMS } from '../studio/core/program.js';
import { interpret } from '../studio/core/nlp.js';
import { Simulator } from '../studio/core/simulator.js';
import { makeOrion6Maker } from '../studio/core/defaults.js';
import { mat4FromRotPos, mat4Pos } from '../studio/core/math3d.js';

test('Analyseur ORL : points, options, blocs et erreurs', () => {
  const r = parseProgram(`# test
POINT P1 = J(0, 10, 20, 0, 60, 0)
POINT P2 = X(250, 0, 150, 180, 0, 0)
VITESSE 50
MOVEJ P1 v=80 z=10
REPETER 2
  MOVEL P2 v=100
  PINCE FERMER
FIN
WAIT 0.5
MESSAGE "fini # vraiment"`);
  assert.equal(r.errors.length, 0, r.errors.map((e) => e.message).join('\n'));
  assert.equal(r.instructions.length, 5);
  assert.equal(r.instructions[1].opts.z, 10);
  assert.equal(r.instructions[2].body.length, 2);
  assert.equal(r.instructions[4].text, 'fini # vraiment');
  const bad = parseProgram('MOVEJ\nFOO 3\nREPETER 2');
  assert.equal(bad.errors.length, 3);
});

test('Assistant langage naturel (FR/EN)', () => {
  const ctx = { q: [0, 0, 0, 0, 90, 0], tcp: { x: 260, y: 0, z: 100 }, points: ['P1'] };
  let r = interpret('monte de 5 cm puis ouvre la pince', ctx);
  assert.deepEqual(r.lines, ['MOVEL REL(0, 0, 50) v=80', 'PINCE OUVRIR']);
  r = interpret('tourne la base de 30 degrés à gauche', ctx);
  assert.deepEqual(r.lines, ['MOVEJ J(30, 0, 0, 0, 90, 0)']);
  r = interpret("mets l'axe 3 à -20°", ctx);
  assert.deepEqual(r.lines, ['MOVEJ J(0, 0, -20, 0, 90, 0)']);
  r = interpret('va à la position repos', ctx);
  assert.deepEqual(r.lines, ['MOVEJ REPOS']);
  r = interpret('prends le cube en 260 -120', ctx);
  assert.equal(r.lines.length, 5);
  r = interpret('dessine un carré de 10 cm', ctx);
  assert.equal(r.lines.length, 5);
  r = interpret('vitesse 30 %', ctx);
  assert.deepEqual(r.lines, ['VITESSE 30']);
  r = interpret('move down 20 mm and open the gripper', ctx);
  assert.deepEqual(r.lines, ['MOVEL REL(0, 0, -20) v=80', 'PINCE OUVRIR']);
  r = interpret('arrête', ctx);
  assert.equal(r.actions[0].type, 'stop');
  r = interpret('blablabla incompréhensible', ctx);
  assert.equal(r.understood, false);
});

function runProgram(text, sim, maxT = 60) {
  const logs = [];
  let err = null;
  const runner = new ProgramRunner(sim, text, {}, { log: (m) => logs.push(m), onError: (e) => { err = e; } });
  assert.equal(runner.errors.length, 0, runner.errors.map((e) => e.message).join('\n'));
  runner.start();
  const dt = sim.params.simulation.dt;
  while (sim.t < maxT) {
    for (let k = 0; k < 40; k++) sim.step(dt); // 10 ms
    const st = runner.tick();
    if (st === 'done' || st === 'error') break;
  }
  return { runner, logs, err };
}

test('Programme de prise-dépose exécuté en simulation (pas-à-pas)', () => {
  const p = makeOrion6Maker();
  const cube = { id: 'c1', size: 0.03, mass: 0.05, pose: mat4FromRotPos([1, 0, 0, 0, 1, 0, 0, 0, 1], [0.26, -0.12, 0.015]) };
  const sim = new Simulator(p, { objects: [cube] });
  const { runner, logs, err } = runProgram(DEMO_PROGRAMS['Prise et dépose (cubes)'], sim);
  assert.equal(err, null, err?.message);
  assert.equal(runner.state, 'done');
  assert.ok(logs.includes('Cycle terminé'));
  const c = mat4Pos(cube.pose);
  assert.ok(Math.abs(c[0] - 0.26) < 0.01 && Math.abs(c[1] - 0.12) < 0.01, `cube déplacé en B : ${c.map((v) => v.toFixed(3))}`);
  assert.ok(Math.abs(c[2] - 0.015) < 0.002, 'cube posé sur la table');
  assert.equal(sim.lostSteps.reduce((a, b) => a + b, 0), 0, `pas perdus : ${sim.lostSteps}`);
});

test('Tous les programmes de démonstration s’exécutent sans erreur', () => {
  for (const [name, text] of Object.entries(DEMO_PROGRAMS)) {
    const sim = new Simulator(makeOrion6Maker());
    const { runner, err } = runProgram(text, sim, 90);
    assert.equal(err, null, `${name} : ${err?.message}`);
    assert.equal(runner.state, 'done', `${name} : état ${runner.state}, défaut ${sim.fault}`);
    assert.equal(sim.fault, null, `${name} : ${sim.fault}`);
  }
});
