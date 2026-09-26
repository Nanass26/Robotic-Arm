// FICHIER GÉNÉRÉ par tools/build-cad.mjs — ne pas modifier à la main.
// Valeurs provisoires (estimations) en attendant la génération depuis la CAO.
// Repères : DH standard de chaque segment (voir docs/04-cinematique.md).

export const MASS_PROPERTIES = {
  maker: {
    source: 'estimation',
    flangeD6: 0.045,
    tcpZ: 0.13,
    links: [
      { mass: 1.30, com: [0, 0.02, -0.01], inertia: [0.0022, 0.0022, 0.0022, 0, 0, 0] },
      { mass: 1.10, com: [-0.05, 0, 0.07], inertia: [0.0015, 0.008, 0.008, 0, 0, 0] },
      { mass: 0.70, com: [0, 0, 0.06], inertia: [0.0012, 0.0012, 0.0006, 0, 0, 0] },
      { mass: 0.65, com: [0, -0.06, 0.05], inertia: [0.0015, 0.0008, 0.0015, 0, 0, 0] },
      { mass: 0.47, com: [0, -0.02, 0], inertia: [0.0006, 0.0006, 0.0004, 0, 0, 0] },
      { mass: 0.08, com: [0, 0, -0.01], inertia: [3e-5, 3e-5, 4e-5, 0, 0, 0] },
    ],
    tool: { mass: 0.22, com: [0, 0, 0.05], inertiaDiag: [4e-4, 4e-4, 1.5e-4] },
  },
  pro: {
    source: 'estimation',
    flangeD6: 0.045,
    tcpZ: 0.13,
    links: [
      { mass: 0.90, com: [0, 0.02, -0.01], inertia: [0.0015, 0.0015, 0.0015, 0, 0, 0] },
      { mass: 0.80, com: [-0.05, 0, 0.06], inertia: [0.001, 0.006, 0.006, 0, 0, 0] },
      { mass: 0.55, com: [0, 0, 0.06], inertia: [0.0009, 0.0009, 0.0005, 0, 0, 0] },
      { mass: 0.55, com: [0, -0.06, 0.04], inertia: [0.0012, 0.0007, 0.0012, 0, 0, 0] },
      { mass: 0.45, com: [0, -0.02, 0], inertia: [0.0005, 0.0005, 0.0004, 0, 0, 0] },
      { mass: 0.08, com: [0, 0, -0.01], inertia: [3e-5, 3e-5, 4e-5, 0, 0, 0] },
    ],
    tool: { mass: 0.22, com: [0, 0, 0.05], inertiaDiag: [4e-4, 4e-4, 1.5e-4] },
  },
};
