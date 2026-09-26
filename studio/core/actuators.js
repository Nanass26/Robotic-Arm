// ORION-6 — Modèles d’actionneurs (grandeurs ramenées côté articulation).
//
// Chaque articulation est modélisée comme une « articulation flexible » (Spong) :
//   rotor (inertie N²·J_rotor) —[ressort de transmission + jeu]— segment de sortie
//
//   • Pas-à-pas : couple magnétique τ = N·T(ω)·sin(Nr·N·(θcmd − θr)), où Nr = nombre de dents du
//     rotor (50 pour 1,8°). Si la charge dépasse le couple disponible, le rotor décroche (perte de pas).
//     Chute de couple au-delà de la vitesse de coin ωc = V / (I·Nr·L + ke).
//   • Brushless QDD / servo : source de couple limitée par une droite couple-vitesse.
//   • Frottements côté sortie : Coulomb + adhérence (Stribeck) + visqueux, lissés.

const TAU = 2 * Math.PI;
const sigmoid = (x) => 1 / (1 + Math.exp(-x));
const softplus = (x) => (x > 30 ? x : Math.log1p(Math.exp(x)));

export class ActuatorModel {
  constructor(ap, supplyVoltage = 24) {
    this.set(ap, supplyVoltage);
  }

  set(ap, supplyVoltage) {
    this.p = { ...ap };
    this.type = ap.type;
    this.G = Math.max(1, ap.gearRatio);
    this.eta = Math.min(1, Math.max(0.05, ap.efficiency));
    this.Ja = this.G * this.G * Math.max(ap.rotorInertia, 1e-9); // inertie rotor ramenée en sortie
    this.kt = Math.max(ap.stiffness || 5000, 1);
    this.backlash = Math.max(ap.backlash || 0, 0);
    this.V = supplyVoltage;
    // Amortissement structurel de la transmission (ζ ≈ 0,3 sur le mode rotor)
    this.ct = 2 * 0.3 * Math.sqrt(this.kt * this.Ja);
    if (this.type === 'stepper') {
      const stepsPerRev = TAU / Math.max(ap.stepAngle, 1e-4);
      this.Nr = stepsPerRev / 4; // dents rotor (2 phases : 4 pas entiers par période électrique)
      const iRatio = ap.ratedCurrent > 0 ? ap.runCurrent / ap.ratedCurrent : 1;
      this.Trun = ap.holdingTorque * Math.min(iRatio, 1.2); // couple de maintien au courant réglé
      const ke = ap.ratedCurrent > 0 ? ap.holdingTorque / ap.ratedCurrent : 0.3; // ≈ constante de FCEM (V·s/rad)
      this.omegaCorner = this.V / (Math.max(ap.runCurrent, 0.1) * this.Nr * Math.max(ap.phaseInductance, 1e-5) + ke);
      this.microStep = (ap.stepAngle / Math.max(ap.microsteps, 1)) / this.G; // résolution en sortie (rad)
    }
  }

  /** Couple magnétique disponible au rotor (N·m moteur) à la vitesse moteur ωm (rad/s). */
  stepperMotorTorque(omegaMotor) {
    const w = Math.abs(omegaMotor);
    const pullout = 0.85 * this.Trun; // couple de décrochage ≈ 85 % du maintien à basse vitesse
    return w <= this.omegaCorner ? pullout : pullout * this.omegaCorner / w;
  }

  /**
   * Capacité de couple en sortie (N·m côté articulation) à la vitesse articulaire ω.
   * `tauSign` : signe du couple demandé (le freinage régénératif reste possible au-delà de la vitesse à vide).
   */
  capacity(omegaJoint, tauSign = 1) {
    if (this.type === 'stepper') {
      return this.G * this.eta * this.stepperMotorTorque(omegaJoint * this.G);
    }
    const peak = this.p.peakTorque;
    const wmax = Math.max(this.p.maxSpeed, 1e-3);
    if (tauSign * omegaJoint < 0) return peak; // freinage
    return peak * Math.max(0, Math.min(1, 1 - Math.abs(omegaJoint) / wmax));
  }

  /** Quantification de la consigne pas-à-pas à la résolution micro-pas (sortie). */
  quantize(qCmd) {
    if (this.type !== 'stepper') return qCmd;
    return Math.round(qCmd / this.microStep) * this.microStep;
  }

  /**
   * Couple magnétique pas-à-pas (côté articulation) sur le rotor.
   * Retourne { tau, k (raideur locale ≥ 0), c (amortissement), elec (erreur électrique rad) }.
   */
  stepperTorque(thetaCmd, thetaR, omegaR) {
    const elec = this.Nr * this.G * (thetaCmd - thetaR);
    const Tm = this.stepperMotorTorque(omegaR * this.G) / 0.85; // amplitude (la sinusoïde atteint le maintien)
    const amp = this.G * Tm;
    const tau = amp * Math.sin(elec);
    const k = Math.max(0, amp * this.Nr * this.G * Math.cos(elec));
    // Amortissement du driver/magnétique faible (résonances des pas-à-pas) : ζ ≈ 0,12.
    // Le simulateur l’applique sur la vitesse relative (ωcmd − ωr).
    const kMax = amp * this.Nr * this.G;
    const c = 2 * 0.12 * Math.sqrt(kMax * this.Ja);
    return { tau, k, c, elec };
  }

  /**
   * Couple transmis par le réducteur (ressort + jeu lissé) du rotor vers la sortie.
   * δ = θr − q. Retourne { tau, k, c } (k et c : dérivées locales).
   */
  transmission(thetaR, q, omegaR, v, backlashOn = true) {
    const d = thetaR - q;
    const dv = omegaR - v;
    const h = backlashOn ? this.backlash / 2 : 0;
    if (h <= 1e-12) {
      return { tau: this.kt * d + this.ct * dv, k: this.kt, c: this.ct, contact: 1, deflection: d };
    }
    // Zone morte lissée : dz(δ) = s·[softplus((δ−h)/s) − softplus((−δ−h)/s)]
    const s = Math.max(h / 12, 1e-7);
    const dz = s * (softplus((d - h) / s) - softplus((-d - h) / s));
    const contact = sigmoid((d - h) / s) + sigmoid((-d - h) / s);
    return { tau: this.kt * dz + this.ct * dv * contact, k: this.kt * contact, c: this.ct * contact, contact, deflection: dz };
  }
}

/**
 * Frottement côté sortie : modèle de Stribeck lissé.
 *   τf(v) = [Fc + (Fs − Fc)·exp(−(v/vs)²)]·tanh(v/ε) + b·v
 * Retourne { tau, slope } (slope = dτf/dv ≥ 0 pour le traitement implicite).
 */
export function frictionTorque(v, fp, eps = 2e-3) {
  const Fc = fp.coulomb, Fs = Math.max(fp.stiction, fp.coulomb), vs = Math.max(fp.stribeckVel, 1e-4), b = fp.viscous;
  const e = Math.exp(-((v / vs) ** 2));
  const mag = Fc + (Fs - Fc) * e;
  const th = Math.tanh(v / eps);
  const tau = mag * th + b * v;
  const slope = mag * (1 - th * th) / eps + b; // (on néglige la dérivée de l’enveloppe de Stribeck)
  return { tau, slope };
}

/** Estimation de frottement pour l’anticipation (sans la raideur de lissage). */
export function frictionEstimate(v, fp) {
  if (Math.abs(v) < 1e-4) return 0;
  return Math.sign(v) * fp.coulomb + fp.viscous * v;
}
