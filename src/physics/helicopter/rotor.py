"""
Rotor aerodynamics of the helicopter template (main rotor + ducted tail rotor).

Main rotor (blade-element + momentum, uniform inflow, quasi-steady)
------------------------------------------------------------------
    C_T = (sigma a / 2) [ theta_75/3 (1 + 3/2 mu^2) - lambda/2 ]          blade element, linear twist
    lambda = lambda_c + lambda_i                                          inflow through the disk (+ down)
    lambda_i = k_GE * C_T / (2 sqrt(mu^2 + lambda^2))                     Glauert
    near vertical descent (-2 <= V_c/v_h <= 0, V_x < v_h): empirical induced velocity
        v_i/v_h = 1 + k1 x + k2 x^2 + k3 x^3 + k4 x^4,  x = V_c/v_h       (Johnson 1980, in Leishman 2006)
    P_main = T (V_c + kappa v_i) + P_0 (1 + k_mu mu^2),   P_0 = sigma Cd0/8 rho A V_t^3
In the 6-DoF simulation V_c is the air velocity through the tip-path plane, so the parasite
power of forward flight (disk tilted forward against the fuselage drag) appears by itself.

Ground effect (Cheeseman & Bennett 1955), at constant thrust the induced velocity scales by
    k_GE = 1 - (R/4z)^2 / (1 + (V/v_i)^2)
with z/R limited to >= 0.5: the formula is singular at z/R = 0.25 (infinite thrust ratio) and the
authors report agreement with flight data for z/R > 0.6.

Tail rotor (ducted fan, expansion ratio 1): P = kappa T^1.5 / sqrt(4 rho A) + profile power.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List

import numpy as np
from scipy.optimize import brentq, minimize_scalar

from .isa import G0
from .params import HeliParams

# empirical induced velocity in vertical descent (Johnson 1980, reproduced in Leishman 2006); coefficients to confirm
K_EMP = (-1.125, -1.372, -1.718, -0.655)
ZR_MIN = 0.5
VRS_BAND = (-1.5, -0.5)  # V_c/v_h (Johnson, NASA/TP-2005-213477: unstable vertical motion about -0.5 to -1.5)
VRS_VX_MAX = 1.0  # V_x/v_h above which VRS effects disappear (Taghizad flight tests, same report)


def ground_effect_factor(z_hub: float, R: float, V: float, v_i: float) -> float:
    if not math.isfinite(z_hub):
        return 1.0
    zr = max(z_hub / R, ZR_MIN)
    speed_term = 1.0 + (V / max(v_i, 1e-3)) ** 2
    return 1.0 - (1.0 / (4.0 * zr)) ** 2 / speed_term


def _empirical_vi_over_vh(x: float) -> float:
    k1, k2, k3, k4 = K_EMP
    return 1.0 + k1 * x + k2 * x ** 2 + k3 * x ** 3 + k4 * x ** 4


@dataclass
class RotorState:
    ct: float
    thrust: float  # N
    lam: float
    lam_i: float
    v_i: float  # m/s
    v_h: float  # hover induced velocity for this thrust, m/s
    mu: float
    k_ge: float
    vrs: bool
    power: float  # W, aerodynamic shaft power of the main rotor
    p_induced: float
    p_profile: float
    p_axial: float


def induced_ratio(x: float, y: float) -> float:
    """Normalised induced velocity v_i/v_h for climb/descent ratio x = V_c/v_h and in-plane ratio y = V_x/v_h.

    Momentum theory (Glauert) everywhere it is valid, taking the smallest positive root of
    v (y^2 + (x+v)^2)^0.5 = 1 (normal-working and windmill branches); in the vortex-ring region
    (-2 <= x <= 0, y < 1) blended with the empirical curve, weight 1 - y.
    """
    if x >= 0.0 or y > 1.5:
        v = 1.0
    else:
        v = min(1.0, 1.0 / max(math.hypot(x, y), 1e-6))
    for _ in range(12):
        s = math.sqrt(y * y + (x + v) ** 2)
        f = v * s - 1.0
        df = s + v * (x + v) / max(s, 1e-9)
        if abs(df) < 1e-9:
            break
        v_new = v - f / df
        v = max(v_new, 1e-4) if v_new > 0 else 0.5 * v
        if abs(f) < 1e-10:
            break
    if -2.0 <= x <= 0.0 and y < 1.0:
        w = 1.0 - y
        v = (1.0 - w) * v + w * _empirical_vi_over_vh(x)
    return v


def main_rotor(p: HeliParams, theta75: float, v_tpp: float, v_inplane: float, rho: float, omega: float,
               z_hub: float = math.inf, lam_i0: float | None = None, iters: int = 30) -> RotorState:
    """Quasi-steady main rotor. v_tpp: air speed through the disk (+ = rotor moving up / air going down).

    The induced velocity is the root of  v_i = k_GE * v_h(T(v_i)) * induced_ratio(V_c/v_h, V_x/v_h),
    with T(v_i) from the blade-element relation; negative thrust uses the same curve mirrored
    (flow reversed). Solved by bracketing (Brent), so there is one well-defined branch at every
    state, including near zero thrust (`lam_i0`, `iters` kept for API compatibility, unused).
    """
    vt = max(omega * p.R, 1e-3)
    A = p.area
    mu = v_inplane / vt
    lam_c = v_tpp / vt
    s_a2 = p.sigma * p.a / 2.0
    vc = v_tpp

    def thrust(vi):
        ct = s_a2 * (theta75 / 3.0 * (1.0 + 1.5 * mu * mu) - (lam_c + vi / vt) / 2.0)
        return ct, ct * rho * A * vt * vt

    def residual(vi):
        ct, T = thrust(vi)
        vh = math.sqrt(abs(T) / (2.0 * rho * A))
        if vh < 1e-6:
            return vi, vh, 1.0
        sign = 1.0 if T >= 0 else -1.0
        x = sign * vc / vh
        y = v_inplane / vh
        vhat = induced_ratio(x, y)
        k = ground_effect_factor(z_hub, p.R, v_inplane, max(vh * vhat, 0.1)) if T > 0 else 1.0
        return vi - sign * k * vh * vhat, vh, k

    lo, hi = -60.0, 80.0
    try:
        vi = brentq(lambda v: residual(v)[0], lo, hi, xtol=1e-6, maxiter=100)
    except ValueError:  # no sign change: fall back to the end with the smaller residual
        vi = lo if abs(residual(lo)[0]) < abs(residual(hi)[0]) else hi
    _, vh, k_ge = residual(vi)
    ct, T = thrust(vi)
    lam_i = vi / vt
    lam = lam_c + lam_i
    p_ind = p.kappa * T * vi if T * vi > 0 else T * vi
    p_prof = p.sigma * p.cd0 / 8.0 * rho * A * vt ** 3 * (1.0 + p.k_mu * mu * mu)
    p_ax = T * vc
    vrs = False
    if vh > 1e-3 and T > 0:
        x = vc / vh
        vrs = VRS_BAND[0] <= x <= VRS_BAND[1] and v_inplane / vh < VRS_VX_MAX
    return RotorState(ct, T, lam, lam_i, vi, vh, mu, k_ge, vrs, p_ind + p_prof + p_ax, p_ind, p_prof, p_ax)


def collective_for_thrust(p: HeliParams, T: float, v_tpp: float, v_inplane: float, rho: float, omega: float,
                          z_hub: float = math.inf) -> float:
    """theta_75 (rad) that gives thrust T (used for trim and as SAS feed-forward)."""
    f = lambda th: main_rotor(p, th, v_tpp, v_inplane, rho, omega, z_hub).thrust - T  # noqa: E731
    return brentq(f, math.radians(-5.0), math.radians(25.0), xtol=1e-7)


def tail_rotor_max_thrust(p: HeliParams, rho: float, omega: float) -> float:
    from .isa import RHO0
    return p.tr_tmax * (rho / RHO0) * (omega / p.omega100) ** 2


def tail_rotor_power(p: HeliParams, thrust: float, rho: float, omega: float) -> float:
    A = math.pi * p.tr_R ** 2
    vt = p.tr_vt * omega / p.omega100
    # ideal ducted fan (Leishman): P = T^1.5 / sqrt(4 sigma_d rho A); sigma_d = 1 -> 1/sqrt(2) of an open rotor
    p_ind = p.kappa * abs(thrust) ** 1.5 / math.sqrt(4.0 * p.tr_sigma_d * rho * A) if rho > 0 else 0.0
    p_prof = p.tr_sigma * p.cd0 / 8.0 * rho * A * vt ** 3
    return p_ind + p_prof


def fin_side_force(p: HeliParams, rho: float, u: float, v: float) -> float:
    """Side force of the vertical fin (N, + = to the left, the same sense as the anti-torque thrust).

    u, v: forward and lateral air-relative velocity of the aircraft in body axes. Thin-surface lift with an
    effective incidence (camber) alpha0 and the sideslip: CL = a (alpha0 - v/u), limited to +-CL_max; zero in
    hover and rearward flight. The fin yaw damping (r x omega) is left in the lumped yaw damping parameter.
    """
    if u <= 1.0:
        return 0.0
    q = 0.5 * rho * (u * u + v * v)
    cl = float(np.clip(p.fin_a * (p.fin_alpha0 - math.atan2(v, u)), -p.fin_clmax, p.fin_clmax))
    return q * p.fin_S * cl


# ------------------------------------------------------------------------------------------------
# Steady-state analysis (trim): hover, level flight, power curve
# ------------------------------------------------------------------------------------------------
@dataclass
class TrimPoint:
    V: float  # true airspeed, m/s
    thrust: float
    alpha_deg: float  # forward disk tilt
    p_main: float  # W
    p_tail: float
    p_acc: float
    p_engines: float  # W, required from the engines (after transmission losses)
    k_ge: float
    v_i: float
    p_parasite: float
    p_induced: float
    p_profile: float
    t_tail: float = 0.0  # N, Fenestron thrust
    f_fin: float = 0.0  # N, fin side force


def level_flight(p: HeliParams, V: float, rho: float, mass: float | None = None, z_hub: float = math.inf,
                 f_drag: float | None = None) -> TrimPoint:
    """Trim in steady level flight (V = 0: hover) at 100 % NR."""
    m = mass if mass is not None else p.mass
    f = p.f_drag if f_drag is None else f_drag
    om = p.omega100
    vt = om * p.R
    W = m * G0
    D = 0.5 * rho * V * V * f
    vh0 = math.sqrt(W / (2 * rho * p.area))
    dl = p.download / (1.0 + (V / vh0) ** 2)  # fuselage download fades with speed (ESTIMADO)
    Wv = W * (1.0 + dl)
    T = math.hypot(Wv, D)
    alpha = math.atan2(D, Wv)
    v_tpp = V * math.sin(alpha)
    v_in = V * math.cos(alpha)
    th = collective_for_thrust(p, T, v_tpp, v_in, rho, om, z_hub)
    rs = main_rotor(p, th, v_tpp, v_in, rho, om, z_hub)
    q_main = rs.power / om
    f_fin = fin_side_force(p, rho, V, 0.0)
    t_tr = q_main / p.tr_arm - f_fin  # the fin, at the same arm, unloads the Fenestron in forward flight
    p_tr = tail_rotor_power(p, t_tr, rho, om)
    p_acc = p.p_acc_kw * 1e3
    p_eng = (rs.power + p_tr + p_acc) / p.eta_tr
    return TrimPoint(V, T, math.degrees(alpha), rs.power, p_tr, p_acc, p_eng, rs.k_ge, rs.v_i, rs.p_axial,
                     rs.p_induced, rs.p_profile, t_tr, f_fin)


def calibrate_drag_area(p: HeliParams, v_kt: float = 136.0, p_kw: float | None = None, rho: float = 1.225) -> float:
    """f such that level flight at the published fast cruise speed needs the AEO max-continuous power.

    Fast cruise 136 kt (Airbus H135 brochure, Feb. 2022; no conditions stated) is assumed to be flown at sea level
    ISA, MTOW, AEO MCP
    (2 x 69 % torque). The result is CALIBRADO and documented as such.
    """
    target = (p_kw if p_kw is not None else p.gearbox_limit_kw("MCP")) * 1e3
    V = v_kt * 0.514444
    return brentq(lambda f: level_flight(p, V, rho, f_drag=f).p_engines - target, 0.05, 5.0, xtol=1e-6)


def power_curve(p: HeliParams, rho: float, mass: float | None = None, v_max_kt: float = 150.0, n: int = 61
                ) -> List[TrimPoint]:
    return [level_flight(p, v * 0.514444, rho, mass) for v in np.linspace(0.0, v_max_kt, n)]


def best_speeds(p: HeliParams, rho: float, mass: float | None = None) -> Dict[str, float]:
    """Speed for max endurance (min power) and max range (min power/speed), in kt, zero wind."""
    pe = lambda v: level_flight(p, v, rho, mass).p_engines  # noqa: E731
    be = minimize_scalar(pe, bounds=(10.0, 75.0), method="bounded").x
    br = minimize_scalar(lambda v: pe(v) / v, bounds=(20.0, 80.0), method="bounded").x
    return {"v_be_kt": be / 0.514444, "p_be_kw": pe(be) / 1e3, "v_br_kt": br / 0.514444,
            "p_br_kw": pe(br) / 1e3}
