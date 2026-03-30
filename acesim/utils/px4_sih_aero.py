"""Python port of the PX4 SIH aerodynamic segment model."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation


def _clamp(value: float, lower: float, upper: float) -> float:
    return float(min(max(value, lower), upper))


@dataclass
class AeroSeg:
    """Semi-empirical aerodynamic flat-plate segment."""

    span: float
    mac: float
    alpha_0_deg: float
    p_b: np.ndarray
    dihedral_deg: float = 0.0
    aspect_ratio: float = -1.0
    flap_chord: float = 0.0
    prop_radius: float = -1.0
    cl_alpha: float = 2.0 * np.pi
    alpha_max_deg: float = 0.0
    alpha_min_deg: float = 0.0

    ETA_POLY: tuple[float, float, float] = (0.0535, -0.2688, 0.5817)
    P0: float = 101325.0
    GAS_R: float = 287.04
    T0_K: float = 288.15
    TEMP_GRADIENT: float = -6.5e-3
    KV: float = np.pi
    CD0: float = 0.04
    CD90: float = 1.98
    ALPHA_BLEND: float = np.pi / 18.0
    K0: float = 0.87

    _alpha_0: float = field(init=False)
    _p_b: np.ndarray = field(init=False)
    _c_bs: Rotation = field(init=False)
    _ar: float = field(init=False)
    _kp: float = field(init=False)
    _kn: float = field(init=False)
    _ale: float = field(init=False)
    _ate: float = field(init=False)
    _afle: float = field(init=False)
    _afte: float = field(init=False)
    _cf: float = field(init=False)
    _prop_radius: float = field(init=False)
    _k_d: float = field(init=False)
    _alpha_max: float = field(init=False)
    _alpha_min: float = field(init=False)
    _alpha_eff: float = field(default=0.0, init=False)
    _alpha_eff_old: float = field(default=0.0, init=False)
    _rho: float = field(default=1.225, init=False)
    _alpha: float = field(default=0.0, init=False)
    _cl: float = field(default=0.0, init=False)
    _cd: float = field(default=0.0, init=False)
    _cm: float = field(default=0.0, init=False)
    _fa: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float), init=False)
    _ma: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float), init=False)
    _v_s: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float), init=False)

    def __post_init__(self) -> None:
        ar_tab = np.array([0.1666, 0.333, 0.4, 0.5, 1.0, 1.25, 2.0, 3.0, 4.0, 6.0], dtype=float)
        ale_tab = np.array([3.00, 3.64, 4.48, 7.18, 10.20, 13.38, 14.84, 14.49, 9.95, 12.93], dtype=float)
        ate_tab = np.array([5.90, 15.51, 32.57, 39.44, 48.22, 59.29, 21.55, 7.74, 7.05, 5.26], dtype=float)
        afle_tab = np.array([59.00, 58.60, 58.20, 50.00, 41.53, 26.70, 23.44, 21.00, 18.63, 14.28], dtype=float)
        afte_tab = np.array([59.00, 58.60, 58.20, 51.85, 41.46, 28.09, 39.40, 35.86, 26.76, 19.76], dtype=float)
        afs_tab = np.array([49.00, 54.00, 56.00, 48.00, 40.00, 29.00, 27.00, 25.00, 24.00, 22.00], dtype=float)

        self._alpha_0 = np.deg2rad(self.alpha_0_deg)
        self._p_b = np.asarray(self.p_b, dtype=float)
        self._c_bs = Rotation.from_euler("x", np.deg2rad(self.dihedral_deg))
        self._ar = self.span / self.mac if self.aspect_ratio <= 0.0 else float(self.aspect_ratio)
        self._kp = self.cl_alpha / (1.0 + 2.0 * (self._ar + 4.0) / (self._ar * (self._ar + 2.0)))
        self._kn = 0.41 * (1.0 - np.exp(-17.0 / self._ar))
        self._ale = np.interp(self._ar, ar_tab, ale_tab)
        self._ate = np.interp(self._ar, ar_tab, ate_tab)
        self._afle = np.interp(self._ar, ar_tab, afle_tab)
        self._afte = np.interp(self._ar, ar_tab, afte_tab)
        afs_rad = np.deg2rad(np.interp(self._ar, ar_tab, afs_tab))
        self._alpha_max = afs_rad if abs(self.alpha_max_deg) < 1.0e-3 else np.deg2rad(self.alpha_max_deg)
        self._alpha_min = -afs_rad if abs(self.alpha_min_deg) < 1.0e-3 else np.deg2rad(self.alpha_min_deg)
        self._cf = _clamp(self.flap_chord, 0.0, self.mac)
        self._prop_radius = float(self.prop_radius)
        self._k_d = 1.0 / (np.pi * self.K0 * self._ar)

    @property
    def rho(self) -> float:
        return self._rho

    @property
    def alpha(self) -> float:
        return self._alpha

    def get_fa(self) -> np.ndarray:
        return self._fa.copy()

    def get_ma(self) -> np.ndarray:
        return self._ma.copy()

    def update_aero(
        self,
        v_b: np.ndarray,
        w_b: np.ndarray,
        alt: float = 0.0,
        control_deflection: float = 0.0,
        thrust: float = 0.0,
    ) -> None:
        pressure = self.P0 * (1.0 - 0.0065 * alt / self.T0_K) ** 5.2561
        temperature = self.T0_K + self.TEMP_GRADIENT * alt
        self._rho = pressure / self.GAS_R / temperature

        rot = self._c_bs
        self._v_s = rot.inv().apply(np.asarray(v_b, dtype=float) + np.cross(np.asarray(w_b, dtype=float), self._p_b))
        if self._prop_radius > 1e-4 and thrust > 0.0:
            self._v_s[0] += np.sqrt(
                max(0.0, 2.0 * thrust / (self._rho * np.pi * self._prop_radius * self._prop_radius))
            )

        vxz2 = self._v_s[0] * self._v_s[0] + self._v_s[2] * self._v_s[2]
        if vxz2 < 0.01:
            self._fa = np.zeros(3, dtype=float)
            self._ma = np.zeros(3, dtype=float)
            self._alpha = 0.0
            return

        self._alpha = np.arctan2(self._v_s[2], self._v_s[0]) - self._alpha_0
        self._alpha = (self._alpha + np.pi) % (2.0 * np.pi) - np.pi
        cl, cd, cm = self._aoa_coeff(self._alpha, np.sqrt(vxz2), control_deflection)

        force_s = (0.5 * self._rho * vxz2 * self.span * self.mac) * np.array(
            [
                cl * np.sin(self._alpha) - cd * np.cos(self._alpha),
                0.0,
                -cl * np.cos(self._alpha) - cd * np.sin(self._alpha),
            ],
            dtype=float,
        )
        moment_s = (0.5 * self._rho * vxz2 * self.span * self.mac * self.mac) * np.array([0.0, cm, 0.0], dtype=float)
        self._fa = rot.apply(force_s)
        self._ma = rot.apply(moment_s) + np.cross(self._p_b, self._fa)

    def _aoa_coeff(self, alpha: float, vxz: float, control_deflection: float) -> tuple[float, float, float]:
        self._alpha_eff_old = self._alpha_eff
        4.5 * self.mac / vxz if vxz > 0.01 else 0.0
        0.5 * self.mac / vxz if vxz > 0.01 else 0.0

        if self._cf / self.mac < 0.999:
            def_a = min(abs(control_deflection), np.deg2rad(70.0))
            eta_f = def_a * def_a * self.ETA_POLY[0] + def_a * self.ETA_POLY[1] + self.ETA_POLY[2]
            theta_f = np.arccos(2.0 * self._cf / self.mac - 1.0)
            tau_f = 1.0 - (theta_f - np.sin(theta_f)) / np.pi
            delta_cl = self._kp * tau_f * eta_f * control_deflection
            d_cl_max = (1.0 - self._cf / self.mac) * delta_cl
            alf0eff = self._solve_alpha_eff(self._kp, self.KV, delta_cl, self._alpha_0)
            self._alpha_eff = alpha - alf0eff
            fte = 1.0
            fle = 1.0
            cl_max = self._f_cl(self._alpha_max - self._alpha_0, fte, fle) + d_cl_max
            alpha_eff_max = alf0eff - self._solve_alpha_eff(
                self._kp,
                self.KV * fle * fle,
                cl_max / (0.25 * (1.0 + np.sqrt(fte)) * (1.0 + np.sqrt(fte))),
                self._alpha_max - self._alpha_0,
            )
            cl_min = self._f_cl(self._alpha_min - self._alpha_0, fte, fle) + d_cl_max
            alpha_eff_min = alf0eff - self._solve_alpha_eff(
                self._kp,
                self.KV * fle * fle,
                cl_min / (0.25 * (1.0 + np.sqrt(fte)) * (1.0 + np.sqrt(fte))),
                self._alpha_min - self._alpha_0,
            )
        else:
            self._alpha_eff = alpha + control_deflection
            fte = 1.0
            fle = 1.0
            alpha_eff_max = self._alpha_max
            alpha_eff_min = self._alpha_min

        cl_high, cd_high, cm_high = self._high_aoa_coeff(self._alpha_eff, control_deflection)
        cl_low = self._f_cl(self._alpha_eff, fte, fle)
        cd_low = self.CD0 + abs(cl_high * np.tan(self._alpha_eff))
        cm_low = -self._f_cm(self._alpha_eff, fte, fle)

        if self._alpha_eff > 0.0:
            blend = 0.5 * (1.0 - np.tanh(4.0 * (self._alpha_eff - alpha_eff_max) / self.ALPHA_BLEND))
        else:
            blend = 0.5 * (1.0 - np.tanh(-4.0 * (self._alpha_eff - alpha_eff_min) / self.ALPHA_BLEND))

        cl = cl_low * blend + cl_high * (1.0 - blend)
        cd = cd_low * blend + cd_high * (1.0 - blend)
        cm = cm_low * blend + cm_high * (1.0 - blend)
        self._cl = cl
        self._cd = cd
        self._cm = cm
        return cl, cd, cm

    def _high_aoa_coeff(self, alpha: float, control_deflection: float) -> tuple[float, float, float]:
        mac_eff = np.sqrt(
            (self.mac - self._cf) * (self.mac - self._cf)
            + self._cf * self._cf
            + 2.0 * (self.mac - self._cf) * self._cf * np.cos(abs(control_deflection))
        )
        alpha_eff = alpha + np.arcsin(self._cf / max(mac_eff, 1e-6) * np.sin(control_deflection))
        cd90_eff = self.CD90 + 0.21 * control_deflection - 0.0426 * control_deflection * control_deflection
        cn = cd90_eff * np.sin(alpha_eff) * (1.0 / (0.56 + 0.44 * np.sin(abs(alpha_eff))) - self._kn)
        ct = 0.5 * self.CD0 * np.cos(alpha_eff)
        cl = cn * np.cos(alpha_eff) - ct * np.sin(alpha_eff)
        cd = cn * np.sin(alpha_eff) + ct * np.cos(alpha_eff)
        cm = -cn * (0.25 - 7.0 / 40.0 * (1.0 - 2.0 / np.pi * abs(alpha_eff)))
        return cl, cd, cm

    def _solve_alpha_eff(self, kp: float, kv: float, delta_cl: float, alpha_0: float) -> float:
        alpha = alpha_0
        for _ in range(3):
            s = np.sin(alpha)
            c = np.cos(alpha)
            abs_s = abs(s)
            sign_s = 1.0 if s >= 0.0 else -1.0
            numerator = -kp * s * c * c - kv * abs_s * s * c - delta_cl
            denominator = (
                kv * abs_s * s * s
                - kv * abs_s * c * c
                - kp * c * c * c
                + 2.0 * kp * c * s * s
                - kv * sign_s * c * c * s
            )
            if abs(denominator) < 1e-6:
                break
            alpha = alpha - numerator / denominator
        return alpha

    def _f_cl(self, alpha: float, fte: float, fle: float) -> float:
        return (
            0.25
            * (1.0 + np.sqrt(fte))
            * (1.0 + np.sqrt(fte))
            * (
                self._kp * np.sin(alpha) * np.cos(alpha) * np.cos(alpha)
                + fle * fle * self.KV * abs(np.sin(alpha)) * np.sin(alpha) * np.cos(alpha)
            )
        )

    def _f_cm(self, alpha: float, fte: float, fle: float) -> float:
        return -0.25 * (1.0 + np.sqrt(fte)) * (1.0 + np.sqrt(fte)) * 0.0625 * (
            -1.0 + 6.0 * np.sqrt(fte) - 5.0 * fte
        ) * self._kp * np.sin(alpha) * np.cos(alpha) + 0.17 * fle * fle * self.KV * abs(np.sin(alpha)) * np.sin(alpha)
