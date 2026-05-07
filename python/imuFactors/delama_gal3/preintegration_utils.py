from __future__ import annotations

import torch

from .utils import *
from .lie_group_utils import SO3, SE3_2, G3


def f_Gamma(g, dt):
    """Compute Gamma preintegration"""
    Gamma = torch.eye(5, device=DEVICE)
    Gamma[:3, 3] = g * dt
    Gamma[:3, 4] = 0.5 * g * (dt**2)
    return Gamma


def f_flux(T, dt):
    """Compute Phi (the flux) preintegration"""
    Phi = T.clone()
    Phi[:, :3, 4] += Phi[:, :3, 3] * dt
    return Phi


def uf_flux(T, dt):
    """Compute unbatch Phi (the flux) preintegration"""
    return f_flux(T.unsqueeze(0), dt).squeeze(0)


def Gamma_G3(g, dt):
    """Compute Gamma preintegration for G3"""
    Gamma = torch.eye(5, device=DEVICE)
    Gamma[:3, 3] = g * dt
    Gamma[:3, 4] = -0.5 * g * (dt**2)
    Gamma[3, 4] = -dt
    return Gamma


def inv_Gamma_G3(g, dt):
    """Compute inverse Gamma preintegration for G3"""
    Gamma = torch.eye(5, device=DEVICE)
    Gamma[:3, 3] = -g * dt
    Gamma[:3, 4] = -0.5 * g * (dt**2)
    Gamma[3, 4] = dt
    return Gamma


def boxminus(T1, T2):
    """Compute the boxminus operator for SE3_2"""
    xi = T1.new_zeros(T1.shape[0], 9)
    xi[:, :3] = SO3.log(bmtm(T1[:, :3, :3], T2[:, :3, :3]))
    xi[:, 3:6] = T2[:, :3, 3] - T1[:, :3, 3]
    xi[:, 6:9] = T2[:, :3, 4] - T1[:, :3, 4]
    return xi


def boxplus(T, xi):
    """Compute the boxmplus operator for SE3_2"""
    T1 = torch.zeros_like(T)
    T1[:, :3, :3] = T[:, :3, :3].bmm(SO3.exp(xi[:, :3]))
    T1[:, :3, 3] = T[:, :3, 3] + xi[:, 3:6]
    T1[:, :3, 4] = T[:, :3, 4] + xi[:, 6:9]
    return T1


def ACI2_XI3(omega, a, dt):
    """Compute the ACI2 preintegration term XI3"""
    w = omega.norm(dim=1)
    k = omega / w.unsqueeze(1)
    kx = SO3.wedge(k)
    ax = SO3.wedge(a)
    dt2 = dt**2
    w2 = w * w
    wdt = w * dt
    sph = torch.sin(wdt)
    cph = torch.cos(wdt)

    mask = w < SO3.TOL
    XI3 = omega.new_empty(omega.shape[0], 3, 3)
    XI3[mask] = (
        0.5
        * dt2
        * (
            ax[mask]
            + sph[mask].unsqueeze(1).unsqueeze(2)
            * (
                -ax[mask].bmm(kx[mask])
                + kx[mask].bmm(ax[mask])
                + bouter(k[mask], a[mask]).bmm(kx[mask]).bmm(kx[mask])
            )
            + (1 - cph[mask]).unsqueeze(1).unsqueeze(2)
            * (
                ax[mask].bmm(kx[mask]).bmm(kx[mask])
                + kx[mask].bmm(kx[mask]).bmm(ax[mask])
                + bouter(k[mask], a[mask]).bmm(kx[mask])
            )
        )
    )

    w = w[~mask]
    k = k[~mask]
    a = a[~mask]
    kx = kx[~mask]
    ax = ax[~mask]
    w2 = w2[~mask]
    wdt = wdt[~mask]
    sph = sph[~mask]
    cph = cph[~mask]

    m1 = 0.5 * dt2
    m2 = (sph - wdt) / w2
    m3 = (sph - wdt * cph) / w2
    m4 = 0.5 * dt2 - (1 - cph) / w2
    m5 = 0.5 * dt2 + (1 - cph - wdt * sph) / w2
    m6 = -(3 * sph - 2 * wdt - wdt * cph) / w2

    m2 = m2.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(ax)
    m3 = m3.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(ax)
    m4 = m4.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(ax)
    m5 = m5.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(ax)
    m6 = m6.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(ax)

    v1 = ax
    v2 = ax.bmm(kx)
    v3 = kx.bmm(ax)
    v4 = ax.bmm(kx).bmm(kx)
    v5 = kx.bmm(kx).bmm(ax)
    v6 = bouter(k, a).bmm(kx)
    v7 = bouter(k, a).bmm(kx).bmm(kx)

    XI3[~mask] = m1 * v1 + m2 * v2 + m3 * v3 + m4 * v4 + m5 * v5 + m5 * v6 + m6 * v7

    return XI3


def ACI2_XI4(omega, a, dt):
    """Compute the ACI2 preintegration term XI3"""
    w = omega.norm(dim=1)
    k = omega / w.unsqueeze(1)
    kx = SO3.wedge(k)
    ax = SO3.wedge(a)
    dt2 = dt**2
    dt3 = dt**3
    w2 = w * w
    w3 = w2 * w
    wdt = w * dt
    sph = wdt.sin()
    cph = wdt.cos()

    mask = w < SO3.TOL
    XI4 = omega.new_empty(omega.shape[0], 3, 3)
    XI4[mask] = (
        dt3
        / 6
        * (
            ax[mask]
            + sph[mask].unsqueeze(1).unsqueeze(2)
            * (
                -ax[mask].bmm(kx[mask])
                + kx[mask].bmm(ax[mask])
                + bouter(k[mask], a[mask]).bmm(kx[mask]).bmm(kx[mask])
            )
            + (1 - cph[mask]).unsqueeze(1).unsqueeze(2)
            * (
                ax[mask].bmm(kx[mask]).bmm(kx[mask])
                + kx[mask].bmm(kx[mask]).bmm(ax[mask])
                + bouter(k[mask], a[mask]).bmm(kx[mask])
            )
        )
    )

    w = w[~mask]
    k = k[~mask]
    a = a[~mask]
    kx = kx[~mask]
    ax = ax[~mask]
    w2 = w2[~mask]
    w3 = w3[~mask]
    wdt = wdt[~mask]
    wdt2 = wdt * wdt
    wdt3 = wdt2 * wdt
    sph = sph[~mask]
    cph = cph[~mask]

    m1 = dt3 / 6
    m2 = (2 * (1 - cph) - wdt2) / (2 * w3)
    m3 = (2 * (1 - cph) - wdt * sph) / w3
    m4 = (sph - wdt) / w3 + dt3 / 6
    m5 = (wdt - 2 * sph + wdt3 / 6 + wdt * cph) / w3
    m6 = (4 * cph - 4 + wdt2 + wdt * sph) / w3

    m2 = m2.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(ax)
    m3 = m3.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(ax)
    m4 = m4.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(ax)
    m5 = m5.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(ax)
    m6 = m6.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(ax)

    v1 = ax
    v2 = ax.bmm(kx)
    v3 = kx.bmm(ax)
    v4 = ax.bmm(kx).bmm(kx)
    v5 = kx.bmm(kx).bmm(ax)
    v6 = bouter(k, a).bmm(kx)
    v7 = bouter(k, a).bmm(kx).bmm(kx)

    XI4[~mask] = m1 * v1 + m2 * v2 + m3 * v3 + m4 * v4 + m5 * v5 + m5 * v6 + m6 * v7

    return XI4
