from __future__ import annotations

import numpy as np
import torch

from .utils import *

torch.set_default_dtype(torch.float64)


########################################################
# Lie Group Utils
#
# phi -> so3 vector (rotation)
# mu -> R^3 vector (velocity)
# rho -> R^3 vector (translation)
# xi = [phi, mu, rho] -> se2(3) vector (extended pose)
# psi = [xi, alpha] -> g(3) vector (extended pose + scale)
#
########################################################


class G3:
    """G(3), the Inhomogeneous Galilean group"""

    Id = torch.eye(5, device=DEVICE)
    Id9 = torch.eye(9, device=DEVICE)

    @classmethod
    def exp(cls, psi):
        """
        Batch exponential
        """
        xi = psi[:, :9]
        alpha = psi[:, 9:10]
        T = xi.new_zeros(xi.shape[0], 5, 5)
        T[:, 3, 3] = 1
        T[:, 4, 4] = 1
        T[:, :3, :3] = SO3.exp(xi[:, :3])
        tmp = xi.new_zeros(xi.shape[0], 3, 2)
        tmp[:, :, 0] = xi[:, 3:6].reshape(-1, 3)
        tmp[:, :, 1] = xi[:, 6:9].reshape(-1, 3)
        T[:, :3, 3:] = SO3.left_jacobian(xi[:, :3]).bmm(tmp)
        T[:, :3, 4:] += alpha.unsqueeze(1) * SO3.J2(xi[:, :3]).bmm(tmp[:, :, :1])
        T[:, 3:4, 4] = alpha
        return T

    @classmethod
    def uexp(cls, psi):
        """
        Non-batch exponential
        """
        return cls.exp(psi.unsqueeze(0)).squeeze()

    @classmethod
    def log(cls, T):
        """
        Batch logarithm
        """
        c = T[:, 3:4, 4]
        phi = SO3.log(T[:, :3, :3])
        J1_inv = SO3.inv_left_jacobian(phi)
        mu = J1_inv.bmm(T[:, :3, 3:4])
        psi = phi.new_zeros(phi.shape[0], 10)
        psi[:, :3] = phi
        psi[:, 3:6] = mu.squeeze(2)
        psi[:, 6:9] = J1_inv.bmm(
            T[:, :3, 4:5] - c.unsqueeze(1) * SO3.J2(phi).bmm(mu)
        ).squeeze(2)
        psi[:, 9:10] = c
        return psi

    @classmethod
    def ulog(cls, T):
        """
        Non-batch logarithm
        """
        return cls.log(T.unsqueeze(0)).squeeze()

    @classmethod
    def Ad(cls, T):
        """
        Batch adjoint
        """
        Adjoint = T.new_zeros(T.shape[0], 10, 10)
        Adjoint[:, :3, :3] = T[:, :3, :3]
        Adjoint[:, 3:6, 3:6] = T[:, :3, :3]
        Adjoint[:, 6:9, 6:9] = T[:, :3, :3]
        Adjoint[:, 9, 9] = 1
        Adjoint[:, 3:6, :3] = SO3.wedge(T[:, :3, 3]).bmm(T[:, :3, :3])
        Adjoint[:, 6:9, :3] = SO3.wedge(T[:, :3, 4] - T[:, 3:4, 4] * T[:, :3, 3]).bmm(
            T[:, :3, :3]
        )
        Adjoint[:, 6:9, 3:6] = -T[:, 3:4, 4].unsqueeze(1) * T[:, :3, :3]
        Adjoint[:, 6:9, 9] = T[:, :3, 3]
        return Adjoint

    @classmethod
    def uAd(cls, T):
        """
        Non-batch adjoint
        """
        return cls.Ad(T.unsqueeze(0)).squeeze()

    @classmethod
    def wedge(cls, psi):
        """
        Batch wedge operator
        """
        Psi = psi.new_zeros(psi.shape[0], 5, 5)
        Psi[:, :3, :3] = SO3.wedge(psi[:, :3])
        Psi[:, :3, 3] = psi[:, 3:6]
        Psi[:, :3, 4] = psi[:, 6:9]
        Psi[:, 3, 4] = psi[:, 9]
        return Psi

    @classmethod
    def uwedge(cls, psi):
        """
        Non-batch wedge operator
        """
        return cls.wedge(psi.unsqueeze(0)).squeeze()

    @classmethod
    def vee(cls, Psi):
        """
        Batch vee operator
        """
        return torch.cat(
            (SO3.vee(Psi[:, :3, :3]), Psi[:, :3, 3], Psi[:, :3, 4], Psi[:, 3:4, 4]), 1
        )

    @classmethod
    def uvee(cls, Psi):
        """
        Non-batch vee operator
        """
        return cls.vee(Psi.unsqueeze(0)).squeeze()

    @classmethod
    def inv(cls, T):
        """
        Batch inverse
        """
        T_inv = torch.zeros_like(T)
        T_inv[:, 3, 3] = 1
        T_inv[:, 4, 4] = 1
        T_inv[:, :3, :3] = T[:, :3, :3].transpose(1, 2)
        T_inv[:, :3, 3] = -bmtv(T[:, :3, :3], T[:, :3, 3])
        T_inv[:, :3, 4] = -bmtv(T[:, :3, :3], T[:, :3, 4] - T[:, 3:4, 4] * T[:, :3, 3])
        T_inv[:, 3, 4] = -T[:, 3, 4]
        return T_inv

    @classmethod
    def uinv(cls, T):
        """
        Non-batch inverse
        """
        return cls.inv(T.unsqueeze(0)).squeeze()

    @classmethod
    def inv_left_jacobian(cls, psi):
        """
        Batch inverse left-Jacobian
        """
        phi = psi[:, :3]  # rotation part
        mu = psi[:, 3:6]  # velocity part
        # rho = psi[:, 6:9]  # translation part
        alpha = psi[:, 9:10]  # scale part
        xi = psi[:, :9]  # extended pose part

        J = phi.new_zeros(phi.shape[0], 10, 10)

        j1_inv = SO3.inv_left_jacobian(phi)
        j2 = SO3.J2(phi)
        u1 = SO3.U1(phi)
        Q1_v, Q1_p = SE3_2.Q1(xi)
        Q2_v = SE3_2.Q2(xi)
        JiQJi_v = j1_inv.bmm(Q1_v).bmm(j1_inv)

        J[:, :3, :3] = j1_inv
        J[:, 3:6, 3:6] = j1_inv
        J[:, 6:9, 6:9] = j1_inv
        J[:, 9, 9] = 1
        J[:, 3:6, :3] = -JiQJi_v
        J[:, 6:9, :3] = -j1_inv.bmm(
            Q1_p.bmm(j1_inv) - alpha.unsqueeze(1) * (Q2_v.bmm(j1_inv) - u1.bmm(JiQJi_v))
        )
        J[:, 6:9, 3:6] = alpha.unsqueeze(1) * j1_inv.bmm(u1).bmm(j1_inv)
        J[:, 6:9, 9:10] = bmv(-j1_inv.bmm(j2), mu).unsqueeze(2)

        return J

    @classmethod
    def uinv_left_jacobian(cls, psi):
        """
        Non-batch inverse left-Jacobian
        """
        return cls.inv_left_jacobian(psi.unsqueeze(0)).squeeze()

    @classmethod
    def left_jacobian(cls, psi):
        """
        Batch left-Jacobian
        """
        phi = psi[:, :3]  # rotation part
        xi = psi[:, :9]  # extended pose part
        mu = psi[:, 3:6]  # velocity part
        # rho = psi[:, 6:9]  # translation part
        alpha = psi[:, 9:10]  # scale part

        J = phi.new_zeros(phi.shape[0], 10, 10)

        j1 = SO3.left_jacobian(phi)
        j2 = SO3.J2(phi)
        u1 = SO3.U1(phi)
        Q1_v, Q1_p = SE3_2.Q1(xi)
        Q2_v = SE3_2.Q2(xi)

        J[:, :3, :3] = j1
        J[:, 3:6, 3:6] = j1
        J[:, 6:9, 6:9] = j1
        J[:, 9, 9] = 1
        J[:, 3:6, :3] = Q1_v
        J[:, 6:9, :3] = Q1_p - alpha.unsqueeze(1) * Q2_v
        J[:, 6:9, 3:6] = -alpha.unsqueeze(1) * u1
        J[:, 6:9, 9:10] = bmv(j2, mu).unsqueeze(2)

        return J

    @classmethod
    def uleft_jacobian(cls, psi):
        """
        Non-batch left-Jacobian
        """
        return cls.left_jacobian(psi.unsqueeze(0)).squeeze()


class SE3_2:
    """SE_2(3), the group of extended poses"""

    Id = torch.eye(5, device=DEVICE)
    Id9 = torch.eye(9, device=DEVICE)

    @classmethod
    def exp(cls, xi):
        """
        Batch exponential
        """
        T = xi.new_zeros(xi.shape[0], 5, 5)
        T[:, 3, 3] = 1
        T[:, 4, 4] = 1
        T[:, :3, :3] = SO3.exp(xi[:, :3])
        tmp = xi.new_zeros(xi.shape[0], 3, 2)
        tmp[:, :, 0] = xi[:, 3:6].reshape(-1, 3)
        tmp[:, :, 1] = xi[:, 6:9].reshape(-1, 3)
        T[:, :3, 3:] = SO3.left_jacobian(xi[:, :3]).bmm(tmp)
        return T

    @classmethod
    def uexp(cls, xi):
        """
        Non-batch exponential
        """
        return cls.exp(xi.unsqueeze(0)).squeeze()

    @classmethod
    def log(cls, T):
        """
        Batch logarithm
        """
        phi = SO3.log(T[:, :3, :3])
        Xi = SO3.inv_left_jacobian(phi).bmm(T[:, :3, 3:5])
        xi = phi.new_zeros(phi.shape[0], 9)
        xi[:, :3] = phi
        xi[:, 3:6] = Xi[:, :, 0]
        xi[:, 6:9] = Xi[:, :, 1]
        return xi

    @classmethod
    def ulog(cls, T):
        """
        Non-batch logarithm
        """
        return cls.log(T.unsqueeze(0)).squeeze()

    @classmethod
    def Ad(cls, T):
        """
        Batch adjoint
        """
        Adjoint = T.new_zeros(T.shape[0], 9, 9)
        Adjoint[:, :3, :3] = T[:, :3, :3]
        Adjoint[:, 3:6, 3:6] = T[:, :3, :3]
        Adjoint[:, 6:9, 6:9] = T[:, :3, :3]
        Adjoint[:, 3:6, :3] = SO3.wedge(T[:, :3, 3]).bmm(T[:, :3, :3])
        Adjoint[:, 6:9, :3] = SO3.wedge(T[:, :3, 4]).bmm(T[:, :3, :3])
        return Adjoint

    @classmethod
    def uAd(cls, T):
        """
        Non-batch adjoint
        """
        return cls.Ad(T.unsqueeze(0)).squeeze()

    @classmethod
    def wedge(cls, xi):
        """
        Batch wedge operator
        """
        Xi = xi.new_zeros(xi.shape[0], 5, 5)
        Xi[:, :3, :3] = SO3.wedge(xi[:, :3])
        Xi[:, :3, 3] = xi[:, 3:6]
        Xi[:, :3, 4] = xi[:, 6:9]
        return Xi

    @classmethod
    def uwedge(cls, xi):
        """
        Non-batch wedge operator
        """
        return cls.wedge(xi.unsqueeze(0)).squeeze()

    @classmethod
    def vee(cls, Xi):
        """
        Batch vee operator
        """
        return torch.cat((SO3.vee(Xi[:, :3, :3]), Xi[:, :3, 3], Xi[:, :3, 4]), 1)

    @classmethod
    def uvee(cls, Xi):
        """
        Non-batch vee operator
        """
        return cls.vee(Xi.unsqueeze(0)).squeeze()

    @classmethod
    def inv(cls, T):
        """
        Batch inverse
        """
        T_inv = torch.zeros_like(T)
        T_inv[:, 3, 3] = 1
        T_inv[:, 4, 4] = 1
        T_inv[:, :3, :3] = T[:, :3, :3].transpose(1, 2)
        T_inv[:, :3, 3:5] = -bmtm(T[:, :3, :3], T[:, :3, 3:5])
        return T_inv

    @classmethod
    def uinv(cls, T):
        """
        Non-batch inverse
        """
        return cls.inv(T.unsqueeze(0)).squeeze()

    @classmethod
    def inv_left_jacobian(cls, xi):
        """
        Batch inverse left-Jacobian
        """
        phi = xi[:, :3]  # rotation part
        # mu = xi[:, 3:6]  # velocity part
        # rho = xi[:, 6:9]  # translation part

        J = phi.new_zeros(phi.shape[0], 9, 9)

        j1_inv = SO3.inv_left_jacobian(phi)
        Q_v, Q_p = SE3_2.Q1(xi)

        JiQJi_v = j1_inv.bmm(Q_v).bmm(j1_inv)
        JiQJi_p = j1_inv.bmm(Q_p).bmm(j1_inv)

        J[:, :3, :3] = j1_inv
        J[:, 3:6, 3:6] = j1_inv
        J[:, 6:9, 6:9] = j1_inv
        J[:, 3:6, :3] = -JiQJi_v
        J[:, 6:9, :3] = -JiQJi_p

        return J

    @classmethod
    def uinv_left_jacobian(cls, xi):
        """
        Non-batch inverse left-Jacobian
        """
        return cls.inv_left_jacobian(xi.unsqueeze(0)).squeeze()

    @classmethod
    def left_jacobian(cls, xi):
        """
        Batch left-Jacobian
        """
        phi = xi[:, :3]  # rotation part
        # mu = xi[:, 3:6]  # velocity part
        # rho = xi[:, 6:9]  # translation part

        J = phi.new_zeros(phi.shape[0], 9, 9)

        j1 = SO3.left_jacobian(phi)
        Q_v, Q_p = SE3_2.Q1(xi)

        J[:, :3, :3] = j1
        J[:, 3:6, 3:6] = j1
        J[:, 6:9, 6:9] = j1
        J[:, 3:6, :3] = Q_v
        J[:, 6:9, :3] = Q_p

        return J

    @classmethod
    def uleft_jacobian(cls, xi):
        """
        Non-batch left-Jacobian
        """
        return cls.left_jacobian(xi.unsqueeze(0)).squeeze()

    @classmethod
    def Q1(cls, xi):
        """
        Batch Q matrices for Jacobian computation
        """
        phi = xi[:, :3]  # rotation part
        mu = xi[:, 3:6]  # velocity part
        rho = xi[:, 6:9]  # translation part

        px = SO3.wedge(phi)
        mx = SO3.wedge(mu)
        rx = SO3.wedge(rho)

        ph = phi.norm(dim=1)
        mask = ph < SO3.TOL
        Q1_v = phi.new_empty(phi.shape[0], 3, 3)
        Q1_p = phi.new_empty(phi.shape[0], 3, 3)
        # Near phi==0, use third order Taylor expansion
        Q1_v[mask] = 0.5 * mx[mask] + 1 / 6 * (
            px[mask].bmm(mx[mask]) + mx[mask].bmm(px[mask])
        )
        Q1_p[mask] = 0.5 * rx[mask] + 1 / 6 * (
            px[mask].bmm(rx[mask]) + rx[mask].bmm(px[mask])
        )

        phi = phi[~mask]
        mu = mu[~mask]
        rho = rho[~mask]

        px = px[~mask]
        mx = mx[~mask]
        rx = rx[~mask]

        ph = ph[~mask]
        ph2 = ph * ph
        ph3 = ph2 * ph
        ph4 = ph3 * ph
        ph5 = ph4 * ph

        cph = ph.cos()
        sph = ph.sin()

        m1 = 0.5
        m2 = (ph - sph) / ph3
        m3 = (0.5 * ph2 + cph - 1.0) / ph4
        m4 = (ph - 1.5 * sph + 0.5 * ph * cph) / ph5

        m2 = m2.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(px)
        m3 = m3.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(px)
        m4 = m4.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(px)

        v1 = mx
        v2 = px.bmm(mx) + mx.bmm(px) + px.bmm(mx).bmm(px)
        v3 = px.bmm(px).bmm(mx) + mx.bmm(px).bmm(px) - 3.0 * px.bmm(mx).bmm(px)
        v4 = px.bmm(mx).bmm(px).bmm(px) + px.bmm(px).bmm(mx).bmm(px)

        t1 = rx
        t2 = px.bmm(rx) + rx.bmm(px) + px.bmm(rx).bmm(px)
        t3 = px.bmm(px).bmm(rx) + rx.bmm(px).bmm(px) - 3.0 * px.bmm(rx).bmm(px)
        t4 = px.bmm(rx).bmm(px).bmm(px) + px.bmm(px).bmm(rx).bmm(px)

        Q1_v[~mask] = m1 * v1 + m2 * v2 + m3 * v3 + m4 * v4
        Q1_p[~mask] = m1 * t1 + m2 * t2 + m3 * t3 + m4 * t4

        return Q1_v, Q1_p

    @classmethod
    def Q2(cls, xi):
        """
        Batch Q matrices for Jacobian computation
        """
        phi = xi[:, :3]  # rotation part
        mu = xi[:, 3:6]  # velocity part
        # rho = xi[:, 6:9]  # translation part

        px = SO3.wedge(phi)
        mx = SO3.wedge(mu)
        # rx = SO3.wedge(rho)

        ph = phi.norm(dim=1)
        mask = ph < SO3.TOL
        Q2_v = phi.new_empty(phi.shape[0], 3, 3)
        # Near phi==0, use fourth order Taylor expansion
        Q2_v[mask] = (
            1 / 6 * mx[mask]
            + 1 / 12 * px[mask].bmm(mx[mask]).bmm(mx[mask])
            + 1 / 24 * mx[mask].bmm(px[mask]).bmm(mx[mask])
        )

        phi = phi[~mask]
        mu = mu[~mask]
        # rho = rho[~mask]

        px = px[~mask]
        mx = mx[~mask]
        # rx = rx[~mask]

        ph = ph[~mask]
        ph2 = ph * ph
        ph3 = ph2 * ph
        ph4 = ph3 * ph
        ph5 = ph4 * ph
        ph6 = ph5 * ph
        ph7 = ph6 * ph

        cph = ph.cos()
        sph = ph.sin()

        m1 = 1 / 6
        m2 = (0.5 * ph2 + cph - 1.0) / ph4
        m3 = (m1 * ph3 - ph + sph) / ph5
        m4 = -(2.0 * cph + ph * sph - 2.0) / ph4
        m5 = (m1 * ph3 + ph * cph + ph - 2.0 * sph) / ph5
        m6 = -(0.75 * ph * cph + (0.25 * ph2 - 0.75) * sph) / ph5
        m7 = (0.25 * ph * cph + 0.5 * ph - 0.75 * sph) / ph5
        m8 = ((0.25 * ph2 - 2.0) * cph - 1.25 * ph * sph + 2.0) / ph6
        m9 = (m1 * ph3 + 1.25 * ph * cph + (0.25 * ph2 - 1.25) * sph) / ph7

        m2 = m2.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(px)
        m3 = m3.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(px)
        m4 = m4.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(px)
        m5 = m5.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(px)
        m6 = m6.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(px)
        m7 = m7.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(px)
        m8 = m8.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(px)
        m9 = m9.unsqueeze(dim=1).unsqueeze(dim=2).expand_as(px)

        v1 = mx
        v2 = mx.bmm(px)
        v3 = mx.bmm(px).bmm(px)
        v4 = px.bmm(mx)
        v5 = px.bmm(px).bmm(mx)
        v6 = px.bmm(mx).bmm(px)
        v7 = px.bmm(px).bmm(mx).bmm(px)
        v8 = px.bmm(mx).bmm(px).bmm(px)
        v9 = px.bmm(px).bmm(mx).bmm(px).bmm(px)

        Q2_v[~mask] = (
            m1 * v1
            + m2 * v2
            + m3 * v3
            + m4 * v4
            + m5 * v5
            + m6 * v6
            + m7 * v7
            + m8 * v8
            + m9 * v9
        )

        return Q2_v


class SO3:
    #  tolerance criterion
    TOL = 1e-9
    Id = torch.eye(3, device=DEVICE)

    @classmethod
    def exp(cls, phi):
        angle = phi.norm(dim=1, keepdim=True)
        mask = angle[:, 0] < cls.TOL
        dim_batch = phi.shape[0]
        Id = cls.Id.expand(dim_batch, 3, 3).float()

        axis = phi[~mask] / angle[~mask]
        c = angle[~mask].cos().unsqueeze(2)
        s = angle[~mask].sin().unsqueeze(2)

        Rot = phi.new_empty(dim_batch, 3, 3)
        Rot[mask] = Id[mask] + SO3.wedge(phi[mask])
        Rot[~mask] = c * Id[~mask] + (1 - c) * bouter(axis, axis) + s * cls.wedge(axis)
        return Rot

    @classmethod
    def uexp(cls, phi):
        """
        Non-batch exponential
        """
        return cls.exp(phi.unsqueeze(0)).squeeze()

    @classmethod
    def log(cls, Rot):
        dim_batch = Rot.shape[0]
        Id = cls.Id.expand(dim_batch, 3, 3)

        cos_angle = (0.5 * btrace(Rot) - 0.5).clamp(-1.0, 1.0)
        # Clip cos(angle) to its proper domain to avoid NaNs from rounding
        # errors
        angle = cos_angle.acos()
        mask = angle < cls.TOL
        if mask.sum() == 0:
            angle = angle.unsqueeze(1).unsqueeze(1)
            return cls.vee((0.5 * angle / angle.sin()) * (Rot - Rot.transpose(1, 2)))
        elif mask.sum() == dim_batch:
            # If angle == close to zero, use first-order Taylor expansion
            return cls.vee(Rot - Id)
        phi = cls.vee(Rot - Id)
        angle = angle
        phi[~mask] = cls.vee(
            (0.5 * angle[~mask] / angle[~mask].sin()).unsqueeze(1).unsqueeze(2)
            * (Rot[~mask] - Rot[~mask].transpose(1, 2))
        )
        return phi

    @classmethod
    def ulog(cls, Rot):
        """
        Non-batch logarithm
        """
        return cls.log(Rot.unsqueeze(0)).squeeze()

    @staticmethod
    def vee(Phi):
        return torch.stack((Phi[:, 2, 1], Phi[:, 0, 2], Phi[:, 1, 0]), dim=1)

    @staticmethod
    def wedge(phi):
        dim_batch = phi.shape[0]
        zero = phi.new_zeros(dim_batch)
        return torch.stack(
            (
                zero,
                -phi[:, 2],
                phi[:, 1],
                phi[:, 2],
                zero,
                -phi[:, 0],
                -phi[:, 1],
                phi[:, 0],
                zero,
            ),
            1,
        ).view(dim_batch, 3, 3)

    @staticmethod
    def uwedge(phi):
        Phi = phi.new_zeros(3, 3)
        Phi[0, 1] = -phi[2]
        Phi[1, 0] = phi[2]
        Phi[0, 2] = phi[1]
        Phi[2, 0] = -phi[1]
        Phi[1, 2] = -phi[0]
        Phi[2, 1] = phi[0]
        return Phi

    @classmethod
    def from_rpy(cls, roll, pitch, yaw):
        return cls.rotz(yaw).bmm(cls.roty(pitch).bmm(cls.rotx(roll)))

    @classmethod
    def rotx(cls, angle_in_radians):
        c = angle_in_radians.cos()
        s = angle_in_radians.sin()
        mat = c.new_zeros((c.shape[0], 3, 3))
        mat[:, 0, 0] = 1
        mat[:, 1, 1] = c
        mat[:, 2, 2] = c
        mat[:, 1, 2] = -s
        mat[:, 2, 1] = s
        return mat

    @classmethod
    def roty(cls, angle_in_radians):
        c = angle_in_radians.cos()
        s = angle_in_radians.sin()
        mat = c.new_zeros((c.shape[0], 3, 3))
        mat[:, 1, 1] = 1
        mat[:, 0, 0] = c
        mat[:, 2, 2] = c
        mat[:, 0, 2] = s
        mat[:, 2, 0] = -s
        return mat

    @classmethod
    def rotz(cls, angle_in_radians):
        c = angle_in_radians.cos()
        s = angle_in_radians.sin()
        mat = c.new_zeros((c.shape[0], 3, 3))
        mat[:, 2, 2] = 1
        mat[:, 0, 0] = c
        mat[:, 1, 1] = c
        mat[:, 0, 1] = -s
        mat[:, 1, 0] = s
        return mat

    @classmethod
    def isclose(cls, x, y):
        return (x - y).abs() < cls.TOL

    @classmethod
    def inv_left_jacobian(cls, phi):
        angle = phi.norm(dim=1)
        mask = angle < cls.TOL
        J = phi.new_empty(phi.shape[0], 3, 3)
        Id = cls.Id.repeat(J.shape[0], 1, 1)
        # Near |phi|==0, use third order Taylor expansion
        J[mask] = Id[mask] - 0.5 * cls.wedge(phi[mask])

        angle = angle[~mask]
        axis = phi[~mask] / angle.unsqueeze(1)
        half_angle = angle / 2
        cot = 1 / torch.tan(half_angle)
        J[~mask] = (
            (half_angle * cot).unsqueeze(1).unsqueeze(1) * Id[~mask]
            + (1 - half_angle * cot).unsqueeze(1).unsqueeze(1) * bouter(axis, axis)
            - half_angle.unsqueeze(1).unsqueeze(1) * cls.wedge(axis)
        )
        return J

    @classmethod
    def uinv_left_jacobian(cls, phi):
        return cls.inv_left_jacobian(phi.unsqueeze(0)).squeeze()

    @classmethod
    def left_jacobian(cls, phi):
        angle = phi.norm(dim=1)
        mask = angle < cls.TOL
        J = phi.new_empty(phi.shape[0], 3, 3)
        # Near |phi|==0, use third order Taylor expansion
        Id = cls.Id.repeat(J.shape[0], 1, 1)
        J[mask] = Id[mask] - 0.5 * cls.wedge(phi[mask])

        angle = angle[~mask]
        axis = phi[~mask] / angle.unsqueeze(1)
        s = torch.sin(angle) / angle
        c = (1 - torch.cos(angle)) / angle

        J[~mask] = (
            s.unsqueeze(1).unsqueeze(1) * Id[~mask]
            + (1 - s).unsqueeze(1).unsqueeze(1) * bouter(axis, axis)
            + c.unsqueeze(1).unsqueeze(1) * cls.wedge(axis)
        )

        return J

    @classmethod
    def uleft_jacobian(cls, phi):
        return cls.left_jacobian(phi.unsqueeze(0)).squeeze()

    @classmethod
    def to_rpy(cls, Rots):
        """Convert a rotation matrix to RPY Euler angles."""

        pitch = torch.atan2(
            -Rots[:, 2, 0], torch.sqrt(Rots[:, 0, 0] ** 2 + Rots[:, 1, 0] ** 2)
        )
        yaw = pitch.new_empty(pitch.shape)
        roll = pitch.new_empty(pitch.shape)

        near_pi_over_two_mask = cls.isclose(pitch, np.pi / 2.0)
        near_neg_pi_over_two_mask = cls.isclose(pitch, -np.pi / 2.0)

        remainder_inds = ~(near_pi_over_two_mask | near_neg_pi_over_two_mask)

        yaw[near_pi_over_two_mask] = 0
        roll[near_pi_over_two_mask] = torch.atan2(
            Rots[near_pi_over_two_mask, 0, 1], Rots[near_pi_over_two_mask, 1, 1]
        )

        yaw[near_neg_pi_over_two_mask] = 0.0
        roll[near_neg_pi_over_two_mask] = -torch.atan2(
            Rots[near_neg_pi_over_two_mask, 0, 1], Rots[near_neg_pi_over_two_mask, 1, 1]
        )

        sec_pitch = 1 / pitch[remainder_inds].cos()
        remainder_mats = Rots[remainder_inds]
        yaw = torch.atan2(
            remainder_mats[:, 1, 0] * sec_pitch, remainder_mats[:, 0, 0] * sec_pitch
        )
        roll = torch.atan2(
            remainder_mats[:, 2, 1] * sec_pitch, remainder_mats[:, 2, 2] * sec_pitch
        )
        rpys = torch.cat(
            [roll.unsqueeze(dim=1), pitch.unsqueeze(dim=1), yaw.unsqueeze(dim=1)], dim=1
        )
        return rpys

    @classmethod
    def from_quaternion(cls, quat, ordering="wxyz"):
        """Form a rotation matrix from a unit length quaternion.
        Valid orderings are 'xyzw' and 'wxyz'.
        """
        if ordering == "xyzw":
            qx = quat[:, 0]
            qy = quat[:, 1]
            qz = quat[:, 2]
            qw = quat[:, 3]
        elif ordering == "wxyz":
            qw = quat[:, 0]
            qx = quat[:, 1]
            qy = quat[:, 2]
            qz = quat[:, 3]

        # Form the matrix
        mat = quat.new_empty(quat.shape[0], 3, 3)

        qx2 = qx * qx
        qy2 = qy * qy
        qz2 = qz * qz

        mat[:, 0, 0] = 1.0 - 2.0 * (qy2 + qz2)
        mat[:, 0, 1] = 2.0 * (qx * qy - qw * qz)
        mat[:, 0, 2] = 2.0 * (qw * qy + qx * qz)

        mat[:, 1, 0] = 2.0 * (qw * qz + qx * qy)
        mat[:, 1, 1] = 1.0 - 2.0 * (qx2 + qz2)
        mat[:, 1, 2] = 2.0 * (qy * qz - qw * qx)

        mat[:, 2, 0] = 2.0 * (qx * qz - qw * qy)
        mat[:, 2, 1] = 2.0 * (qw * qx + qy * qz)
        mat[:, 2, 2] = 1.0 - 2.0 * (qx2 + qy2)
        return mat

    @classmethod
    def to_quaternion(cls, Rots, ordering="wxyz"):
        """Convert a rotation matrix to a unit length quaternion.
        Valid orderings are 'xyzw' and 'wxyz'.
        """
        tmp = 1 + Rots[:, 0, 0] + Rots[:, 1, 1] + Rots[:, 2, 2]
        tmp[tmp < 0] = 0
        qw = 0.5 * torch.sqrt(tmp)
        qx = qw.new_empty(qw.shape[0])
        qy = qw.new_empty(qw.shape[0])
        qz = qw.new_empty(qw.shape[0])

        near_zero_mask = qw.abs() < cls.TOL

        if near_zero_mask.sum() > 0:
            cond1_mask = (
                near_zero_mask
                * (Rots[:, 0, 0] > Rots[:, 1, 1])
                * (Rots[:, 0, 0] > Rots[:, 2, 2])
            )
            cond1_inds = cond1_mask.nonzero()

            if len(cond1_inds) > 0:
                cond1_inds = cond1_inds.squeeze()
                R_cond1 = Rots[cond1_inds].view(-1, 3, 3)
                d = 2.0 * torch.sqrt(
                    1.0 + R_cond1[:, 0, 0] - R_cond1[:, 1, 1] - R_cond1[:, 2, 2]
                ).view(-1)
                qw[cond1_inds] = (R_cond1[:, 2, 1] - R_cond1[:, 1, 2]) / d
                qx[cond1_inds] = 0.25 * d
                qy[cond1_inds] = (R_cond1[:, 1, 0] + R_cond1[:, 0, 1]) / d
                qz[cond1_inds] = (R_cond1[:, 0, 2] + R_cond1[:, 2, 0]) / d

            cond2_mask = near_zero_mask * (Rots[:, 1, 1] > Rots[:, 2, 2])
            cond2_inds = cond2_mask.nonzero()

            if len(cond2_inds) > 0:
                cond2_inds = cond2_inds.squeeze()
                R_cond2 = Rots[cond2_inds].view(-1, 3, 3)
                d = (
                    2.0
                    * torch.sqrt(
                        1.0 + R_cond2[:, 1, 1] - R_cond2[:, 0, 0] - R_cond2[:, 2, 2]
                    ).squeeze()
                )
                tmp = (R_cond2[:, 0, 2] - R_cond2[:, 2, 0]) / d
                qw[cond2_inds] = tmp
                qx[cond2_inds] = (R_cond2[:, 1, 0] + R_cond2[:, 0, 1]) / d
                qy[cond2_inds] = 0.25 * d
                qz[cond2_inds] = (R_cond2[:, 2, 1] + R_cond2[:, 1, 2]) / d

            cond3_mask = (
                near_zero_mask & cond1_mask.logical_not() & cond2_mask.logical_not()
            )
            cond3_inds = cond3_mask

            if len(cond3_inds) > 0:
                R_cond3 = Rots[cond3_inds].view(-1, 3, 3)
                d = (
                    2.0
                    * torch.sqrt(
                        1.0 + R_cond3[:, 2, 2] - R_cond3[:, 0, 0] - R_cond3[:, 1, 1]
                    ).squeeze()
                )
                qw[cond3_inds] = (R_cond3[:, 1, 0] - R_cond3[:, 0, 1]) / d
                qx[cond3_inds] = (R_cond3[:, 0, 2] + R_cond3[:, 2, 0]) / d
                qy[cond3_inds] = (R_cond3[:, 2, 1] + R_cond3[:, 1, 2]) / d
                qz[cond3_inds] = 0.25 * d

        far_zero_mask = near_zero_mask.logical_not()
        far_zero_inds = far_zero_mask
        if len(far_zero_inds) > 0:
            R_fz = Rots[far_zero_inds]
            d = 4.0 * qw[far_zero_inds]
            qx[far_zero_inds] = (R_fz[:, 2, 1] - R_fz[:, 1, 2]) / d
            qy[far_zero_inds] = (R_fz[:, 0, 2] - R_fz[:, 2, 0]) / d
            qz[far_zero_inds] = (R_fz[:, 1, 0] - R_fz[:, 0, 1]) / d

        # Check ordering last
        if ordering == "xyzw":
            quat = torch.stack([qx, qy, qz, qw], dim=1)
        elif ordering == "wxyz":
            quat = torch.stack([qw, qx, qy, qz], dim=1)
        return quat

    @classmethod
    def normalize(cls, Rots):
        U, _, V = torch.svd(Rots)
        S = cls.Id.clone().repeat(Rots.shape[0], 1, 1)
        S[:, 2, 2] = torch.det(U) * torch.det(V)
        return U.bmm(S).bmm(V.transpose(1, 2))

    @classmethod
    def qmul(cls, q, r, ordering="wxyz"):
        """
        Multiply quaternion(s) q with quaternion(s) r.
        """
        terms = bouter(r, q)
        w = terms[:, 0, 0] - terms[:, 1, 1] - terms[:, 2, 2] - terms[:, 3, 3]
        x = terms[:, 0, 1] + terms[:, 1, 0] - terms[:, 2, 3] + terms[:, 3, 2]
        y = terms[:, 0, 2] + terms[:, 1, 3] + terms[:, 2, 0] - terms[:, 3, 1]
        z = terms[:, 0, 3] - terms[:, 1, 2] + terms[:, 2, 1] + terms[:, 3, 0]
        xyz = torch.stack((x, y, z), dim=1)
        xyz[w < 0] *= -1
        w[w < 0] *= -1
        if ordering == "wxyz":
            q = torch.cat((w.unsqueeze(1), xyz), dim=1)
        else:
            q = torch.cat((xyz, w.unsqueeze(1)), dim=1)
        return q / q.norm(dim=1, keepdim=True)

    @staticmethod
    def sinc(x):
        return x.sin() / x

    @classmethod
    def qexp(cls, xi, ordering="wxyz"):
        """
        Convert exponential maps to quaternions.
        """
        theta = xi.norm(dim=1, keepdim=True)
        w = (0.5 * theta).cos()
        xyz = 0.5 * cls.sinc(0.5 * theta / np.pi) * xi
        return torch.cat((w, xyz), 1)

    @classmethod
    def qlog(cls, q, ordering="wxyz"):
        """
        Applies the log map to quaternions.
        """
        n = 0.5 * torch.norm(q[:, 1:], p=2, dim=1, keepdim=True)
        n = torch.clamp(n, min=1e-8)
        q = q[:, 1:] * torch.acos(torch.clamp(q[:, :1], min=-1.0, max=1.0))
        r = q / n
        return r

    @classmethod
    def qinv(cls, q, ordering="wxyz"):
        "Quaternion inverse"
        r = torch.empty_like(q)
        if ordering == "wxyz":
            r[:, 1:4] = -q[:, 1:4]
            r[:, 0] = q[:, 0]
        else:
            r[:, :3] = -q[:, :3]
            r[:, 3] = q[:, 3]
        return r

    @classmethod
    def qnorm(cls, q):
        "Quaternion normalization"
        return q / q.norm(dim=1, keepdim=True)

    @classmethod
    def qinterp(cls, qs, t, t_int):
        idxs = np.searchsorted(t, t_int)
        idxs0 = idxs - 1
        idxs0[idxs0 < 0] = 0
        idxs1 = idxs
        idxs1[idxs1 == t.shape[0]] = t.shape[0] - 1
        q0 = qs[idxs0]
        q1 = qs[idxs1]
        tau = torch.zeros_like(t_int)
        dt = (t[idxs1] - t[idxs0])[idxs0 != idxs1]
        tau[idxs0 != idxs1] = (t_int - t[idxs0])[idxs0 != idxs1] / dt
        return cls.slerp(q0, q1, tau)

    @classmethod
    def slerp(cls, q0, q1, tau, DOT_THRESHOLD=0.9995):
        """Spherical linear interpolation."""

        dot = (q0 * q1).sum(dim=1)
        q1[dot < 0] = -q1[dot < 0]
        dot[dot < 0] = -dot[dot < 0]

        q = torch.zeros_like(q0)
        tmp = q0 + tau.unsqueeze(1) * (q1 - q0)
        tmp = tmp[dot > DOT_THRESHOLD]
        q[dot > DOT_THRESHOLD] = tmp / tmp.norm(dim=1, keepdim=True)

        theta_0 = dot.acos()
        sin_theta_0 = theta_0.sin()
        theta = theta_0 * tau
        sin_theta = theta.sin()
        s0 = (theta.cos() - dot * sin_theta / sin_theta_0).unsqueeze(1)
        s1 = (sin_theta / sin_theta_0).unsqueeze(1)
        q[dot < DOT_THRESHOLD] = ((s0 * q0) + (s1 * q1))[dot < DOT_THRESHOLD]
        return q / q.norm(dim=1, keepdim=True)

    @classmethod
    def J2(cls, phi):
        angle = phi.norm(dim=1)
        mask = angle < cls.TOL
        J = phi.new_empty(phi.shape[0], 3, 3)
        # Near |phi|==0, use third order Taylor expansion
        Id = cls.Id.repeat(J.shape[0], 1, 1)
        J[mask] = 0.5 * Id[mask] - 1 / 6 * cls.wedge(phi[mask])

        angle = angle[~mask]
        angle2 = torch.pow(angle, 2)
        axis = phi[~mask] / angle.unsqueeze(1)
        s = (angle - torch.sin(angle)) / angle2
        c = (1 - torch.cos(angle)) / angle2

        J[~mask] = (
            c.unsqueeze(1).unsqueeze(1) * Id[~mask]
            + (0.5 - c).unsqueeze(1).unsqueeze(1) * bouter(axis, axis)
            + s.unsqueeze(1).unsqueeze(1) * cls.wedge(axis)
        )

        return J

    @classmethod
    def uJ2(cls, phi):
        return cls.J2(phi.unsqueeze(0)).squeeze()

    @classmethod
    def J3(cls, phi):
        angle = phi.norm(dim=1)
        mask = angle < cls.TOL
        J = phi.new_empty(phi.shape[0], 3, 3)
        # Near |phi|==0, use third order Taylor expansion
        Id = cls.Id.repeat(J.shape[0], 1, 1)
        J[mask] = 1 / 6 * Id[mask] - 1 / 24 * cls.wedge(phi[mask])

        angle = angle[~mask]
        angle3 = torch.pow(angle, 3)
        axis = phi[~mask] / angle.unsqueeze(1)
        s = (angle - torch.sin(angle)) / angle3
        c = (torch.pow(angle, 2) + 2 * torch.cos(angle) - 2) / angle3

        J[~mask] = (
            s.unsqueeze(1).unsqueeze(1) * Id[~mask]
            + (1 / 6 - s).unsqueeze(1).unsqueeze(1) * bouter(axis, axis)
            + c.unsqueeze(1).unsqueeze(1) * cls.wedge(axis)
        )

        return J

    @classmethod
    def uJ3(cls, phi):
        return cls.J3(phi.unsqueeze(0)).squeeze()

    @classmethod
    def U1(cls, phi):
        angle = phi.norm(dim=1)
        mask = angle < cls.TOL
        J = phi.new_empty(phi.shape[0], 3, 3)
        # Near |phi|==0, use third order Taylor expansion
        Id = cls.Id.repeat(J.shape[0], 1, 1)
        J[mask] = 0.5 * Id[mask] + 1 / 3 * cls.wedge(phi[mask])

        angle = angle[~mask]
        angle2 = torch.pow(angle, 2)
        cosa = torch.cos(angle)
        sina = torch.sin(angle)
        axis = phi[~mask] / angle.unsqueeze(1)
        s = (angle * sina + cosa - 1) / angle2
        c = (sina - angle * cosa) / angle2

        J[~mask] = (
            s.unsqueeze(1).unsqueeze(1) * Id[~mask]
            + (0.5 - s).unsqueeze(1).unsqueeze(1) * bouter(axis, axis)
            + c.unsqueeze(1).unsqueeze(1) * cls.wedge(axis)
        )

        return J

    @classmethod
    def uU1(cls, phi):
        return cls.U1(phi.unsqueeze(0)).squeeze()
