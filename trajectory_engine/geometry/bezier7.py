from __future__ import annotations

from models.types import XYZ3
from trajectory_engine.models.trajectory_primitives import Bezier7Coefficients3D, Bezier7ControlPoints3D


def _add(a: XYZ3, b: XYZ3) -> XYZ3:
    return XYZ3(a.x + b.x, a.y + b.y, a.z + b.z)


def _sub(a: XYZ3, b: XYZ3) -> XYZ3:
    return XYZ3(a.x - b.x, a.y - b.y, a.z - b.z)


def _scale(v: XYZ3, factor: float) -> XYZ3:
    return XYZ3(v.x * factor, v.y * factor, v.z * factor)


def _combine(
    p0: XYZ3,
    c0: float,
    p1: XYZ3,
    c1: float,
    p2: XYZ3,
    c2: float,
    p3: XYZ3,
    c3: float,
    p4: XYZ3,
    c4: float,
    p5: XYZ3,
    c5: float,
    p6: XYZ3,
    c6: float,
    p7: XYZ3,
    c7: float,
) -> XYZ3:
    return XYZ3(
        p0.x * c0 + p1.x * c1 + p2.x * c2 + p3.x * c3 + p4.x * c4 + p5.x * c5 + p6.x * c6 + p7.x * c7,
        p0.y * c0 + p1.y * c1 + p2.y * c2 + p3.y * c3 + p4.y * c4 + p5.y * c5 + p6.y * c6 + p7.y * c7,
        p0.z * c0 + p1.z * c1 + p2.z * c2 + p3.z * c3 + p4.z * c4 + p5.z * c5 + p6.z * c6 + p7.z * c7,
    )


class Bezier7Curve3D:
    def __init__(self, control_points: Bezier7ControlPoints3D) -> None:
        self.control_points = control_points
        self.coefficients = self._compute_coefficients(control_points)

    @staticmethod
    def _compute_coefficients(points: Bezier7ControlPoints3D) -> Bezier7Coefficients3D:
        p0 = points.p0
        p1 = points.p1
        p2 = points.p2
        p3 = points.p3
        p4 = points.p4
        p5 = points.p5
        p6 = points.p6
        p7 = points.p7
        return Bezier7Coefficients3D(
            a0=p0.copy(),
            a1=_scale(_sub(p1, p0), 7.0),
            a2=_scale(_combine(p0, 1.0, p1, -2.0, p2, 1.0, p3, 0.0, p4, 0.0, p5, 0.0, p6, 0.0, p7, 0.0), 21.0),
            a3=_scale(_combine(p0, 1.0, p1, -3.0, p2, 3.0, p3, -1.0, p4, 0.0, p5, 0.0, p6, 0.0, p7, 0.0), -35.0),
            a4=_scale(_combine(p0, 1.0, p1, -4.0, p2, 6.0, p3, -4.0, p4, 1.0, p5, 0.0, p6, 0.0, p7, 0.0), 35.0),
            a5=_scale(_combine(p0, 1.0, p1, -5.0, p2, 10.0, p3, -10.0, p4, 5.0, p5, -1.0, p6, 0.0, p7, 0.0), -21.0),
            a6=_scale(_combine(p0, 1.0, p1, -6.0, p2, 15.0, p3, -20.0, p4, 15.0, p5, -6.0, p6, 1.0, p7, 0.0), 7.0),
            a7=_combine(p0, -1.0, p1, 7.0, p2, -21.0, p3, 35.0, p4, -35.0, p5, 21.0, p6, -7.0, p7, 1.0),
        )

    @staticmethod
    def from_handles(start: XYZ3, end: XYZ3, out_tangent: XYZ3, in_tangent: XYZ3) -> "Bezier7Curve3D":
        p0 = start.copy()
        p1 = _add(p0, out_tangent)
        p2 = _add(p0, _scale(out_tangent, 2.0))
        p3 = _add(p0, _scale(out_tangent, 3.0))
        p7 = end.copy()
        p6 = _add(p7, in_tangent)
        p5 = _add(p7, _scale(in_tangent, 2.0))
        p4 = _add(p7, _scale(in_tangent, 3.0))
        return Bezier7Curve3D(Bezier7ControlPoints3D(p0, p1, p2, p3, p4, p5, p6, p7))

    @staticmethod
    def linear(start: XYZ3, end: XYZ3) -> "Bezier7Curve3D":
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z
        return Bezier7Curve3D(
            Bezier7ControlPoints3D(
                start.copy(),
                XYZ3(start.x + dx / 7.0, start.y + dy / 7.0, start.z + dz / 7.0),
                XYZ3(start.x + 2.0 * dx / 7.0, start.y + 2.0 * dy / 7.0, start.z + 2.0 * dz / 7.0),
                XYZ3(start.x + 3.0 * dx / 7.0, start.y + 3.0 * dy / 7.0, start.z + 3.0 * dz / 7.0),
                XYZ3(start.x + 4.0 * dx / 7.0, start.y + 4.0 * dy / 7.0, start.z + 4.0 * dz / 7.0),
                XYZ3(start.x + 5.0 * dx / 7.0, start.y + 5.0 * dy / 7.0, start.z + 5.0 * dz / 7.0),
                XYZ3(start.x + 6.0 * dx / 7.0, start.y + 6.0 * dy / 7.0, start.z + 6.0 * dz / 7.0),
                end.copy(),
            )
        )

    def point(self, u: float) -> XYZ3:
        u = max(0.0, min(1.0, float(u)))
        c = self.coefficients
        return _add(
            _scale(
                _add(
                    _scale(
                        _add(
                            _scale(
                                _add(
                                    _scale(
                                        _add(
                                            _scale(
                                                _add(_scale(_add(_scale(c.a7, u), c.a6), u), c.a5),
                                                u,
                                            ),
                                            c.a4,
                                        ),
                                        u,
                                    ),
                                    c.a3,
                                ),
                                u,
                            ),
                            c.a2,
                        ),
                        u,
                    ),
                    c.a1,
                ),
                u,
            ),
            c.a0,
        )

    def first_derivative(self, u: float) -> XYZ3:
        u = max(0.0, min(1.0, float(u)))
        c = self.coefficients
        return _add(
            _scale(
                _add(
                    _scale(
                        _add(
                            _scale(
                                _add(
                                    _scale(
                                        _add(_scale(_add(_scale(c.a7, 7.0), _scale(c.a6, 6.0)), u),
                                             _scale(c.a5, 5.0)),
                                        u,
                                    ),
                                    _scale(c.a4, 4.0),
                                ),
                                u,
                            ),
                            _scale(c.a3, 3.0),
                        ),
                        u,
                    ),
                    _scale(c.a2, 2.0),
                ),
                u,
            ),
            c.a1,
        )

    def second_derivative(self, u: float) -> XYZ3:
        u = max(0.0, min(1.0, float(u)))
        c = self.coefficients
        return _add(
            _scale(
                _add(
                    _scale(
                        _add(
                            _scale(_add(_scale(_add(_scale(c.a7, 42.0), _scale(c.a6, 30.0)), u), _scale(c.a5, 20.0)), u),
                            _scale(c.a4, 12.0),
                        ),
                        u,
                    ),
                    _scale(c.a3, 6.0),
                ),
                u,
            ),
            _scale(c.a2, 2.0),
        )

    def third_derivative(self, u: float) -> XYZ3:
        u = max(0.0, min(1.0, float(u)))
        c = self.coefficients
        return _add(
            _scale(
                _add(
                    _scale(_add(_scale(_add(_scale(c.a7, 210.0), _scale(c.a6, 120.0)), u), _scale(c.a5, 60.0)), u),
                    _scale(c.a4, 24.0),
                ),
                u,
            ),
            _scale(c.a3, 6.0),
        )
