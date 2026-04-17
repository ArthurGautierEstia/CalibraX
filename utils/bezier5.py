from __future__ import annotations

from utils.bezier3 import _to_vec3, _vec3_add


def _vec3_scale(values: list[float], scale: float) -> list[float]:
    return [float(values[0]) * scale, float(values[1]) * scale, float(values[2]) * scale]


class Bezier5Coefficients:
    """
    Quintic Bezier polynomial coefficients for one scalar axis:
    f(t) = A*t^5 + B*t^4 + C*t^3 + D*t^2 + E*t + F, with t in [0, 1].
    """

    def __init__(self, p0: float, p1: float, p2: float, p3: float, p4: float, p5: float) -> None:
        self.p0 = float(p0)
        self.p1 = float(p1)
        self.p2 = float(p2)
        self.p3 = float(p3)
        self.p4 = float(p4)
        self.p5 = float(p5)

        self.a = 0.0
        self.b = 0.0
        self.c = 0.0
        self.d = 0.0
        self.e = 0.0
        self.f = 0.0

        self._5a = 0.0
        self._4b = 0.0
        self._3c = 0.0
        self._2d = 0.0
        self._20a = 0.0
        self._12b = 0.0
        self._6c = 0.0

        self.compute()

    def compute(self) -> None:
        self.a = -self.p0 + 5.0 * self.p1 - 10.0 * self.p2 + 10.0 * self.p3 - 5.0 * self.p4 + self.p5
        self.b = 5.0 * (self.p0 - 4.0 * self.p1 + 6.0 * self.p2 - 4.0 * self.p3 + self.p4)
        self.c = -10.0 * (self.p0 - 3.0 * self.p1 + 3.0 * self.p2 - self.p3)
        self.d = 10.0 * (self.p0 - 2.0 * self.p1 + self.p2)
        self.e = -5.0 * (self.p0 - self.p1)
        self.f = self.p0

        self._5a = 5.0 * self.a
        self._4b = 4.0 * self.b
        self._3c = 3.0 * self.c
        self._2d = 2.0 * self.d
        self._20a = 20.0 * self.a
        self._12b = 12.0 * self.b
        self._6c = 6.0 * self.c

    def value(self, t: float) -> float:
        t = float(t)
        return ((((self.a * t + self.b) * t + self.c) * t + self.d) * t + self.e) * t + self.f

    def velocity(self, t: float) -> float:
        t = float(t)
        return (((self._5a * t + self._4b) * t + self._3c) * t + self._2d) * t + self.e

    def acceleration(self, t: float) -> float:
        t = float(t)
        return ((self._20a * t + self._12b) * t + self._6c) * t + self._2d

    def as_list(self) -> list[float]:
        return [self.a, self.b, self.c, self.d, self.e, self.f]


class Bezier5Coefficients3D:
    """
    Quintic Bezier polynomial coefficients for x(t), y(t), z(t).
    Inputs:
      - p0: segment start point
      - p5: segment end point
      - t_out: visible outgoing influence vector from p0 (creates p1 = p0 + t_out)
      - t_in: visible incoming influence vector from p5 (creates p4 = p5 + t_in)

    The second handles are inferred as p2 = p0 + 2*t_out and p3 = p5 + 2*t_in.
    This makes the parametric acceleration zero at both segment boundaries, which
    gives C2 continuity at chained keypoints when the visible handles are C1-compatible.
    """

    def __init__(
        self,
        p0: list[float] | tuple[float, float, float],
        p5: list[float] | tuple[float, float, float],
        t_out: list[float] | tuple[float, float, float] | None = None,
        t_in: list[float] | tuple[float, float, float] | None = None,
    ) -> None:
        self.p0 = _to_vec3(p0)
        self.p5 = _to_vec3(p5)
        self.t_out = _to_vec3([0.0, 0.0, 0.0] if t_out is None else t_out)
        self.t_in = _to_vec3([0.0, 0.0, 0.0] if t_in is None else t_in)

        self.p1 = [0.0, 0.0, 0.0]
        self.p2 = [0.0, 0.0, 0.0]
        self.p3 = [0.0, 0.0, 0.0]
        self.p4 = [0.0, 0.0, 0.0]

        self.x = Bezier5Coefficients(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.y = Bezier5Coefficients(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.z = Bezier5Coefficients(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.compute()

    def compute(self) -> None:
        self.p1 = _vec3_add(self.p0, self.t_out)
        self.p2 = _vec3_add(self.p0, _vec3_scale(self.t_out, 2.0))
        self.p4 = _vec3_add(self.p5, self.t_in)
        self.p3 = _vec3_add(self.p5, _vec3_scale(self.t_in, 2.0))

        self.x = Bezier5Coefficients(self.p0[0], self.p1[0], self.p2[0], self.p3[0], self.p4[0], self.p5[0])
        self.y = Bezier5Coefficients(self.p0[1], self.p1[1], self.p2[1], self.p3[1], self.p4[1], self.p5[1])
        self.z = Bezier5Coefficients(self.p0[2], self.p1[2], self.p2[2], self.p3[2], self.p4[2], self.p5[2])

    def point(self, t: float) -> list[float]:
        return [self.x.value(t), self.y.value(t), self.z.value(t)]

    def first_derivative(self, t: float) -> list[float]:
        return [self.x.velocity(t), self.y.velocity(t), self.z.velocity(t)]

    def second_derivative(self, t: float) -> list[float]:
        return [self.x.acceleration(t), self.y.acceleration(t), self.z.acceleration(t)]

    def coefficients_xyz(self) -> tuple[list[float], list[float], list[float]]:
        return self.x.as_list(), self.y.as_list(), self.z.as_list()


class Bezier5Sample:
    """
    Node for chained Bezier5 segments.
    - point: [x, y, z]
    - t_out: outgoing visible influence vector for segment starting at this node
    - t_in: incoming visible influence vector for segment ending at this node
    """

    def __init__(
        self,
        point: list[float] | tuple[float, float, float],
        t_out: list[float] | tuple[float, float, float] | None = None,
        t_in: list[float] | tuple[float, float, float] | None = None,
    ) -> None:
        self.point = _to_vec3(point)
        self.t_out = _to_vec3([0.0, 0.0, 0.0] if t_out is None else t_out)
        self.t_in = _to_vec3([0.0, 0.0, 0.0] if t_in is None else t_in)


class Bezier5:
    """
    Multi-segment quintic Bezier utility.
    Segment i uses:
      start = samples[i]
      end = samples[i + 1]
      t_out = start.t_out
      t_in = end.t_in
    """

    def __init__(self, samples: list[Bezier5Sample] | None = None) -> None:
        self._samples: list[Bezier5Sample] = []
        self.segments: list[Bezier5Coefficients3D] = []
        if samples is not None:
            self.set_samples(samples)

    def set_samples(self, samples: list[Bezier5Sample]) -> None:
        self._samples = list(samples)
        self.segments = []

    def compute(self) -> None:
        self.segments = []
        if len(self._samples) < 2:
            return

        for i in range(len(self._samples) - 1):
            start = self._samples[i]
            end = self._samples[i + 1]
            coeffs = Bezier5Coefficients3D(
                p0=start.point,
                p5=end.point,
                t_out=start.t_out,
                t_in=end.t_in,
            )
            coeffs.compute()
            self.segments.append(coeffs)
