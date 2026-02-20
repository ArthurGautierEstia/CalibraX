def _to_vec3(values: list[float] | tuple[float, float, float]) -> list[float]:
    out = [0.0, 0.0, 0.0]
    if values is None:
        return out
    for i in range(min(3, len(values))):
        out[i] = float(values[i])
    return out


def _vec3_add(a: list[float], b: list[float]) -> list[float]:
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


class Bezier3Coefficients:
    """
    Cubic polynomial coefficients for one scalar axis:
    f(t) = A*t^3 + B*t^2 + C*t + D, with t in [0, 1].
    """

    def __init__(self, p0: float, p1: float, p2: float, p3: float) -> None:
        self.p0 = float(p0)
        self.p1 = float(p1)
        self.p2 = float(p2)
        self.p3 = float(p3)

        self.a = 0.0
        self.b = 0.0
        self.c = 0.0
        self.d = 0.0

        self._6a = 0.0
        self._3a = 0.0
        self._2b = 0.0
        
        self.compute()

    def compute(self) -> None:
        self.a = self.p3 - self.p0 + 3.0 * (self.p1 - self.p2)
        self.b = 3.0 * (self.p0 - 2.0 * self.p1 + self.p2)
        self.c = 3.0 * (self.p1 - self.p0)
        self.d = self.p0

        self._6a = 6.0 * self.a
        self._3a = 3.0 * self.a
        self._2b = 2.0 * self.b

    def value(self, t: float) -> float:
        t = float(t)
        return ((self.a * t + self.b) * t + self.c) * t + self.d

    def velocity(self, t: float) -> float:
        t = float(t)
        return (self._3a * t + self._2b) * t + self.c

    def acceleration(self, t: float) -> float:
        t = float(t)
        return self._6a * t + self._2b

    def as_list(self) -> list[float]:
        return [self.a, self.b, self.c, self.d]


class Bezier3Coefficients3D:
    """
    Cubic Bezier polynomial coefficients for x(t), y(t), z(t).
    Inputs:
      - p0: segment start point
      - p3: segment end point
      - t_out: tangent/influence vector from p0 (creates p1 = p0 + t_out)
      - t_in: tangent/influence vector from p3 (creates p2 = p3 + t_in)
    """

    def __init__(
        self,
        p0: list[float] | tuple[float, float, float],
        p3: list[float] | tuple[float, float, float],
        t_out: list[float] | tuple[float, float, float] | None = None,
        t_in: list[float] | tuple[float, float, float] | None = None,
    ) -> None:
        self.p0 = _to_vec3(p0)
        self.p3 = _to_vec3(p3)
        self.t_out = _to_vec3([0.0, 0.0, 0.0] if t_out is None else t_out)
        self.t_in = _to_vec3([0.0, 0.0, 0.0] if t_in is None else t_in)

        self.p1 = _vec3_add(self.p0, self.t_out)
        self.p2 = _vec3_add(self.p3, self.t_in)

        self.x = Bezier3Coefficients(self.p0[0], self.p1[0], self.p2[0], self.p3[0])
        self.y = Bezier3Coefficients(self.p0[1], self.p1[1], self.p2[1], self.p3[1])
        self.z = Bezier3Coefficients(self.p0[2], self.p1[2], self.p2[2], self.p3[2])

    def compute(self) -> None:
        self.p1 = _vec3_add(self.p0, self.t_out)
        self.p2 = _vec3_add(self.p3, self.t_in)

        self.x = Bezier3Coefficients(self.p0[0], self.p1[0], self.p2[0], self.p3[0])
        self.y = Bezier3Coefficients(self.p0[1], self.p1[1], self.p2[1], self.p3[1])
        self.z = Bezier3Coefficients(self.p0[2], self.p1[2], self.p2[2], self.p3[2])

    def point(self, t: float) -> list[float]:
        return [self.x.value(t), self.y.value(t), self.z.value(t)]

    def first_derivative(self, t: float) -> list[float]:
        return [self.x.velocity(t), self.y.velocity(t), self.z.velocity(t)]

    def second_derivative(self, t: float) -> list[float]:
        return [self.x.acceleration(t), self.y.acceleration(t), self.z.acceleration(t)]

    def coefficients_xyz(self) -> tuple[list[float], list[float], list[float]]:
        return self.x.as_list(), self.y.as_list(), self.z.as_list()


class Bezier3Sample:
    """
    Node for chained Bezier3 segments.
    - point: [x, y, z]
    - t_out: outgoing influence vector for segment starting at this node
    - t_in: incoming influence vector for segment ending at this node
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


class Bezier3:
    """
    Multi-segment cubic Bezier utility.
    Segment i uses:
      start = samples[i]
      end = samples[i + 1]
      t_out = start.t_out
      t_in = end.t_in
    """

    def __init__(self, samples: list[Bezier3Sample] | None = None) -> None:
        self._samples: list[Bezier3Sample] = []
        self.segments: list[Bezier3Coefficients3D] = []
        if samples is not None:
            self.set_samples(samples)

    def set_samples(self, samples: list[Bezier3Sample]) -> None:
        self._samples = list(samples)
        self.segments = []

    def compute(self) -> None:
        self.segments = []
        if len(self._samples) < 2:
            return

        for i in range(len(self._samples) - 1):
            start = self._samples[i]
            end = self._samples[i + 1]
            coeffs = Bezier3Coefficients3D(
                p0=start.point,
                p3=end.point,
                t_out=start.t_out,
                t_in=end.t_in,
            )
            coeffs.compute()
            self.segments.append(coeffs)
