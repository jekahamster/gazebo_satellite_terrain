from dataclasses import dataclass
from typing import Generic, TypeVar

T = TypeVar("T", int, float)


@dataclass(frozen=True, slots=True)
class Coord(Generic[T]):
    """
    2D coordinate; generic over T so you can use Coord[int] (e.g. tile indices)
    or Coord[float] (e.g. meters, GPS-derived positions).
    """
    x: T = 0  # type: ignore[assignment]
    y: T = 0  # type: ignore[assignment]

    def __repr__(self) -> str:
        return f"({self.x}, {self.y})"

    def __add__(self, other: "Coord[T] | int | float") -> "Coord[T]":
        if isinstance(other, Coord):
            return Coord(self.x + other.x, self.y + other.y)  # type: ignore[return-value]
        elif isinstance(other, int | float):
            return Coord(self.x + other, self.y + other)  # type: ignore[return-value]
        else:
            raise TypeError(f"Operand not supported between instances of '{type(self).__name__}' and '{type(other)}'")

    def __neg__(self) -> "Coord[T]":
        return Coord(-self.x, -self.y)  # type: ignore[return-value]

    def __sub__(self, other: "Coord[T] | int | float") -> "Coord[T]":
        return self + -other  # type: ignore[return-value]

    def __mul__(self, other: "Coord[T] | int | float") -> "Coord[T]":
        if isinstance(other, Coord):
            return Coord(self.x * other.x, self.y * other.y)  # type: ignore[return-value]
        elif isinstance(other, int | float):
            return Coord(self.x * other, self.y * other)  # type: ignore[return-value]
        else:
            raise TypeError(f"Operand not supported between instances of '{type(self).__name__}' and '{type(other)}'")

    def __floordiv__(self, other: "Coord[T] | int | float") -> "Coord[int]":
        if isinstance(other, Coord):
            return Coord(int(self.x // other.x), int(self.y // other.y))
        elif isinstance(other, int | float):
            return Coord(int(self.x // other), int(self.y // other))
        else:
            raise TypeError(f"Operand not supported between instances of '{type(self).__name__}' and '{type(other)}'")

    def __truediv__(self, other: "Coord[T] | int | float") -> "Coord[float]":
        if isinstance(other, Coord):
            return Coord(self.x / other.x, self.y / other.y)
        elif isinstance(other, int | float):
            return Coord(self.x / other, self.y / other)
        else:
            raise TypeError(f"Operand not supported between instances of '{type(self).__name__}' and '{type(other)}'")

    def __radd__(self, other: "Coord[T] | int | float") -> "Coord[T]":
        return self + other  # type: ignore[return-value]

    def __rsub__(self, other: "Coord[T] | int | float") -> "Coord[T]":
        return self - other  # type: ignore[return-value]

    def __rmul__(self, other: "Coord[T] | int | float") -> "Coord[T]":
        return self * other  # type: ignore[return-value]

    def __lt__(self, other: "Coord[T] | int | float") -> bool:
        if isinstance(other, Coord):
            return self.x < other.x and self.y < other.y  # type: ignore[return-value]
        elif isinstance(other, int | float):
            return self.x < other and self.y < other  # type: ignore[return-value]
        else:
            raise TypeError(f"'<' not supported between instances of '{type(self).__name__}' and '{type(other)}'")

    def __le__(self, other: "Coord[T] | int | float") -> bool:
        if isinstance(other, Coord):
            return self.x <= other.x and self.y <= other.y  # type: ignore[return-value]
        elif isinstance(other, int | float):
            return self.x <= other and self.y <= other  # type: ignore[return-value]
        else:
            raise TypeError(f"'<=' not supported between instances of '{type(self).__name__}' and '{type(other)}'")

    def __gt__(self, other: "Coord[T] | int | float") -> bool:
        return not self <= other

    def __ge__(self, other: "Coord[T] | int | float") -> bool:
        return not self < other

