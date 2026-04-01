from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, TypeVar

T = TypeVar("T")


@dataclass(slots=True)
class Result(Generic[T]):
    value: T | None = None
    error: str | None = None
    code: str | None = None

    @property
    def ok(self) -> bool:
        return self.error is None

    def expect(self, message: str | None = None) -> T:
        if self.error is not None:
            raise RuntimeError(message or self.error)
        return self.value

    def __bool__(self) -> bool:
        return self.error is None
