from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, TypeVar

T = TypeVar("T")


@dataclass(slots=True)
class Result(Generic[T]):
    ok: bool
    value: T | None = None
    error: str | None = None
    code: str | None = None

    @classmethod
    def success(cls, value: T | None = None) -> "Result[T]":
        return cls(ok=True, value=value)

    @classmethod
    def failure(cls, error: str, code: str = "error") -> "Result[T]":
        return cls(ok=False, error=error, code=code)

    def unwrap(self) -> T:
        if not self.ok:
            raise ValueError(self.error or "Result is not ok")
        return self.value

    def __bool__(self) -> bool:
        return self.ok
