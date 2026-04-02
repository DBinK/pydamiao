
from dataclasses import dataclass
from typing import Callable, Generic, Optional, TypeVar, cast

T = TypeVar("T")
U = TypeVar("U")


@dataclass(slots=True, frozen=True)
class Result(Generic[T]):
    """表示一次高层电机操作的结果。

    Attributes:
        value: 操作成功时返回的值。
        error: 操作失败时的人类可读错误信息。
        code: 稳定的机器可读错误码。
    """

    value: T | None = None
    error: str | None = None
    code: str | None = None

    @classmethod
    def ok(cls, value: Optional[T] = None) -> "Result[T]":
        """创建成功结果。"""
        return cls(value=value)

    @classmethod
    def err(cls, error: str, code: Optional[str] = None) -> "Result[T]":
        """创建错误结果。"""
        return cls(error=error, code=code)

    @property
    def is_ok(self) -> bool:
        return self.error is None

    @property
    def is_err(self) -> bool:
        return self.error is not None

    def unwrap(self) -> T:
        """获取结果中的值。"""
        if self.is_err:
            raise RuntimeError(f"Unwrap failed: {self.error}")
        return cast(T, self.value)

    def unwrap_or(self, default: T) -> T:
        """获取结果中的值，如果结果为错误则返回默认值。"""
        if self.is_ok:
            return cast(T, self.value)
        return default

    def map(self, func: Callable[[T], U]) -> "Result[U]":
        """将结果中的值映射为另一个结果, 以实现链式调用。"""
        if self.is_ok:
            return Result.ok(func(cast(T, self.value)))
        return Result.err(cast(str, self.error), self.code)

    def __bool__(self) -> bool:
        """返回结果是否成功。"""
        return self.is_ok
