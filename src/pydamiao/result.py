from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, TypeVar, cast

T = TypeVar("T")


@dataclass(slots=True)
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

    @property
    def ok(self) -> bool:
        """返回本次操作是否成功。"""
        return self.error is None

    def expect(self, message: str | None = None) -> T:
        """返回结果值，失败时抛出异常。

        Args:
            message: 可选的覆盖错误消息。

        Returns:
            成功时的结果值。

        Raises:
            RuntimeError: 当结果中包含错误时抛出。
        """
        if self.error is not None:
            raise RuntimeError(message or self.error)
        return cast(T, self.value)

    def __bool__(self) -> bool:
        return self.error is None
