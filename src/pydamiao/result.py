# src/pydamiao/result.py

from typing import Optional, Generic, TypeVar

T = TypeVar('T')


class DamiaoError(Exception):
    """Base exception for Damiao motor errors."""
    
    def __init__(self, message: str, code: Optional[int] = None):
        self.message = message
        self.code = code
        super().__init__(self.message)
    
    def __str__(self) -> str:
        if self.code is not None:
            return f"[Error {self.code}] {self.message}"
        return self.message


def check_result(result: 'Result[T]', raise_on_error: bool = True) -> Optional[T]:
    """
    Check a Result and optionally raise an error.
    
    This provides a simple way to handle results in Python style:
    
    Example:
        # Option 1: Check manually
        result = motor.enable()
        if not result.ok:
            print(f"Failed: {result.error}")
        
        # Option 2: Auto-raise (default)
        motor.enable()  # Raises DamiaoError on failure
        
        # Option 3: Disable auto-raise
        result = motor.enable(raise_on_error=False)
        if result.ok:
            print("Success!")
    """
    if not result.ok and raise_on_error:
        raise DamiaoError(result.error or "Unknown error", result.code)
    return result.value if result.ok else None


class Result(Generic[T]):
    """
    Simple result container for operation status.
    
    Pythonic usage patterns:
    
    1. Check with boolean:
        if motor.enable():
            print("Enabled!")
    
    2. Get value or None:
        value = motor.read_param(MotorReg.PMAX)
        if value is not None:
            print(f"PMAX: {value}")
    
    3. Try-except:
        try:
            motor.enable()
        except DamiaoError as e:
            print(f"Failed: {e}")
    """
    
    def __init__(
        self, 
        ok: bool = True, 
        value: Optional[T] = None, 
        error: Optional[str] = None,
        code: Optional[int] = None
    ):
        self.ok = ok
        self._value = value
        self.error = error
        self.code = code
    
    @classmethod
    def success(cls, value: Optional[T] = None) -> 'Result[T]':
        """Create a successful result."""
        return cls(ok=True, value=value)
    
    @classmethod
    def failure(
        cls, 
        error: str, 
        code: Optional[int] = None
    ) -> 'Result[T]':
        """Create a failed result."""
        return cls(ok=False, value=None, error=error, code=code)
    
    @property
    def value(self) -> Optional[T]:
        """Get the value (returns None if failed)."""
        return self._value
    
    def __bool__(self) -> bool:
        """Allow using Result in boolean context."""
        return self.ok
    
    def __repr__(self) -> str:
        if self.ok:
            return f"Result.success({self._value!r})"
        else:
            return f"Result.failure({self.error!r}, code={self.code})"
