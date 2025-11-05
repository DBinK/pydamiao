# app/core/logging.py

from loguru import logger
import sys
from pathlib import Path

# 中文注释：确保日志目录存在
LOG_DIR = Path("logs")
LOG_DIR.mkdir(exist_ok=True)

# 中文注释：取消默认 handler
logger.remove()

# 控制台输出（彩色、INFO 以上）
logger.add(
    sys.stderr,
    level="INFO",
    colorize=True,
    enqueue=True,
)

__all__ = ["logger"]
