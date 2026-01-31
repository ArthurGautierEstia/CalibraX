import re

def str_to_float(s, default: float = 0.0) -> float:
    if re.match(r'^[+-]?(\d+\.?\d*|\.\d+)$', s.strip()):
        return float(s)
    return default
