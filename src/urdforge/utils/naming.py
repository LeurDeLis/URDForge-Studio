"""URDForge Naming Utilities.

Provides sanitization and deduplication of CAD part/assembly names
to conform to ROS / ROS 2 URDF XML naming conventions.
"""

from __future__ import annotations

import re
from typing import Dict, Set


def sanitize_urdf_name(name: str, fallback: str = "link") -> str:
    """Sanitize arbitrary CAD part or assembly name into a valid URDF identifier.

    Rules:
    - Replaces spaces, dashes, slashes, backslashes, colons, dots with underscores.
    - Removes characters not in [a-zA-Z0-9_].
    - Ensures the name does not start with a digit.
    - Removes duplicate consecutive underscores.
    """
    if not name or not name.strip():
        name = fallback

    # Strip extension if present
    name = re.sub(r"\.(step|stp|sldprt|sldasm|iges|igs)$", "", name, flags=re.IGNORECASE)

    # Replace separators with underscore
    cleaned = re.sub(r"[\s\-\/\:\.\,\(\)\[\]\{\}\<\>\+\=\#\@\$\%\^\&\*\!\?\~]+", "_", name)

    # Remove invalid characters
    cleaned = re.sub(r"[^a-zA-Z0-9_]", "", cleaned)

    # Remove consecutive underscores
    cleaned = re.sub(r"_+", "_", cleaned).strip("_")

    if not cleaned:
        cleaned = fallback

    # URDF names should ideally start with an alphabet or underscore
    if cleaned[0].isdigit():
        cleaned = f"n_{cleaned}"

    return cleaned


class UniqueNameGenerator:
    """Generates unique names while preserving human-readable CAD identifiers."""

    def __init__(self) -> None:
        self._used_names: Set[str] = set()
        self._base_counters: Dict[str, int] = {}

    def get_unique_name(self, candidate_name: str, fallback_prefix: str = "link") -> str:
        """Return a unique sanitized name."""
        base = sanitize_urdf_name(candidate_name, fallback=fallback_prefix)

        if base not in self._used_names:
            self._used_names.add(base)
            self._base_counters[base] = 1
            return base

        # Name collision: increment counter
        count = self._base_counters.get(base, 1) + 1
        while True:
            candidate = f"{base}_{count}"
            if candidate not in self._used_names:
                self._used_names.add(candidate)
                self._base_counters[base] = count
                return candidate
            count += 1
