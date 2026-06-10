from __future__ import annotations

from PyQt6.QtWidgets import QLabel


def simplify_status_text(text: str) -> str:
    normalized = str(text or "").strip().lower()
    if not normalized:
        return "Non chargée"
    if "modification" in normalized or "modifi" in normalized:
        return "Modifiée"
    if "non enregistr" in normalized:
        return "Non enregistrée"
    if "non charg" in normalized or "aucun" in normalized:
        return "Non chargée"
    if "à jour" in normalized or "a jour" in normalized:
        return "À jour"
    if "enregistr" in normalized:
        return "Enregistrée"
    if "charg" in normalized:
        return "Chargée"
    return str(text).strip()


def apply_status_badge(label: QLabel, text: str, color: str) -> None:
    badge_text = simplify_status_text(text)
    background = str(color or "#808080").strip() or "#808080"
    foreground = _foreground_for_background(background)
    label.setText(badge_text)
    label.setStyleSheet(
        "QLabel {"
        f"background-color: {background};"
        f"color: {foreground};"
        "font-size: 12px;"
        "font-weight: 600;"
        "padding: 2px 8px;"
        "border-radius: 6px;"
        "}"
    )


def _foreground_for_background(color: str) -> str:
    normalized = color.strip().lstrip("#")
    if len(normalized) != 6:
        return "#ffffff"
    try:
        red = int(normalized[0:2], 16)
        green = int(normalized[2:4], 16)
        blue = int(normalized[4:6], 16)
    except ValueError:
        return "#ffffff"
    luminance = (0.299 * red + 0.587 * green + 0.114 * blue) / 255.0
    return "#1f1f1f" if luminance > 0.62 else "#ffffff"
