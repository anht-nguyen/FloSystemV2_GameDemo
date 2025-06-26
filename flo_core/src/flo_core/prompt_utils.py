# flo_core/prompt_utils.py
"""
Generate human‑friendly verbal prompts for the dual‑arm Simon‑Says game.

Inputs are *gesture codes* (e.g. "D_WAVE") for the robot’s left and
right arms plus the boolean flag `simon_says`.
The function returns a full sentence, e.g.::

    >>> build_prompt("S_RAISE", "D_SWING_LATERAL", True)
    'Simon says raise your right arm and swing your left arm sideways.'
"""

from typing import Dict

# ---------------------------------------------------------------------------
# 1.  Gesture → natural‑language template
# ---------------------------------------------------------------------------
# Each value is a format string that will receive the keyword ``side`` which
# will be substituted with "left", "right" or "both".
# Keep everything lower‑case; we capitalise the final sentence afterwards.
_VERB_TEMPLATES: Dict[str, str] = {
    "WAVE":          "wave your {side} arm",
    "SWING_LATERAL": "swing your {side} arm sideways",
    "RAISE":         "raise your {side} arm",
    "REACH_SIDE":    "reach out to your {side} side with your {side} arm",
    "TOUCH_MOUTH":   "touch your mouth with your {side} hand",
    "TOUCH_HEAD":    "touch your head with your {side} hand",
}


def _strip_prefix(gesture_code: str) -> str:
    """Drop the D_/S_ prefix and return the core gesture key."""
    return gesture_code.replace("D_", "").replace("S_", "")


def _clause(gesture_code: str, player_side: str) -> str:
    """Return an English clause for a single arm."""
    key = _strip_prefix(gesture_code)
    template = _VERB_TEMPLATES.get(key)
    if template:
        return template.format(side=player_side)
    # Fallback: simple generic wording
    return f"{key.lower()} with your {player_side} arm"


# ---------------------------------------------------------------------------
# 2.  Public helper
# ---------------------------------------------------------------------------

def build_prompt(left_action: str, right_action: str, simon_says: bool) -> str:
    """Compose the full verbal instruction for the player.

    Parameters
    ----------
    left_action  : str
        Gesture code executed by the **robot's LEFT** arm.
    right_action : str
        Gesture code executed by the **robot's RIGHT** arm.
    simon_says   : bool
        Whether to prefix the sentence with "Simon says".

    Returns
    -------
    str
        A natural‑language sentence, capitalised and punctuated.
    """
    # Normalise empty fields to the empty string
    left_action = left_action or ""
    right_action = right_action or ""

    clauses = []

    # Case 1: same gesture on both arms
    if left_action and right_action and left_action == right_action:
        key = _strip_prefix(left_action)
        template = _VERB_TEMPLATES.get(key, "{verb} with both arms")
        if "{side}" in template:
            clause = template.format(side="both")
        else:
            clause = template.replace("{verb}", key.lower())
        clauses.append(clause)

    # Case 2: independent gestures
    else:
        if left_action:
            clauses.append(_clause(left_action, "right"))   # mirror robot→player
        if right_action:
            clauses.append(_clause(right_action, "left"))

    # Default if no actions were supplied
    if not clauses:
        body = "nothing to do"
    else:
        body = " and ".join(clauses)

    prefix = "Simon says " if simon_says else ""
    sentence = f"{prefix}{body}"

    # Capitalise first letter and ensure a period.
    return sentence[0].upper() + sentence[1:] + "."
