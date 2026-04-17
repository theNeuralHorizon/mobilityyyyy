"""Pure-function edge-sample decision logic.

Given green/orange counts per edge, the entry edge, and a LiDAR wall map,
pick an exit. No ROS dependency — unit-testable.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

EDGES = ('N', 'S', 'E', 'W')
OPPOSITE = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}


@dataclass
class EdgeDecision:
    chosen: str
    reason: str
    confidence: float
    alternatives: list[str]


def decide_exit(
    counts: Dict[str, int],
    entry_edge: Optional[str],
    wall_blocked: Dict[str, bool] | None = None,
    min_pixels: int = 50,
    near_tie_ratio: float = 0.15,
) -> EdgeDecision:
    """Apply the green-follow / orange-follow rules.

    Rules:
      1. Exclude the entry edge from candidates.
      2. Exclude edges where LiDAR reports a wall within ~0.45 m (wall_blocked).
      3. Rank remaining edges by pixel count descending.
      4. If top-two differ by < near_tie_ratio and the secondary is the edge
         orthogonal to entry (i.e., the straight-ahead continuation), pick
         the secondary — this handles corner tiles where the "turn" edge
         narrowly wins the pixel count but straight is intended.
         Actually the opposite: corner tiles have green on two adjacent
         edges; entering along one of them means the continuation is a
         turn, so pick the NOT-collinear edge. Implemented as: on near-tie,
         prefer the edge orthogonal to entry (turn) over collinear (straight).
      5. Below min_pixels on all candidates → return chosen='' (handler falls
         back to SEEK or STOP_AND_ROTATE).
    """
    wall_blocked = wall_blocked or {e: False for e in EDGES}

    candidates = [e for e in EDGES
                  if e != entry_edge and not wall_blocked.get(e, False)]
    if not candidates:
        return EdgeDecision(chosen='', reason='no_candidates', confidence=0.0, alternatives=[])

    ranked = sorted(candidates, key=lambda e: counts.get(e, 0), reverse=True)
    top = ranked[0]
    top_n = counts.get(top, 0)

    if top_n < min_pixels:
        return EdgeDecision(chosen='', reason='no_green_above_threshold',
                            confidence=0.0, alternatives=ranked)

    if len(ranked) == 1:
        return EdgeDecision(chosen=top, reason='single_candidate',
                            confidence=1.0, alternatives=[])

    second = ranked[1]
    second_n = counts.get(second, 0)
    spread = (top_n - second_n) / max(top_n, 1)

    if spread < near_tie_ratio and entry_edge is not None:
        # Near-tie: prefer orthogonal to entry (turn) over collinear (straight)
        collinear = OPPOSITE.get(entry_edge)
        if top == collinear and second != collinear:
            return EdgeDecision(chosen=second, reason='near_tie_prefer_turn',
                                confidence=0.6, alternatives=[top])
        # otherwise fall through to top

    return EdgeDecision(chosen=top,
                        reason='top_by_pixel_count',
                        confidence=min(1.0, 0.5 + spread),
                        alternatives=ranked[1:])
