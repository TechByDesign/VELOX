"""
Data models for the force analysis package.
"""
from enum import Enum
from typing import Optional, List
from mathutils import Vector


class SupportType(Enum):
    FIXED = 'fixed'     # All DOFs constrained (X,Y,Z translations)
    PINNED = 'pinned'   # X,Y translations constrained, Z rotation free
    ROLLER = 'roller'   # Single axis constraint (specified by direction)


class Support:
    """Represents a support constraint in the structural analysis.
    
    Attributes:
        vertex_index: Index of the vertex where support is applied
        support_type: Type of support (FIXED, PINNED, ROLLER)
        direction: For ROLLER supports, the normal vector of the rolling plane
        reaction_forces: Calculated reaction forces at this support
    """
    def __init__(self, vertex_index: int, support_type: SupportType = SupportType.FIXED, 
                 direction: Optional[Vector] = None):
        self.vertex_index = vertex_index
        self.support_type = support_type
        self.direction = direction.normalized() if direction else Vector((0, 0, 1))
        self.reaction_forces = None
    
    def get_constraints(self) -> List[bool]:
        """Returns a list of boolean constraints [x, y, z] where True means constrained."""
        if self.support_type == SupportType.FIXED:
            return [True, True, True]  # All translations constrained
        elif self.support_type == SupportType.PINNED:
            return [True, True, False]  # X,Y constrained, Z free
        elif self.support_type == SupportType.ROLLER:
            # Only constrain movement in the direction normal to the rolling plane
            normal = self.direction
            # This is simplified - in practice, would need to project onto global axes
            return [
                bool(abs(normal.x) > 0.9), 
                bool(abs(normal.y) > 0.9), 
                bool(abs(normal.z) > 0.9)
            ]
        return [False, False, False]
