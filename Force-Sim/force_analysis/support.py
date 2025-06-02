"""
Support system for force analysis.

This module defines the support types and support management functionality.
"""

from enum import Enum
from typing import Dict, Optional, List, Any
from mathutils import Vector
import threading

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
        if not isinstance(vertex_index, int) or vertex_index < 0:
            raise ValueError("vertex_index must be a non-negative integer")
        if not isinstance(support_type, SupportType):
            raise TypeError("support_type must be a SupportType enum")
            
        self.vertex_index = vertex_index
        self.support_type = support_type
        self.direction = direction.normalized() if direction else Vector((0, 0, 1))
        self.reaction_forces: Optional[Dict[str, float]] = None
    
    def get_constraints(self) -> List[bool]:
        """Returns a list of boolean constraints [x, y, z] where True means constrained."""
        try:
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
        except Exception as e:
            print(f"Error in get_constraints: {e}")
            return [False, False, False]

# Thread-local storage for supports to support multiple view layers/scenes
class SupportManager:
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(SupportManager, cls).__new__(cls)
                cls._instance._supports: Dict[int, Support] = {}
                cls._instance._lock = threading.RLock()
        return cls._instance
    
    def add_support(self, vertex_index: int, support_type: SupportType = SupportType.FIXED, 
                   direction: Optional[Vector] = None) -> bool:
        """Add or update a support at the given vertex."""
        if not isinstance(vertex_index, int) or vertex_index < 0:
            return False
            
        with self._lock:
            try:
                self._supports[vertex_index] = Support(vertex_index, support_type, direction)
                return True
            except (ValueError, TypeError) as e:
                print(f"Error adding support: {e}")
                return False
    
    def remove_support(self, vertex_index: int) -> bool:
        """Remove support from the given vertex."""
        with self._lock:
            try:
                if vertex_index in self._supports:
                    del self._supports[vertex_index]
                    return True
                return False
            except Exception as e:
                print(f"Error removing support: {e}")
                return False
    
    def clear_supports(self) -> None:
        """Remove all supports."""
        with self._lock:
            self._supports.clear()
    
    def get_supports(self) -> List[Support]:
        """Get a list of all supports."""
        with self._lock:
            return list(self._supports.values())
    
    def get_support(self, vertex_index: int) -> Optional[Support]:
        """Get the support at the given vertex, or None if none exists."""
        with self._lock:
            return self._supports.get(vertex_index)
    
    def has_support(self, vertex_index: int) -> bool:
        """Check if a support exists at the given vertex."""
        with self._lock:
            return vertex_index in self._supports

# Global instance for backward compatibility
_support_manager = SupportManager()

# Public API
add_support = _support_manager.add_support
remove_support = _support_manager.remove_support
clear_supports = _support_manager.clear_supports
get_supports = _support_manager.get_supports
get_support = _support_manager.get_support
has_support = _support_manager.has_support

# For backward compatibility
supports = {}

def _sync_supports():
    """Sync the global supports dict with the support manager (for backward compatibility)."""
    global supports
    supports = {s.vertex_index: s for s in get_supports()}

# Initialize the global supports dict
_sync_supports()
