# -*- coding: utf-8 -*-
from functools import total_ordering
import numpy as np

__all__ = [ 'node' ]


@total_ordering
class node(object):


    def __init__(self, x: int, y: int):

        self.x = x
        self.y = y
        self._delay = np.random.binomial(10, 0.1)

    @property
    def delay(self) -> int:
        return self._delay

    def __lt__(self, other: 'node') :
        return (self.x < other.x) or (self.y < other.y)
    def __repr__(self) -> str:
        return 'node(%r, %r)' % (self.x, self.y)
