import math


class FacePointer:
    def __init__(self, ID, x, y, w, h):
        self.ID = ID
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def distance_from_face(self, FacePointer):
        return math.sqrt( math.pow((FacePointer.x - self.x), 2) +
                   math.pow((FacePointer.y - self.y), 2)
        )


