import math

class Vector2D:
    def __init__(self, y, z):
        self.y = y
        self.z = z

    # Overload operators
    def __repr__(self):
        return self

    def __str__(self):
        return "Vector" + str((self.y, self.z))

    def __add__(self, other):
        return Vector2D(self.y + other.y, self.z + other.z)

    def __iadd__(self, other):
        self.y += other.y
        self.z += other.z
        return self

    def __sub__(self, other):
        return Vector2D(self.y - other.y, self.z - other.z)

    def __isub__(self, other):
        self.y -= other.y
        self.z -= other.z
        return self

    def __mul__(self, other):
        return Vector2D(self.y * other, self.z * other)

    def __truediv__(self, other):
        if other != 0:
            return Vector2D(self.y / other, self.z / other)
        else:
            return self

    def __neg__(self):
        return Vector2D(-self.y, -self.z)

    def __lt__(self, other):
        return self.length() < other.length()

    def __gt__(self, other):
        return self.length() > other.length()

    def get_angle(self):
        result = math.asin(self.sin())
        if(self.cos() < 0):
            result = math.pi - result
        return result

    def set_angle(self, angle):
        length = self.length()

        self.y = length * math.cos(angle)
        self.z = length * math.sin(angle)

    def length(self):
        return math.hypot(self.y , self.z)
    
    def normalized(self):
        return self/self.length()

    def sin(self):
        if self.length() == 0:
            return 0

        return self.z / self.length()

    def cos(self):
        if self.length() == 0:
            return 0

        return self.y / self.length()

    # Static functions
    @staticmethod
    def minimal(vector1, vector2):
        if vector1 < vector2:
            return vector1
        else:
            return vector2

    @staticmethod
    def maximal(vector1, vector2):
        if vector1 > vector2:
            return vector1
        else:
            return vector2

    @staticmethod
    def distance(vector1, vector2):
        return (vector2-vector2).length()

    @staticmethod
    def perpendicular(vector):
        return Vector2D(-vector.sin(), vector.cos())

    @staticmethod
    def list(vector):
        return [vector.y, vector.z]