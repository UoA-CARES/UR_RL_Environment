import math3d

print(math3d.__version__)

vector1 = math3d.Vector(1, 2, 3)
vector2 = math3d.Vector(4, 5, 6)

distance_squared = vector1.dist_squared(vector2)
print(distance_squared)
