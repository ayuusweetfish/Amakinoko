from math import sin, cos, pi
r = 13.5
for i in range(0, 11):
  a = -30 - (120 / 11.) * i
  a = a * pi / 180
  print(r * cos(a), r * sin(a))
