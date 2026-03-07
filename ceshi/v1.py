import math

def quadratic(a, b, c):
    discriminant = b*b - 4*a*c  #Δ  
    if discriminant < 0:
        return None  # No real roots
    root1 = (-b + math.sqrt(discriminant)) / (2*a)
    root2 = (-b - math.sqrt(discriminant)) / (2*a)
    return (root1, root2)

d=quadratic(2,3,1)
print(d)  # Output: (-0.5, -1.0)
if d!=(-0.5, -1.0):
    print("错误")
else:
    print("正确")