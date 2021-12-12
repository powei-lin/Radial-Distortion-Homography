import sympy
from sympy.core.symbol import symbols
from sympy import pprint, collect, expand
# w_p, l_p,  = sympy.symbols('w_p, l_p')
h11 = sympy.symbols('h11')
h12 = sympy.symbols('h12')
h13 = sympy.symbols('h13')
h21 = sympy.symbols('h21')
h22 = sympy.symbols('h22')
h23 = sympy.symbols('h23')
h31 = sympy.symbols('h31')
h32 = sympy.symbols('h32')
h33 = sympy.symbols('h33')
x = symbols('x')
y = symbols('y')
l = symbols('l')
w = 1+l*(x*x+y*y)
x_p = symbols('x_p')
y_p = symbols('y_p')
l_p = symbols('l_p')
w_p = 1+l_p*(x_p*x_p+y_p*y_p)

lhs = (w_p*h11 - x_p*h31)*x + (w_p*h12 - x_p*h32)*y + (w_p*h13 - x_p*h33)*w
expand_lhs = expand(lhs)

# pprint(expand_lhs)
l_p_p = collect(expand_lhs, l_p, evaluate=False)[l_p]
print(l_p_p)
expand_lhs -= l_p_p*l_p
expand_lhs = expand(expand_lhs)

h31_p = collect(expand_lhs, h31, evaluate=False)[h31]
print(h31_p)
expand_lhs -= h31_p*h31
expand_lhs = expand(expand_lhs)

h32_p = collect(expand_lhs, h32, evaluate=False)[h32]
print(h32_p)
expand_lhs -= h32_p*h32
expand_lhs = expand(expand_lhs)

h33_p = collect(expand_lhs, h33, evaluate=False)[h33]
print(h33_p)
expand_lhs -= h33_p*h33
expand_lhs = expand(expand_lhs)
print(-expand_lhs)
# expand_lhs -= collect(expand_lhs, l_p, evaluate=False)[l_p]*l_p
# print(lhs[h31])
# eq0 = sympy.Eq(lhs, 0)
# pexpr = sympy.solve(eq0, h31, h32)
# pprint(pexpr)