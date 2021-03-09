A = rand(1:4, 3, 3)
B = A
C = copy(A)
[ B C]
A[1] = 17
[ B C]


x = ones(3)
A*x

Asym = A + A'
Apd = A'A

b=ones(3)
# Tall matrix or fat least squares result is returned or minimum norm solution

A\b