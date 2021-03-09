
n = 0
while n < 10
    n += 1
    println(n)
end 

myfavoriteanumals = ("penguins", "cats", "sugargliders")
k = 1
while k<=length(myfavoriteanumals)
    println(myfavoriteanumals[k])
    k += 1
end

# for loop
for x in 1:10
    println(x)
end

# for loop
for x in myfavoriteanumals
    println(x)
end

# for loop ϵ
for x = myfavoriteanumals
    println(x)
end

#=
Multiple for loops example
=#
m , n = 5, 5
X = zeros(m,n)
for i in 1:m
    for j in 1:n
        X[i, j] = i + j
    end 
end

Y = zeros(m, n)
m , n = 5, 5
for i in 1:m, j in 1:n
    Y[i, j] = i + j
end

m , n = 5, 5
C = [i + j for i in 1:m, j in 1:n]