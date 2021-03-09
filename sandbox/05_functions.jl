function sayi(name)
    println("Hi $name !Nice to meet you")
end
sayi("Adi")

function f(x)
    x^2
end

f(2)

# function
sayhi2(name) = println("Hi $name !Nice to meet you")
sayhi2("Adi")

# lamda function

f3 = x -> x^3
f3(3)


#Mutating! vs immutating functions
a = [3,2,1,4]
sort(a)
a

sort!(a)
a

# Broadcasting vs not Broadcasting (Broadcasting is applied to individual elements)
A = rand(3,3)
f(A)
f.(A)