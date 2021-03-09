methods(+)
@which 3+3

import Base: +
+(x::String, y::String) = string(x, y)
"Hello " + "World"

foo(x, y) = println("Duck typed function")
foo(x::Int, y::Int) = println("Foo with two ints")
foo(x::Float64, y::Int) = println("Foo with int and float")
foo(x::Float64, y::Float64) = println("Foo with two float")
@which foo(1, 1)
foo(1, true)
