a = rand(1)[1]
if a < 4
    println("less than 4")
elseif a>5
    println("Greater than 5")
else
    println("Between 4 and 5")
end

(4>5) ? 4 : 5

(4>5) && println("4 is greater than 5")