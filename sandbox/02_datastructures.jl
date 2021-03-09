# Dictionaries
telephonebook = Dict("Adi" => "911-Emergency", "ghostbusters" =>"555-SCREAM")
telephonebook["Kramer"] = "123-GIDDYUP"
println(telephonebook)
println(telephonebook["Adi"])
pop!(telephonebook, "ghostbusters")
# tuples (ordered and immutable)
myfavoriteanumals = ("penguins", "cats", "sugargliders")
myfavoriteanumals[1]
# arrays (ordered)
myFriends = ["TED", "JOHN", "JULIA"]
fibonacci = [1, 1, 2,3,5, 8, 13]
mixedType = [1, 1, 2,3,"Hi"]
myFriends[2] ="Potter"
push!(fibonacci, 21)
pop!(fibonacci)
rand(4,3)
# Can create array of array
nums = [[1,2,3], [2,3],[4,6,7]]
