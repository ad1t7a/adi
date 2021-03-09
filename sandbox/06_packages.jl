#Pkg.add("Example")
#using Example

hello(who::String) = "Hello $who"

import Pkg; 
Pkg.add("Colors")
using Colors
palette = distinguishable_colors(100)
rand(palette, 3, 3)