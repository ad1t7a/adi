import Pkg; 
Pkg.add("Plots")
Pkg.add("PlotlyJS")
using Plots
using PlotlyJS

x = -3:0.1:3
f3 = x -> x^2
y = f3.(x)
#gr() backend
plotlyjs()
plot(x, y, label = "line")
scatter!(x, y, label="scatter")


#getting fancier
globaltemperatues = [14.4, 14.5, 14.8, 15.2, 15.5, 15.8]
numPirates = [50000, 30000, 1000, 500, 200, 50]
plot(numPirates, globaltemperatues, legend =false)
scatter!(numPirates, globaltemperatues, legend =false)
xflip!()

xlabel!("Number of pirates")
ylabel!("Temperature")
title!("Influence of the pirate population on temperatures")
p1 = plot(x,x)
p2 = plot(x, x.^2)
p3 = plot(x, x.^3)
p4 = plot(x, x.^4)
plot(p1, p2, p3, p4, layout=(2,2), legend=false)
