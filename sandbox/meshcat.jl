import Pkg; 
Pkg.add("MeshCat")
Pkg.add("ColorTypes")
Pkg.add("GeometryTypes")
using MeshCat
vis = Visualizer()
open(vis)

using ColorTypes
using GeometryTypes
verts = 
colors = [RGB(p...) for p in verts]
setobject!(vis, PointCloud(verts, colors))