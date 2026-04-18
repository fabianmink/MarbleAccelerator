
file = openfile("point_output.csv", "w")

z_min = -30
z_max = 30
z_step = 0.1  --must not be zero, even if min and max are identical; otherwise infinite loop

r_min = 0
r_max = 30
r_step = 0.1  --must not be zero, even if min and max are identical; otherwise infinite loop


for r = r_min,r_max,r_step do
for z = z_min,z_max,z_step do
	A, B1, B2, Sig, E, H1, H2, Je, Js, Mu1, Mu2, Pe, Ph = mo_getpointvalues(r,z)
	
	str = r .. ", " .. z .. ", " .. A .. ", " .. B1 .. ", " .. B2
	write(file,str.."\n")
end
print(str)
end

closefile(file)

