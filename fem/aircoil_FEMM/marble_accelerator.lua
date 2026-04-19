--
--   Copyright (c) 2026 Fabian Mink <fabian.mink@iem.thm.de>
--
--   All rights reserved.
--
--   Redistribution and use in source and binary forms, with or without
--   modification, are permitted provided that the following conditions are met:
--
--   1. Redistributions of source code must retain the above copyright notice, this
--      list of conditions and the following disclaimer.
--   2. Redistributions in binary form must reproduce the above copyright notice,
--      this list of conditions and the following disclaimer in the documentation
--      and/or other materials provided with the distribution.
--
--   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
--   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
--   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
--   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
--   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
--   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
--   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
--   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
--   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


zm = -15

--Geometry (for 0.5mm Wire diameter)
l_coil = 15   --coil length
ri = 8.4 --coil inner radius
ro = 16 --coil outer radius
Ncoil = 180 --number of turns per coil 

function coil_create(ri, ro, l, z, material, circuit, turns)
	am = 1 --Automesh on
	ms = 10 --Mesh size (if automesh off)
	
	--construct coil
	mi_addnode(ri,l/2+z)
	mi_addnode(ri,-l/2+z)
	mi_addnode(ro,l/2+z)
	mi_addnode(ro,-l/2+z)
	mi_addsegment(ri,-l/2+z,ri,l/2+z)
	mi_addsegment(ri,l/2+z,ro,l/2+z)
	mi_addsegment(ro,-l/2+z,ro,l/2+z)
	mi_addsegment(ri,-l/2+z,ro,-l/2+z)
	
	mi_addcircprop(circuit, 0, 1)
	mi_addblocklabel((ri+ro)/2,z)
	mi_selectlabel((ri+ro)/2,z)
	mi_setblockprop(material,am,ms,circuit,90,0,turns)
	mi_clearselected()
end


function marble_create(r, z, material, groupno)
	am = 1 --Automesh on
	ms = 10 --Mesh size (if automesh off)
	
	--construct marble
	mi_addnode(0,z+rm)	
	mi_addnode(0,z-rm)
	mi_addarc(0,(z-rm),0,(z+rm),180,1)

	--Marble
	mi_addblocklabel(r/2,z)
	mi_selectlabel(r/2,z)
	mi_setblockprop(material,am,ms,'',0,groupno,0)
	mi_clearselected()
end

function extract_point_values(r_max, r_step, z_max, z_step, filename)

	file = openfile(filename, "w")

	z_min = -z_max

	r_min = 0

	for z = z_min,z_max,z_step do
	for r = r_min,r_max,r_step do
		A, B1, B2, Sig, E, H1, H2, Je, Js, Mu1, Mu2, Pe, Ph = mo_getpointvalues(r,z)
		
		str = r .. ", " .. z .. ", " .. A .. ", " .. B1 .. ", " .. B2
		write(file,str.."\n")
	end
	--print(str)
	end

	closefile(file)

end
