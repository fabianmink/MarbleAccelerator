--0
--  Simulation of steel marble in coil (FEMM)
--  
--  Copyright (c) 2025 Fabian Mink <fabian.mink@iem.thm.de>
--  All rights reserved.
--
--  Redistribution and use in source and binary forms, with or without
--  modification, are permitted provided that the following conditions are met:
--
--  1. Redistributions of source code must retain the above copyright notice, this
--    list of conditions and the following disclaimer.
--  2. Redistributions in binary form must reproduce the above copyright notice,
--     this list of conditions and the following disclaimer in the documentation
--     and/or other materials provided with the distribution.
--
--  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
--  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
--  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
--  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
--  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
--  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
--  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
--  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
--  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--

--coil current
Icoil = 20 

--marble z position
zm = -15  

--Geometry (for 0.5mm Wire diameter)
l_coil = 15   --coil length
ri = 8.4 --coil inner radius
ro = 16 --coil outer radius
Ncoil = 180 --number of turns per coil 

rm = 12.7/2  --marble radius

rbound = 150   --radius outer boundary

am = 1 --Automesh on
ms = 10 --Mesh size (optional, only if automesh off)


newdocument(0)
mi_probdef(0,"millimeters","axi",1e-008)

--construct coil
mi_addnode(ri,l_coil/2)
mi_addnode(ri,-l_coil/2)
mi_addnode(ro,l_coil/2)
mi_addnode(ro,-l_coil/2)
mi_addsegment(ri,-l_coil/2,ri,l_coil/2)
mi_addsegment(ri,l_coil/2,ro,l_coil/2)
mi_addsegment(ro,-l_coil/2,ro,l_coil/2)
mi_addsegment(ri,-l_coil/2,ro,-l_coil/2)


--construct marble
mi_addnode(0,zm+rm)	
mi_addnode(0,zm-rm)
mi_addarc(0,(zm-rm),0,(zm+rm),180,1)


--boundary
mi_addnode(0,rbound)
mi_addnode(0,-rbound)
mi_addsegment(0,-rbound,0,rbound)
mi_addarc(0,-rbound,0,rbound,180,1)

--Load and select materials
--mi_addcircprop('circuit', Icoil, 1) 

mi_getmaterial('Air')	
mi_getmaterial('Copper')
mi_getmaterial('Steel castings, as cast');

-- coil 
mi_addblocklabel((ri+ro)/2,0)	
mi_selectlabel((ri+ro)/2,0)	
mi_setblockprop('Copper',am,ms,'circuit',90,0,Ncoil)    
mi_clearselected()

--Marble
mi_addblocklabel(1,zm)	
mi_selectlabel(1,zm)
mi_setblockprop('Steel castings, as cast',am,ms,'',0,0,0)    
mi_clearselected()

--Air
mi_addblocklabel(ro+1,0)
mi_selectlabel(ro+1,0)
mi_setblockprop('Air',am,ms,'',0,0,0)
mi_clearselected()

--boundary
mi_selectarcsegment(1,rbound)
mi_addboundprop("a0", 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0)
mi_setarcsegmentprop(1, "a0", 0, 0)
mi_clearselected()

mi_saveas("marble_coil.fem")



mi_addcircprop('circuit', Icoil, 1) -- circuit, series connection

mi_analyze(0)
mi_loadsolution()
	

mo_selectblock(1,zm)
Fr = mo_blockintegral(18)
Fz = mo_blockintegral(19)
mo_clearblock()
	
mo_groupselectblock()  --select everything
Emag = mo_blockintegral(2)
Eco = mo_blockintegral(17)
mo_clearblock()

str = Icoil .. "A, ".. zm .. "mm, " .. Fr .. "N, " .. Fz .. "N, " .. Eco .. "Ws, " .. Emag .. "Ws"
print(str)

--mo_showdensityplot(1,0,1.0,0.0,"bmag")
--mo_zoomnatural()
--mo_zoom(0,-40,50,40)
--mo_savebitmap("outfile".. istep .. ".bmp")


--mi_close()  --close preprocessor
--mo_close() --close postprocessor


