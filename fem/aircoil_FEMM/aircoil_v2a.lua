--this simulation is for 3D-printed coil carrier, v2a

Icoil_a = 20 --coil a current
Icoil_b = 0  --coil b current 
zm = -50       --marble z position totally out of coils
--zm = -10.5       --marble z position middle of coil a

l_coil = 15   --coil length
l_sep = 6 --seperator length - distance between 2 coils
ri = 8 --coil inner diameter
ro = 12 --coil outer diameter
Ncoil = 50 --number of turns per coil 

rm = 12.7/2  --marble radius

--rbound = 75
rbound = 150   --radius outer boundary

am = 1 --Automesh on
ms = 10 --Mesh size (optional, only if automesh off)

file = openfile("output.csv", "w")

istep = 100
--for kz=-20,0,0.25 do

newdocument(0)
mi_probdef(0,"millimeters","axi",1e-008)

--construct coil a
mi_addnode(ri,-l_sep/2)
mi_addnode(ri,-l_sep/2-l_coil)
mi_addnode(ro,-l_sep/2-l_coil)
mi_addnode(ro,-l_sep/2)
mi_addsegment(ri,-l_sep/2,ri,-l_sep/2-l_coil)
mi_addsegment(ri,-l_sep/2-l_coil,ro,-l_sep/2-l_coil)
mi_addsegment(ro,-l_sep/2-l_coil,ro,-l_sep/2)
mi_addsegment(ro,-l_sep/2,ri,-l_sep/2)

--construct coil b
mi_addnode(ri,l_sep/2)
mi_addnode(ri,l_sep/2+l_coil)
mi_addnode(ro,l_sep/2+l_coil)
mi_addnode(ro,l_sep/2)
mi_addsegment(ri,l_sep/2,ri,l_sep/2+l_coil)
mi_addsegment(ri,l_sep/2+l_coil,ro,l_sep/2+l_coil)
mi_addsegment(ro,l_sep/2+l_coil,ro,l_sep/2)
mi_addsegment(ro,l_sep/2,ri,l_sep/2)


--Kugel konstruieren
mi_addnode(0,zm+rm)	
mi_addnode(0,zm-rm)
mi_addarc(0,(zm-rm),0,(zm+rm),180,1)


--Punkte Begrenzung zu berechnender Bereich
mi_addnode(0,rbound)
mi_addnode(0,-rbound)
mi_addsegment(0,-rbound,0,rbound)
mi_addarc(0,-rbound,0,rbound,180,1)

--Materialien laden und zuweisen
--mi_addcircprop('circuit', Icoil, 1) --Stromkreis: I A, in Serie  --unten!!

--Materialien einlesen
mi_getmaterial('Air')	
mi_getmaterial('Copper')
mi_getmaterial('Steel castings, as cast');

-- coil a
mi_addblocklabel((ri+ro)/2,-(l_sep+l_coil)/2)	
mi_selectlabel((ri+ro)/2,-(l_sep+l_coil)/2)	
mi_setblockprop('Copper',am,ms,'circuit_a',90,0,Ncoil)    
mi_clearselected()

-- coil b
mi_addblocklabel((ri+ro)/2,(l_sep+l_coil)/2)	
mi_selectlabel((ri+ro)/2,(l_sep+l_coil)/2)	
mi_setblockprop('Copper',am,ms,'circuit_b',90,0,Ncoil)    
mi_clearselected()

--Kugel
mi_addblocklabel(1,zm)	
mi_selectlabel(1,zm)
mi_setblockprop('Steel castings, as cast',am,ms,'',0,0,0)    
mi_clearselected()

--Luft
mi_addblocklabel(ro+1,0)
mi_selectlabel(ro+1,0)
mi_setblockprop('Air',am,ms,'',0,0,0)
mi_clearselected()

--boundary
mi_selectarcsegment(1,rbound)
mi_addboundprop("a0", 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0)
mi_setarcsegmentprop(1, "a0", 0, 0)
mi_clearselected()

mi_saveas("aircoil.fem")

--mi_zoomnatural()

for Icoil_a=0,30,1 do 
	mi_addcircprop('circuit_a', Icoil_a, 1) --Stromkreis: I A, in Serie
	mi_addcircprop('circuit_b', Icoil_b, 1) --Stromkreis: I A, in Serie
	

	mi_analyze(0)
	mi_loadsolution()
	

	mo_selectblock(1,zm)
	Fr = mo_blockintegral(18)
	--print(Fr) 
	Fz = mo_blockintegral(19)
	mo_clearblock()
	--print(Fz)
	
	mo_groupselectblock()  --select everything

	Emag = mo_blockintegral(2)
	Eco = mo_blockintegral(17)
	mo_clearblock()

	str = Icoil_a .. ", " .. Icoil_b .. ", ".. zm .. ", " .. Fr .. ", " .. Fz .. ", " .. Eco .. ", " .. Emag
	--print(str)
	write(file,str.."\n")

	--mo_showdensityplot(1,0,1.0,0.0,"bmag")
	--mo_zoomnatural()
	--mo_zoom(0,-40,50,40)
	--mo_savebitmap("outfile".. istep .. ".bmp")
	istep = istep + 1;
end


--mi_close()  --close preprocessor
--mo_close() --close postprocessor

--end

closefile(file)