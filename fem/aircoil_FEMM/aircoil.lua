Icoil = 10 
Ncoil = 500

ky = 0
kr = 6

rbound = 75

rx = 8
d = 4
h = 20

am = 1 --Automesh on
ms = 10 --Mesh size (optional, only if automesh off)

file = openfile("output.csv", "w")

istep = 100
for ky=-12,2,0.5 do 


newdocument(0)
mi_probdef(0,"millimeters","axi",1e-008)


--Kugel konstruieren
mi_addnode(0,ky+kr)	
mi_addnode(0,ky-kr)
mi_addarc(0,(ky-kr),0,(ky+kr),180,1)

	
--Spulen konstruieren
mi_addnode(rx,-h/2)
mi_addnode(rx,h/2)
mi_addnode(rx+d,-h/2)
mi_addnode(rx+d,h/2)


mi_addsegment(rx,-h/2,rx,h/2)
mi_addsegment(rx,h/2,rx+d,h/2)
mi_addsegment(rx+d,h/2,rx+d,-h/2)
mi_addsegment(rx+d,-h/2,rx,-h/2)


--Punkte Begrenzung zu berechnender Bereich
mi_addnode(0,rbound)
mi_addnode(0,-rbound)
mi_addsegment(0,-rbound,0,rbound)
mi_addarc(0,-rbound,0,rbound,180,1)

--Materialien laden und zuweisen
mi_addcircprop('circuit', Icoil, 1) --Stromkreis: I A, in Serie

--Materialien einlesen
mi_getmaterial('Air')	
mi_getmaterial('Copper')
mi_getmaterial('Steel castings, as cast');

-- Spule
mi_addblocklabel(rx+d/2,0)	
mi_selectlabel(rx+d/2,0)	
mi_setblockprop('Copper',am,ms,'circuit',90,0,Ncoil)    
mi_clearselected()

--Kugel
mi_addblocklabel(1,ky)	
mi_selectlabel(1,ky)
mi_setblockprop('Steel castings, as cast',am,ms,'',0,0,0)    
mi_clearselected()

--Luft
mi_addblocklabel(rx+d+1,0)
mi_selectlabel(rx+d+1,0)
mi_setblockprop('Air',am,ms,'',0,0,0)
mi_clearselected()

--boundary
mi_selectarcsegment(1,rbound)
mi_addboundprop("a0", 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0)
mi_setarcsegmentprop(1, "a0", 0, 0)
mi_clearselected()

mi_saveas("aircoil.fem")

mi_zoomnatural()
--mi_createmesh() --works, but pops-up window which has to be closed manually


for Icoil=0,10,2 do 
	--Strom nochmal laden / aendern
	mi_addcircprop('circuit', Icoil, 1) --Stromkreis: I A, in Serie

	mi_analyze(0)
	mi_loadsolution()
	

	mo_selectblock(1,ky)
	Fx = mo_blockintegral(18)
	print(Fx) 
	Fy = mo_blockintegral(19)
	mo_clearblock()
	print(Fy)
	--print("y-Force on anchor: Fy/N = " .. Fy)
	mo_groupselectblock()  --select everything

	--Emag = mo_blockintegral(2)
	Eco = mo_blockintegral(17)
	mo_clearblock()

	str = Icoil .. ", ".. ky .. ", " .. Fx .. ", " .. Fy .. ", " .. Eco
	--print(str)
	write(file,str.."\n")

	mo_showdensityplot(1,0,1.0,0.0,"bmag")
	--mo_zoomnatural()
	mo_zoom(0,-40,50,40)

	mo_savebitmap("outfile".. istep .. ".bmp")
	istep = istep + 1;
end


mi_close()  --close preprocessor
mo_close() --close postprocessor

end

closefile(file)