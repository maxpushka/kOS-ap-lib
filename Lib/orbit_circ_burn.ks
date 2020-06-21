clearscreen.

print "Counting down".	
FROM {local countdown is 3.} UNTIL countdown=0 STEP {set countdown to countdown - 1.} DO {
	print "..." + countdown.
	wait 1.
}
print "LIFTOFF!".
lock throttle to 1.
lock steering to up.
stage.
wait 5.

until (stage:solidfuel=128) {
	clearscreen.
	print "SolidFuel: " + stage:solidfuel.
	wait 0.1.
}
lock steering to prograde.
wait 5.
stage.

until ETA:apoapsis < 1 {
	clearscreen.
	print "ETA:apoapsis = " + ETA:apoapsis.
	wait 0.25.
}
print "Start burn".
wait 1.
//========================== BURN ==========================
set stopburn to false.
lock throttle to 1.
until stopburn {
	clearscreen.
	set data to burndata.
	
	if (data[5]<0) {
		set stopburn to true.
	}
	else if (data[5] < 100) {
		lock throttle to Max(data[5]/100, 0.01).
	}
	
	lock steering to Heading(90, data[0]).
	
	print "Fi: " + data[0].
	print "dA: " + data[1].
	print "Vh: " + data[2].
	print "Vz: " + data[3].
	print "Vorb: " + data[4].
	print "dVorb: " + data[5].
}
lock throttle to 0.
clearscreen.
print "Circularization complete".
orbitData.

function burndata {
	set eng to englist.

	set Rad to ship:body:radius + ship:altitude.
	set g_alt to body:Mu/Rad^2. //ускорение свободного падения на текущей высоте
	set Vh to VXCL(ship:up:vector, ship:velocity:orbit):mag. //горизонтальная скорость
	set Vz to ship:verticalspeed. //вертикальная скорость
	set Vorb to sqrt(ship:body:Mu/Rad). //1 косм. на текущей высоте
	set Acentr to Vh^2/Rad. //центростремительное ускорение
	set AThr to eng[0]*Throttle/ship:mass.
	
	set dVorb to Vorb-Vh.
	set dA to g_alt-Acentr-Vz. //MAX(Min(Vz, 2), -2).
	print "Athr=" + Athr.
	print "dA=" + dA.
	set fi to ARCSIN(Min(Max(dA/AThr,-1), 1)).
	
	print "g_alt=" + g_alt.
	print "Acentr=" + Acentr.
	print "englist flag=" + eng[2].
	print "AThr=" + AThr.
	
	return list(fi, dA, Vh, Vz, Vorb, dVorb).
}
	
function englist {
	list engines in allEngines.
	
	set ActiveEng to list().
	ActiveEng:clear.
	
	set eng_isp to 0.
	set eng_thrust to 0.
	
	for eng in allEngines {
		if eng:ignition and (not eng:flameout) {
			ActiveEng:add(eng).
		}
	}
	
	for eng in ActiveEng {
		set eng_thrust to eng_thrust + eng:availablethrust.
		set eng_isp to eng_isp + eng:isp.
	}
	
	if (ActiveEng:length=0) {return list(0,0,false).}
	else {return list(eng_thrust, eng_isp/ActiveEng:length, true).}
}

function orbitData {
	print "".
	print "ORBIT PARAMETERS".
	print "================".
	print "Apoapsis: " + ORBIT:APOAPSIS.
	print "Periapsis: " + ORBIT:PERIAPSIS.
	print "Eccentricity: " + ORBIT:ECCENTRICITY.
	print "Inclination: " + ORBIT:INCLINATION.
	print "================".
}
