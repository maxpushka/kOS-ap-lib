set targetIncl to 2.8.

clearscreen.
rcs on.
sas off.
SET x TO VECDRAW(V(0,0,0), v(1,0,0)*10, RGB(0,0,255), "X", 1.0, TRUE, 0.2, TRUE, TRUE).
SET y TO VECDRAW(V(0,0,0), v(0,1,0)*10, RGB(0,255,0), "Y", 1.0, TRUE, 0.2, TRUE, TRUE).
SET z TO VECDRAW(V(0,0,0), v(0,0,1)*10, RGB(255,0,0), "Z", 1.0, TRUE, 0.2, TRUE, TRUE).
//===========================================================================================
set Rad to ship:body:radius + ship:altitude.
set Vorb to sqrt(ship:body:Mu/Rad).
set dI to targetIncl-orbit:inclination.
set dIp to dI.
set dVincl to 2*Vorb*sin(abs(dI/2)).
if dI > 0 {set vec to v(0,dVincl,0).}
else {set vec to v(0,-dVincl,0).}

print "dI = " + dI.
print "dVincl = " + dVincl.
SET inclVector TO VECDRAW(V(0,0,0), vec, RGB(255,255,0), "inclVector", 1.0, TRUE, 0.2, TRUE, TRUE).

lock steering to vec.
from {local foo to 5.} until foo=0 step {set foo to foo-1.} do {
	print "..."+foo.
	wait 1.
}

until abs(dI) < 0.001 { 
	clearscreen.
	set dI to targetIncl-orbit:inclination.
	set dVincl to 2*Vorb*sin(abs(dI/2)).
	if dI > 0 {set vec to v(0,dVincl,0).}
	else {set vec to v(0,-dVincl,0).}
	
	lock steering to vec.
	lock throttle to MAX(abs(vec:mag/dIp)/1000, 0.001).
	
	SET inclVector TO VECDRAW(V(0,0,0), vec, RGB(255,255,0), "inclVector", 1.0, TRUE, 0.2, TRUE, TRUE).
	print "dI = " + dI.
	print "dVincl = " + dVincl.
}

set throttle to 0.
sas on.

set x:show to false.
set y:show to false.
set z:show to false.
set inclVector:show to false.
