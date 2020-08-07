function Rendezvous {
	parameter targetShip.
	set targetShip to vessel(targetShip). //converting str to Vessel type object
	
	clearscreen.
	if (targetShip:typename <> "Vessel") {
		print "Error: the selected target is not a vessel.".
		return false.
	}
	
	set placeholder to "                          ".
	set t_wait to 0.
	set prev_t_wait to 0.
	set TOF to 0.
	set t_wait_dt to makeDerivator_dt(0,1).
	
	when (t_wait > 0) AND (t_wait_dt:call(t_wait) < 0) then {
		set kuniverse:timewarp:rate to 0.
		if true {wait 1.}
		kuniverse:timewarp:warpto(time:seconds + t_wait - 60).
	}
	
	set startburn to false.
	until startburn {
		local pi is constant:pi.
		local w_target is sqrt(body:Mu/targetShip:orbit:semimajoraxis^3).
		local w_ship is sqrt(body:Mu/orbit:semimajoraxis^3).
		
		local a_tranfer is (targetShip:orbit:semimajoraxis+orbit:semimajoraxis)/2.
		set TOF to pi*sqrt(a_tranfer^3/body:Mu).
		local alpha_lead is constant:radtodeg*(w_target*TOF).
		local fi_final is 180-alpha_lead.
		
		local vecS is ship:position - body:position.
		local vecM is targetShip:position - body:position.
		local fi_init is VANG(vecM, vecS).
		
		set t_wait to constant:degtorad*(fi_final-fi_init)/(w_target-w_ship).
		
		if (t_wait < 0) AND (prev_t_wait > 0) {set startburn to true.}
		
		local line is 0.
		print "w_target = " + w_target at(0,line).
		local line is line+1.
		print "w_ship   = " + w_ship at(0,line).
		local line is line+1.
		print "tof = " + ceiling(tof) + placeholder at(0,line).
		local line is line+1.
		print "alpha_lead = " + alpha_lead at(0,line).
		local line is line+1.
		print "fi_init = " + fi_init at(0,line).
		local line is line+1.
		print "fi_final = " + fi_final at(0,line).
		local line is line+1.
		print "t_wait = " + ceiling(t_wait) + placeholder at(0,line).
		local line is line+1.
		print "prev_t_wait = " + prev_t_wait at(0,line).
		
		wait 1.
		set prev_t_wait to t_wait.
	}
	
	local burnTimestamp is time:seconds + TOF.
	local transferR is (positionat(targetShip, burnTimestamp)-body:position):mag - body:radius.
	HoffmanTransfer(transferR).
}