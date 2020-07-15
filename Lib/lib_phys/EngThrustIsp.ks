function EngThrustIsp {
	list engines in allEngines.
	
	set ActiveEng to list().
	
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
	
	if (ActiveEng:length=0) {return list(10^(-15),10^(-15)).}
	else {return list(eng_thrust, eng_isp/ActiveEng:length).}
}