function EngThrustIsp {
	list engines in allEngines.
	
	set ActiveEng to list().
	
	set eng_isp to 0.
	set eng_thrust to 0.
	
	for eng in allEngines {
		if (eng:ignition) AND (NOT eng:flameout) {
			ActiveEng:add(eng).
			set eng_thrust to eng_thrust + eng:availablethrust.
			set eng_isp to eng_isp + eng:isp.
		}
	}

	return list(eng_thrust, eng_isp/ActiveEng:length).
}