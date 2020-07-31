function Import {
	parameter scriptName.
	
	local path is "0:/kOS_ap_lib/Lib/".
	local scriptDB is lexicon().
	
	scriptDB:add("Ascend", 
		list("Ascend.ks", "lib_phys/VerticalAccelCalc.ks", 
			"lib_phys/MachNumber.ks", "lib_phys/EngThrustIsp.ks", 
			"/lib_math/BisectionSolver.ks", "lib_math/Derivator.ks")).
	scriptDB:add("BurnTime",
		list("lib_phys/BurnTime.ks", "lib_phys/EngThrustIsp.ks")).
	scriptDB:add("ApsisChange",
		list("lib_orbit/ApsisChange.ks", "lib_phys/VisVivaCalc.ks", 
			scriptDB["BurnTime"], "lib_phys/EngThrustIsp.ks")).
	scriptDB:add("Circularize", 
		list("lib_orbit/Circularize.ks", "lib_phys/EngThrustIsp.ks")).
	scriptDB:add("inclChange", 
		list("lib_orbit/inclChange.ks", "lib_phys/EngThrustIsp.ks")).
	scriptDB:add("MoonTransfer", 
		list("lib_orbit/MoonTransfer.ks", "lib_phys/EngThrustIsp.ks")).
		
	if scriptDB:haskey(scriptName) {imp(scriptDB[scriptName]).}
	else {return false.}
	
	function imp {
		parameter scriptList.
		for script in scriptList {
			if script:typename("List") {imp(script)).}
			else {runoncepath(path+script).}
		}
	}
}