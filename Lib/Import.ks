global path is "0:/kOS_ap_lib/Lib/".
global scriptDB is lexicon().

//lib_phys
scriptDB:add("EngThrustIsp", 
	list("lib_phys/EngThrustIsp.ks")).
scriptDB:add("MachNumber", 
	list("lib_phys/MachNumber.ks")).
scriptDB:add("VerticalAccelCalc", 
	list("lib_phys/VerticalAccelCalc.ks")).
scriptDB:add("VisVivaCalc", 
	list("lib_phys/VisVivaCalc.ks")).
scriptDB:add("BurnTime",
	list("lib_phys/BurnTime.ks", scriptDB["EngThrustIsp"])).

//lib_math
scriptDB:add("BisectionSolver", 
	list("/lib_math/BisectionSolver.ks")).
scriptDB:add("Derivator", 
	list("lib_math/Derivator.ks")).

//lib_orbit
scriptDB:add("ApsisChange",
	list("lib_orbit/ApsisChange.ks", scriptDB["VisVivaCalc"], 
		scriptDB["BurnTime"], scriptDB["EngThrustIsp"])
	).
scriptDB:add("Circularize", 
	list("lib_orbit/Circularize.ks", scriptDB["EngThrustIsp"])).
scriptDB:add("inclChange", 
	list("lib_orbit/inclChange.ks", scriptDB["EngThrustIsp"])).
scriptDB:add("TransferToMoon", 
	list("lib_orbit/TransferToMoon.ks", scriptDB["EngThrustIsp"],
		scriptDB["ApsisChange"])	
	).

scriptDB:add("Ascend",
	list("Ascend.ks", scriptDB["VerticalAccelCalc"], 
		scriptDB["MachNumber"], scriptDB["EngThrustIsp"], 
		scriptDB["BisectionSolver"], scriptDB["Derivator"])
	).

function Import {
	parameter scriptName.
	
	if scriptDB:haskey(scriptName) {imp(scriptDB[scriptName]).}
	else {return false.}
	
	function imp {
		parameter scriptList.
		for script in scriptList {
			if (script:typename() = "List") {imp(script).}
			else {runoncepath(path+script).}
		}
	}
}