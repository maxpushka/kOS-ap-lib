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
scriptDB:add("HohmannTransfer",
	list("lib_orbit/HohmannTransfer.ks", scriptDB["VisVivaCalc"], 
		scriptDB["BurnTime"], scriptDB["EngThrustIsp"])
	).
scriptDB:add("Circularize", 
	list("lib_orbit/Circularize.ks", scriptDB["EngThrustIsp"])).
scriptDB:add("Change_LAN_Inc", 
	list("lib_orbit/Change_LAN_Inc.ks", scriptDB["EngThrustIsp"])).
scriptDB:add("TransferToMoon", 
	list("lib_orbit/TransferToMoon.ks", scriptDB["EngThrustIsp"],
		scriptDB["HohmannTransfer"], scriptDB["VisVivaCalc"],
		scriptDB["BurnTime"], scriptDB["Derivator"])	
	).

scriptDB:add("Ascend",
	list("Ascend.ks", scriptDB["VerticalAccelCalc"], 
		scriptDB["MachNumber"], scriptDB["EngThrustIsp"], 
		scriptDB["BisectionSolver"], scriptDB["Derivator"])
	).
scriptDB:add("Rendezvous",
	list("Rendezvous", scriptDB["HohmannTransfer"],
		scriptDB["Derivator"], scriptDB["BurnTime"])
	).
scriptDB:add("Dock", list("Dock")).
scriptDB:add("Land", list("Land", scriptDB["EngThrustIsp"])).

function Import {
	parameter scriptName.
	
	if scriptName:istype("String") {
		if scriptDB:haskey(scriptName) {imp(scriptDB[scriptName]).}
		else {
			print "Script " + scriptName + " does not exist".
			return false.
		}
	}
	else if scriptName:istype("List") {
		for i in scriptName {
			if scriptDB:haskey(i) {imp(scriptDB[i]).}
			else {
				print "Script " + i + " does not exist".
				return false.
			}
		}
	}
	else {
		print "Error: Import parameters must be either String or List of strings".
		return false.
	}
	print "Import is successfully completed".
	return true.
	
	function imp {
		parameter scriptList.
		for script in scriptList {
			if (script:typename() = "List") {imp(script).}
			else {runoncepath(path+script).}
		}
		return true.
	}
}