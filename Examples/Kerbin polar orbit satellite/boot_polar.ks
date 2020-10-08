//Import script
runoncepath("0:/kOS_ap_lib/Lib/Import.ks").
import("Ascend").

//Set ascend script's parameters
set targetOrbit to 72000.
set targetIncl to 90.
set finalPitch to 85.
set gravTurnAlt to 100. 
set gravTurnV to 150.
set accLimit to 5.0.
set pre_stage to 0.5.
set post_stage to 1.
set jettisonFairing to true.
set jettisonAlt to 60000.
set deployAntennas to true.
set deploySolar to true.
set autoWarp to true.

//Running the script
Ascend(targetOrbit, targetIncl, finalPitch, 
	gravTurnAlt, gravTurnV, accLimit, 
	pre_stage, post_stage, jettisonFairing, 
	jettisonAlt, deployAntennas, deploySolar, autoWarp).
	
//Preparing to deploy satellite
lock steering to prograde.
wait 5.
PRINT "Deploying satellite in ".
FROM {local countdown is 3.} UNTIL countdown = 0 STEP {SET countdown to countdown - 1.} DO {
    print "..." + countdown.
    WAIT 1.
}
stage. //staging satellite
print "Staging complete.".