clearscreen.
set config:ipu to 999999.

set vIsp to 300.

set dryMass to 0.1.
set g to body:mu / body:radius^2.

set simAltGoal to 0.
set simVelGoal to 0.

set prevVelVec to V(0,0,0).
set prevVelTime to -1.
function predictStopMargin {
    // init data
    set simDt to 0.1.

    set simVelVec to ship:velocity:surface.
    set simAlt to alt:radar.
    set simMass to ship:mass.

    set gravityVec to up:vector * -g.

    set thrust to ship:availablethrust * 0.93. // to compensate for steering/atm ISP losses

    set massFlowFull to thrust / (vIsp * 9.81).

    set observedAccelVec to (
        simVelVec - prevVelVec
        ) / (time:seconds - prevVelTime).

    if prevVelTime < 0 { 
        set prevVelTime to time:seconds.
        set prevVelVec  to simVelVec.
        return 99999. 
    }
    
    set aeroAccelVec to observedAccelVec - gravityVec.
    set prevVelTime to time:seconds.
    set prevVelVec  to simVelVec.

    set initVelMag to max(0.001, simVelVec:mag).
    set initDensity to body:atm:altitudepressure(ship:altitude).

    set iters to 0.
    
    until false {
        set iters to iters + 1.
        set simSpeed to simVelVec:mag.

        set simDt to max(0.05, simSpeed / 500 ).

        // trapezoidididadidal mass integration baka
        local midpointMass is max(dryMass, simMass - (massFlowFull * simDt / 2)).

        // calculating thrust acceleration vector
        set thrustDir to -simVelVec:normalized.
        set thrustAccelMag to thrust / midpointMass.
        set thrustAccelVec to thrustDir * thrustAccelMag.

        // update sim mass
        set simMass to max( dryMass, simMass - massFlowFull * simDt ).

        // set dragSpeedScale to ( simSpeed / initVelMag )^2.
        // lol nvm the below is faster for some reason
        set speedRatio to simSpeed / initVelMag.
        set dragSpeedScale to speedRatio * speedRatio.

        set simDensity to body:atm:altitudepressure(simAlt).
        if initDensity > 0 {
            set dragDensityScale to simDensity / initDensity.
        } else {
            set dragDensityScale to 1.
        }

        set simDragVec to aeroAccelVec * dragSpeedScale * dragDensityScale.

        // compute total acceleration vector 
        set accelVec to thrustAccelVec + gravityVec + simDragVec.

        // update velocity vector using current acceleration
        set oldVerticalVel to vdot(simVelVec, up:vector).

        set simVelVec to simVelVec + accelVec * simDt.

        set verticalVel to vdot(simVelVec, up:vector).

        // trapezoidal altitude integration
        set simAlt to simAlt + ((oldVerticalVel + verticalVel) / 2) * simDt.

        if simAlt < simAltGoal {
            print "iters per sim: " + iters at (0, 2).
            return simAlt.
        }
        if verticalVel >= 0 or simSpeed < simVelGoal {
            print "iters per sim: " + iters at (0, 2).
            return simAlt.
        }
    }
}

clearscreen.
print " - GUIDANCE CPU - 
Waiting for reentry to start simulations...".

// Find the Main CPU core on the ship via its VAB Name Tag
local mainCore is 0.
for p in ship:parts {
    if p:tag = "booster" {
        set mainCore to p:getmodule("kOSProcessor").
    }
}

set maxTPS to 10.

set lastTickT to 0.
set lastTickPrintT to 0.
set ticks to 0.

until false { 
    if time:seconds - lastTickT > (1/maxTPS) {
        set lastTickT to time:seconds.

        if ( ship:verticalSpeed < -20 and ship:altitude < 60000 ) {
            set dryMass to 0.1.
            for p in ship:parts {
                set dryMass to dryMass + p:drymass.
            }

            if mainCore <> 0 {
                mainCore:connection:sendmessage(predictStopMargin()).
            }

            set ticks to ticks + 1.
            if time:seconds - lastTickPrintT >= 1 {
                set lastTickPrintT to time:seconds.
                print "Running... TPS: " + (ticks) + "                             " at (0,1).
                set ticks to 0.
            }
        }
    }

    wait 0.
}