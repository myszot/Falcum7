clearscreen.
set config:ipu to 999999.

set touchdownSpeed to -6.7. // m/s
set suicideBurnStopMargin to 10. // m
set guidedDescentHoverHeight to 20. //m
set totalFuelNeededForLanding to 500. // LF units (yes for now im assumming its powered by LF not some methane shi)

set standingRadarAlt to 40. // just a guess. later calculated
set dryMass to 0.1. // later calculated at APPROACH init

set state to "".
set countdown to 5. // s
set T to -countdown.

local stage2CPU is 0.
for p in ship:parts {
    if p:tag = "stage2" {
        set stage2CPU to p:getmodule("kOSProcessor").
    }
}

set state_f_path to "1:/state.json".

function attemptStateRecovery {
    IF EXISTS(state_f_path) {
        SET read_state TO readJson(state_f_path).
        if read_state:HASKEY("state") {
            return list(true, read_state["state"]).
        }
    }

    return list(false, "").
}

function updateStateFile {
    parameter s.
    local state_lex to lexicon().
    set state_lex["state"] to s:tostring():toupper().

    WRITEJSON(state_lex, state_f_path).
}

// helpers
set g to body:mu / body:radius^2.

// expects the furthest away part from the root to be the engines/legs
function calculateStandingHeight {
    local lowestOffset is 0.
    
    for p in ship:parts {
        // Get the 3D vector from the new root part to this specific part
        local relVector is p:position - ship:rootpart:position.
        
        // Project that vector onto the rocket's spine.
        // (ship:facing:vector points out the nose, so -ship:facing:vector points down to the engines)
        local distanceDown is vdot(relVector, -ship:facing:vector).
        
        // Track whichever part sticks out the furthest out the bottom
        if distanceDown > lowestOffset {
            set lowestOffset to distanceDown.
        }
    }
    return lowestOffset.
}

function setState {
    parameter s.

    clearscreen.
    print s at (0, 1).

    if s = "ASCENT" {
        // stage.
    } else if s = "MECO" {
        
    } else if s = "COAST" {
        stage2CPU:connection:sendmessage("STAGE SEPARATION").

        lock steering to lookdirup(-ship:velocity:surface, facing:topvector).
        stage.
        rcs on.

    } else if s = "REENTRY" {
        // updating shit for the guidance since we finnaly got the booster all alone
        set standingRadarAlt to calculateStandingHeight() + 1.
        print standingRadarAlt at (0, 5).

        for p in ship:parts {
            set dryMass to dryMass + p:drymass.
        }

    } else if s = "REENTRY BURN" {
        rcs off.

        // Fixes the integer overflow of gridfins broken math (FUCK THIS MOD)
        for p in ship:parts {
            if p:name:contains("grid") and p:hasmodule("ModuleControlSurface") {
                p:getmodule("ModuleControlSurface"):setfield("authority limiter", 2).
            }
        }

        ag2 on.
    } else if s = "APPROACH" {
        core:messages:clear().
        set ignitionAlt to -1.
        //ag3 on.
    } else if s = "SUICIDE BURN" {
    } else if s = "LANDING" {
        ag2 off.
        rcs on.
    } else if s = "TOUCHDOWN" {
        lock steering to heading(90, 90).
        lock throttle to 0.0.
    }

    set state to s.
    updateStateFile(s).
}


set initT to 0.
function updatePad {
    until ship:availablethrust > 0 { print "Standing by for liftoff..." at (0, 0). wait 0. }
    
    set initT to time:seconds.
    set countdown to 0.
    setState("ASCENT").
}


function updateAscent {
    print "Ascent handled by Stage 2 CPU" at (0, 3).

    if T > 130 { setState("COAST"). }
}


function updateCoast {
    if ( ship:verticalSpeed < -20 and ship:altitude < 60000 ) { setState("REENTRY"). }
}


function updateReentry {
    if ( ship:altitude < 35000 ) { setState("REENTRY BURN"). }
}


function updateReentryBurn {
    lock throttle to 1.0.

    if ship:velocity:surface:mag < 850 {
        lock throttle to 0.0.
        setState("APPROACH"). 
    }
}


// set nextPredictionT to 0.
function updateApproach {

    // get data n compute
    local ralt is alt:radar.
    
    // if time:seconds > nextPredictionT {
    //     set nextPredictionT to time:seconds + 0.2.

    //     local t0 is time:seconds.
    //     set simStopMargin to predictStopMargin().
    //     print "Predict sim time: " + (time:seconds - t0) at (0,5).
    // }
    
    if not core:messages:empty {
        local msg is core:messages:pop().
        set simStopMargin to msg:content. // Updates our ignition height dynamically
        print simStopMargin at (0, 12).
        set ignitionAlt to ralt - simStopMargin + suicideBurnStopMargin + standingRadarAlt.
    }

    // action
    if ralt < ignitionAlt { setState("SUICIDE BURN"). }

    // logging
    print "Sim Ignition Altitude: " + round(ignitionAlt,1) at (0,2).
    print "Altitude: " + round(ralt,1) at (0,3).
}


function updateSuicideBurn {

    lock throttle to 1.

    // get data
    local ralt is alt:radar.
    local safeThrust is ship:availablethrust * 0.90.
    local maxDecel is (safeThrust / ship:mass) - g.
    
    // PID's ideal curve in the background
    // (Still using absolute values here for the trigger math to keep it simple)
    local idealSpeed is sqrt(2 * maxDecel * max(0.1, ralt - GuidedDescentHoverHeight)).
    local actualSpeed is abs(ship:verticalspeed).

    if actualSpeed <= idealSpeed +10 {
        setState("LANDING").
    }
    if ralt < 50 + standingRadarAlt {
        setState("LANDING").
    }
}


set Kp to 2.0.
set Ki to 0.1.
set Kd to 0.2.
set descentPID to PIDLOOP(Kp, Ki, Kd, -1, 1).
function updateLanding {
    // get data
    set ralt to alt:radar.
    set vspd to abs(ship:verticalspeed).
    set currMass to ship:mass.
    set thrust to ship:availablethrust.
    
    // compute
    set hoverThrottle to (currMass * g) / thrust.
    
    // Calculate Net Deceleration with a 90% thrust safety margin
    local safeThrust is ship:availablethrust * 0.90.
    local maxDecel is (safeThrust / ship:mass) - g.
    
    // Calculate ideal velocity (make it negative because we are falling)
    // Use max(0.1, ralt) to prevent square root of negative numbers if terrain clips
    local idealCurveSpeed is -sqrt(2 * maxDecel * max(0.1, ralt - GuidedDescentHoverHeight - standingRadarAlt)).
    local idealSpeed is min(touchdownSpeed, idealCurveSpeed).
    
    // Feed the PID loop
    set descentPID:setpoint to idealSpeed.

    set descentPID:minoutput to -g.
    set descentPID:maxoutput to (thrust / currMass) - g.

    set requestedAcc to descentPID:update(time:seconds, ship:verticalspeed).

    local requestedThrottle is (currMass * requestedAcc) / thrust.
    
    // Action
    lock throttle to hoverThrottle + requestedThrottle.
    
    // Deploy gear early in this phase
    if ralt < 500 { gear on. }
    
    if ship:status = "LANDED" or ralt <= standingRadarAlt+0.5 { 
        setState("TOUCHDOWN").
        print "Touchdown Vspeed: " + vspd at (0, 2).
    }
    
    // Logging so you can watch the magic
    print "Landing TWR: " + round(thrust / (currMass*g)) at (0, 7).
    print "Ideal Spd:  " + round(idealSpeed, 1) + " m/s   " at (0, 8).
    print "Actual Spd: " + round(ship:verticalspeed, 1) + " m/s   " at (0, 9).
    print "Err: " + round(ship:verticalspeed - idealSpeed, 2) at (0, 10).
    print "PID outpud: " + round(throttle, 2) at (0, 11).

}


function updateTouchdown {
    wait 999999.
}


// attempt state recovery
local attempt to attemptStateRecovery().
local success to attempt[0].
local recovered_state to attempt[1].

if success { 
    // quietly slide in the recovered state
    set T to 0. 
    set state to recovered_state.
} else {
    setState("PAD").
}

set targetTPS to 50.

set lastTickT to 0.
set lastTickPrintT to 0.
set ticks to 0.

until false {
    if time:seconds - lastTickT >= (1/targetTPS) {
        
        set lastTickT to time:seconds.

        if state = "PAD" { updatePad(). }
        else if state = "ASCENT"    { updateAscent(). }
        else if state = "COAST"     { updateCoast(). }
        else if state = "REENTRY"   { updateReentry(). }
        else if state = "REENTRY BURN"   { updateReentryBurn(). }
        else if state = "APPROACH"  { updateApproach(). }
        else if state = "SUICIDE BURN" { updateSuicideBurn(). }
        else if state = "LANDING"   { updateLanding(). }
        else if state = "TOUCHDOWN" { updateTouchdown(). } // break.

        set T to time:seconds - initT - countdown + 0.01.

        if T < 0 { print "T" + round(T, 2) at (0, 0). }
        else { print "T+" + round(T, 2) at (0, 0). }

        
        set ticks to ticks + 1.
        if time:seconds - lastTickPrintT >= 1 {
            set lastTickPrintT to time:seconds.
            print "TPS: " + (ticks) at (0,39).
            set ticks to 0.
        }
    }

    wait 0.
}