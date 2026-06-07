set config:ipu to 99999.

// ---- orbit targets ----
set targetAlt to 120000.  // (im lazy its always a non eccentric orbit)
set targetInc to 0.
set targetVt  to SQRT(ship:body:mu / (ship:body:radius + targetAlt)).     // orbital velocity
set target_normal to SHIP:BODY:NORTH:VECTOR.  // target inclination normal vector

// ---- pre cilcularization targets ----
set target_coast_pe to -50000. 
set target_coast_sma to ((targetAlt + target_coast_pe) / 2) + ship:body:radius.

set countdown to 5.
set T to -countdown.

// just initial guess. its later measured anyway
set isp to 348.
set m_flow to 0.

function update_vehicle_stats {
    // Get active engines
    LIST ENGINES IN eng_list.
    LOCAL total_thrust IS 0.
    LOCAL total_mflow IS 0.
    
    FOR eng IN eng_list {
        if eng:IGNITION AND NOT eng:FLAMEOUT {
            set total_thrust to total_thrust + eng:AVAILABLETHRUST.
            // Mass flow = Thrust / (Isp * g0)
            set total_mflow to total_mflow + (eng:AVAILABLETHRUST / (eng:VISP * 9.80665)).
        
            set isp to eng:VISP.
        }
    }
    
    // Guard against division by zero if engines are off
    if total_thrust = 0 { RETURN LIST(0, 0, 0). }

    LOCAL effective_isp IS total_thrust / (total_mflow * 9.80665).

    RETURN LIST(effective_isp, total_mflow). // Return as a list for the guidance loop
}

function predictApPe {
    parameter guessA, guessB, isBase.

    set simCpuT to time:seconds.

    set iters to 0.
    set simTime to 0.
    set simDt to 0.5.

    // init shit    
    set simPosVec to ship:position - ship:body:position.
    set simVelVec to ship:velocity:orbit.
    set simMass to ship:mass.

    // constant shit
    set thrust to ship:availablethrust. // to compensate for steering/atm ISP losses
    set massFlowFull to thrust / (isp * 9.81).
    set mu to ship:body:mu.

    until false {
        set simUpVec to simPosVec:normalized.
        set simFwdVec to vCrs(simUpVec, target_normal):normalized.
        set simGravVec to -simUpVec * (SHIP:BODY:MU / simPosVec:sqrmagnitude).

        // trapezoidididadidal mass integration baka
        set midpointMass to max(0.1, simMass - (massFlowFull * simDt / 2)).

        // calculating thrust acceleration vector
        set simPitchTan to guessA + (guessB * simTime).
        set thrustDir to (simFwdVec + (simPitchTan * simUpVec)):NORMALIZED.
        set thrustAccelMag to thrust / midpointMass.
        set thrustAccelVec to thrustDir * thrustAccelMag.

        // update sim mass
        set simMass to max(0.1, simMass - massFlowFull * simDt).

        // compute total acceleration vector 
        set accelVec to thrustAccelVec + simGravVec.

        // update velocity vector using current acceleration
        set oldVelVec to simVelVec.
        set simVelVec to simVelVec + accelVec * simDt.
        
        // trapezoidal altitude integration
        set simPosVec to simPosVec + ((oldVelVec + simVelVec) / 2) * simDt.

        set iters to iters + 1.
        set simTime to simTime + simDt.

        // Calculate the simulated SMA
        set sim_sma to 1 / ((2 / simPosVec:mag) - (simVelVec:sqrmagnitude / mu)).

        if        target_coast_sma - sim_sma > 20000 {
            set simDt to 2.0.
        } else if target_coast_sma - sim_sma > 5000 {
            set simDt to 0.5.
        } else {
            set simDt to 0.05.
        }

        // ---- EXIT CONDITIONS ----
        if sim_sma > target_coast_sma {
            print "exit condition: target SMA reached  " at (0, 4).
            break.
        } else if iters >= 300 {
            print "exit condition: timed out           " at (0, 4).
            break.
        }
    }

    local r_body is simPosVec:mag.
    local v2 is simVelVec:sqrmagnitude.

    // Calculate Eccentricity Vector to find the shape of the orbit
    local hVec is vCrs(simPosVec, simVelVec).
    local eVec is (vCrs(simVelVec, hVec) / mu) - simPosVec:normalized.
    
    // We already hit our target SMA to exit the loop, so we calculate AP and PE
    local final_sma is 1 / ((2 / r_body) - (v2 / mu)).
    local predicted_ap is (final_sma * (1 + eVec:mag)) - ship:body:radius.
    local predicted_pe is (final_sma * (1 - eVec:mag)) - ship:body:radius.

    if isBase {
        print " ---- BASE SIMULATION ---- " at (0, 3).
        // (here goes exit condition)
        print "AP:           " + round(predicted_ap/1000, 2) + " / " + round(targetAlt/1000, 2) + " km    " at (0, 5).
        print "PE:           " + round(predicted_pe/1000, 2) + " / " + round(target_coast_pe/1000, 2) + " km     " at (0, 6).
        print "sim length:   " + round(simTime, 1) + "s   " at (0, 7).
        print "iters:        " + iters + "   " at (0, 8).
        print "cpu time:     " + round((time:seconds-simCpuT)*1000) + "ms   " at (0, 9).
    }

    return list(predicted_ap, predicted_pe, simTime).
}

set state to "".
function setState { 
    parameter s.

    clearscreen.
    print s at (0, 1).

    if s = "VERTICAL CLIMB" {
        stage.

    } else if s = "PITCH PROGRAM" {
        lock STEERING to HEADING(90+targetInc, 80).

    } else if s = "STAGE SEPARATION" { 
    } else if s = "CLOSED LOOP GUIDANCE" {
        set new_stats to update_vehicle_stats().
        set isp to new_stats[0].
        set m_flow to new_stats[1].

    } else if s = "COAST" {     // SECO!!!!
        set new_stats to update_vehicle_stats().
        set isp to new_stats[0].
        set m_flow to new_stats[1].

        lock throttle to 0.

    } else if s = "CIRCULARIZATION BURN" {
    } else if s = "ORBITAL INJECTION COMPLETE" {
        wait 999999.
    }

    set state to s.
}


set initT to 0.
function updatePad {
    if T = -countdown { 
        print "press ENTER for launch" at (0,0).
        set lol to terminal:input:getchar().
        set lol to lol.  // so compiler doesnt complain abt unused var lol
        clearscreen.

        set initT to time:seconds.

    } else if T >= 0 { 
        setState("VERTICAL CLIMB").
    }

    if ship:availableThrust > 0 {
        set initT to time:seconds.
        set countdown to 0.
        setState("VERTICAL CLIMB").
    }
}

set rollAlt to 500.
function updateClimb {
    lock throttle to 1.

    lock STEERING to HEADING(0, 90).
    if ship:altitude >= rollAlt { lock STEERING to HEADING(90+targetInc, 90). }  // THE roll
    if ship:altitude >= 1000 { setState("PITCH PROGRAM"). }
}

set last_mass to 0.
set handover_alt   to 40000.
set handover_pitch to 10.
function updatePitchProgram {

    LOCAL cur_alt IS SHIP:ALTITUDE.
    LOCAL srf_prog_pitch IS 90 - VANG(SHIP:SRFPROGRADE:VECTOR, SHIP:UP:VECTOR).
    LOCAL target_pitch IS 90.

    if srf_prog_pitch > 85 {
        set target_pitch to 84.
    }
    // Shape the turn based on altitude
    else if cur_alt < handover_alt {
        // Scales smoothly from 90 down to 20 degrees based on a sine curve
        LOCAL progress_ratio IS cur_alt / handover_alt.
        set target_pitch to 90 - ((90-handover_pitch) * SIN(progress_ratio * 90)).
    }
    else {
        set target_pitch to 22. // Flat hold above handover altitude
    }
    
    set aoa_limit to 5.
    if cur_alt > 5000 {
        // Scales from 7 degrees at 5km to 35 degrees at 30km
        set aoa_limit to 5 + (28 * MIN(1, (cur_alt - 5000) / 25000)).
    }
    
    LOCAL final_steer_pitch IS MIN(srf_prog_pitch + aoa_limit, MAX(srf_prog_pitch - aoa_limit, target_pitch)).
    
    // Command the steering
    lock STEERING to HEADING(90+targetInc, final_steer_pitch).
    
    // stage sep detection
    if not core:messages:empty {
        local msg is core:messages:pop().
        if msg:content = "STAGE SEPARATION" { setState("STAGE SEPARATION"). }
    }

    if (last_mass - SHIP:MASS) > (last_mass * 0.2) { setState("STAGE SEPARATION"). }  // backup for stage sep detection
    set last_mass to ship:mass.

    set pitch to 90 - VANG(SHIP:FACING:FOREVECTOR, SHIP:UP:vector).
    print "Pitch: " + round(pitch, 1) + "°" at (0, 3).
    print "Target: " + round(final_steer_pitch, 1) + "°" at (0, 4).
    print "AOA: " + round(abs(srf_prog_pitch - pitch), 1) + "°" at (0, 5).
    print "AOA limit: " + round(aoa_limit, 1) + "°" at (0, 6).

    print "stage sep handled by Booster CPU" at (0, 8).
}

function updateStageSeparation {
    stage.
    if ship:availablethrust > 0 { 
        wait 2.5. 
        setState("CLOSED LOOP GUIDANCE"). 
    }
}

// --- init shit ---
set A to 0.21.   // Initial pitch height guess (i have no idea how many degrees up)
set B to -0.01.  // Nose flattening rate guess

FUNCTION updateGuidance {
    local delta is 0.01.

    // Run baseline pass
    local basePass is predictApPe(A, B, true).
    local baseAp is basePass[0].
    local basePe is basePass[1].
    local elapsed is basePass[2].

    // Run perturbed passes
    local a_pass is predictApPe(A + delta, B, false).
    local ap_A is a_pass[0].

    local b_pass is predictApPe(A,         B + delta, false).
    local pe_B  is b_pass[1].

    // Calculate partial derivatives
    local dAp_dA is (ap_A - baseAp) / delta.
    local dPe_dB is (pe_B - basePe) / delta.
    
    // wow far off are we from the dream orbit?
    local errAp is targetAlt - baseAP.
    local errPe  is target_coast_pe - basePE. 

    if abs(dAp_dA) > 0.001 and abs(dPe_dB) > 0.001 {
        
        local step_A is errAp / dAp_dA.
        local step_B is errPe / dPe_dB.

        // RATE LIMITERS
        set step_A to max(-0.02, min(0.02, step_A)).
        set step_B to max(-0.0005, min(0.0005, step_B)).

        // Apply the fix with a gentle learning rate
        set A to A + (step_A * 0.1).
        set B to B + (step_B * 0.1).

        // SAFETY CLAMPS
        set A to max(0, min(tan(45), A)). 
        set B to max(-0.05, min(0.05, B)).
    }

    // ---- PRODUCE THE STEERING OUTPUT ----

    local realUp is (ship:position - ship:body:position):normalized.
    local realFwd is vCrs(realUp, target_normal):normalized.

    // Sideways inclination hold (Kill any left/right drift)
    local current_vc is vdot(ship:velocity:orbit, target_normal).
    local yawNudge is -0.01 * current_vc.

    // Build the final 3D guidance arrow and command the autopilot
    local steerDirection is realFwd + (A * realUp) + (yawNudge * target_normal).
    
    // only update steering if SECO is more than 5s away (to prevent sim going crazy near end)
    if elapsed > 5 { lock steering to lookdirup(steerDirection:normalized, ship:facing:topvector). } 

    local current_sma is ship:orbit:semimajoraxis.

    // log
    print " ---- Live status ---- " at (0, 11).
    print "A (Pitch):    " + round(A, 4) + "    " at (0, 12).
    print "B (Rate):    " + round(B, 5) + "    " at (0, 13).
    print "Time to SECO: " + round(elapsed, 1) + "s   " at (0, 14).
    print "SMA Progress: " + round(current_sma/1000, 1) + " / " + round(target_coast_sma/1000, 1) + " km    " at (0, 15).
    
    // ---- CUTOFF CHECK (SECO) ----
    if current_sma >= target_coast_sma { setState("COAST"). }
}

function updateCoast {
    // get data
    local realUp  is (ship:position - ship:body:position):normalized.
    local realFwd is vCrs(realUp, target_normal):normalized.

    local future_time is TIME:SECONDS + ETA:APOAPSIS.
    local v_at_ap is VELOCITYAT(SHIP, future_time):ORBIT:MAG.
    local delta_v to targetVt - v_at_ap.

    local v_e to (isp * 9.80665).
    local m_final to ship:mass / (constant():e ^ (delta_v / v_e)).
    local m_delta to ship:mass - m_final.

    local burn_time to m_delta / m_flow.
    local burn_eta  to eta:apoapsis - (burn_time/2).

    lock steering to lookDirUp(realFwd, ship:facing:topvector).

    print "Circularization burn ETA:    " + round(burn_eta, 1) + "s   " at (0, 3). 
    print "Circularization burn length: " + round(burn_time, 1) + "s   " at (0, 4).
    print "Circularization burn deltaV: " + round(delta_v, 1) + "m/s   " at (0, 5).

    if burn_eta <= 0 { setState("CIRCULARIZATION BURN"). }
}

function updateCircularization {

    lock throttle to 1.0.

    local Vt to ship:velocity:orbit:mag.
    local delta_v to targetVt - Vt.

    local v_e to (isp * 9.80665).
    local m_final to ship:mass / (constant():e ^ (delta_v / v_e)).
    local m_delta to ship:mass - m_final.

    local burn_time to m_delta / m_flow.

    if Vt >= targetVt { lock throttle to 0. setState("ORBITAL INJECTION COMPLETE"). }

    print "Remainting burn time: " + round(burn_time, 1) + "s   " at (0, 3). 
    print "Remaining deltaV:     " + round(delta_v, 1) + "m/s   " at (0, 4).
}

setState("PAD").

set targetTPS to 50.

set lastTickT to 0.
set lastTickPrintT to 0.
set ticks to 0.

until false {
    if time:seconds - lastTickT >= (1/targetTPS) {

        set lastTickT to time:seconds.

        if state = "PAD" { updatePad(). }
        else if state = "VERTICAL CLIMB" { updateClimb(). }
        else if state = "PITCH PROGRAM" { updatePitchProgram(). }
        else if state = "STAGE SEPARATION" { updateStageSeparation(). }
        else if state = "CLOSED LOOP GUIDANCE" { updateGuidance(). }
        else if state = "COAST" { updateCoast(). } 
        else if state = "CIRCULARIZATION BURN" { updateCircularization(). } 
        else if state = "ORBITAL INJECTION COMPLETE" {} 

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
