// Wait for KSP physics to settle
WAIT UNTIL SHIP:UNPACKED.

// Define your unique repository folder name
LOCAL repo_dir IS "0:/Falcum7/".
LOCAL file_name is "stage-2".

CLEARSCREEN.
print "F7 " + file_name.
PRINT "Initializing Boot Loader...".
print "".

IF HOMECONNECTION:ISCONNECTED {
    PRINT "KSC Connection active. Syncing isolated scripts...".
    
    // Copy from your specific folder on the Archive to the Local drive
    // Run this once while connected to KSC to build a tiny binary file_name
    COMPILE repo_dir + file_name + ".ks" TO repo_dir + file_name + ".ksm".
    COPYPATH(repo_dir + file_name + ".ksm", "1:/").
    
    PRINT "Sync complete. Running target file_name...".
    RUNPATH("1:/" + file_name + ".ksm").

} ELSE {
    PRINT "No connection. Attempting local boot...".
    IF EXISTS("1:/" + file_name + ".ksm") {
        RUNPATH("1:/" + file_name + ".ksm").
    } ELSE {
        PRINT "BOOT FAILED: No localized autopilot script found".
    }
}
