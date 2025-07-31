# Reboot Analysis Report

## Issue Summary

System locks at comma logo after UI-triggered reboot/power off button press. Investigation reveals the root cause is a hanging build process during system startup.

## Root Cause Analysis

### Issue Flow
1. **UI reboot button** → Sets `DoReboot` parameter in `selfdrive/ui/qt/offroad/settings.cc:349`
2. **Manager processes DoReboot** → Calls `HARDWARE.reboot()` in `system/manager/manager.py:356-358`
3. **System reboots** → Executes `launch_chffrplus.sh` startup script
4. **Missing prebuilt file** → Triggers build process at `launch_chffrplus.sh:101-104`
5. **Build hangs** → Comma logo spinner continues indefinitely

### Key Evidence

#### Reboot/Power Off Implementation
- **UI buttons**: `selfdrive/ui/qt/offroad/settings.cc:296-304` (reboot) and `selfdrive/ui/qt/offroad/settings.cc:301-304` (power off)
- **Parameter setting**: `params.putBool("DoReboot", true)` and `params.putBool("DoShutdown", true)`
- **Manager processing**: `system/manager/manager.py:324-333` monitors these parameters
- **Hardware calls**: `system/hardware/tici/hardware.py:109-110` and `system/hardware/tici/hardware.py:312-313`

#### Boot Sequence
- **Launch script**: `launch_chffrplus.sh` handles system startup
- **Build check**: Line 101 checks for `/home/vcar/Winsurf/nagaspilot/prebuilt` file
- **Build trigger**: If prebuilt file missing, calls `./build.py` at line 102
- **Spinner display**: `system/manager/build.py:91` shows comma logo during build

#### Build Process Analysis
- **Build system**: Uses SCons with progress tracking (`system/manager/build.py:20-78`)
- **Spinner implementation**: `system/ui/spinner.py` displays comma logo with rotating spinner
- **Retry mechanism**: Build retries with reduced parallelism if it fails (`system/manager/build.py:36-61`)
- **Hang condition**: Build process gets stuck indefinitely, spinner continues

### File System Evidence
- **No prebuilt file exists**: `/home/vcar/Winsurf/nagaspilot/prebuilt` not found
- **Build cache location**: `/data/scons_cache` may contain corrupted data
- **Launch flow**: System always triggers build on missing prebuilt marker

## Solutions

### Option 1: Create Prebuilt File (Quick Fix)
```bash
touch /home/vcar/Winsurf/nagaspilot/prebuilt
```
**Risk**: Low - Bypasses build requirement  
**Impact**: Immediate resolution of hang issue

### Option 2: Force Rebuild with Cleanup
```bash
cd /home/vcar/Winsurf/nagaspilot/system/manager
rm -rf /data/scons_cache/*  # Clear build cache
./build.py  # Manual build to verify
touch ../../prebuilt  # Create prebuilt marker
```
**Risk**: Medium - May reveal underlying build issues  
**Impact**: Addresses root cause and verifies build system

### Option 3: Investigate Build Environment
- Clear corrupted build cache: `/data/scons_cache`
- Check disk space and memory availability
- Verify file system integrity
- Review build logs for specific failure points

## Prevention Measures

### Short-term
1. Monitor for missing prebuilt file after updates
2. Implement build timeout mechanism in startup script
3. Add fallback mechanism if build fails

### Long-term
1. Improve build reliability and error handling
2. Add build status monitoring to UI
3. Implement graceful degradation for build failures
4. Add build cache validation checks

## Technical Details

### Critical Files
- `launch_chffrplus.sh:101-104` - Build trigger logic
- `system/manager/build.py:91` - Spinner initialization
- `system/ui/spinner.py:86-99` - Comma logo display
- `selfdrive/ui/qt/offroad/settings.cc:344-368` - Reboot/shutdown handlers
- `system/manager/manager.py:356-361` - Parameter processing

### Parameters Involved
- `DoReboot` - Triggers system reboot
- `DoShutdown` - Triggers system shutdown
- Build cache location: `/data/scons_cache`
- Prebuilt marker: `/home/vcar/Winsurf/nagaspilot/prebuilt`

## Recommended Action

**Immediate**: Implement Option 1 (create prebuilt file) to resolve the hang

**Follow-up**: Execute Option 2 to verify build system integrity and prevent future occurrences

The comma logo spinner indicates the system is properly detecting the missing prebuilt file and attempting to build, but the build process is hanging due to environment issues rather than code problems.