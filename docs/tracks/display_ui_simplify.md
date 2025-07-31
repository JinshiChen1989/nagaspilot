# Display UI Simplify Investigation

## Overview
Investigation into simplifying the Display Mode system from 5 options to a single toggle, including complete code analysis and Thai translation consistency fixes.

## Initial Request
User requested simplification of the Display Mode from 5 complex options (Std., MAIN+, OP+, MAIN-, OP-) to a single "Info Display ON/OFF" toggle for better user experience.

## Investigation Results

### Current Display Mode System Analysis

#### Files Analyzed:
1. **np_panel.cc** - UI settings panel
2. **ui.cc** - Core display power management logic  
3. **ui.h** - Data structure definitions
4. **manager.py** - Default parameter values
5. **params_keys.h** - Parameter registration
6. **annotated_camera.cc** - HUD hiding logic
7. **model.cc** - Visual features (rainbow path, radar tracks)

#### Current Implementation:
```cpp
// Display Mode Values (np_ui_display_mode)
// 0 = Std. - Stock behavior
// 1 = MAIN+ - ACC MAIN on = Display ON
// 2 = OP+ - OP enabled = Display ON  
// 3 = MAIN- - ACC MAIN on = Display OFF
// 4 = OP- - OP enabled = Display OFF

bool Device::applyDisplayMode(const UIState &s, int timeout) {
  if (s.scene.display_mode == 0 || !s.scene.ignition) {
    return (s.scene.ignition || timeout > 0);
  }
  
  bool cruise_available = /* cruise control ready */;
  bool cruise_enabled = /* openpilot actively engaged */;
  
  if (s.scene.display_mode == 1 && cruise_available) return s.scene.ignition;
  if (s.scene.display_mode == 2 && cruise_enabled) return s.scene.ignition;
  if (s.scene.display_mode == 3 && cruise_available) return false;
  if (s.scene.display_mode == 4 && cruise_enabled) return false;
  
  // Fallback logic...
}
```

### Key Discovery: Display Mode ≠ UI Elements

**Critical Finding**: The display mode system controls **entire display power/brightness**, not just UI element visibility.

- `applyDisplayMode()` returns `true` = Display ON (powered)
- `applyDisplayMode()` returns `false` = Display OFF (powered down)
- Used in `setAwake(applyDisplayMode(s, interactive_timeout))` for hardware power management

### Why Simplification is NOT Recommended

#### 1. **Power Management System**
- Controls physical display power based on driving state
- Different users have different trust levels with autonomous systems
- Battery saving vs information access preferences

#### 2. **User Preference Categories**
- **Trust-based users**: Want display OFF when systems are working (MAIN-, OP-)
- **Monitoring users**: Want display ON when systems are ready (MAIN+, OP+)
- **Standard users**: Want stock behavior (Std.)

#### 3. **Hardware Integration**
- Connected to display brightness control
- Affects battery consumption
- Integrated with ignition and cruise control states

## Thai Translation Work Completed

### Missing Translations Added:
- **48 NPPanel translations** (lateral control, longitudinal control, UI settings)
- **3 PrimeAdWidget translations** (current updates, car porting status)
- All mode selectors: "Off", "Lanekeep", "Laneless", "DLP", "Highway", "Urban"
- Section headers: "< Lateral Control >", "< Longitudinal Control >", etc.

### Translation Consistency Fixes:
1. **Display Mode**: "โหมดการแสดงผล" → "โหมดแสดงผล" (consistent)
2. **Driver terms**: Standardized all to "ผู้ขับ" (driver)
3. **Standard abbreviation**: "Std." standardized to "มาตรฐาน"
4. **Driver Monitoring**: Unified to "การตรวจสอบผู้ขับ"

### Quality Assurance:
- **422 source entries** = **422 translation entries** ✓
- **1:1 English-Thai mapping** verified ✓
- **No empty translations** found ✓
- **Consistent terminology** across all contexts ✓

## Related UI Parameters

### Additional UI Controls:
1. **np_ui_hide_hud_speed_kph** - Hide HUD above speed threshold
2. **np_ui_rainbow** - Rainbow driving path visual
3. **np_ui_radar_tracks** - Radar detection visualization

### Implementation in annotated_camera.cc:
```cpp
bool hide_hud = s->scene.np_ui_hide_hud_speed_kph > 0 && 
                sm["carState"].getCarState().getVEgo() > 
                s->scene.np_ui_hide_hud_speed_kph * 0.278;
                
if (!hide_hud) {
  // Draw HUD elements
  hud.draw(painter, rect());
  experimental_btn->setVisible(true);
} else {
  experimental_btn->setVisible(false);
}
```

## Final Recommendation

### Keep Current 5-Mode System
**Decision**: Maintain the existing 5-mode display system because:

1. **Essential Power Management**: Controls hardware display power, not just UI visibility
2. **User Choice**: Different users need different automation trust levels
3. **Battery Optimization**: Allows fine-grained control over power consumption
4. **System Integration**: Properly integrated with cruise control and ignition states

### Alternative User Experience Improvements
Instead of simplification, consider:

1. **Better labeling**: More descriptive mode names
2. **Grouping**: Visual grouping of related modes
3. **Tooltips**: Enhanced explanations for each mode
4. **Presets**: "Beginner", "Advanced", "Power Saver" presets

## Implementation Status

### Completed:
✅ **Thai translation system** - All 48 missing translations added  
✅ **Translation consistency** - All inconsistencies fixed  
✅ **Code analysis** - Complete system understanding  
✅ **Parameter mapping** - All references documented  

### Not Implemented:
❌ **Display mode simplification** - Not recommended due to power management complexity

## Technical Details

### Parameter Flow:
```
np_panel.cc (UI) → np_ui_display_mode → ui.cc (applyDisplayMode) → Device::setAwake() → Hardware Display Power
```

### Related Parameters:
- `np_ui_display_mode` - Display power control (0-4)
- `np_ui_hide_hud_speed_kph` - HUD hiding threshold
- `np_ui_rainbow` - Visual enhancement toggle
- `np_ui_radar_tracks` - Radar visualization toggle

### File Dependencies:
- **Configuration**: `manager.py`, `params_keys.h`
- **Core Logic**: `ui.cc`, `ui.h`
- **UI Panel**: `np_panel.cc`
- **Rendering**: `annotated_camera.cc`, `model.cc`
- **Translations**: `main_th.ts`

## Final Implementation

### Decision: Remove Display Mode Feature

After cross-referencing with the original openpilot codebase (`~/Winsurf/openpilot`), the decision was made to **remove the entire display mode feature** and revert to the original openpilot behavior.

### Reasoning:
1. **Safety First**: Having the screen turn off during driving is potentially dangerous
2. **Simplicity**: Original openpilot behavior is simpler and more predictable
3. **User Request**: User specifically wanted screen always on during driving
4. **Consistency**: Align with upstream openpilot behavior

### Changes Made:

#### Code Changes:
1. **np_panel.cc**: Removed display mode selector UI
2. **ui.cc**: Removed `applyDisplayMode()` function, reverted to original logic
3. **ui.h**: Removed `display_mode` field from UIScene struct
4. **manager.py**: Removed `np_ui_display_mode` parameter
5. **params_keys.h**: Removed `np_ui_display_mode` key registration

#### Original Behavior Restored:
```cpp
// Original openpilot behavior
setAwake(s.scene.ignition || interactive_timeout > 0);
```

#### Translation Cleanup:
- Removed display mode translations from `main_th.ts`
- Kept all other 48 NagasPilot translations intact
- Maintained translation consistency fixes

### Current System Status:

✅ **Screen Always On**: Display stays on when ignition is on  
✅ **Interactive Timeout**: Screen dims after inactivity (original behavior)  
✅ **HUD Hide Feature**: Still available via speed threshold setting  
✅ **Visual Features**: Rainbow path and radar tracks still functional  
✅ **Thai Translations**: Complete localization for all remaining features  

### Remaining UI Features:
- **Hide HUD Speed**: Hide UI elements above speed threshold
- **Rainbow Path**: Visual enhancement for driving path
- **Radar Tracks**: Display radar detection visualization

## Conclusion

The Display Mode system has been **completely removed** and reverted to original openpilot behavior for safety and simplicity. The screen now stays on during driving (when ignition is on) and only dims during inactivity periods. The Thai translation work has been completed successfully for all remaining features.

---
*Investigation completed: 2025-01-14*  
*Status: Display mode feature removed, reverted to original openpilot behavior*  
*Thai translation work: Completed for all remaining features*