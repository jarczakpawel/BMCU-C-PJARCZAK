/* 
 * ============================================================================
 * DUAL MICROSWITCH SYSTEM CONFIGURATION
 * ============================================================================
 * 
 * This file contains configurable parameters for the dual microswitch
 * filament detection system.
 * ============================================================================
 */

// ============================================================================
// SYSTEM ENABLE
// ============================================================================
// true  = Dual microswitch system active
// false = Single microswitch system (classic mode)
#define IS_TWO_MICROSWITCH_ENABLED  true

// ============================================================================
// ADC VOLTAGE THRESHOLDS
// ============================================================================
//
// Typical voltage divider configuration:
// - No microswitch:     ~0.0-0.5V  (open circuit or pull-down)
// - External only:      ~1.4-1.7V  (first divider)
// - Both switches:      ~2.0-2.5V  (second divider)
// - Internal only:      anomaly, should not occur

#define THRESHOLD_OFFLINE        0.6f   // Below this value = no microswitch
#define THRESHOLD_EXTERNAL_MIN   1.4f   // Minimum for external only
#define THRESHOLD_EXTERNAL_MAX   1.7f   // Maximum for external only
#define THRESHOLD_BOTH           1.7f   // Above this value = both switches