#include <stdbool.h>
#include <stdint.h>
#include "action_layer.h"
#include "action_util.h"
#include "keycodes.h"
#include "modifiers.h"
#include "quantum_keycodes.h"
#include "oled_driver.h"
#include "quantum.h"
#include "action.h"

#include QMK_KEYBOARD_H

extern uint8_t is_master;

enum layers {
 _BASE = 0,
 _SYMB,
 _NAV,
 _SETTING,
 _NUMFUNC,
 _OSL_T_1,
 _OSR_T_1,
 _OSL_T_2,
 _OSL_H_2,
 _MOUSE,
 _MACRO,
 _LOCK,
};

enum user_keycodes {
  // tap dance keycodes
  ALT_OSL1 = 1,
  ALT_OSL2 = 2,

  // layer key codes
  QWERTY = SAFE_RANGE,
  LOWER,
  RAISE,

  // other custom key codes
  ALT_TAB,

  // custom oneshot mod implementation with no timers
  OS_SHFT,
  OS_CTRL,
  OS_ALT,
};

// callum
typedef enum oneshot_state {
	os_untouched,
	os_pressed,
	os_up_used,
} oneshot_state;

oneshot_state os_shft_state = os_untouched;
oneshot_state os_ctrl_state = os_untouched;
oneshot_state os_alt_state = os_untouched;

// Mouse key speed and acceleration.
#undef MOUSEKEY_DELAY
#define MOUSEKEY_DELAY          0
#undef MOUSEKEY_INTERVAL
#define MOUSEKEY_INTERVAL       16
#undef MOUSEKEY_WHEEL_DELAY
#define MOUSEKEY_WHEEL_DELAY    0
#undef MOUSEKEY_MAX_SPEED
#define MOUSEKEY_MAX_SPEED      6
#undef MOUSEKEY_TIME_TO_MAX
#define MOUSEKEY_TIME_TO_MAX    64

// Momentary mode. ACL buttons will take effect when pressed and deactivate on release.
// KC_ACL0 < KC_ACL1 < unmodified < KC_ACL2
#define MK_MOMENTARY_ACCEL

// unfortunately the setting below doesnt work right now
#undef MK_C_OFFSET_0
#define MK_C_OFFSET_0 1
#undef MK_C_INTERVAL_0
#define MK_C_INTERVAL_0 16


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

[_BASE] = LAYOUT(
  KC_ESC, KC_1, KC_2, KC_3, KC_4, KC_5,                           KC_6, KC_7, KC_8, KC_9, KC_0, KC_GRV,
  LT(_MACRO, KC_TAB), KC_Q, KC_W, KC_E, KC_R, KC_T,                           KC_Y, KC_U, KC_I, KC_O, KC_P, KC_MINS,
  LM(_MOUSE, MOD_LSFT), KC_A, KC_S, KC_D, KC_F, KC_G,           KC_H, KC_J, KC_K, KC_L, KC_SCLN, RSFT_T(KC_QUOT),
  KC_LCTL, KC_Z, KC_X, KC_C, KC_V, KC_B, KC_BTN2,                 KC_BTN1, KC_N, KC_M, KC_COMM, KC_DOT, KC_SLSH, KC_RCTL,
                    KC_LALT, KC_LGUI, TD(ALT_OSL1), LT(_NUMFUNC, KC_SPC),      OSM(MOD_RSFT), TD(ALT_OSL2), KC_RALT, KC_RGUI),

[_SYMB] = LAYOUT(
  KC_LALT, KC_F1, KC_F2, KC_F3, KC_F4, KC_F5,                     KC_F6, KC_F7, KC_F8, KC_F9, KC_F10, KC_F11,
  ALT_TAB, KC_P1, KC_P2, KC_P3, KC_P4, KC_P5,                     KC_P6, KC_P7, KC_P8, KC_P9, KC_P0, KC_F12,
  KC_ESC, KC_EXLM, KC_AT, KC_HASH, KC_DLR, KC_PERC,               KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_TILD,
  KC_TRNS, KC_BSLS, KC_LBRC, KC_RBRC, KC_PIPE, KC_NO, TG(_LOCK),    KC_NO, KC_PLUS, KC_MINS, KC_EQL, KC_LCBR, KC_RCBR, KC_TRNS,
                      KC_TRNS, KC_TRNS, KC_TRNS, KC_SPC,         KC_ENT, KC_UNDS, KC_TRNS, KC_TRNS),

[_NAV] = LAYOUT(
  KC_ESC, KC_NO, KC_NO, KC_NO, KC_BRIU, KC_BRID,                                    KC_MRWD, KC_MFFD, KC_MPLY, KC_MUTE, KC_VOLD, KC_VOLU,
  KC_TAB, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,                                        LCTL(KC_Y), KC_HOME, KC_UP, KC_END, KC_DEL, KC_NO,
  KC_LSFT, KC_NO, KC_LALT, KC_LSFT, KC_LCTL, KC_NO,                                 KC_TAB, KC_LEFT, KC_DOWN, KC_RGHT, KC_BSPC, KC_WH_U,
  KC_LCTL, LCTL(KC_Z), LCTL(KC_X), LCTL(KC_C), LCTL(KC_V), KC_NO, KC_NO, KC_SCRL,   KC_NO, KC_PGUP, KC_PGDN, KC_ENT, RGUI(KC_T), KC_WH_D,
                              KC_TRNS, KC_TRNS, KC_TRNS, KC_SPC,                      KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

[_NUMFUNC] = LAYOUT(
  KC_TRNS, KC_F1, KC_F2, KC_F3, KC_F4, KC_F5,                                           KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NUM,
  KC_LALT, KC_F3, KC_F4, KC_F5, KC_F6, KC_F7,                                           KC_PAST, KC_P7, KC_P8, KC_P9, KC_PPLS, KC_PAUS,
  KC_LSFT, KC_F2, KC_F10, KC_F11, KC_F12, KC_F8,                                        KC_F9, KC_P4, KC_P5, KC_P6, KC_BSPC, KC_ESC,
  KC_LCTL, KC_F1, LALT(KC_LEFT), LALT(KC_RGHT), LALT(KC_F5), KC_F9, LGUI(KC_PGUP),   KC_TRNS, KC_PMNS, KC_P1, KC_P2, KC_P3, KC_PSLS, KC_RCTL,
                    KC_TRNS, KC_TRNS, KC_NO, KC_TRNS,                                   KC_ENT, KC_P0, KC_PDOT, KC_RALT),

[_OSL_T_1] = LAYOUT(
  KC_NO, KC_NO, LCTL(KC_PSCR), KC_PSCR, KC_NO, KC_NO,                                 KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
  LGUI(KC_DOWN), KC_NO, KC_NO, QK_LEAD, KC_NO, KC_NO,          KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
  KC_ESC, LCTL(KC_HOME), OS_ALT, OS_SHFT, OS_CTRL, KC_NO,                               KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
  OSM(MOD_RCTL|MOD_RSFT), LCTL(KC_END), KC_ENT, KC_CAPS, KC_NUM, KC_INS, KC_NO,                   KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
                                        KC_NO, KC_NO, KC_NO, KC_TRNS,                 KC_TRNS, KC_NO, KC_TRNS, KC_TRNS),

[_OSR_T_1] = LAYOUT(
  KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,                                     KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, QK_BOOT,
  KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,                                     KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
  KC_TRNS, KC_TRNS, KC_S, KC_D, KC_F, KC_TRNS,                     KC_TRNS, OS_ALT, OS_SHFT, OS_CTRL, KC_TRNS, KC_TRNS,
  KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,                            KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
                    KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,                           KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

[_OSL_T_2] = LAYOUT(
  KC_NO, KC_NO, KC_NO, KC_CALC, LALT(KC_F4), LCTL(KC_F5),                             KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  LGUI(KC_UP), RCS(KC_1), RCS(KC_2), RCS(KC_3), KC_NO, KC_NO,                         KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  KC_ESC, KC_NO, KC_PSCR, KC_NO, KC_NO, KC_NO,                                        KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,                                    KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
                                    KC_NO, KC_NO, KC_NO, KC_NO,                       KC_NO, KC_NO, KC_NO, KC_NO),

[_OSL_H_2] = LAYOUT(
  KC_TRNS, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,                                         KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  KC_TRNS, KC_NO, KC_HOME, KC_UP, KC_END, KC_NO,                                      KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  KC_TRNS, KC_NO, KC_LEFT, KC_DOWN, KC_RGHT, KC_NO,                                   KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  KC_TRNS, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,                                  KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_TRNS,
                  KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,                                 KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

[_MOUSE] = LAYOUT(
  KC_TRNS, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,                                         KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  KC_TRNS, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,                                         KC_NO, KC_NO, KC_MS_U, KC_NO, KC_NO, KC_NO,
  KC_LSFT, KC_ACL0, KC_BTN2, KC_BTN3, KC_BTN1, KC_NO,                                         KC_BTN1, KC_MS_L, KC_MS_D, KC_MS_R, KC_WH_U, KC_NO,
  KC_TRNS, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,                                  KC_NO, KC_NO, KC_WH_L, KC_NO, KC_WH_R, KC_WH_D, KC_TRNS,
                  KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,                                 KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

[_LOCK] = LAYOUT(
  KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,                                         KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,                                      KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,                                   KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, TG(_LOCK),                                  KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
                  KC_NO, KC_NO, KC_NO, KC_NO,                                 KC_NO, KC_NO, KC_NO, KC_NO),

[_MACRO] = LAYOUT(
  KC_NO, KC_NO, KC_NO, DM_REC2, DM_PLY2, KC_NO,                               KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  KC_NO, KC_NO, KC_NO, DM_REC1, DM_PLY1, KC_NO,                                   KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,                                   KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
  KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,                            KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
                  KC_NO, KC_NO, KC_NO, KC_NO,                                 KC_NO, KC_NO, KC_NO, KC_NO)
};

typedef struct {
  bool is_press_action;
  int state;
} tap;

bool is_oneshot_active = false; // callum

typedef enum {
  TD_NONE = 0,
  SINGLE_TAP = 1,
  SINGLE_HOLD = 2,
  DOUBLE_TAP = 3,
  DOUBLE_HOLD = 4,
  TRIPLE_TAP = 5,
  TRIPLE_HOLD = 6
} td_state_t;

td_state_t cur_dance (tap_dance_state_t *state);
void alt_finished (tap_dance_state_t *state, void *user_data);
void alt_reset (tap_dance_state_t *state, void *user_data);

void set_keylog(uint16_t keycode, keyrecord_t *record);

td_state_t cur_dance (tap_dance_state_t *state) {
  if (state->count == 1) {
    if (state->pressed) return SINGLE_HOLD;
    else return SINGLE_TAP;
  }
  else if (state->count == 2) {
    if (state->pressed) return DOUBLE_HOLD;
    else return DOUBLE_TAP;
  }
  else if (state->count == 3) {
    if (state->interrupted || !state->pressed)  return TRIPLE_TAP;
    else return TRIPLE_HOLD;
  }
  else return 8;
}

typedef struct {
    bool is_press_action;
    td_state_t state;
} td_tap_t;

static td_tap_t alttap_state = {
  .is_press_action = true,
  .state = TD_NONE
};

void alt_finished (tap_dance_state_t *state, void *user_data) {
  alttap_state.state = cur_dance(state);
  switch (alttap_state.state) {
    case SINGLE_TAP: layer_on(_OSL_T_1); is_oneshot_active = true; break;
    case SINGLE_HOLD: layer_on(_SYMB); break;
    case DOUBLE_TAP: set_oneshot_layer(_OSL_T_2, ONESHOT_START); clear_oneshot_layer_state(ONESHOT_PRESSED); break;
    case DOUBLE_HOLD: layer_on(_OSL_H_2); break;
    default: break;
  }
}

void alt_reset (tap_dance_state_t *state, void *user_data) {
  switch (alttap_state.state) {
    case SINGLE_TAP: break;
    case SINGLE_HOLD: layer_off(_SYMB); break;
    case DOUBLE_TAP: break;
    case DOUBLE_HOLD: layer_off(_OSL_H_2); break;
    default: break;
  }
  alttap_state.state = 0;
}

 void alt2_finished (tap_dance_state_t *state, void *user_data) {
  alttap_state.state = cur_dance(state);
  switch (alttap_state.state) {
    case SINGLE_TAP: layer_on(_OSR_T_1); is_oneshot_active = true; break;
    case SINGLE_HOLD: layer_on(_NAV); break;
    case DOUBLE_HOLD: layer_on(_MOUSE); break;
    default: break;
  }
}

void alt2_reset (tap_dance_state_t *state, void *user_data) {
  switch (alttap_state.state) {
    case SINGLE_TAP: break;
    case SINGLE_HOLD: layer_off(_NAV); break;
    case DOUBLE_TAP: break;
    case DOUBLE_HOLD: layer_off(_MOUSE); break;
    case TRIPLE_HOLD: break;
    default: break;
  }
  alttap_state.state = 0;
}

tap_dance_action_t  tap_dance_actions[] = {
  [ALT_OSL1] = ACTION_TAP_DANCE_FN_ADVANCED(NULL,alt_finished, alt_reset),
  [ALT_OSL2] = ACTION_TAP_DANCE_FN_ADVANCED(NULL,alt2_finished, alt2_reset)
};

//callum
bool ignoreOneshot = false;
bool is_oneshot_modifier_queued = false;

bool is_all_modifiers_unqueued(void) {
  bool atleastOneUpUsed = (os_shft_state == os_up_used ||
    os_ctrl_state == os_up_used ||
    os_alt_state == os_up_used);

  bool noPressed = (os_shft_state != os_pressed ||
    os_ctrl_state != os_pressed ||
    os_alt_state != os_pressed);

  return atleastOneUpUsed && noPressed;
}

bool isHomeRowActionAlreadyUsed = false;

void turn_off_homerow(void) {
  layer_off(_OSL_T_1);
  layer_off(_OSR_T_1);
  unregister_code(KC_LSFT);
  unregister_code(KC_LCTL);
  unregister_code(KC_LALT);
  is_oneshot_active = false;
  isHomeRowActionAlreadyUsed = false;

  os_shft_state = os_untouched;
  os_ctrl_state = os_untouched;
  os_alt_state = os_untouched;
}

bool is_homerow_mod_key(uint16_t keycode) {
  switch (keycode) {
  case OS_SHFT:
  case OS_CTRL:
  case OS_ALT:
      return true;
  default:
      return false;
  }
}

// end callum

bool process_record_keymap(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    // case KC_TRNS:
    // case KC_NO:
      /* Always cancel one-shot layer when another key gets pressed */
      // if (record->event.pressed && is_oneshot_layer_active())
      // clear_oneshot_layer_state(ONESHOT_OTHER_KEY_PRESSED);
      // return true;
    case QK_BOOT:
      /* Don't allow reset from oneshot layer state */
      if (record->event.pressed && is_oneshot_layer_active()){
        clear_oneshot_layer_state(ONESHOT_OTHER_KEY_PRESSED);
        return false;
      }
      return true;
    default:
      return true;
  }
  return true;
}


bool is_alt_tab_active = false;

layer_state_t layer_state_set_user(layer_state_t state) {
	if (is_alt_tab_active) {
		unregister_code(KC_LALT);
		is_alt_tab_active = false;
	}

  return state;
}

// dynamic macro hooks
bool is_recording_macro = false;

bool dynamic_macro_record_start_user(int8_t direction) {
  is_recording_macro = true;
  return true;
}

bool dynamic_macro_record_end_user(int8_t direction) {
  is_recording_macro = false;
  return true;
}
// end dynamic macro hooks

//SSD1306 OLED update loop, make sure to enable OLED_DRIVER_ENABLE=yes in rules.mk
#ifdef OLED_ENABLE

/* status variables */
led_t led_usb_state;

static void print_status_narrow(void) {
  /* Print current layer */
  oled_set_cursor(0,1);

  int curr_layer = get_highest_layer(layer_state);
  switch (curr_layer) {
    case _BASE:
      oled_write_ln("Base", false);
      break;
    case _SYMB:
      oled_write_ln("Symb", false);
      break;
    case _NAV:
      oled_write_ln("Nav", false);
      break;
    case _NUMFUNC:
      oled_write_ln("Num", false);
      break;
    case _OSL_T_1:
      oled_write_ln("OSL-1", false);
      break;
    case _OSR_T_1:
      oled_write_ln("OSR-1", false);
      break;
    case _OSL_T_2:
    case _OSL_H_2:
      oled_write_ln("OSL-2", false);
      break;
    case _MOUSE:
      oled_write_ln("Mouse", false);
      break;
    case _MACRO:
      oled_write_ln("Macro", false);
      break;
    case _LOCK:
      oled_write_ln("Lock", false);
      break;
  }

  if (!is_keyboard_master()) return;

  oled_set_cursor(0,3);
  oled_write_ln("Caps", led_usb_state.caps_lock);

  oled_set_cursor(0,5);
  oled_write_ln("Num", !led_usb_state.num_lock);

  oled_set_cursor(0,7);
  oled_write_ln(is_recording_macro ? "REC" : "", false);
}

oled_rotation_t oled_init_user(oled_rotation_t rotation) {
  return OLED_ROTATION_270;
}

bool oled_task_user(void) {
  led_usb_state = host_keyboard_led_state();

  print_status_narrow();

  return false;
}

#endif


bool atleastOneHrmUpUsed(void) {
    return (os_shft_state == os_up_used ||
    os_ctrl_state == os_up_used ||
    os_alt_state == os_up_used);
}

bool allHrmUnpressed(void) {
    return (os_shft_state != os_pressed &&
    os_ctrl_state != os_pressed &&
    os_alt_state != os_pressed);
}

bool someHrmPressed(void) {
    return !allHrmUnpressed();
}


void process_homerow_keys(
  oneshot_state *state,
  uint16_t mod,
  uint16_t trigger,
  uint16_t keycode,
  keyrecord_t *record
) {
    if (keycode != trigger) return;

    if (record->event.pressed) {
        *state = os_pressed;
        register_code(mod);
        return;
    } else {
        // on hrm keyup
        *state = os_up_used;

        if (someHrmPressed()) return;

        if (isHomeRowActionAlreadyUsed) {
            turn_off_homerow();
            return;
        }

        if (atleastOneHrmUpUsed()) {
            is_oneshot_modifier_queued = true;
            layer_off(_OSL_T_1);
            layer_off(_OSR_T_1);
        }
    }
}

bool process_callum(uint16_t keycode, keyrecord_t *record) {
    if (is_homerow_mod_key(keycode)) {
        process_homerow_keys(&os_shft_state, KC_LSFT, OS_SHFT, keycode, record);
        process_homerow_keys(&os_ctrl_state, KC_LCTL, OS_CTRL, keycode, record);
        process_homerow_keys(&os_alt_state, KC_LALT, OS_ALT, keycode, record);
    } else {
        if (is_oneshot_modifier_queued) {
            is_oneshot_modifier_queued = false;
            ignoreOneshot = true; // the modifiers are still pressed but ignore oneshot for the subsequent processes
            return true;
        }

        if (someHrmPressed()) {
            isHomeRowActionAlreadyUsed = true;
            return true;
        }

        // on click of "true" oneshot key (e.g. osl + a), set ignoreOneShot so that hrm and osl will be cleared next
        if (!is_oneshot_modifier_queued && !someHrmPressed()) {
            ignoreOneshot = true;
            return true;
        }
    }

    return false;
}

void post_process_record_user(uint16_t keycode, keyrecord_t *record) {
    if (ignoreOneshot) { // after pressing the final key in the callum process, turn off homerow mods
        turn_off_homerow();
        ignoreOneshot = false;
    }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  if (is_oneshot_active) {
    bool isReturnProcess = process_callum(keycode, record);
    if (isReturnProcess) return true;
  }

  // tri layer setup and custom keycodes
  switch (keycode) {
  case ALT_TAB: // super alt tab macro
    if (record->event.pressed) {
      if (!is_alt_tab_active) {
        is_alt_tab_active = true;
        register_code(KC_LALT);
      }
      register_code(KC_TAB);
    } else {
      unregister_code(KC_TAB);
    }
    break;
    return false;
  }
  return true;
}

// leader key
void leader_start_user(void) {
    // Do something when the leader key is pressed
}

void leader_end_user(void) {
    if (leader_sequence_one_key(KC_N)) { // ñ
        register_mods(MOD_LALT);
        tap_code16(KC_P1);
        tap_code16(KC_P6);
        tap_code16(KC_P4);
        unregister_mods(MOD_LALT);
    }
    else if (leader_sequence_one_key(KC_S)) { // snipping tool
        register_mods(MOD_LGUI);
        register_mods(MOD_LSFT);
        tap_code16(KC_S);
        unregister_mods(MOD_LGUI);
        unregister_mods(MOD_LSFT);
    }
    else if (leader_sequence_two_keys(KC_N, KC_N)) { // Ñ
        register_mods(MOD_LALT);
        tap_code16(KC_P1);
        tap_code16(KC_P6);
        tap_code16(KC_P5);
        unregister_mods(MOD_LALT);
    }
    // some samples:
    // else if (leader_sequence_one_key(KC_F)) {
        // SEND_STRING("Some string.");
    // }
    // else if (leader_sequence_two_keys(KC_D, KC_D)) {
        // Leader, d, d => Ctrl+A, Ctrl+C
        // SEND_STRING(SS_LCTL("a") SS_LCTL("c"));
    // }
}
