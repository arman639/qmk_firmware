#include QMK_KEYBOARD_H
#include <stdio.h>
#include "wpm.h"
extern uint8_t is_master;


#define _LAYER0 0
#define _LAYER1 1
#define _LAYER2 2
#define _LAYER3 3
#define _LAYER4 4
#define _LAYER5 5
#define _LAYER6 6
#define _LAYER7 7

enum custom_keycodes {
    QWERTY = SAFE_RANGE,
    LOWER,
    RAISE,
    ADJUST,
	EXTRA,
	ALT_TAB
};

//Tap dance enums
enum {
  ALT_OSL1 = 1
};

//Tap dance end

//TD(ALT_OSL1)

 const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

[_LAYER0] = LAYOUT(KC_ESC, KC_1, KC_2, KC_3, KC_4, KC_5, KC_6, KC_7, KC_8, KC_9, KC_0, KC_GRV, KC_TAB, KC_Q, KC_W, KC_E, KC_R, KC_T, KC_Y, KC_U, KC_I, KC_O, KC_P, KC_MINS, KC_LSFT, KC_A, LALT_T(KC_S), LSFT_T(KC_D), LCTL_T(KC_F), KC_G, KC_H, LCTL_T(KC_J), LSFT_T(KC_K), LALT_T(KC_L), KC_SCLN, RSFT_T(KC_QUOT), KC_LCTL, KC_Z, KC_X, KC_C, KC_V, KC_B, KC_LBRC, KC_RBRC, KC_N, KC_M, KC_COMM, KC_DOT, KC_SLSH, KC_RSFT, KC_LALT, KC_LGUI, TD(ALT_OSL1), KC_SPC, KC_ENT, MO(2), KC_RALT, KC_RCTL),

[_LAYER1] = LAYOUT(KC_LALT, KC_F1, KC_F2, KC_F3, KC_F4, KC_F5, KC_F6, KC_F7, KC_F8, KC_F9, KC_F10, KC_F11, ALT_TAB, KC_P1, KC_P2, KC_P3, KC_P4, KC_P5, KC_P6, KC_P7, KC_P8, KC_P9, KC_P0, KC_F12, KC_ESC, KC_EXLM, KC_AT, KC_HASH, KC_DLR, KC_PERC, KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_TILD, KC_TRNS, KC_BSLS, KC_LBRC, KC_RBRC, KC_PIPE, KC_NO, KC_PSCR, KC_NO, KC_PLUS, KC_UNDS, KC_EQL, KC_LCBR, KC_RCBR, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_SPC, KC_ENT, MO(3), KC_TRNS, KC_TRNS),

[_LAYER2] = LAYOUT(KC_ESC, KC_NO, KC_NO, KC_NO, KC_BRIU, KC_BRID, KC_MRWD, KC_MFFD, KC_MPLY, KC_MUTE, KC_VOLD, KC_VOLU, KC_TAB, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_HOME, KC_UP, KC_END, KC_DEL, KC_NO, KC_LSFT, KC_NO, KC_LALT, KC_LSFT, KC_LCTL, KC_NO, KC_PGUP, KC_LEFT, KC_DOWN, KC_RGHT, KC_BSPC, KC_WH_U, KC_LCTL, LCTL(KC_Z), LCTL(KC_X), LCTL(KC_C), LCTL(KC_V), KC_NO, KC_NO, KC_SCRL, KC_PGDN, LCTL(KC_PGUP), KC_NO, LCTL(KC_PGDN), RGUI(KC_T), KC_WH_D, KC_TRNS, KC_TRNS, MO(3), KC_SPC, KC_ENT, KC_TRNS, KC_TRNS, KC_TRNS),

[_LAYER3] = LAYOUT(KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, QK_BOOT, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_Y, KC_U, KC_I, KC_O, KC_P, KC_MINS, KC_TRNS, LCTL(KC_LALT), KC_LALT, KC_LSFT, KC_LCTL, KC_NO, KC_H, KC_J, KC_K, KC_L, KC_SCLN, KC_QUOT, KC_TRNS, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_N, KC_M, KC_COMM, KC_DOT, KC_SLSH, KC_RSFT, KC_TRNS, KC_TRNS, KC_TRNS, KC_NO, KC_NO, KC_TRNS, KC_TRNS, KC_TRNS),

[_LAYER4] = LAYOUT(KC_TRNS, KC_F1, KC_F2, KC_F3, KC_F4, KC_F5, KC_NO, KC_PSLS, KC_PAST, KC_PMNS, KC_NO, KC_NUM, KC_LALT, KC_F6, KC_F7, KC_F8, KC_F9, LCA(KC_PAUS), KC_F9, KC_P7, KC_P8, KC_P9, KC_PPLS, KC_PAUS, KC_LSFT, KC_WH_U, KC_F10, KC_F11, KC_F12, KC_PGUP, KC_NO, KC_P4, KC_P5, KC_P6, KC_BSPC, KC_ESC, KC_LCTL, KC_WH_D, LALT(KC_LEFT), LALT(KC_RGHT), LALT(KC_F5), KC_PGDN, KC_TRNS, KC_TRNS, KC_NO, KC_P1, KC_P2, KC_P3, KC_PEQL, KC_RCTL, KC_TRNS, KC_TRNS, KC_NO, KC_SPC, KC_ENT, KC_P0, KC_PDOT, KC_RALT),

[_LAYER5] = LAYOUT(KC_NO, KC_NO, LCTL(KC_PSCR), KC_PSCR, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, LGUI(KC_DOWN), LGUI(KC_1), LGUI(KC_2), LGUI(KC_3), LGUI(KC_4), LGUI(KC_5), KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_ESC, LCTL(KC_HOME), KC_NO, KC_APP, KC_BTN1, KC_NO, KC_NO, KC_NO, LAG(KC_K), KC_NO, KC_NO, KC_NO, KC_NO, LCTL(KC_END), KC_INS, KC_CAPS, LSFT(KC_F10), KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO),

[_LAYER6] = LAYOUT(KC_NO, KC_NO, KC_NO, KC_CALC, LALT(KC_F4), LCTL(KC_F5), KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, LGUI(KC_UP), RCS(KC_1), RCS(KC_2), RCS(KC_3), KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_ESC, KC_NO, KC_PSCR, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO),

[_LAYER7] = LAYOUT(KC_TRNS, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_TRNS, KC_NO, KC_HOME, KC_UP, KC_END, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_TRNS, KC_NO, KC_LEFT, KC_DOWN, KC_RGHT, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_TRNS, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS) 

};

typedef struct {
  bool is_press_action;
  int state;
} tap;

enum {
  SINGLE_TAP = 1,
  SINGLE_HOLD = 2,
  DOUBLE_TAP = 3,
  DOUBLE_HOLD = 4,
  TRIPLE_TAP = 5,
  TRIPLE_HOLD = 6
};

int cur_dance (qk_tap_dance_state_t *state);
void alt_finished (qk_tap_dance_state_t *state, void *user_data);
void alt_reset (qk_tap_dance_state_t *state, void *user_data);

void        set_keylog(uint16_t keycode, keyrecord_t *record);



int cur_dance (qk_tap_dance_state_t *state) {
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

static tap alttap_state = {
  .is_press_action = true,
  .state = 0
};

void alt_finished (qk_tap_dance_state_t *state, void *user_data) {
  alttap_state.state = cur_dance(state);
  switch (alttap_state.state) {
    case SINGLE_TAP: set_oneshot_layer(5, ONESHOT_START); clear_oneshot_layer_state(ONESHOT_PRESSED); break;
    case SINGLE_HOLD: layer_on(1); break;
    case DOUBLE_TAP: set_oneshot_layer(6, ONESHOT_START); clear_oneshot_layer_state(ONESHOT_PRESSED); break;
    case DOUBLE_HOLD: layer_on(4); break;
	case TRIPLE_HOLD: layer_on(7); break;
    //Last case is for fast typing. Assuming your key is `f`:
    //For example, when typing the word `buf4fer`, and you want to make sure that you send `ff` and not `Esc`.
    //In order to type `ff` when typing fast, the next character will have to be hit within the `TAPPING_TERM`, which by default is 200ms.
  }
}

void alt_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (alttap_state.state) {
    case SINGLE_TAP: break;
    case SINGLE_HOLD: layer_off(1); break;
    case DOUBLE_TAP: break;
    case DOUBLE_HOLD: layer_off(4); break;
	case TRIPLE_HOLD: layer_off(7); break;
  }
  alttap_state.state = 0;
}

qk_tap_dance_action_t tap_dance_actions[] = {
  [ALT_OSL1]     = ACTION_TAP_DANCE_FN_ADVANCED(NULL,alt_finished, alt_reset)
};

bool process_record_keymap(uint16_t keycode, keyrecord_t *record) {

  switch (keycode) {
    case KC_TRNS:
    case KC_NO:
      /* Always cancel one-shot layer when another key gets pressed */
      if (record->event.pressed && is_oneshot_layer_active())
      clear_oneshot_layer_state(ONESHOT_OTHER_KEY_PRESSED);
      return true;
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



// uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
//     switch (keycode) {
//         case SFT_T(KC_SPC):
//             return TAPPING_TERM + 1250;
//         default:
//             return TAPPING_TERM;
//     }
// }



enum layers {
    _QWERTY,
    _LOWER,
    _RAISE,
    _ADJUST,
    _UTIL,
	_ALT_TAB
};
 

bool is_alt_tab_active = false;

layer_state_t layer_state_set_user(layer_state_t state) {
	if (is_alt_tab_active) {
		unregister_code(KC_LALT);
		is_alt_tab_active = false;
	}
	
  return state;
}
 
//SSD1306 OLED update loop, make sure to enable OLED_DRIVER_ENABLE=yes in rules.mk
#ifdef OLED_ENABLE
 
/* KEYBOARD PET START */
 
/* settings */
#define MIN_WALK_SPEED 10
#define MIN_RUN_SPEED 40
 
/* advanced settings */
#define ANIM_FRAME_DURATION 200 // how long each frame lasts in ms
#define ANIM_SIZE 96 // number of bytes in array. If you change sprites, minimize for adequate firmware size. max is 1024
 
/* timers */
uint32_t anim_timer = 0;
uint32_t anim_sleep = 0;
 
/* current frame */
uint8_t current_frame = 0;
 
/* status variables */
int current_wpm = 0;
led_t led_usb_state;
 


 
static void print_logo_narrow(void) {
 
    /* wpm counter */
    char wpm_str[8];
    oled_set_cursor(0,8);
    sprintf(wpm_str, " %03d", current_wpm);
    oled_write(wpm_str, false);
 
    oled_set_cursor(0,9);
    oled_write(" wpm", false);
}
 
static void print_status_narrow(void) {
 
 
    oled_set_cursor(0,3);
 
    switch (get_highest_layer(default_layer_state)) {
        case _QWERTY:
            oled_write("QWRTY", false);
            break;
        case _ADJUST:
            oled_write("ADJUST", false);
            break;
        default:
            oled_write("UNDEF", false);
    }
 
    oled_set_cursor(0,5);
 
    /* Print current layer */
    oled_write("LAYER", false);
 
    oled_set_cursor(0,6);
 
    switch (get_highest_layer(layer_state)) {
        case _QWERTY:
            oled_write("Base ", false);
            break;
        case _ADJUST:
            oled_write("Adjst", false);
            break;
        case _RAISE:
            oled_write("Raise", false);
            break;
        case _LOWER:
            oled_write("Lower", false);
            break;
		case _UTIL:
            oled_write("Util", false);
            break;	
		case _ALT_TAB:
            oled_write("AltTab", false);
            break;
    }
 
    /* caps lock */
    oled_set_cursor(0,8);
    oled_write("Caps", led_usb_state.caps_lock);
}
 
oled_rotation_t oled_init_user(oled_rotation_t rotation) {
    return OLED_ROTATION_270;
}
 
bool oled_task_user(void) {
 
    /* KEYBOARD PET VARIABLES START */
 
    current_wpm = get_current_wpm();
    led_usb_state = host_keyboard_led_state();
 
    /* KEYBOARD PET VARIABLES END */
 
    if (is_keyboard_master()) {
        print_status_narrow();
    } else {
        print_logo_narrow();
    }
	
	return false;
}
 
#endif
 
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
	// #ifdef OLED_ENABLE
  //       set_keylog(keycode, record);
	// #endif
	
    switch (keycode) {
        case QWERTY:
            if (record->event.pressed) {
                set_single_persistent_default_layer(_QWERTY);
            }
            return false;
        case LOWER:
            if (record->event.pressed) {
                layer_on(_LOWER);
                update_tri_layer(_LOWER, _RAISE, _ADJUST);
            } else {
                layer_off(_LOWER);
                update_tri_layer(_LOWER, _RAISE, _ADJUST);
            }
            return false;
        case RAISE:
            if (record->event.pressed) {
                layer_on(_RAISE);
                update_tri_layer(_LOWER, _RAISE, _ADJUST);
            } else {
                layer_off(_RAISE);
                update_tri_layer(_LOWER, _RAISE, _ADJUST);
            }
            return false;
		case ADJUST:
            if (record->event.pressed) {
                layer_on(_ADJUST);
                update_tri_layer(_LOWER, _RAISE, _ADJUST);
            } else {
                layer_off(_ADJUST);
                update_tri_layer(_LOWER, _RAISE, _ADJUST);
            }
            return false;
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