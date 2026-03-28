/*
 * main.c  –  Smartwatch Application
 * Target : PIC24FJ256GA705 (PIC Curiosity Board)
 * Compiler: XC16
 *
 * ── Architecture ──────────────────────────────────────────────────────────────
 *  • TMR1 ISR fires every second  →  sets g_tick + toggles g_blink_state
 *  • Main loop (~10 ms / iteration via DELAY_milliseconds)
 *      1. Consume g_tick  →  advance RTC, check alarm
 *      2. Read buttons, detect long-press S1
 *      3. Read accelerometer  →  detect shake / flip
 *      4. Drive state machine  →  MODE_CLOCK / MODE_MENU / MODE_ALARM
 *      5. Render display only when something changed
 *
 * ── Driver API used (drivers NOT modified) ───────────────────────────────────
 *  oledC_clearScreen(), oledC_setBackground(color)
 *  oledC_DrawString(x,y,sx,sy,str,color)
 *  oledC_DrawLine(x0,y0,x1,y1,width,color)
 *  oledC_DrawCircle(cx,cy,r,color)
 *  oledC_DrawRectangle(x0,y0,x1,y1,color)
 *  oledC_DrawPoint(x,y,color)  /  oledC_DrawThickPoint(cx,cy,w,color)
 *  i2cReadSlaveRegister(devAddW, regAddr, *byte)
 *  i2cWriteSlave(devAddW, regAddr, byte)
 *  DELAY_milliseconds(ms)
 *  SYSTEM_Initialize()
 *
 * ── Board pin assignments (adjust macros if your wiring differs) ──────────────
 *  LED1 → RA8 (active HIGH)    LED2 → RA9 (active HIGH)
 *  S1   → RC9 (active LOW)     S2   → RC5 (active LOW)
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>                       /* sinf, cosf  – link with -lm */

#include "System/system.h"
#include "System/delay.h"
#include "oledDriver/oledC.h"
#include "oledDriver/oledC_colors.h"
#include "oledDriver/oledC_shapes.h"
#include "Accel_i2c.h"




/* ═══════════════════════════════════════════════════════════════════════════
 * HARDWARE PIN MACROS
 * ═══════════════════════════════════════════════════════════════════════════ */

/* LEDs – output, active HIGH */
#define LED1_TRIS       TRISAbits.TRISA8
#define LED1_LAT        LATAbits.LATA8
#define LED2_TRIS       TRISAbits.TRISA9
#define LED2_LAT        LATAbits.LATA9

#define LED1_ON()       (LED1_LAT = 1)
#define LED1_OFF()      (LED1_LAT = 0)
#define LED2_ON()       (LED2_LAT = 1)
#define LED2_OFF()      (LED2_LAT = 0)

/* Buttons – input, active LOW, internal CN pull-up enabled */
#define S1_TRIS         TRISAbits.TRISA11
#define S1_CNPU         CNPUAbits.CNPUA11
#define S1_PIN          PORTAbits.RA11
#define S1_PRESSED()    (S1_PIN == 0)

#define S2_TRIS         TRISAbits.TRISA12
#define S2_CNPU         CNPUAbits.CNPUA12
#define S2_PIN          PORTAbits.RA12
#define S2_PRESSED()    (S2_PIN == 0)

/* ═══════════════════════════════════════════════════════════════════════════
 * ADXL345 ACCELEROMETER CONSTANTS
 * ═══════════════════════════════════════════════════════════════════════════ */

#define ACCEL_WRITE_ADDR    0x3A        /* 7-bit addr 0x1D << 1 */
#define ACCEL_REG_DEVID     0x00
#define ACCEL_REG_PWRCTL    0x2D
#define ACCEL_REG_DATAX0    0x32
#define ACCEL_REG_DATAY0    0x34
#define ACCEL_REG_DATAZ0    0x36
#define ACCEL_MEASURE_BIT   0x08

/* ═══════════════════════════════════════════════════════════════════════════
 * DISPLAY CONSTANTS  (SSD1351 OLED Click – 96 × 96 px colour)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define SCR_W   96u
#define SCR_H   96u

/* Analog clock geometry */
#define CLK_CX  47u         /* center X */
#define CLK_CY  44u         /* center Y */
#define CLK_R   40u         /* face radius */
#define H_SEC   36u         /* second hand length */
#define H_MIN   28u         /* minute hand length */
#define H_HOUR  18u         /* hour hand length */

/* Colour palette used by the app */
#define COL_BG          OLEDC_COLOR_BLACK
#define COL_TEXT        OLEDC_COLOR_WHITE
#define COL_TIME        OLEDC_COLOR_CYAN
#define COL_DATE        OLEDC_COLOR_YELLOW
#define COL_ALARM_ICON  OLEDC_COLOR_RED
#define COL_FACE        OLEDC_COLOR_DARKGRAY
#define COL_HAND_SEC    OLEDC_COLOR_RED
#define COL_HAND_MIN    OLEDC_COLOR_WHITE
#define COL_HAND_HOUR   OLEDC_COLOR_YELLOW
#define COL_MENU_SEL    OLEDC_COLOR_CYAN
#define COL_MENU_ITEM   OLEDC_COLOR_WHITE
#define COL_MENU_HDR    OLEDC_COLOR_YELLOW
#define COL_CORNER_CLK  OLEDC_COLOR_DARKGRAY
#define COL_ALARM_BG    OLEDC_COLOR_DARKRED
#define COL_ALARM_TEXT  OLEDC_COLOR_WHITE

/* ═══════════════════════════════════════════════════════════════════════════
 * TIMING / DETECTION CONSTANTS
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Long-press duration in seconds (exact, ISR-timed) */
#define LONG_PRESS_SECS     2u

/* Shake: squared magnitude of Δaccel vector must exceed this */
#define SHAKE_MAG_SQ        640000L     /* threshold ≈ 800 raw units */

/* Flip: Z-axis below this value → device face-down */
#define FLIP_Z_THRESH       (-200)      /* ~0.8 g in ±2g, 4mg/LSB range */

/* Alarm auto-stop after N seconds */
#define ALARM_AUTO_STOP     20u

/* Days per month (non-leap year) */
static const uint8_t k_days[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

/* ═══════════════════════════════════════════════════════════════════════════
 * TYPE DEFINITIONS
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    MODE_CLOCK = 0,
    MODE_MENU,
    MODE_ALARM
} AppMode;

typedef enum {
    DISP_DIGITAL = 0,
    DISP_ANALOG
} DispMode;

typedef enum {
    FMT_24H = 0,
    FMT_12H
} TimeFmt;

/* Menu states – each submenu step is its own state */
typedef enum {
    MENU_MAIN = 0,
    MENU_DISP_MODE,         /* choose digital / analog         */
    MENU_TIME_FMT,          /* choose 24h / 12h                */
    MENU_TIME_H,            /* edit hours                      */
    MENU_TIME_M,            /* edit minutes                    */
    MENU_TIME_S,            /* edit seconds                    */
    MENU_DATE_D,            /* edit day                        */
    MENU_DATE_M,            /* edit month                      */
    MENU_ALARM_H,           /* edit alarm hour                 */
    MENU_ALARM_M,           /* edit alarm minute               */
    MENU_ALARM_OFF          /* confirm disable alarm           */
} MenuSt;

#define MENU_ITEM_COUNT  6u
static const char * const k_menu_labels[MENU_ITEM_COUNT] = {
    "Display Mode",
    "12H/24H Format",
    "Set Time",
    "Set Date",
    "Set Alarm",
    "Alarm Off"
};

/* ═══════════════════════════════════════════════════════════════════════════
 * GLOBAL APPLICATION STATE
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Real-time clock */
static uint8_t  g_sec   = 0;
static uint8_t  g_min   = 0;
static uint8_t  g_hour  = 12;

/* Date */
static uint8_t  g_day   = 1;
static uint8_t  g_month = 1;

/* Alarm */
static uint8_t  g_al_hour    = 7;
static uint8_t  g_al_min     = 0;
static bool     g_al_enabled = false;
static bool     g_al_ringing = false;
static uint8_t  g_al_secs    = 0;      /* seconds elapsed since alarm started */

/* Presentation settings */
static DispMode g_disp = DISP_DIGITAL;
static TimeFmt  g_fmt  = FMT_24H;

/* App-level state machine */
static AppMode  g_mode = MODE_CLOCK;

/* Menu state machine */
static MenuSt   g_menu_st  = MENU_MAIN;
static uint8_t  g_menu_cur = 0;        /* cursor row in main menu */
static uint8_t  g_edit_val = 0;        /* scratch register for value editing */

/* ISR-owned flags (volatile prevents compiler from caching them) */
volatile bool     g_tick        = false;   /* set every second by TMR1 ISR       */
volatile bool     g_blink_state = false;   /* toggled every second for alarm blink */
volatile uint16_t g_uptime_secs = 0u;      /* free-running seconds counter         */

/* Last hand positions drawn on analog face (0xFF = not yet drawn) */
static uint8_t s_hand_sec  = 0xFFu;
static uint8_t s_hand_min  = 0xFFu;
static uint8_t s_hand_hour = 0xFFu;


/* ═══════════════════════════════════════════════════════════════════════════
 * FORWARD DECLARATIONS
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Initialisation */
static void hw_init(void);
static void timer1_init(void);
static void accel_init(void);
static void adc_init(void);

/* Potentiometer */
static uint16_t adc_read_pot(void);

/* RTC / alarm */
static void rtc_advance(void);
static void alarm_check(void);
static void alarm_stop(void);

/* Sensors */
static void    accel_read_xyz(int16_t *x, int16_t *y, int16_t *z);
static bool    accel_shaken(void);
static bool    accel_flipped(void);

/* Display helpers */
static void draw_hand(uint8_t cx, uint8_t cy, uint8_t len, float angle_rad, uint16_t color);
static void draw_alarm_icon(uint8_t x, uint8_t y);
static void draw_corner_clock(void);
static void draw_analog_ticks(void);

/* Top-level renderers */
static void render_clock(void);
static void render_digital(void);
static void render_analog(void);
static void render_analog_update(void);   /* partial: hands only, no flash */
static void render_menu(void);
static void render_alarm(void);

/* Menu logic */
static void menu_enter_state(MenuSt st);
static void menu_exit(void);
static void menu_s1_press(void);
static void menu_s2_press(void);

/* ═══════════════════════════════════════════════════════════════════════════
 * TMR1 INTERRUPT SERVICE ROUTINE  (fires every 1 second)
 *
 * Setup: FCY = 4 MHz (FRC 8 MHz / 2)
 *   Prescaler 1:256  →  4,000,000/256 = 15,625 ticks/s
 *   PR1 = 15624  →  interrupt every exactly 1 second
 * ═══════════════════════════════════════════════════════════════════════════ */
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    g_tick        = true;
    g_blink_state = !g_blink_state;
    g_uptime_secs++;
    IFS0bits.T1IF = 0;              /* clear interrupt flag */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * HARDWARE INITIALISATION
 * ═══════════════════════════════════════════════════════════════════════════ */

static void hw_init(void)
{
    /* LEDs: output, start off */
    LED1_TRIS = 0;  LED1_OFF();
    LED2_TRIS = 0;  LED2_OFF();

    /* Buttons: input with weak pull-ups (active LOW) */
    S1_TRIS  = 1;   S1_CNPU  = 1;
    S2_TRIS  = 1;   S2_CNPU  = 1;
}

/* Configure TMR1 for 1-second period */
static void timer1_init(void)
{
    T1CON   = 0x0030;   /* 1:256 prescaler, timer off        */
    TMR1    = 0;
    PR1     = 15624;    /* (4 000 000 / 256) - 1 = 15624     */
    IPC0bits.T1IP = 4;  /* interrupt priority 4              */
    IFS0bits.T1IF = 0;  /* clear flag                        */
    IEC0bits.T1IE = 1;  /* enable TMR1 interrupt             */
    T1CONbits.TON = 1;  /* start timer                       */
}

/* Initialise ADXL345 into measurement mode */
static void accel_init(void)
{
    i2cWriteSlave(ACCEL_WRITE_ADDR, ACCEL_REG_PWRCTL, ACCEL_MEASURE_BIT);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * POTENTIOMETER  –  AN8 / RB12  (component R10 on board)
 *
 * 10-bit ADC → returns 0–1023.
 * Used for menu navigation and live value editing.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void adc_init(void)
{
    ANSBbits.ANSB12   = 1;   /* RB12 / AN8 → analog mode */
    TRISBbits.TRISB12 = 1;   /* input */

    AD1CON1bits.ADON = 0;    /* ensure off before configuring */
    AD1CON1 = 0x0000;        /* integer output, manual sample/convert */
    AD1CON2 = 0x0000;        /* Vdd/Vss reference, CH0, 1 sample/interrupt */
    AD1CON3 = 0x0F02;        /* Tad = 3·Tcy (750 ns @ 4 MHz), 15 Tad sample */
    AD1CHS  = 8u;            /* positive input = AN8 */
    AD1CON1bits.ADON = 1;    /* enable ADC module */
    DELAY_milliseconds(1);   /* allow module to stabilise before first sample */
}

/* Single-shot read with timeout so a misconfigured ADC never hangs the loop. */
static uint16_t adc_read_pot(void)
{
    uint16_t timeout = 50000u;
    AD1CON1bits.SAMP = 1;    /* start sampling */
    __builtin_nop(); __builtin_nop(); __builtin_nop();
    __builtin_nop(); __builtin_nop(); __builtin_nop();  /* ~1.5 µs settle */
    AD1CON1bits.SAMP = 0;    /* end sample → conversion begins */
    while (!AD1CON1bits.DONE && --timeout);
    AD1CON1bits.DONE = 0;
    return (uint16_t)ADC1BUF0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * RTC – advance time and date by one second
 * Called from main loop after consuming g_tick.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void rtc_advance(void)
{
    if (++g_sec < 60u) return;
    g_sec = 0;

    if (++g_min < 60u) return;
    g_min = 0;

    if (++g_hour < 24u) return;
    g_hour = 0;

    /* Midnight roll-over → advance date */
    if (++g_day <= k_days[g_month - 1u]) return;
    g_day = 1;

    if (++g_month <= 12u) return;
    g_month = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ALARM LOGIC
 * ═══════════════════════════════════════════════════════════════════════════ */

static void alarm_check(void)
{
    if (!g_al_enabled || g_al_ringing) return;

    /* Trigger at HH:MM:00 */
    if (g_hour == g_al_hour && g_min == g_al_min && g_sec == 0u) {
        g_al_ringing = true;
        g_al_secs    = 0;
        g_mode       = MODE_ALARM;
    }
}

static void alarm_stop(void)
{
    g_al_ringing = false;
    g_al_secs    = 0;
    g_mode       = MODE_CLOCK;
    oledC_setBackground(COL_BG);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ACCELEROMETER HELPERS
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Read signed 16-bit X/Y/Z from ADXL345 via I2C */
static void accel_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t lo, hi;

    i2cReadSlaveRegister(ACCEL_WRITE_ADDR, ACCEL_REG_DATAX0,     &lo);
    i2cReadSlaveRegister(ACCEL_WRITE_ADDR, ACCEL_REG_DATAX0 + 1, &hi);
    *x = (int16_t)((uint16_t)hi << 8 | lo);

    i2cReadSlaveRegister(ACCEL_WRITE_ADDR, ACCEL_REG_DATAY0,     &lo);
    i2cReadSlaveRegister(ACCEL_WRITE_ADDR, ACCEL_REG_DATAY0 + 1, &hi);
    *y = (int16_t)((uint16_t)hi << 8 | lo);

    i2cReadSlaveRegister(ACCEL_WRITE_ADDR, ACCEL_REG_DATAZ0,     &lo);
    i2cReadSlaveRegister(ACCEL_WRITE_ADDR, ACCEL_REG_DATAZ0 + 1, &hi);
    *z = (int16_t)((uint16_t)hi << 8 | lo);
}

/* Shake: compare current reading to previous; true if large delta */
static bool accel_shaken(void)
{
    static int16_t px = 0, py = 0, pz = 0;
    int16_t x, y, z;
    accel_read_xyz(&x, &y, &z);

    int32_t dx = (int32_t)(x - px);
    int32_t dy = (int32_t)(y - py);
    int32_t dz = (int32_t)(z - pz);
    px = x;  py = y;  pz = z;

    return ((dx*dx + dy*dy + dz*dz) > SHAKE_MAG_SQ);
}

/* Flip: device is face-down when Z is strongly negative */
static bool accel_flipped(void)
{
    int16_t x, y, z;
    accel_read_xyz(&x, &y, &z);
    return (z < (int16_t)FLIP_Z_THRESH);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * DISPLAY HELPERS
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * Draw a clock hand of length `len` from (cx,cy).
 * angle_rad = radians clockwise from 12 o'clock position.
 */
static void draw_hand(uint8_t cx, uint8_t cy, uint8_t len, float angle_rad, uint16_t color)
{
    int8_t ex = (int8_t)((float)cx + (float)len * sinf(angle_rad));
    int8_t ey = (int8_t)((float)cy - (float)len * cosf(angle_rad));
    oledC_DrawLine(cx, cy, (uint8_t)ex, (uint8_t)ey, 1, color);
}

/*
 * Draw a small alarm-bell icon (7×8 px) at pixel (x, y).
 * Uses simple rectangles/lines composited from the color.
 */
static void draw_alarm_icon(uint8_t x, uint8_t y)
{
    /* Bell dome */
    oledC_DrawRectangle(x+1, y,   x+5, y+4, COL_ALARM_ICON);
    /* Bell base / clapper */
    oledC_DrawLine(x,   y+4, x+6, y+4, 1, COL_ALARM_ICON);
    oledC_DrawLine(x+2, y+5, x+4, y+5, 1, COL_ALARM_ICON);
    oledC_DrawPoint(x+3, y+6, COL_ALARM_ICON);
}

/*
 * Small live clock string in the top-right corner.
 * Used to keep the time visible while inside any menu screen.
 */
static void draw_corner_clock(void)
{
    char buf[10];
    uint8_t h = g_hour;
    if (g_fmt == FMT_12H) {
        h = h % 12u;
        if (h == 0u) h = 12u;
    }
    snprintf(buf, sizeof(buf), "%02u:%02u:%02u", h, g_min, g_sec);
    /* Clear the text area first so old digit pixels don't bleed through.
     * "HH:MM:SS" = 8 chars × 6 px wide × 8 px tall at scale 1. */
    oledC_DrawRectangle(36u, 0u, 95u, 8u, COL_BG);
    oledC_DrawString(36u, 0u, 1u, 1u, (uint8_t *)buf, COL_CORNER_CLK);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * DIGITAL CLOCK RENDERER
 * ═══════════════════════════════════════════════════════════════════════════ */
static void render_digital(void)
{
    char    time_buf[10];
    char    date_buf[8];
    uint8_t h  = g_hour;
    bool    pm = false;

    if (g_fmt == FMT_12H) {
        pm = (h >= 12u);
        h  =  h % 12u;
        if (h == 0u) h = 12u;
    }

    /* ── Large HH:MM:SS ── */
    snprintf(time_buf, sizeof(time_buf), "%02u:%02u:%02u", h, g_min, g_sec);
    oledC_DrawString(0u, 30u, 2u, 2u, (uint8_t *)time_buf, COL_TIME);

    /* AM/PM label (12h mode only) */
    if (g_fmt == FMT_12H)
        oledC_DrawString(72u, 52u, 1u, 1u, (uint8_t *)(pm ? "PM" : "AM"), COL_TEXT);

    /* ── Date: DD/MM ── */
    snprintf(date_buf, sizeof(date_buf), "%02u/%02u", g_day, g_month);
    oledC_DrawString(4u, 76u, 1u, 1u, (uint8_t *)date_buf, COL_DATE);

    /* ── Alarm icon top-right ── */
    if (g_al_enabled)
        draw_alarm_icon(84u, 4u);
}

/* ── Shared helper: draw all 12 hour tick marks ── */
static void draw_analog_ticks(void)
{
    const float TWO_PI = 6.283185f;
    uint8_t i;
    for (i = 0u; i < 12u; i++) {
        float a  = TWO_PI * (float)i / 12.0f;
        float sa = sinf(a);
        float ca = cosf(a);
        uint8_t ox = (uint8_t)((float)CLK_CX + (float)CLK_R        * sa);
        uint8_t oy = (uint8_t)((float)CLK_CY - (float)CLK_R        * ca);
        uint8_t ix = (uint8_t)((float)CLK_CX + (float)(CLK_R - 6u) * sa);
        uint8_t iy = (uint8_t)((float)CLK_CY - (float)(CLK_R - 6u) * ca);
        oledC_DrawLine(ix, iy, ox, oy, 1u, COL_TEXT);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ANALOG CLOCK RENDERER  –  full redraw (called once on mode entry)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void render_analog(void)
{
    const float TWO_PI = 6.283185f;

    draw_analog_ticks();

    /* ── Hands (all white) ── */
    float sec_a  = TWO_PI * (float)g_sec  / 60.0f;
    float min_a  = TWO_PI * ((float)g_min  + (float)g_sec  / 60.0f) / 60.0f;
    float hval   = (float)(g_hour % 12u)  + (float)g_min  / 60.0f;
    float hour_a = TWO_PI * hval / 12.0f;

    draw_hand(CLK_CX, CLK_CY, H_HOUR, hour_a, COL_TEXT);
    draw_hand(CLK_CX, CLK_CY, H_MIN,  min_a,  COL_TEXT);
    draw_hand(CLK_CX, CLK_CY, H_SEC,  sec_a,  COL_TEXT);
    oledC_DrawThickPoint(CLK_CX, CLK_CY, 2u, COL_TEXT);

    /* am/pm bottom-left (12h mode only) */
    if (g_fmt == FMT_12H)
        oledC_DrawString(2u, 86u, 1u, 1u,
                         (uint8_t *)((g_hour >= 12u) ? "pm" : "am"), COL_TEXT);

    /* Date DD/MM bottom-right */
    {
        char dline[6];
        snprintf(dline, sizeof(dline), "%02u/%02u", g_day, g_month);
        oledC_DrawString(58u, 86u, 1u, 1u, (uint8_t *)dline, COL_TEXT);
    }

    if (g_al_enabled)
        draw_alarm_icon(84u, 4u);

    /* Store drawn state so render_analog_update() can erase these hands */
    s_hand_sec  = g_sec;
    s_hand_min  = g_min;
    s_hand_hour = g_hour;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ANALOG CLOCK PARTIAL UPDATE  –  erase old hands, redraw ticks, draw new
 * No screen clear → no flash.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void render_analog_update(void)
{
    const float TWO_PI = 6.283185f;

    /* Erase previously drawn hands by painting them in the background colour */
    if (s_hand_sec != 0xFFu) {
        float old_sec_a  = TWO_PI * (float)s_hand_sec / 60.0f;
        float old_min_a  = TWO_PI * ((float)s_hand_min  + (float)s_hand_sec / 60.0f) / 60.0f;
        float old_hval   = (float)(s_hand_hour % 12u)   + (float)s_hand_min  / 60.0f;
        float old_hour_a = TWO_PI * old_hval / 12.0f;
        draw_hand(CLK_CX, CLK_CY, H_HOUR, old_hour_a, COL_BG);
        draw_hand(CLK_CX, CLK_CY, H_MIN,  old_min_a,  COL_BG);
        draw_hand(CLK_CX, CLK_CY, H_SEC,  old_sec_a,  COL_BG);
        oledC_DrawThickPoint(CLK_CX, CLK_CY, 2u, COL_BG);
    }

    /* Restore any tick marks that were erased by the hands above */
    draw_analog_ticks();

    /* Draw new hands */
    float sec_a  = TWO_PI * (float)g_sec  / 60.0f;
    float min_a  = TWO_PI * ((float)g_min  + (float)g_sec  / 60.0f) / 60.0f;
    float hval   = (float)(g_hour % 12u)  + (float)g_min  / 60.0f;
    float hour_a = TWO_PI * hval / 12.0f;
    draw_hand(CLK_CX, CLK_CY, H_HOUR, hour_a, COL_TEXT);
    draw_hand(CLK_CX, CLK_CY, H_MIN,  min_a,  COL_TEXT);
    draw_hand(CLK_CX, CLK_CY, H_SEC,  sec_a,  COL_TEXT);
    oledC_DrawThickPoint(CLK_CX, CLK_CY, 2u, COL_TEXT);

    s_hand_sec  = g_sec;
    s_hand_min  = g_min;
    s_hand_hour = g_hour;
}

static void render_clock(void)
{
    oledC_setBackground(COL_BG);
    oledC_clearScreen();
    if (g_disp == DISP_DIGITAL) {
        /* Invalidate stored analog state so partial update starts fresh next time */
        s_hand_sec = 0xFFu;
        render_digital();
    } else {
        render_analog();   /* stores current hand positions into s_hand_* */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ALARM SCREEN RENDERER
 * Blinks by alternating background colour every second (g_blink_state).
 * ═══════════════════════════════════════════════════════════════════════════ */
static void render_alarm(void)
{
    if (g_blink_state) {
        oledC_setBackground(COL_ALARM_BG);
    } else {
        oledC_setBackground(COL_BG);
    }
    oledC_clearScreen();

    oledC_DrawString(8u,  20u, 2u, 2u, (uint8_t *)"ALARM!",    COL_ALARM_TEXT);

    char tbuf[8];
    snprintf(tbuf, sizeof(tbuf), "%02u:%02u", g_al_hour, g_al_min);
    oledC_DrawString(24u, 52u, 1u, 1u, (uint8_t *)tbuf,        COL_ALARM_TEXT);

    oledC_DrawString(2u,  72u, 1u, 1u, (uint8_t *)"btn/shake", COL_ALARM_TEXT);
    oledC_DrawString(2u,  82u, 1u, 1u, (uint8_t *)"to stop",   COL_ALARM_TEXT);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MENU RENDERER
 * Each MenuSt has its own layout.  The live corner clock is always present.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void render_menu(void)
{
    oledC_setBackground(COL_BG);
    oledC_clearScreen();

    draw_corner_clock();    /* live clock always visible in menus */

    switch (g_menu_st) {

    /* ─── MAIN MENU ─── */
    case MENU_MAIN: {
        /* Items: scale-1 text (8 px tall), 11 px row pitch, start at y=10.
         * Selected row: filled white box drawn first, then text in black.
         * Other rows: white text on black. */
        uint8_t sel_y = (uint8_t)(10u + g_menu_cur * 11u);
        oledC_DrawRectangle(0u, sel_y - 2u, 95u, sel_y + 9u, COL_TEXT);

        for (uint8_t i = 0u; i < MENU_ITEM_COUNT; i++) {
            uint8_t y   = (uint8_t)(10u + i * 11u);
            if (y > 88u) break;
            uint16_t col = (i == g_menu_cur) ? COL_BG : COL_MENU_ITEM;
            oledC_DrawString(4u, y, 1u, 1u,
                             (uint8_t *)k_menu_labels[i], col);
        }
        break;
    }

    /* ─── DISPLAY MODE ─── */
    case MENU_DISP_MODE:
        oledC_DrawString(0u, 12u, 1u, 1u, (uint8_t *)"Display Mode:", COL_MENU_HDR);
        oledC_DrawString(4u, 30u, 1u, 1u,
            (uint8_t *)((g_edit_val == 0u) ? "[Digital]" : " Digital "),
            (g_edit_val == 0u) ? COL_MENU_SEL : COL_MENU_ITEM);
        oledC_DrawString(4u, 44u, 1u, 1u,
            (uint8_t *)((g_edit_val == 1u) ? "[Analog] " : " Analog  "),
            (g_edit_val == 1u) ? COL_MENU_SEL : COL_MENU_ITEM);
        oledC_DrawString(0u, 82u, 1u, 1u, (uint8_t *)"S1:toggle S2:set", COL_FACE);
        break;

    /* ─── TIME FORMAT ─── */
    case MENU_TIME_FMT:
        oledC_DrawString(0u, 12u, 1u, 1u, (uint8_t *)"Time Format:", COL_MENU_HDR);
        oledC_DrawString(4u, 30u, 1u, 1u,
            (uint8_t *)((g_edit_val == 0u) ? "[24h]" : " 24h "),
            (g_edit_val == 0u) ? COL_MENU_SEL : COL_MENU_ITEM);
        oledC_DrawString(4u, 44u, 1u, 1u,
            (uint8_t *)((g_edit_val == 1u) ? "[12h]" : " 12h "),
            (g_edit_val == 1u) ? COL_MENU_SEL : COL_MENU_ITEM);
        oledC_DrawString(0u, 82u, 1u, 1u, (uint8_t *)"S1:toggle S2:set", COL_FACE);
        break;

    /* ─── SET TIME  (H → M → S, box highlights active field) ─── */
    case MENU_TIME_H:
    case MENU_TIME_M:
    case MENU_TIME_S: {
        uint8_t active = (uint8_t)(g_menu_st - MENU_TIME_H);   /* 0=H 1=M 2=S */

        /* Values: active field comes from g_edit_val; committed fields from globals */
        uint8_t hv = (active == 0u) ? g_edit_val : g_hour;
        uint8_t mv = (active == 1u) ? g_edit_val : g_min;
        uint8_t sv = (active == 2u) ? g_edit_val : g_sec;

        char fh[4], fm[4], fs[4];
        snprintf(fh, sizeof(fh), "%02u", hv);
        snprintf(fm, sizeof(fm), "%02u", mv);
        snprintf(fs, sizeof(fs), "%02u", sv);

        /* Title */
        oledC_DrawString(20u, 14u, 1u, 1u, (uint8_t *)"SET TIME", COL_MENU_HDR);

        /* Up arrow  (tip points up at y=30, shaft to y=42) */
        oledC_DrawLine(5u, 30u, 1u, 35u, 1u, COL_TEXT);
        oledC_DrawLine(5u, 30u, 9u, 35u, 1u, COL_TEXT);
        oledC_DrawLine(5u, 30u, 5u, 42u, 1u, COL_TEXT);

        /* Down arrow (tip points down at y=68, shaft from y=56) */
        oledC_DrawLine(5u, 68u, 1u, 63u, 1u, COL_TEXT);
        oledC_DrawLine(5u, 68u, 9u, 63u, 1u, COL_TEXT);
        oledC_DrawLine(5u, 56u, 5u, 68u, 1u, COL_TEXT);

        /* Field X anchors (scale-2 → each 2-digit field is 24 px wide) */
        static const uint8_t fx[3] = { 12u, 40u, 68u };
        const uint8_t fy = 44u;
        const char * const fv[3] = { fh, fm, fs };

        for (uint8_t i = 0u; i < 3u; i++) {
            uint16_t col = (i == active) ? COL_TEXT : COL_FACE;
            oledC_DrawString(fx[i], fy, 2u, 2u, (uint8_t *)fv[i], col);
        }

        /* White outline box around the active field (DrawRectangle is filled,
         * so use four lines to get a border-only look) */
        {
            uint8_t bx0 = fx[active] - 2u,  bx1 = fx[active] + 24u;
            uint8_t by0 = fy - 2u,           by1 = fy + 16u;
            oledC_DrawLine(bx0, by0, bx1, by0, 1u, COL_TEXT);
            oledC_DrawLine(bx0, by1, bx1, by1, 1u, COL_TEXT);
            oledC_DrawLine(bx0, by0, bx0, by1, 1u, COL_TEXT);
            oledC_DrawLine(bx1, by0, bx1, by1, 1u, COL_TEXT);
        }

        oledC_DrawString(4u, 82u, 1u, 1u, (uint8_t *)"S1:+1  S2:next", COL_FACE);
        break;
    }

    /* ─── SET DATE  (D → M, box highlights active field) ─── */
    case MENU_DATE_D:
    case MENU_DATE_M: {
        uint8_t active = (uint8_t)(g_menu_st - MENU_DATE_D);   /* 0=D 1=M */

        uint8_t dv = (active == 0u) ? g_edit_val : g_day;
        uint8_t mv = (active == 1u) ? g_edit_val : g_month;

        char fd[4], fm[4];
        snprintf(fd, sizeof(fd), "%02u", dv);
        snprintf(fm, sizeof(fm), "%02u", mv);

        oledC_DrawString(20u, 14u, 1u, 1u, (uint8_t *)"SET DATE", COL_MENU_HDR);

        /* Up / down arrows */
        oledC_DrawLine(5u, 30u, 1u, 35u, 1u, COL_TEXT);
        oledC_DrawLine(5u, 30u, 9u, 35u, 1u, COL_TEXT);
        oledC_DrawLine(5u, 30u, 5u, 42u, 1u, COL_TEXT);
        oledC_DrawLine(5u, 68u, 1u, 63u, 1u, COL_TEXT);
        oledC_DrawLine(5u, 68u, 9u, 63u, 1u, COL_TEXT);
        oledC_DrawLine(5u, 56u, 5u, 68u, 1u, COL_TEXT);

        static const uint8_t dfx[2] = { 20u, 56u };
        const uint8_t dfy = 44u;
        const char * const dfv[2] = { fd, fm };

        for (uint8_t i = 0u; i < 2u; i++) {
            uint16_t col = (i == active) ? COL_TEXT : COL_FACE;
            oledC_DrawString(dfx[i], dfy, 2u, 2u, (uint8_t *)dfv[i], col);
        }
        {
            uint8_t bx0 = dfx[active] - 2u,  bx1 = dfx[active] + 24u;
            uint8_t by0 = dfy - 2u,           by1 = dfy + 16u;
            oledC_DrawLine(bx0, by0, bx1, by0, 1u, COL_TEXT);
            oledC_DrawLine(bx0, by1, bx1, by1, 1u, COL_TEXT);
            oledC_DrawLine(bx0, by0, bx0, by1, 1u, COL_TEXT);
            oledC_DrawLine(bx1, by0, bx1, by1, 1u, COL_TEXT);
        }

        oledC_DrawString(4u, 82u, 1u, 1u, (uint8_t *)"S1:+1  S2:next", COL_FACE);
        break;
    }

    /* ─── SET ALARM  (H → M, box highlights active field) ─── */
    case MENU_ALARM_H:
    case MENU_ALARM_M: {
        uint8_t active = (uint8_t)(g_menu_st - MENU_ALARM_H);  /* 0=H 1=M */

        uint8_t hv = (active == 0u) ? g_edit_val : g_al_hour;
        uint8_t mv = (active == 1u) ? g_edit_val : g_al_min;

        char fh[4], fm[4];
        snprintf(fh, sizeof(fh), "%02u", hv);
        snprintf(fm, sizeof(fm), "%02u", mv);

        oledC_DrawString(16u, 14u, 1u, 1u, (uint8_t *)"SET ALARM", COL_MENU_HDR);

        /* Up / down arrows */
        oledC_DrawLine(5u, 30u, 1u, 35u, 1u, COL_ALARM_ICON);
        oledC_DrawLine(5u, 30u, 9u, 35u, 1u, COL_ALARM_ICON);
        oledC_DrawLine(5u, 30u, 5u, 42u, 1u, COL_ALARM_ICON);
        oledC_DrawLine(5u, 68u, 1u, 63u, 1u, COL_ALARM_ICON);
        oledC_DrawLine(5u, 68u, 9u, 63u, 1u, COL_ALARM_ICON);
        oledC_DrawLine(5u, 56u, 5u, 68u, 1u, COL_ALARM_ICON);

        static const uint8_t afx[2] = { 20u, 56u };
        const uint8_t afy = 44u;
        const char * const afv[2] = { fh, fm };

        for (uint8_t i = 0u; i < 2u; i++) {
            uint16_t col = (i == active) ? COL_TEXT : COL_FACE;
            oledC_DrawString(afx[i], afy, 2u, 2u, (uint8_t *)afv[i], col);
        }
        {
            uint8_t bx0 = afx[active] - 2u,  bx1 = afx[active] + 24u;
            uint8_t by0 = afy - 2u,           by1 = afy + 16u;
            oledC_DrawLine(bx0, by0, bx1, by0, 1u, COL_ALARM_ICON);
            oledC_DrawLine(bx0, by1, bx1, by1, 1u, COL_ALARM_ICON);
            oledC_DrawLine(bx0, by0, bx0, by1, 1u, COL_ALARM_ICON);
            oledC_DrawLine(bx1, by0, bx1, by1, 1u, COL_ALARM_ICON);
        }

        oledC_DrawString(4u, 82u, 1u, 1u, (uint8_t *)"S1:+1  S2:next", COL_FACE);
        break;
    }

    /* ─── DISABLE ALARM ─── */
    case MENU_ALARM_OFF:
        oledC_DrawString(0u, 20u, 1u, 1u, (uint8_t *)"Disable Alarm?", COL_ALARM_ICON);
        oledC_DrawString(0u, 44u, 1u, 1u, (uint8_t *)"S2: Yes",        COL_MENU_SEL);
        oledC_DrawString(0u, 58u, 1u, 1u, (uint8_t *)"S1: No / back",  COL_MENU_ITEM);
        break;

    default: break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MENU STATE MACHINE HELPERS
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Enter a submenu: switch state and pre-load current setting into g_edit_val */
static void menu_enter_state(MenuSt st)
{
    g_menu_st = st;
    switch (st) {
    case MENU_DISP_MODE: g_edit_val = (g_disp == DISP_DIGITAL) ? 0u : 1u; break;
    case MENU_TIME_FMT:  g_edit_val = (g_fmt  == FMT_24H)      ? 0u : 1u; break;
    case MENU_TIME_H:    g_edit_val = g_hour;      break;
    case MENU_TIME_M:    g_edit_val = g_min;       break;
    case MENU_TIME_S:    g_edit_val = g_sec;       break;
    case MENU_DATE_D:    g_edit_val = g_day;       break;
    case MENU_DATE_M:    g_edit_val = g_month;     break;
    case MENU_ALARM_H:   g_edit_val = g_al_hour;   break;
    case MENU_ALARM_M:   g_edit_val = g_al_min;    break;
    default: break;
    }
}

/* Leave menu entirely → return to clock */
static void menu_exit(void)
{
    g_mode     = MODE_CLOCK;
    g_menu_st  = MENU_MAIN;
    g_menu_cur = 0u;
}

/*
 * S1 short-press inside menu:
 *   MENU_MAIN → move cursor down (wraps)
 *   submenus  → increment edit value (wraps)
 *   confirm dialogs → cancel / go back
 */
static void menu_s1_press(void)
{
    switch (g_menu_st) {
    case MENU_MAIN:
        g_menu_cur = (g_menu_cur + 1u) % MENU_ITEM_COUNT;
        break;

    case MENU_DISP_MODE:
    case MENU_TIME_FMT:
        g_edit_val ^= 1u;           /* toggle 0 ↔ 1 */
        break;

    case MENU_TIME_H:
    case MENU_ALARM_H:
        g_edit_val = (g_edit_val + 1u) % 24u;
        break;

    case MENU_TIME_M:
    case MENU_TIME_S:
    case MENU_ALARM_M:
        g_edit_val = (g_edit_val + 1u) % 60u;
        break;

    case MENU_DATE_D: {
        /* Wrap within valid days for current (or editing) month */
        uint8_t mx = k_days[g_month - 1u];
        g_edit_val = (g_edit_val % mx) + 1u;
        break;
    }
    case MENU_DATE_M:
        g_edit_val = (g_edit_val % 12u) + 1u;
        break;

    case MENU_ALARM_OFF:
        /* S1 = cancel – go back to main menu without disabling */
        g_menu_st = MENU_MAIN;
        break;

    default: break;
    }
}

/*
 * S2 short-press inside menu:
 *   MENU_MAIN → enter the highlighted submenu
 *   submenus  → commit value and advance to next editing step (or back to MAIN)
 */
static void menu_s2_press(void)
{
    switch (g_menu_st) {

    /* ── Enter selected submenu ── */
    case MENU_MAIN:
        switch (g_menu_cur) {
        case 0u: menu_enter_state(MENU_DISP_MODE); break;
        case 1u: menu_enter_state(MENU_TIME_FMT);  break;
        case 2u: menu_enter_state(MENU_TIME_H);    break;
        case 3u: menu_enter_state(MENU_DATE_D);    break;
        case 4u: menu_enter_state(MENU_ALARM_H);   break;
        case 5u: menu_enter_state(MENU_ALARM_OFF); break;
        default: break;
        }
        break;

    /* ── Apply settings and advance ── */
    case MENU_DISP_MODE:
        g_disp    = (g_edit_val == 0u) ? DISP_DIGITAL : DISP_ANALOG;
        g_menu_st = MENU_MAIN;
        break;

    case MENU_TIME_FMT:
        g_fmt     = (g_edit_val == 0u) ? FMT_24H : FMT_12H;
        g_menu_st = MENU_MAIN;
        break;

    case MENU_TIME_H:
        g_hour = g_edit_val;
        menu_enter_state(MENU_TIME_M);
        break;

    case MENU_TIME_M:
        g_min = g_edit_val;
        menu_enter_state(MENU_TIME_S);
        break;

    case MENU_TIME_S:
        g_sec     = g_edit_val;
        g_menu_st = MENU_MAIN;
        break;

    case MENU_DATE_D:
        g_day = g_edit_val;
        menu_enter_state(MENU_DATE_M);
        break;

    case MENU_DATE_M:
        g_month   = g_edit_val;
        g_menu_st = MENU_MAIN;
        break;

    case MENU_ALARM_H:
        g_al_hour = g_edit_val;
        menu_enter_state(MENU_ALARM_M);
        break;

    case MENU_ALARM_M:
        g_al_min     = g_edit_val;
        g_al_enabled = true;            /* activates the alarm */
        g_menu_st    = MENU_MAIN;
        break;

    case MENU_ALARM_OFF:
        g_al_enabled = false;
        g_al_ringing = false;
        g_menu_st    = MENU_MAIN;
        break;

    default:
        g_menu_st = MENU_MAIN;
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MAIN
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    /* ── Hardware + peripheral init ── */
    SYSTEM_Initialize();
    hw_init();
    oledC_setup();
    oledC_setBackground(COL_BG);
    oledC_clearScreen();
    i2c1_open();
    accel_init();
    adc_init();
    timer1_init();          /* starts 1-second ISR */

    /* ── Button edge-detection state ── */
    bool s1_prev = false;
    bool s2_prev = false;

    /* ── Long-press tracking for S1 (time-based, not iteration-based) ── */
    uint16_t s1_press_start = 0u;   /* g_uptime_secs snapshot when S1 went down */
    bool     s1_long_fired  = false; /* prevent re-firing while held */

    /* ── Redraw flag: set whenever state changes ── */
    bool need_draw = true;

    /* ──────────────────────────────────────────────────────────────────────
     * MAIN LOOP  (~10 ms / iteration)
     * ────────────────────────────────────────────────────────────────────── */
    while (1) {

        /* ── 1. CONSUME SECOND TICK ───────────────────────────────────── */
        if (g_tick) {
            g_tick = false;
            rtc_advance();
            alarm_check();

            /* Count seconds while alarm is ringing; auto-stop after limit */
            if (g_al_ringing) {
                if (++g_al_secs >= ALARM_AUTO_STOP) {
                    alarm_stop();
                }
            }

            if (g_mode == MODE_MENU) {
                /* Menu: just overwrite the corner clock text — no screen clear,
                 * no flicker.  Full redraw only happens on button presses. */
                draw_corner_clock();
            } else if (g_mode == MODE_CLOCK && g_disp == DISP_ANALOG) {
                /* Analog clock: erase old hands, redraw ticks, draw new hands.
                 * No screen clear → no flash. */
                render_analog_update();
            } else {
                need_draw = true;   /* digital / alarm screens: full redraw */
            }
        }

        /* ── 2. READ BUTTONS ──────────────────────────────────────────── */
        bool s1 = S1_PRESSED();
        bool s2 = S2_PRESSED();

        bool s1_rising = (s1 && !s1_prev);
        bool s2_rising = (s2 && !s2_prev);

        /* Long-press: exactly LONG_PRESS_SECS seconds, ISR-timed.
         * Record the second-count when S1 first goes down; fire once when
         * the elapsed seconds reach the threshold; reset on release. */
        if (s1_rising) {
            s1_press_start = g_uptime_secs;
            s1_long_fired  = false;
        } else if (!s1) {
            s1_long_fired = false;
        }
        bool s1_long = false;
        if (s1 && !s1_long_fired &&
            (g_uptime_secs - s1_press_start) >= LONG_PRESS_SECS)
        {
            s1_long       = true;
            s1_long_fired = true;
        }

        /* ── 3. READ POTENTIOMETER (AN8 / RB12) ──────────────────────── */
        uint16_t pot = adc_read_pot();   /* 0–1023 */

        /* ── 4. READ ACCELEROMETER ────────────────────────────────────── */
        bool shaken  = accel_shaken();
        bool flipped = accel_flipped();

        /* ── 5. STATE MACHINE ─────────────────────────────────────────── */
        switch (g_mode) {

        /* ============================================================
         * CLOCK MODE
         * ============================================================ */
        case MODE_CLOCK:
            /* LED feedback */
            if (s1) LED1_ON(); else LED1_OFF();
            if (s2) LED2_ON(); else LED2_OFF();

            /* Long-press S1  →  enter menu */
            if (s1_long) {
                g_mode     = MODE_MENU;
                g_menu_st  = MENU_MAIN;
                g_menu_cur = 0u;
                need_draw  = true;
            }
            break;

        /* ============================================================
         * MENU MODE
         * ============================================================ */
        case MODE_MENU:
            /* LED feedback: on while the corresponding button is held */
            if (s1) LED1_ON(); else LED1_OFF();
            if (s2) LED2_ON(); else LED2_OFF();

            /* Shake or flip  →  exit menu immediately */
            if (shaken || flipped) {
                menu_exit();
                LED1_OFF();  LED2_OFF();
                need_draw = true;
                break;
            }

            /* ── Potentiometer: live value control (Set Time / Set Alarm only) ── */
            {
                uint8_t nv = 0u;
                switch (g_menu_st) {

                case MENU_TIME_H:
                case MENU_ALARM_H:
                    nv = (uint8_t)((uint32_t)pot * 24u / 1024u);
                    if (nv != g_edit_val) { g_edit_val = nv; need_draw = true; }
                    break;

                case MENU_TIME_M:
                case MENU_TIME_S:
                case MENU_ALARM_M:
                    nv = (uint8_t)((uint32_t)pot * 60u / 1024u);
                    if (nv != g_edit_val) { g_edit_val = nv; need_draw = true; }
                    break;

                default: break;
                }
            }

            /* S1 short-press  →  fine +1 nudge (useful when pot is hard to position precisely) */
            if (s1_rising) {
                menu_s1_press();
                need_draw = true;
            }

            /* S2 short-press  →  confirm / advance to next field */
            if (s2_rising) {
                menu_s2_press();
                need_draw = true;
            }
            break;

        /* ============================================================
         * ALARM MODE
         * ============================================================ */
        case MODE_ALARM:
            /* LED feedback */
            if (s1) LED1_ON(); else LED1_OFF();
            if (s2) LED2_ON(); else LED2_OFF();

            /* Any button press, shake, or flip  →  stop alarm */
            if (s1_rising || s2_rising || shaken || flipped) {
                alarm_stop();
                need_draw = true;
            }
            break;
        }

        /* ── 5. RENDER (only when something changed) ──────────────────── */
        if (need_draw) {
            need_draw = false;
            switch (g_mode) {
            case MODE_CLOCK: render_clock(); break;
            case MODE_MENU:  render_menu();  break;
            case MODE_ALARM: render_alarm(); break;
            }
        }

        /* ── 6. UPDATE EDGE-DETECTION STATE + PACE LOOP ──────────────── */
        s1_prev = s1;
        s2_prev = s2;

        DELAY_milliseconds(10);     /* ~100 Hz loop rate; long-press is ISR-timed */
    }

    return 0;
}
/* End of File */
