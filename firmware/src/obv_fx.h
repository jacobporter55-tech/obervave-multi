/* SPDX-License-Identifier: GPL-3.0-only */
/* The effects after the output stage: a stereo chorus, a shimmer (an octave-up pitch shifter that feeds the
 * reverb and is fed back from its tail; it took the delay's place, memory and time), and a
 * plate reverb (J. Dattorro's figure-of-eight tank, 1997). All lines are int16 in one .pool buffer; the
 * signals are Q15 as the output stage leaves them. Integers only.
 * REV_DIV 1: the reverb at the full rate (the sound comes first). 2: at half the rate, if the chip turns out
 * to need it: a dark hall loses little by it and it halves the reverb's time and memory. */
#ifndef REV_DIV
#define REV_DIV 2u
#endif
#define RVL(x) ((uint32_t)((x) * 44100u / 29761u / REV_DIV))   /* the paper's 29761 Hz lengths at our rate */
#ifndef SH_DECAY
#define SH_DECAY 30500                          /* 0.93 */
#endif
#ifndef SH_FB
#define SH_FB 26000
#endif
#ifndef SH_TONE
#define SH_TONE 150
#endif
#ifndef SH_DAMP
#define SH_DAMP 29000
#endif
#define SH_N 4096u                              /* the shimmer's line: 93 ms */
#define SH_W 2048u                              /* a grain: 46 ms, two heads half a grain apart */
#define CHO_N 1024u
#define REV_PRE_N (8192u / REV_DIV)             /* up to 120 ms of pre-delay */
#define RV_IN1 RVL(142)
#define RV_IN2 RVL(107)
#define RV_IN3 RVL(379)
#define RV_IN4 RVL(277)
#define RV_AP1 RVL(672)
#define RV_D1 RVL(4453)
#define RV_AP2 RVL(1800)
#define RV_D2 RVL(3720)
#define RV_AP3 RVL(908)
#define RV_D3 RVL(4217)
#define RV_AP4 RVL(2656)
#define RV_D4 RVL(3163)
#define REV_N (REV_PRE_N + RV_IN1 + RV_IN2 + RV_IN3 + RV_IN4 + RV_AP1 + 16u + RV_D1 + RV_AP2 + RV_D2 + RV_AP3 + 16u + RV_D3 + RV_AP4 + RV_D4)
#define RV_BW (REV_DIV == 2u ? 24000 : 16700)   /* the input's one-pole: ~5 kHz either way */
#define FX_MEM_N (CHO_N + SH_N + REV_N)

typedef struct { int16_t *b; uint32_t len, w; } line_t;   /* w: where the next sample goes */
static int16_t fx_mem[FX_MEM_N] __attribute__((section(".pool")));
static uint32_t fx_used;

static inline int32_t sat16(int32_t v) { return v > 32767 ? 32767 : v < -32767 ? -32767 : v; }
static void line_init(line_t *l, uint32_t len) { l->b = fx_mem + fx_used; l->len = len; l->w = 0; fx_used += len; }
static inline __attribute__((always_inline)) int32_t line_read(const line_t *l, uint32_t back)   /* 1 <= back <= len */
{
    uint32_t i = l->w + l->len - back;
    if (i >= l->len) i -= l->len;
    return l->b[i];
}
static inline __attribute__((always_inline)) void line_write(line_t *l, int32_t v)
{
    l->b[l->w] = (int16_t)sat16(v);
    if (++l->w >= l->len) l->w = 0;
}
/* Dattorro's allpass: the line holds the inner node, the output is tapped from it. g Q15, signed */
static inline __attribute__((always_inline)) int32_t ap_step(line_t *l, int32_t in, int32_t g)
{
    int32_t d = line_read(l, l->len), v = in - ((d * g) >> 15);
    line_write(l, v);
    return d + ((v * g) >> 15);
}
/* the same with the delay swaying by e (Q8 samples, |e| <= 8 << 8) around base; the line is base + 16 long */
static inline __attribute__((always_inline)) int32_t ap_mod(line_t *l, int32_t in, int32_t g, uint32_t base, int32_t e)
{
    uint32_t back = (uint32_t)((int32_t)base + (e >> 8));
    int32_t f = e & 255, a = line_read(l, back), b = line_read(l, back + 1u), d = a + (((b - a) * f) >> 8), v;
    v = in - ((d * g) >> 15);
    line_write(l, v);
    return d + ((v * g) >> 15);
}
static inline int32_t tri_q(uint32_t ph)        /* a triangle -2048..2047 (so Q8 = +-8 samples) */
{
    int32_t t = (int32_t)(ph >> 19);            /* 0..8191 */
    return t < 4096 ? t - 2048 : 6143 - t;
}

/* ---- the state */
static line_t cho, shl, rv_pre, rv_in[4], rv_ap1, rv_d1, rv_ap2, rv_d2, rv_ap3, rv_d3, rv_ap4, rv_d4;
static uint32_t cho_ph, cho_inc, rv_ph1, rv_ph2;
static int32_t cho_depth_k, cho_mix, sh_mix, rev_mix, sh_fb, sh_tone_k, rev_decay, rev_damp_k;
static int32_t sh_lp, sh_hp, sh_out;            /* the shifted signal's tone (a one-pole low-pass, a DC block), its last sample */
static int32_t sh_env, sh_fb_now, sh_in_lp;               /* the shifted level; BLOOM as this call applies it (ridden by the level) */
static uint32_t sh_ph;                          /* 0 .. SH_W - 1: head A's place in its grain */
static int32_t rv_lp_in, rv_lpl, rv_lpr, rv_yl, rv_yr;
#if REV_DIV == 2u
static int32_t rv_hold;
#endif
static uint32_t rev_pre_n = 1, sh_active, rev_active;
#if REV_DIV == 2u
static uint32_t fx_odd;
#endif

static void fx_init(void)
{
    line_init(&cho, CHO_N);
    line_init(&shl, SH_N);
    line_init(&rv_pre, REV_PRE_N);
    line_init(&rv_in[0], RV_IN1); line_init(&rv_in[1], RV_IN2); line_init(&rv_in[2], RV_IN3); line_init(&rv_in[3], RV_IN4);
    line_init(&rv_ap1, RV_AP1 + 16u); line_init(&rv_d1, RV_D1); line_init(&rv_ap2, RV_AP2); line_init(&rv_d2, RV_D2);
    line_init(&rv_ap3, RV_AP3 + 16u); line_init(&rv_d3, RV_D3); line_init(&rv_ap4, RV_AP4); line_init(&rv_d4, RV_D4);
}

/* the patch's numbers, once per ISR call. quarter: frames per quarter note (the arp's clock) */
/* the effects' sound comes from part 1's patch; how much of each part goes in is that part's MIX (a send, from the
 * voices: obv_engine.h). any_*: some part sends to it. */
static void fx_block(const patch_t *p, uint32_t quarter, uint32_t any_c, uint32_t any_d, uint32_t any_r)
{
    int32_t d, t;
    (void)quarter;
    d = p->v[P_CHO_RATE]; cho_inc = (uint32_t)(10 + ((d * d * 590) >> 14)) * 974u;   /* 0.1 .. 5.9 Hz */
    cho_depth_k = p->v[P_CHO_DEPTH] * 67;       /* x the triangle's +-2048 = Q16 samples: up to +-6 ms */
    cho_mix = any_c != 0;                       /* the sends carry the levels: the returns are at full */
    sh_mix = any_d != 0;                        /* P_DLY_MIX is each part's SHIMMER send now */
    rev_mix = any_r != 0;
    /* JPad: the shimmer hall's sound from the drone's (part 3's) SHIMMER page: DECAY, BLOOM, TONE */
    d = p->v[P_DLY_FB]; sh_fb = (d * d * 32) >> 4;          /* BLOOM: 0 .. ~0.7 back into the shifter (ridden down as it grows) */
    t = p->v[P_DLY_TONE]; sh_tone_k = clampi(((t * t) >> 6) + 6, 6, 256);   /* TONE: dark .. bright octave */
    d = p->v[P_REV_SIZE]; rev_decay = 20000 + ((d * d * 11600) >> 14);   /* DECAY: 0.61 .. 0.96 (about 2 s .. 30 s) */
    rev_damp_k = SH_DAMP;                       /* the one-pole's k at the half rate */
    if (REV_DIV == 1u) {                        /* the same corner at the full rate: k' = 1 - sqrt(1 - k) */
        uint32_t x = (uint32_t)(32768 - rev_damp_k) << 15, r = 0, bit = 1u << 15;
        for (; bit; bit >>= 1)
            if ((r + bit) * (r + bit) <= x) r += bit;
        rev_damp_k = 32768 - (int32_t)r;
    }
    rev_pre_n = 1u + 30u * (41u / REV_DIV);      /* ~28 ms */
    if (sh_mix) sh_active = 24u * FS; else if (sh_active > HALF_FRAMES) sh_active -= HALF_FRAMES; else sh_active = 0;
    if (rev_mix || sh_active) rev_active = 24u * FS; else if (rev_active > HALF_FRAMES) rev_active -= HALF_FRAMES; else rev_active = 0;
}

static inline __attribute__((always_inline)) int32_t line_read_q16(const line_t *l, int32_t back_q16)
{
    uint32_t b = (uint32_t)back_q16 >> 16;
    int32_t f = (back_q16 >> 8) & 255, a = line_read(l, b), c = line_read(l, b + 1u);
    return a + (((c - a) * f) >> 8);
}
/* 4-point Hermite for a tap that is always moving (the chorus): no treble loss between the samples */
static inline __attribute__((always_inline)) int32_t line_read_q16_h(const line_t *l, int32_t back_q16)
{
    uint32_t b = (uint32_t)back_q16 >> 16;
    int32_t f = (back_q16 >> 8) & 255, p0 = line_read(l, b - 1u), p1 = line_read(l, b), p2 = line_read(l, b + 1u), p3 = line_read(l, b + 2u);
    int32_t c1 = (p2 - p0) >> 1, c2 = p0 - ((5 * p1) >> 1) + 2 * p2 - (p3 >> 1), c3 = ((p3 - p0) >> 1) + ((3 * (p1 - p2)) >> 1);
    return p1 + ((f * (c1 + ((f * (c2 + ((f * c3) >> 8))) >> 8))) >> 8);
}

/* one reverb sample at the half rate: in -> pre-delay -> bandwidth -> 4 diffusers -> the two tank halves */
static inline void rev_tick(int32_t in)
{
    int32_t x, a, b, e1 = tri_q(rv_ph1), e2 = tri_q(rv_ph2), dec = rev_decay, dk = rev_damp_k;
    rv_ph1 += 97391u * REV_DIV;                 /* ~1.0 Hz and ~0.7 Hz: the tank's sway against metallic rings */
    rv_ph2 += 68174u * REV_DIV;
    line_write(&rv_pre, in);
    x = line_read(&rv_pre, rev_pre_n);
    rv_lp_in += ((x - rv_lp_in) * RV_BW) >> 15;
    x = ap_step(&rv_in[0], rv_lp_in, 24576);
    x = ap_step(&rv_in[1], x, 24576);
    x = ap_step(&rv_in[2], x, 20480);
    x = ap_step(&rv_in[3], x, 20480);
    /* left half, fed by the right's end */
    a = x + ((line_read(&rv_d4, RV_D4) * dec) >> 15);
    a = ap_mod(&rv_ap1, a, -22938, RV_AP1, e1);
    line_write(&rv_d1, a);
    b = line_read(&rv_d1, RV_D1);
    rv_lpl += ((b - rv_lpl) * dk) >> 15;
    b = ap_step(&rv_ap2, (rv_lpl * dec) >> 15, 16384);
    line_write(&rv_d2, b);
    /* right half, fed by the left's end */
    a = x + ((line_read(&rv_d2, RV_D2) * dec) >> 15);
    a = ap_mod(&rv_ap3, a, -22938, RV_AP3, e2);
    line_write(&rv_d3, a);
    b = line_read(&rv_d3, RV_D3);
    rv_lpr += ((b - rv_lpr) * dk) >> 15;
    b = ap_step(&rv_ap4, (rv_lpr * dec) >> 15, 16384);
    line_write(&rv_d4, b);
    /* the output taps (the paper's), x 0.6 */
    a = line_read(&rv_d3, RVL(266)) + line_read(&rv_d3, RVL(2974)) - line_read(&rv_ap4, RVL(1913)) + line_read(&rv_d4, RVL(1996))
      - line_read(&rv_d1, RVL(1990)) - line_read(&rv_ap2, RVL(187)) - line_read(&rv_d2, RVL(1066));
    b = line_read(&rv_d1, RVL(353)) + line_read(&rv_d1, RVL(3627)) - line_read(&rv_ap2, RVL(1228)) + line_read(&rv_d2, RVL(2673))
      - line_read(&rv_d3, RVL(2111)) - line_read(&rv_ap4, RVL(335)) - line_read(&rv_d4, RVL(121));
    rv_yl = (a * 19661) >> 15;
    rv_yr = (b * 19661) >> 15;
}

/* the last stage: up to 3/4 of full scale untouched, above it a tanh knee into the top quarter (never a hard clip:
 * the mix of six parts and three effects can go over) */
static inline int32_t limit16(int32_t x)
{
    int32_t a = x < 0 ? -x : x, y;
    if (a <= 24576) return x;
    y = 24576 + (softclip(clampi(a - 24576, 0, 16383) * 4) >> 2);
    return x < 0 ? -y : y;
}
/* the output limiter: per 16 frames it looks at the block's peak first, so the gain is already down when a loud
 * block starts (no overshoot); it comes back up over ~50 ms. Below LIM_CEIL nothing changes: gain 1.0 exactly. */
#define LIM_CEIL 29000
static int32_t lim_g = 32768;                   /* Q15 */
static void lim_process(int32_t *out, uint32_t n)
{
    uint32_t b, i;
    for (b = 0; b < n; b += 16u) {
        int32_t peak = 0, target, g0 = lim_g, g1;
        uint32_t m = n - b < 16u ? n - b : 16u;
        for (i = 0; i < 2u * m; i++) {
            int32_t a = out[2u * b + i];
            if (a < 0) a = -a;
            if (a > peak) peak = a;
        }
        target = peak > LIM_CEIL ? (int32_t)(((uint32_t)LIM_CEIL << 15) / (uint32_t)peak) : 32768;
        if (target < g0) g0 = g1 = target;      /* down: at once, from the block's first frame */
        else g1 = g0 + ((target - g0) >> 7) + (target > g0);   /* up: slowly (~50 ms) */
        lim_g = g1;
        for (i = 0; i < m; i++) {
            int32_t g = g0 + (((g1 - g0) * (int32_t)i) >> 4);
            out[2u * (b + i)] = limit16((int32_t)(((int64_t)out[2u * (b + i)] * g) >> 15));
            out[2u * (b + i) + 1u] = limit16((int32_t)(((int64_t)out[2u * (b + i) + 1u] * g) >> 15));
        }
    }
}
static void fx_process(int32_t *out, const int32_t *bc, const int32_t *bd, const int32_t *br, uint32_t n)
{
    uint32_t i, do_sh = sh_active != 0, do_rev = rev_active != 0;
    int32_t cm = cho_mix;
    sh_fb_now = (int32_t)((uint32_t)((sh_fb * (32768 - rev_decay)) >> 14) * 2048u / (2048u + (uint32_t)sh_env));   /* the louder the bloom, the less goes
                                                   round again: it cannot run away, even with the reverb near a freeze */
    for (i = 0; i < n; i++) {
        int32_t l = out[2u * i], r = out[2u * i + 1u], rin = softclip(br[i]);   /* the sends soft-clipped as the dry is:
                                                   a loud part cannot drive the effects (and the sum) into the hard limit */
        if (cm) {                               /* two taps a quarter turn apart on one line, around 12 ms */
            int32_t bl = (529 << 16) + tri_q(cho_ph) * cho_depth_k, br2 = (529 << 16) + tri_q(cho_ph + 0x40000000u) * cho_depth_k;
            cho_ph += cho_inc;
            line_write(&cho, softclip(bc[i]));            /* the send already took its share of the dry */
            l += line_read_q16_h(&cho, bl);
            r += line_read_q16_h(&cho, br2);
        }
        if (do_sh) {                            /* shimmer: two heads read twice as fast as the line is written (an
                                                   octave up), each faded in and out over its grain, half a grain apart */
            uint32_t pa = sh_ph, pb = pa + SH_W / 2u, ga, gb;
            int32_t a, b, y;
            if (pb >= SH_W) pb -= SH_W;
            int32_t x = softclip(bd[i]) + (((rv_yl + rv_yr) >> 1) * sh_fb_now >> 15);
            sh_in_lp += (x - sh_in_lp) >> 5;    /* a high-pass at ~220 Hz into the shifter: the grains' 21 Hz flutter
                                                   cannot pile up as rumble round the loop (it only adds the top) */
            line_write(&shl, x - sh_in_lp);
            a = line_read(&shl, SH_W + 1u - pa);  /* (the distance shrinks a sample a sample: 2x speed) */
            b = line_read(&shl, SH_W + 1u - pb);
            ga = pa < SH_W / 2u ? pa : SH_W - pa; gb = SH_W / 2u - ga;   /* triangles: ga + gb = SH_W / 2 */
            y = (a * (int32_t)ga + b * (int32_t)gb) >> 10;
            if (++sh_ph >= SH_W) sh_ph = 0;
            sh_lp += ((y - sh_lp) * sh_tone_k) >> 8;
            sh_hp += (sh_lp - sh_hp) >> 9;      /* (a DC block: the loop must not creep) */
            sh_out = sh_lp - sh_hp;
            { int32_t m = sh_out < 0 ? -sh_out : sh_out; sh_env += (m - sh_env) >> (m > sh_env ? 7 : 13); }
            rin += (softclip(bd[i]) >> 1) + sh_out + (sh_out >> 1);   /* into the plate: the send and its octave (x1.5) */
        }
        if (do_rev) {
#if REV_DIV == 2u                               /* half rate: a pair in, the output interpolated between ticks */
            int32_t pl = rv_yl, pr = rv_yr, din = 2 * rin;
            if (!(fx_odd ^= 1u)) {
                rev_tick((rv_hold + din) >> 2);
                l += (pl + rv_yl) >> 1;
                r += (pr + rv_yr) >> 1;
            } else {
                l += rv_yl;
                r += rv_yr;
            }
            rv_hold = din;
#else
            rev_tick(rin);
            l += rv_yl;
            r += rv_yr;
#endif
        }
        out[2u * i] = l;
        out[2u * i + 1u] = r;
    }
    lim_process(out, n);
}
