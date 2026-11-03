/* SPDX-License-Identifier: GPL-3.0-only */
/* The engine: 8 voices in the audio ISR. The main loop sends note events through a ring and writes the live
 * patch and the controllers; the ISR owns the voices, the held-note list, the arpeggiator and the sustain. */
#ifndef RING_PUBLISH
#define RING_PUBLISH() __asm__ volatile("" ::: "memory")
#endif
#define NV 8u
#define NPART 6u                                /* multitimbral: 6 parts share the 8 voices; MIDI ch 1-6 play them */
#define NHELD 16u
#define ENV_ONE (1 << 24)
#define LEVEL_FLOOR 2048                        /* -78 dB: a release below this frees the voice */

enum { E_IDLE, E_ATT, E_DEC, E_SUS, E_REL };
typedef struct { uint32_t stage; int32_t level; } env_t;   /* level Q24 */

typedef struct {
    osc_t o1, o2;
    uint32_t noise;
    env_t fe, ae;
    svf_state_t f1, f2;
    svf_coef_t c1, c2;
    uint32_t inc1, inc2, pw1, pw2;              /* this block's oscillator settings */
    int32_t amp0, amp1;                         /* Q15 at the block's start and end */
    int32_t pitch16, target16;                  /* Q16 semitones: the glide */
    int32_t det8;                               /* unison / dual offset, Q8 */
    int32_t gl, gr;                             /* pan gains Q12 */
    int32_t drift;                              /* the vintage random walk */
    uint32_t lfo_blk;                           /* blocks since the note on (LFO delay / fade) */
    int32_t lfo_fade;                           /* Q15 */
    uint32_t seq;                               /* allocation order */
    uint8_t note, vel, gate;                    /* gate: the key (or the arp) holds it */
    uint8_t part, sus;                          /* the part that plays it; sus: let go, the part's pedal holds it */
    int32_t gd, gm;                             /* gd: the dry share (the chorus send taken out), Q8; gm: the mono send gain, Q12 */
} voice_t;

/* the parts: each a whole OberVave sound. The panel (keys, knobs, arp, HOLD) works on the focus part; `live` is it. */
static patch_t parts[NPART];
static volatile uint32_t focus;                 /* the main loop writes (with focus_set), the ISR follows */
#define live (parts[focus])
static voice_t vc[NV];
static uint32_t vseq, rr;                       /* allocation counter, round-robin pointer */

/* main loop -> ISR: note events */
typedef struct { uint8_t type, note, vel, part; } nev_t;   /* type 1 on, 0 off; part 255 = the focus part */
#define NQ 64u
#define FOCUS 255u
static nev_t nq[NQ];
static volatile uint32_t nq_w, nq_r;
static void note_event_p(uint32_t part, uint32_t on, uint32_t note, uint32_t vel)
{
    if (nq_w - nq_r >= NQ) return;              /* a flood: drop (the ISR drains 64 per block) */
    nq[nq_w % NQ].type = (uint8_t)on;
    nq[nq_w % NQ].note = (uint8_t)note;
    nq[nq_w % NQ].vel = (uint8_t)vel;
    nq[nq_w % NQ].part = (uint8_t)part;
    RING_PUBLISH();
    nq_w++;
}
static void note_event(uint32_t on, uint32_t note, uint32_t vel) { note_event_p(FOCUS, on, note, vel); }

/* the controllers (main loop writes), one set per part */
static volatile int32_t ctl_bend[NPART];        /* -8192..8191 */
static volatile uint32_t ctl_sus[NPART], ctl_hold, ctl_cc7[NPART] = {127, 127, 127, 127, 127, 127};
static volatile uint32_t ctl_trk_part[2] = {0, 0};   /* the sequencer's tracks A and B: which part each plays */
#if BOARD_SMK37
static volatile int32_t ctl_vib;                /* the SMK-37's mod wheel 0..127: a vibrato from the LFO, up to +-1/2 semitone */
#endif
static volatile int32_t ctl_tune8;              /* global tune, Q8 semitones */
static volatile uint32_t ctl_tempo = 120, ctl_ext_quarter;   /* MIDI clock: frames per quarter, 0 = none */
static volatile uint32_t ctl_arp_restart, ctl_panic;

/* ISR -> main loop */
static volatile int32_t ui_vlevel[NV];          /* each voice's amp envelope, Q15 */
static volatile uint8_t ui_vnote[NV];
static volatile uint32_t ui_render_us, ui_arp_step;
#define SCOPE_N 240u
static int16_t scope[SCOPE_N];
static volatile uint32_t scope_state, scope_stride = 4;   /* 0 idle, 1 armed, 2 running, 3 done */

/* ---- the held notes (ISR): in order of pressing, physically down or held by the pedal / latch */
static struct { uint8_t note, vel, phys; } held[NHELD];
static uint32_t nheld;
static uint32_t rnd_state = 0x9E3779B9u;

/* ---- what each part derives from its patch, once per block (ISR) */
typedef struct {
    int32_t fenv_amt, keytrk, velflt, lfo_pd, lfo_fd, lfo_pwd, lfo_ad, wheel_q, bend8, env_o2, det_fine8;
    int32_t lvl1, lvl2, lvln, drive_q12, reso_k, vint, sus_f, sus_a, vol_q12, morph_q;
    int32_t cut_sm, reso_sm;                    /* CUTOFF and RESO slewed (Q8), ~25 ms: a CC sweep in semitone steps glides */
    uint32_t pw1_base, pw2_base, w1, w2, ftype, sync_on, xmod_q, glide_k;
    int32_t snd_c, snd_d, snd_r;                /* the effect sends, Q8 */
    uint32_t lfo_ph, lfo_rnd_a, lfo_rnd_b;      /* each part has its own LFO */
    int32_t lfo_val;                            /* this block, Q15 */
    int32_t last_pitch16;                       /* where a glide starts from */
    uint32_t sus_was;
} part_t;
static part_t pt[NPART];

static void held_remove(uint32_t note)
{
    uint32_t i, j;
    for (i = j = 0; i < nheld; i++)
        if (held[i].note != note) held[j++] = held[i];
    nheld = j;
}

/* ---- vintage: each voice card has its own small offsets, scaled by the knob */
static const int16_t VINT_TUNE8[NV] = {-20, 12, 7, -15, 18, -9, 3, -17};     /* +-8 cents */
static const int16_t VINT_CUT8[NV] = {-300, 250, 120, -380, 60, 330, -180, 220};   /* +-1.5 semitones */
static const int8_t VINT_ENV[NV] = {10, -6, 14, -12, 3, -16, 8, -4};         /* /128: +-12 % on the times */
static const int8_t PAN_CARD[NV] = {-40, 40, -24, 24, -56, 56, -10, 10};     /* the originals' pan pots */

static void voice_pan(voice_t *v, int32_t pan)   /* -64..64 */
{
    uint32_t pos = (uint32_t)(clampi(pan, -64, 64) + 64);
    v->gl = PAN_Q12[128u - pos];
    v->gr = PAN_Q12[pos];
}

static void voice_start(voice_t *v, uint32_t part, uint32_t note, uint32_t vel, int32_t det8, int32_t pan)
{
    const patch_t *P = &parts[part];
    v->part = (uint8_t)part;
    v->sus = 0;
    v->note = (uint8_t)note;
    v->vel = (uint8_t)vel;
    v->gate = 1;
    v->seq = ++vseq;
    v->det8 = det8;
    v->target16 = (int32_t)note << 16;
    v->pitch16 = P->v[P_GLIDE] ? pt[part].last_pitch16 : v->target16;
    v->fe.stage = E_ATT;
    v->ae.stage = E_ATT;
    v->lfo_blk = 0;
    v->lfo_fade = P->v[P_LFO_DELAY] ? 0 : 32767;
    voice_pan(v, pan);
}

/* round robin over the cards; steal the oldest released, then the part's own oldest, then the oldest. `avoid`:
 * a bit per voice this trigger already took (DUAL / UNISON must not take one card twice) */
static volatile uint32_t ctl_keep;             /* bit p: part p's held voices are never stolen by another part (drones) */
static inline int voice_kept(uint32_t v, uint32_t part) { return vc[v].part != part && vc[v].gate && (ctl_keep >> vc[v].part & 1u); }
static uint32_t voice_alloc(uint32_t part, uint32_t avoid)
{
    uint32_t i, v, pick = 0, best = 0xFFFFFFFFu;
    for (i = 0; i < NV; i++) {
        v = (rr + i) % NV;
        if (vc[v].ae.stage == E_IDLE && !(avoid >> v & 1u)) { rr = v + 1u; return v; }
    }
    for (v = 0; v < NV; v++)
        if (!(avoid >> v & 1u) && !vc[v].gate && vc[v].seq < best) { best = vc[v].seq; pick = v; }
    if (best == 0xFFFFFFFFu)
        for (v = 0; v < NV; v++)
            if (!(avoid >> v & 1u) && vc[v].part == part && vc[v].seq < best) { best = vc[v].seq; pick = v; }
    if (best == 0xFFFFFFFFu)                    /* the oldest of all, a kept (drone) part's voices spared */
        for (v = 0; v < NV; v++)
            if (!(avoid >> v & 1u) && !voice_kept(v, part) && vc[v].seq < best) { best = vc[v].seq; pick = v; }
    if (best == 0xFFFFFFFFu)
        for (v = 0; v < NV; v++)
            if (!(avoid >> v & 1u) && vc[v].seq < best) { best = vc[v].seq; pick = v; }
    rr = pick + 1u;
    return pick;
}

static uint32_t uni_count(const patch_t *P)
{
    uint32_t m = P->v[P_MODE];
    return m == M_MONO ? 1u : m == M_UNISON ? clampi(P->v[P_UNI_N], 1, NV) : 0u;
}

static void voice_trigger(uint32_t part, uint32_t note, uint32_t vel)
{
    const patch_t *P = &parts[part];
    uint32_t mode = P->v[P_MODE], n, i, legato = 0, grp[NV], ng = 0, took = 0;
    int32_t det = (P->v[P_UNI_DET] * P->v[P_UNI_DET]) >> 6, spread = P->v[P_SPREAD];
    if (mode == M_POLY) {
        uint32_t v = voice_alloc(part, 0);
        voice_start(&vc[v], part, note, vel, 0, (PAN_CARD[v] * spread) >> 7);
    } else if (mode == M_DUAL) {
        uint32_t a = voice_alloc(part, 0), b = voice_alloc(part, 1u << a);
        voice_start(&vc[a], part, note, vel, -det / 2, (-48 * spread) >> 7);
        voice_start(&vc[b], part, note, vel, det / 2, (48 * spread) >> 7);
    } else {
        n = uni_count(P);
        for (i = 0; i < NV; i++)                /* the part's group: its voices a key holds */
            if (vc[i].part == part && vc[i].gate && ng < NV) grp[ng++] = i;
        legato = ng == n;
        if (!legato)                            /* a fresh group: the old one lets go, n cards are taken */
            for (i = 0; i < ng; i++) vc[grp[i]].gate = 0;
        for (i = 0; i < n; i++) {
            int32_t d = n > 1 ? (det * (int32_t)(2u * i) / (int32_t)(n - 1u)) - det : 0;
            int32_t pan = n > 1 ? ((int32_t)(128u * i) / (int32_t)(n - 1u) - 64) * spread >> 7 : 0;
            if (legato) {                       /* a key still down: slide, no retrigger */
                voice_t *v = &vc[grp[i]];
                v->note = (uint8_t)note;
                v->target16 = (int32_t)note << 16;
                if (!P->v[P_GLIDE]) v->pitch16 = v->target16;
                v->det8 = d;
                v->sus = 0;
                voice_pan(v, pan);
            } else {
                uint32_t v = voice_alloc(part, took);
                took |= 1u << v;
                voice_start(&vc[v], part, note, vel, d, pan);
            }
        }
    }
    pt[part].last_pitch16 = (int32_t)note << 16;
    if (P->v[P_LFO_PHASE] == LP_SYNC && !legato) {
        for (i = 0; i < NV; i++)
            if (vc[i].part == part && vc[i].gate && vc[i].note != note) break;
        if (i == NV) pt[part].lfo_ph = 0;       /* the first key of a chord restarts the part's LFO */
    }
}

/* the key (or the arp) let go of a note. In MONO / UNISON a key still held takes over (legato): the focus part
 * only, it has the held-note list. */
static void voice_release(uint32_t part, uint32_t note, uint32_t from_arp)
{
    const patch_t *P = &parts[part];
    uint32_t v, mode = P->v[P_MODE];
    if ((mode == M_MONO || mode == M_UNISON) && part == focus && !from_arp && nheld && P->v[P_ARP_MODE] == A_OFF) {
        uint32_t hit = 0;
        for (v = 0; v < NV; v++)
            if (vc[v].part == part && vc[v].gate && vc[v].note == note) hit = 1;
        if (hit) {
            for (v = 0; v < NV; v++)
                if (vc[v].part == part && vc[v].gate) {
                    vc[v].note = held[nheld - 1u].note;
                    vc[v].target16 = (int32_t)vc[v].note << 16;
                    if (!P->v[P_GLIDE]) vc[v].pitch16 = vc[v].target16;
                }
            pt[part].last_pitch16 = (int32_t)held[nheld - 1u].note << 16;
        }
        return;
    }
    for (v = 0; v < NV; v++)
        if (vc[v].part == part && vc[v].gate && vc[v].note == note) {
            if (part != focus && ctl_sus[part]) vc[v].sus = 1;   /* another part's pedal: held until it lets go */
            else vc[v].gate = 0;
        }
}

static void voices_release_part(uint32_t part)
{
    uint32_t v;
    for (v = 0; v < NV; v++)
        if (vc[v].part == part) { vc[v].gate = 0; vc[v].sus = 0; }
}
static void voices_release_all(void)
{
    uint32_t v;
    for (v = 0; v < NV; v++) { vc[v].gate = 0; vc[v].sus = 0; }
}

static void seq_tick(void);                     /* the step sequencer (obv_seq.h): per block, after the arp */

/* the click (main: ctl_click on / off) and a one-bar count-in before a recording (main: ctl_count) */
static volatile uint32_t ctl_click, ctl_count, ui_seq_count;
static int32_t click_amp;
static uint32_t click_ph, click_inc;
static void click_start(uint32_t accent)        /* beat 1 higher and louder */
{
    click_amp = accent ? 9000 : 6000;
    click_ph = 0;
    click_inc = accent ? 233739600u : 155826400u;   /* 2400 / 1600 Hz in 2^32 / 44100 steps */
}

/* ---- the arpeggiator (per block, 0.36 ms steps) */
static struct { uint32_t cnt, period, gate_at, pos, dir, sounding, note, was_on, part; } arp;

/* ISR -> main loop: the notes the arp plays (REC records them, not the keys that feed it) */
#define AQ 32u
static uint8_t arp_q[AQ][2];
static volatile uint32_t arp_qw, arp_qr;
static void arp_post(uint32_t on, uint32_t note)
{
    if (arp_qw - arp_qr >= AQ) return;
    arp_q[arp_qw % AQ][0] = (uint8_t)on;
    arp_q[arp_qw % AQ][1] = (uint8_t)note;
    RING_PUBLISH();
    arp_qw++;
}

static void arp_stop_note(void)
{
    if (arp.sounding) { voice_release(arp.part, arp.note, 1); arp.sounding = 0; arp_post(0, arp.note); }
}

static void arp_step(void)
{
    uint8_t sorted[NHELD];
    uint32_t i, j, n = nheld, oct = clampi(live.v[P_ARP_OCT], 1, 4), len, k, mode = live.v[P_ARP_MODE], idx;
    for (i = 0; i < n; i++) sorted[i] = held[i].note;
    if (mode != A_PLAY)
        for (i = 1; i < n; i++) {               /* insertion sort: 16 at most */
            uint8_t t = sorted[i];
            for (j = i; j && sorted[j - 1u] > t; j--) sorted[j] = sorted[j - 1u];
            sorted[j] = t;
        }
    len = n * oct;
    if (mode == A_RND)
        k = xorshift(&rnd_state) % len;
    else if (mode == A_UPDN && len > 2u) {
        if (arp.pos >= 2u * len - 2u) arp.pos = 0;
        k = arp.pos < len ? arp.pos : 2u * len - 2u - arp.pos;
        arp.pos++;
    } else {
        if (arp.pos >= len) arp.pos = 0;
        k = mode == A_DOWN ? len - 1u - arp.pos : arp.pos;
        arp.pos++;
    }
    idx = k % n;
    arp.note = (uint32_t)clampi((int32_t)sorted[idx] + 12 * (int32_t)(k / n), 0, 127);
    for (i = 0; i < n; i++)                     /* the velocity of that key */
        if (held[i].note == sorted[idx]) break;
    arp.part = focus;
    voice_trigger(focus, arp.note, i < n ? held[i].vel : 100u);
    arp_post(1, arp.note);
    arp.sounding = 1;
    ui_arp_step++;
}

static void arp_tick(void)
{
    uint32_t mode = live.v[P_ARP_MODE], div = DIV_PER_QUARTER[live.v[P_ARP_DIV] % NDIV], q;
    if (mode == A_OFF) {
        if (arp.was_on) { arp_stop_note(); arp.was_on = 0; }
        return;
    }
    if (!arp.was_on) { voices_release_part(focus); arp.was_on = 1; arp.pos = 0; arp.cnt = 0xFFFFFFFFu; }
    if (ctl_arp_restart) { ctl_arp_restart = 0; arp.pos = 0; arp.cnt = 0xFFFFFFFFu; }
    if (!nheld) { arp_stop_note(); arp.pos = 0; arp.cnt = 0xFFFFFFFFu; return; }
    q = ctl_ext_quarter ? ctl_ext_quarter : 60u * FS / clampi(ctl_tempo, 40, 240);
    arp.period = q / div;
    if (arp.period < CTL) arp.period = CTL;
    arp.gate_at = (arp.period * (live.v[P_ARP_GATE] + 1u)) >> 7;
    if (arp.cnt == 0xFFFFFFFFu || arp.cnt + CTL >= arp.period) {
        arp.cnt = arp.cnt == 0xFFFFFFFFu ? 0 : arp.cnt + CTL - arp.period;
        arp_stop_note();
        arp_step();
    } else {
        arp.cnt += CTL;
        if (arp.sounding && arp.cnt >= arp.gate_at && live.v[P_ARP_GATE] < 127u) arp_stop_note();
    }
}

/* ---- note events (ISR) */
static void ev_note_on(uint32_t note, uint32_t vel)
{
    uint32_t i, arp_on = live.v[P_ARP_MODE] != A_OFF, sus = ctl_sus[focus] || ctl_hold;
    if (arp_on && sus) {                        /* latch: a new chord after all keys were up starts over */
        for (i = 0; i < nheld; i++)
            if (held[i].phys) break;
        if (i == nheld) { nheld = 0; arp.pos = 0; arp.cnt = 0xFFFFFFFFu; }
    }
    held_remove(note);
    if (nheld == NHELD) { for (i = 1; i < NHELD; i++) held[i - 1u] = held[i]; nheld--; }
    held[nheld].note = (uint8_t)note;
    held[nheld].vel = (uint8_t)vel;
    held[nheld].phys = 1;
    nheld++;
    if (arp_on) {
        if (nheld == 1u) { arp.pos = 0; arp.cnt = 0xFFFFFFFFu; }   /* the first key: a step right away */
        return;
    }
    voice_trigger(focus, note, vel);
}

static void ev_note_off(uint32_t note)
{
    uint32_t i;
    for (i = 0; i < nheld; i++)
        if (held[i].note == note) held[i].phys = 0;
    if (ctl_sus[focus] || ctl_hold) return;     /* the pedal / HOLD keeps it */
    held_remove(note);
    if (live.v[P_ARP_MODE] != A_OFF) return;
    voice_release(focus, note, 0);
}

static void ev_sustain_up(void)                 /* the pedal / HOLD let go: what no key holds is released */
{
    uint32_t i, j, arp_on = live.v[P_ARP_MODE] != A_OFF;
    uint8_t gone[NHELD], ng = 0;
    for (i = j = 0; i < nheld; i++)
        if (held[i].phys) held[j++] = held[i];
        else gone[ng++] = held[i].note;
    nheld = j;
    if (arp_on) return;
    for (i = 0; i < ng; i++) voice_release(focus, gone[i], 0);
}

static uint32_t eng_focus;                      /* the focus part the held list belongs to */
static void engine_events(void)
{
    uint32_t sus, p, v;
    if (ctl_panic) { ctl_panic = 0; nheld = 0; voices_release_all(); arp_stop_note(); }
    if (eng_focus != focus) {                   /* the panel moved to another part: the old one lets go of the keys */
        uint32_t i;
        arp_stop_note();                        /* (the arp remembers its part) */
        for (i = 0; i < nheld; i++) voice_release(eng_focus, held[i].note, 1);
        nheld = 0;
        pt[eng_focus].sus_was = 0;
        eng_focus = focus;
        arp.was_on = 0;
    }
    sus = ctl_sus[focus] || ctl_hold;
    if (!sus && pt[focus].sus_was) ev_sustain_up();
    pt[focus].sus_was = sus;
    for (p = 0; p < NPART; p++) {               /* the other parts: their pedal let go releases what it held */
        if (p == focus) continue;
        if (!ctl_sus[p] && pt[p].sus_was)
            for (v = 0; v < NV; v++)
                if (vc[v].part == p && vc[v].sus) { vc[v].gate = 0; vc[v].sus = 0; }
        pt[p].sus_was = ctl_sus[p];
    }
    while (nq_r != nq_w) {
        nev_t e = nq[nq_r % NQ];
        nq_r++;
        if (e.part == FOCUS || e.part == focus) {   /* the focus part: through the held list, the arp, HOLD */
            if (e.type) ev_note_on(e.note, e.vel);
            else ev_note_off(e.note);
        } else if (e.part < NPART) {            /* another part: straight to its voices */
            if (e.type) voice_trigger(e.part, e.note, e.vel);
            else voice_release(e.part, e.note, 0);
        }
    }
}

/* ---- envelopes: analog RC curves per block (CTL frames), level Q24 */
static inline void env_tick(env_t *e, uint32_t a, uint32_t d, int32_t sus, uint32_t r, int32_t gate, int32_t vint)
{
    int32_t k, target;
    if (gate && e->stage >= E_REL) e->stage = E_ATT;
    if (!gate && e->stage && e->stage < E_REL) e->stage = E_REL;
    switch (e->stage) {
    case E_ATT:
        k = (int32_t)ENV_K_ATT[a];
        k += (int32_t)(((int64_t)k * vint) >> 7);
        target = ENV_ONE + (ENV_ONE * 3 >> 3);  /* aims at 1.3, stops at 1.0 */
        e->level += (int32_t)(((int64_t)(target - e->level) * k) >> 24);
        if (e->level >= ENV_ONE) { e->level = ENV_ONE; e->stage = E_DEC; }
        break;
    case E_DEC:
        k = (int32_t)ENV_K_DEC[d];
        k += (int32_t)(((int64_t)k * vint) >> 7);
        e->level += (int32_t)(((int64_t)(sus - e->level) * k) >> 24) - 16;
        if (e->level <= sus + 64) { e->level = sus; e->stage = E_SUS; }
        break;
    case E_SUS:
        e->level = sus;                         /* the knob moves it */
        break;
    case E_REL:
        k = (int32_t)ENV_K_DEC[r];
        k += (int32_t)(((int64_t)k * vint) >> 7);
        e->level += (int32_t)(((int64_t)(0 - e->level) * k) >> 24) - 32;
        if (e->level < LEVEL_FLOOR) { e->level = 0; e->stage = E_IDLE; }
        break;
    default:
        e->level = 0;
    }
}

/* ---- the LFO (global; per voice a phase offset in SPREAD and a delay / fade) */
static int32_t lfo_shape(const part_t *g, uint32_t ph, uint32_t shape)
{
    int32_t p;
    switch (shape) {
    case L_TRI:
        p = (int32_t)((ph + 0x40000000u) >> 15);
        return p < 65536 ? p - 32768 : 98303 - p;
    case L_SAW: return 32767 - (int32_t)(ph >> 16);
    case L_RAMP: return (int32_t)(ph >> 16) - 32768;
    case L_SQR: return ph < 0x80000000u ? 32767 : -32767;
    case L_SH: return (int32_t)(g->lfo_rnd_b >> 16) - 32768;
    default: {                                  /* smooth random: a line from one random value to the next */
        int32_t a = (int32_t)(g->lfo_rnd_a >> 16) - 32768, b = (int32_t)(g->lfo_rnd_b >> 16) - 32768;
        return a + (int32_t)(((int64_t)(b - a) * (ph >> 8)) >> 24);
    }
    }
}

static void lfo_tick(uint32_t p)
{
    part_t *g = &pt[p];
    const patch_t *P = &parts[p];
    uint32_t inc = LFO_INC[P->v[P_LFO_RATE]], old = g->lfo_ph;
    g->lfo_ph += inc;
    if (g->lfo_ph < old) { g->lfo_rnd_a = g->lfo_rnd_b; g->lfo_rnd_b = xorshift(&rnd_state); }
    g->lfo_val = lfo_shape(g, g->lfo_ph, P->v[P_LFO_SHAPE]);
}

/* ---- one control block for a voice: pitch, cutoff, envelopes, amp */
static void block_globals(uint32_t p)           /* a part's derived numbers, once per block */
{
    part_t *g = &pt[p];
    const patch_t *pp = &parts[p];
#undef live
#define live (*pp)
    int32_t d, vol;
    int32_t fenv_amt, keytrk, velflt, lfo_pd, lfo_fd, lfo_pwd, lfo_ad, wheel_q, bend8, env_o2, det_fine8;
    int32_t lvl1, lvl2, lvln, drive_q12, reso_k, vint, sus_f, sus_a, vol_q12, morph_q;
    uint32_t pw1_base, pw2_base, w1, w2, ftype, sync_on, xmod_q, glide_k;
    fenv_amt = (int32_t)live.v[P_FENV_AMT] - 64;
    keytrk = live.v[P_KEYTRK];
    velflt = live.v[P_VEL_FLT];
    d = live.v[P_LFO_PITCH]; lfo_pd = (d * d) >> 7;
    d = live.v[P_LFO_FLT]; lfo_fd = (d * d) >> 7;
    d = live.v[P_LFO_PWM]; lfo_pwd = (d * d) >> 7;
    lfo_ad = live.v[P_LFO_AMP];
    wheel_q = live.v[P_MOD_DEPTH];               /* the knob and the mod wheel (CC 1) are the same value */
    bend8 = (ctl_bend[p] * live.v[P_BEND] * 256) >> 13;
    env_o2 = (int32_t)live.v[P_ENV_O2] - 64;
    det_fine8 = ((int32_t)live.v[P_O2_FINE] - 64) * 2;
    d = live.v[P_O1_LVL]; lvl1 = (d * d) >> 7;
    d = live.v[P_O2_LVL]; lvl2 = (d * d) >> 7;
    d = live.v[P_NOISE]; lvln = (d * d) >> 7;
    d = live.v[P_DRIVE]; drive_q12 = d ? 4096 + (d * d * 7 * 4096 / 16129) : 0;
    g->cut_sm += ((live.v[P_CUTOFF] << 8) - g->cut_sm) >> 6;   /* tau = 64 blocks = 23 ms */
    g->reso_sm += ((live.v[P_RESO] << 8) - g->reso_sm) >> 6;
    reso_k = 8192 - g->reso_sm * 8400 / (127 * 256);
    vint = live.v[P_VINTAGE];
    sus_f = live.v[P_FS] * (ENV_ONE / 127);
    sus_a = live.v[P_AS] * (ENV_ONE / 127);
    vol = (int32_t)((live.v[P_VOLUME] * ctl_cc7[p]) >> 7);
    vol_q12 = (vol * vol) >> 2;
    pw1_base = ((uint32_t)live.v[P_O1_PW] << 25) + (1u << 24);
    pw2_base = ((uint32_t)live.v[P_O2_PW] << 25) + (1u << 24);
    w1 = live.v[P_O1_WAVE];
    w2 = live.v[P_O2_WAVE];
    ftype = live.v[P_FTYPE];
    morph_q = live.v[P_FMORPH];                  /* SEM: 0 LP, 64 notch, 127 HP */
    sync_on = live.v[P_SYNC];
    d = live.v[P_XMOD]; xmod_q = (uint32_t)((d * d) >> 7);
    glide_k = GLIDE_K[live.v[P_GLIDE]];
    g->snd_c = live.v[P_CHO_MIX];                    /* as the old mix: the chorus takes this much of the dry */
    g->snd_d = (live.v[P_DLY_MIX] * 180) >> 7;       /* wet up to 0.7 of the dry (Q8) */
    g->snd_r = (live.v[P_REV_MIX] * 180) >> 7;
    g->fenv_amt = fenv_amt; g->keytrk = keytrk; g->velflt = velflt; g->lfo_pd = lfo_pd; g->lfo_fd = lfo_fd;
    g->lfo_pwd = lfo_pwd; g->lfo_ad = lfo_ad; g->wheel_q = wheel_q; g->bend8 = bend8; g->env_o2 = env_o2;
    g->det_fine8 = det_fine8; g->lvl1 = lvl1; g->lvl2 = lvl2; g->lvln = lvln; g->drive_q12 = drive_q12;
    g->reso_k = reso_k; g->vint = vint; g->sus_f = sus_f; g->sus_a = sus_a; g->vol_q12 = vol_q12; g->morph_q = morph_q;
    g->pw1_base = pw1_base; g->pw2_base = pw2_base; g->w1 = w1; g->w2 = w2; g->ftype = ftype; g->sync_on = sync_on;
    g->xmod_q = xmod_q; g->glide_k = glide_k;
#undef live
#define live (parts[focus])
}

static inline uint32_t pw_mod(uint32_t base, int32_t mod)
{
    int64_t p = (int64_t)base + ((int64_t)mod << 8);
    if (p < (1 << 26)) p = 1 << 26;
    if (p > (int64_t)0xFFFFFFFFu - (1 << 26)) p = (int64_t)0xFFFFFFFFu - (1 << 26);
    return (uint32_t)p;
}

static void voice_block(voice_t *v, uint32_t card)
{
    const part_t *g = &pt[v->part];
    const patch_t *pp = &parts[v->part];
#undef live
#define live (*pp)
    int32_t fenv_amt = g->fenv_amt, keytrk = g->keytrk, velflt = g->velflt, lfo_pd = g->lfo_pd, lfo_fd = g->lfo_fd;
    int32_t lfo_pwd = g->lfo_pwd, lfo_ad = g->lfo_ad, wheel_q = g->wheel_q, bend8 = g->bend8, env_o2 = g->env_o2;
    int32_t det_fine8 = g->det_fine8, reso_k = g->reso_k, vint = g->vint, sus_f = g->sus_f, sus_a = g->sus_a;
    int32_t cut_sm = g->cut_sm;
    uint32_t pw1_base = g->pw1_base, pw2_base = g->pw2_base, ftype = g->ftype, glide_k = g->glide_k;
    int32_t lfo, p8, p1, p2, cut, fenv, tmp, velg, trem, vt = (vint * VINT_ENV[card]) >> 7, lfo_to;
    uint32_t ph;
    /* glide */
    if (v->pitch16 != v->target16) {
        if (!glide_k) v->pitch16 = v->target16;
        else {
            int32_t diff = v->target16 - v->pitch16;
            v->pitch16 += (int32_t)(((int64_t)diff * glide_k) >> 16);
            if (diff > -64 && diff < 64) v->pitch16 = v->target16;
        }
    }
    /* envelopes */
    env_tick(&v->fe, live.v[P_FA], live.v[P_FD], sus_f, live.v[P_FR], v->gate, vt);
    env_tick(&v->ae, live.v[P_AA], live.v[P_AD], sus_a, live.v[P_AR], v->gate, vt);
    fenv = v->fe.level >> 9;                    /* Q15 */
    /* the LFO as this voice sees it */
    ph = g->lfo_ph;
    if (live.v[P_LFO_PHASE] == LP_SPREAD) ph += card << 29;
    lfo = live.v[P_LFO_PHASE] == LP_SPREAD ? lfo_shape(g, ph, live.v[P_LFO_SHAPE]) : g->lfo_val;
    if (v->lfo_fade < 32767) {
        uint32_t dl = LFO_DELAY_BLK[live.v[P_LFO_DELAY]];
        if (v->lfo_blk >= dl) { v->lfo_fade += 32767 / (int32_t)(dl / 2u + 1u) + 1; if (v->lfo_fade > 32767) v->lfo_fade = 32767; }
        v->lfo_blk++;
    }
    lfo = (lfo * v->lfo_fade) >> 15;
    /* vintage: a slow random walk on the pitch */
    v->drift += (int32_t)(xorshift(&v->noise) & 31u) - 16;
    v->drift -= v->drift >> 11;
    /* pitch, Q8 semitones */
    p8 = (v->pitch16 >> 8) + bend8 + ctl_tune8 + v->det8 + ((VINT_TUNE8[card] * vint) >> 7) + ((v->drift * vint) >> 10);
    p8 += (lfo * wheel_q * 4) >> 15;            /* the wheel's vibrato */
#if BOARD_SMK37
    p8 += (lfo * ctl_vib) >> 15;                /* the SMK-37's mod wheel: pitch only, gentle */
#endif
    lfo_to = (lfo * lfo_pd * 24) >> 15;
    p1 = p8 + (live.v[P_LFO_PITCH_TO] != TO_O2 ? lfo_to : 0);
    p2 = p8 + (live.v[P_LFO_PITCH_TO] != TO_O1 ? lfo_to : 0) + ((int32_t)live.v[P_O2_SEMI] - 24) * 256 + det_fine8
       + ((fenv * env_o2 * 97) >> 15);
    v->inc1 = pitch_inc(p1);
    v->inc2 = pitch_inc(p2);
    tmp = (lfo * lfo_pwd);
    v->pw1 = pw_mod(pw1_base, live.v[P_LFO_PWM_TO] != TO_O2 ? tmp : 0);
    v->pw2 = pw_mod(pw2_base, live.v[P_LFO_PWM_TO] != TO_O1 ? tmp : 0);
    /* cutoff, Q8 semitones */
    cut = cut_sm + 8 * 256 + ((fenv * fenv_amt * 390) >> 15)
        + ((int32_t)v->note - 60) * keytrk * 2 + (((int32_t)v->vel - 64) * velflt >> 1)
        + ((lfo * lfo_fd * 97) >> 15) + ((lfo * wheel_q * 48) >> 15) + ((VINT_CUT8[card] * vint) >> 7);
    svf_coef(&v->c1, cut, reso_k);
    if (ftype == F_LP24) svf_coef(&v->c2, cut, 5793);
    /* amp */
    velg = 32767 - ((((127 - (int32_t)v->vel) * (int32_t)live.v[P_VEL_AMP]) >> 7) * 258);
    trem = 32767 - (((32767 - lfo) * lfo_ad) >> 8);
    v->amp0 = v->amp1;
    v->amp1 = (int32_t)(((int64_t)(v->ae.level >> 9) * velg >> 15) * trem >> 15);
    ui_vlevel[card] = v->amp1;
    v->amp1 = (v->amp1 * g->vol_q12) >> 12;     /* the part's volume (one part alone: as the old output stage) */
    ui_vnote[card] = v->note;
    v->gd = 256 - g->snd_c;                     /* the mix: dry less what the chorus takes; the sends go through the part's bus */
    v->gm = (g->snd_c | g->snd_d | g->snd_r) ? (v->gl + v->gr) >> 1 : 0;
#undef live
#define live (parts[focus])
}

/* ---- the samples of one voice: oscillators -> mix -> drive -> filter -> amp -> pan */
static void voice_render(voice_t *v, int32_t *out, int32_t *pbus)
{
    const part_t *g = &pt[v->part];
    uint32_t i, inc1 = v->inc1, inc2 = v->inc2, pw1 = v->pw1, pw2 = v->pw2, d1 = inc1 >> 15, d2 = inc2 >> 15;
    int32_t amp0 = v->amp0, da = v->amp1 - v->amp0, o2n = v->o2.prev;
    int32_t gl = (v->gl * v->gd) >> 8, gr = (v->gr * v->gd) >> 8, gm = v->gm;
    int32_t l1 = g->lvl1, l2 = g->lvl2, ln = g->lvln, drv = g->drive_q12, morph_q = g->morph_q;
    uint32_t ft = g->ftype, xq = g->xmod_q, sy = g->sync_on, ww1 = g->w1, ww2 = g->w2;
    int32_t hw1 = (ww1 & OSC_SAW ? -65536 : 0) + (ww1 & OSC_PLS ? -65536 : 0);
    int32_t hw2 = (ww2 & OSC_SAW ? -65536 : 0) + (ww2 & OSC_PLS ? -65536 : 0);
    for (i = 0; i < CTL; i++) {
        int32_t s1, s2, mix, lp, bp, hp, y, amp, rt = -1;
        uint32_t ie = inc1;
        if (xq) {                               /* OSC2 -> OSC1 linear FM, the step never below inc/16 */
            int32_t m = (int32_t)(inc1 >> 8 > 0x200000u ? 0x200000u : inc1 >> 8) * (((o2n >> 8) * (int32_t)xq) >> 6);
            m += (int32_t)inc1;
            ie = m < (int32_t)(inc1 >> 4) ? inc1 >> 4 : (uint32_t)m;
            d1 = ie >> 15;
        }
        s1 = osc_step(&v->o1, ie, d1, pw1, ww1, hw1, -1);
        if (sy && v->o1.ph < ie && d1)
            rt = (int32_t)(v->o1.ph / d1);
        s2 = osc_step(&v->o2, inc2, d2, pw2, ww2, hw2, rt);
        o2n = v->o2.prev;
        mix = s1 * l1 + s2 * l2;
        if (ln) mix += ((int32_t)(xorshift(&v->noise) >> 16) - 32768) * ln;
        mix >>= 7;
        if (drv) mix = softclip(((mix >> 3) * drv) >> 9);
        else mix = clampi(mix, -65535, 65535);
        svf_step(&v->c1, &v->f1, mix, &lp, &bp, &hp);
        switch (ft) {
        case F_LP12: y = lp; break;
        case F_HP12: y = hp; break;
        case F_BP12: y = bp; break;
        case F_NOTCH: y = hp + lp; break;
        case F_SEM: y = lp + (((hp - lp) * morph_q) >> 7); break;   /* the SEM's LP .. notch .. HP pot */
        default: y = svf_lp(&v->c2, &v->f2, lp); break;
        }
        y = clampi(y, -65000, 65000);
        amp = amp0 + ((da * (int32_t)i) >> 4);
        y = (y * amp) >> 16;
        out[2u * i] += (y * gl) >> 12;
        out[2u * i + 1u] += (y * gr) >> 12;
        if (gm) pbus[i] += (y * gm) >> 12;      /* the part's mono bus: one add a voice, the sends once a part */
    }
    (void)o2n;
}

#define BUS_N 512u                              /* the frames of one call at most */
static int32_t bus_c[BUS_N], bus_d[BUS_N], bus_r[BUS_N];   /* the effect sends, mono */
static int32_t bus_p[NPART][CTL];               /* each part's mono send bus, one block */
static void engine_audio(int32_t *out, uint32_t n)
{
    uint32_t b, v, i, p, any_c = 0, any_d = 0, any_r = 0;
    static int32_t sc_prev;
    static uint32_t sc_i, sc_sub;
    if (n > BUS_N) n = BUS_N;
    for (p = 0; p < NPART; p++) {               /* the effects: their sound from part 1, a send from each part */
        any_c |= parts[p].v[P_CHO_MIX];
        any_d |= parts[p].v[P_DLY_MIX];
        any_r |= parts[p].v[P_REV_MIX];
    }
    fx_block(&parts[0], ctl_ext_quarter ? ctl_ext_quarter : 60u * FS / clampi(ctl_tempo, 40, 240), any_c, any_d, any_r);
    for (i = 0; i < n; i++) bus_c[i] = bus_d[i] = bus_r[i] = 0;
    for (b = 0; b < n; b += CTL) {
        engine_events();
        arp_tick();
        seq_tick();
        {   /* only the parts that sound need their numbers (the LFOs run on: a FREE one keeps its phase) */
            uint32_t on = 0;
            for (v = 0; v < NV; v++)
                if (vc[v].ae.stage != E_IDLE || vc[v].gate) on |= 1u << vc[v].part;
            for (p = 0; p < NPART; p++) {
                lfo_tick(p);
                if (on >> p & 1u) block_globals(p);
                else {                          /* silent: the slews run on, so a note starts where the old engine would */
                    pt[p].cut_sm += ((parts[p].v[P_CUTOFF] << 8) - pt[p].cut_sm) >> 6;
                    pt[p].reso_sm += ((parts[p].v[P_RESO] << 8) - pt[p].reso_sm) >> 6;
                }
            }
        }
        {
            uint32_t sent = 0;
            for (v = 0; v < NV; v++) {
                if (vc[v].ae.stage == E_IDLE && !vc[v].gate) {
                    if (ui_vlevel[v]) { ui_vlevel[v] = 0; vc[v].amp0 = vc[v].amp1 = 0; }
                    continue;
                }
                voice_block(&vc[v], v);
                p = vc[v].part;
                if (vc[v].gm && !(sent >> p & 1u)) { sent |= 1u << p; for (i = 0; i < CTL; i++) bus_p[p][i] = 0; }
                voice_render(&vc[v], out + 2u * b, bus_p[p]);
            }
            for (p = 0; p < NPART; p++) {       /* each sending part into the effects, at its send levels */
                int32_t c = pt[p].snd_c, d = pt[p].snd_d, r = pt[p].snd_r;
                if (!(sent >> p & 1u)) continue;
                for (i = 0; i < CTL; i++) {
                    int32_t m = bus_p[p][i];
                    bus_c[b + i] += (m * c) >> 8;
                    bus_d[b + i] += (m * d) >> 8;
                    bus_r[b + i] += (m * r) >> 8;
                }
            }
        }
    }
    for (i = 0; i < n; i++) {                   /* the output stage: a gentle tanh, never a hard clip (the volume: per part, in the voices) */
        int32_t l = softclip(clampi(out[2u * i], -258048, 258047));   /* (the old stage: x / 4 x volume up to 3.94) */
        int32_t r = softclip(clampi(out[2u * i + 1u], -258048, 258047));
        int32_t m = (l + r) >> 1;
        out[2u * i] = l;
        out[2u * i + 1u] = r;
        if (scope_state == 1u) {                /* armed: wait for a rising zero crossing */
            if (sc_prev < 0 && m >= 0) { scope_state = 2; sc_i = 0; sc_sub = 0; }
        } else if (scope_state == 2u) {
            if (!sc_sub++) scope[sc_i++] = (int16_t)m;
            if (sc_sub >= scope_stride) sc_sub = 0;
            if (sc_i >= SCOPE_N) scope_state = 3;
        }
        sc_prev = m;
    }
    fx_process(out, bus_c, bus_d, bus_r, n);
    for (i = 0; i < n && click_amp; i++) {     /* the click: after the effects, never recorded */
        int32_t c;
        click_ph += click_inc;
        c = click_ph & 0x80000000u ? click_amp : -click_amp;
        out[2u * i] += c;
        out[2u * i + 1u] += c;
        click_amp -= (click_amp >> 8) + 1;      /* ~6 ms */
        if (click_amp < 64) click_amp = 0;
    }
}

static int engine_quiet(void)                   /* nothing sounds: a flash erase is safe */
{
    uint32_t v;
    for (v = 0; v < NV; v++)
        if (vc[v].ae.stage != E_IDLE || vc[v].gate) return 0;
    return nq_r == nq_w;
}

static void engine_init(void)
{
    uint32_t v;
    for (v = 0; v < NV; v++) {
        vc[v].noise = 0x12345678u * (v + 1u) + 0x9E3779B9u;
        vc[v].o1.ph = xorshift(&vc[v].noise);  /* free-running oscillators start anywhere */
        vc[v].o2.ph = xorshift(&vc[v].noise);
        vc[v].gl = vc[v].gr = 2896;
        vc[v].gd = 256;
    }
    for (v = 0; v < NPART; v++) {
        pt[v].lfo_rnd_a = xorshift(&rnd_state);
        pt[v].lfo_rnd_b = xorshift(&rnd_state);
        pt[v].last_pitch16 = 60 << 16;
        pt[v].cut_sm = parts[v].v[P_CUTOFF] << 8;
        pt[v].reso_sm = parts[v].v[P_RESO] << 8;
    }
    fx_init();
}
