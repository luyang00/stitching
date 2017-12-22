#define _mm_srli_si128(a, imm) \
({ \
__m128i ret; \
if ((imm) <= 0) { \
ret = a; \
} \
else if ((imm) > 15) { \
ret = _mm_setzero_si128(); \
} \
else { \
ret = vreinterpretq_m128i_s8(vextq_s8(vreinterpretq_s8_m128i(a), vdupq_n_s8(0), (imm))); \
} \
ret; \
})

#define _mm_srai_epi16(a, imm) \
({ \
__m128i ret; \
if ((imm) <= 0) { \
ret = a; \
} \
else if ((imm) > 31) { \
ret = vreinterpretq_m128i_s16(vshrq_n_s16(vreinterpretq_s16_m128i(a), 16)); \
ret = vreinterpretq_m128i_s16(vshrq_n_s16(vreinterpretq_s16_m128i(ret), 16)); \
} \
else { \
ret = vreinterpretq_m128i_s32(vshrq_n_s16(vreinterpretq_s16_m128i(a), (imm))); \
} \
ret; \
})
