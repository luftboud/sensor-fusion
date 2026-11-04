#ifndef SENSOR_FUSION_SIMD_H
#define SENSOR_FUSION_SIMD_H

#include <cstddef>

// For different ISA here are provided different options of simd libs
//   x86-64 -- SSE2, or AVX (when the compiler was given -mavx / /arch:AVX)
//   ARM64 -- NEON
// Anything else stays scalar.
// Loads and stores are unaligned: std::vector does not promise 32-byte alignment.

#if defined(_M_X64) || defined(__x86_64__)
    #if defined(__AVX__)
        #define MATRIX_SIMD_AVX 1
    #else
        #define MATRIX_SIMD_SSE2 1
    #endif
    #if defined(_MSC_VER)
        #include <intrin.h>
    #else
        #include <immintrin.h>
    #endif
#elif defined(_M_ARM64) || defined(__aarch64__)
    #define MATRIX_SIMD_NEON 1
    #include <arm_neon.h>
#endif

namespace simd {

#if defined(MATRIX_SIMD_AVX)
inline constexpr std::size_t width = 4;
#elif defined(MATRIX_SIMD_SSE2) || defined(MATRIX_SIMD_NEON)
inline constexpr std::size_t width = 2;
#else
inline constexpr std::size_t width = 1;
#endif

inline bool& enabled() noexcept {
    static bool value = true;
    return value;
}

inline const char* backend_name() noexcept {
#if defined(MATRIX_SIMD_AVX)
    return "avx";
#elif defined(MATRIX_SIMD_SSE2)
    return "sse2";
#elif defined(MATRIX_SIMD_NEON)
    return "neon";
#else
    return "scalar";
#endif
}

inline void add(double* dst, const double* a, const double* b, std::size_t n) {
    std::size_t i = 0;
#if defined(MATRIX_SIMD_AVX)
    for (; i + 4 <= n; i += 4) {
        const __m256d va = _mm256_loadu_pd(a + i);
        const __m256d vb = _mm256_loadu_pd(b + i);
        _mm256_storeu_pd(dst + i, _mm256_add_pd(va, vb));
    }
#elif defined(MATRIX_SIMD_SSE2)
    for (; i + 2 <= n; i += 2) {
        const __m128d va = _mm_loadu_pd(a + i);
        const __m128d vb = _mm_loadu_pd(b + i);
        _mm_storeu_pd(dst + i, _mm_add_pd(va, vb));
    }
#elif defined(MATRIX_SIMD_NEON)
    for (; i + 2 <= n; i += 2) {
        const float64x2_t va = vld1q_f64(a + i);
        const float64x2_t vb = vld1q_f64(b + i);
        vst1q_f64(dst + i, vaddq_f64(va, vb));
    }
#endif
    for (; i < n; ++i)
        dst[i] = a[i] + b[i];
}

inline void sub(double* dst, const double* a, const double* b, std::size_t n) {
    std::size_t i = 0;
#if defined(MATRIX_SIMD_AVX)
    for (; i + 4 <= n; i += 4) {
        const __m256d va = _mm256_loadu_pd(a + i);
        const __m256d vb = _mm256_loadu_pd(b + i);
        _mm256_storeu_pd(dst + i, _mm256_sub_pd(va, vb));
    }
#elif defined(MATRIX_SIMD_SSE2)
    for (; i + 2 <= n; i += 2) {
        const __m128d va = _mm_loadu_pd(a + i);
        const __m128d vb = _mm_loadu_pd(b + i);
        _mm_storeu_pd(dst + i, _mm_sub_pd(va, vb));
    }
#elif defined(MATRIX_SIMD_NEON)
    for (; i + 2 <= n; i += 2) {
        const float64x2_t va = vld1q_f64(a + i);
        const float64x2_t vb = vld1q_f64(b + i);
        vst1q_f64(dst + i, vsubq_f64(va, vb));
    }
#endif
    for (; i < n; ++i)
        dst[i] = a[i] - b[i];
}

inline void scale(double* dst, const double* src, double factor, std::size_t n) {
    std::size_t i = 0;
#if defined(MATRIX_SIMD_AVX)
    const __m256d vfactor = _mm256_set1_pd(factor);
    for (; i + 4 <= n; i += 4) {
        const __m256d v = _mm256_loadu_pd(src + i);
        _mm256_storeu_pd(dst + i, _mm256_mul_pd(v, vfactor));
    }
#elif defined(MATRIX_SIMD_SSE2)
    const __m128d vfactor = _mm_set1_pd(factor);
    for (; i + 2 <= n; i += 2) {
        const __m128d v = _mm_loadu_pd(src + i);
        _mm_storeu_pd(dst + i, _mm_mul_pd(v, vfactor));
    }
#elif defined(MATRIX_SIMD_NEON)
    const float64x2_t vfactor = vdupq_n_f64(factor);
    for (; i + 2 <= n; i += 2) {
        const float64x2_t v = vld1q_f64(src + i);
        vst1q_f64(dst + i, vmulq_f64(v, vfactor));
    }
#endif
    for (; i < n; ++i)
        dst[i] = src[i] * factor;
}

// dst[i] += alpha * x[i]. dst and x must not overlap.
inline void axpy(double* dst, const double* x, double alpha, std::size_t n) {
    std::size_t i = 0;
#if defined(MATRIX_SIMD_AVX)
    const __m256d valpha = _mm256_set1_pd(alpha);
    for (; i + 4 <= n; i += 4) {
        const __m256d vy = _mm256_loadu_pd(dst + i);
        const __m256d vx = _mm256_loadu_pd(x + i);
        _mm256_storeu_pd(dst + i, _mm256_add_pd(vy, _mm256_mul_pd(vx, valpha)));
    }
#elif defined(MATRIX_SIMD_SSE2)
    const __m128d valpha = _mm_set1_pd(alpha);
    for (; i + 2 <= n; i += 2) {
        const __m128d vy = _mm_loadu_pd(dst + i);
        const __m128d vx = _mm_loadu_pd(x + i);
        _mm_storeu_pd(dst + i, _mm_add_pd(vy, _mm_mul_pd(vx, valpha)));
    }
#elif defined(MATRIX_SIMD_NEON)
    const float64x2_t valpha = vdupq_n_f64(alpha);
    for (; i + 2 <= n; i += 2) {
        const float64x2_t vy = vld1q_f64(dst + i);
        const float64x2_t vx = vld1q_f64(x + i);
        vst1q_f64(dst + i, vfmaq_f64(vy, vx, valpha));
    }
#endif
    for (; i < n; ++i)
        dst[i] += alpha * x[i];
}

}

#undef MATRIX_SIMD_AVX
#undef MATRIX_SIMD_SSE2
#undef MATRIX_SIMD_NEON

#endif //SENSOR_FUSION_SIMD_H
