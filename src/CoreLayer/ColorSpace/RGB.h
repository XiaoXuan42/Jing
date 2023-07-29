#pragma once
#include <CoreLayer/Math/Math.h>
#include <CoreLayer/Utils.h>

#include <iostream>
#include <ostream>
//* 使用RGB三通道颜色空间

class SpectrumRGB {
public:
    SpectrumRGB() : rgb(0.f) {}

    SpectrumRGB(float f) : rgb(f) {}

    SpectrumRGB(float r, float g, float b) : rgb(r, g, b) {}

    SpectrumRGB(Vector3f _rgb) : rgb(_rgb) {}

    SpectrumRGB operator+(const SpectrumRGB &rhs) const {
        return SpectrumRGB(rgb + rhs.rgb);
    }

    SpectrumRGB &operator+=(const SpectrumRGB &rhs) {
        rgb += rhs.rgb;
        return *this;
    }

    SpectrumRGB operator-(const SpectrumRGB &rhs) const {
        return SpectrumRGB(rgb - rhs.rgb);
    }

    SpectrumRGB &operator-=(const SpectrumRGB &rhs) {
        rgb -= rhs.rgb;
        return *this;
    }

    SpectrumRGB operator*(const SpectrumRGB &rhs) const {
        return SpectrumRGB(rgb * rhs.rgb);
    }

    SpectrumRGB &operator*=(const SpectrumRGB &rhs) {
        rgb *= rhs.rgb;
        return *this;
    }

    SpectrumRGB operator*(float f) const { return SpectrumRGB(rgb * f); }

    SpectrumRGB &operator*=(float f) {
        rgb *= f;
        return *this;
    }

    SpectrumRGB operator/(const SpectrumRGB &rhs) const {
        return SpectrumRGB(rgb / rhs.rgb);
    }

    SpectrumRGB &operator/=(const SpectrumRGB &rhs) {
        rgb /= rhs.rgb;
        return *this;
    }

    SpectrumRGB operator/(float f) const { return SpectrumRGB(rgb / f); }

    SpectrumRGB &operator/=(float f) {
        rgb /= f;
        return *this;
    }

    float operator[](int i) const { return rgb[i]; }

    float &operator[](int i) { return rgb[i]; }

    bool isZero() const { return rgb.isZero(); }

    void debugPrint() const {
        printf("[rgb](");
        for (int i = 0; i < 3; ++i) {
            std::cout << (i == 0 ? '\0' : ',') << rgb[i];
        }
        printf(")%c", '\n');
        fflush(stdout);
    }

    static int cntChannel() { return 3; }

    static SpectrumRGB blackBody(float t) {
        int idx = t / blackbody_step_size;
        int idx2 = idx + 1;
        float c = (t - blackbody_step_size * idx) / blackbody_step_size;
        const float *s1 = nullptr, *s2 = nullptr;

        if (idx2 > blackbody_table_max_index) {
            s1 = blackbody_table[blackbody_table_max_index];
            return SpectrumRGB(s1[0], s1[1], s1[2]);
        }
        s1 = blackbody_table[idx];
        s2 = blackbody_table[idx2];
        float r = s1[0] + c * (s2[0] - s1[0]), g = s1[1] + c * (s2[1] - s1[1]),
              b = s1[2] + c * (s2[2] - s1[2]);
        return SpectrumRGB(r, g, b);
    }

private:
    static constexpr float blackbody_table[][3] = {
        {0.0, 0.0, 0.0},
        {3.884807387278206e-75, -4.2213374023827593e-76,
         -2.6091241747206567e-77},
        {6.6397888197395605e-37, -7.21490876020439e-38, -4.459515020725815e-39},
        {1.1139687625815974e-23, -1.2059057993077572e-24,
         -7.538624792601123e-26},
        {1.3971894542147655e-16, -1.4481189369826652e-17,
         -1.0255520947726915e-18},
        {4.713194381923347e-12, -4.3594745668220553e-13,
         -4.1092881078421953e-14},
        {6.287860371455172e-09, -4.827040398346214e-10, -6.691546162714129e-11},
        {1.191367738700279e-06, -6.913672589633753e-08, -1.533913795986454e-08},
        {6.40303978279593e-05, -2.381243278413397e-06, -9.753124708134025e-07},
        {0.0014596795555848034, -2.1651890483381037e-05,
         -2.5586643719615425e-05},
        {0.018094610714823385, 0.0001557864638571948, -0.0003541993110514076},
        {0.1433148151579503, 0.004703090171082871, -0.003034825988121136},
        {0.8088390455278637, 0.04653822196042297, -0.017912091623545096},
        {3.5111050596482136, 0.2899257185006324, -0.07834219988858107},
        {12.386838597010042, 1.334929874242545, -0.2666853486944127},
        {36.99081979407975, 4.919759255118806, -0.7277879411053572},
        {96.4258837460271, 15.250800960580698, -1.6058270641588175},
        {224.660048497098, 41.151507632974585, -2.7852600017066926},
        {476.55689973183433, 99.10846442517065, -3.270950058376723},
        {933.9470121050686, 217.13676028329792, -0.027727036411469896},
        {1711.0263462105252, 439.23262462720766, 13.872326961122894}};
    static constexpr int blackbody_step_size = 100;
    static constexpr int blackbody_highest_temperature = 2000;
    static constexpr int blackbody_table_max_index =
        sizeof(blackbody_table) / (sizeof(float) * 3) - 1;

    Vector3f rgb;
};

inline SpectrumRGB operator*(float f, const SpectrumRGB &spectrum) {
    return spectrum * f;
}

inline Vector3f toVec3(const SpectrumRGB &spectrum) {
    return Vector3f{spectrum[0], spectrum[1], spectrum[2]};
}

inline SpectrumRGB toSpectrum(const Vector3f &vec) {
    return SpectrumRGB(vec[0], vec[1], vec[2]);
}