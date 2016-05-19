#pragma once

#include <cstdlib>

namespace PoseEstimation
{
    class Color
    {
    public:
        double r;
        double g;
        double b;

        Color(const Color &c)
            : r(c.r), g(c.g), b(c.b)
        {}

        Color(double r = 0, double g = 0, double b = 0)
            : r(r), g(g), b(b)
        {}

        __attribute__((weak))
        static const Color RED;
        __attribute__((weak))
        static const Color GREEN;
        __attribute__((weak))
        static const Color BLUE;
        __attribute__((weak))
        static const Color YELLOW;
        __attribute__((weak))
        static const Color MAGENTA;
        __attribute__((weak))
        static const Color CYAN;

        static inline Color fromRGB(double r, double g, double b)
        {
            return Color(r, g, b);
        }

        static inline Color fromHSL(double h, double s = 1.0, double l = 0.5)
        {
            static Color c;
            float q = l + s - l * s;
            float p = 2.0 * l - q;
            c.r = _hue2rgb(p, q, h + 1.0/3.0);
            c.g = _hue2rgb(p, q, h);
            c.b = _hue2rgb(p, q, h - 1.0/3.0);
            return c;
        }

        static inline Color random(double s = 1.0, double l = 0.5)
        {
            double h = std::rand() * 1.0 / RAND_MAX;
            return fromHSL(h, s, l);
        }

    private:
        static inline float _hue2rgb(double p, double q, double t)
        {
            if (t < 0.0) t += 1.0;
            if (t > 1.0) t -= 1.0;
            if (t < 1.0/6.0) return p + (q - p) * 6.0 * t;
            if (t < 1.0/2.0) return q;
            if (t < 2.0/3.0) return p + (q - p) * (2.0/3.0 - t) * 6.0;
            return p;
        }
    };

    const Color Color::RED(1, 0, 0);
    const Color Color::GREEN(0, 1, 0);
    const Color Color::BLUE(0, 0, 1);
    const Color Color::YELLOW(1, 1, 0);
    const Color Color::MAGENTA(0, 1, 1);
    const Color Color::CYAN(1, 0, 1);
}
