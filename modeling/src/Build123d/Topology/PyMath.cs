namespace Build123d;

/// <summary>Python's numeric behaviour, where the port must match it exactly.</summary>
internal static class PyMath
{
    /// <summary>Python's <c>%</c> on floats: the result takes the sign of the divisor.</summary>
    internal static double Mod(double a, double b)
    {
        // As CPython computes it: fmod, then moved into the divisor's sign
        var r = a % b;
        if (r != 0 && (r < 0) != (b < 0))
            r += b;
        return r;
    }

    /// <summary>
    /// SciPy's bounded scalar minimisation (<c>minimize_scalar(method="bounded")</c>, Brent's method on an interval),
    /// returning the minimising x and the function's value there.
    /// </summary>
    internal static (double X, double Fun) MinimizeBounded(Func<double, double> func, double x1, double x2,
                                                           double xatol = 1e-5, int maxiter = 500)
    {
        double sqrtEps = Math.Sqrt(2.2e-16);
        double goldenMean = 0.5 * (3.0 - Math.Sqrt(5.0));
        double a = x1, b = x2;
        double fulc = a + goldenMean * (b - a);
        double nfc = fulc, xf = fulc;
        double rat = 0.0, e = 0.0;
        double x = xf;
        double fx = func(x);
        int num = 1;
        double ffulc = fx, fnfc = fx;
        double xm = 0.5 * (a + b);
        double tol1 = sqrtEps * Math.Abs(xf) + xatol / 3.0;
        double tol2 = 2.0 * tol1;

        while (Math.Abs(xf - xm) > tol2 - 0.5 * (b - a))
        {
            bool golden = true;
            // Check for a parabolic fit
            if (Math.Abs(e) > tol1)
            {
                golden = false;
                double r = (xf - nfc) * (fx - ffulc);
                double q = (xf - fulc) * (fx - fnfc);
                double p = (xf - fulc) * q - (xf - nfc) * r;
                q = 2.0 * (q - r);
                if (q > 0.0)
                    p = -p;
                q = Math.Abs(q);
                r = e;
                e = rat;
                // Is the parabola acceptable?
                if (Math.Abs(p) < Math.Abs(0.5 * q * r) && p > q * (a - xf) && p < q * (b - xf))
                {
                    rat = (p + 0.0) / q;
                    x = xf + rat;
                    if (x - a < tol2 || b - x < tol2)
                    {
                        double s = Math.Sign(xm - xf) + (xm - xf == 0 ? 1 : 0);
                        rat = tol1 * s;
                    }
                }
                else
                    golden = true;
            }
            if (golden)
            {
                e = xf >= xm ? a - xf : b - xf;
                rat = goldenMean * e;
            }
            double si = Math.Sign(rat) + (rat == 0 ? 1 : 0);
            x = xf + si * Math.Max(Math.Abs(rat), tol1);
            double fu = func(x);
            num++;
            if (fu <= fx)
            {
                if (x >= xf)
                    a = xf;
                else
                    b = xf;
                (fulc, ffulc) = (nfc, fnfc);
                (nfc, fnfc) = (xf, fx);
                (xf, fx) = (x, fu);
            }
            else
            {
                if (x < xf)
                    a = x;
                else
                    b = x;
                if (fu <= fnfc || nfc == xf)
                {
                    (fulc, ffulc) = (nfc, fnfc);
                    (nfc, fnfc) = (x, fu);
                }
                else if (fu <= ffulc || fulc == xf || fulc == nfc)
                    (fulc, ffulc) = (x, fu);
            }
            xm = 0.5 * (a + b);
            tol1 = sqrtEps * Math.Abs(xf) + xatol / 3.0;
            tol2 = 2.0 * tol1;
            if (num >= maxiter)
                break;
        }
        return (xf, fx);
    }
}
