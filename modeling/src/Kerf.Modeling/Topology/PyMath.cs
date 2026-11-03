namespace Kerf.Modeling;

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

    /// <summary>
    /// SciPy's Nelder-Mead minimisation of a function of one variable within bounds (<c>minimize(method="Nelder-Mead",
    /// bounds=[(lower, upper)], tol=tol)</c>), returning the minimising x.
    /// </summary>
    internal static double MinimizeNelderMead(Func<double, double> func, double x0, double lower, double upper, double tol)
    {
        const double rho = 1, chi = 2, psi = 0.5, sigma = 0.5;
        const int maxIterations = 200, maxCalls = 200;
        double Clip(double x) => Math.Clamp(x, lower, upper);

        x0 = Clip(x0);
        double[] sim = [x0, x0 != 0 ? (1 + 0.05) * x0 : 0.00025];
        // Reflect a vertex above the upper bound into the interior, then clip
        for (int k = 0; k < 2; k++)
            sim[k] = Clip(sim[k] > upper ? 2 * upper - sim[k] : sim[k]);

        int calls = 0;
        double? F(double x)
        {
            if (calls >= maxCalls)
                return null;
            calls++;
            return func(x);
        }
        double[] fsim = [double.PositiveInfinity, double.PositiveInfinity];
        void Sort()
        {
            // argsort of two values: stable
            if (fsim[1] < fsim[0])
            {
                (sim[0], sim[1]) = (sim[1], sim[0]);
                (fsim[0], fsim[1]) = (fsim[1], fsim[0]);
            }
        }
        for (int k = 0; k < 2; k++)
        {
            if (F(sim[k]) is not { } f)
                break;
            fsim[k] = f;
        }
        Sort();

        int iterations = 1;
        while (calls < maxCalls && iterations < maxIterations)
        {
            if (Math.Abs(sim[1] - sim[0]) <= tol && Math.Abs(fsim[0] - fsim[1]) <= tol)
                break;
            if (Step() is false)
            {
                Sort();
                continue;
            }
            iterations++;
            Sort();
        }
        return sim[0];

        // One iteration; false when the call budget ran out part way
        bool Step()
        {
            var xbar = sim[0];
            var xr = Clip((1 + rho) * xbar - rho * sim[1]);
            if (F(xr) is not { } fxr)
                return false;
            if (fxr < fsim[0])
            {
                var xe = Clip((1 + rho * chi) * xbar - rho * chi * sim[1]);
                if (F(xe) is not { } fxe)
                    return false;
                (sim[1], fsim[1]) = fxe < fxr ? (xe, fxe) : (xr, fxr);
                return true;
            }
            // With one variable fsim[-2] is fsim[0], so the reflection is never kept here
            bool shrink;
            if (fxr < fsim[1])
            {
                var xc = Clip((1 + psi * rho) * xbar - psi * rho * sim[1]);
                if (F(xc) is not { } fxc)
                    return false;
                shrink = !(fxc <= fxr);
                if (!shrink)
                    (sim[1], fsim[1]) = (xc, fxc);
            }
            else
            {
                var xcc = Clip((1 - psi) * xbar + psi * sim[1]);
                if (F(xcc) is not { } fxcc)
                    return false;
                shrink = !(fxcc < fsim[1]);
                if (!shrink)
                    (sim[1], fsim[1]) = (xcc, fxcc);
            }
            if (shrink)
            {
                sim[1] = Clip(sim[0] + sigma * (sim[1] - sim[0]));
                if (F(sim[1]) is not { } fs)
                    return false;
                fsim[1] = fs;
            }
            return true;
        }
    }
}
