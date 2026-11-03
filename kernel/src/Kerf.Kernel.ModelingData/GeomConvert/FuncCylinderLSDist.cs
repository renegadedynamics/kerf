// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncCylinderLSDist.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncCylinderLSDist.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// The least-squares criterion for fitting a cylinder of given axis direction dir to points P(i):
/// F(x0, y0, z0, R) = Sum[|(P(i) - Loc)^dir|^2 - R^2]^2, with its gradient, for math_BFGS. The variables are the
/// axis location (X(1) to X(3)) and the radius X(4).
/// </summary>
/// <remarks>
/// OCCT: <see cref="Gradient"/> scales G(6) instead of G(4) (occt-issues.md). OCCT's release build writes past
/// the 4-element vector and returns an unscaled radius derivative; the port throws. <see cref="Values"/> is right.
/// </remarks>
internal sealed class FuncCylinderLSDist : MultipleVarFunctionWithGradient
{
    private NCollection_HArray1<Xyz>? _points;
    private Direction3D _dir = new Direction3D();

    /// <summary>A function without points (set them with <see cref="SetPoints"/>).</summary>
    public FuncCylinderLSDist()
    {
    }

    /// <summary>The function for thePoints (kept, not copied) and the axis direction theDir.</summary>
    public FuncCylinderLSDist(NCollection_HArray1<Xyz> points, in Direction3D dir)
    {
        _points = points;
        _dir = dir;
    }

    /// <summary>Sets the points (kept, not copied).</summary>
    public void SetPoints(NCollection_HArray1<Xyz> points) => _points = points;

    /// <summary>Sets the axis direction.</summary>
    public void SetDir(in Direction3D dir) => _dir = dir;

    /// <summary>4: the axis location and the radius.</summary>
    public override int VariableCount => 4;

    /// <summary>The criterion at X.</summary>
    public override bool Value(math_Vector x, out double f)
    {
        var loc = new Xyz(x[1], x[2], x[3]);
        double r2 = x[4] * x[4];
        f = 0.0;
        int i;
        for (i = _points!.Lower(); i <= _points.Upper(); ++i)
        {
            var v = new Vector3D(_points.Value(i) - loc);
            double d2 = v.CrossSquareMagnitude(new Vector3D(_dir));
            double d = d2 - r2;
            f += d * d;
        }

        return true;
    }

    /// <summary>The gradient at X.</summary>
    /// <exception cref="OutOfRangeException">Always, after OCCT's error described on the class.</exception>
    public override bool Gradient(math_Vector X, math_Vector g)
    {
        var loc = new Xyz(X[1], X[2], X[3]);
        double r = X[4], r2 = r * r;
        double x = _dir.X, y = _dir.Y, z = _dir.Z;
        g.Init(0.0);
        int i;
        for (i = _points!.Lower(); i <= _points.Upper(); ++i)
        {
            var v = new Vector3D(_points.Value(i) - loc);
            double d2 = v.CrossSquareMagnitude(new Vector3D(_dir));
            double d = d2 - r2;
            double dx0 = 2.0 * (v.Z * x - v.X * z) * z - 2.0 * (v.X * y - v.Y * x) * y;
            double dy0 = -2.0 * (v.Y * z - v.Z * y) * z + 2.0 * (v.X * y - v.Y * x) * x;
            double dz0 = 2.0 * (v.Y * z - v.Z * y) * y - 2.0 * (v.Z * x - v.X * z) * x;
            g[1] += d * dx0;
            g[2] += d * dy0;
            g[3] += d * dz0;
            //
            g[4] += d;
        }

        g *= 2;
        g[6] *= -2.0 * r; // OCCT: G(6) instead of G(4), past the end of the vector
        return true;
    }

    /// <summary>The criterion and its gradient at X.</summary>
    public override bool Values(math_Vector X, out double f, math_Vector g)
    {
        var loc = new Xyz(X[1], X[2], X[3]);
        double r = X[4], r2 = r * r;
        double x = _dir.X, y = _dir.Y, z = _dir.Z;
        f = 0.0;
        g.Init(0.0);
        int i;
        for (i = _points!.Lower(); i <= _points.Upper(); ++i)
        {
            var v = new Vector3D(_points.Value(i) - loc);
            double d2 = v.CrossSquareMagnitude(new Vector3D(_dir));
            double d = d2 - r2;
            double dx0 = 2.0 * (v.Z * x - v.X * z) * z - 2.0 * (v.X * y - v.Y * x) * y;
            double dy0 = -2.0 * (v.Y * z - v.Z * y) * z + 2.0 * (v.X * y - v.Y * x) * x;
            double dz0 = 2.0 * (v.Y * z - v.Z * y) * y - 2.0 * (v.Z * x - v.X * z) * x;
            g[1] += d * dx0;
            g[2] += d * dy0;
            g[3] += d * dz0;
            //
            g[4] += d;
            //
            f += d * d;
        }

        g *= 2;
        g[4] *= -2.0 * r;
        return true;
    }
}
