// Port of: src/FoundationClasses/TKMath/Convert/Convert_ParabolaToBSplineCurve.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_ParabolaToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts an arc of a 2D parabola into a non-rational B-spline curve (degree 2, 3 poles of weight 1). The
/// parabola's parametrisation is P(U) = Loc + F * (U * U * XDir + 2 * U * YDir), F the focal length.
/// </summary>
public class ParabolaToBSplineCurve : ConicToBSplineCurve
{
    private const int TheDegree = 2;
    private const int MaxNbKnots = 2;
    private const int MaxNbPoles = 3;

    /// <summary>
    /// The B-spline curve equivalent to the parabola between the parameters <paramref name="u1"/> and
    /// <paramref name="u2"/>, with the parabola's orientation. Throws <see cref="DomainErrorException"/> if U1 = U2.
    /// </summary>
    public ParabolaToBSplineCurve(in Parabola2D prb, double u1, double u2)
        : base(MaxNbPoles, MaxNbKnots, TheDegree)
    {
        Standard_DomainError_Raise_if(Math.Abs(u2 - u1) < Epsilon(0.0), "Convert_ParabolaToBSplineCurve");

        double uf = Min(u1, u2);
        double ul = Max(u1, u2);

        double p = prb.Parameter;

        _isPeriodic = false;
        _knots[1] = uf;
        _mults[1] = 3;
        _knots[2] = ul;
        _mults[2] = 3;

        _weights[1] = 1.0;
        _weights[2] = 1.0;
        _weights[3] = 1.0;

        Direction2D ox = prb.Axis.XDirection;
        Direction2D oy = prb.Axis.YDirection;
        double s = (ox.X * oy.Y - ox.Y * oy.X > 0.0) ? 1 : -1;

        // poles expressed in the reference mark
        _poles[1] = new Point2D((uf * uf) / (2.0 * p), s * uf);
        _poles[2] = new Point2D((uf * ul) / (2.0 * p), s * (uf + ul) / 2.0);
        _poles[3] = new Point2D((ul * ul) / (2.0 * p), s * ul);

        // replace the bspline in the mark of the parabola
        Transform2D trsf = new Transform2D().WithTransformation(prb.Axis.XAxis, Constants.OX2d());
        _poles[1] = _poles[1].Transformed(trsf);
        _poles[2] = _poles[2].Transformed(trsf);
        _poles[3] = _poles[3].Transformed(trsf);
    }
}
