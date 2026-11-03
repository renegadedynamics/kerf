// Port of: src/FoundationClasses/TKMath/Convert/Convert_HyperbolaToBSplineCurve.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_HyperbolaToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts an arc of a 2D hyperbola into a rational B-spline curve (degree 2, 3 poles). The hyperbola's
/// parametrisation is P(U) = Loc + MajorRadius * cosh(U) * XDir + MinorRadius * sinh(U) * YDir.
/// </summary>
public class HyperbolaToBSplineCurve : ConicToBSplineCurve
{
    private const int TheDegree = 2;
    private const int MaxNbKnots = 2;
    private const int MaxNbPoles = 3;

    /// <summary>
    /// The B-spline curve equivalent to the hyperbola between the parameters <paramref name="u1"/> and
    /// <paramref name="u2"/>, with the hyperbola's orientation. Throws <see cref="DomainErrorException"/> if U1 = U2.
    /// </summary>
    public HyperbolaToBSplineCurve(in Hyperbola2D h, double u1, double u2)
        : base(MaxNbPoles, MaxNbKnots, TheDegree)
    {
        Standard_DomainError_Raise_if(Math.Abs(u2 - u1) < Epsilon(0.0), "Convert_HyperbolaToBSplineCurve");

        double uf = Min(u1, u2);
        double ul = Max(u1, u2);

        _isPeriodic = false;
        _knots[1] = uf;
        _mults[1] = 3;
        _knots[2] = ul;
        _mults[2] = 3;

        // construction of hyperbola in the reference xOy.

        double R = h.MajorRadius();
        double r = h.MinorRadius();
        Direction2D ox = h.Axis().XDirection();
        Direction2D oy = h.Axis().YDirection();
        double s = (ox.X() * oy.Y() - ox.Y() * oy.X() > 0.0) ? 1 : -1;

        // poles expressed in the reference mark
        // the 2nd pole is at the intersection of 2 tangents to the curve
        // at points P(UF), P(UL)
        // the weight of this pole is equal to : std::cosh((UL-UF)/2)

        _weights[1] = 1.0;
        _weights[2] = Math.Cosh((ul - uf) / 2);
        _weights[3] = 1.0;

        double delta = Math.Sinh(ul - uf);
        double x = R * (Math.Sinh(ul) - Math.Sinh(uf)) / delta;
        double y = s * r * (Math.Cosh(ul) - Math.Cosh(uf)) / delta;
        _poles[1] = new Point2D(R * Math.Cosh(uf), s * r * Math.Sinh(uf));
        _poles[2] = new Point2D(x, y);
        _poles[3] = new Point2D(R * Math.Cosh(ul), s * r * Math.Sinh(ul));

        // replace the bspline in the mark of the hyperbola
        Transform2D trsf = new Transform2D().WithTransformation(h.Axis().XAxis(), Constants.OX2d());
        _poles[1] = _poles[1].Transformed(trsf);
        _poles[2] = _poles[2].Transformed(trsf);
        _poles[3] = _poles[3].Transformed(trsf);
    }
}
