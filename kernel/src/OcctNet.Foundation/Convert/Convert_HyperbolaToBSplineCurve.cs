// Port of: src/FoundationClasses/TKMath/Convert/Convert_HyperbolaToBSplineCurve.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_HyperbolaToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Converts an arc of a 2D hyperbola into a rational B-spline curve (degree 2, 3 poles). The hyperbola's
/// parametrisation is P(U) = Loc + MajorRadius * cosh(U) * XDir + MinorRadius * sinh(U) * YDir.
/// </summary>
public class Convert_HyperbolaToBSplineCurve : Convert_ConicToBSplineCurve
{
    private const int TheDegree = 2;
    private const int MaxNbKnots = 2;
    private const int MaxNbPoles = 3;

    /// <summary>
    /// The B-spline curve equivalent to the hyperbola between the parameters <paramref name="U1"/> and
    /// <paramref name="U2"/>, with the hyperbola's orientation. Throws <see cref="Standard_DomainError"/> if U1 = U2.
    /// </summary>
    public Convert_HyperbolaToBSplineCurve(in gp_Hypr2d H, double U1, double U2)
        : base(MaxNbPoles, MaxNbKnots, TheDegree)
    {
        Standard_DomainError_Raise_if(Math.Abs(U2 - U1) < Epsilon(0.0), "Convert_HyperbolaToBSplineCurve");

        double UF = Min(U1, U2);
        double UL = Max(U1, U2);

        myIsPeriodic = false;
        myKnots[1] = UF;
        myMults[1] = 3;
        myKnots[2] = UL;
        myMults[2] = 3;

        // construction of hyperbola in the reference xOy.

        double R = H.MajorRadius();
        double r = H.MinorRadius();
        gp_Dir2d Ox = H.Axis().XDirection();
        gp_Dir2d Oy = H.Axis().YDirection();
        double S = (Ox.X() * Oy.Y() - Ox.Y() * Oy.X() > 0.0) ? 1 : -1;

        // poles expressed in the reference mark
        // the 2nd pole is at the intersection of 2 tangents to the curve
        // at points P(UF), P(UL)
        // the weight of this pole is equal to : std::cosh((UL-UF)/2)

        myWeights[1] = 1.0;
        myWeights[2] = Math.Cosh((UL - UF) / 2);
        myWeights[3] = 1.0;

        double delta = Math.Sinh(UL - UF);
        double x = R * (Math.Sinh(UL) - Math.Sinh(UF)) / delta;
        double y = S * r * (Math.Cosh(UL) - Math.Cosh(UF)) / delta;
        myPoles[1] = new gp_Pnt2d(R * Math.Cosh(UF), S * r * Math.Sinh(UF));
        myPoles[2] = new gp_Pnt2d(x, y);
        myPoles[3] = new gp_Pnt2d(R * Math.Cosh(UL), S * r * Math.Sinh(UL));

        // replace the bspline in the mark of the hyperbola
        gp_Trsf2d Trsf = new();
        Trsf.SetTransformation(H.Axis().XAxis(), gp.OX2d());
        myPoles[1].Transform(Trsf);
        myPoles[2].Transform(Trsf);
        myPoles[3].Transform(Trsf);
    }
}
