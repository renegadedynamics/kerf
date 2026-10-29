// Port of: src/FoundationClasses/TKMath/Convert/Convert_ParabolaToBSplineCurve.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_ParabolaToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Converts an arc of a 2D parabola into a non-rational B-spline curve (degree 2, 3 poles of weight 1). The
/// parabola's parametrisation is P(U) = Loc + F * (U * U * XDir + 2 * U * YDir), F the focal length.
/// </summary>
public class Convert_ParabolaToBSplineCurve : Convert_ConicToBSplineCurve
{
    private const int TheDegree = 2;
    private const int MaxNbKnots = 2;
    private const int MaxNbPoles = 3;

    /// <summary>
    /// The B-spline curve equivalent to the parabola between the parameters <paramref name="U1"/> and
    /// <paramref name="U2"/>, with the parabola's orientation. Throws <see cref="Standard_DomainError"/> if U1 = U2.
    /// </summary>
    public Convert_ParabolaToBSplineCurve(in gp_Parab2d Prb, double U1, double U2)
        : base(MaxNbPoles, MaxNbKnots, TheDegree)
    {
        Standard_DomainError_Raise_if(Math.Abs(U2 - U1) < Epsilon(0.0), "Convert_ParabolaToBSplineCurve");

        double UF = Min(U1, U2);
        double UL = Max(U1, U2);

        double p = Prb.Parameter();

        myIsPeriodic = false;
        myKnots[1] = UF;
        myMults[1] = 3;
        myKnots[2] = UL;
        myMults[2] = 3;

        myWeights[1] = 1.0;
        myWeights[2] = 1.0;
        myWeights[3] = 1.0;

        gp_Dir2d Ox = Prb.Axis().XDirection();
        gp_Dir2d Oy = Prb.Axis().YDirection();
        double S = (Ox.X() * Oy.Y() - Ox.Y() * Oy.X() > 0.0) ? 1 : -1;

        // poles expressed in the reference mark
        myPoles[1] = new gp_Pnt2d((UF * UF) / (2.0 * p), S * UF);
        myPoles[2] = new gp_Pnt2d((UF * UL) / (2.0 * p), S * (UF + UL) / 2.0);
        myPoles[3] = new gp_Pnt2d((UL * UL) / (2.0 * p), S * UL);

        // replace the bspline in the mark of the parabola
        gp_Trsf2d Trsf = new();
        Trsf.SetTransformation(Prb.Axis().XAxis(), gp.OX2d());
        myPoles[1].Transform(Trsf);
        myPoles[2].Transform(Trsf);
        myPoles[3].Transform(Trsf);
    }
}
