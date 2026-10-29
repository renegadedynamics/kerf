// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Check2dBSplineCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Check2dBSplineCurve.lxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_Check2dBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Checks the end tangents of a 2D B-spline curve: whether they are reversed with respect to the third or n-3rd pole,
/// and builds a curve with fixed tangents.
/// </summary>
public sealed class GeomLib_Check2dBSplineCurve
{
    private readonly Geom2d_BSplineCurve myCurve;
    private bool myDone;
    private readonly bool myFixFirstTangent;
    private readonly bool myFixLastTangent;
    private readonly double myAngularTolerance;
    private readonly double myTolerance;

    private readonly int myIndSecondPole;
    private readonly int myIndPrelastPole;

    /// <summary>Checks the end tangents of Curve, with a distance and an angular tolerance.</summary>
    public GeomLib_Check2dBSplineCurve(Geom2d_BSplineCurve Curve, double Tolerance, double AngularTolerance)
    {
        myCurve = Curve;
        myDone = false;
        myFixFirstTangent = false;
        myFixLastTangent = false;
        myAngularTolerance = Math.Abs(AngularTolerance);
        myTolerance = Math.Abs(Tolerance);
        myIndSecondPole = -1;
        myIndPrelastPole = -1;

        int ii, num_poles;
        double tangent_magnitude, value, vector_magnitude;
        num_poles = myCurve.NbPoles();

        if ((!myCurve.IsPeriodic()) && num_poles >= 4)
        {

            gp_Vec2d tangent, tangent_normalized = new(), a_vector, avector_normalized;

            double CrossProdTol = myAngularTolerance;

            // Near first
            tangent = new gp_Vec2d(myCurve.Pole(1), myCurve.Pole(2));
            tangent_magnitude = tangent.Magnitude();
            if (tangent_magnitude > myTolerance)
            {
                tangent_normalized = tangent / tangent_magnitude;
            }

            for (ii = 3; ii <= num_poles; ii++)
            {
                a_vector = new gp_Vec2d(myCurve.Pole(1), myCurve.Pole(ii));
                vector_magnitude = a_vector.Magnitude();

                if (tangent_magnitude > myTolerance && vector_magnitude > myTolerance)
                {
                    avector_normalized = a_vector / vector_magnitude;

                    double CrossProd = tangent_normalized ^ avector_normalized;
                    if (Math.Abs(CrossProd) > CrossProdTol)
                    {
                        break;
                    }

                    value = tangent.Dot(a_vector);
                    if (value < 0.0e0)
                    {
                        myFixFirstTangent = true;
                        myIndSecondPole = ii;
                        break;
                    }
                }
            }

            // Near last
            tangent = new gp_Vec2d(myCurve.Pole(num_poles), myCurve.Pole(num_poles - 1));
            tangent_magnitude = tangent.Magnitude();
            if (tangent_magnitude > myTolerance)
            {
                tangent_normalized = tangent / tangent_magnitude;
            }

            for (ii = num_poles - 2; ii >= 1; ii--)
            {
                a_vector = new gp_Vec2d(myCurve.Pole(num_poles), myCurve.Pole(ii));
                vector_magnitude = a_vector.Magnitude();

                if (tangent_magnitude > myTolerance && vector_magnitude > myTolerance)
                {
                    avector_normalized = a_vector / vector_magnitude;

                    double CrossProd = tangent_normalized ^ avector_normalized;
                    if (Math.Abs(CrossProd) > CrossProdTol)
                    {
                        break;
                    }

                    value = tangent.Dot(a_vector);
                    if (value < 0.0e0)
                    {
                        myFixLastTangent = true;
                        myIndPrelastPole = ii;
                        break;
                    }
                }
            }
        } // if (( ! myCurve->IsPeriodic() )&& num_poles >= 4)
        else
        {
            myDone = true;
        }
    }

    /// <summary>Whether the check is complete: the curve needs no fix, or a fix was made.</summary>
    public bool IsDone() => myDone;

    /// <summary>Whether the first and the last tangents need a fix.</summary>
    public void NeedTangentFix(out bool FirstFlag, out bool LastFlag)
    {
        FirstFlag = myFixFirstTangent;
        LastFlag = myFixLastTangent;
    }

    /// <summary>
    /// Fixes the tangents of the checked curve in place, as OCCT does. Internal: only ported code calls it, on a
    /// curve it has just built (a frozen curve throws Standard_ImmutableObject). Callers use <see cref="FixedTangent"/>.
    /// </summary>
    internal void FixTangent(bool FirstFlag, bool LastFlag)
    {
        FixTangentOnCurve(myCurve, FirstFlag, LastFlag);
    }

    /// <summary>
    /// A copy of the curve with the first and/or last tangents fixed, or null when no requested end needs a fix.
    /// </summary>
    public Geom2d_BSplineCurve? FixedTangent(bool FirstFlag, bool LastFlag)
    {
        Geom2d_BSplineCurve? new_curve = null;
        if ((myFixFirstTangent && FirstFlag) || (myFixLastTangent && LastFlag))
        {
            new_curve = myCurve.Copy() as Geom2d_BSplineCurve;

            FixTangentOnCurve(new_curve!, FirstFlag, LastFlag);
        }
        return new_curve;
    }

    private void FixTangentOnCurve(Geom2d_BSplineCurve theCurve, bool FirstFlag, bool LastFlag)
    {
        if (myFixFirstTangent && FirstFlag)
        {
            gp_XY XY1 = theCurve.Pole(1).XY();
            gp_XY XY2 = theCurve.Pole(myIndSecondPole).XY();
            double NbSamples = myIndSecondPole - 1;
            for (int i = 2; i < myIndSecondPole; i++)
            {
                double ii = i - 1;
                gp_Pnt2d aNewPole = new((1.0 - ii / NbSamples) * XY1 + ii / NbSamples * XY2);
                theCurve.SetPole(i, aNewPole);
            }
        }

        if (myFixLastTangent && LastFlag)
        {
            int num_poles = theCurve.NbPoles();

            gp_XY XY1 = theCurve.Pole(num_poles).XY();
            gp_XY XY2 = theCurve.Pole(myIndPrelastPole).XY();
            double NbSamples = num_poles - myIndPrelastPole;
            for (int i = num_poles - 1; i > myIndPrelastPole; i--)
            {
                double ii = num_poles - i;
                gp_Pnt2d aNewPole = new((1.0 - ii / NbSamples) * XY1 + ii / NbSamples * XY2);
                theCurve.SetPole(i, aNewPole);
            }
        }

        myDone = true;
    }
}
