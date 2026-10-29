// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CompCurveToBSplineCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CompCurveToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;

namespace Occt;

/// <summary>
/// Converts bounded 3D curves to B-splines and concatenates them into one B-spline curve, reparametrised for C1
/// continuity where the tangents allow it. A working object: <c>Add</c> appends curves.
/// </summary>
public sealed class GeomConvert_CompCurveToBSplineCurve
{
    private Geom_BSplineCurve? myCurve;
    private double myTol;
    private readonly Convert_ParameterisationType myType;

    /// <summary>An empty concatenation; theParameterisation is used to convert the conics added.</summary>
    public GeomConvert_CompCurveToBSplineCurve(Convert_ParameterisationType theParameterisation = Convert_TgtThetaOver2)
    {
        myTol = Precision.Confusion();
        myType = theParameterisation;
    }

    /// <summary>A concatenation starting with BasisCurve; Parameterisation is used to convert conics.</summary>
    public GeomConvert_CompCurveToBSplineCurve(Geom_BoundedCurve BasisCurve,
                                               Convert_ParameterisationType Parameterisation = Convert_TgtThetaOver2)
    {
        myTol = Precision.Confusion();
        myType = Parameterisation;
        if (BasisCurve is Geom_BSplineCurve)
        {
            myCurve = (Geom_BSplineCurve)BasisCurve.Copy();
        }
        else
        {
            myCurve = GeomConvert.CurveToBSplineCurve(BasisCurve, myType);
        }
    }

    /// <summary>
    /// Appends NewCurve at either end of the B-spline (reversing it if needed). Returns false if neither of its end
    /// points is within Tolerance of an end point of the B-spline. Tolerance is also used to lower the multiplicity
    /// of the joint knot, down to MinM (0: the knot can be removed). If the result would be closed, After chooses
    /// the end: true appends after the last point. WithRatio reparametrises the added curve for C1 continuity;
    /// false keeps its parametrisation, which can slow down algorithms such as CPnts_AbscissaPoint on the result.
    /// </summary>
    public bool Add(Geom_BoundedCurve NewCurve, double Tolerance, bool After = false, bool WithRatio = true,
                    int MinM = 0)
    {
        // conversion
        Geom_BSplineCurve Bs;
        if (NewCurve is Geom_BSplineCurve)
        {
            Bs = (Geom_BSplineCurve)NewCurve.Copy();
        }
        else
        {
            Bs = GeomConvert.CurveToBSplineCurve(NewCurve, myType);
        }

        if (myCurve is null)
        {
            myCurve = Bs;
            return true;
        }

        bool avant, apres;
        myTol = Tolerance;

        // Use actual curve endpoints instead of poles for proper G0 continuity check.
        // G0 continuity (positional continuity) requires curves to share endpoints.
        // For non-clamped or periodic B-splines, first/last poles may not coincide with endpoints.
        gp_Pnt aCurveStart = myCurve.StartPoint();
        gp_Pnt aCurveEnd = myCurve.EndPoint();
        gp_Pnt aBsStart = Bs.StartPoint();
        gp_Pnt aBsEnd = Bs.EndPoint();

        avant = ((aCurveStart.Distance(aBsStart) < myTol) || (aCurveStart.Distance(aBsEnd) < myTol));
        apres = ((aCurveEnd.Distance(aBsStart) < myTol) || (aCurveEnd.Distance(aBsEnd) < myTol));

        // Will myCurve be (or become) closed?
        if (avant && apres)
        { // Resolve the ambiguity
            if (After)
            {
                avant = false;
            }
            else
            {
                apres = false;
            }
        }

        // The private Add may raise the degree of the current curve in place: a curve handed out by
        // BSplineCurve() (frozen) is copied first, so it never changes.
        if ((apres || avant) && myCurve.IsFrozen())
        {
            myCurve = (Geom_BSplineCurve)myCurve.Copy();
        }

        // OCCT passes the field by reference; the private Add only changes its degree, then assigns myCurve.
        Geom_BSplineCurve aCurrent = myCurve;

        // Append after?
        if (apres)
        {
            if (aCurveEnd.Distance(aBsEnd) < myTol)
            {
                Bs.Reverse();
            }

            Add(ref aCurrent, ref Bs, true, WithRatio, MinM);
            return true;
        }
        // Prepend before?
        else if (avant)
        {
            if (aCurveStart.Distance(aBsStart) < myTol)
            {
                Bs.Reverse();
            }

            Add(ref Bs, ref aCurrent, false, WithRatio, MinM);
            return true;
        }

        return false;
    }

    // Concat two BSplineCurves.
    private void Add(ref Geom_BSplineCurve FirstCurve, ref Geom_BSplineCurve SecondCurve, bool After, bool WithRatio,
                     int MinM)
    {
        // Harmonize the degrees.
        int Deg = Max(FirstCurve.Degree(), SecondCurve.Degree());
        if (FirstCurve.Degree() < Deg)
        {
            FirstCurve.IncreaseDegree(Deg);
        }

        if (SecondCurve.Degree() < Deg)
        {
            SecondCurve.IncreaseDegree(Deg);
        }

        // Declarations
        double L1, L2;
        int ii, jj;
        double Ratio = 1, Ratio1, Ratio2, Delta1, Delta2;
        int NbP1 = FirstCurve.NbPoles(), NbP2 = SecondCurve.NbPoles();
        int NbK1 = FirstCurve.NbKnots(), NbK2 = SecondCurve.NbKnots();
        NCollection_Array1<double> Noeuds = new(1, NbK1 + NbK2 - 1);
        NCollection_Array1<gp_Pnt> Poles = new(1, NbP1 + NbP2 - 1);
        NCollection_Array1<double> Poids = new(1, NbP1 + NbP2 - 1);
        NCollection_Array1<int> Mults = new(1, NbK1 + NbK2 - 1);

        // Reparameterization ratio (C1 if possible)
        if (WithRatio)
        {
            L1 = FirstCurve.DN(FirstCurve.LastParameter(), 1).Magnitude();
            L2 = SecondCurve.DN(SecondCurve.FirstParameter(), 1).Magnitude();

            if ((L1 > Precision.Confusion()) && (L2 > Precision.Confusion()))
            {
                Ratio = L1 / L2;
            }

            if ((Ratio < Precision.Confusion()) || (Ratio > 1 / Precision.Confusion()))
            {
                Ratio = 1;
            }
        }

        if (After)
        {
            // Do not move the first curve
            Ratio1 = 1;
            Delta1 = 0;
            Ratio2 = 1 / Ratio;
            Delta2 = Ratio2 * SecondCurve.Knot(1) - FirstCurve.Knot(NbK1);
        }
        else
        {
            // Do not move the second curve
            Ratio1 = Ratio;
            Delta1 = Ratio1 * FirstCurve.Knot(NbK1) - SecondCurve.Knot(1);
            Ratio2 = 1;
            Delta2 = 0;
        }

        // The Knots
        double eps;
        for (ii = 1; ii <= NbK1; ii++)
        {
            Noeuds[ii] = Ratio1 * FirstCurve.Knot(ii) - Delta1;
            if (ii > 1)
            {
                eps = Epsilon(Math.Abs(Noeuds[ii - 1]));
                if (eps < 5.0e-10)
                {
                    eps = 5.0e-10;
                }

                if (Noeuds[ii] - Noeuds[ii - 1] <= eps)
                {
                    Noeuds[ii] += eps;
                }
            }

            Mults[ii] = FirstCurve.Multiplicity(ii);
        }

        Mults[NbK1] = FirstCurve.Degree();
        for (ii = 2, jj = NbK1 + 1; ii <= NbK2; ii++, jj++)
        {
            Noeuds[jj] = Ratio2 * SecondCurve.Knot(ii) - Delta2;
            eps = Epsilon(Math.Abs(Noeuds[jj - 1]));
            if (eps < 5.0e-10)
            {
                eps = 5.0e-10;
            }

            if (Noeuds[jj] - Noeuds[jj - 1] <= eps)
            {
                Noeuds[jj] += eps;
            }

            Mults[jj] = SecondCurve.Multiplicity(ii);
        }

        Ratio = FirstCurve.Weight(NbP1);
        Ratio /= SecondCurve.Weight(1);
        // The Poles and Weights
        for (ii = 1; ii < NbP1; ii++)
        {
            Poles[ii] = FirstCurve.Pole(ii);
            Poids[ii] = FirstCurve.Weight(ii);
        }

        for (ii = 1, jj = NbP1; ii <= NbP2; ii++, jj++)
        {
            Poles[jj] = SecondCurve.Pole(ii);
            //
            // Note: the weights may not necessarily connect with C0 continuity,
            // hence the use of Ratio
            //
            Poids[jj] = Ratio * SecondCurve.Weight(ii);
        }

        // Create the BSpline
        Geom_BSplineCurve aCurve = new(Poles, Poids, Noeuds, Mults, Deg);
        myCurve = aCurve;

        // Optionally reduce multiplicity down to MinM
        bool Ok = true;
        int M = Mults[NbK1];
        while ((M > MinM) && Ok)
        {
            M--;
            Ok = aCurve.RemoveKnot(NbK1, M, myTol);
        }
    }

    /// <summary>The concatenated B-spline (frozen: the same object until the next Add), or null if empty.</summary>
    public Geom_BSplineCurve? BSplineCurve()
    {
        myCurve?.Freeze();
        return myCurve;
    }

    // The result without freezing it, for ported code that discards this object (GeomConvert's statics).
    internal Geom_BSplineCurve? InternalBSplineCurve() => myCurve;

    /// <summary>Clears the result.</summary>
    public void Clear() => myCurve = null;
}
