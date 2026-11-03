// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_CompCurveToBSplineCurve.hxx
// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_CompCurveToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace Occt;

/// <summary>
/// Converts bounded 2D curves to B-splines and concatenates them into one B-spline curve, reparametrised for C1
/// continuity where the tangents allow it. A working object: <c>Add</c> appends curves.
/// </summary>
public sealed class Geom2dConvert_CompCurveToBSplineCurve
{
    private Geom2d_BSplineCurve? myCurve;
    private double myTol;
    private readonly ParameterisationType myType;

    /// <summary>An empty concatenation; Parameterisation is used to convert the conics added.</summary>
    public Geom2dConvert_CompCurveToBSplineCurve(ParameterisationType Parameterisation = TgtThetaOver2)
    {
        myTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        myType = Parameterisation;
    }

    /// <summary>A concatenation starting with BasisCurve; Parameterisation is used to convert conics.</summary>
    public Geom2dConvert_CompCurveToBSplineCurve(Geom2d_BoundedCurve BasisCurve,
                                                 ParameterisationType Parameterisation = TgtThetaOver2)
    {
        myTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        myType = Parameterisation;
        if (BasisCurve is Geom2d_BSplineCurve)
        {
            myCurve = (Geom2d_BSplineCurve)BasisCurve.Copy();
        }
        else
        {
            myCurve = Geom2dConvert.CurveToBSplineCurve(BasisCurve, myType);
        }
    }

    /// <summary>
    /// Appends NewCurve at either end of the B-spline (reversing it if needed). Returns false if neither of its ends
    /// is within Tolerance of an end of the B-spline. Tolerance is also used to lower the multiplicity of the joint
    /// knot. If the result would be closed, After chooses the end: true appends after the last point.
    /// </summary>
    public bool Add(Geom2d_BoundedCurve NewCurve, double Tolerance, bool After = false)
    {
        // conversion
        Geom2d_BSplineCurve Bs;
        if (NewCurve is Geom2d_BSplineCurve)
        {
            Bs = (Geom2d_BSplineCurve)NewCurve.Copy();
        }
        else
        {
            Bs = Geom2dConvert.CurveToBSplineCurve(NewCurve, myType);
        }

        if (myCurve is null)
        {
            myCurve = Bs;
            return true;
        }

        myTol = Tolerance;
        double aSqTol = Tolerance * Tolerance;

        int LBs = Bs.NbPoles(), LCb = myCurve.NbPoles();
        double d1 = myCurve.Pole(1).SquareDistance(Bs.Pole(1));
        double d2 = myCurve.Pole(1).SquareDistance(Bs.Pole(LBs));

        bool isBeforeReversed = (myCurve.Pole(1).SquareDistance(Bs.Pole(1)) < aSqTol) && (d1 < d2);
        bool isBefore = (myCurve.Pole(1).SquareDistance(Bs.Pole(LBs)) < aSqTol) || isBeforeReversed;

        d1 = myCurve.Pole(LCb).SquareDistance(Bs.Pole(1));
        d2 = myCurve.Pole(LCb).SquareDistance(Bs.Pole(LBs));

        bool isAfterReversed = (myCurve.Pole(LCb).SquareDistance(Bs.Pole(LBs)) < aSqTol) && (d2 < d1);
        bool isAfter = (myCurve.Pole(LCb).SquareDistance(Bs.Pole(1)) < aSqTol) || isAfterReversed;

        // myCurve and NewCurve together form a closed curve
        if (isBefore && isAfter)
        {
            if (After)
            {
                isBefore = false;
            }
            else
            {
                isAfter = false;
            }
        }

        // The private Add may raise the degree of the current curve in place: a curve handed out by
        // BSplineCurve() (frozen) is copied first, so it never changes.
        if ((isAfter || isBefore) && myCurve.IsFrozen)
        {
            myCurve = (Geom2d_BSplineCurve)myCurve.Copy();
        }

        // OCCT passes the field by reference; the private Add only reads it, then assigns myCurve.
        Geom2d_BSplineCurve aCurrent = myCurve;
        if (isAfter)
        {
            if (isAfterReversed)
            {
                Bs.Reverse();
            }

            Add(ref aCurrent, ref Bs, true);
            return true;
        }
        else if (isBefore)
        {
            if (isBeforeReversed)
            {
                Bs.Reverse();
            }

            Add(ref Bs, ref aCurrent, false);
            return true;
        }

        return false;
    }

    // Concat two BSplineCurves.
    private void Add(ref Geom2d_BSplineCurve FirstCurve, ref Geom2d_BSplineCurve SecondCurve, bool After)
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
        double L1, L2, U_de_raccord;
        int ii, jj;
        double Ratio = 1, Ratio1, Ratio2, Delta1, Delta2;
        int NbP1 = FirstCurve.NbPoles(), NbP2 = SecondCurve.NbPoles();
        int NbK1 = FirstCurve.NbKnots(), NbK2 = SecondCurve.NbKnots();
        NCollection_Array1<double> Noeuds = new(1, NbK1 + NbK2 - 1);
        NCollection_Array1<Point2D> Poles = new(1, NbP1 + NbP2 - 1);
        NCollection_Array1<double> Poids = new(1, NbP1 + NbP2 - 1);
        NCollection_Array1<int> Mults = new(1, NbK1 + NbK2 - 1);

        // Reparameterization ratio (C1 if possible)
        L1 = FirstCurve.DN(FirstCurve.LastParameter(), 1).Magnitude();
        L2 = SecondCurve.DN(SecondCurve.FirstParameter(), 1).Magnitude();

        if ((L1 > global::Kerf.Engine.Foundation.Precision.Confusion()) && (L2 > global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            Ratio = L1 / L2;
        }

        if ((Ratio < global::Kerf.Engine.Foundation.Precision.Confusion()) || (Ratio > 1 / global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            Ratio = 1;
        }

        if (After)
        {
            // Do not move the first curve
            Ratio1 = 1;
            Delta1 = 0;
            Ratio2 = 1 / Ratio;
            Delta2 = Ratio2 * SecondCurve.Knot(1) - FirstCurve.Knot(NbK1);
            U_de_raccord = FirstCurve.LastParameter();
        }
        else
        {
            // Do not move the second curve
            Ratio1 = Ratio;
            Delta1 = Ratio1 * FirstCurve.Knot(NbK1) - SecondCurve.Knot(1);
            Ratio2 = 1;
            Delta2 = 0;
            U_de_raccord = SecondCurve.FirstParameter();
        }

        // The Knots

        for (ii = 1; ii < NbK1; ii++)
        {
            Noeuds[ii] = Ratio1 * FirstCurve.Knot(ii) - Delta1;
            Mults[ii] = FirstCurve.Multiplicity(ii);
        }

        Noeuds[NbK1] = U_de_raccord;
        Mults[NbK1] = FirstCurve.Degree();
        for (ii = 2, jj = NbK1 + 1; ii <= NbK2; ii++, jj++)
        {
            Noeuds[jj] = Ratio2 * SecondCurve.Knot(ii) - Delta2;
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
        Geom2d_BSplineCurve aCurve = new(Poles, Poids, Noeuds, Mults, Deg);
        myCurve = aCurve;

        // Optionally reduce multiplicity
        bool Ok = true;
        int M = Mults[NbK1];
        while ((M > 0) && Ok)
        {
            M--;
            Ok = aCurve.RemoveKnot(NbK1, M, myTol);
        }
    }

    /// <summary>The concatenated B-spline (frozen: the same object until the next Add), or null if empty.</summary>
    public Geom2d_BSplineCurve? BSplineCurve()
    {
        myCurve?.Freeze();
        return myCurve;
    }

    // The result without freezing it, for ported code that discards this object (Geom2dConvert's statics).
    internal Geom2d_BSplineCurve? InternalBSplineCurve() => myCurve;

    /// <summary>Clears the result.</summary>
    public void Clear() => myCurve = null;
}
