// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CompBezierSurfacesToBSplineSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CompBezierSurfacesToBSplineSurface.lxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CompBezierSurfacesToBSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Converts a grid of adjacent non-rational Bezier surfaces into the data of one B-spline surface. Beziers(i, j)
/// is the patch of U index i (row) and V index j (column); consecutive patches must share their bounding
/// iso-curves with compatible parameterisations (not checked). The results are read with Poles, UKnots, VKnots,
/// UMultiplicities, VMultiplicities, UDegree and VDegree, and passed to a Geom_BSplineSurface constructor.
/// </summary>
/// <remarks>
/// The grid's patches are not modified: OCCT raises their degrees in place to the grid's, the port raises copies.
/// </remarks>
public sealed class GeomConvert_CompBezierSurfacesToBSplineSurface
{
    private int myUDegree;
    private int myVDegree;
    private NCollection_HArray1<int> myVMults = null!;
    private NCollection_HArray1<int> myUMults = null!;
    private NCollection_HArray1<double> myUKnots;
    private NCollection_HArray1<double> myVKnots;
    private NCollection_HArray2<Point3D> myPoles = null!;
    private bool isrational;
    private bool myDone;

    /// <summary>
    /// The data of a C0 "piecewise Bezier" B-spline surface equivalent to the grid: knots 0, 1, 2, ... in each
    /// direction, the degrees the highest of the grid's, inner multiplicities equal to the degrees.
    /// </summary>
    /// <exception cref="MethodNotImplementedException">A patch is rational.</exception>
    public GeomConvert_CompBezierSurfacesToBSplineSurface(NCollection_Array2<Geom_BezierSurface> Beziers)
    {
        int ii;
        myDone = true;
        // Choice of knots
        myUKnots = new NCollection_HArray1<double>(1, Beziers.ColLength() + 1);
        for (ii = 0; ii < myUKnots.Length(); ii++)
        {
            myUKnots.SetValue(ii + 1, ii);
        }

        myVKnots = new NCollection_HArray1<double>(1, Beziers.RowLength() + 1);
        for (ii = 0; ii < myVKnots.Length(); ii++)
        {
            myVKnots.SetValue(ii + 1, ii);
        }

        // Compute the Poles
        Perform(Beziers);
    }

    /// <summary>
    /// The data of a B-spline surface equivalent to the grid, with knots spaced by the patches' pseudo-lengths
    /// along the iso-curves at 0.3, and as many inner knots removed as Tolerance allows (C0 is imposed without
    /// check). With RemoveKnots false, the knots are kept with multiplicity at least 1.
    /// </summary>
    /// <exception cref="MethodNotImplementedException">A patch is rational.</exception>
    /// <remarks>
    /// OCCT: the V knot removal loop tests the U multiplicity (occt-issues.md), so V knots are only removed when
    /// the last U removal failed, and then the loop may not end.
    /// </remarks>
    public GeomConvert_CompBezierSurfacesToBSplineSurface(NCollection_Array2<Geom_BezierSurface> Beziers,
                                                          double Tolerance,
                                                          bool RemoveKnots = true)
    {
        int ii, jj, multU = 0, multV, minus;
        bool Ok;
        Vector3D vec;
        double V1, V2, V3, Ratio, L1, L2, Tol, val;
        Point3D P1, P2, P3;
        Geom_Curve FirstCurve, SecondCurve;

        myDone = true;

        // Choice of knots

        myUKnots = new NCollection_HArray1<double>(1, Beziers.ColLength() + 1);
        myVKnots = new NCollection_HArray1<double>(1, Beziers.RowLength() + 1);

        // --> in U
        myUKnots.SetValue(1, 0);
        jj = myVKnots.Length() / 2;
        FirstCurve = Beziers[1, jj].VIso(0.3);
        FirstCurve.D0(0, out P1);
        FirstCurve.D0(0.5, out P2);
        FirstCurve.D1(1, out P3, out vec);

        L1 = P1.Distance(P2) + P2.Distance(P3);
        myUKnots.SetValue(2, L1);

        V1 = vec.Magnitude();
        // if the parameterization is too strange we keep the pseudo-length
        if ((V1 > 1000 * L1) || (V1 < L1 * 1.0e-3))
        {
            V1 = L1;
        }

        for (ii = 2; ii < myUKnots.Length(); ii++)
        {
            SecondCurve = Beziers[ii, jj].VIso(0.3);
            SecondCurve.D1(0, out P1, out vec);
            V2 = vec.Magnitude();
            SecondCurve.D0(0.5, out P2);
            SecondCurve.D1(1, out P3, out vec);
            V3 = vec.Magnitude();
            L2 = P1.Distance(P2) + P2.Distance(P3);

            // Compute the ratio, avoiding degenerate cases...
            if ((V2 > 1000 * L2) || (V2 < L2 * 1.0e-3))
            {
                V2 = L2;
            }

            if ((V3 > 1000 * L2) || (V3 < L2 * 1.0e-3))
            {
                V3 = L2;
            }

            Ratio = 1;
            if ((V1 > global::Kerf.Engine.Foundation.Precision.Confusion()) && (V2 > global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                Ratio = V2 / V1;
            }

            if ((Ratio < global::Kerf.Engine.Foundation.Precision.Confusion()) || (Ratio > 1 / global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                Ratio = 1;
            }

            // Deduce a new knot
            val = myUKnots.Value(ii);
            myUKnots.SetValue(ii + 1, val + Ratio * (val - myUKnots.Value(ii - 1)));

            // And off we go again, for another round
            L1 = L2;
            V1 = V3;
            FirstCurve = SecondCurve;
        }

        // --> en V
        myVKnots.SetValue(1, 0);
        ii = myUKnots.Length() / 2;
        FirstCurve = Beziers[ii, 1].UIso(0.3);
        FirstCurve.D0(0, out P1);
        FirstCurve.D0(0.5, out P2);
        FirstCurve.D1(1, out P3, out vec);

        L1 = P1.Distance(P2) + P2.Distance(P3);
        myVKnots.SetValue(2, L1);

        V1 = vec.Magnitude();
        // if the parameterization is too strange we keep the pseudo-length
        if ((V1 > 1000 * L1) || (V1 < L1 * 1.0e-3))
        {
            V1 = L1;
        }

        for (jj = 2; jj < myVKnots.Length(); jj++)
        {
            SecondCurve = Beziers[ii, jj].UIso(0.3);
            SecondCurve.D1(0, out P1, out vec);
            V2 = vec.Magnitude();
            SecondCurve.D0(0.5, out P2);
            SecondCurve.D1(1, out P3, out vec);
            V3 = vec.Magnitude();
            L2 = P1.Distance(P2) + P2.Distance(P3);

            // Compute the ratio, avoiding degenerate cases...
            if ((V2 > 1000 * L2) || (V2 < L2 * 1.0e-3))
            {
                V2 = L2;
            }

            if ((V3 > 1000 * L2) || (V3 < L2 * 1.0e-3))
            {
                V3 = L2;
            }

            Ratio = 1;
            if ((V1 > global::Kerf.Engine.Foundation.Precision.Confusion()) && (V2 > global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                Ratio = V2 / V1;
            }

            if ((Ratio < global::Kerf.Engine.Foundation.Precision.Confusion()) || (Ratio > 1 / global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                Ratio = 1;
            }

            // Deduce a new knot
            val = myVKnots.Value(jj);
            myVKnots.SetValue(jj + 1, val + Ratio * (val - myVKnots.Value(jj - 1)));

            // And off we go again, for another round
            L1 = L2;
            V1 = V3;
            FirstCurve = SecondCurve;
        }

        // Compute the Poles
        Perform(Beziers);

        // Reduction de la multiplicite
        var Surface = new Geom_BSplineSurface(myPoles,
                                              myUKnots,
                                              myVKnots,
                                              myUMults,
                                              myVMults,
                                              myUDegree,
                                              myVDegree);

        if (RemoveKnots)
        {
            minus = 0;
        }
        else
        {
            minus = 1;
        }

        for (ii = myUKnots.Length() - 1; ii > 1; ii--)
        {
            Ok = true;
            Tol = Tolerance / 2;
            multU = myUMults.Value(ii) - 1;
            for (; Ok && multU > minus; multU--, Tol /= 2)
            {
                Ok = Surface.RemoveUKnot(ii, multU, Tol);
            }
        }

        for (ii = myVKnots.Length() - 1; ii > 1; ii--)
        {
            Ok = true;
            Tol = Tolerance / 2;
            multV = myVMults.Value(ii) - 1;
            for (; Ok && multU > minus; multV--, Tol /= 2) // OCCT: tests multU, not multV
            {
                Ok = Surface.RemoveVKnot(ii, multV, Tol);
            }
        }

        // The new fields have been computed....
        myPoles = new NCollection_HArray2<Point3D>(Surface.InternalPoles());
        myUMults = new NCollection_HArray1<int>(Surface.InternalUMultiplicities());
        myVMults = new NCollection_HArray1<int>(Surface.InternalVMultiplicities());
        myUKnots = new NCollection_HArray1<double>(Surface.InternalUKnots());
        myVKnots = new NCollection_HArray1<double>(Surface.InternalVKnots());
    }

    /// <summary>
    /// The data of a B-spline surface equivalent to the grid, with the given knots (one more than the patches in
    /// each direction), made of continuity UContinuity and VContinuity (C0 to C3) by knot removal where the
    /// deformation stays below Tolerance; IsDone is false where it doesn't.
    /// </summary>
    /// <exception cref="ConstructionErrorException">
    /// A continuity above C3 or G1/G2, or a continuity not below the degree in a direction with several patches.
    /// </exception>
    /// <exception cref="MethodNotImplementedException">A patch is rational.</exception>
    public GeomConvert_CompBezierSurfacesToBSplineSurface(NCollection_Array2<Geom_BezierSurface> Beziers,
                                                          NCollection_Array1<double> UKnots,
                                                          NCollection_Array1<double> VKnots,
                                                          Continuity UContinuity = C0,
                                                          Continuity VContinuity = C0,
                                                          double Tolerance = 1.0e-4)
    {
        int decu = 0, decv = 0;
        bool Ok;

        myDone = true;

        // Retrieve the knots
        myUKnots = new NCollection_HArray1<double>(1, Beziers.ColLength() + 1);
        myUKnots.Assign(UKnots);

        myVKnots = new NCollection_HArray1<double>(1, Beziers.RowLength() + 1);
        myVKnots.Assign(VKnots);

        // Compute the Poles
        Perform(Beziers);

        // Obtaining the correct continuity

        switch (UContinuity)
        {
            case C0:
                decu = 0;
                break;
            case C1:
                decu = 1;
                break;
            case C2:
                decu = 2;
                break;
            case C3:
                decu = 3;
                break;
            default:
                throw new ConstructionErrorException("GeomConvert_CompBezierSurfacesToBSpl:: UContinuity error");
        }

        switch (VContinuity)
        {
            case C0:
                decv = 0;
                break;
            case C1:
                decv = 1;
                break;
            case C2:
                decv = 2;
                break;
            case C3:
                decv = 3;
                break;
            default:
                throw new ConstructionErrorException("GeomConvert_CompBezierSurfacesToBSpl:: VContinuity error");
        }

        if ((decu > 0) || (decv > 0))
        {
            int ii;
            int multU = myUDegree - decu;
            Standard_ConstructionError_Raise_if(
                ((multU <= 0) && (myUKnots.Length() > 2)),
                "GeomConvert_CompBezierSurfacesToBSpl:: UContinuity or Udeg error");

            int multV = myVDegree - decv;
            Standard_ConstructionError_Raise_if(
                ((multV <= 0) && (myVKnots.Length() > 2)),
                "GeomConvert_CompBezierSurfacesToBSpl:: VContinuity or Vdeg error");

            var Surface = new Geom_BSplineSurface(myPoles,
                                                  myUKnots,
                                                  myVKnots,
                                                  myUMults,
                                                  myVMults,
                                                  myUDegree,
                                                  myVDegree);

            if (decu > 0)
            {
                for (ii = 2; ii < myUKnots.Length(); ii++)
                {
                    Ok = Surface.RemoveUKnot(ii, multU, Tolerance);
                    if (!Ok)
                    {
                        myDone = Ok;
                    }
                }
            }

            if (decv > 0)
            {
                for (ii = 2; ii < myVKnots.Length(); ii++)
                {
                    Ok = Surface.RemoveVKnot(ii, multV, Tolerance);
                    if (!Ok)
                    {
                        myDone = Ok;
                    }
                }
            }

            // The new fields have been computed....
            myPoles = new NCollection_HArray2<Point3D>(Surface.InternalPoles());
            myUMults = new NCollection_HArray1<int>(Surface.InternalUMultiplicities());
            myVMults = new NCollection_HArray1<int>(Surface.InternalVMultiplicities());
        }
    }

    /// <summary>Used by the constructors: degrees, poles and multiplicities of the grid for the current knots.</summary>
    private void Perform(NCollection_Array2<Geom_BezierSurface> Beziers)
    {
        int IU, IV;

        // (1) Determine the degrees and whether the result is rational.
        isrational = false;
        myUDegree = 1;
        myVDegree = 1;

        for (IU = Beziers.LowerRow(); IU <= Beziers.UpperRow(); IU++)
        {
            for (IV = Beziers.LowerCol(); IV <= Beziers.UpperCol(); IV++)
            {
                if (Beziers[IU, IV].IsURational() || Beziers[IU, IV].IsVRational())
                {
                    isrational = true;
                }

                myUDegree = (Beziers[IU, IV].UDegree() > myUDegree) ? Beziers[IU, IV].UDegree() : myUDegree;

                myVDegree = (Beziers[IU, IV].VDegree() > myVDegree) ? Beziers[IU, IV].VDegree() : myVDegree;
            }
        }

        Standard_NotImplemented_Raise_if(isrational, "CompBezierSurfacesToBSpl : rational !");

        // (2) Loop over the patches  -----------------------------

        Geom_BezierSurface Patch;
        int UIndex, VIndex, uindex, vindex, udeb, vdeb;
        int upol, vpol, ii;

        myPoles = new NCollection_HArray2<Point3D>(
            1,
            (myUDegree + 1) * Beziers.ColLength() - myUKnots.Length() + 2,
            1,
            (myVDegree + 1) * Beziers.RowLength() - myVKnots.Length() + 2);

        for (IU = Beziers.LowerRow(); IU <= Beziers.UpperRow(); IU++)
        {
            UIndex = (IU - 1) * myUDegree + 1;
            for (IV = Beziers.LowerCol(); IV <= Beziers.UpperCol(); IV++)
            {
                // OCCT raises the degree of the caller's patch in place; the port raises a copy
                Patch = (Geom_BezierSurface)Beziers[IU, IV].Copy();
                VIndex = (IV - 1) * myVDegree + 1;

                // (2.1) Increase the degree
                Patch.Increase(myUDegree, myVDegree);

                // (2.2) Poles to copy
                if (IU == 1)
                {
                    udeb = 1;
                }
                else
                {
                    udeb = 2;
                }

                if (IV == 1)
                {
                    vdeb = 1;
                }
                else
                {
                    vdeb = 2;
                }

                uindex = UIndex + udeb - 1;

                for (upol = udeb; upol <= myUDegree + 1; upol++, uindex++)
                {
                    vindex = VIndex + vdeb - 1;
                    for (vpol = vdeb; vpol <= myVDegree + 1; vpol++, vindex++)
                    {
                        myPoles[uindex, vindex] = Patch.Pole(upol, vpol);
                    }
                }

                // (2.3) Poles a sommer
                if (udeb == 2)
                {
                    vindex = VIndex + vdeb - 1;
                    for (vpol = vdeb; vpol <= myVDegree + 1; vpol++, vindex++)
                    {
                        myPoles[UIndex, vindex].ChangeCoord() += Patch.Pole(1, vpol).Coord();
                    }
                }

                if (vdeb == 2)
                {
                    uindex = UIndex + udeb - 1;
                    for (upol = udeb; upol <= myUDegree + 1; upol++, uindex++)
                    {
                        myPoles[uindex, VIndex].ChangeCoord() += Patch.Pole(upol, 1).Coord();
                    }
                }

                if (udeb == 2 && vdeb == 2)
                {
                    myPoles[UIndex, VIndex].ChangeCoord() += Patch.Pole(1, 1).Coord();
                }
            }
        }

        // (3) Elimination of redundancies
        // because in the previous loop we count:
        // - 2 times the poles associated with simple knots
        // - 4 times the poles associated with double knots (in U and V)

        // (3.1) Elimination in U
        for (UIndex = myUDegree + 1, ii = 2; ii < myUKnots.Length(); ii++, UIndex += myUDegree)
        {
            for (vpol = 1; vpol <= myPoles.UpperCol(); vpol++)
            {
                myPoles[UIndex, vpol].ChangeCoord() *= 0.5;
            }
        }

        // (3.2) Elimination en V
        for (VIndex = myVDegree + 1, ii = 2; ii < myVKnots.Length(); ii++, VIndex += myVDegree)
        {
            for (upol = 1; upol <= myPoles.UpperRow(); upol++)
            {
                myPoles[upol, VIndex].ChangeCoord() *= 0.5;
            }
        }

        // (4) Init of multiplicities
        myUMults = new NCollection_HArray1<int>(1, myUKnots.Length());
        myUMults.Init(myUDegree);
        myUMults.SetValue(1, myUDegree + 1);
        myUMults.SetValue(myUMults.Upper(), myUDegree + 1);

        myVMults = new NCollection_HArray1<int>(1, myVKnots.Length());
        myVMults.Init(myVDegree);
        myVMults.SetValue(1, myVDegree + 1);
        myVMults.SetValue(myVMults.Upper(), myVDegree + 1);
    }

    /// <summary>The number of knots in U of the computed surface.</summary>
    public int NbUKnots() => myUKnots.Length();

    /// <summary>The number of poles in U of the computed surface.</summary>
    public int NbUPoles() => myPoles.ColLength();

    /// <summary>The number of knots in V of the computed surface.</summary>
    public int NbVKnots() => myVKnots.Length();

    /// <summary>The number of poles in V of the computed surface.</summary>
    public int NbVPoles() => myPoles.RowLength();

    /// <summary>A copy of the poles of the computed surface.</summary>
    public NCollection_HArray2<Point3D> Poles() => new(myPoles);

    /// <summary>A copy of the U knots of the computed surface.</summary>
    public NCollection_HArray1<double> UKnots() => new(myUKnots);

    /// <summary>The U degree of the computed surface.</summary>
    public int UDegree() => myUDegree;

    /// <summary>A copy of the V knots of the computed surface.</summary>
    public NCollection_HArray1<double> VKnots() => new(myVKnots);

    /// <summary>The V degree of the computed surface.</summary>
    public int VDegree() => myVDegree;

    /// <summary>A copy of the U multiplicities of the computed surface.</summary>
    public NCollection_HArray1<int> UMultiplicities() => new(myUMults);

    /// <summary>A copy of the V multiplicities of the computed surface.</summary>
    public NCollection_HArray1<int> VMultiplicities() => new(myVMults);

    /// <summary>
    /// False when a continuity requested at construction could not be reached within the tolerance (the data are
    /// then those of the surface as far as it could be smoothed).
    /// </summary>
    public bool IsDone() => myDone;
}
