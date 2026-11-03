// Port of: src/Draw/TKTopTest/GeometryTest/GeometryTest_CurveCommands.cxx (to2d, to3d, project, projonplane, crvpoints, crvtpoints)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Engine.Numerics;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using math_Vector = Kerf.Engine.Numerics.VectorBase<double>;

namespace OcctNet.DrawLite;

/// <summary>
/// GeometryTest_CurveCommands.cxx's projections and discretisations (unit 7W): <c>to2d</c> and <c>to3d</c>
/// (GeomAPI), <c>project</c> (GeomProjLib::Curve2d), <c>projonplane</c> (GeomProjLib::ProjectOnPlane), <c>crvpoints</c>
/// (GCPnts_QuasiUniformDeflection) and <c>crvtpoints</c> (GCPnts_TangentialDeflection) with their deviation check.
/// </summary>
/// <remarks>
/// The file's other commands live elsewhere (<c>bisec</c> in <see cref="GeometryTestBisecCommands"/>, <c>intersect</c>
/// in <see cref="GeometryTestIntersectCommands"/>, <c>getcurvcontinuity</c> in
/// <see cref="GeometryTestContinuityCommands"/>). <c>crvpoints</c> and <c>crvtpoints</c> bind the polyline as a plain
/// B-spline curve: DrawTrSurf_BSplineCurve's display settings (no poles, knots drawn as orange crosses) need a view.
/// They read their arguments without checking their count, as OCCT; a missing word reads as empty (0).
/// </remarks>
internal static class GeometryTestCurveCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "to2d", To2d);
        DrawCxx.Add(theInterpreter, "to3d", To3d);
        DrawCxx.Add(theInterpreter, "project", Project);
        DrawCxx.Add(theInterpreter, "projonplane", ProjOnPlane);
        DrawCxx.Add(theInterpreter, "crvpoints", CrvPoints);
        DrawCxx.Add(theInterpreter, "crvtpoints", CrvTPoints);
    }

    // a[i], or an empty word past the end (Draw's argv[argc] is a null pointer)
    private static string Arg(IReadOnlyList<string> a, int i) => i < a.Count ? a[i] : "";

    // to2d c2dname c3d [plane (XOY)]
    private static int To2d(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        // get the curve
        Curve3D? C = DrawTrSurf.GetCurve(theDI, a[2]);
        if (C is null)
        {
            return 1;
        }

        Surface? S;
        if (n >= 4)
        {
            S = DrawTrSurf.GetSurface(theDI, a[3]);
            if (S is null)
            {
                return 1;
            }
        }
        else
        {
            S = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        }

        if (S is not PlaneSurface P)
        {
            return 1;
        }
        Curve2D? r = GeomAPI.To2d(C, P.Pln());
        DrawTrSurf.Set(theDI, a[1], r);
        return 0;
    }

    // to3d c3dname c2d [plane (XOY)]
    private static int To3d(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        Curve2D? C = DrawTrSurf.GetCurve2d(theDI, a[2]);
        if (C is null)
        {
            return 1;
        }

        Surface? S;
        if (n >= 4)
        {
            S = DrawTrSurf.GetSurface(theDI, a[3]);
            if (S is null)
            {
                return 1;
            }
        }
        else
        {
            S = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        }

        if (S is not PlaneSurface P)
        {
            return 1;
        }
        Curve3D r = GeomAPI.To3d(C, P.Pln());

        DrawTrSurf.Set(theDI, a[1], r);
        return 0;
    }

    // project result2d c3d surf [-e p] [-v n] [-t tol]
    private static int Project(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n == 1)
        {
            di.Append("project result2d c3d surf [-e p] [-v n] [-t tol]\n");
            di.Append("   -e p   : extent the surface of <p>%\n");
            di.Append("   -v n   : verify the projection at <n> points.\n");
            di.Append("   -t tol : set the tolerance for approximation\n");
            return 0;
        }

        if (n < 4)
        {
            return 1;
        }
        Surface? GS = DrawTrSurf.GetSurface(theDI, a[3]);
        if (GS is null)
        {
            return 1;
        }

        Curve3D? GC = DrawTrSurf.GetCurve(theDI, a[2]);
        if (GC is null)
        {
            return 1;
        }

        double tolerance = global::Kerf.Engine.Foundation.Precision.Confusion();

        GS.Bounds(out double U1, out double U2, out double V1, out double V2);

        bool Verif = false;
        int NbPoints = 0;

        int index = 4;
        while (index + 1 < n)
        {
            if (a[index].Length == 0 || a[index][0] != '-')
            {
                return 1;
            }

            char anOption = a[index].Length > 1 ? a[index][1] : '\0';
            if (anOption == 'e')
            {
                double p = theDI.Atof(a[index + 1]);
                double dU = p * (U2 - U1) / 100.0;
                double dV = p * (V2 - V1) / 100.0;
                U1 -= dU;
                U2 += dU;
                V1 -= dV;
                V2 += dV;
            }
            else if (anOption == 'v')
            {
                Verif = true;
                NbPoints = theDI.Atoi(a[index + 1]);
            }
            else if (anOption == 't')
            {
                tolerance = theDI.Atof(a[index + 1]);
            }
            index += 2;
        }

        Curve2D? G2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(GC, GS, U1, U2, V1, V2, ref tolerance);

        if (G2d is null)
        {
            di.Append("\nProjection Failed\n");
            return 1;
        }
        else
        {
            DrawTrSurf.Set(theDI, a[1], G2d);
        }
        if (Verif)
        { // verify the projection on n points
            if (NbPoints <= 0)
            {
                di.Append(" n must be positive\n");
                return 0;
            }

            double DistMax = -1.0;
            U1 = GC.FirstParameter();
            U2 = GC.LastParameter();
            double dU = (U2 - U1) / (NbPoints + 1);
            for (int i = 0; i <= NbPoints + 1; i++)
            {
                double U = U1 + i * dU;
                Point3D P1 = GC.Value(U);
                Point2D P2d = G2d.Value(U);
                Point3D P2 = GS.Value(P2d.X(), P2d.Y());
                double Dist = P1.Distance(P2);
                di.Append(" Parameter = ").Append(U).Append("\tDistance = ").Append(Dist).Append("\n");
                if (Dist > DistMax)
                {
                    DistMax = Dist;
                }
            }
            di.Append(" **** Distance Maximale : ").Append(DistMax).Append("\n");
        }

        return 0;
    }

    // projonplane r C3d Plane [dx dy dz] [0/1]
    private static int ProjOnPlane(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        Surface? S = DrawTrSurf.GetSurface(theDI, a[3]);
        if (S is null)
        {
            return 1;
        }

        if (S is not PlaneSurface Pl)
        {
            di.Append(" The surface must be a plane\n");
            return 1;
        }

        Curve3D? C = DrawTrSurf.GetCurve(theDI, a[2]);
        if (C is null)
        {
            return 1;
        }

        bool Param = true;
        if ((n == 5 && theDI.Atoi(a[4]) == 0) || (n == 8 && theDI.Atoi(a[7]) == 0))
        {
            Param = false;
        }

        Direction3D D;

        if (n == 8)
        {
            D = new Direction3D(theDI.Atof(a[4]), theDI.Atof(a[5]), theDI.Atof(a[6]));
        }
        else
        {
            D = Pl.Pln().Position().Direction();
        }

        Curve3D? Res = global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(C, Pl, D, Param);

        DrawTrSurf.Set(theDI, a[1], Res);
        return 0;
    }

    // the curve, or the wire as a composite curve; null after the failure message
    private static CurveAdaptorBase? CurveOrWire(DrawInterpreter theDI, string theName)
    {
        Curve3D? C = DrawTrSurf.GetCurve(theDI, theName);
        if (C is not null)
        {
            return new CurveAdaptor(C);
        }

        // try getting a wire
        Wire aWire = ShapeCasts.Wire(theDI.GetShape(theName, ShapeType.Wire));
        if (aWire.IsNull)
        {
            // Message::SendFail (decision 405)
            theDI.Output.WriteLine($"cannot evaluate the argument {theName} as a curve");
            return null;
        }
        var aCompCurve = new WireCurveAdaptor(aWire);
        return aCompCurve;
    }

    // the polyline through the points as a degree 1 B-spline curve, bound, and its deviation from the curve
    private static void BindPolyline(DrawInterpreter theDI, DrawResult di, string theName, CurveAdaptorBase theCurve, int nbp,
                                     Func<int, Point3D> theValue, Func<int, double> theParameter)
    {
        var aPoles = new NCollection_Array1<Point3D>(1, nbp);
        var aKnots = new NCollection_Array1<double>(1, nbp);
        var aMults = new NCollection_Array1<int>(1, nbp);

        for (int i = 1; i <= nbp; ++i)
        {
            aPoles[i] = theValue(i);
            aKnots[i] = theParameter(i);
            aMults[i] = 1;
        }

        aMults[1] = 2;
        aMults[nbp] = 2;

        var aPnts = new BSplineCurve3D(aPoles, aKnots, aMults, 1);
        DrawTrSurf.Set(theDI, theName, aPnts);

        // check deviation
        ComputeDeviation(theCurve, aPnts, out double dmax, out double ufmax, out double ulmax, out int imax);
        di.Append("Max defl: ").Append(dmax).Append(" ").Append(ufmax).Append(" ").Append(ulmax).Append(" ").Append(imax).Append("\n");
    }

    // crvpoints result <curve or wire> deflection // OCCT: the word count isn't checked
    private static int CrvPoints(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        CurveAdaptorBase? aHCurve = CurveOrWire(theDI, Arg(a, 2));
        if (aHCurve is null)
        {
            return 1;
        }

        double defl = theDI.Atof(Arg(a, 3));

        var PntGen = new QuasiUniformDeflection(aHCurve, defl);

        if (!PntGen.IsDone)
        {
            di.Append("Points generation failed\n");
            return 1;
        }

        int nbp = PntGen.PointCount;
        di.Append("Nb points : ").Append(nbp).Append("\n");

        BindPolyline(theDI, di, a[1], aHCurve, nbp, PntGen.Value, PntGen.Parameter);
        return 0;
    }

    // crvtpoints result <curve or wire> deflection angular deflection - tangential deflection points
    private static int CrvTPoints(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        int aMinPntsNb = 2;
        double angle = global::Kerf.Engine.Foundation.Precision.Angular();

        CurveAdaptorBase? aHCurve = CurveOrWire(theDI, Arg(a, 2));
        if (aHCurve is null)
        {
            return 1;
        }
        double defl = theDI.Atof(Arg(a, 3));

        if (n > 4)
        {
            angle = theDI.Atof(a[4]);
        }

        if (n > 5)
        {
            aMinPntsNb = theDI.Atoi(a[5]);
        }

        var PntGen = new TangentialDeflection(aHCurve, angle, defl, aMinPntsNb);

        int nbp = PntGen.PointCount;
        di.Append("Nb points : ").Append(nbp).Append("\n");

        BindPolyline(theDI, di, a[1], aHCurve, nbp, PntGen.Value, PntGen.Parameter);
        return 0;
    }

    // Static method computing deviation of curve and polyline
    private static void ComputeDeviation(CurveAdaptorBase theCurve, BSplineCurve3D thePnts, out double theDmax, out double theUfMax,
                                         out double theUlMax, out int theImax)
    {
        theDmax = 0.0;
        theUfMax = 0.0;
        theUlMax = 0.0;
        theImax = 0;

        // take knots
        PackedList<double> aKnots = thePnts.Knots();
        int nbp = aKnots.Count;

        for (int i = 1; i < nbp; ++i)
        {
            double u1 = aKnots[i - 1], u2 = aKnots[i]; // aKnots(i), aKnots(i + 1)
            double d = CompLocalDev(theCurve, u1, u2);
            if (d > theDmax)
            {
                theDmax = d;
                theImax = i;
                theUfMax = u1;
                theUlMax = u2;
            }
        }
    }

    private static double CompLocalDev(CurveAdaptorBase theCurve, double u1, double u2)
    {
        var aLowBorder = new math_Vector(1, 1);
        var aUppBorder = new math_Vector(1, 1);
        var aSteps = new math_Vector(1, 1);
        //
        aLowBorder[1] = u1;
        aUppBorder[1] = u2;
        aSteps[1] = (aUppBorder[1] - aLowBorder[1]) * 0.01; // Run PSO on even distribution with 100 points.
        //
        var aFunc1 = new DistFunction(theCurve, u1, u2);
        //
        var aT = new math_Vector(1, 1);
        var aFunc = new DistFunctionMV(aFunc1);

        var aFinder = new Pso(aFunc, aLowBorder, aUppBorder, aSteps); // Choose 32 best points from 100 above.
        aFinder.Perform(aSteps, out double aValue, aT);
        double d;

        double x1 = Math.Max(u1, aT[1] - aSteps[1]);
        bool Ok = aFunc1.Value(x1, out double d1);
        if (!Ok)
        {
            return Math.Sqrt(-aValue);
        }
        double x2 = Math.Min(u2, aT[1] + aSteps[1]);
        Ok = aFunc1.Value(x2, out double d2);
        if (!Ok)
        {
            return Math.Sqrt(-aValue);
        }
        if (d1 <= aValue || d2 <= aValue)
        {
            double dmin = Math.Min(d1, Math.Min(aValue, d2));
            return Math.Sqrt(-dmin);
        }

        var anOptLoc = new BrentMinimum(global::Kerf.Engine.Foundation.Precision.PConfusion());
        anOptLoc.Perform(aFunc1, x1, aT[1], x2);

        if (anOptLoc.IsDone)
        {
            d = -anOptLoc.Minimum();
        }
        else
        {
            d = -aValue;
        }
        return Math.Sqrt(d);
    }
}
