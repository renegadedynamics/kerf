// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// ShapeAnalysis_Curve and ShapeAnalysis_Surface against native OCCT.
/// </summary>
/// <remarks>
/// The geometry is built natively from random raw values (every kind of curve and surface) or taken from random native
/// shapes (BRepToolsIOOracleTests' primitives, sweeps, Booleans, fillets and lofts). It is written by native OCCT to a
/// BRep file inside an edge or a face, and both sides read that file, so they analyse the same geometry bit for bit.
/// The points to project are made from the port's evaluation of the geometry, moved off it at random, and handed to
/// both sides as the same raw coordinates. Both sides run the same sequence of calls on their analyzers (which cache
/// singularities, closure values and extrema).
/// </remarks>
public class ShapeAnalysisOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-shapeanalysis-").FullName;
    private int myFileNumber;

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    /// <summary>The native shape written to a file and read back by both sides.</summary>
    private (N.TopoDS_Shape N, TopoDS_Shape P) RoundTrip(N.TopoDS_Shape theShape)
    {
        string aFile = Path.Combine(myDirectory, $"s{myFileNumber++}.brep");
        Assert.True(N.BRepTools.Write(theShape, aFile));
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }

        Assert.True(BRepTools.Read(out TopoDS_Shape aPShape, aFile));
        return (aNShape, aPShape);
    }

    private static N.gp_Pnt NPnt(Oracle theOracle, double theMin = -10, double theMax = 10)
        => new(theOracle.Real(theMin, theMax), theOracle.Real(theMin, theMax), theOracle.Real(theMin, theMax));

    private static gp_Pnt FromN(N.gp_Pnt theP) => new(theP.X(), theP.Y(), theP.Z());

    private static gp_Pnt2d FromN(N.gp_Pnt2d theP) => new(theP.X(), theP.Y());

    private static N.TColgp_Array1OfPnt NPoles(Oracle theOracle, int theNb)
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(1, theNb);
        for (int i = 1; i <= theNb; i++)
        {
            aPoles[i] = new gp_Pnt(theOracle.Real(-5, 5), theOracle.Real(-5, 5), theOracle.Real(-5, 5));
        }

        return Oracle.ToNative(aPoles);
    }

    private static N.Geom_BSplineCurve NBSplineCurve(Oracle theOracle, bool theSmooth)
    {
        int aDegree = theOracle.Int(theSmooth ? 2 : 1, 4);
        theOracle.BSplineKnots(aDegree, false, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults);
        int aNb = 0;
        for (int i = 1; i <= aMults.Length(); i++)
        {
            if (theSmooth && i > 1 && i < aMults.Length())
            {
                aMults[i] = Math.Min(aMults[i], aDegree - 1);
            }

            aNb += aMults[i];
        }

        aNb -= aDegree + 1;
        var aPoles = new NCollection_Array1<gp_Pnt>(1, aNb);
        for (int i = 1; i <= aNb; i++)
        {
            aPoles[i] = new gp_Pnt(theOracle.Real(-5, 5), theOracle.Real(-5, 5), theOracle.Real(-5, 5));
        }

        NCollection_Array1<double>? aWeights = theOracle.Bool() ? theOracle.Weights(aNb, true) : null;
        return Oracle.ToNativeCurve(aPoles, aWeights, aKnots, aMults, aDegree, false);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Curves
    // ---------------------------------------------------------------------------------------------------------------

    /// <summary>A random native curve of one of the kinds, with a parameter range inside its bounds.</summary>
    private static (N.Geom_Curve Curve, double First, double Last, string What) NativeCurve(Oracle theOracle, int theIndex)
    {
        (N.gp_Ax2 anAx2, _) = GeomOracleInputs.Ax2s(theOracle);
        using (anAx2)
        {
            switch (theIndex % 9)
            {
                case 0:
                {
                    using N.gp_Pnt aP = NPnt(theOracle);
                    (N.gp_Dir aD, _) = GeomOracleInputs.Dirs(theOracle);
                    using (aD)
                    {
                        double f = theOracle.Real(-10, 0);
                        return (new N.Geom_Line(aP, aD), f, f + theOracle.Real(0.5, 10), "line");
                    }
                }
                case 1:
                {
                    double f = theOracle.Real(-1, 6);
                    return (new N.Geom_Circle(anAx2, theOracle.Real(0.5, 5)), f, f + theOracle.Real(0.5, 2 * Math.PI), "circle");
                }
                case 2:
                {
                    double aMajor = theOracle.Real(1, 5);
                    double f      = theOracle.Real(-1, 6);
                    return (new N.Geom_Ellipse(anAx2, aMajor, aMajor * theOracle.Real(0.2, 1)), f,
                            f + theOracle.Real(0.5, 2 * Math.PI), "ellipse");
                }
                case 3:
                {
                    double f = theOracle.Real(-2, 0);
                    return (new N.Geom_Hyperbola(anAx2, theOracle.Real(1, 3), theOracle.Real(0.5, 3)), f, f + theOracle.Real(0.5, 3),
                            "hyperbola");
                }
                case 4:
                {
                    double f = theOracle.Real(-3, 0);
                    return (new N.Geom_Parabola(anAx2, theOracle.Real(0.2, 3)), f, f + theOracle.Real(0.5, 5), "parabola");
                }
                case 5:
                case 6:
                {
                    N.Geom_BSplineCurve aBS = NBSplineCurve(theOracle, false);
                    return (aBS, aBS.FirstParameter(), aBS.LastParameter(), "bspline");
                }
                case 7:
                {
                    using N.TColgp_Array1OfPnt aPoles = NPoles(theOracle, theOracle.Int(2, 6));
                    return (new N.Geom_BezierCurve(aPoles), 0, 1, "bezier");
                }
                default:
                {
                    // an offset of a planar smooth B-spline: its poles in the XY plane
                    int aDegree = 3;
                    theOracle.BSplineKnots(aDegree, false, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults);
                    int aNb = 0;
                    for (int i = 1; i <= aMults.Length(); i++)
                    {
                        if (i > 1 && i < aMults.Length())
                        {
                            aMults[i] = 1;
                        }

                        aNb += aMults[i];
                    }

                    aNb -= aDegree + 1;
                    var aPoles = new NCollection_Array1<gp_Pnt>(1, aNb);
                    for (int i = 1; i <= aNb; i++)
                    {
                        aPoles[i] = new gp_Pnt(theOracle.Real(-5, 5), theOracle.Real(-5, 5), 0);
                    }

                    using N.Geom_BSplineCurve aBasis = Oracle.ToNativeCurve(aPoles, null, aKnots, aMults, aDegree, false);
                    using var aZ = new N.gp_Dir(0, 0, 1);
                    return (new N.Geom_OffsetCurve(aBasis, theOracle.Real(-0.3, 0.3), aZ), aBasis.FirstParameter(),
                            aBasis.LastParameter(), "offset");
                }
            }
        }
    }

    /// <summary>The curve written in an edge and read back by both sides.</summary>
    private (N.Geom_Curve N, Geom_Curve P) RoundTripCurve(N.Geom_Curve theCurve, double theFirst, double theLast)
    {
        using var aMaker = new N.BRepBuilderAPI_MakeEdge(theCurve, theFirst, theLast);
        Assert.True(aMaker.IsDone());
        (N.TopoDS_Shape aN, TopoDS_Shape aP) = RoundTrip(aMaker.Shape());
        double f = 0, l = 0;
        N.Geom_Curve? aNC = N.BRep_Tool.Curve(N.TopoDS.Edge(aN), ref f, ref l);
        Geom_Curve? aPC = BRep_Tool.Curve(TopoDS.Edge(aP), out _, out _);
        Assert.NotNull(aNC);
        Assert.NotNull(aPC);
        return (aNC!, aPC!);
    }

    [Fact]
    public void ShapeAnalysis_Curve_projects_and_validates_as_native_OCCT()
    {
        var anOracle = new Oracle(output, 42011);
        var aNSAC = new N.ShapeAnalysis_Curve();
        var aSAC = new ShapeAnalysis_Curve();
        double[] aPrecis = [1e-7, 1e-4, 1e-2];
        for (int i = 0; i < 270; i++)
        {
            (N.Geom_Curve aNative, double f, double l, string aWhat) = NativeCurve(anOracle, i);
            (N.Geom_Curve aNC, Geom_Curve aPC) = RoundTripCurve(aNative, f, l);
            aNative.Dispose();
            string aCase = $"curve {i} ({aWhat})";

            // half of the bounded kinds again as trimmed curves, made on both sides from the curve read
            if (aWhat is "circle" or "ellipse" or "bspline" && anOracle.Bool())
            {
                double t1 = f + (l - f) * anOracle.Real(0, 0.3), t2 = f + (l - f) * anOracle.Real(0.7, 1);
                aNC = new N.Geom_TrimmedCurve(aNC, t1, t2);
                aPC = new Geom_TrimmedCurve(aPC, t1, t2);
                f = aPC.FirstParameter();
                l = aPC.LastParameter();
                aCase += ", trimmed";
            }

            using var aNAdaptor = new N.GeomAdaptor_Curve(aNC, f, l);
            var aPAdaptor = new GeomAdaptor_Curve(aPC, f, l);
            for (int k = 0; k < 8; k++)
            {
                double t = f + (l - f) * anOracle.Real(-0.1, 1.1);
                gp_Pnt aP = aPC.Value(t);
                double anOff = k % 4 == 0 ? 0 : anOracle.Real(0, k % 4 == 1 ? 1e-5 : 3);
                aP = aP.Translated(new gp_Vec(anOracle.Real(-1, 1), anOracle.Real(-1, 1), anOracle.Real(-1, 1)) * anOff);
                if (k == 7 && aWhat == "circle")
                {
                    aP = ((Geom_Circle)(aPC is Geom_TrimmedCurve aT ? aT.BasisCurve() : aPC)).Location(); // on the axis
                }

                using N.gp_Pnt aNP = Oracle.ToNative(aP);
                double aPreci = aPrecis[k % 3];
                bool anAdjust = anOracle.Bool();
                string aStep = $"{aCase}, point {k}";

                using var aNProj = new N.gp_Pnt();
                double aNParam = 0;
                double aND = aNSAC.Project(aNC, aNP, aPreci, aNProj, ref aNParam, anAdjust);
                double aPD = aSAC.Project(aPC, aP, aPreci, out gp_Pnt aPProj, out double aPParam, anAdjust);
                anOracle.Same(aND, aPD, aStep + ", Project");
                anOracle.Same(aNParam, aPParam, aStep + ", Project param");
                anOracle.Same(aNProj, aPProj, aStep + ", Project point");

                double cf = f + (l - f) * anOracle.Real(0, 0.4), cl = f + (l - f) * anOracle.Real(0.6, 1);
                aND = aNSAC.Project(aNC, aNP, aPreci, aNProj, ref aNParam, cf, cl, anAdjust);
                aPD = aSAC.Project(aPC, aP, aPreci, out aPProj, out aPParam, cf, cl, anAdjust);
                anOracle.Same(aND, aPD, aStep + ", Project on [cf, cl]");
                anOracle.Same(aNParam, aPParam, aStep + ", Project on [cf, cl] param");
                anOracle.Same(aNProj, aPProj, aStep + ", Project on [cf, cl] point");

                aND = aNSAC.Project(aNAdaptor, aNP, aPreci, aNProj, ref aNParam, anAdjust);
                aPD = aSAC.Project(aPAdaptor, aP, aPreci, out aPProj, out aPParam, anAdjust);
                anOracle.Same(aND, aPD, aStep + ", Project(adaptor)");
                anOracle.Same(aNParam, aPParam, aStep + ", Project(adaptor) param");
                anOracle.Same(aNProj, aPProj, aStep + ", Project(adaptor) point");

                aND = aNSAC.ProjectAct(aNAdaptor, aNP, aPreci, aNProj, ref aNParam);
                aPD = aSAC.ProjectAct(aPAdaptor, aP, aPreci, out aPProj, out aPParam);
                anOracle.Same(aND, aPD, aStep + ", ProjectAct");
                anOracle.Same(aNParam, aPParam, aStep + ", ProjectAct param");
                anOracle.Same(aNProj, aPProj, aStep + ", ProjectAct point");

                double aPrev = t + (l - f) * anOracle.Real(-0.05, 0.05);
                aND = aNSAC.NextProject(aPrev, aNC, aNP, aPreci, aNProj, ref aNParam, cf, cl, anAdjust);
                aPD = aSAC.NextProject(aPrev, aPC, aP, aPreci, out aPProj, out aPParam, cf, cl, anAdjust);
                anOracle.Same(aND, aPD, aStep + ", NextProject");
                anOracle.Same(aNParam, aPParam, aStep + ", NextProject param");
                anOracle.Same(aNProj, aPProj, aStep + ", NextProject point");

                aND = aNSAC.NextProject(aPrev, aNAdaptor, aNP, aPreci, aNProj, ref aNParam);
                aPD = aSAC.NextProject(aPrev, aPAdaptor, aP, aPreci, out aPProj, out aPParam);
                anOracle.Same(aND, aPD, aStep + ", NextProject(adaptor)");
                anOracle.Same(aNParam, aPParam, aStep + ", NextProject(adaptor) param");
                anOracle.Same(aNProj, aPProj, aStep + ", NextProject(adaptor) point");
            }

            // ValidateRange on copies (it may reverse the curve)
            for (int k = 0; k < 4; k++)
            {
                using N.Geom_Curve aNCopy = N.Geom_Curve.DownCast(aNC.Copy())!;
                var aPCopy = (Geom_Curve)aPC.Copy();
                double aFirst = f + (l - f) * anOracle.Real(-0.3, 1.3), aLast = f + (l - f) * anOracle.Real(-0.3, 1.3);
                if (k == 3)
                {
                    aLast = aFirst;
                }

                double aNF = aFirst, aNL = aLast, aPF = aFirst, aPL = aLast;
                double aPreci = aPrecis[k % 3];
                anOracle.Same(aNSAC.ValidateRange(aNCopy, ref aNF, ref aNL, aPreci), aSAC.ValidateRange(aPCopy, ref aPF, ref aPL, aPreci),
                              $"{aCase}, ValidateRange {k}");
                anOracle.Same(aNF, aPF, $"{aCase}, ValidateRange {k} first");
                anOracle.Same(aNL, aPL, $"{aCase}, ValidateRange {k} last");
                using N.gp_Pnt aNV = aNCopy.Value(0.5 * (f + l));
                anOracle.Same(aNV, aPCopy.Value(0.5 * (f + l)), $"{aCase}, ValidateRange {k} curve");
            }

            // planarity, closure, periodicity
            foreach (bool aGiven in new[] { false, true })
            {
                double aPreci = anOracle.Bool() ? 0 : 1e-3;
                gp_XYZ aNormal = aGiven ? new gp_XYZ(0, 0, 1) : new gp_XYZ();
                using var aNNormal = new N.gp_XYZ(aNormal.X(), aNormal.Y(), aNormal.Z());
                anOracle.Same(N.ShapeAnalysis_Curve.IsPlanar(aNC, aNNormal, aPreci), ShapeAnalysis_Curve.IsPlanar(aPC, ref aNormal, aPreci),
                              $"{aCase}, IsPlanar");
                anOracle.Same(new gp_XYZ(aNNormal.X(), aNNormal.Y(), aNNormal.Z()), aNormal, $"{aCase}, IsPlanar normal");
            }

            double aClosedPreci = anOracle.Bool() ? -1 : anOracle.Real(1e-7, 10);
            anOracle.Same(N.ShapeAnalysis_Curve.IsClosed(aNC, aClosedPreci), ShapeAnalysis_Curve.IsClosed(aPC, aClosedPreci),
                          $"{aCase}, IsClosed");
            anOracle.Same(N.ShapeAnalysis_Curve.IsPeriodic(aNC), ShapeAnalysis_Curve.IsPeriodic(aPC), $"{aCase}, IsPeriodic");
            aNC.Dispose();
        }

        anOracle.Report("ShapeAnalysis_Curve: Project, ProjectAct, NextProject, ValidateRange, IsPlanar, IsClosed, IsPeriodic");
    }

    [Fact]
    public void ShapeAnalysis_Curve_SelectForwardSeam_as_native_OCCT()
    {
        var anOracle = new Oracle(output, 42012);
        var aNSAC = new N.ShapeAnalysis_Curve();
        var aSAC = new ShapeAnalysis_Curve();
        for (int i = 0; i < 500; i++)
        {
            (N.Geom2d_Curve N, Geom2d_Curve P) Make()
            {
                double x = anOracle.Real(-5, 5), y = anOracle.Real(-5, 5);
                N.gp_Dir2d aND;
                gp_Dir2d aPD;
                switch (anOracle.Int(0, 2))
                {
                    case 0:
                    {
                        double s = anOracle.Bool() ? 1 : -1;
                        aND = new N.gp_Dir2d(0, s);
                        aPD = new gp_Dir2d(0, s);
                        break;
                    }
                    case 1:
                    {
                        double s = anOracle.Bool() ? 1 : -1;
                        aND = new N.gp_Dir2d(s, 0);
                        aPD = new gp_Dir2d(s, 0);
                        break;
                    }
                    default:
                        (aND, aPD) = GeomOracleInputs.Dir2ds(anOracle);
                        break;
                }

                using N.gp_Pnt2d aNP = new(x, y);
                var aNL = new N.Geom2d_Line(aNP, aND);
                var aPL = new Geom2d_Line(new gp_Pnt2d(x, y), aPD);
                aND.Dispose();
                if (anOracle.Bool())
                {
                    double t1 = anOracle.Real(-3, 3), t2 = t1 + (anOracle.Bool() ? 1 : -1) * anOracle.Real(0.5, 3);
                    return (new N.Geom2d_TrimmedCurve(aNL, t1, t2), new Geom2d_TrimmedCurve(aPL, t1, t2));
                }

                return (aNL, aPL);
            }

            (N.Geom2d_Curve aN1, Geom2d_Curve aP1) = Make();
            (N.Geom2d_Curve aN2, Geom2d_Curve aP2) = Make();
            anOracle.Same(aNSAC.SelectForwardSeam(aN1, aN2), aSAC.SelectForwardSeam(aP1, aP2), $"pair {i}");
            aN1.Dispose();
            aN2.Dispose();
        }

        anOracle.Report("ShapeAnalysis_Curve.SelectForwardSeam");
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Surfaces
    // ---------------------------------------------------------------------------------------------------------------

    /// <summary>A random native surface of one of the kinds, with a parameter domain for its face.</summary>
    private static (N.Geom_Surface Surface, double U1, double U2, double V1, double V2, string What) NativeSurface(Oracle theOracle,
                                                                                                                    int theIndex)
    {
        (N.gp_Ax3 anAx3, _) = GeomOracleInputs.Ax3s(theOracle);
        using (anAx3)
        {
            double u1 = theOracle.Real(-1, 1), v1 = theOracle.Real(-3, 0);
            switch (theIndex % 12)
            {
                case 0:
                    return (new N.Geom_Plane(anAx3), u1, u1 + theOracle.Real(1, 5), v1, v1 + theOracle.Real(1, 5), "plane");
                case 1:
                    return (new N.Geom_CylindricalSurface(anAx3, theOracle.Real(0.5, 5)), 0, theOracle.Bool() ? 2 * Math.PI : theOracle.Real(1, 6),
                            v1, v1 + theOracle.Real(1, 5), "cylinder");
                case 2:
                {
                    double anAng = theOracle.Real(0.2, 1.2) * (theOracle.Bool() ? 1 : -1);
                    double aR    = theOracle.Bool() ? 0 : theOracle.Real(0.5, 3);
                    // V from the apex (or near it) upwards
                    double aVApex = -aR / Math.Sin(anAng);
                    double aV1    = theOracle.Bool() ? aVApex : aVApex + theOracle.Real(0.1, 1) * Math.Sign(-aVApex == 0 ? 1 : -aVApex);
                    return (new N.Geom_ConicalSurface(anAx3, anAng, aR), 0, 2 * Math.PI, Math.Min(aV1, aV1 + 3), Math.Max(aV1, aV1 + 3),
                            "cone");
                }
                case 3:
                    return (new N.Geom_SphericalSurface(anAx3, theOracle.Real(0.5, 5)), 0, 2 * Math.PI,
                            theOracle.Bool() ? -Math.PI / 2 : theOracle.Real(-1.5, 0), theOracle.Bool() ? Math.PI / 2 : theOracle.Real(0.1, 1.5),
                            "sphere");
                case 4:
                {
                    double aMajor = theOracle.Real(2, 5);
                    return (new N.Geom_ToroidalSurface(anAx3, aMajor, theOracle.Real(0.5, 1.9)), 0, 2 * Math.PI, 0, 2 * Math.PI, "torus");
                }
                case 5:
                {
                    // a profile in the XZ plane of the frame, revolved about Z: a smooth B-spline or a segment through the axis
                    N.Geom_Curve aProfile;
                    if (theOracle.Bool())
                    {
                        aProfile = NBSplineCurve(theOracle, true);
                    }
                    else
                    {
                        using var aP0 = new N.gp_Pnt(0, 0, theOracle.Real(-2, 2));
                        using var aD = new N.gp_Dir(theOracle.Real(0.2, 1), 0, theOracle.Real(-1, 1));
                        aProfile = new N.Geom_TrimmedCurve(new N.Geom_Line(aP0, aD), 0, theOracle.Real(1, 4));
                    }

                    using var aO = new N.gp_Pnt(0, 0, 0);
                    using var aZ = new N.gp_Dir(0, 0, 1);
                    using var anAxis = new N.gp_Ax1(aO, aZ);
                    var aRev = new N.Geom_SurfaceOfRevolution(aProfile, anAxis);
                    double aPF = aProfile.FirstParameter(), aPL = aProfile.LastParameter();
                    aProfile.Dispose();
                    return (aRev, 0, 2 * Math.PI, aPF, aPL, "revolution");
                }
                case 6:
                {
                    N.Geom_Curve aBasis;
                    double f, l;
                    if (theOracle.Bool())
                    {
                        (N.gp_Ax2 anAx2, _) = GeomOracleInputs.Ax2s(theOracle);
                        aBasis = new N.Geom_Circle(anAx2, theOracle.Real(0.5, 3));
                        anAx2.Dispose();
                        f = 0;
                        l = theOracle.Bool() ? 2 * Math.PI : theOracle.Real(1, 6);
                    }
                    else
                    {
                        var aBS = NBSplineCurve(theOracle, false);
                        f      = aBS.FirstParameter();
                        l      = aBS.LastParameter();
                        aBasis = aBS;
                    }

                    (N.gp_Dir aD, _) = GeomOracleInputs.Dirs(theOracle);
                    var anExtr = new N.Geom_SurfaceOfLinearExtrusion(aBasis, aD);
                    aD.Dispose();
                    aBasis.Dispose();
                    return (anExtr, f, l, v1, v1 + theOracle.Real(1, 5), "extrusion");
                }
                case 7:
                {
                    int aNU = theOracle.Int(2, 5), aNV = theOracle.Int(2, 5);
                    var aPoles = new NCollection_Array2<gp_Pnt>(1, aNU, 1, aNV);
                    for (int i = 1; i <= aNU; i++)
                    {
                        for (int j = 1; j <= aNV; j++)
                        {
                            aPoles[i, j] = new gp_Pnt(i + theOracle.Real(-0.4, 0.4), j + theOracle.Real(-0.4, 0.4), theOracle.Real(-1, 1));
                        }
                    }

                    using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(aPoles);
                    return (new N.Geom_BezierSurface(aNPoles), 0, 1, 0, 1, "bezier");
                }
                case 8:
                case 9:
                {
                    // a B-spline surface; one in two with its U-first row collapsed to a point (a degenerated edge)
                    bool aDeg = theIndex % 12 == 9;
                    int aUDeg = theOracle.Int(1, 3), aVDeg = theOracle.Int(1, 3);
                    theOracle.BSplineKnots(aUDeg, false, out NCollection_Array1<double> aUK, out NCollection_Array1<int> aUM);
                    theOracle.BSplineKnots(aVDeg, false, out NCollection_Array1<double> aVK, out NCollection_Array1<int> aVM);
                    int aNU = -aUDeg - 1, aNV = -aVDeg - 1;
                    for (int i = 1; i <= aUM.Length(); i++)
                    {
                        aNU += aUM[i];
                    }

                    for (int i = 1; i <= aVM.Length(); i++)
                    {
                        aNV += aVM[i];
                    }

                    var aPoles = new NCollection_Array2<gp_Pnt>(1, aNU, 1, aNV);
                    var anApex = new gp_Pnt(0, aNV / 2.0, theOracle.Real(1, 3));
                    for (int i = 1; i <= aNU; i++)
                    {
                        for (int j = 1; j <= aNV; j++)
                        {
                            aPoles[i, j] = aDeg && i == 1
                                ? anApex
                                : new gp_Pnt(i + theOracle.Real(-0.4, 0.4), j + theOracle.Real(-0.4, 0.4), theOracle.Real(-1, 1));
                        }
                    }

                    using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(aPoles);
                    using N.TColStd_Array1OfReal aNUK = Oracle.ToNative(aUK), aNVK = Oracle.ToNative(aVK);
                    using N.TColStd_Array1OfInteger aNUM = Oracle.ToNative(aUM), aNVM = Oracle.ToNative(aVM);
                    var aBS = new N.Geom_BSplineSurface(aNPoles, aNUK, aNVK, aNUM, aNVM, aUDeg, aVDeg);
                    return (aBS, aUK[1], aUK[aUK.Length()], aVK[1], aVK[aVK.Length()], aDeg ? "degenerated bspline" : "bspline");
                }
                case 10:
                {
                    // an offset of a smooth Bezier surface or of a cylinder
                    if (theOracle.Bool())
                    {
                        var aPoles = new NCollection_Array2<gp_Pnt>(1, 3, 1, 3);
                        for (int i = 1; i <= 3; i++)
                        {
                            for (int j = 1; j <= 3; j++)
                            {
                                aPoles[i, j] = new gp_Pnt(i * 2, j * 2, theOracle.Real(-0.5, 0.5));
                            }
                        }

                        using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(aPoles);
                        using var aBasis = new N.Geom_BezierSurface(aNPoles);
                        return (new N.Geom_OffsetSurface(aBasis, theOracle.Real(-0.5, 0.5)), 0, 1, 0, 1, "offset of bezier");
                    }

                    using var aCyl = new N.Geom_CylindricalSurface(anAx3, theOracle.Real(1, 3));
                    return (new N.Geom_OffsetSurface(aCyl, theOracle.Real(-0.5, 0.5)), 0, theOracle.Real(1, 2 * Math.PI), v1, v1 + 2,
                            "offset of cylinder");
                }
                default:
                {
                    // rectangular trimmed cylinders and spheres
                    if (theOracle.Bool())
                    {
                        using var aCyl = new N.Geom_CylindricalSurface(anAx3, theOracle.Real(0.5, 3));
                        double aU2 = theOracle.Bool() ? 2 * Math.PI : theOracle.Real(1, 5);
                        double aV2 = v1 + theOracle.Real(0.5, 4);
                        return (new N.Geom_RectangularTrimmedSurface(aCyl, 0, aU2, v1, aV2), 0, aU2, v1, aV2, "trimmed cylinder");
                    }

                    using var aSph = new N.Geom_SphericalSurface(anAx3, theOracle.Real(0.5, 3));
                    double aVa = theOracle.Real(-1.5, 0), aVb = theOracle.Real(0.1, 1.5);
                    return (new N.Geom_RectangularTrimmedSurface(aSph, 0, Math.PI, aVa, aVb), 0, Math.PI, aVa, aVb, "trimmed sphere");
                }
            }
        }
    }

    /// <summary>The surface written in a face and read back by both sides.</summary>
    private (N.Geom_Surface N, Geom_Surface P) RoundTripSurface(N.Geom_Surface theSurface, double theU1, double theU2, double theV1,
                                                                double theV2)
    {
        using var aMaker = new N.BRepBuilderAPI_MakeFace(theSurface, theU1, theU2, theV1, theV2, 1e-7);
        Assert.True(aMaker.IsDone());
        (N.TopoDS_Shape aN, TopoDS_Shape aP) = RoundTrip(aMaker.Shape());
        return (N.BRep_Tool.Surface(N.TopoDS.Face(aN))!, BRep_Tool.Surface(TopoDS.Face(aP))!);
    }

    private sealed class Counts
    {
        public int Surfaces;
        public int Projections;
        public int Singular;
    }

    /// <summary>The same calls on a native and a port analyzer of the same surface, compared.</summary>
    private static void CompareSurface(Oracle theOracle, N.Geom_Surface theN, Geom_Surface theP, double theU1, double theU2,
                                       double theV1, double theV2, string theCase, Counts theCounts)
    {
        theCounts.Surfaces++;
        using var aNSAS = new N.ShapeAnalysis_Surface(theN);
        var aSAS = new ShapeAnalysis_Surface(theP);
        double[] aPrecis = [1e-7, 1e-4, 1e-2];

        // singularities
        foreach (double aPreci in new[] { 0.0, 1e-7, 1e-3, 1e10 })
        {
            theOracle.Same(aNSAS.NbSingularities(aPreci), aSAS.NbSingularities(aPreci), $"{theCase}, NbSingularities({aPreci})");
        }

        var aSingular = new List<gp_Pnt>();
        for (int num = 1; num <= 5; num++)
        {
            double aNPreci = 0, aNF = 0, aNL = 0, aPPreci = 0, aPF = 0, aPL = 0;
            bool aNU = false, aPU = false;
            using var aNP = new N.gp_Pnt();
            using var aNF2 = new N.gp_Pnt2d();
            using var aNL2 = new N.gp_Pnt2d();
            gp_Pnt aPP = new gp_Pnt();
            gp_Pnt2d aPF2 = new gp_Pnt2d(), aPL2 = new gp_Pnt2d();
            bool aNOk = aNSAS.Singularity(num, ref aNPreci, aNP, aNF2, aNL2, ref aNF, ref aNL, ref aNU);
            bool aPOk = aSAS.Singularity(num, ref aPPreci, ref aPP, ref aPF2, ref aPL2, ref aPF, ref aPL, ref aPU);
            string aWhat = $"{theCase}, Singularity({num})";
            theOracle.Same(aNOk, aPOk, aWhat);
            if (aNOk)
            {
                theOracle.Same(aNPreci, aPPreci, aWhat + " preci");
                theOracle.Same(aNP, aPP, aWhat + " point");
                theOracle.Same(aNF2, aPF2, aWhat + " first");
                theOracle.Same(aNL2, aPL2, aWhat + " last");
                theOracle.Same(aNF, aPF, aWhat + " first param");
                theOracle.Same(aNL, aPL, aWhat + " last param");
                theOracle.Same(aNU, aPU, aWhat + " U iso");
                if (aPPreci < 1e-3)
                {
                    aSingular.Add(aPP);
                }
            }
        }

        // closure
        foreach (bool aU in new[] { true, false })
        {
            double aPreci = theOracle.Int(0, 2) switch { 0 => -1, 1 => 1e-7, _ => 1e-3 };
            bool aNC = aU ? aNSAS.IsUClosed(aPreci) : aNSAS.IsVClosed(aPreci);
            bool aPC = aU ? aSAS.IsUClosed(aPreci) : aSAS.IsVClosed(aPreci);
            theOracle.Same(aNC, aPC, $"{theCase}, Is{(aU ? 'U' : 'V')}Closed");
            theOracle.Same(aNSAS.Gap(), aSAS.Gap(), $"{theCase}, Is{(aU ? 'U' : 'V')}Closed gap");
        }

        theOracle.Same(aNSAS.UCloseVal(), aSAS.UCloseVal(), $"{theCase}, UCloseVal");
        theOracle.Same(aNSAS.VCloseVal(), aSAS.VCloseVal(), $"{theCase}, VCloseVal");

        // projections
        for (int k = 0; k < 8; k++)
        {
            double u = theU1 + (theU2 - theU1) * theOracle.Real(-0.05, 1.05), v = theV1 + (theV2 - theV1) * theOracle.Real(-0.05, 1.05);
            gp_Pnt aP = theP.Value(u, v);
            double anOff = k % 4 == 0 ? 0 : theOracle.Real(0, k % 4 == 1 ? 1e-5 : 2);
            aP = aP.Translated(new gp_Vec(theOracle.Real(-1, 1), theOracle.Real(-1, 1), theOracle.Real(-1, 1)) * anOff);
            if (k == 7 && aSingular.Count > 0)
            {
                aP = aSingular[theOracle.Int(0, aSingular.Count - 1)];
            }

            using N.gp_Pnt aNP = Oracle.ToNative(aP);
            double aPreci = aPrecis[k % 3];
            string aStep = $"{theCase}, point {k}";
            theCounts.Projections++;

            using (N.gp_Pnt2d aNUV = aNSAS.ValueOfUV(aNP, aPreci))
            {
                theOracle.Same(aNUV, aSAS.ValueOfUV(aP, aPreci), aStep + ", ValueOfUV");
            }

            theOracle.Same(aNSAS.Gap(), aSAS.Gap(), aStep + ", ValueOfUV gap");

            var aPrev = new gp_Pnt2d(u + (theU2 - theU1) * theOracle.Real(-0.03, 0.03), v + (theV2 - theV1) * theOracle.Real(-0.03, 0.03));
            double aMaxPreci = theOracle.Bool() ? -1 : 1e-3;
            using (N.gp_Pnt2d aNPrev = Oracle.ToNative(aPrev))
            using (N.gp_Pnt2d aNUV = aNSAS.NextValueOfUV(aNPrev, aNP, aPreci, aMaxPreci))
            {
                theOracle.Same(aNUV, aSAS.NextValueOfUV(aPrev, aP, aPreci, aMaxPreci), aStep + ", NextValueOfUV");
            }

            theOracle.Same(aNSAS.Gap(), aSAS.Gap(), aStep + ", NextValueOfUV gap");

            double aNU = aPrev.X(), aNV = aPrev.Y(), aPU = aPrev.X(), aPV = aPrev.Y();
            theOracle.Same(aNSAS.UVFromIso(aNP, aPreci, ref aNU, ref aNV), aSAS.UVFromIso(aP, aPreci, ref aPU, ref aPV),
                           aStep + ", UVFromIso");
            theOracle.Same(aNU, aPU, aStep + ", UVFromIso U");
            theOracle.Same(aNV, aPV, aStep + ", UVFromIso V");

            // degeneracy
            double aDegPreci = k % 2 == 0 ? 1e-7 : 1e-3;
            theOracle.Same(aNSAS.IsDegenerated(aNP, aDegPreci), aSAS.IsDegenerated(aP, aDegPreci), aStep + ", IsDegenerated");
            theOracle.Same(aNSAS.Gap(), aSAS.Gap(), aStep + ", IsDegenerated gap");

            using var aNF = new N.gp_Pnt2d();
            using var aNL = new N.gp_Pnt2d();
            gp_Pnt2d aPF = new gp_Pnt2d(), aPL = new gp_Pnt2d();
            double aNFP = 0, aNLP = 0, aPFP = 0, aPLP = 0;
            bool aNOk = aNSAS.DegeneratedValues(aNP, aDegPreci, aNF, aNL, ref aNFP, ref aNLP);
            bool aPOk = aSAS.DegeneratedValues(aP, aDegPreci, ref aPF, ref aPL, ref aPFP, ref aPLP);
            theOracle.Same(aNOk, aPOk, aStep + ", DegeneratedValues");
            if (aNOk)
            {
                theCounts.Singular++;
                theOracle.Same(aNF, aPF, aStep + ", DegeneratedValues first");
                theOracle.Same(aNL, aPL, aStep + ", DegeneratedValues last");
                theOracle.Same(aNFP, aPFP, aStep + ", DegeneratedValues first param");
                theOracle.Same(aNLP, aPLP, aStep + ", DegeneratedValues last param");
            }

            theOracle.Same(aNSAS.Gap(), aSAS.Gap(), aStep + ", DegeneratedValues gap");

            var aResult = new gp_Pnt2d(u, v);
            var aNeighbour = new gp_Pnt2d(u + 0.1, v + 0.1);
            using N.gp_Pnt2d aNResult = Oracle.ToNative(aResult);
            using N.gp_Pnt2d aNNeighbour = Oracle.ToNative(aNeighbour);
            theOracle.Same(aNSAS.ProjectDegenerated(aNP, aDegPreci, aNNeighbour, aNResult),
                           aSAS.ProjectDegenerated(aP, aDegPreci, aNeighbour, ref aResult), aStep + ", ProjectDegenerated");
            theOracle.Same(aNResult, aResult, aStep + ", ProjectDegenerated result");

            // a straight pcurve: along a boundary or across
            var aA = new gp_Pnt2d(k % 2 == 0 ? theU1 : u, theV1 + (theV2 - theV1) * theOracle.Real(0, 0.5));
            var aB = new gp_Pnt2d(k % 2 == 0 ? theU1 : u + 0.3, theV1 + (theV2 - theV1) * theOracle.Real(0.5, 1));
            using N.gp_Pnt2d aNA = Oracle.ToNative(aA), aNB = Oracle.ToNative(aB);
            double aTol = theOracle.Bool() ? 1e-7 : 1e-2;
            theOracle.Same(aNSAS.IsDegenerated(aNA, aNB, aTol, 10), aSAS.IsDegenerated(aA, aB, aTol, 10), aStep + ", IsDegenerated(p2d)");
        }

        // the iso boxes
        (Func<N.Bnd_Box> N, Func<Bnd_Box> P, string What)[] aBoxes =
        [
            (aNSAS.GetBoxUF, aSAS.GetBoxUF, "UF"), (aNSAS.GetBoxUL, aSAS.GetBoxUL, "UL"),
            (aNSAS.GetBoxVF, aSAS.GetBoxVF, "VF"), (aNSAS.GetBoxVL, aSAS.GetBoxVL, "VL"),
        ];
        foreach ((Func<N.Bnd_Box> aNGet, Func<Bnd_Box> aPGet, string aWhat) in aBoxes)
        {
            using N.Bnd_Box aNBox = aNGet();
            Bnd_Box aPBox = aPGet();
            theOracle.Same(aNBox.IsVoid(), aPBox.IsVoid(), $"{theCase}, box {aWhat} void");
            if (!aNBox.IsVoid())
            {
                double x0 = 0, y0 = 0, z0 = 0, x1 = 0, y1 = 0, z1 = 0;
                aNBox.Get(ref x0, ref y0, ref z0, ref x1, ref y1, ref z1);
                aPBox.Get(out double px0, out double py0, out double pz0, out double px1, out double py1, out double pz1);
                theOracle.Same(x0, px0, $"{theCase}, box {aWhat} xmin");
                theOracle.Same(y0, py0, $"{theCase}, box {aWhat} ymin");
                theOracle.Same(z0, pz0, $"{theCase}, box {aWhat} zmin");
                theOracle.Same(x1, px1, $"{theCase}, box {aWhat} xmax");
                theOracle.Same(y1, py1, $"{theCase}, box {aWhat} ymax");
                theOracle.Same(z1, pz1, $"{theCase}, box {aWhat} zmax");
            }
        }
    }

    [Fact]
    public void ShapeAnalysis_Surface_on_every_kind_of_surface_as_native_OCCT()
    {
        var anOracle = new Oracle(output, 42013);
        var aCounts = new Counts();
        for (int i = 0; i < 240; i++)
        {
            (N.Geom_Surface aNative, double u1, double u2, double v1, double v2, string aWhat) = NativeSurface(anOracle, i);
            (N.Geom_Surface aNS, Geom_Surface aPS) = RoundTripSurface(aNative, u1, u2, v1, v2);
            aNative.Dispose();
            CompareSurface(anOracle, aNS, aPS, u1, u2, v1, v2, $"surface {i} ({aWhat})", aCounts);
            aNS.Dispose();
        }

        output.WriteLine($"{aCounts.Surfaces} surfaces, {aCounts.Projections} points, {aCounts.Singular} found in singularities");
        anOracle.Report("ShapeAnalysis_Surface on random surfaces: singularities, closure, ValueOfUV, NextValueOfUV, UVFromIso, degeneracy, iso boxes");
    }

    [Fact]
    public void ShapeAnalysis_Surface_on_native_shapes_as_native_OCCT()
    {
        var anOracle = new Oracle(output, 42014);
        var aCounts = new Counts();
        for (int i = 0; i < 48; i++)
        {
            using N.TopoDS_Shape aShape = BRepToolsIOOracleTests.NativeShape(anOracle, i, out string aWhat);
            (N.TopoDS_Shape aN, TopoDS_Shape aP) = RoundTrip(aShape);
            using var aNMap = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(aN, N.TopAbs_ShapeEnum.TopAbs_FACE, aNMap);
            var aPMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aP, TopAbs_FACE, aPMap);
            Assert.Equal(aNMap.Extent(), aPMap.Extent());
            for (int f = 1; f <= aPMap.Extent(); f++)
            {
                TopoDS_Face aPF = TopoDS.Face(aPMap[f]);
                BRepTools.UVBounds(aPF, out double u1, out double u2, out double v1, out double v2);
                using N.Geom_Surface aNS = N.BRep_Tool.Surface(N.TopoDS.Face(aNMap.FindKey(f)))!;
                Geom_Surface aPS = BRep_Tool.Surface(aPF)!;
                CompareSurface(anOracle, aNS, aPS, u1, u2, v1, v2, $"shape {i} ({aWhat}) face {f}", aCounts);
            }

            aN.Dispose();
        }

        output.WriteLine($"{aCounts.Surfaces} faces, {aCounts.Projections} points, {aCounts.Singular} found in singularities");
        anOracle.Report("ShapeAnalysis_Surface on the faces of native shapes");
    }
}
