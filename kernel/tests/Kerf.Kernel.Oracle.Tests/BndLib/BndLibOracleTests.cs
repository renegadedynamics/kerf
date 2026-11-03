// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.Oracle.Tests.AdaptorOracleInputs;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// BndLib_Add3dCurve, BndLib_AddSurface (and through them GeomBndLib) against native BRepBndLib::Add, AddOptimal
/// and AddOBB on edges and faces without triangulation, the port's side mirroring BRepBndLib's call path.
/// </summary>
/// <remarks>
/// <para>
/// The native shapes are made by BRepBuilderAPI_MakeEdge and MakeFace from random geometry built identically on both
/// sides. The port takes the geometry the shapes store as OCCT makes it: MakeEdge's curve without its trims, and
/// MakeFace's surface (trimmed for offsets of swept surfaces) and its UIso/VIso edge curves (BRepLib_MakeFace::Init);
/// each stored curve, read back through BRepTools::Write and GeomTools, is matched against them as a check (the
/// read-back geometry itself isn't used: GeomTools' gp_Dir constructors renormalise, an ulp off now and then). The
/// parameters are read natively: BRep_Tool::Range of each edge, BRepTools::UVBounds of each face, the tolerances.
/// </para>
/// <para>
/// Not compared: AddOptimal on faces that are neither natural restrictions nor "CanUseEdges" surfaces (it needs the
/// pcurves' 2D boxes, BRepTopAdaptor_FClass2d and Extrema_ExtSS), and the few cases where native OCCT reads
/// uninitialised memory (<see cref="SegmentsWithZeroTolerance(Curve3D, double, double)"/>).
/// </para>
/// <para>
/// BRepAdaptor_Curve and BRepAdaptor_Surface derive from GeomAdaptor_TransformedCurve/Surface (not from
/// GeomAdaptor_Curve/Surface), with the shape's location as transformation; the port uses those adaptors with the
/// identity, or with the location of a moved shape (AddOBB). In particular GeomBndLib_Curve treats an offset curve
/// behind a BRepAdaptor_Curve as an "other" curve (its dynamic_cast to GeomAdaptor_Curve fails), and so does the port.
/// </para>
/// </remarks>
public class BndLibOracleTests(ITestOutputHelper output) : IDisposable
{
    private static readonly int CasesPerKind = Tape.Cases(40, 6);

    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-bndlib-").FullName;
    private int myFileNumber;

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    // ---------------------------------------------------------------------------------------------------------
    // Geometry of native shapes, read back into the port

    private string NativeWrite(N.TopoDS_Shape theShape)
    {
        string aFile = Path.Combine(myDirectory, $"s{myFileNumber++}.brep");
        Assert.True(N.BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile).Replace("\r\n", "\n", StringComparison.Ordinal);
    }

    private static string Section(string theText, string theHeader, string theNext)
    {
        int aStart = theText.IndexOf("\n" + theHeader + " ", StringComparison.Ordinal) + 1;
        Assert.True(aStart > 0, $"no {theHeader} section");
        int anEnd = theText.IndexOf("\n" + theNext + " ", aStart, StringComparison.Ordinal);
        return theText.Substring(aStart, anEnd + 1 - aStart);
    }

    /// <summary>The 3D curve of a native edge, null if it has none.</summary>
    private Curve3D? EdgeCurve(N.TopoDS_Edge theEdge)
    {
        var aSet = new CurveSet();
        aSet.Read(new StringReader(Section(NativeWrite(theEdge), "Curves", "Polygon3D")));
        Assert.Null(aSet.Curve(2));
        return aSet.Curve(1);
    }

    /// <summary>Checks that the native face stores a surface of the port surface's type.</summary>
    private void CheckFaceSurface(N.TopoDS_Face theFace, Surface thePort, string theWhat)
        => Assert.True(FaceSurface(theFace).GetType() == thePort.GetType(), theWhat);

    /// <summary>The surface of a native face.</summary>
    private Surface FaceSurface(N.TopoDS_Face theFace)
    {
        var aSet = new SurfaceSet();
        aSet.Read(new StringReader(Section(NativeWrite(theFace), "Surfaces", "Triangulations")));
        Assert.Null(aSet.Surface(2));
        return aSet.Surface(1)!;
    }

    /// <summary>An edge as BRepAdaptor_Curve sees it: its curve, its range and its tolerance.</summary>
    private sealed record EdgeData(Curve3D? Curve, double First, double Last, double Tolerance, bool Degenerated, bool Geometric);

    /// <summary>
    /// An edge with the port's copy of its 3D curve: of theCandidates, the one the stored curve (read back) matches,
    /// same type and same point at mid-range. The read-back curve itself is only a check: GeomTools' gp_Dir
    /// constructors renormalise the stored directions, which moves some of them by an ulp.
    /// </summary>
    private EdgeData Edge(N.TopoDS_Edge theEdge, IReadOnlyList<Curve3D> theCandidates, string theWhat)
    {
        double aF = 0, aL = 0;
        N.BRep_Tool.Range(theEdge, ref aF, ref aL);
        Curve3D? aStored = EdgeCurve(theEdge);
        Curve3D? aCurve = null;
        if (aStored is not null)
        {
            Point3D aP = aStored.Value(0.5 * (aF + aL));
            foreach (Curve3D aC in theCandidates)
            {
                if (aC.GetType() == aStored.GetType()
                    && aC.Value(0.5 * (aF + aL)).Distance(aP) <= 1e-9 * Math.Max(1.0, aP.XYZ().Modulus()))
                {
                    aCurve = aC;
                    break;
                }
            }
            Assert.True(aCurve is not null, $"{theWhat}: no curve of the port matches the stored {Kerf.Engine.Foundation.OcctTypeNames.Of(aStored.GetType())}");
        }
        return new EdgeData(aCurve, aF, aL, N.BRep_Tool.Tolerance(theEdge), N.BRep_Tool.Degenerated(theEdge),
                            N.BRep_Tool.IsGeometric(theEdge));
    }

    private List<EdgeData> FaceEdges(N.TopoDS_Face theFace, IReadOnlyList<Curve3D> theIsos, string theWhat)
    {
        var aList = new List<EdgeData>();
        using var anExp = new N.TopExp_Explorer(theFace, N.TopAbs_ShapeEnum.TopAbs_EDGE);
        for (; anExp.More(); anExp.Next())
        {
            using N.TopoDS_Shape aS = anExp.Current();
            using N.TopoDS_Edge anE = N.TopoDS.Edge(aS);
            aList.Add(Edge(anE, theIsos, theWhat));
        }
        return aList;
    }

    private static (double U1, double U2, double V1, double V2) UVBounds(N.TopoDS_Face theFace)
    {
        double u1 = 0, u2 = 0, v1 = 0, v2 = 0;
        N.BRepTools.UVBounds(theFace, ref u1, ref u2, ref v1, ref v2);
        return (u1, u2, v1, v2);
    }

    private static List<(Point3D P, double Tol)> Vertices(N.TopoDS_Shape theShape)
    {
        var aList = new List<(Point3D, double)>();
        using var anExp = new N.TopExp_Explorer(theShape, N.TopAbs_ShapeEnum.TopAbs_VERTEX);
        for (; anExp.More(); anExp.Next())
        {
            using N.TopoDS_Shape aS = anExp.Current();
            using N.TopoDS_Vertex aV = N.TopoDS.Vertex(aS);
            using N.gp_Pnt aP = N.BRep_Tool.Pnt(aV);
            aList.Add((new Point3D(aP.X(), aP.Y(), aP.Z()), N.BRep_Tool.Tolerance(aV)));
        }
        return aList;
    }

    // ---------------------------------------------------------------------------------------------------------
    // Random shapes

    private static (double First, double Last) CurveRange(Oracle theO, Curve3D theC)
    {
        double aFirst = theC.FirstParameter(), aLast = theC.LastParameter();
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aLast))
        {
            double a = theO.Real(-5, 0), b = theO.Real(0.5, 5);
            return (a, a + b);
        }
        if (theO.Bool())
        {
            return (aFirst, aLast);
        }
        double u = theO.Real(aFirst, aLast), v = theO.Real(aFirst, aLast);
        if (Math.Abs(u - v) < 1e-3 * (aLast - aFirst))
        {
            return (aFirst, aFirst + 0.5 * (aLast - aFirst));
        }
        return u < v ? (u, v) : (v, u);
    }

    private static (double First, double Last) SurfaceRange(Oracle theO, double theFirst, double theLast)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(theLast))
        {
            double a = theO.Real(-5, 0), b = theO.Real(0.5, 5);
            return (a, a + b);
        }
        if (theO.Bool())
        {
            return (theFirst, theLast);
        }
        double u = theO.Real(theFirst, theLast), v = theO.Real(theFirst, theLast);
        if (Math.Abs(u - v) < 1e-2 * (theLast - theFirst))
        {
            return (theFirst, theFirst + 0.5 * (theLast - theFirst));
        }
        return u < v ? (u, v) : (v, u);
    }

    /// <summary>A native edge on a random curve of the given kind and range; null if MakeEdge fails.</summary>
    private static N.TopoDS_Edge? RandomEdge(Oracle theO, int theKind, out string theWhat, out Curve3D thePort)
    {
        (N.Geom_Curve aNC, Curve3D aPC, _) = RandomCurve(theO, theKind);
        // BRepLib_MakeEdge kills trimmed curves
        thePort = aPC;
        while (thePort is TrimmedCurve3D aT)
        {
            thePort = aT.BasisCurve();
        }
        using (aNC)
        {
            (double aF, double aL) = CurveRange(theO, aPC);
            theWhat = $"{CurveKinds[theKind]} [{aF:R}, {aL:R}]";
            using var aME = new N.BRepBuilderAPI_MakeEdge(aNC, aF, aL);
            return aME.IsDone() ? aME.Edge() : null;
        }
    }

    /// <summary>
    /// A native face on a random surface of the given kind, on a random finite UV range or (theWhole) on the whole
    /// surface, with the port's copies of the surface and of the iso curves MakeFace stores; null if MakeFace fails
    /// or the whole surface is unbounded.
    /// </summary>
    private static N.TopoDS_Face? RandomFace(Oracle theO, int theKind, bool theWhole, out string theWhat,
                                             out Surface thePort, out List<Curve3D> theIsos)
    {
        (N.Geom_Surface aNS, Surface aPS) = RandomSurface(theO, theKind);
        thePort = aPS;
        theIsos = [];
        using (aNS)
        {
            aPS.Bounds(out double u1, out double u2, out double v1, out double v2);
            double aU1 = u1, aU2 = u2, aV1 = v1, aV2 = v2;
            N.TopoDS_Face? aFace;
            if (theWhole)
            {
                theWhat = $"{SurfaceKinds[theKind]} whole";
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u2) || global::Kerf.Engine.Foundation.Precision.IsInfinite(v1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
                {
                    return null;
                }
                try
                {
                    using var aMF = new N.BRepBuilderAPI_MakeFace(aNS, 1e-7);
                    aFace = aMF.IsDone() ? aMF.Face() : null;
                }
                catch (Exception)
                {
                    return null;
                }
            }
            else
            {
                (aU1, aU2) = SurfaceRange(theO, u1, u2);
                (aV1, aV2) = SurfaceRange(theO, v1, v2);
                theWhat = $"{SurfaceKinds[theKind]} [{aU1:R}, {aU2:R}] x [{aV1:R}, {aV2:R}]";
                try
                {
                    using var aMF = new N.BRepBuilderAPI_MakeFace(aNS, aU1, aU2, aV1, aV2, 1e-7);
                    aFace = aMF.IsDone() ? aMF.Face() : null;
                }
                catch (Exception)
                {
                    return null;
                }
            }
            if (aFace is not null)
            {
                (thePort, theIsos) = MakeFaceGeometry(aPS, aU1, aU2, aV1, aV2);
            }
            return aFace;
        }
    }

    /// <summary>
    /// The surface BRepLib_MakeFace::Init(S, UMin, UMax, VMin, VMax) stores, and the iso curves it makes the edges'
    /// 3D curves of, built in the port as OCCT builds them.
    /// </summary>
    private static (Surface Surface, List<Curve3D> Isos) MakeFaceGeometry(Surface SS, double UMin, double UMax,
                                                                                   double VMin, double VMax)
    {
        Surface S = SS, BS = SS;
        if (S is TrimmedSurface RS)
        {
            BS = RS.BasisSurface();
        }
        BS.Bounds(out double umin, out double umax, out double vmin, out double vmax);
        if (BS.GetType() == typeof(OffsetSurface))
        {
            var OS = (OffsetSurface)BS;
            Surface Base = OS.BasisSurface();
            if (Base.GetType() == typeof(ExtrusionSurface))
            {
                S = global::Kerf.Engine.Foundation.Precision.IsInfinite(umin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(umax)
                      ? new TrimmedSurface(OS, UMin, UMax, VMin, VMax)
                      : new TrimmedSurface(OS, VMin, VMax, false);
            }
            else if (Base.GetType() == typeof(RevolutionSurface))
            {
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(vmin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(vmax))
                {
                    S = new TrimmedSurface(OS, VMin, VMax, false);
                }
            }
        }
        double epsilon = global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (S.IsUPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(umin, umax, epsilon, ref UMin, ref UMax);
        }
        if (S.IsVPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(vmin, vmax, epsilon, ref VMin, ref VMax);
        }
        var anIsos = new List<Curve3D>();
        if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(UMin))
        {
            anIsos.Add(S.UIso(UMin));
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(UMax))
        {
            anIsos.Add(S.UIso(UMax));
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMin))
        {
            anIsos.Add(S.VIso(VMin));
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax))
        {
            anIsos.Add(S.VIso(VMax));
        }
        return (S, anIsos);
    }


    // ---------------------------------------------------------------------------------------------------------
    // Comparison

    // Plan 3.2's third level (occt-issues.md, unit 4B's GeomBndLib_Parabola row; Kerf unit 10K1, decision 1429): the
    // box of a parabolic arc adds each coordinate's extremum, which OCCT's missed, so it contains native's box and may
    // be larger. The recorded native calls stay the same, in the same order.
    private static bool IsParabolic(Curve3D theCurve) => theCurve switch
    {
        ParabolaCurve3D => true,
        TrimmedCurve3D aTrimmed => IsParabolic(aTrimmed.BasisCurve()),
        _ => false,
    };

    private static void SameBox(Oracle theO, N.Bnd_Box theN, in Box theP, string theWhat, double theTol = Oracle.Tolerance,
                                bool theIsParabolic = false)
    {
        theO.Same(theN.IsVoid(), theP.IsVoid, theWhat + " IsVoid");
        if (theN.IsVoid())
        {
            return;
        }
        theO.Same(theN.IsOpenXmin(), theP.IsOpenXmin, theWhat + " IsOpenXmin");
        theO.Same(theN.IsOpenXmax(), theP.IsOpenXmax, theWhat + " IsOpenXmax");
        theO.Same(theN.IsOpenYmin(), theP.IsOpenYmin, theWhat + " IsOpenYmin");
        theO.Same(theN.IsOpenYmax(), theP.IsOpenYmax, theWhat + " IsOpenYmax");
        theO.Same(theN.IsOpenZmin(), theP.IsOpenZmin, theWhat + " IsOpenZmin");
        theO.Same(theN.IsOpenZmax(), theP.IsOpenZmax, theWhat + " IsOpenZmax");
        if (theIsParabolic)
        {
            Below(theN.GetXMin(), theP.GetXMin(), theWhat + " Xmin");
            Below(-theN.GetXMax(), -theP.GetXMax(), theWhat + " Xmax");
            Below(theN.GetYMin(), theP.GetYMin(), theWhat + " Ymin");
            Below(-theN.GetYMax(), -theP.GetYMax(), theWhat + " Ymax");
            Below(theN.GetZMin(), theP.GetZMin(), theWhat + " Zmin");
            Below(-theN.GetZMax(), -theP.GetZMax(), theWhat + " Zmax");
            theO.Same(theN.GetGap(), theP.GetGap(), theWhat + " Gap", theTol);
            return;
        }
        theO.Same(theN.GetXMin(), theP.GetXMin(), theWhat + " Xmin", theTol);
        theO.Same(theN.GetXMax(), theP.GetXMax(), theWhat + " Xmax", theTol);
        theO.Same(theN.GetYMin(), theP.GetYMin(), theWhat + " Ymin", theTol);
        theO.Same(theN.GetYMax(), theP.GetYMax(), theWhat + " Ymax", theTol);
        theO.Same(theN.GetZMin(), theP.GetZMin(), theWhat + " Zmin", theTol);
        theO.Same(theN.GetZMax(), theP.GetZMax(), theWhat + " Zmax", theTol);
        theO.Same(theN.GetGap(), theP.GetGap(), theWhat + " Gap", theTol);
    }

    // the port's bound at or beyond native's
    private static void Below(double theNative, double thePort, string theWhat)
        => Assert.True(thePort <= theNative + 1e-12 * Math.Max(1.0, Math.Abs(theNative)), $"{theWhat}: native {theNative}, port {thePort}");

    private static void SameOBB(Oracle theO, N.Bnd_OBB theN, in OrientedBox theP, string theWhat, double theTol,
                                bool theIsParabolic = false)
    {
        theO.Same(theN.IsVoid(), theP.IsVoid, theWhat + " IsVoid");
        if (theN.IsVoid())
        {
            return;
        }
        theO.Same(theN.IsAABox(), theP.IsAABox, theWhat + " IsAABox");
        global::Kerf.Engine.Geometry.Xyz c, x, y, z;
        using (N.gp_XYZ aC = theN.Center())
        {
            c = new global::Kerf.Engine.Geometry.Xyz(aC.X(), aC.Y(), aC.Z());
        }
        using (N.gp_XYZ aX = theN.XDirection())
        {
            x = new global::Kerf.Engine.Geometry.Xyz(aX.X(), aX.Y(), aX.Z());
        }
        using (N.gp_XYZ aY = theN.YDirection())
        {
            y = new global::Kerf.Engine.Geometry.Xyz(aY.X(), aY.Y(), aY.Z());
        }
        using (N.gp_XYZ aZ = theN.ZDirection())
        {
            z = new global::Kerf.Engine.Geometry.Xyz(aZ.X(), aZ.Y(), aZ.Z());
        }
        double hx = theN.XHSize(), hy = theN.YHSize(), hz = theN.ZHSize();
        if (theIsParabolic)
        {
            // the port's oriented box of a parabolic arc contains native's corners
            double aTol = theTol * Math.Max(1.0, c.Modulus() + hx + hy + hz);
            foreach (int i in new[] { -1, 1 })
            {
                foreach (int j in new[] { -1, 1 })
                {
                    foreach (int k in new[] { -1, 1 })
                    {
                        global::Kerf.Engine.Geometry.Xyz aCorner = c + x * (i * hx) + y * (j * hy) + z * (k * hz) - theP.Center();
                        Assert.True(Math.Abs(aCorner.Dot(theP.XDirection())) <= theP.XHSize() + aTol
                                    && Math.Abs(aCorner.Dot(theP.YDirection())) <= theP.YHSize() + aTol
                                    && Math.Abs(aCorner.Dot(theP.ZDirection())) <= theP.ZHSize() + aTol,
                                    theWhat + ": the port's oriented box doesn't contain native's");
                    }
                }
            }
            return;
        }
        theO.Same(c, theP.Center(), theWhat + " Center", theTol);
        theO.Same(x, theP.XDirection(), theWhat + " XDirection", theTol);
        theO.Same(y, theP.YDirection(), theWhat + " YDirection", theTol);
        theO.Same(z, theP.ZDirection(), theWhat + " ZDirection", theTol);
        theO.Same(hx, theP.XHSize(), theWhat + " XHSize", theTol);
        theO.Same(hy, theP.YHSize(), theWhat + " YHSize", theTol);
        theO.Same(hz, theP.ZHSize(), theWhat + " ZHSize", theTol);
    }

    // ---------------------------------------------------------------------------------------------------------
    // The port's side of BRepBndLib

    /// <summary>BRepAdaptor_Curve(E): the edge's curve on its range, transformed by its location.</summary>
    private static TransformedCurve EdgeAdaptor(EdgeData theE, in Transform3D theLoc)
        => new(theE.Curve!, theE.First, theE.Last, theLoc);

    /// <summary>BRepBndLib::Add of a face or an edge (no triangulation), moved by theLoc.</summary>
    private static void PortAdd(Surface? theSurf, (double U1, double U2, double V1, double V2) theUV, double theFaceTol,
                                List<EdgeData> theEdges, in Transform3D theLoc, ref Box theB)
    {
        if (theSurf is not null)
        {
            // BS.Initialize(F, false) to read the type, then BS.Initialize(F) (UV bounds)
            var aBSAll = new TransformedSurface(theSurf, theLoc);
            if (aBSAll.GetType() != SurfaceKind.Plane)
            {
                var aBS = new TransformedSurface(theSurf, theUV.U1, theUV.U2, theUV.V1, theUV.V2, theLoc);
                AddSurface.Add(aBS, theFaceTol, ref theB);
            }
            else
            {
                Assert.NotEmpty(theEdges);
                foreach (EdgeData anE in theEdges)
                {
                    if (anE.Geometric)
                    {
                        Add3dCurve.Add(EdgeAdaptor(anE, theLoc), anE.Tolerance, ref theB);
                    }
                }
                theB.Enlarge(theFaceTol);
            }
            return;
        }
        foreach (EdgeData anE in theEdges)
        {
            Add3dCurve.Add(EdgeAdaptor(anE, theLoc), anE.Tolerance, ref theB);
        }
    }

    /// <summary>BRepBndLib.cxx's CanUseEdges.</summary>
    private static bool CanUseEdges(SurfaceAdaptorBase BS)
    {
        SurfaceKind aST = BS.GetType();
        if (aST == SurfaceKind.Plane || aST == SurfaceKind.Cylinder
            || aST == SurfaceKind.Cone || aST == SurfaceKind.SurfaceOfExtrusion)
        {
            return true;
        }
        else if (aST == SurfaceKind.SurfaceOfRevolution)
        {
            return BS.BasisCurve().GetType() == CurveKind.Line;
        }
        else if (aST == SurfaceKind.OffsetSurface)
        {
            return CanUseEdges(BS.BasisSurface());
        }
        else if (aST == SurfaceKind.BSplineSurface)
        {
            BSplineSurface aBSpl = BS.BSpline();
            return (aBSpl.UDegree() == 1 && aBSpl.UKnotCount == 2) || (aBSpl.VDegree() == 1 && aBSpl.VKnotCount == 2);
        }
        else if (aST == SurfaceKind.BezierSurface)
        {
            BezierSurface aBz = BS.Bezier();
            return aBz.UDegree() == 1 || aBz.VDegree() == 1;
        }
        return false;
    }

    /// <summary>BRepBndLib_1.cxx's IsLinear.</summary>
    private static bool IsLinear(CurveAdaptorBase theC)
    {
        CurveKind aCT = theC.GetType();
        if (aCT == CurveKind.OffsetCurve)
        {
            return IsLinear(new CurveAdaptor(theC.OffsetCurve().BasisCurve()));
        }
        if (aCT == CurveKind.BSplineCurve || aCT == CurveKind.BezierCurve)
        {
            return theC.Degree() == 1 && theC.Continuity() != Continuity.C0;
        }
        return aCT == CurveKind.Line;
    }

    /// <summary>BRepBndLib_1.cxx's IsPlanar.</summary>
    private static bool IsPlanar(SurfaceAdaptorBase theS)
    {
        SurfaceKind aST = theS.GetType();
        if (aST == SurfaceKind.OffsetSurface)
        {
            return IsPlanar(theS.BasisSurface());
        }
        if (aST == SurfaceKind.SurfaceOfExtrusion)
        {
            return IsLinear(theS.BasisCurve());
        }
        if (aST == SurfaceKind.BSplineSurface || aST == SurfaceKind.BezierSurface)
        {
            if (theS.UDegree() != 1 || theS.VDegree() != 1)
            {
                return false;
            }
            return theS.UContinuity() != Continuity.C0 && theS.VContinuity() != Continuity.C0;
        }
        return aST == SurfaceKind.Plane;
    }

    /// <summary>BRepBndLib_1.cxx's IsWCS.</summary>
    private static int IsWCS(in Direction3D theDir)
    {
        double aToler = global::Kerf.Engine.Foundation.Precision.Angular() * global::Kerf.Engine.Foundation.Precision.Angular();
        double aX = theDir.X(), aY = theDir.Y(), aZ = theDir.Z();
        double aVx = aY * aY + aZ * aZ, aVy = aX * aX + aZ * aZ, aVz = aX * aX + aY * aY;
        if (aVz < aToler)
        {
            return 3;
        }
        if (aVy < aToler)
        {
            return 2;
        }
        if (aVx < aToler)
        {
            return 1;
        }
        return 0;
    }

    private static Direction3D ToDir(N.gp_Vec theV) => new(theV.X(), theV.Y(), theV.Z());

    /// <summary>
    /// The port's side of BRepBndLib::AddOBB (no triangulation, not optimal, shape tolerances): the vertices when
    /// the shape is linear or planar with linear edges (CheckPoints), else ComputePCA with native mass properties.
    /// </summary>
    private OrientedBox PortOBB(N.TopoDS_Shape theShape, Surface? theSurf, (double U1, double U2, double V1, double V2) theUV,
                                   double theFaceTol, List<EdgeData> theEdges, bool theIsFace)
    {
        // PointsForOBB with triangulation allowed but absent
        bool aPointsOnly;
        if (theIsFace)
        {
            aPointsOnly = IsPlanar(new TransformedSurface(theSurf!, new Transform3D()))
                          && theEdges.TrueForAll(e => !e.Geometric || IsLinear(EdgeAdaptor(e, new Transform3D())));
        }
        else
        {
            aPointsOnly = IsLinear(EdgeAdaptor(theEdges[0], new Transform3D()));
        }

        var anOBB = new OrientedBox();
        if (aPointsOnly)
        {
            PointsOnlyCases++;
            List<(Point3D P, double Tol)> aVerts = Vertices(theShape);
            var aPnts = new NCollection_Array1<Point3D>(0, aVerts.Count - 1);
            var aTols = new NCollection_Array1<double>(0, aVerts.Count - 1);
            for (int i = 0; i < aVerts.Count; i++)
            {
                aPnts[i] = aVerts[i].P;
                aTols[i] = aVerts[i].Tol;
            }
            anOBB.ReBuild(aPnts, aTols, false);
            return anOBB;
        }

        // ComputePCA, with ComputeProperties computed natively
        PcaCases++;
        using var aGCommon = new N.GProp_GProps();
        using (var aG = new N.GProp_GProps())
        {
            if (theIsFace)
            {
                N.BRepGProp.SurfaceProperties(theShape, aG, true);
            }
            else
            {
                N.BRepGProp.LinearProperties(theShape, aG, true);
            }
            aGCommon.Add(aG);
        }
        Transform3D aTrsf = new();
        using (N.GProp_PrincipalProps aPP = aGCommon.PrincipalProperties())
        using (N.gp_Vec aV1 = aPP.FirstAxisOfInertia())
        using (N.gp_Vec aV2 = aPP.SecondAxisOfInertia())
        using (N.gp_Vec aV3 = aPP.ThirdAxisOfInertia())
        using (N.gp_Pnt aCM = aGCommon.CentreOfMass())
        {
            int anIdx1 = IsWCS(ToDir(aV1));
            int anIdx2 = IsWCS(ToDir(aV2));
            if (anIdx1 == 0 || anIdx2 == 0)
            {
                CoordinateSystem aLocCoordSys = new(new Point3D(aCM.X(), aCM.Y(), aCM.Z()), ToDir(aV3), ToDir(aV1));
                aTrsf = aTrsf.WithTransformation(aLocCoordSys);
            }
        }

        // BRepBndLib::Add of the moved shape
        Box aShapeBox = new();
        PortAdd(theSurf, theUV, theFaceTol, theEdges, aTrsf, ref aShapeBox);
        if (aShapeBox.IsVoid)
        {
            return anOBB;
        }

        Point3D aPMin = aShapeBox.CornerMin();
        Point3D aPMax = aShapeBox.CornerMax();
        global::Kerf.Engine.Geometry.Xyz aXDir = new(1, 0, 0);
        global::Kerf.Engine.Geometry.Xyz aYDir = new(0, 1, 0);
        global::Kerf.Engine.Geometry.Xyz aZDir = new(0, 0, 1);
        global::Kerf.Engine.Geometry.Xyz aCenter = (aPMin.XYZ() + aPMax.XYZ()) / 2.0;
        global::Kerf.Engine.Geometry.Xyz anOBBHSize = (aPMax.XYZ() - aPMin.XYZ()) / 2.0;
        bool anIdentity = aTrsf.Form() == TransformForm.Identity;
        if (!anIdentity)
        {
            aTrsf = aTrsf.Inverted();
            aTrsf.Transforms(ref aCenter);
            Matrix3x3 aMat = aTrsf.HVectorialPart();
            aXDir = new global::Kerf.Engine.Geometry.Xyz(aMat.Value(1, 1), aMat.Value(2, 1), aMat.Value(3, 1));
            aYDir = new global::Kerf.Engine.Geometry.Xyz(aMat.Value(1, 2), aMat.Value(2, 2), aMat.Value(3, 2));
            aZDir = new global::Kerf.Engine.Geometry.Xyz(aMat.Value(1, 3), aMat.Value(2, 3), aMat.Value(3, 3));
        }
        anOBB.SetCenter(new Point3D(aCenter));
        anOBB.SetXComponent(new Direction3D(aXDir), anOBBHSize.X());
        anOBB.SetYComponent(new Direction3D(aYDir), anOBBHSize.Y());
        anOBB.SetZComponent(new Direction3D(aZDir), anOBBHSize.Z());
        anOBB.SetAABox(anIdentity);
        return anOBB;
    }

    // ---------------------------------------------------------------------------------------------------------
    // Cases where native OCCT reads uninitialised memory

    /// <summary>
    /// True when GeomBndLib_BSplineCurve::Box would segment a periodic B-spline (theC, or the basis of a trimmed or
    /// offset curve) with a segment tolerance of 0: when [theU1, theU2] differs from the curve's range but, after
    /// ElCLib::AdjustPeriodic, ends a whole period from its start to the last bit (U2 = Last - an ulp, from
    /// BRepTools::UVBounds). Geom_BSplineCurve::Segment(U1, U2, 0) then leaves a pole of the result unwritten:
    /// native OCCT reads uninitialised memory there (the box changes from run to run), the port reads 0
    /// (occt-issues.md). Such cases can't be compared.
    /// </summary>
    private static bool SegmentsWithZeroTolerance(Curve3D theC, double theU1, double theU2)
    {
        while (true)
        {
            if (theC is TrimmedCurve3D aT)
            {
                theC = aT.BasisCurve();
            }
            else if (theC is OffsetCurve3D anO)
            {
                theC = anO.BasisCurve();
            }
            else
            {
                break;
            }
        }
        if (theC is not BSplineCurve3D aBS || !aBS.IsPeriodic)
        {
            return false;
        }
        if (aBS.FirstParameter() == theU1 && aBS.LastParameter() == theU2)
        {
            return false;
        }
        double aU1 = theU1, aU2 = theU2;
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(aBS.FirstParameter(), aBS.LastParameter(), global::Kerf.Engine.Foundation.Precision.PConfusion(), ref aU1, ref aU2);
        double aPeriod = aBS.LastParameter() - aBS.FirstParameter();
        double aMinDiff = Math.Min(Math.Abs(aU2 - aU1), Math.Min(Math.Abs(aU2 - aPeriod - aU1), Math.Abs(aU1 - aPeriod - aU2)));
        return aMinDiff == 0.0;
    }

    /// <summary><see cref="SegmentsWithZeroTolerance"/> for a face: its edges and the basis curve of a swept surface.</summary>
    private static bool SegmentsWithZeroTolerance(Surface theS, (double U1, double U2, double V1, double V2) theUV,
                                                  List<EdgeData> theEdges)
    {
        foreach (EdgeData anE in theEdges)
        {
            if (anE.Curve is not null && SegmentsWithZeroTolerance(anE.Curve, anE.First, anE.Last))
            {
                return true;
            }
        }
        while (true)
        {
            if (theS is TrimmedSurface aT)
            {
                theS = aT.BasisSurface();
            }
            else if (theS is OffsetSurface anO)
            {
                theS = anO.BasisSurface();
            }
            else
            {
                break;
            }
        }
        return theS switch
        {
            ExtrusionSurface anExt => SegmentsWithZeroTolerance(anExt.BasisCurve(), theUV.U1, theUV.U2),
            RevolutionSurface aRev => SegmentsWithZeroTolerance(aRev.BasisCurve(), theUV.V1, theUV.V2),
            _ => false,
        };
    }

    // ---------------------------------------------------------------------------------------------------------
    // Tests

    [Fact]
    public void Add_Edges_through_BRepBndLib()
    {
        var anOracle = new Oracle(output, 20261101);
        int aCases = 0;
        for (int aKind = 0; aKind < CurveKinds.Length; aKind++)
        {
            for (int c = 0; c < CasesPerKind; c++)
            {
                using N.TopoDS_Edge? aNE = RandomEdge(anOracle, aKind, out string aW, out Curve3D aPC);
                if (aNE is null)
                {
                    continue;
                }
                EdgeData anE = Edge(aNE, [aPC], aW);
                if (SegmentsWithZeroTolerance(anE.Curve!, anE.First, anE.Last))
                {
                    UndefinedCases++;
                    continue;
                }
                Assert.NotNull(anE.Curve);

                using var aNB = new N.Bnd_Box();
                N.BRepBndLib.Add(aNE, aNB, false);
                Box aPB = new();
                PortAdd(null, default, 0, [anE], new Transform3D(), ref aPB);
                SameBox(anOracle, aNB, aPB, aW, theIsParabolic: IsParabolic(aPC));

                // useTriangulation = true: the edge has no polygon, so the same path
                using var aNB2 = new N.Bnd_Box();
                N.BRepBndLib.Add(aNE, aNB2, true);
                SameBox(anOracle, aNB2, aPB, aW + " (useTriangulation)", theIsParabolic: IsParabolic(aPC));
                aCases++;
            }
        }
        output.WriteLine($"{aCases} edges");
        output.WriteLine($"{UndefinedCases} cases skipped (OCCT reads uninitialised memory, see SegmentsWithZeroTolerance)");
        anOracle.Report("BndLib_Add3dCurve through BRepBndLib::Add on edges");
    }

    [Fact]
    public void AddOptimal_Edges_through_BRepBndLib()
    {
        var anOracle = new Oracle(output, 20261102);
        int aCases = 0;
        for (int aKind = 0; aKind < CurveKinds.Length; aKind++)
        {
            for (int c = 0; c < CasesPerKind; c++)
            {
                using N.TopoDS_Edge? aNE = RandomEdge(anOracle, aKind, out string aW, out Curve3D aPC);
                if (aNE is null)
                {
                    continue;
                }
                EdgeData anE = Edge(aNE, [aPC], aW);
                foreach (bool aUseTol in new[] { false, true })
                {
                    using var aNB = new N.Bnd_Box();
                    N.BRepBndLib.AddOptimal(aNE, aNB, false, aUseTol);

                    // aLocBox from BndLib_Add3dCurve::AddOptimal, then B.Update(aLocBox.Get())
                    Box aLocBox = new();
                    Add3dCurve.AddOptimal(EdgeAdaptor(anE, new Transform3D()), aUseTol ? anE.Tolerance : 0.0, ref aLocBox);
                    Box aPB = new();
                    if (!aLocBox.IsVoid)
                    {
                        aLocBox.Get(out double x, out double y, out double z, out double X, out double Y, out double Z);
                        aPB.Update(x, y, z, X, Y, Z);
                    }
                    SameBox(anOracle, aNB, aPB, $"{aW} useShapeTolerance={aUseTol}", theIsParabolic: IsParabolic(aPC));
                }
                aCases++;
            }
        }
        output.WriteLine($"{aCases} edges");
        anOracle.Report("BndLib_Add3dCurve::AddOptimal through BRepBndLib::AddOptimal on edges");
    }

    [Fact]
    public void Add_Faces_through_BRepBndLib()
    {
        var anOracle = new Oracle(output, 20261103);
        int aCases = 0, aPlanes = 0;
        int[] aPerKind = new int[SurfaceKinds.Length];
        for (int aKind = 0; aKind < SurfaceKinds.Length; aKind++)
        {
            for (int c = 0; c < CasesPerKind; c++)
            {
                using N.TopoDS_Face? aNF = RandomFace(anOracle, aKind, anOracle.Int(0, 4) == 0, out string aW,
                                                          out Surface aSurf, out List<Curve3D> anIsos);
                if (aNF is null)
                {
                    continue;
                }
                CheckFaceSurface(aNF, aSurf, aW);
                List<EdgeData> anEdges = FaceEdges(aNF, anIsos, aW);
                if (SegmentsWithZeroTolerance(aSurf, UVBounds(aNF), anEdges))
                {
                    UndefinedCases++;
                    continue;
                }
                var anUV = UVBounds(aNF);
                double aTolF = N.BRep_Tool.Tolerance(aNF);
                if (new TransformedSurface(aSurf, new Transform3D()).GetType() == SurfaceKind.Plane)
                {
                    aPlanes++;
                }

                using var aNB = new N.Bnd_Box();
                N.BRepBndLib.Add(aNF, aNB, false);
                Box aPB = new();
                PortAdd(aSurf, anUV, aTolF, anEdges, new Transform3D(), ref aPB);
                SameBox(anOracle, aNB, aPB, aW);
                aCases++;
                aPerKind[aKind]++;
            }
        }
        output.WriteLine($"{aCases} faces ({aPlanes} planar, through their edges): "
                         + string.Join(", ", SurfaceKinds.Select((k, i) => $"{k} {aPerKind[i]}")));
        output.WriteLine($"{UndefinedCases} cases skipped (OCCT reads uninitialised memory, see SegmentsWithZeroTolerance)");
        anOracle.Report("BndLib_AddSurface through BRepBndLib::Add on faces");
    }

    [Fact]
    public void AddOptimal_Faces_through_BRepBndLib()
    {
        var anOracle = new Oracle(output, 20261104);
        int aCases = 0, anEdgeCases = 0, aSkipped = 0;
        int[] aPerKind = new int[SurfaceKinds.Length];
        for (int aKind = 0; aKind < SurfaceKinds.Length; aKind++)
        {
            for (int c = 0; c < CasesPerKind; c++)
            {
                bool aWhole = anOracle.Int(0, 2) == 0;
                using N.TopoDS_Face? aNF = RandomFace(anOracle, aKind, aWhole, out string aW, out Surface aSurf, out List<Curve3D> anIsos);
                if (aNF is null)
                {
                    continue;
                }
                CheckFaceSurface(aNF, aSurf, aW);
                var aBS = new TransformedSurface(aSurf, new Transform3D()); // BS.Initialize(F, false)
                bool aCanUseEdges = CanUseEdges(aBS);
                if (!aCanUseEdges && !N.BRep_Tool.NaturalRestriction(aNF))
                {
                    // FindExactUVBounds through the pcurves (BndLib_Add2dCurve) and AdjustFaceBox (Extrema_ExtSS,
                    // BRepTopAdaptor_FClass2d): not mirrored here
                    aSkipped++;
                    continue;
                }
                List<EdgeData> anEdges = FaceEdges(aNF, anIsos, aW);
                double aTolF = N.BRep_Tool.Tolerance(aNF);
                foreach (bool aUseTol in new[] { false, true })
                {
                    using var aNB = new N.Bnd_Box();
                    N.BRepBndLib.AddOptimal(aNF, aNB, false, aUseTol);

                    Box aLocBox = new();
                    if (aCanUseEdges)
                    {
                        Assert.NotEmpty(anEdges);
                        foreach (EdgeData anE in anEdges)
                        {
                            if (anE.Degenerated || !anE.Geometric)
                            {
                                continue;
                            }
                            Box anEBox = new();
                            Add3dCurve.AddOptimal(EdgeAdaptor(anE, new Transform3D()), aUseTol ? anE.Tolerance : 0.0, ref anEBox);
                            aLocBox.Add(anEBox);
                        }
                    }
                    else
                    {
                        // natural restriction: the unrestricted adaptor's bounds
                        double aTol = aUseTol ? aTolF : 0.0;
                        AddSurface.AddOptimal(aBS, aBS.FirstUParameter(), aBS.LastUParameter(),
                                                     aBS.FirstVParameter(), aBS.LastVParameter(), aTol, ref aLocBox);
                    }
                    Box aPB = new();
                    if (!aLocBox.IsVoid)
                    {
                        aLocBox.Get(out double x, out double y, out double z, out double X, out double Y, out double Z);
                        aPB.Update(x, y, z, X, Y, Z);
                    }
                    SameBox(anOracle, aNB, aPB, $"{aW} useShapeTolerance={aUseTol}");
                }
                aCases++;
                aPerKind[aKind]++;
                if (aCanUseEdges)
                {
                    anEdgeCases++;
                }
            }
        }
        output.WriteLine($"{aCases} faces ({anEdgeCases} through their edges), {aSkipped} non-natural faces needing pcurves skipped: "
                         + string.Join(", ", SurfaceKinds.Select((k, i) => $"{k} {aPerKind[i]}")));
        anOracle.Report("BndLib_AddSurface/Add3dCurve::AddOptimal through BRepBndLib::AddOptimal on faces");
    }

    [Fact]
    public void AddOBB_Edges_and_Faces_through_BRepBndLib()
    {
        var anOracle = new Oracle(output, 20261105);
        int anEdges = 0, aFaces = 0;
        for (int aKind = 0; aKind < CurveKinds.Length; aKind++)
        {
            for (int c = 0; c < CasesPerKind / 2; c++)
            {
                using N.TopoDS_Edge? aNE = RandomEdge(anOracle, aKind, out string aW, out Curve3D aPC);
                if (aNE is null)
                {
                    continue;
                }
                EdgeData anE = Edge(aNE, [aPC], aW);
                if (SegmentsWithZeroTolerance(anE.Curve!, anE.First, anE.Last))
                {
                    UndefinedCases++;
                    continue;
                }
                using var aNOBB = new N.Bnd_OBB();
                N.BRepBndLib.AddOBB(aNE, aNOBB);
                OrientedBox aPOBB = PortOBB(aNE, null, default, 0, [anE], false);
                SameOBB(anOracle, aNOBB, aPOBB, aW, ObbTolerance, IsParabolic(aPC));
                anEdges++;
            }
        }
        for (int aKind = 0; aKind < SurfaceKinds.Length; aKind++)
        {
            for (int c = 0; c < CasesPerKind / 2; c++)
            {
                using N.TopoDS_Face? aNF = RandomFace(anOracle, aKind, false, out string aW, out Surface aSurf, out List<Curve3D> anIsos);
                if (aNF is null)
                {
                    continue;
                }
                CheckFaceSurface(aNF, aSurf, aW);
                List<EdgeData> anEdgeList = FaceEdges(aNF, anIsos, aW);
                if (SegmentsWithZeroTolerance(aSurf, UVBounds(aNF), anEdgeList))
                {
                    UndefinedCases++;
                    continue;
                }
                using var aNOBB = new N.Bnd_OBB();
                N.BRepBndLib.AddOBB(aNF, aNOBB);
                OrientedBox aPOBB = PortOBB(aNF, aSurf, UVBounds(aNF), N.BRep_Tool.Tolerance(aNF), anEdgeList, true);
                SameOBB(anOracle, aNOBB, aPOBB, aW, ObbTolerance);
                aFaces++;
            }
        }
        output.WriteLine($"{anEdges} edges, {aFaces} faces; {PointsOnlyCases} from vertices (CheckPoints), {PcaCases} through ComputePCA");
        output.WriteLine($"{UndefinedCases} cases skipped (OCCT reads uninitialised memory, see SegmentsWithZeroTolerance)");
        anOracle.Report("BRepBndLib::AddOBB on edges and faces");
    }

    private const double ObbTolerance = Oracle.Tolerance;

    private int PointsOnlyCases;
    private int UndefinedCases;
    private int PcaCases;
}
