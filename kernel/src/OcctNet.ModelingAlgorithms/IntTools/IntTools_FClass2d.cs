// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FClass2d.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FClass2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The 2D point classifier of a face used by the Boolean component: each wire is sampled once into a polygon
/// (<see cref="Class2d"/>), and points the polygons can't decide go to <see cref="BRepClass_FClassifier"/> over a
/// face explorer made on the first such point.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21), cached per face by <see cref="IntTools_Context.FClass2d"/>. Its queries keep no
/// result-changing state: the only thing they change is the lazily made face explorer, whose probing restarts at
/// every classification (decision 723).
/// </para>
/// <para>
/// OCCT's DEBUG_PCLASS_POLYGON block (DrawTrSurf) is left out.
/// </para>
/// </remarks>
public sealed class IntTools_FClass2d
{
    private readonly NCollection_Sequence<Class2d> TabClass = new();
    private readonly NCollection_Sequence<int> TabOrien = new();
    private double Toluv;
    private TopoDS_Face Face;
    // The period of the surface, set in U1..U2, V1..V2 and never read (as decision 221's fields)
#pragma warning disable CS0414
    private double U1;
    private double V1;
    private double U2;
    private double V2;
#pragma warning restore CS0414
    private double Umin;
    private double Umax;
    private double Vmin;
    private double Vmax;
    private bool myIsHole;

    // OCCT's mutable std::unique_ptr<BRepClass_FaceExplorer>, made by the first query the polygons can't decide.
    private BRepClass_FaceExplorer? myFExplorer;

    /// <summary>An empty classifier (every point is IN). OCCT leaves its numbers uninitialised; the port's are 0.</summary>
    public IntTools_FClass2d()
    {
        Face = new TopoDS_Face();
    }

    /// <summary>The classifier of the face <paramref name="aFace"/> with the tolerance <paramref name="TolUV"/> in its parameter space.</summary>
    public IntTools_FClass2d(in TopoDS_Face aFace, double TolUV)
    {
        Toluv = TolUV;
        Face = aFace;
        Init(Face, Toluv);
    }

    /// <summary>True if the last wire sampled with a non-zero area is clockwise (a hole), or no wire was.</summary>
    public bool IsHole() => myIsHole;

    /// <summary>Initializes the classifier with the face and the tolerance in its parameter space.</summary>
    /// <remarks>
    /// As in OCCT, a second <c>Init</c> appends the new face's polygons to the old ones and keeps the face explorer made
    /// for the old face (occt-issues.md); IntTools_Context never re-initializes a classifier.
    /// </remarks>
    public void Init(in TopoDS_Face aFace, double TolUV)
    {
        bool WireIsNotEmpty, Ancienpnt3dinitialise, degenerated;
        int firstpoint, NbEdges;
        int iX, aNbs1, nbs, Avant, BadWire;
        double u, du, Tole, Tol, pfbid, plbid;
        double FlecheU, FlecheV, TolVertex1, TolVertex;
        double uFirst, uLast;
        double aPrCf, aPrCf2;
        //
        TopoDS_Edge edge;
        TopAbs_Orientation Or;
        BRepTools_WireExplorer aWExp = new();
        TopExp_Explorer aExpF = new(), aExp = new();
        Geom2d_Curve? aC2D;
        Point3D Ancienpnt3d = new();
        NCollection_Sequence<Point2D> SeqPnt2d = new();
        NCollection_DataMap<int, int> anIndexMap = new();
        NCollection_Sequence<Vector2D> aD1Prev = new();
        NCollection_Sequence<Vector2D> aD1Next = new();
        //
        aPrCf = global::Kerf.Engine.Foundation.Precision.Confusion();
        aPrCf2 = aPrCf * aPrCf;
        myIsHole = true;
        //
        Toluv = TolUV;
        Face = aFace;
        Face.Orientation(TopAbs_FORWARD);
        BRepAdaptor_Surface surf = new();
        surf.Initialize(aFace, false);
        //
        Tole = 0.0;
        Tol = 0.0;
        Umin = Vmin = RealLast();
        Umax = Vmax = -Umin;
        BadWire = 0;
        //
        // if face has several wires and one of them is bad,
        // it is necessary to process all of them for correct
        // calculation of Umin, Umax, Vmin, Vmax - ifv, 23.08.06
        //
        aExpF.Init(Face, TopAbs_WIRE);
        for (; aExpF.More(); aExpF.Next())
        {
            TopoDS_Wire aW = TopoDS.Wire(aExpF.Current());
            //
            firstpoint = 1;
            FlecheU = 0.0;
            FlecheV = 0.0;
            TolVertex1 = 0.0;
            TolVertex = 0.0;
            WireIsNotEmpty = false;
            Ancienpnt3dinitialise = false;
            Ancienpnt3d.SetCoord(0.0, 0.0, 0.0);
            //
            SeqPnt2d.Clear();
            anIndexMap.Clear();
            aD1Prev.Clear();
            aD1Next.Clear();
            //
            // NbEdges
            NbEdges = 0;
            aExp.Init(aW, TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                NbEdges++;
            }
            //
            aWExp.Init(aW, Face);
            for (; aWExp.More(); aWExp.Next())
            {
                NbEdges--;
                edge = aWExp.Current();
                Or = edge.Orientation();
                if (Or != TopAbs_FORWARD && Or != TopAbs_REVERSED)
                {
                    continue;
                }
                //
                aC2D = BRep_Tool.CurveOnSurface(edge, Face, out pfbid, out plbid);
                if (aC2D is null)
                {
                    // OCCT returns here, leaving the classifier half built: the wires sampled so far, Umin..Vmax as far
                    // as they got, U1..V2 unset (occt-issues.md)
                    return;
                }
                //
                BRepAdaptor_Curve2d C = new(edge, Face);
                BRepAdaptor_Curve C3d = new();
                //------------------------------------------
                degenerated = false;
                if (BRep_Tool.Degenerated(edge) || BRep_Tool.IsClosed(edge, Face))
                {
                    degenerated = true;
                }
                //
                TopExp.Vertices(edge, out TopoDS_Vertex Va, out TopoDS_Vertex Vb);
                //
                TolVertex1 = 0.0;
                TolVertex = 0.0;
                if (Va.IsNull())
                {
                    degenerated = true;
                }
                else
                {
                    TolVertex1 = BRep_Tool.Tolerance(Va);
                }
                if (Vb.IsNull())
                {
                    degenerated = true;
                }
                else
                {
                    TolVertex = BRep_Tool.Tolerance(Vb);
                }
                //
                if (TolVertex < TolVertex1)
                {
                    TolVertex = TolVertex1;
                }
                //
                //-- Verification of cases when forgotten to code degenereted
                if (!degenerated)
                {
                    // check that whole curve is located in vicinity of its middle point
                    // (within sphere of Precision::Confusion() diameter)
                    C3d.Initialize(edge, Face);
                    Point3D P3da = C3d.Value(0.5 * (pfbid + plbid));
                    du = plbid - pfbid;
                    const int NBSTEPS = 10;
                    double aPrec2 = 0.25 * global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion();
                    degenerated = true;
                    for (int i = 0; i <= NBSTEPS; i++)
                    {
                        double U = pfbid + i * du / NBSTEPS;
                        Point3D P3db = C3d.Value(U);
                        double aR2 = P3da.SquareDistance(P3db);
                        if (aR2 > aPrec2)
                        {
                            degenerated = false;
                            break;
                        }
                    }
                } // if(!degenerated)
                //-- ----------------------------------------
                Tole = BRep_Tool.Tolerance(edge);
                if (Tole > Tol)
                {
                    Tol = Tole;
                }
                //
                // NbSamples +> nbs
                nbs = Geom2dInt_Geom2dCurveTool.NbSamples(C);
                if (nbs > 2)
                {
                    nbs *= 4;
                }
                du = (plbid - pfbid) / (double)(nbs - 1);
                //
                if (Or == TopAbs_FORWARD)
                {
                    u = pfbid;
                    uFirst = pfbid;
                    uLast = plbid;
                }
                else
                {
                    u = plbid;
                    uFirst = plbid;
                    uLast = pfbid;
                    du = -du;
                }
                //
                // aPrms
                aNbs1 = nbs + 1;
                NCollection_Array1<double> aPrms = new(1, aNbs1);
                //
                if (nbs == 2)
                {
                    double aCoef = 0.0025;
                    aPrms[1] = uFirst;
                    aPrms[2] = uFirst + aCoef * (uLast - uFirst);
                    aPrms[3] = uLast;
                }
                else if (nbs > 2)
                {
                    aNbs1 = nbs;
                    aPrms[1] = uFirst;
                    for (iX = 2; iX < aNbs1; ++iX)
                    {
                        aPrms[iX] = u + (iX - 1) * du;
                    }
                    aPrms[aNbs1] = uLast;
                }
                //
                //-- ------------------------------------------------------------
                //-- Check distance uv between the start point of the edge
                //-- and the last point saved in SeqPnt2d
                //-- To to set the first point of the current
                //-- afar from the last saved point
                Avant = SeqPnt2d.Length();
                for (iX = firstpoint; iX <= aNbs1; iX++)
                {
                    bool IsRealCurve3d;
                    int ii;
                    double aDstX;
                    Point2D P2d;
                    Point3D P3d = new();
                    //
                    u = aPrms[iX];
                    P2d = C.Value(u);
                    if (P2d.X() < Umin)
                    {
                        Umin = P2d.X();
                    }
                    if (P2d.X() > Umax)
                    {
                        Umax = P2d.X();
                    }
                    if (P2d.Y() < Vmin)
                    {
                        Vmin = P2d.Y();
                    }
                    if (P2d.Y() > Vmax)
                    {
                        Vmax = P2d.Y();
                    }
                    //
                    aDstX = RealLast();
                    if (!degenerated)
                    {
                        P3d = C3d.Value(u);
                        if (!SeqPnt2d.IsEmpty())
                        {
                            if (Ancienpnt3dinitialise)
                            {
                                aDstX = P3d.SquareDistance(Ancienpnt3d);
                            }
                        }
                    }
                    //
                    IsRealCurve3d = true;
                    if (aDstX < aPrCf2)
                    {
                        if (iX > 1)
                        {
                            double aDstX1;
                            Point3D MidP3d;
                            //
                            MidP3d = C3d.Value(0.5 * (u + aPrms[iX - 1]));
                            aDstX1 = P3d.SquareDistance(MidP3d);
                            if (aDstX1 < aPrCf2)
                            {
                                IsRealCurve3d = false;
                            }
                        }
                    }
                    //
                    if (IsRealCurve3d)
                    {
                        if (!degenerated)
                        {
                            Ancienpnt3d = P3d;
                            Ancienpnt3dinitialise = true;
                        }
                        SeqPnt2d.Append(P2d);
                    }
                    //
                    ii = SeqPnt2d.Length();
                    if (ii > (Avant + 4))
                    {
                        double ul, dU, dV;
                        Point2D Pp;
                        //
                        Line2D Lin = new(SeqPnt2d[ii - 2], new Direction2D(new Vector2D(SeqPnt2d[ii - 2], SeqPnt2d[ii])));
                        ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(Lin, SeqPnt2d[ii - 1]);
                        Pp = global::Kerf.Engine.Geometry.ElCLib.Value(ul, Lin);
                        dU = Math.Abs(Pp.X() - SeqPnt2d[ii - 1].X());
                        dV = Math.Abs(Pp.Y() - SeqPnt2d[ii - 1].Y());
                        if (dU > FlecheU)
                        {
                            FlecheU = dU;
                        }
                        if (dV > FlecheV)
                        {
                            FlecheV = dV;
                        }
                    }
                } // for(iX=firstpoint; iX<=aNbs1; iX++) {
                //
                if (BadWire != 0)
                {
                    continue; // if face has several wires and one of them is bad,
                              // it is necessary to process all of them for correct
                              // calculation of Umin, Umax, Vmin, Vmax - ifv, 23.08.06
                }
                //
                if (firstpoint == 1)
                {
                    firstpoint = 2;
                }
                WireIsNotEmpty = true;
                // Append the derivative of the first parameter.
                double aU = aPrms[1];
                C.D1(aU, out _, out Vector2D aV);

                if (Or == TopAbs_REVERSED)
                {
                    aV.Reverse();
                }

                aD1Next.Append(aV);

                // Append the derivative of the last parameter.
                aU = aPrms[aNbs1];
                C.D1(aU, out _, out aV);

                if (Or == TopAbs_REVERSED)
                {
                    aV.Reverse();
                }

                if (NbEdges > 0)
                {
                    aD1Prev.Append(aV);
                }
                else
                {
                    aD1Prev.Prepend(aV);
                }

                // Fill the map anIndexMap.
                if (Avant > 0)
                {
                    anIndexMap.Bind(Avant, aD1Next.Length());
                }
                else
                {
                    anIndexMap.Bind(1, aD1Next.Length());
                }
            } // for(;aWExp.More(); aWExp.Next()) {
            // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            //
            if (NbEdges != 0)
            {
                //-- count ++ with normal explorer and -- with Wire Explorer
                NCollection_Array1<Point2D> PClass = new(1, 2);
                Point2D anInitPnt = new(0.0, 0.0);
                //
                PClass.Init(anInitPnt);
                TabClass.Append(new Class2d(PClass, FlecheU, FlecheV, Umin, Vmin, Umax, Vmax));
                BadWire = 1;
                TabOrien.Append(-1);
            }
            //
            else if (WireIsNotEmpty)
            {
                if (SeqPnt2d.Length() > 3)
                {
                    global::Kerf.Engine.Meshing.Poly.PolygonProperties(SeqPnt2d, out double aS, out double aPer);

                    double anExpThick = Max(2.0 * Math.Abs(aS) / aPer, 1e-7);
                    double aDefl = Max(FlecheU, FlecheV);
                    double aDiscrDefl = Min(aDefl * 0.1, anExpThick * 10.0);
                    bool isChanged = false;
                    while (aDefl > anExpThick && aDiscrDefl > 1e-7)
                    {
                        // Deflection of the polygon is too much for this ratio of area and perimeter,
                        // and this might lead to self-intersections.
                        // Discretize the wire more tightly to eliminate the error.
                        firstpoint = 1;
                        isChanged = true;
                        SeqPnt2d.Clear();
                        FlecheU = 0.0;
                        FlecheV = 0.0;
                        for (aWExp.Init(TopoDS.Wire(aExpF.Current()), Face); aWExp.More(); aWExp.Next())
                        {
                            edge = aWExp.Current();
                            Or = edge.Orientation();
                            if (Or == TopAbs_FORWARD || Or == TopAbs_REVERSED)
                            {
                                BRep_Tool.Range(edge, Face, out pfbid, out plbid);
                                if (Math.Abs(plbid - pfbid) < 1.0e-9)
                                {
                                    continue;
                                }
                                BRepAdaptor_Curve2d C = new(edge, Face);
                                GCPnts_QuasiUniformDeflection aDiscr = new(C, aDiscrDefl);
                                if (!aDiscr.IsDone())
                                {
                                    break;
                                }
                                int nbp = aDiscr.NbPoints();
                                int iStep = 1, i = 1, iEnd = nbp + 1;
                                if (Or == TopAbs_REVERSED)
                                {
                                    iStep = -1;
                                    i = nbp;
                                    iEnd = 0;
                                }
                                if (firstpoint == 2)
                                {
                                    i += iStep;
                                }
                                for (; i != iEnd; i += iStep)
                                {
                                    Point2D aP2d = C.Value(aDiscr.Parameter(i));
                                    SeqPnt2d.Append(aP2d);
                                }
                                if (nbp > 2)
                                {
                                    int ii = SeqPnt2d.Length();
                                    Line2D Lin = new(SeqPnt2d[ii - 2], new Direction2D(new Vector2D(SeqPnt2d[ii - 2], SeqPnt2d[ii])));
                                    double ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(Lin, SeqPnt2d[ii - 1]);
                                    Point2D Pp = global::Kerf.Engine.Geometry.ElCLib.Value(ul, Lin);
                                    double dU = Math.Abs(Pp.X() - SeqPnt2d[ii - 1].X());
                                    double dV = Math.Abs(Pp.Y() - SeqPnt2d[ii - 1].Y());
                                    if (dU > FlecheU)
                                    {
                                        FlecheU = dU;
                                    }
                                    if (dV > FlecheV)
                                    {
                                        FlecheV = dV;
                                    }
                                }
                                firstpoint = 2;
                            }
                        }
                        // OCCT doesn't recompute the polygon's area and perimeter inside the loop, so anExpThick
                        // stays the first polygon's
                        anExpThick = Max(2.0 * Math.Abs(aS) / aPer, 1e-7);
                        aDefl = Max(FlecheU, FlecheV);
                        aDiscrDefl = Min(aDiscrDefl * 0.1, anExpThick * 10.0);
                    }

                    if (isChanged)
                    {
                        global::Kerf.Engine.Meshing.Poly.PolygonProperties(SeqPnt2d, out aS, out aPer);
                    }
                    //
                    if (FlecheU < Toluv)
                    {
                        FlecheU = Toluv;
                    }

                    if (FlecheV < Toluv)
                    {
                        FlecheV = Toluv;
                    }

                    TabClass.Append(new Class2d(SeqPnt2d, FlecheU, FlecheV, Umin, Vmin, Umax, Vmax));
                    //
                    if (Math.Abs(aS) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        BadWire = 1;
                        TabOrien.Append(-1);
                    }
                    else
                    {
                        if (aS > 0.0)
                        {
                            myIsHole = false;
                            TabOrien.Append(1);
                        }
                        else
                        {
                            myIsHole = true;
                            TabOrien.Append(0);
                        }
                    }
                }
                else
                {
                    BadWire = 1;
                    TabOrien.Append(-1);
                    SeqPnt2d.Clear();
                    TabClass.Append(new Class2d(SeqPnt2d, FlecheU, FlecheV, Umin, Vmin, Umax, Vmax));
                }
            } // else if(WireIsNotEmpty)
        } // for(; aExpF.More();  aExpF.Next()) {
        //
        int nbtabclass = TabClass.Length();
        //
        if (nbtabclass > 0)
        {
            //-- if an error on a wire was detected : all TabOrien set to -1
            if (BadWire != 0)
            {
                TabOrien[1] = -1;
            }

            if (surf.GetType() == SurfaceKind.Cone || surf.GetType() == SurfaceKind.Cylinder
                || surf.GetType() == SurfaceKind.Torus || surf.GetType() == SurfaceKind.Sphere
                || surf.GetType() == SurfaceOfRevolution)
            {
                double uuu = M_PI + M_PI - (Umax - Umin);
                if (uuu < 0)
                {
                    uuu = 0;
                }
                U1 = Umin - uuu * 0.5;
                U2 = U1 + M_PI + M_PI;
            }
            else
            {
                U1 = U2 = 0.0;
            }

            if (surf.GetType() == SurfaceKind.Torus)
            {
                double uuu = M_PI + M_PI - (Vmax - Vmin);
                if (uuu < 0)
                {
                    uuu = 0;
                }

                V1 = Vmin - uuu * 0.5;
                V2 = V1 + M_PI + M_PI;
            }
            else
            {
                V1 = V2 = 0.0;
            }
        }
    }

    /// <summary>The state of a point far outside the face's UV box (IN if the face has no box).</summary>
    public TopAbs_State PerformInfinitePoint()
    {
        if (Umax == -RealLast() || Vmax == -RealLast() || Umin == RealLast() || Vmin == RealLast())
        {
            return (TopAbs_IN);
        }
        Point2D P = new(Umin - (Umax - Umin), Vmin - (Vmax - Vmin));
        return (Perform(P, false));
    }

    /// <summary>
    /// The state of the point <paramref name="_Puv"/> in the face. On a periodic surface the point is also tried at
    /// its periods within the face's UV box, if <paramref name="RecadreOnPeriodic"/>.
    /// </summary>
    public TopAbs_State Perform(in Point2D _Puv, bool RecadreOnPeriodic = true)
    {
        int nbtabclass = TabClass.Length();
        if (nbtabclass == 0)
        {
            return TopAbs_IN;
        }

        //-- U1 is the First Param and U2 is in this case U1+Period
        double u = _Puv.X();
        double v = _Puv.Y();
        double uu = u;
        double vv = v;
        TopAbs_State aStatus = TopAbs_UNKNOWN;

        BRepAdaptor_Surface surf = new();
        surf.Initialize(Face, false);

        bool IsUPer = surf.IsUPeriodic();
        bool IsVPer = surf.IsVPeriodic();
        double uperiod = IsUPer ? surf.UPeriod() : 0.0;
        double vperiod = IsVPer ? surf.VPeriod() : 0.0;

        bool urecadre, vrecadre, bUseClassifier;
        int dedans;
        //
        urecadre = false;
        vrecadre = false;
        //
        if (RecadreOnPeriodic)
        {
            if (IsUPer)
            {
                GeomInt.AdjustPeriodic(uu, Umin, Umax, uperiod, out uu, out _);
            } // if (IsUPer) {
            //
            if (IsVPer)
            {
                GeomInt.AdjustPeriodic(vv, Vmin, Vmax, vperiod, out vv, out _);
            } // if (IsVPer) {
        }
        //
        for (;;)
        {
            dedans = 1;
            Point2D Puv = new(u, v);
            bUseClassifier = (TabOrien[1] == -1);
            if (!bUseClassifier)
            {
                int n, cur, TabOrien_n;
                for (n = 1; n <= nbtabclass; n++)
                {
                    cur = (int)TabClass[n].SiDans(Puv);
                    TabOrien_n = TabOrien[n];

                    if (cur == 1)
                    {
                        if (TabOrien_n == 0)
                        {
                            dedans = -1;
                            break;
                        }
                    }
                    else if (cur == -1)
                    {
                        if (TabOrien_n == 1)
                        {
                            dedans = -1;
                            break;
                        }
                    }
                    else
                    {
                        dedans = 0;
                        break;
                    }
                } // for(n=1; n<=nbtabclass; n++)

                if (dedans == 0)
                {
                    bUseClassifier = true;
                }
                else
                {
                    aStatus = (dedans == 1) ? TopAbs_IN : TopAbs_OUT;
                }
            } // if(TabOrien(1)!=-1) {
            // compute state of the point using face classifier
            if (bUseClassifier)
            {
                // compute tolerance to use in face classifier
                double aURes, aVRes, aFCTol;
                bool bUIn, bVIn;
                //
                aURes = surf.UResolution(Toluv);
                aVRes = surf.VResolution(Toluv);
                //
                bUIn = (u >= Umin) && (u <= Umax);
                bVIn = (v >= Vmin) && (v <= Vmax);
                //
                if (bUIn == bVIn)
                {
                    aFCTol = Min(aURes, aVRes);
                }
                else
                {
                    aFCTol = (!bUIn) ? aURes : aVRes;
                }
                //

                myFExplorer ??= new BRepClass_FaceExplorer(Face);

                BRepClass_FClassifier aClassifier = new();
                aClassifier.Perform(myFExplorer, Puv, aFCTol);
                aStatus = aClassifier.State();
            }

            if (!RecadreOnPeriodic || (!IsUPer && !IsVPer))
            {
                return aStatus;
            }

            if (aStatus == TopAbs_IN || aStatus == TopAbs_ON)
            {
                return aStatus;
            }

            if (!urecadre)
            {
                u = uu;
                urecadre = true;
            }
            else
            {
                if (IsUPer)
                {
                    u += uperiod;
                }
            }

            if (u > Umax || !IsUPer)
            {
                if (!vrecadre)
                {
                    v = vv;
                    vrecadre = true;
                }
                else
                {
                    if (IsVPer)
                    {
                        v += vperiod;
                    }
                }

                u = uu;

                if (v > Vmax || !IsVPer)
                {
                    return aStatus;
                }
            }
        } // while (1)
    }

    /// <summary>
    /// The state of the point <paramref name="_Puv"/> tested with an offset of <paramref name="Tol"/>: ON where some
    /// offset points are IN and some OUT (internal use).
    /// </summary>
    public TopAbs_State TestOnRestriction(in Point2D _Puv, double Tol, bool RecadreOnPeriodic = true)
    {
        int nbtabclass = TabClass.Length();
        if (nbtabclass == 0)
        {
            return TopAbs_IN;
        }

        //-- U1 is the First Param and U2 in this case is U1+Period
        double u = _Puv.X();
        double v = _Puv.Y();
        double uu = u, vv = v;

        BRepAdaptor_Surface surf = new();
        surf.Initialize(Face, false);
        bool IsUPer = surf.IsUPeriodic();
        bool IsVPer = surf.IsVPeriodic();
        double uperiod = IsUPer ? surf.UPeriod() : 0.0;
        double vperiod = IsVPer ? surf.VPeriod() : 0.0;
        TopAbs_State aStatus = TopAbs_UNKNOWN;
        bool urecadre = false, vrecadre = false;
        int dedans;

        if (RecadreOnPeriodic)
        {
            if (IsUPer)
            {
                GeomInt.AdjustPeriodic(uu, Umin, Umax, uperiod, out uu, out _);
            } // if (IsUPer) {
            //
            if (IsVPer)
            {
                GeomInt.AdjustPeriodic(vv, Vmin, Vmax, vperiod, out vv, out _);
            } // if (IsVPer) {
        }
        //
        for (;;)
        {
            dedans = 1;
            Point2D Puv = new(u, v);

            if (TabOrien[1] != -1)
            {
                for (int n = 1; n <= nbtabclass; n++)
                {
                    int cur = (int)TabClass[n].SiDans_OnMode(Puv, Tol);
                    if (cur == 1)
                    {
                        if (TabOrien[n] == 0)
                        {
                            dedans = -1;
                            break;
                        }
                    }
                    else if (cur == -1)
                    {
                        if (TabOrien[n] == 1)
                        {
                            dedans = -1;
                            break;
                        }
                    }
                    else
                    {
                        dedans = 0;
                        break;
                    }
                }
                if (dedans == 0)
                {
                    aStatus = TopAbs_ON;
                }
                if (dedans == 1)
                {
                    aStatus = TopAbs_IN;
                }
                if (dedans == -1)
                {
                    aStatus = TopAbs_OUT;
                }
            }
            else
            { //-- TabOrien(1)=-1  Wrong  Wire

                myFExplorer ??= new BRepClass_FaceExplorer(Face);

                BRepClass_FClassifier aClassifier = new();
                aClassifier.Perform(myFExplorer, Puv, Tol);
                aStatus = aClassifier.State();
            }

            if (!RecadreOnPeriodic || (!IsUPer && !IsVPer))
            {
                return aStatus;
            }
            if (aStatus == TopAbs_IN || aStatus == TopAbs_ON)
            {
                return aStatus;
            }

            if (!urecadre)
            {
                u = uu;
                urecadre = true;
            }
            else if (IsUPer)
            {
                u += uperiod;
            }
            if (u > Umax || !IsUPer)
            {
                if (!vrecadre)
                {
                    v = vv;
                    vrecadre = true;
                }
                else if (IsVPer)
                {
                    v += vperiod;
                }

                u = uu;

                if (v > Vmax || !IsVPer)
                {
                    return aStatus;
                }
            }
        } // for (;;)
    }
}
