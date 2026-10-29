// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_FClass2d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_FClass2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// A fast classifier of 2D points in a face: each wire is sampled once into a polygon (CSLib_Class2d), and points
/// that the polygons can't decide go to <see cref="BRepClass_FaceClassifier"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21) built once for a face; its queries don't change it. OCCT forbids copies (Copy and
/// the assignment throw Standard_ConstructionError): the port has neither.
/// </remarks>
public sealed class BRepTopAdaptor_FClass2d
{
    private readonly NCollection_Sequence<CSLib_Class2d> TabClass = new();
    private readonly NCollection_Sequence<int> TabOrien = new();
    private double Toluv;
    private TopoDS_Face Face;
    // OCCT sets the period of the surface in U1..U2, V1..V2 and never reads it (as decision 221's fields)
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

    // Increments theValue by theIncrement towards theDirection, ensuring that the result is
    // different from theValue. For large values of theValue with small theIncrement the result of
    // theValue + theIncrement can be equal to theValue due to the limited resolution of double
    // precision. This function guarantees to return the next representable value in the direction of
    // theDirection in such cases.
    private static double safeIncrement(double theValue, double theDirection, double theIncrement)
    {
        double aNextValue = theValue + theIncrement;
        return aNextValue == theValue ? NextAfter(theValue, theDirection) : aNextValue;
    }

    // std::nextafter
    private static double NextAfter(double theFrom, double theTo)
    {
        if (double.IsNaN(theFrom) || double.IsNaN(theTo))
        {
            return theFrom + theTo;
        }

        if (theFrom == theTo)
        {
            return theTo;
        }

        return theTo > theFrom ? Math.BitIncrement(theFrom) : Math.BitDecrement(theFrom);
    }

    // Checks whether the curve is degenerated between theStartParam and theEndParam.
    // The check is performed by sampling the curve at several points and measuring the distance to the
    // start point. If all sampled points are closer than Precision::Confusion() to the start point, the
    // curve is considered degenerated.
    private static bool isDegenerated(BRepAdaptor_Curve theCurve, double theStartParam, double theEndParam)
    {
        double aParametricStep = (theEndParam - theStartParam) * 0.1;
        gp_Pnt aStartPoint = theCurve.Value(theStartParam);
        for (double aCurrParam = theStartParam; aCurrParam < theEndParam;
             aCurrParam = safeIncrement(aCurrParam, theEndParam, aParametricStep))
        {
            gp_Pnt aCurrentPoint = theCurve.Value(aCurrParam);
            if (aStartPoint.SquareDistance(aCurrentPoint) > Precision.Confusion())
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>A classifier of the face <paramref name="aFace"/> with the tolerance <paramref name="TolUV"/> in its parameter space.</summary>
    public BRepTopAdaptor_FClass2d(in TopoDS_Face aFace, double TolUV)
    {
        Toluv = TolUV;
        Face = aFace;
        U1 = 0.0;
        V1 = 0.0;
        U2 = 0.0;
        V2 = 0.0;

        //-- dead end on surfaces defined on more than one period

        Face.Orientation(TopAbs_FORWARD);
        var surf = new BRepAdaptor_Surface();
        surf.Initialize(aFace, false);
        TopoDS_Edge edge;
        TopAbs_Orientation Or;
        var WireExplorer = new BRepTools_WireExplorer();

        Umin = Vmin = 0.0; // RealLast();
        Umax = Vmax = -Umin;

        int aNbE = 0;
        const double eps = 1.0e-10;
        bool anIsBadWire = false;
        for (var aFaceExplorer = new TopExp_Explorer(Face, TopAbs_WIRE); (aFaceExplorer.More() && !anIsBadWire);
             aFaceExplorer.Next())
        {
            int nbpnts = 0;
            var SeqPnt2d = new NCollection_Sequence<gp_Pnt2d>();
            int firstpoint = 1;
            double FlecheU = 0.0;
            double FlecheV = 0.0;
            bool WireIsNotEmpty = false;
            int NbEdges = 0;

            var Explorer = new TopExp_Explorer();
            for (Explorer.Init(aFaceExplorer.Current(), TopAbs_EDGE); Explorer.More(); Explorer.Next())
            {
                NbEdges++;
            }

            aNbE = NbEdges;

            gp_Pnt Ancienpnt3d = new(0, 0, 0);
            bool Ancienpnt3dinitialise = false;

            for (WireExplorer.Init(TopoDS.Wire(aFaceExplorer.Current()), Face); WireExplorer.More();
                 WireExplorer.Next())
            {
                NbEdges--;
                edge = WireExplorer.Current();
                Or = edge.Orientation();
                if (Or == TopAbs_FORWARD || Or == TopAbs_REVERSED)
                {
                    double pfbid, plbid;
                    if (BRep_Tool.CurveOnSurface(edge, Face, out pfbid, out plbid) is null)
                    {
                        // OCCT returns here, leaving the rest of the classifier as it is
                        return;
                    }

                    if (Math.Abs(plbid - pfbid) < 1.0e-9)
                    {
                        continue;
                    }

                    bool degenerated = false;
                    if (BRep_Tool.Degenerated(edge))
                    {
                        degenerated = true;
                    }

                    if (BRep_Tool.IsClosed(edge, Face))
                    {
                        degenerated = true;
                    }

                    TopExp.Vertices(edge, out TopoDS_Vertex Va, out TopoDS_Vertex Vb);
                    if (Va.IsNull() || Vb.IsNull())
                    {
                        degenerated = true;
                    }

                    var aCurveAdaptor2D = new BRepAdaptor_Curve2d(edge, Face);
                    var aCurveAdaptor3D = new BRepAdaptor_Curve(edge, Face);
                    //-- Check cases when it was forgotten to code degenerated :  PRO17410 (janv 99)
                    if (!degenerated)
                    {
                        degenerated = isDegenerated(aCurveAdaptor3D, pfbid, plbid);
                    }

                    //-- ----------------------------------------
                    // int nbs = 1 + Geom2dInt_Geom2dCurveTool::NbSamples(C);
                    int nbs = Geom2dInt_Geom2dCurveTool.NbSamples(aCurveAdaptor2D);
                    //-- Attention to rational bsplines of degree 3. (ends of circles among others)
                    if (nbs > 2)
                    {
                        nbs *= 4;
                    }

                    double du = (plbid - pfbid) / (double)(nbs - 1);
                    double u = 0.0;
                    if (Or == TopAbs_FORWARD)
                    {
                        u = pfbid;
                    }
                    else
                    {
                        u = plbid;
                        du = -du;
                    }

                    //-- ------------------------------------------------------------
                    //-- Check distance uv between the start point of the edge
                    //-- and the last point registered in SeqPnt2d
                    //-- Try to remote the first point of the current edge
                    //-- from the last saved point

                    if (firstpoint == 2)
                    {
                        u += du;
                    }

                    int Avant = nbpnts;
                    for (int e = firstpoint; e <= nbs; e++)
                    {
                        gp_Pnt2d P2d = aCurveAdaptor2D.Value(u);
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

                        double dist3dptcourant_ancienpnt = 1e+20; // RealLast();
                        gp_Pnt P3d = new();
                        if (!degenerated)
                        {
                            P3d = aCurveAdaptor3D.Value(u);
                            if (nbpnts > 1 && Ancienpnt3dinitialise)
                            {
                                dist3dptcourant_ancienpnt = P3d.Distance(Ancienpnt3d);
                            }
                        }

                        bool IsRealCurve3d = true; // patch
                        if (dist3dptcourant_ancienpnt < Precision.Confusion())
                        {
                            gp_Pnt MidP3d = aCurveAdaptor3D.Value(u - du / 2.0);
                            if (P3d.Distance(MidP3d) < Precision.Confusion())
                            {
                                IsRealCurve3d = false;
                            }
                        }

                        if (IsRealCurve3d)
                        {
                            if (!degenerated)
                            {
                                Ancienpnt3d = P3d;
                                Ancienpnt3dinitialise = true;
                            }

                            nbpnts++;
                            SeqPnt2d.Append(P2d);
                        }

                        u += du;
                        int ii = nbpnts;
                        //  Modified by Sergey KHROMOV - Fri Apr 19 09:46:12 2002 Begin
                        if (ii > (Avant + 4) && SeqPnt2d[ii - 2].SquareDistance(SeqPnt2d[ii]) != 0.0)
                        //  Modified by Sergey KHROMOV - Fri Apr 19 09:46:13 2002 End
                        {
                            var Lin = new gp_Lin2d(SeqPnt2d[ii - 2], new gp_Dir2d(new gp_Vec2d(SeqPnt2d[ii - 2], SeqPnt2d[ii])));
                            double ul = ElCLib.Parameter(Lin, SeqPnt2d[ii - 1]);
                            gp_Pnt2d Pp = ElCLib.Value(ul, Lin);
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
                    } // for(e=firstpoint

                    if (firstpoint == 1)
                    {
                        firstpoint = 2;
                    }

                    WireIsNotEmpty = true;
                } // if(Or==FORWARD,REVERSED
            } //-- Edges -> for(Ware.Explorer

            if (NbEdges != 0)
            {
                //-- on compte ++ with a normal explorer and with the Wire Explorer
                var PClass = new NCollection_Array1<gp_Pnt2d>(1, 2);
                //// modified by jgv, 28.04.2009 ////
                PClass.Init(new gp_Pnt2d(0.0, 0.0));
                /////////////////////////////////////
                TabClass.Append(new CSLib_Class2d(PClass, FlecheU, FlecheV, Umin, Vmin, Umax, Vmax));
                anIsBadWire = true;
                TabOrien.Append(-1);
            }
            else if (WireIsNotEmpty)
            {
                var PClass = new NCollection_Array1<gp_Pnt2d>(1, nbpnts);
                double square = 0.0;

                //-------------------------------------------------------------------
                //-- ** The mode of calculation was somewhat changed
                //-- Before Oct 31 97 , the total angle of
                //-- rotation of the wire was evaluated on all angles except for the last
                //-- ** Now, exactly the angle of rotation is evaluated
                //-- If a value remote from 2PI or -2PI is found, it means that there is
                //-- an uneven number of loops

                if (nbpnts > 3)
                {
                    int im1 = nbpnts - 1;
                    int im0 = 1;
                    PClass[im1] = SeqPnt2d.Value(im1);
                    PClass[nbpnts] = SeqPnt2d.Value(nbpnts);

                    double aPer = 0.0;
                    for (int ii = 1; ii < nbpnts; ii++, im0++, im1++)
                    {
                        if (im1 >= nbpnts)
                        {
                            im1 = 1;
                        }

                        PClass[ii] = SeqPnt2d.Value(ii);
                        square += (PClass[im0].X() - PClass[im1].X()) * (PClass[im0].Y() + PClass[im1].Y()) * .5;
                        aPer += (PClass[im0].XY() - PClass[im1].XY()).Modulus();
                    }

                    double anExpThick = Max(2.0 * Math.Abs(square) / aPer, 1e-7);
                    double aDefl = Max(FlecheU, FlecheV);
                    double aDiscrDefl = Min(aDefl * 0.1, anExpThick * 10.0);
                    while (aDefl > anExpThick && aDiscrDefl > 1e-7)
                    {
                        // Deflection of the polygon is too much for this ratio of area and perimeter,
                        // and this might lead to self-intersections.
                        // Discretize the wire more tightly to eliminate the error.
                        firstpoint = 1;
                        SeqPnt2d.Clear();
                        FlecheU = 0.0;
                        FlecheV = 0.0;
                        for (WireExplorer.Init(TopoDS.Wire(aFaceExplorer.Current()), Face); WireExplorer.More();
                             WireExplorer.Next())
                        {
                            edge = WireExplorer.Current();
                            Or = edge.Orientation();
                            if (Or == TopAbs_FORWARD || Or == TopAbs_REVERSED)
                            {
                                BRep_Tool.Range(edge, Face, out double pfbid, out double plbid);
                                if (Math.Abs(plbid - pfbid) < 1.0e-9)
                                {
                                    continue;
                                }

                                var C = new BRepAdaptor_Curve2d(edge, Face);
                                var aDiscr = new GCPnts_QuasiUniformDeflection(C, aDiscrDefl);
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
                                    gp_Pnt2d aP2d = C.Value(aDiscr.Parameter(i));
                                    SeqPnt2d.Append(aP2d);
                                }

                                if (nbp > 2)
                                {
                                    int ii = SeqPnt2d.Length();
                                    var Lin = new gp_Lin2d(SeqPnt2d[ii - 2], new gp_Dir2d(new gp_Vec2d(SeqPnt2d[ii - 2], SeqPnt2d[ii])));
                                    double ul = ElCLib.Parameter(Lin, SeqPnt2d[ii - 1]);
                                    gp_Pnt2d Pp = ElCLib.Value(ul, Lin);
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

                        nbpnts = SeqPnt2d.Length();
                        PClass.Resize(1, nbpnts, false);
                        im1 = nbpnts - 1;
                        im0 = 1;
                        PClass[im1] = SeqPnt2d.Value(im1);
                        PClass[nbpnts] = SeqPnt2d.Value(nbpnts);
                        square = 0.0;
                        aPer = 0.0;
                        for (int ii = 1; ii < nbpnts; ii++, im0++, im1++)
                        {
                            if (im1 >= nbpnts)
                            {
                                im1 = 1;
                            }

                            PClass[ii] = SeqPnt2d.Value(ii);
                            square +=
                                (PClass[im0].X() - PClass[im1].X()) * (PClass[im0].Y() + PClass[im1].Y()) * .5;
                            aPer += (PClass[im0].XY() - PClass[im1].XY()).Modulus();
                        }

                        anExpThick = Max(2.0 * Math.Abs(square) / aPer, 1e-7);
                        aDefl = Max(FlecheU, FlecheV);
                        aDiscrDefl = Min(aDiscrDefl * 0.1, anExpThick * 10.0);
                    }

                    //-- FlecheU*=10.0;
                    //-- FlecheV*=10.0;
                    if (aNbE == 1 && FlecheU < eps && FlecheV < eps && Math.Abs(square) < eps)
                    {
                        TabOrien.Append(1);
                    }
                    else
                    {
                        TabOrien.Append(((square < 0.0) ? 1 : 0));
                    }

                    if (FlecheU < Toluv)
                    {
                        FlecheU = Toluv;
                    }

                    if (FlecheV < Toluv)
                    {
                        FlecheV = Toluv;
                    }

                    TabClass.Append(new CSLib_Class2d(PClass, FlecheU, FlecheV, Umin, Vmin, Umax, Vmax));
                } // if(nbpoints>3
                else
                {
                    anIsBadWire = true;
                    TabOrien.Append(-1);
                    var xPClass = new NCollection_Array1<gp_Pnt2d>(1, 2);
                    xPClass[1] = SeqPnt2d[1];
                    xPClass[2] = SeqPnt2d[2];
                    TabClass.Append(new CSLib_Class2d(xPClass, FlecheU, FlecheV, Umin, Vmin, Umax, Vmax));
                }
            } // else if(WareIsNotEmpty
        } // for(FaceExplorer

        int nbtabclass = TabClass.Length();

        if (nbtabclass > 0)
        {
            //-- If an error was detected on a wire: set all TabOrien to -1
            if (anIsBadWire)
            {
                TabOrien[1] = -1;
            }

            if (surf.GetType() == GeomAbs_Cone || surf.GetType() == GeomAbs_Cylinder
                || surf.GetType() == GeomAbs_Torus || surf.GetType() == GeomAbs_Sphere
                || surf.GetType() == GeomAbs_SurfaceOfRevolution)
            {
                U1 = 0.0; // modified by NIZHNY-OFV  Thu May 31 14:24:10 2001 ---> //Umin-uuu*0.5;
                U2 = 2 * M_PI; // modified by NIZHNY-OFV  Thu May 31 14:24:35 2001 ---> //U1+M_PI+M_PI;
            }
            else
            {
                U1 = U2 = 0.0;
            }

            if (surf.GetType() == GeomAbs_Torus)
            {
                V1 = 0.0; // modified by NIZHNY-OFV  Thu May 31 14:24:55 2001 ---> //Vmin-uuu*0.5;
                V2 = 2 * M_PI; // modified by NIZHNY-OFV  Thu May 31 14:24:59 2001 ---> //V1+M_PI+M_PI;
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

        gp_Pnt2d P = new(Umin - (Umax - Umin), Vmin - (Vmax - Vmin));
        return (Perform(P, false));
    }

    /// <summary>
    /// The state of the point <paramref name="_Puv"/> in the face. On a periodic surface the point is also tried at
    /// its periods within the face's UV box, if <paramref name="RecadreOnPeriodic"/>.
    /// </summary>
    public TopAbs_State Perform(in gp_Pnt2d _Puv, bool RecadreOnPeriodic = true)
        => Classify(_Puv, Toluv, RecadreOnPeriodic, false);

    /// <summary>
    /// The state of the point <paramref name="_Puv"/> tested with an offset of <paramref name="Tol"/>: ON where the
    /// polygons can't decide (internal use).
    /// </summary>
    public TopAbs_State TestOnRestriction(in gp_Pnt2d _Puv, double Tol, bool RecadreOnPeriodic = true)
        => Classify(_Puv, Tol, RecadreOnPeriodic, true);

    // Perform and TestOnRestriction, which OCCT writes out twice: they differ in the polygon test (SiDans or
    // SiDans_OnMode with Tol), in what an undecided polygon gives (the face classifier with Toluv capped at 4, or ON)
    // and in the face classifier's tolerance for a bad wire (Toluv or Tol).
    private TopAbs_State Classify(in gp_Pnt2d _Puv, double Tol, bool RecadreOnPeriodic, bool theOnMode)
    {
        int dedans;
        int nbtabclass = TabClass.Length();

        if (nbtabclass == 0)
        {
            return (TopAbs_IN);
        }

        //-- U1 is the First Param and U2 in this case is U1+Period
        double u = _Puv.X();
        double v = _Puv.Y();
        double uu = u, vv = v;

        var surf = new BRepAdaptor_Surface();
        surf.Initialize(Face, false);
        bool IsUPer = surf.IsUPeriodic();
        bool IsVPer = surf.IsVPeriodic();
        double uperiod = IsUPer ? surf.UPeriod() : 0.0;
        double vperiod = IsVPer ? surf.VPeriod() : 0.0;
        TopAbs_State aStatus = TopAbs_UNKNOWN;
        bool urecadre = false, vrecadre = false;

        if (RecadreOnPeriodic)
        {
            if (IsUPer)
            {
                if (uu < Umin)
                {
                    while (uu < Umin)
                    {
                        uu += uperiod;
                    }
                }
                else
                {
                    while (uu >= Umin)
                    {
                        uu -= uperiod;
                    }

                    uu += uperiod;
                }
            }

            if (IsVPer)
            {
                if (vv < Vmin)
                {
                    while (vv < Vmin)
                    {
                        vv += vperiod;
                    }
                }
                else
                {
                    while (vv >= Vmin)
                    {
                        vv -= vperiod;
                    }

                    vv += vperiod;
                }
            }
        }

        for (;;)
        {
            dedans = 1;
            gp_Pnt2d Puv = new(u, v);

            if (TabOrien[1] != -1)
            {
                for (int n = 1; n <= nbtabclass; n++)
                {
                    int cur = theOnMode ? (int)TabClass[n].SiDans_OnMode(Puv, Tol) : (int)TabClass[n].SiDans(Puv);
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
                    if (theOnMode)
                    {
                        aStatus = TopAbs_ON;
                    }
                    else
                    {
                        var aClassifier = new BRepClass_FaceClassifier();
                        double m_Toluv = (Toluv > 4.0) ? 4.0 : Toluv;
                        // aClassifier.Perform(Face,Puv,Toluv);
                        aClassifier.Perform(Face, Puv, m_Toluv);
                        aStatus = aClassifier.State();
                    }
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
            {
                //-- TabOrien(1)=-1    False Wire
                var aClassifier = new BRepClass_FaceClassifier();
                aClassifier.Perform(Face, Puv, theOnMode ? Tol : Toluv);
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

    /// <summary>OCCT's Destroy: clears the polygons, after which every point is IN.</summary>
    internal void Destroy() => TabClass.Clear();
}
