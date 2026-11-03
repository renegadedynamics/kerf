// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntPatch_IType;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The intersection of two implicit surfaces (planes, cylinders, cones, spheres and tori) bounded by their domains:
/// lines (conics, analytic curves, restrictions or, for two cylinders without a geometric solution, walking lines)
/// and isolated points.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's class. OCCT's .cxx is split into partial files by subject: this one
/// (the class and Perform), Boundaries and Restrictions (the points and segments of the domains put on the lines),
/// Plane (plane/quadric pairs), CyCy (cylinder/cylinder, with its walking lines) and Quadrics (the other pairs and
/// the tori). <see cref="Point"/> returns a copy (OCCT's const reference).
/// </remarks>
public sealed partial class IntPatch_ImpImpIntersection
{
    /// <summary>The status of an intersection.</summary>
    public enum IntStatus
    {
        /// <summary>OK. Good intersection result.</summary>
        IntStatus_OK,

        /// <summary>
        /// The intersection curve is too long (as in OCCT's bug 26894): its value and derivatives can't be computed
        /// precisely enough in IEEE 754 doubles, so OCCT's algorithms can't work with it correctly.
        /// </summary>
        IntStatus_InfiniteSectionCurve,

        /// <summary>The algorithm cannot finish correctly.</summary>
        IntStatus_Fail,
    }

    private IntStatus myDone;
    private bool empt;
    private bool tgte;
    private bool oppo;
    private readonly NCollection_Sequence<IntPatch_Point> spnt = new();
    private readonly NCollection_Sequence<IntPatch_Line> slin = new();
    private readonly IntPatch_TheSOnBounds solrst = new();

    /// <summary>An empty intersection (not done).</summary>
    public IntPatch_ImpImpIntersection()
    {
        myDone = IntStatus.IntStatus_Fail;
        empt = true;
        tgte = false;
        oppo = false;
    }

    /// <summary>
    /// Intersects <paramref name="S1"/> bounded by <paramref name="D1"/> with <paramref name="S2"/> bounded by
    /// <paramref name="D2"/>. With <paramref name="theIsReqToKeepRLine"/> (kept for TopOpeBRep), a restriction line
    /// is kept beside another line even where they coincide.
    /// </summary>
    public IntPatch_ImpImpIntersection(Adaptor3d_Surface S1,
                                       Adaptor3d_TopolTool D1,
                                       Adaptor3d_Surface S2,
                                       Adaptor3d_TopolTool D2,
                                       double TolArc,
                                       double TolTang,
                                       bool theIsReqToKeepRLine = false)
    {
        Perform(S1, D1, S2, D2, TolArc, TolTang, theIsReqToKeepRLine);
    }

    /// <summary>
    /// Intersects <paramref name="S1"/> bounded by <paramref name="D1"/> with <paramref name="S2"/> bounded by
    /// <paramref name="D2"/>; see the constructor.
    /// </summary>
    /// <exception cref="ConstructionErrorException">A surface is not a plane, cylinder, cone, sphere or torus.</exception>
    public void Perform(Adaptor3d_Surface S1,
                        Adaptor3d_TopolTool D1,
                        Adaptor3d_Surface S2,
                        Adaptor3d_TopolTool D2,
                        double TolArc,
                        double TolTang,
                        bool theIsReqToKeepRLine = false)
    {
        myDone = IntStatus.IntStatus_Fail;
        spnt.Clear();
        slin.Clear();

        bool isPostProcessingRequired = true;

        empt = true;
        tgte = false;
        oppo = false;

        bool all1 = false;
        bool all2 = false;
        bool SameSurf = false;
        bool multpoint = false;

        bool nosolonS1 = false;
        // indique s il y a des points sur restriction du carreau 1
        bool nosolonS2 = false;
        // indique s il y a des points sur restriction du carreau 2
        int i, nbpt, nbseg;
        NCollection_Sequence<IntPatch_TheSegmentOfTheSOnBounds> edg1 = new(), edg2 = new();
        NCollection_Sequence<IntPatch_ThePathPointOfTheSOnBounds> pnt1 = new(), pnt2 = new();
        //
        // On commence par intersecter les supports des surfaces
        IntSurf_Quadric quad1 = new(), quad2 = new();
        IntPatch_ArcFunction AFunc = new();
        const double Tolang = 1.0e-8;
        SurfaceKind typs1, typs2;
        bool bEmpty = false;
        //
        int iT1 = SetQuad(S1, out typs1, ref quad1);
        int iT2 = SetQuad(S2, out typs2, ref quad2);
        //
        if (iT1 == 0 || iT2 == 0)
        {
            throw new ConstructionErrorException();
        }
        //
        bool bReverse = iT1 > iT2;
        int iTT = iT1 * 10 + iT2;
        //
        switch (iTT)
        {
            case 11:
                { // Plane/Plane
                    if (!IntPP(quad1, quad2, Tolang, TolTang, ref SameSurf, slin))
                    {
                        return;
                    }
                    break;
                }
            //
            case 12:
            case 21:
                { // Plane/Cylinder
                    double VMin, VMax, H;
                    //
                    Adaptor3d_Surface aSCyl = bReverse ? S1 : S2;
                    VMin = aSCyl.FirstVParameter();
                    VMax = aSCyl.LastVParameter();
                    H = (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMin) || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax))
                          ? 0
                          : (VMax - VMin);
                    //
                    if (!IntPCy(quad1, quad2, Tolang, TolTang, bReverse, ref empt, slin, H))
                    {
                        return;
                    }
                    bEmpty = empt;
                    break;
                }
            //
            case 13:
            case 31:
                { // Plane/Cone
                    if (!IntPCo(quad1, quad2, Tolang, TolTang, bReverse, ref empt, ref multpoint, slin, spnt))
                    {
                        return;
                    }
                    bEmpty = empt;
                    break;
                }
            //
            case 14:
            case 41:
                { // Plane/Sphere
                    if (!IntPSp(quad1, quad2, Tolang, TolTang, bReverse, ref empt, slin, spnt))
                    {
                        return;
                    }
                    bEmpty = empt;
                    break;
                }
            //
            case 15:
            case 51:
                { // Plane/Torus
                    if (!IntPTo(quad1, quad2, TolTang, bReverse, ref empt, slin))
                    {
                        return;
                    }
                    bEmpty = empt;
                    break;
                }
            //
            case 22:
                { // Cylinder/Cylinder
                    Box2d aBox1 = new(), aBox2 = new();

                    double aU1f = S1.FirstUParameter();
                    double aU1l = S1.LastUParameter();
                    double aU2f = S2.FirstUParameter();
                    double aU2l = S2.LastUParameter();

                    const double anUperiod = 2.0 * M_PI;

                    if (aU1l - aU1f > anUperiod)
                    {
                        aU1l = aU1f + anUperiod;
                    }

                    if (aU2l - aU2f > anUperiod)
                    {
                        aU2l = aU2f + anUperiod;
                    }

                    aBox1.Add(new Point2D(aU1f, S1.FirstVParameter()));
                    aBox1.Add(new Point2D(aU1l, S1.LastVParameter()));
                    aBox2.Add(new Point2D(aU2f, S2.FirstVParameter()));
                    aBox2.Add(new Point2D(aU2l, S2.LastVParameter()));

                    // Resolution is too big if the cylinder radius is
                    // too small. Therefore, we shall bind its value above.
                    // Here, we use simple constant.
                    double a2DTol = Min(1.0e-4, Min(S1.UResolution(TolTang), S2.UResolution(TolTang)));

                    myDone = IntCyCy(quad1, quad2, TolTang, a2DTol, aBox1, aBox2, ref empt, ref SameSurf,
                                     ref multpoint, slin, spnt);

                    if (myDone == IntStatus.IntStatus_Fail)
                    {
                        return;
                    }

                    bEmpty = empt;
                    if (!slin.IsEmpty())
                    {
                        if (slin.Value(1) is IntPatch_WLine)
                        { // No geometric solution
                            isPostProcessingRequired = false;
                        }
                    }

                    break;
                }
            //
            case 23:
            case 32:
                { // Cylinder/Cone
                    if (!IntCyCo(quad1, quad2, TolTang, bReverse, ref empt, ref multpoint, slin, spnt))
                    {
                        return;
                    }
                    bEmpty = empt;
                    break;
                }
            //
            case 24:
            case 42:
                { // Cylinder/Sphere
                    if (!IntCySp(quad1, quad2, TolTang, bReverse, ref empt, ref multpoint, slin, spnt))
                    {
                        return;
                    }
                    bEmpty = empt;
                    break;
                }
            //
            case 25:
            case 52:
                { // Cylinder/Torus
                    if (!IntCyTo(quad1, quad2, TolTang, bReverse, ref empt, slin))
                    {
                        return;
                    }
                    bEmpty = empt;
                    break;
                }
            //
            case 33:
                { // Cone/Cone
                    if (!IntCoCo(quad1, quad2, TolTang, ref empt, ref SameSurf, ref multpoint, slin, spnt))
                    {
                        return;
                    }
                    bEmpty = empt;
                    break;
                }
            //
            case 34:
            case 43:
                { // Cone/Sphere
                    if (!IntCoSp(quad1, quad2, TolTang, bReverse, ref empt, ref multpoint, slin, spnt))
                    {
                        return;
                    }
                    bEmpty = empt;
                    break;
                }
            //
            case 35:
            case 53:
                { // Cone/Torus
                    if (!IntCoTo(quad1, quad2, TolTang, bReverse, ref empt, slin))
                    {
                        return;
                    }
                    // OCCT: bEmpty is not set from empt here, as it is for every other pair, so an empty cone/torus
                    // intersection goes on to the boundary processing.
                    break;
                }
            //
            case 44:
                { // Sphere/Sphere
                    if (!IntSpSp(quad1, quad2, TolTang, ref empt, ref SameSurf, slin, spnt))
                    {
                        return;
                    }
                    bEmpty = empt;
                    break;
                }
            //
            case 45:
            case 54:
                { // Sphere/Torus
                    if (!IntSpTo(quad1, quad2, TolTang, bReverse, ref empt, slin))
                    {
                        return;
                    }
                    bEmpty = empt;
                    break;
                }
            //
            case 55:
                { // Torus/Torus
                    if (!IntToTo(quad1, quad2, TolTang, ref SameSurf, ref empt, slin))
                    {
                        return;
                    }
                    bEmpty = empt;
                    break;
                }
            //
            default:
                {
                    throw new ConstructionErrorException();
                }
        }
        //
        if (bEmpty)
        {
            if (myDone == IntStatus.IntStatus_Fail)
            {
                myDone = IntStatus.IntStatus_OK;
            }

            return;
        }
        //

        if (isPostProcessingRequired)
        {
            if (!SameSurf)
            {
                AFunc.SetQuadric(quad2);
                AFunc.Set(S1);

                solrst.Perform(AFunc, D1, TolArc, TolTang);
                if (!solrst.IsDone())
                {
                    return;
                }

                if (solrst.AllArcSolution() && typs1 == typs2)
                {
                    all1 = true;
                }
                nbpt = solrst.NbPoints();
                nbseg = solrst.NbSegments();
                for (i = 1; i <= nbpt; i++)
                {
                    pnt1.Append(solrst.Point(i));
                }
                for (i = 1; i <= nbseg; i++)
                {
                    edg1.Append(solrst.Segment(i));
                }
                nosolonS1 = (nbpt == 0) && (nbseg == 0);

                if (nosolonS1 && all1)
                { // cas de face sans restrictions
                    all1 = false;
                }
            } // if (!SameSurf) {
            else
            {
                nosolonS1 = true;
            }

            if (!SameSurf)
            {
                AFunc.SetQuadric(quad1);
                AFunc.Set(S2);

                solrst.Perform(AFunc, D2, TolArc, TolTang);
                if (!solrst.IsDone())
                {
                    return;
                }

                if (solrst.AllArcSolution() && typs1 == typs2)
                {
                    all2 = true;
                }

                nbpt = solrst.NbPoints();
                nbseg = solrst.NbSegments();
                for (i = 1; i <= nbpt; i++)
                {
                    pnt2.Append(solrst.Point(i));
                }

                for (i = 1; i <= nbseg; i++)
                {
                    edg2.Append(solrst.Segment(i));
                }

                nosolonS2 = (nbpt == 0) && (nbseg == 0);

                if (nosolonS2 && all2)
                { // cas de face sans restrictions
                    all2 = false;
                }
            } // if (!SameSurf) {
            else
            {
                nosolonS2 = true;
            }
            //
            if (SameSurf || (all1 && all2))
            {
                // faces "paralleles" parfaites
                empt = false;
                tgte = true;
                slin.Clear();
                spnt.Clear();

                Point3D Ptreference = new();

                switch (typs1)
                {
                    case Plane:
                        {
                            Ptreference = (S1.Plane()).Location();
                        }
                        break;
                    case SurfaceKind.Cylinder:
                        {
                            Ptreference = global::Kerf.Engine.Geometry.ElSLib.Value(0.0, 0.0, S1.Cylinder());
                        }
                        break;
                    case SurfaceKind.Sphere:
                        {
                            Ptreference = global::Kerf.Engine.Geometry.ElSLib.Value(M_PI / 4.0, M_PI / 4.0, S1.Sphere());
                        }
                        break;
                    case SurfaceKind.Cone:
                        {
                            Ptreference = global::Kerf.Engine.Geometry.ElSLib.Value(0.0, 10.0, S1.Cone());
                        }
                        break;
                    case SurfaceKind.Torus:
                        {
                            Ptreference = global::Kerf.Engine.Geometry.ElSLib.Value(0.0, 0.0, S1.Torus());
                        }
                        break;
                    default:
                        break;
                }
                //
                oppo = quad1.Normale(Ptreference).Dot(quad2.Normale(Ptreference)) < 0.0;
                myDone = IntStatus.IntStatus_OK;
                return;
            } // if (SameSurf || (all1 && all2)) {

            if (!nosolonS1 || !nosolonS2)
            {
                empt = false;
                // C est la qu il faut commencer a bosser...
                PutPointsOnLine(S1, S2, pnt1, slin, true, D1, quad1, quad2, multpoint, TolArc);

                PutPointsOnLine(S1, S2, pnt2, slin, false, D2, quad2, quad1, multpoint, TolArc);

                if (edg1.Length() != 0)
                {
                    ProcessSegments(edg1, slin, quad1, quad2, true, TolArc);
                }

                if (edg2.Length() != 0)
                {
                    ProcessSegments(edg2, slin, quad1, quad2, false, TolArc);
                }

                if (edg1.Length() != 0 || edg2.Length() != 0)
                {
                    ProcessRLine(slin, quad1, quad2, TolArc, theIsReqToKeepRLine);
                }
            } // if (!nosolonS1 || !nosolonS2) {
            else
            {
                empt = ((slin.Length() == 0) && (spnt.Length() == 0));
            }
        }

        int nblin = slin.Length(), aNbPnt = spnt.Length();
        //
        // modified by NIZNHY-PKV Tue Sep 06 10:03:35 2011f
        if (aNbPnt != 0)
        {
            NCollection_Sequence<IntPatch_Point> aSIP = new();
            //
            for (i = 1; i <= aNbPnt; ++i)
            {
                Point2D aP2D;
                TopAbs_State aState1, aState2;
                //
                IntPatch_Point aIP = spnt[i];
                aIP.Parameters(out double aU1, out double aV1, out double aU2, out double aV2);
                //
                aP2D = new Point2D(aU1, aV1);
                aState1 = D1.Classify(aP2D, TolArc);
                //
                aP2D = new Point2D(aU2, aV2);
                aState2 = D2.Classify(aP2D, TolArc);
                //
                if (aState1 != TopAbs_OUT && aState2 != TopAbs_OUT)
                {
                    aSIP.Append(aIP);
                }
            }
            //
            spnt.Clear();
            //
            aNbPnt = aSIP.Length();
            for (i = 1; i <= aNbPnt; ++i)
            {
                spnt.Append(aSIP[i]);
            }
            //
        } //  if (aNbPnt) {
        // modified by NIZNHY-PKV Tue Sep 06 10:18:20 2011t
        //
        for (i = 1; i <= nblin; i++)
        {
            IntPatch_IType thetype = slin.Value(i).ArcType();
            if ((thetype == IntPatch_Ellipse) || (thetype == IntPatch_Circle) || (thetype == IntPatch_Lin)
                || (thetype == IntPatch_Parabola) || (thetype == IntPatch_Hyperbola))
            {
                ((IntPatch_GLine)slin.Value(i)).ComputeVertexParameters(TolArc);
            }
            else if (thetype == IntPatch_Analytic)
            {
                ((IntPatch_ALine)slin.Value(i)).ComputeVertexParameters(TolArc);
            }
            else if (thetype == IntPatch_Restriction)
            {
                ((IntPatch_RLine)slin.Value(i)).ComputeVertexParameters(TolArc);
            }
        }
        //
        //----------------------------------------------------------------
        //-- On place 2 vertex sur les courbes de GLine qui n en
        //-- contiennent pas.
        for (i = 1; i <= nblin; i++)
        {
            Point3D P;
            IntPatch_Point point = new();
            double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
            if (slin.Value(i).ArcType() == IntPatch_Circle)
            {
                IntPatch_GLine glin = (IntPatch_GLine)slin.Value(i);
                if (glin.NbVertex() == 0)
                {
                    Circle3D Circ = glin.Circle();
                    P = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, Circ);
                    quad1.Parameters(P, ref u1, ref v1);
                    quad2.Parameters(P, ref u2, ref v2);
                    point.SetValue(P, TolArc, false);
                    point.SetParameters(u1, v1, u2, v2);
                    point.SetParameter(0.0);
                    glin.AddVertex(point);

                    P = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, Circ);
                    quad1.Parameters(P, ref u1, ref v1);
                    quad2.Parameters(P, ref u2, ref v2);
                    point.SetValue(P, TolArc, false);
                    point.SetParameters(u1, v1, u2, v2);
                    point.SetParameter(M_PI + M_PI);
                    glin.AddVertex(point);
                }
            }

            else if (slin.Value(i).ArcType() == IntPatch_Ellipse)
            {
                IntPatch_GLine glin = (IntPatch_GLine)slin.Value(i);
                if (glin.NbVertex() == 0)
                {
                    Ellipse3D Elips = glin.Ellipse();
                    P = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, Elips);
                    quad1.Parameters(P, ref u1, ref v1);
                    quad2.Parameters(P, ref u2, ref v2);
                    point.SetValue(P, TolArc, false);
                    point.SetParameters(u1, v1, u2, v2);
                    point.SetParameter(0.0);
                    glin.AddVertex(point);

                    P = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, Elips);
                    quad1.Parameters(P, ref u1, ref v1);
                    quad2.Parameters(P, ref u2, ref v2);
                    point.SetValue(P, TolArc, false);
                    point.SetParameters(u1, v1, u2, v2);
                    point.SetParameter(M_PI + M_PI);
                    glin.AddVertex(point);
                }
            }
        }
        myDone = IntStatus.IntStatus_OK;
    }

    /// <summary>Returns true if the computation was successful.</summary>
    public bool IsDone() => (GetStatus() != IntStatus.IntStatus_Fail);

    /// <summary>Returns the status.</summary>
    public IntStatus GetStatus() => myDone;

    /// <summary>Returns true if there is no intersection.</summary>
    public bool IsEmpty()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return empt;
    }

    /// <summary>
    /// Returns true if the two patches are entirely tangent: every restriction arc of one patch is inside the
    /// geometric base of the other.
    /// </summary>
    public bool TangentFaces()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return tgte;
    }

    /// <summary>
    /// Returns true when <see cref="TangentFaces"/> is true and the normals at a point of the two surfaces are
    /// opposite.
    /// </summary>
    /// <exception cref="DomainErrorException"><see cref="TangentFaces"/> is false.</exception>
    public bool OppositeFaces()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        if (!tgte)
        {
            throw new DomainErrorException();
        }
        return oppo;
    }

    /// <summary>Returns the number of "single" points.</summary>
    public int NbPnts()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return spnt.Length();
    }

    /// <summary>Returns the point of range <paramref name="Index"/>.</summary>
    public IntPatch_Point Point(int Index)
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return spnt.Value(Index);
    }

    /// <summary>Returns the number of intersection lines.</summary>
    public int NbLines()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return slin.Length();
    }

    /// <summary>Returns the line of range <paramref name="Index"/>.</summary>
    public IntPatch_Line Line(int Index)
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return slin.Value(Index);
    }

    //=================================================================================================

    private static int SetQuad(Adaptor3d_Surface theS, out SurfaceKind theTS, ref IntSurf_Quadric theQuad)
    {
        theTS = theS.GetType();
        int iRet = 0;
        switch (theTS)
        {
            case Plane:
                theQuad.SetValue(theS.Plane());
                iRet = 1;
                break;
            case SurfaceKind.Cylinder:
                theQuad.SetValue(theS.Cylinder());
                iRet = 2;
                break;
            case SurfaceKind.Cone:
                theQuad.SetValue(theS.Cone());
                iRet = 3;
                break;
            case SurfaceKind.Sphere:
                theQuad.SetValue(theS.Sphere());
                iRet = 4;
                break;
            case SurfaceKind.Torus:
                theQuad.SetValue(theS.Torus());
                iRet = 5;
                break;
            default:
                break;
        }
        //
        return iRet;
    }
}
