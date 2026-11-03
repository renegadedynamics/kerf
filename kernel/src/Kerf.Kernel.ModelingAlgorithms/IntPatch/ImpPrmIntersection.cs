// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpPrmIntersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpPrmIntersection.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpPrmIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The intersection of a quadric (plane, cylinder, sphere, cone) with a parametric surface: start points on the
/// parametric surface's restrictions and inside it, walking lines between them (IntPatch_TheIWalking), the
/// restrictions lying on the quadric as restriction lines, then the vertices on the quadric's restrictions and, for
/// cones, spheres and cylinders, the lines split at seams and poles.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: <see cref="Perform"/> fills it, and its lines are the caller's to edit. The file-static
/// functions of OCCT's .cxx are private static methods (IntPatch_ImpPrmIntersection.Tools.cs). The quadric is built
/// before the functions are given it and never changed afterwards, so their copies of it (decision 608) see what
/// OCCT's pointers see.
/// </remarks>
public partial class ImpPrmIntersection
{
    private bool _done;
    private bool _empt;
    private readonly NCollection_Sequence<PatchPoint> _spnt = new();
    private readonly NCollection_Sequence<PatchLine> _slin = new();
    private readonly TheSOnBounds _solrst = new();
    private readonly PatchTheSearchInside _solins = new();
    private bool _isStartPnt;
    private double _uStart;
    private double _vStart;

    public ImpPrmIntersection()
    {
        _done = false;
        _empt = false;
        _isStartPnt = false;
        _uStart = 0.0;
        _vStart = 0.0;
    }

    public ImpPrmIntersection(SurfaceAdaptorBase surf1, TopolTool d1, SurfaceAdaptorBase surf2,
                                       TopolTool d2, double tolArc, double tolTang, double fleche, double pas)
        : this()
    {
        Perform(surf1, d1, surf2, d2, tolArc, tolTang, fleche, pas);
    }

    /// <summary>Searches the solution from the given point of the parametric surface.</summary>
    public void SetStartPoint(double u, double v)
    {
        _isStartPnt = true;
        _uStart = u;
        _vStart = v;
    }

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone => _done;

    /// <summary>True if there is no intersection. Throws StdFail_NotDone if not done.</summary>
    public bool IsEmpty
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _empt;
        }
    }

    /// <summary>The number of single points. Throws StdFail_NotDone if not done.</summary>
    public int PntCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _spnt.Length();
        }
    }

    /// <summary>The single point <paramref name="index"/>. Throws StdFail_NotDone if not done.</summary>
    public PatchPoint Point(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _spnt.Value(index);
    }

    /// <summary>The number of lines. Throws StdFail_NotDone if not done.</summary>
    public int LineCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _slin.Length();
        }
    }

    /// <summary>The line <paramref name="index"/>. Throws StdFail_NotDone if not done.</summary>
    public PatchLine Line(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _slin.Value(index);
    }

    /// <summary>
    /// Computes the intersection. One of the surfaces must be a plane, a cylinder, a sphere or a cone (else
    /// Standard_ConstructionError); D1 and D2 are their domains; TolArc the tolerance on the restrictions, TolTang
    /// the tangency tolerance, Fleche the deflection and Pas the step of the walking.
    /// </summary>
    public void Perform(SurfaceAdaptorBase surf1, TopolTool d1, SurfaceAdaptorBase surf2,
                        TopolTool d2, double tolArc, double tolTang, double fleche, double pas)
    {
        bool reversed, procf, procl, dofirst, dolast;
        int indfirst = 0, indlast = 0, ind2, nbSegm;
        int nbPointIns, nbPointRst, nblines, nbpts, nbPointDep;
        double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0, paramf, paraml, currentparam;

        TheSegmentOfTheSOnBounds thesegm;
        PathPoint pPoint;

        RLine rline;
        WLine wline;
        ThePathPointOfTheSOnBounds pStartf = new(), pStartl = new();
        PatchPoint ptdeb = new(), ptfin = new(), ptbis;

        PatchLineType typ;
        WalkingTransition tLine = new(), tArc = new();
        WalkingTypeTrans trans1, trans2;
        Point3D valpt;
        Vector3D tgline, tgrst = new(), norm1, norm2, d1u = new(), d1v = new();
        Direction3D dirNormale;
        Vector3D vecNormale;

        Point2D p2d;
        Vector2D d2d;

        Curve2DAdaptorBase currentarc;
        SurfaceKind typeS1, typeS2;
        SurfaceQuadric quad = new();
        TheSurfFunction func = new();
        PatchArcFunction aFunc = new();
        //
        typeS1 = surf1.GetType();
        typeS2 = surf2.GetType();

        paramf = 0.0;
        paraml = 0.0;
        trans1 = WalkingTypeTrans.Undecided;
        trans2 = WalkingTypeTrans.Undecided;
        //
        _done = false;
        _empt = true;
        _slin.Clear();
        _spnt.Clear();
        //
        reversed = false;
        switch (typeS1)
        {
            case Plane:
                quad.SetValue(surf1.Plane());
                break;

            case SurfaceKind.Cylinder:
                quad.SetValue(surf1.Cylinder());
                break;

            case SurfaceKind.Sphere:
                quad.SetValue(surf1.Sphere());
                break;

            case SurfaceKind.Cone:
                quad.SetValue(surf1.Cone());
                break;

            default:
            {
                reversed = true;
                switch (typeS2)
                {
                    case Plane:
                        quad.SetValue(surf2.Plane());
                        break;

                    case SurfaceKind.Cylinder:
                        quad.SetValue(surf2.Cylinder());
                        break;

                    case SurfaceKind.Sphere:
                        quad.SetValue(surf2.Sphere());
                        break;

                    case SurfaceKind.Cone:
                        quad.SetValue(surf2.Cone());
                        break;
                    default:
                        throw new ConstructionErrorException();
                }
            }
            break;
        }
        //
        double localPas;
        if (reversed)
        {
            localPas = GetLocalStep(surf1, pas);
        }
        else
        {
            localPas = GetLocalStep(surf2, pas);
        }

        func.SetImplicitSurface(quad);
        func.Set(QuadricTool.Tolerance(quad));
        aFunc.SetQuadric(quad);
        //
        if (!reversed)
        {
            func.Set(surf2);
            aFunc.Set(surf2);
        }
        else
        {
            func.Set(surf1);
            aFunc.Set(surf1);
        }
        //
        if (!reversed)
        {
            _solrst.Perform(aFunc, d2, tolArc, tolTang);
        }
        else
        {
            _solrst.Perform(aFunc, d1, tolArc, tolTang);
        }
        if (!_solrst.IsDone)
        {
            return;
        }
        //
        NCollection_Sequence<PathPoint> seqpdep = new();
        NCollection_Sequence<InteriorPoint> seqpins = new();
        //
        nbPointRst = _solrst.PointCount;
        NCollection_Array1<int> destination = new(1, nbPointRst + 1);
        destination.Init(0);
        if (nbPointRst != 0)
        {
            if (!reversed)
            {
                ComputeTangency(_solrst, seqpdep, d2, func, surf2, destination);
            }
            else
            {
                ComputeTangency(_solrst, seqpdep, d1, func, surf1, destination);
            }
        }
        //
        bool searchIns = true;
        if (quad.TypeQuadric() == Plane && _solrst.SegmentCount > 0)
        {
            // For such kind of cases it is possible that whole surface is on one side of plane,
            // plane only touches surface and does not cross it,
            // so no inner points exist.
            searchIns = false;
            TopolTool t;
            if (reversed)
            {
                t = d1;
            }
            else
            {
                t = d2;
            }
            int nbSamples = t.NbSamples();
            math_Vector valf = new(1, 1), uVap = new(1, 2);
            t.SamplePoint(1, out Point2D s2d, out _);
            uVap[1] = s2d.X;
            uVap[2] = s2d.Y;
            func.Value(uVap, valf);
            double rvalf = Math.CopySign(1.0, valf[1]);
            for (int i = 2; i <= nbSamples; ++i)
            {
                t.SamplePoint(i, out s2d, out _);
                uVap[1] = s2d.X;
                uVap[2] = s2d.Y;
                func.Value(uVap, valf);
                if (rvalf * valf[1] < 0.0)
                {
                    searchIns = true;
                    break;
                }
            }
        }
        // Recherche des points interieurs
        nbPointIns = 0;
        if (searchIns)
        {
            if (!reversed)
            {
                if (_isStartPnt)
                {
                    _solins.Perform(func, surf2, _uStart, _vStart);
                }
                else
                {
                    _solins.Perform(func, surf2, d2, tolTang);
                }
            }
            else
            {
                if (_isStartPnt)
                {
                    _solins.Perform(func, surf1, _uStart, _vStart);
                }
                else
                {
                    _solins.Perform(func, surf1, d1, tolTang);
                }
            }
            nbPointIns = _solins.PointCount;
            for (int i = 1; i <= nbPointIns; i++)
            {
                seqpins.Append(_solins.Value(i));
            }
        }
        //
        nbPointDep = seqpdep.Length();
        //
        if (nbPointDep != 0 || nbPointIns != 0)
        {
            PatchTheIWalking iwalk = new(tolTang, fleche, localPas);
            iwalk.Perform(seqpdep, seqpins, func, reversed ? surf1 : surf2, reversed);

            if (!iwalk.IsDone)
            {
                return;
            }

            double vmin, vmax, tolV = 1.0e-14;
            if (!reversed)
            { // Surf1 is quadric
                vmin = surf1.FirstVParameter();
                vmax = surf1.LastVParameter();
            }
            else
            { // Surf2 is quadric
                vmin = surf2.FirstVParameter();
                vmax = surf2.LastVParameter();
            }
            //
            nblines = iwalk.LineCount;
            for (int j = 1; j <= nblines; j++)
            {
                PatchTheIWLineOfTheIWalking iwline = iwalk.Value(j);
                LineOn2S thelin = iwline.Line();

                nbpts = thelin.PointCount;
                if (nbpts >= 2)
                {
                    tgline = iwline.TangentVector(out int k);
                    if (k >= 1 && k <= nbpts)
                    {
                    }
                    else
                    {
                        k = nbpts >> 1;
                    }
                    valpt = thelin.Value(k).Value();

                    if (!reversed)
                    {
                        thelin.Value(k).ParametersOnS2(out u2, out v2);
                        norm1 = quad.Normale(valpt);
                        surf2.D1(u2, v2, out _, out d1u, out d1v);
                        norm2 = d1u.Crossed(d1v);
                    }
                    else
                    {
                        thelin.Value(k).ParametersOnS1(out u2, out v2);
                        norm2 = quad.Normale(valpt);
                        surf1.D1(u2, v2, out _, out d1u, out d1v);
                        norm1 = d1u.Crossed(d1v);
                    }
                    if (tgline.DotCross(norm2, norm1) > 0.0)
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }

                    //
                    double anU1 = 0.0, anU2, anV2;

                    SurfaceKind typQuad = quad.TypeQuadric();
                    bool arecadr = false;
                    valpt = thelin.Value(1).Value();
                    quad.Parameters(valpt, ref anU1, ref v1);

                    if ((v1 < vmin) && (vmin - v1 < tolV))
                    {
                        v1 = vmin;
                    }
                    if ((v1 > vmax) && (v1 - vmax < tolV))
                    {
                        v1 = vmax;
                    }

                    if (reversed)
                    {
                        thelin.SetUV(1, false, anU1, v1); //-- on va lire u2,v2
                        thelin.Value(1).ParametersOnS1(out anU2, out anV2);
                    }
                    else
                    {
                        thelin.SetUV(1, true, anU1, v1); //-- on va lire u1,v1
                        thelin.Value(1).ParametersOnS2(out anU2, out anV2);
                    }

                    if (typQuad == SurfaceKind.Cylinder || typQuad == SurfaceKind.Cone || typQuad == SurfaceKind.Sphere)
                    {
                        arecadr = true;
                    }
                    //
                    for (k = 2; k <= nbpts; ++k)
                    {
                        valpt = thelin.Value(k).Value();
                        quad.Parameters(valpt, ref u1, ref v1);
                        //
                        if ((v1 < vmin) && (vmin - v1 < tolV))
                        {
                            v1 = vmin;
                        }
                        if ((v1 > vmax) && (v1 - vmax < tolV))
                        {
                            v1 = vmax;
                        }
                        //
                        if (arecadr)
                        {
                            double cf, twoPI;
                            //
                            cf = 0.0;
                            twoPI = M_PI + M_PI;
                            if ((u1 - anU1) > 1.5 * M_PI)
                            {
                                while ((u1 - anU1) > (1.5 * M_PI + cf * twoPI))
                                {
                                    cf = cf + 1.0;
                                }
                                u1 = u1 - cf * twoPI;
                            }
                            //
                            else
                            {
                                while ((u1 - anU1) < (-1.5 * M_PI - cf * twoPI))
                                {
                                    cf = cf + 1.0;
                                }
                                u1 = u1 + cf * twoPI;
                            }
                        }
                        //
                        if (reversed)
                        {
                            thelin.SetUV(k, false, u1, v1);

                            thelin.Value(k).ParametersOnS1(out u2, out v2);
                            switch (typeS1)
                            {
                                case SurfaceKind.Cylinder:
                                case SurfaceKind.Cone:
                                case SurfaceKind.Sphere:
                                case SurfaceKind.Torus:
                                    while (u2 < (anU2 - 1.5 * M_PI))
                                    {
                                        u2 += M_PI + M_PI;
                                    }
                                    while (u2 > (anU2 + 1.5 * M_PI))
                                    {
                                        u2 -= M_PI + M_PI;
                                    }
                                    break;
                                default:
                                    break;
                            }
                            if (typeS2 == SurfaceKind.Torus)
                            {
                                while (v2 < (anV2 - 1.5 * M_PI))
                                {
                                    v2 += M_PI + M_PI;
                                }
                                while (v2 > (anV2 + 1.5 * M_PI))
                                {
                                    v2 -= M_PI + M_PI;
                                }
                            }
                            thelin.SetUV(k, true, u2, v2);
                        }
                        else
                        {
                            thelin.SetUV(k, true, u1, v1);

                            thelin.Value(k).ParametersOnS2(out u2, out v2);
                            switch (typeS2)
                            {
                                case SurfaceKind.Cylinder:
                                case SurfaceKind.Cone:
                                case SurfaceKind.Sphere:
                                case SurfaceKind.Torus:
                                    while (u2 < (anU2 - 1.5 * M_PI))
                                    {
                                        u2 += M_PI + M_PI;
                                    }
                                    while (u2 > (anU2 + 1.5 * M_PI))
                                    {
                                        u2 -= M_PI + M_PI;
                                    }
                                    break;
                                default:
                                    break;
                            }
                            if (typeS2 == SurfaceKind.Torus)
                            {
                                while (v2 < (anV2 - 1.5 * M_PI))
                                {
                                    v2 += M_PI + M_PI;
                                }
                                while (v2 > (anV2 + 1.5 * M_PI))
                                {
                                    v2 -= M_PI + M_PI;
                                }
                            }
                            thelin.SetUV(k, false, u2, v2);
                        }

                        anU1 = u1;
                        anU2 = u2;
                        anV2 = v2;
                    }
                    // <-A
                    wline = new WLine(thelin, false, trans1, trans2);
                    wline.SetCreatingWayInfo(WLine.IntPatch_WLType.WLImpPrm);

                    if (iwline.HasFirstPoint && !iwline.IsTangentAtBegining)
                    {
                        indfirst = iwline.FirstPointIndex();
                        pPoint = seqpdep.Value(indfirst);
                        tgline = pPoint.Direction3d();
                        int themult = pPoint.Multiplicity();
                        for (int i = nbPointRst; i >= 1; i--)
                        {
                            if (destination.Value(i) == indfirst)
                            {
                                if (!reversed)
                                { //-- typeS1 = Pln || Cyl || Sph || Cone
                                    quad.Parameters(pPoint.Value(), ref u1, ref v1);

                                    if ((v1 < vmin) && (vmin - v1 < tolV))
                                    {
                                        v1 = vmin;
                                    }
                                    if ((v1 > vmax) && (v1 - vmax < tolV))
                                    {
                                        v1 = vmax;
                                    }

                                    pPoint.Parameters(themult, out u2, out v2);
                                    surf2.D1(u2, v2, out _, out d1u, out d1v); //-- @@@@
                                }
                                else
                                { //-- typeS1 != Pln && Cyl && Sph && Cone
                                    quad.Parameters(pPoint.Value(), ref u2, ref v2);

                                    if ((v2 < vmin) && (vmin - v2 < tolV))
                                    {
                                        v2 = vmin;
                                    }
                                    if ((v2 > vmax) && (v2 - vmax < tolV))
                                    {
                                        v2 = vmax;
                                    }

                                    pPoint.Parameters(themult, out u1, out v1);
                                    surf1.D1(u1, v1, out _, out d1u, out d1v); //-- @@@@
                                }

                                vecNormale = d1u.Crossed(d1v);
                                //-- Modif du 27 Septembre 94 (Recadrage des pts U,V)
                                ptdeb.SetValue(pPoint.Value(), tolArc, false);
                                ptdeb.SetParameters(u1, v1, u2, v2);
                                ptdeb.SetParameter(1.0);

                                Recadre(reversed, typeS1, typeS2, ref ptdeb, iwline, 1, u1, v1, u2, v2);

                                currentarc = _solrst.Point(i).Arc();
                                currentparam = _solrst.Point(i).Parameter();
                                currentarc.D1(currentparam, out p2d, out d2d);
                                tgrst = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

                                double squaremagnitudeVecNormale = vecNormale.SquareMagnitude;
                                if (squaremagnitudeVecNormale > 1e-13)
                                {
                                    dirNormale = new Direction3D(vecNormale);
                                    global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(tgline, tgrst, dirNormale, ref tLine, ref tArc);
                                }
                                else
                                {
                                    tLine.SetValue(true, WalkingTypeTrans.Undecided);
                                    tArc.SetValue(true, WalkingTypeTrans.Undecided);
                                }

                                ptdeb.SetArc(reversed, currentarc, currentparam, tLine, tArc);
                                if (!_solrst.Point(i).IsNew())
                                {
                                    ptdeb.SetVertex(reversed, _solrst.Point(i).Vertex());
                                }
                                wline.AddVertex(ptdeb);
                                if (themult == 0)
                                {
                                    wline.SetFirstPoint(wline.VertexCount);
                                }

                                themult--;
                            }
                        }
                    }
                    else if (iwline.IsTangentAtBegining)
                    {
                        Point3D psol = thelin.Value(1).Value();
                        thelin.Value(1).ParametersOnS1(out u1, out v1);
                        thelin.Value(1).ParametersOnS2(out u2, out v2);
                        ptdeb.SetValue(psol, tolArc, true);
                        ptdeb.SetParameters(u1, v1, u2, v2);
                        ptdeb.SetParameter(1.0);
                        wline.AddVertex(ptdeb);
                        wline.SetFirstPoint(wline.VertexCount);
                    }
                    else
                    {
                        Point3D psol = thelin.Value(1).Value();
                        thelin.Value(1).ParametersOnS1(out u1, out v1);
                        thelin.Value(1).ParametersOnS2(out u2, out v2);
                        ptdeb.SetValue(psol, tolArc, false);
                        ptdeb.SetParameters(u1, v1, u2, v2);
                        ptdeb.SetParameter(1.0);
                        wline.AddVertex(ptdeb);
                        wline.SetFirstPoint(wline.VertexCount);
                    }

                    if (iwline.HasLastPoint && !iwline.IsTangentAtEnd)
                    {
                        indlast = iwline.LastPointIndex();
                        pPoint = seqpdep.Value(indlast);
                        tgline = pPoint.Direction3d().Reversed();
                        int themult = pPoint.Multiplicity();
                        for (int i = nbPointRst; i >= 1; i--)
                        {
                            if (destination.Value(i) == indlast)
                            {
                                if (!reversed)
                                {
                                    quad.Parameters(pPoint.Value(), ref u1, ref v1);

                                    if ((v1 < vmin) && (vmin - v1 < tolV))
                                    {
                                        v1 = vmin;
                                    }
                                    if ((v1 > vmax) && (v1 - vmax < tolV))
                                    {
                                        v1 = vmax;
                                    }

                                    pPoint.Parameters(themult, out u2, out v2);
                                    surf2.D1(u2, v2, out _, out d1u, out d1v); //-- @@@@
                                    vecNormale = d1u.Crossed(d1v); //-- @@@@
                                }
                                else
                                {
                                    quad.Parameters(pPoint.Value(), ref u2, ref v2);

                                    if ((v2 < vmin) && (vmin - v2 < tolV))
                                    {
                                        v2 = vmin;
                                    }
                                    if ((v2 > vmax) && (v2 - vmax < tolV))
                                    {
                                        v2 = vmax;
                                    }

                                    pPoint.Parameters(themult, out u1, out v1);
                                    surf1.D1(u1, v1, out _, out d1u, out d1v); //-- @@@@
                                    vecNormale = d1u.Crossed(d1v); //-- @@@@
                                }

                                ptfin.SetValue(pPoint.Value(), tolArc, false);
                                ptfin.SetParameters(u1, v1, u2, v2);
                                ptfin.SetParameter(nbpts);

                                Recadre(reversed, typeS1, typeS2, ref ptfin, iwline, nbpts - 1, u1, v1, u2, v2);

                                currentarc = _solrst.Point(i).Arc();
                                currentparam = _solrst.Point(i).Parameter();
                                currentarc.D1(currentparam, out p2d, out d2d);
                                tgrst = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

                                double squaremagnitudeVecNormale = vecNormale.SquareMagnitude;
                                if (squaremagnitudeVecNormale > 1e-13)
                                {
                                    dirNormale = new Direction3D(vecNormale);
                                    global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(tgline, tgrst, dirNormale, ref tLine, ref tArc);
                                }
                                else
                                {
                                    tLine.SetValue(true, WalkingTypeTrans.Undecided);
                                    tArc.SetValue(true, WalkingTypeTrans.Undecided);
                                }

                                ptfin.SetArc(reversed, currentarc, currentparam, tLine, tArc);
                                if (!_solrst.Point(i).IsNew())
                                {
                                    ptfin.SetVertex(reversed, _solrst.Point(i).Vertex());
                                }
                                wline.AddVertex(ptfin);
                                if (themult == 0)
                                {
                                    wline.SetLastPoint(wline.VertexCount);
                                }

                                themult--;
                            }
                        }
                    }
                    else if (iwline.IsTangentAtEnd)
                    {
                        Point3D psol = thelin.Value(nbpts).Value();
                        thelin.Value(nbpts).ParametersOnS1(out u1, out v1);
                        thelin.Value(nbpts).ParametersOnS2(out u2, out v2);
                        ptfin.SetValue(psol, tolArc, true);
                        ptfin.SetParameters(u1, v1, u2, v2);
                        ptfin.SetParameter(nbpts);
                        wline.AddVertex(ptfin);
                        wline.SetLastPoint(wline.VertexCount);
                    }
                    else
                    {
                        Point3D psol = thelin.Value(nbpts).Value();
                        thelin.Value(nbpts).ParametersOnS1(out u1, out v1);
                        thelin.Value(nbpts).ParametersOnS2(out u2, out v2);
                        ptfin.SetValue(psol, tolArc, false);
                        ptfin.SetParameters(u1, v1, u2, v2);
                        ptfin.SetParameter(nbpts);
                        wline.AddVertex(ptfin);
                        wline.SetLastPoint(wline.VertexCount);
                    }
                    //
                    // Il faut traiter les points de passage.
                    _slin.Append(wline);
                } // if(Nbpts>=2) {
            } // for (j=1; j<=Nblines; j++) {

            // ON GERE LES RACCORDS ENTRE LIGNES. ELLE NE PEUVENT SE RACCORDER
            // QUE SUR DES POINTS DE TANGENCE

            nblines = _slin.Length();
            for (int j = 1; j <= nblines - 1; j++)
            {
                dofirst = dolast = false;
                WLine wlin1 = (WLine)_slin.Value(j);
                if (wlin1.HasFirstPoint)
                {
                    ptdeb = wlin1.FirstPoint(out indfirst);
                    if (ptdeb.IsTangencyPoint)
                    {
                        dofirst = true;
                    }
                }
                if (wlin1.HasLastPoint)
                {
                    ptfin = wlin1.LastPoint(out indlast);
                    if (ptfin.IsTangencyPoint)
                    {
                        dolast = true;
                    }
                }

                if (dofirst || dolast)
                {
                    for (int k = j + 1; k <= nblines; k++)
                    {
                        WLine wlin2 = (WLine)_slin.Value(k);
                        if (wlin2.HasFirstPoint)
                        {
                            ptbis = wlin2.FirstPoint(out ind2);
                            if (ptbis.IsTangencyPoint)
                            {
                                if (dofirst)
                                {
                                    if (ptdeb.Value().DistanceTo(ptbis.Value()) <= tolArc)
                                    {
                                        ptdeb.SetMultiple(true);
                                        if (!ptbis.IsMultiple)
                                        {
                                            ptbis.SetMultiple(true);
                                            wlin2.Replace(ind2, ptbis);
                                        }
                                    }
                                }
                                if (dolast)
                                {
                                    if (ptfin.Value().DistanceTo(ptbis.Value()) <= tolArc)
                                    {
                                        ptfin.SetMultiple(true);
                                        if (!ptbis.IsMultiple)
                                        {
                                            ptbis.SetMultiple(true);
                                            wlin2.Replace(ind2, ptbis);
                                        }
                                    }
                                }
                            }
                        }
                        if (wlin2.HasLastPoint)
                        {
                            ptbis = wlin2.LastPoint(out ind2);
                            if (ptbis.IsTangencyPoint)
                            {
                                if (dofirst)
                                {
                                    if (ptdeb.Value().DistanceTo(ptbis.Value()) <= tolArc)
                                    {
                                        ptdeb.SetMultiple(true);
                                        if (!ptbis.IsMultiple)
                                        {
                                            ptbis.SetMultiple(true);
                                            wlin2.Replace(ind2, ptbis);
                                        }
                                    }
                                }
                                if (dolast)
                                {
                                    if (ptfin.Value().DistanceTo(ptbis.Value()) <= tolArc)
                                    {
                                        ptfin.SetMultiple(true);
                                        if (!ptbis.IsMultiple)
                                        {
                                            ptbis.SetMultiple(true);
                                            wlin2.Replace(ind2, ptbis);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if (dofirst)
                    {
                        wlin1.Replace(indfirst, ptdeb);
                    }
                    if (dolast)
                    {
                        wlin1.Replace(indlast, ptfin);
                    }
                }
            }
        } // if (seqpdep.Length() != 0 || seqpins.Length() != 0) {
        //
        // Treatment the segments
        nbSegm = _solrst.SegmentCount;
        if (nbSegm != 0)
        {
            for (int i = 1; i <= nbSegm; i++)
            {
                thesegm = _solrst.Segment(i);
                // Check if segment is degenerated
                if (thesegm.HasFirstPoint && thesegm.HasLastPoint)
                {
                    double tol2 = global::Kerf.Engine.Foundation.Precision.Confusion();
                    tol2 *= tol2;
                    Point3D pf = thesegm.FirstPoint().Value();
                    Point3D pl = thesegm.LastPoint().Value();
                    if (pf.SquareDistanceTo(pl) <= tol2)
                    {
                        // segment can be degenerated - check inner point
                        paramf = thesegm.FirstPoint().Parameter();
                        paraml = thesegm.LastPoint().Parameter();
                        Point2D _p2d = thesegm.Curve().Value(.57735 * paramf + 0.42265 * paraml);
                        Point3D pm;
                        if (reversed)
                        {
                            surf1.D0(_p2d.X, _p2d.Y, out pm);
                        }
                        else
                        {
                            surf2.D0(_p2d.X, _p2d.Y, out pm);
                        }
                        if (pm.SquareDistanceTo(pf) <= tol2)
                        {
                            // Degenerated
                            continue;
                        }
                    }
                }

                //----------------------------------------------------------------------
                // on cree une ligne d intersection contenant uniquement le segment.
                // VOIR POUR LA TRANSITION DE LA LIGNE
                // On ajoute aussi un polygone pour le traitement des intersections
                // entre ligne et restrictions de la surface implicite (PutVertexOnLine)
                //----------------------------------------------------------------------
                //-- Calcul de la transition sur la rline (12 fev 97)
                //-- reversed a le sens de OnFirst
                //--
                dofirst = dolast = false;
                procf = false;
                procl = false;
                WalkingTransition tLineUnk = new(), tArcUnk = new();

                PatchPoint _thepointAtBeg = new();
                PatchPoint _thepointAtEnd = new();

                bool transitionOK = false;

                if (thesegm.HasFirstPoint)
                {
                    double _u1 = 0.0, _v1 = 0.0, _u2 = 0.0, _v2 = 0.0;

                    dofirst = true;
                    pStartf = thesegm.FirstPoint();
                    paramf = pStartf.Parameter();

                    Point2D _p2d = thesegm.Curve().Value(paramf);
                    HVertex? _vtx = null;
                    if (!pStartf.IsNew())
                    {
                        _vtx = pStartf.Vertex();
                    }
                    Point3D _Pp = pStartf.Value();
                    _thepointAtBeg.SetValue(_Pp, pStartf.Tolerance(), false);
                    if (!reversed)
                    { //-- typeS1 = Pln || Cyl || Sph || Cone
                        quad.Parameters(_Pp, ref _u1, ref _v1);
                        _u2 = _p2d.X;
                        _v2 = _p2d.Y;
                    }
                    else
                    { //-- typeS1 != Pln && Cyl && Sph && Cone
                        quad.Parameters(_Pp, ref _u2, ref _v2);
                        _u1 = _p2d.X;
                        _v1 = _p2d.Y;
                    }
                    _thepointAtBeg.SetParameters(_u1, _v1, _u2, _v2);
                    _thepointAtBeg.SetParameter(paramf);
                    if (!pStartf.IsNew())
                    {
                        _thepointAtBeg.SetVertex(reversed, _vtx!);
                    }
                    _thepointAtBeg.SetArc(reversed, thesegm.Curve(), paramf, tLineUnk, tArcUnk);

                    surf1.D1(_u1, _v1, out _, out Vector3D d1u1, out Vector3D d1v1);
                    norm1 = d1u1.Crossed(d1v1);
                    surf2.D1(_u2, _v2, out _, out Vector3D d1u2, out Vector3D d1v2);
                    norm2 = d1u2.Crossed(d1v2);

                    thesegm.Curve().D1(paramf, out _p2d, out Vector2D _d2d);
                    tgline = new Vector3D();
                    if (reversed)
                    {
                        tgline = Vector3D.LinearForm(_d2d.X, d1u1, _d2d.Y, d1v1);
                    }
                    else
                    {
                        tgline = Vector3D.LinearForm(_d2d.X, d1u2, _d2d.Y, d1v2);
                    }
                    _u1 = tgline.DotCross(norm2, norm1);
                    transitionOK = true;
                    if (_u1 > 0.00000001)
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else if (_u1 < -0.00000001)
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }
                    else
                    {
                        transitionOK = false;
                    }
                }
                if (thesegm.HasLastPoint)
                {
                    double _u1 = 0.0, _v1 = 0.0, _u2 = 0.0, _v2 = 0.0;

                    dolast = true;
                    pStartl = thesegm.LastPoint();
                    paraml = pStartl.Parameter();

                    Point2D _p2d = thesegm.Curve().Value(paraml);
                    HVertex? _vtx = null;
                    if (!pStartl.IsNew())
                    {
                        _vtx = pStartl.Vertex();
                    }
                    Point3D _Pp = pStartl.Value();
                    _thepointAtEnd.SetValue(_Pp, pStartl.Tolerance(), false);
                    if (!reversed)
                    { //-- typeS1 = Pln || Cyl || Sph || Cone
                        quad.Parameters(_Pp, ref _u1, ref _v1);
                        _u2 = _p2d.X;
                        _v2 = _p2d.Y;
                    }
                    else
                    { //-- typeS1 != Pln && Cyl && Sph && Cone
                        quad.Parameters(_Pp, ref _u2, ref _v2);
                        _u1 = _p2d.X;
                        _v1 = _p2d.Y;
                    }
                    _thepointAtEnd.SetParameters(_u1, _v1, _u2, _v2);
                    _thepointAtEnd.SetParameter(paraml);
                    if (!pStartl.IsNew())
                    {
                        _thepointAtEnd.SetVertex(reversed, _vtx!);
                    }
                    _thepointAtEnd.SetArc(reversed, thesegm.Curve(), paraml, tLineUnk, tArcUnk);

                    surf1.D1(_u1, _v1, out _, out Vector3D d1u1, out Vector3D d1v1);
                    norm1 = d1u1.Crossed(d1v1);
                    surf2.D1(_u2, _v2, out _, out Vector3D d1u2, out Vector3D d1v2);
                    norm2 = d1u2.Crossed(d1v2);

                    thesegm.Curve().D1(paraml, out _p2d, out Vector2D _d2d);
                    tgline = new Vector3D();
                    if (reversed)
                    {
                        tgline = Vector3D.LinearForm(_d2d.X, d1u1, _d2d.Y, d1v1);
                    }
                    else
                    {
                        tgline = Vector3D.LinearForm(_d2d.X, d1u2, _d2d.Y, d1v2);
                    }
                    _u1 = tgline.DotCross(norm2, norm1);
                    transitionOK = true;
                    if (_u1 > 0.00000001)
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else if (_u1 < -0.00000001)
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }
                    else
                    {
                        transitionOK = false;
                    }
                }
                if (!transitionOK)
                {
                    //-- rline = new IntPatch_RLine (thesegm.Curve(),reversed,false);
                    rline = new RLine(false);
                    if (reversed)
                    {
                        rline.SetArcOnS1(thesegm.Curve());
                    }
                    else
                    {
                        rline.SetArcOnS2(thesegm.Curve());
                    }
                }
                else
                {
                    //-- rline = new IntPatch_RLine (thesegm.Curve(),reversed,false,trans1,trans2);
                    rline = new RLine(false, trans1, trans2);
                    if (reversed)
                    {
                        rline.SetArcOnS1(thesegm.Curve());
                    }
                    else
                    {
                        rline.SetArcOnS2(thesegm.Curve());
                    }
                }

                //------------------------------
                //-- Ajout des points
                //--
                if (thesegm.HasFirstPoint)
                {
                    rline.AddVertex(_thepointAtBeg);
                    rline.SetFirstPoint(rline.VertexCount);
                }

                if (thesegm.HasLastPoint)
                {
                    rline.AddVertex(_thepointAtEnd);
                    rline.SetLastPoint(rline.VertexCount);
                }

                // Polygone sur restriction solution
                if (dofirst && dolast)
                {
                    double prm;
                    Point3D ptpoly;
                    PntOn2S p2s = new();
                    LineOn2S Thelin = new();
                    Curve2DAdaptorBase arcsegm = thesegm.Curve();
                    int nbsample = 100;

                    if (!reversed)
                    {
                        for (int j = 1; j <= nbsample; j++)
                        {
                            prm = paramf + (j - 1) * (paraml - paramf) / (nbsample - 1);
                            arcsegm.D0(prm, out p2d);
                            surf2.D0(p2d.X, p2d.Y, out ptpoly);

                            quad.Parameters(ptpoly, ref u1, ref v1);
                            p2s.SetValue(ptpoly, u1, v1, p2d.X, p2d.Y);
                            Thelin.Add(p2s);
                        }
                    }
                    else
                    {
                        for (int j = 1; j <= nbsample; j++)
                        {
                            prm = paramf + (j - 1) * (paraml - paramf) / (nbsample - 1);
                            arcsegm.D0(prm, out p2d);
                            surf1.D0(p2d.X, p2d.Y, out ptpoly);

                            quad.Parameters(ptpoly, ref u2, ref v2);
                            p2s.SetValue(ptpoly, p2d.X, p2d.Y, u2, v2);
                            Thelin.Add(p2s);
                        }
                    }
                    rline.Add(Thelin);
                }

                if (dofirst || dolast)
                {
                    nblines = _slin.Length();
                    for (int j = 1; j <= nblines; j++)
                    {
                        PatchLine slinj = _slin.Value(j);
                        typ = slinj.ArcType();
                        if (typ == PatchLineType.Walking)
                        {
                            nbpts = ((WLine)slinj).VertexCount;
                        }
                        else
                        {
                            nbpts = ((RLine)slinj).VertexCount;
                        }
                        for (int k = 1; k <= nbpts; k++)
                        {
                            if (typ == PatchLineType.Walking)
                            {
                                ptdeb = ((WLine)slinj).Vertex(k);
                            }
                            else
                            {
                                ptdeb = ((RLine)slinj).Vertex(k);
                            }
                            if (dofirst)
                            {
                                if (ptdeb.Value().DistanceTo(pStartf.Value()) <= tolArc)
                                {
                                    ptdeb.SetMultiple(true);
                                    if (typ == PatchLineType.Walking)
                                    {
                                        ((WLine)slinj).Replace(k, ptdeb);
                                    }
                                    else
                                    {
                                        ((RLine)slinj).Replace(k, ptdeb);
                                    }
                                    ptdeb.SetParameter(paramf);
                                    rline.AddVertex(ptdeb);
                                    if (!procf)
                                    {
                                        procf = true;
                                        rline.SetFirstPoint(rline.VertexCount);
                                    }
                                }
                            }
                            if (dolast)
                            {
                                if (dofirst)
                                { //-- on recharge le ptdeb
                                    if (typ == PatchLineType.Walking)
                                    {
                                        ptdeb = ((WLine)slinj).Vertex(k);
                                    }
                                    else
                                    {
                                        ptdeb = ((RLine)slinj).Vertex(k);
                                    }
                                }
                                if (ptdeb.Value().DistanceTo(pStartl.Value()) <= tolArc)
                                {
                                    ptdeb.SetMultiple(true);
                                    if (typ == PatchLineType.Walking)
                                    {
                                        ((WLine)slinj).Replace(k, ptdeb);
                                    }
                                    else
                                    {
                                        ((RLine)slinj).Replace(k, ptdeb);
                                    }
                                    ptdeb.SetParameter(paraml);
                                    rline.AddVertex(ptdeb);
                                    if (!procl)
                                    {
                                        procl = true;
                                        rline.SetLastPoint(rline.VertexCount);
                                    }
                                }
                            }
                        }
                    }
                }
                _slin.Append(rline);
            }
        } // if (NbSegm)
        //
        // on traite les restrictions de la surface implicite

        for (int i = 1, nbLin = _slin.Length(); i <= nbLin; i++)
        {
            PointLine l = (PointLine)_slin.Value(i);

            if (!reversed)
            {
                RstInt.PutVertexOnLine(l, surf1, d1, surf2, true, tolTang);
            }
            else
            {
                RstInt.PutVertexOnLine(l, surf2, d2, surf1, false, tolTang);
            }

            if (l.NbPnts() <= 2)
            {
                bool cond = l.NbPnts() < 2;
                if (!cond)
                {
                    cond = (l.Point(1).IsSame(l.Point(2), global::Kerf.Engine.Foundation.Precision.Confusion()));
                }

                if (cond)
                {
                    _slin.Remove(i);
                    i--;
                    nbLin--;
                    continue;
                }
            }

            if (l.ArcType() == PatchLineType.Walking)
            {
                WLine wl = (WLine)l;
                _slin.Append(wl);
                _slin.Remove(i);
                i--;
                nbLin--;
            }
        }

        // Now slin is filled as follows: lower indices correspond to Restriction line,
        // after (higher indices) - only Walking-line.

        double tol3d = Math.Max(func.Tolerance(), tolTang);
        SurfaceAdaptorBase qSurf = (reversed) ? surf2 : surf1;
        SurfaceAdaptorBase otherSurf = (reversed) ? surf1 : surf2;

        for (int i = 1; i <= _slin.Length(); i++)
        {
            PointLine l1 = (PointLine)_slin.Value(i);
            RLine? rl1 = l1 as RLine;

            if (rl1 is null)
            {
                // Walking-Walking cases are not supported
                break;
            }

            Curve2DAdaptorBase arc = (rl1.IsArcOnS1 ? rl1.ArcOnS1() : rl1.ArcOnS2())!;
            if (arc.GetType() != CurveKind.Line)
            {
                // Restriction line must be isoline.
                // Other cases are not supported by
                // existing algorithms.

                break;
            }

            bool isFirstDeleted = false;

            for (int j = i + 1; j <= _slin.Length(); j++)
            {
                PointLine l2 = (PointLine)_slin.Value(j);
                RLine? rl2 = l2 as RLine;

                // Here aL1 (i-th line) is Restriction-line and aL2 (j-th line) is
                // Restriction or Walking

                if (rl2 is not null)
                {
                    Curve2DAdaptorBase arc2 = (rl2.IsArcOnS1 ? rl2.ArcOnS1() : rl2.ArcOnS2())!;
                    if (arc2.GetType() != CurveKind.Line)
                    {
                        // Restriction line must be isoline.
                        // Other cases are not supported by
                        // existing algorithms.

                        continue;
                    }
                }

                // aDir can be equal to one of following four values only
                //(because Reastriction line is boundary of rectangular surface):
                // either {0, 1} or {0, -1} or {1, 0} or {-1, 0}.
                Direction2D dir = arc.Line().Direction;

                double tol2d = otherSurf.UResolution(tol3d),
                       period = otherSurf.IsVPeriodic ? otherSurf.VPeriod() : 0.0;

                if (Math.Abs(dir.X) < 0.5)
                { // Restriction directs along V-direction
                    tol2d = otherSurf.VResolution(tol3d);
                    period = otherSurf.IsUPeriodic ? otherSurf.UPeriod() : 0.0;
                }

                bool isCoincide = IsCoincide(func, l2, arc, rl1.IsArcOnS1, tol3d, tol2d, period);

                if (isCoincide)
                {
                    if (rl2 is null)
                    { // Delete Walking-line
                        _slin.Remove(j);
                        j--;
                    }
                    else
                    { // Restriction-Restriction
                        Curve2DAdaptorBase arc2 = (rl2.IsArcOnS1 ? rl2.ArcOnS1() : rl2.ArcOnS2())!;

                        double range2 = arc2.LastParameter() - arc2.FirstParameter();
                        double range1 = arc.LastParameter() - arc.FirstParameter();

                        if (range2 > range1)
                        {
                            isFirstDeleted = true;
                            break;
                        }
                        else
                        { // Delete j-th line
                            _slin.Remove(j);
                            j--;
                        }
                    }
                }
            } // for(int j = i + 1; j <= slin.Length(); j++)

            if (isFirstDeleted)
            { // Delete i-th line
                _slin.Remove(i);
                i--;
            }
        } // for (int i = 1; i <= slin.Length(); i++)

        _empt = (_slin.Length() == 0 && _spnt.Length() == 0);
        _done = true;

        if (_slin.Length() == 0)
        {
            return;
        }

        bool isDecomposeRequired = (quad.TypeQuadric() == SurfaceKind.Cone) || (quad.TypeQuadric() == SurfaceKind.Sphere)
                                   || (quad.TypeQuadric() == SurfaceKind.Cylinder)
                                   || (quad.TypeQuadric() == SurfaceKind.Torus);

        if (!isDecomposeRequired)
        {
            return;
        }

        // post processing for cones and spheres

        TopolTool pDomain = (reversed) ? d1 : d2;

        NCollection_Sequence<PatchLine> dslin = new();
        bool isDecompose = false;
        for (int i = 1; i <= _slin.Length(); i++)
        {
            if (DecomposeResult((PointLine)_slin.Value(i), reversed, quad, pDomain, qSurf, otherSurf, tolArc,
                                tol3d, dslin))
            {
                isDecompose = true;
            }
        }

        if (!isDecompose)
        {
            return;
        }

        _slin.Clear();
        for (int i = 1; i <= dslin.Length(); i++)
        {
            _slin.Append(dslin.Value(i));
        }
    }
}
