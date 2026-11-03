// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpPrmIntersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpPrmIntersection.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpPrmIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntPatch_IType;
using static Occt.IntSurf_TypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Occt;

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
public partial class IntPatch_ImpPrmIntersection
{
    private bool done;
    private bool empt;
    private readonly NCollection_Sequence<IntPatch_Point> spnt = new();
    private readonly NCollection_Sequence<IntPatch_Line> slin = new();
    private readonly IntPatch_TheSOnBounds solrst = new();
    private readonly IntPatch_TheSearchInside solins = new();
    private bool myIsStartPnt;
    private double myUStart;
    private double myVStart;

    public IntPatch_ImpPrmIntersection()
    {
        done = false;
        empt = false;
        myIsStartPnt = false;
        myUStart = 0.0;
        myVStart = 0.0;
    }

    public IntPatch_ImpPrmIntersection(SurfaceAdaptorBase Surf1, TopolTool D1, SurfaceAdaptorBase Surf2,
                                       TopolTool D2, double TolArc, double TolTang, double Fleche, double Pas)
        : this()
    {
        Perform(Surf1, D1, Surf2, D2, TolArc, TolTang, Fleche, Pas);
    }

    /// <summary>Searches the solution from the given point of the parametric surface.</summary>
    public void SetStartPoint(double U, double V)
    {
        myIsStartPnt = true;
        myUStart = U;
        myVStart = V;
    }

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone() => done;

    /// <summary>True if there is no intersection. Throws StdFail_NotDone if not done.</summary>
    public bool IsEmpty()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return empt;
    }

    /// <summary>The number of single points. Throws StdFail_NotDone if not done.</summary>
    public int NbPnts()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return spnt.Length();
    }

    /// <summary>The single point <paramref name="Index"/>. Throws StdFail_NotDone if not done.</summary>
    public IntPatch_Point Point(int Index)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return spnt.Value(Index);
    }

    /// <summary>The number of lines. Throws StdFail_NotDone if not done.</summary>
    public int NbLines()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return slin.Length();
    }

    /// <summary>The line <paramref name="Index"/>. Throws StdFail_NotDone if not done.</summary>
    public IntPatch_Line Line(int Index)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return slin.Value(Index);
    }

    /// <summary>
    /// Computes the intersection. One of the surfaces must be a plane, a cylinder, a sphere or a cone (else
    /// Standard_ConstructionError); D1 and D2 are their domains; TolArc the tolerance on the restrictions, TolTang
    /// the tangency tolerance, Fleche the deflection and Pas the step of the walking.
    /// </summary>
    public void Perform(SurfaceAdaptorBase Surf1, TopolTool D1, SurfaceAdaptorBase Surf2,
                        TopolTool D2, double TolArc, double TolTang, double Fleche, double Pas)
    {
        bool reversed, procf, procl, dofirst, dolast;
        int indfirst = 0, indlast = 0, ind2, NbSegm;
        int NbPointIns, NbPointRst, Nblines, Nbpts, NbPointDep;
        double U1 = 0.0, V1 = 0.0, U2 = 0.0, V2 = 0.0, paramf, paraml, currentparam;

        IntPatch_TheSegmentOfTheSOnBounds thesegm;
        IntSurf_PathPoint PPoint;

        IntPatch_RLine rline;
        IntPatch_WLine wline;
        IntPatch_ThePathPointOfTheSOnBounds PStartf = new(), PStartl = new();
        IntPatch_Point ptdeb = new(), ptfin = new(), ptbis;

        IntPatch_IType typ;
        IntSurf_Transition TLine = new(), TArc = new();
        IntSurf_TypeTrans trans1, trans2;
        Point3D valpt;
        Vector3D tgline, tgrst = new(), norm1, norm2, d1u = new(), d1v = new();
        Direction3D DirNormale;
        Vector3D VecNormale;

        Point2D p2d;
        Vector2D d2d;

        Curve2DAdaptorBase currentarc;
        SurfaceKind typeS1, typeS2;
        IntSurf_Quadric Quad = new();
        IntPatch_TheSurfFunction Func = new();
        IntPatch_ArcFunction AFunc = new();
        //
        typeS1 = Surf1.GetType();
        typeS2 = Surf2.GetType();

        paramf = 0.0;
        paraml = 0.0;
        trans1 = IntSurf_Undecided;
        trans2 = IntSurf_Undecided;
        //
        done = false;
        empt = true;
        slin.Clear();
        spnt.Clear();
        //
        reversed = false;
        switch (typeS1)
        {
            case Plane:
                Quad.SetValue(Surf1.Plane());
                break;

            case SurfaceKind.Cylinder:
                Quad.SetValue(Surf1.Cylinder());
                break;

            case SurfaceKind.Sphere:
                Quad.SetValue(Surf1.Sphere());
                break;

            case SurfaceKind.Cone:
                Quad.SetValue(Surf1.Cone());
                break;

            default:
            {
                reversed = true;
                switch (typeS2)
                {
                    case Plane:
                        Quad.SetValue(Surf2.Plane());
                        break;

                    case SurfaceKind.Cylinder:
                        Quad.SetValue(Surf2.Cylinder());
                        break;

                    case SurfaceKind.Sphere:
                        Quad.SetValue(Surf2.Sphere());
                        break;

                    case SurfaceKind.Cone:
                        Quad.SetValue(Surf2.Cone());
                        break;
                    default:
                        throw new ConstructionErrorException();
                }
            }
            break;
        }
        //
        double aLocalPas;
        if (reversed)
        {
            aLocalPas = GetLocalStep(Surf1, Pas);
        }
        else
        {
            aLocalPas = GetLocalStep(Surf2, Pas);
        }

        Func.SetImplicitSurface(Quad);
        Func.Set(IntSurf_QuadricTool.Tolerance(Quad));
        AFunc.SetQuadric(Quad);
        //
        if (!reversed)
        {
            Func.Set(Surf2);
            AFunc.Set(Surf2);
        }
        else
        {
            Func.Set(Surf1);
            AFunc.Set(Surf1);
        }
        //
        if (!reversed)
        {
            solrst.Perform(AFunc, D2, TolArc, TolTang);
        }
        else
        {
            solrst.Perform(AFunc, D1, TolArc, TolTang);
        }
        if (!solrst.IsDone())
        {
            return;
        }
        //
        NCollection_Sequence<IntSurf_PathPoint> seqpdep = new();
        NCollection_Sequence<IntSurf_InteriorPoint> seqpins = new();
        //
        NbPointRst = solrst.NbPoints();
        NCollection_Array1<int> Destination = new(1, NbPointRst + 1);
        Destination.Init(0);
        if (NbPointRst != 0)
        {
            if (!reversed)
            {
                ComputeTangency(solrst, seqpdep, D2, Func, Surf2, Destination);
            }
            else
            {
                ComputeTangency(solrst, seqpdep, D1, Func, Surf1, Destination);
            }
        }
        //
        bool SearchIns = true;
        if (Quad.TypeQuadric() == Plane && solrst.NbSegments() > 0)
        {
            // For such kind of cases it is possible that whole surface is on one side of plane,
            // plane only touches surface and does not cross it,
            // so no inner points exist.
            SearchIns = false;
            TopolTool T;
            if (reversed)
            {
                T = D1;
            }
            else
            {
                T = D2;
            }
            int aNbSamples = T.NbSamples();
            math_Vector Valf = new(1, 1), UVap = new(1, 2);
            T.SamplePoint(1, out Point2D s2d, out _);
            UVap[1] = s2d.X();
            UVap[2] = s2d.Y();
            Func.Value(UVap, Valf);
            double rvalf = Math.CopySign(1.0, Valf[1]);
            for (int i = 2; i <= aNbSamples; ++i)
            {
                T.SamplePoint(i, out s2d, out _);
                UVap[1] = s2d.X();
                UVap[2] = s2d.Y();
                Func.Value(UVap, Valf);
                if (rvalf * Valf[1] < 0.0)
                {
                    SearchIns = true;
                    break;
                }
            }
        }
        // Recherche des points interieurs
        NbPointIns = 0;
        if (SearchIns)
        {
            if (!reversed)
            {
                if (myIsStartPnt)
                {
                    solins.Perform(Func, Surf2, myUStart, myVStart);
                }
                else
                {
                    solins.Perform(Func, Surf2, D2, TolTang);
                }
            }
            else
            {
                if (myIsStartPnt)
                {
                    solins.Perform(Func, Surf1, myUStart, myVStart);
                }
                else
                {
                    solins.Perform(Func, Surf1, D1, TolTang);
                }
            }
            NbPointIns = solins.NbPoints();
            for (int i = 1; i <= NbPointIns; i++)
            {
                seqpins.Append(solins.Value(i));
            }
        }
        //
        NbPointDep = seqpdep.Length();
        //
        if (NbPointDep != 0 || NbPointIns != 0)
        {
            IntPatch_TheIWalking iwalk = new(TolTang, Fleche, aLocalPas);
            iwalk.Perform(seqpdep, seqpins, Func, reversed ? Surf1 : Surf2, reversed);

            if (!iwalk.IsDone())
            {
                return;
            }

            double Vmin, Vmax, TolV = 1.0e-14;
            if (!reversed)
            { // Surf1 is quadric
                Vmin = Surf1.FirstVParameter();
                Vmax = Surf1.LastVParameter();
            }
            else
            { // Surf2 is quadric
                Vmin = Surf2.FirstVParameter();
                Vmax = Surf2.LastVParameter();
            }
            //
            Nblines = iwalk.NbLines();
            for (int j = 1; j <= Nblines; j++)
            {
                IntPatch_TheIWLineOfTheIWalking iwline = iwalk.Value(j);
                IntSurf_LineOn2S thelin = iwline.Line();

                Nbpts = thelin.NbPoints();
                if (Nbpts >= 2)
                {
                    tgline = iwline.TangentVector(out int k);
                    if (k >= 1 && k <= Nbpts)
                    {
                    }
                    else
                    {
                        k = Nbpts >> 1;
                    }
                    valpt = thelin.Value(k).Value();

                    if (!reversed)
                    {
                        thelin.Value(k).ParametersOnS2(out U2, out V2);
                        norm1 = Quad.Normale(valpt);
                        Surf2.D1(U2, V2, out _, out d1u, out d1v);
                        norm2 = d1u.Crossed(d1v);
                    }
                    else
                    {
                        thelin.Value(k).ParametersOnS1(out U2, out V2);
                        norm2 = Quad.Normale(valpt);
                        Surf1.D1(U2, V2, out _, out d1u, out d1v);
                        norm1 = d1u.Crossed(d1v);
                    }
                    if (tgline.DotCross(norm2, norm1) > 0.0)
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }

                    //
                    double AnU1 = 0.0, AnU2, AnV2;

                    SurfaceKind typQuad = Quad.TypeQuadric();
                    bool arecadr = false;
                    valpt = thelin.Value(1).Value();
                    Quad.Parameters(valpt, ref AnU1, ref V1);

                    if ((V1 < Vmin) && (Vmin - V1 < TolV))
                    {
                        V1 = Vmin;
                    }
                    if ((V1 > Vmax) && (V1 - Vmax < TolV))
                    {
                        V1 = Vmax;
                    }

                    if (reversed)
                    {
                        thelin.SetUV(1, false, AnU1, V1); //-- on va lire u2,v2
                        thelin.Value(1).ParametersOnS1(out AnU2, out AnV2);
                    }
                    else
                    {
                        thelin.SetUV(1, true, AnU1, V1); //-- on va lire u1,v1
                        thelin.Value(1).ParametersOnS2(out AnU2, out AnV2);
                    }

                    if (typQuad == SurfaceKind.Cylinder || typQuad == SurfaceKind.Cone || typQuad == SurfaceKind.Sphere)
                    {
                        arecadr = true;
                    }
                    //
                    for (k = 2; k <= Nbpts; ++k)
                    {
                        valpt = thelin.Value(k).Value();
                        Quad.Parameters(valpt, ref U1, ref V1);
                        //
                        if ((V1 < Vmin) && (Vmin - V1 < TolV))
                        {
                            V1 = Vmin;
                        }
                        if ((V1 > Vmax) && (V1 - Vmax < TolV))
                        {
                            V1 = Vmax;
                        }
                        //
                        if (arecadr)
                        {
                            double aCf, aTwoPI;
                            //
                            aCf = 0.0;
                            aTwoPI = M_PI + M_PI;
                            if ((U1 - AnU1) > 1.5 * M_PI)
                            {
                                while ((U1 - AnU1) > (1.5 * M_PI + aCf * aTwoPI))
                                {
                                    aCf = aCf + 1.0;
                                }
                                U1 = U1 - aCf * aTwoPI;
                            }
                            //
                            else
                            {
                                while ((U1 - AnU1) < (-1.5 * M_PI - aCf * aTwoPI))
                                {
                                    aCf = aCf + 1.0;
                                }
                                U1 = U1 + aCf * aTwoPI;
                            }
                        }
                        //
                        if (reversed)
                        {
                            thelin.SetUV(k, false, U1, V1);

                            thelin.Value(k).ParametersOnS1(out U2, out V2);
                            switch (typeS1)
                            {
                                case SurfaceKind.Cylinder:
                                case SurfaceKind.Cone:
                                case SurfaceKind.Sphere:
                                case SurfaceKind.Torus:
                                    while (U2 < (AnU2 - 1.5 * M_PI))
                                    {
                                        U2 += M_PI + M_PI;
                                    }
                                    while (U2 > (AnU2 + 1.5 * M_PI))
                                    {
                                        U2 -= M_PI + M_PI;
                                    }
                                    break;
                                default:
                                    break;
                            }
                            if (typeS2 == SurfaceKind.Torus)
                            {
                                while (V2 < (AnV2 - 1.5 * M_PI))
                                {
                                    V2 += M_PI + M_PI;
                                }
                                while (V2 > (AnV2 + 1.5 * M_PI))
                                {
                                    V2 -= M_PI + M_PI;
                                }
                            }
                            thelin.SetUV(k, true, U2, V2);
                        }
                        else
                        {
                            thelin.SetUV(k, true, U1, V1);

                            thelin.Value(k).ParametersOnS2(out U2, out V2);
                            switch (typeS2)
                            {
                                case SurfaceKind.Cylinder:
                                case SurfaceKind.Cone:
                                case SurfaceKind.Sphere:
                                case SurfaceKind.Torus:
                                    while (U2 < (AnU2 - 1.5 * M_PI))
                                    {
                                        U2 += M_PI + M_PI;
                                    }
                                    while (U2 > (AnU2 + 1.5 * M_PI))
                                    {
                                        U2 -= M_PI + M_PI;
                                    }
                                    break;
                                default:
                                    break;
                            }
                            if (typeS2 == SurfaceKind.Torus)
                            {
                                while (V2 < (AnV2 - 1.5 * M_PI))
                                {
                                    V2 += M_PI + M_PI;
                                }
                                while (V2 > (AnV2 + 1.5 * M_PI))
                                {
                                    V2 -= M_PI + M_PI;
                                }
                            }
                            thelin.SetUV(k, false, U2, V2);
                        }

                        AnU1 = U1;
                        AnU2 = U2;
                        AnV2 = V2;
                    }
                    // <-A
                    wline = new IntPatch_WLine(thelin, false, trans1, trans2);
                    wline.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLImpPrm);

                    if (iwline.HasFirstPoint() && !iwline.IsTangentAtBegining())
                    {
                        indfirst = iwline.FirstPointIndex();
                        PPoint = seqpdep.Value(indfirst);
                        tgline = PPoint.Direction3d();
                        int themult = PPoint.Multiplicity();
                        for (int i = NbPointRst; i >= 1; i--)
                        {
                            if (Destination.Value(i) == indfirst)
                            {
                                if (!reversed)
                                { //-- typeS1 = Pln || Cyl || Sph || Cone
                                    Quad.Parameters(PPoint.Value(), ref U1, ref V1);

                                    if ((V1 < Vmin) && (Vmin - V1 < TolV))
                                    {
                                        V1 = Vmin;
                                    }
                                    if ((V1 > Vmax) && (V1 - Vmax < TolV))
                                    {
                                        V1 = Vmax;
                                    }

                                    PPoint.Parameters(themult, out U2, out V2);
                                    Surf2.D1(U2, V2, out _, out d1u, out d1v); //-- @@@@
                                }
                                else
                                { //-- typeS1 != Pln && Cyl && Sph && Cone
                                    Quad.Parameters(PPoint.Value(), ref U2, ref V2);

                                    if ((V2 < Vmin) && (Vmin - V2 < TolV))
                                    {
                                        V2 = Vmin;
                                    }
                                    if ((V2 > Vmax) && (V2 - Vmax < TolV))
                                    {
                                        V2 = Vmax;
                                    }

                                    PPoint.Parameters(themult, out U1, out V1);
                                    Surf1.D1(U1, V1, out _, out d1u, out d1v); //-- @@@@
                                }

                                VecNormale = d1u.Crossed(d1v);
                                //-- Modif du 27 Septembre 94 (Recadrage des pts U,V)
                                ptdeb.SetValue(PPoint.Value(), TolArc, false);
                                ptdeb.SetParameters(U1, V1, U2, V2);
                                ptdeb.SetParameter(1.0);

                                Recadre(reversed, typeS1, typeS2, ref ptdeb, iwline, 1, U1, V1, U2, V2);

                                currentarc = solrst.Point(i).Arc();
                                currentparam = solrst.Point(i).Parameter();
                                currentarc.D1(currentparam, out p2d, out d2d);
                                tgrst.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);

                                double squaremagnitudeVecNormale = VecNormale.SquareMagnitude();
                                if (squaremagnitudeVecNormale > 1e-13)
                                {
                                    DirNormale = new Direction3D(VecNormale);
                                    IntSurf.MakeTransition(tgline, tgrst, DirNormale, ref TLine, ref TArc);
                                }
                                else
                                {
                                    TLine.SetValue(true, IntSurf_Undecided);
                                    TArc.SetValue(true, IntSurf_Undecided);
                                }

                                ptdeb.SetArc(reversed, currentarc, currentparam, TLine, TArc);
                                if (!solrst.Point(i).IsNew())
                                {
                                    ptdeb.SetVertex(reversed, solrst.Point(i).Vertex());
                                }
                                wline.AddVertex(ptdeb);
                                if (themult == 0)
                                {
                                    wline.SetFirstPoint(wline.NbVertex());
                                }

                                themult--;
                            }
                        }
                    }
                    else if (iwline.IsTangentAtBegining())
                    {
                        Point3D psol = thelin.Value(1).Value();
                        thelin.Value(1).ParametersOnS1(out U1, out V1);
                        thelin.Value(1).ParametersOnS2(out U2, out V2);
                        ptdeb.SetValue(psol, TolArc, true);
                        ptdeb.SetParameters(U1, V1, U2, V2);
                        ptdeb.SetParameter(1.0);
                        wline.AddVertex(ptdeb);
                        wline.SetFirstPoint(wline.NbVertex());
                    }
                    else
                    {
                        Point3D psol = thelin.Value(1).Value();
                        thelin.Value(1).ParametersOnS1(out U1, out V1);
                        thelin.Value(1).ParametersOnS2(out U2, out V2);
                        ptdeb.SetValue(psol, TolArc, false);
                        ptdeb.SetParameters(U1, V1, U2, V2);
                        ptdeb.SetParameter(1.0);
                        wline.AddVertex(ptdeb);
                        wline.SetFirstPoint(wline.NbVertex());
                    }

                    if (iwline.HasLastPoint() && !iwline.IsTangentAtEnd())
                    {
                        indlast = iwline.LastPointIndex();
                        PPoint = seqpdep.Value(indlast);
                        tgline = PPoint.Direction3d().Reversed();
                        int themult = PPoint.Multiplicity();
                        for (int i = NbPointRst; i >= 1; i--)
                        {
                            if (Destination.Value(i) == indlast)
                            {
                                if (!reversed)
                                {
                                    Quad.Parameters(PPoint.Value(), ref U1, ref V1);

                                    if ((V1 < Vmin) && (Vmin - V1 < TolV))
                                    {
                                        V1 = Vmin;
                                    }
                                    if ((V1 > Vmax) && (V1 - Vmax < TolV))
                                    {
                                        V1 = Vmax;
                                    }

                                    PPoint.Parameters(themult, out U2, out V2);
                                    Surf2.D1(U2, V2, out _, out d1u, out d1v); //-- @@@@
                                    VecNormale = d1u.Crossed(d1v); //-- @@@@
                                }
                                else
                                {
                                    Quad.Parameters(PPoint.Value(), ref U2, ref V2);

                                    if ((V2 < Vmin) && (Vmin - V2 < TolV))
                                    {
                                        V2 = Vmin;
                                    }
                                    if ((V2 > Vmax) && (V2 - Vmax < TolV))
                                    {
                                        V2 = Vmax;
                                    }

                                    PPoint.Parameters(themult, out U1, out V1);
                                    Surf1.D1(U1, V1, out _, out d1u, out d1v); //-- @@@@
                                    VecNormale = d1u.Crossed(d1v); //-- @@@@
                                }

                                ptfin.SetValue(PPoint.Value(), TolArc, false);
                                ptfin.SetParameters(U1, V1, U2, V2);
                                ptfin.SetParameter(Nbpts);

                                Recadre(reversed, typeS1, typeS2, ref ptfin, iwline, Nbpts - 1, U1, V1, U2, V2);

                                currentarc = solrst.Point(i).Arc();
                                currentparam = solrst.Point(i).Parameter();
                                currentarc.D1(currentparam, out p2d, out d2d);
                                tgrst.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);

                                double squaremagnitudeVecNormale = VecNormale.SquareMagnitude();
                                if (squaremagnitudeVecNormale > 1e-13)
                                {
                                    DirNormale = new Direction3D(VecNormale);
                                    IntSurf.MakeTransition(tgline, tgrst, DirNormale, ref TLine, ref TArc);
                                }
                                else
                                {
                                    TLine.SetValue(true, IntSurf_Undecided);
                                    TArc.SetValue(true, IntSurf_Undecided);
                                }

                                ptfin.SetArc(reversed, currentarc, currentparam, TLine, TArc);
                                if (!solrst.Point(i).IsNew())
                                {
                                    ptfin.SetVertex(reversed, solrst.Point(i).Vertex());
                                }
                                wline.AddVertex(ptfin);
                                if (themult == 0)
                                {
                                    wline.SetLastPoint(wline.NbVertex());
                                }

                                themult--;
                            }
                        }
                    }
                    else if (iwline.IsTangentAtEnd())
                    {
                        Point3D psol = thelin.Value(Nbpts).Value();
                        thelin.Value(Nbpts).ParametersOnS1(out U1, out V1);
                        thelin.Value(Nbpts).ParametersOnS2(out U2, out V2);
                        ptfin.SetValue(psol, TolArc, true);
                        ptfin.SetParameters(U1, V1, U2, V2);
                        ptfin.SetParameter(Nbpts);
                        wline.AddVertex(ptfin);
                        wline.SetLastPoint(wline.NbVertex());
                    }
                    else
                    {
                        Point3D psol = thelin.Value(Nbpts).Value();
                        thelin.Value(Nbpts).ParametersOnS1(out U1, out V1);
                        thelin.Value(Nbpts).ParametersOnS2(out U2, out V2);
                        ptfin.SetValue(psol, TolArc, false);
                        ptfin.SetParameters(U1, V1, U2, V2);
                        ptfin.SetParameter(Nbpts);
                        wline.AddVertex(ptfin);
                        wline.SetLastPoint(wline.NbVertex());
                    }
                    //
                    // Il faut traiter les points de passage.
                    slin.Append(wline);
                } // if(Nbpts>=2) {
            } // for (j=1; j<=Nblines; j++) {

            // ON GERE LES RACCORDS ENTRE LIGNES. ELLE NE PEUVENT SE RACCORDER
            // QUE SUR DES POINTS DE TANGENCE

            Nblines = slin.Length();
            for (int j = 1; j <= Nblines - 1; j++)
            {
                dofirst = dolast = false;
                IntPatch_WLine wlin1 = (IntPatch_WLine)slin.Value(j);
                if (wlin1.HasFirstPoint())
                {
                    ptdeb = wlin1.FirstPoint(out indfirst);
                    if (ptdeb.IsTangencyPoint())
                    {
                        dofirst = true;
                    }
                }
                if (wlin1.HasLastPoint())
                {
                    ptfin = wlin1.LastPoint(out indlast);
                    if (ptfin.IsTangencyPoint())
                    {
                        dolast = true;
                    }
                }

                if (dofirst || dolast)
                {
                    for (int k = j + 1; k <= Nblines; k++)
                    {
                        IntPatch_WLine wlin2 = (IntPatch_WLine)slin.Value(k);
                        if (wlin2.HasFirstPoint())
                        {
                            ptbis = wlin2.FirstPoint(out ind2);
                            if (ptbis.IsTangencyPoint())
                            {
                                if (dofirst)
                                {
                                    if (ptdeb.Value().Distance(ptbis.Value()) <= TolArc)
                                    {
                                        ptdeb.SetMultiple(true);
                                        if (!ptbis.IsMultiple())
                                        {
                                            ptbis.SetMultiple(true);
                                            wlin2.Replace(ind2, ptbis);
                                        }
                                    }
                                }
                                if (dolast)
                                {
                                    if (ptfin.Value().Distance(ptbis.Value()) <= TolArc)
                                    {
                                        ptfin.SetMultiple(true);
                                        if (!ptbis.IsMultiple())
                                        {
                                            ptbis.SetMultiple(true);
                                            wlin2.Replace(ind2, ptbis);
                                        }
                                    }
                                }
                            }
                        }
                        if (wlin2.HasLastPoint())
                        {
                            ptbis = wlin2.LastPoint(out ind2);
                            if (ptbis.IsTangencyPoint())
                            {
                                if (dofirst)
                                {
                                    if (ptdeb.Value().Distance(ptbis.Value()) <= TolArc)
                                    {
                                        ptdeb.SetMultiple(true);
                                        if (!ptbis.IsMultiple())
                                        {
                                            ptbis.SetMultiple(true);
                                            wlin2.Replace(ind2, ptbis);
                                        }
                                    }
                                }
                                if (dolast)
                                {
                                    if (ptfin.Value().Distance(ptbis.Value()) <= TolArc)
                                    {
                                        ptfin.SetMultiple(true);
                                        if (!ptbis.IsMultiple())
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
        NbSegm = solrst.NbSegments();
        if (NbSegm != 0)
        {
            for (int i = 1; i <= NbSegm; i++)
            {
                thesegm = solrst.Segment(i);
                // Check if segment is degenerated
                if (thesegm.HasFirstPoint() && thesegm.HasLastPoint())
                {
                    double tol2 = global::Kerf.Engine.Foundation.Precision.Confusion();
                    tol2 *= tol2;
                    Point3D aPf = thesegm.FirstPoint().Value();
                    Point3D aPl = thesegm.LastPoint().Value();
                    if (aPf.SquareDistance(aPl) <= tol2)
                    {
                        // segment can be degenerated - check inner point
                        paramf = thesegm.FirstPoint().Parameter();
                        paraml = thesegm.LastPoint().Parameter();
                        Point2D _p2d = thesegm.Curve().Value(.57735 * paramf + 0.42265 * paraml);
                        Point3D aPm;
                        if (reversed)
                        {
                            Surf1.D0(_p2d.X(), _p2d.Y(), out aPm);
                        }
                        else
                        {
                            Surf2.D0(_p2d.X(), _p2d.Y(), out aPm);
                        }
                        if (aPm.SquareDistance(aPf) <= tol2)
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
                IntSurf_Transition TLineUnk = new(), TArcUnk = new();

                IntPatch_Point _thepointAtBeg = new();
                IntPatch_Point _thepointAtEnd = new();

                bool TransitionOK = false;

                if (thesegm.HasFirstPoint())
                {
                    double _u1 = 0.0, _v1 = 0.0, _u2 = 0.0, _v2 = 0.0;

                    dofirst = true;
                    PStartf = thesegm.FirstPoint();
                    paramf = PStartf.Parameter();

                    Point2D _p2d = thesegm.Curve().Value(paramf);
                    HVertex? _vtx = null;
                    if (!PStartf.IsNew())
                    {
                        _vtx = PStartf.Vertex();
                    }
                    Point3D _Pp = PStartf.Value();
                    _thepointAtBeg.SetValue(_Pp, PStartf.Tolerance(), false);
                    if (!reversed)
                    { //-- typeS1 = Pln || Cyl || Sph || Cone
                        Quad.Parameters(_Pp, ref _u1, ref _v1);
                        _u2 = _p2d.X();
                        _v2 = _p2d.Y();
                    }
                    else
                    { //-- typeS1 != Pln && Cyl && Sph && Cone
                        Quad.Parameters(_Pp, ref _u2, ref _v2);
                        _u1 = _p2d.X();
                        _v1 = _p2d.Y();
                    }
                    _thepointAtBeg.SetParameters(_u1, _v1, _u2, _v2);
                    _thepointAtBeg.SetParameter(paramf);
                    if (!PStartf.IsNew())
                    {
                        _thepointAtBeg.SetVertex(reversed, _vtx!);
                    }
                    _thepointAtBeg.SetArc(reversed, thesegm.Curve(), paramf, TLineUnk, TArcUnk);

                    Surf1.D1(_u1, _v1, out _, out Vector3D d1u1, out Vector3D d1v1);
                    norm1 = d1u1.Crossed(d1v1);
                    Surf2.D1(_u2, _v2, out _, out Vector3D d1u2, out Vector3D d1v2);
                    norm2 = d1u2.Crossed(d1v2);

                    thesegm.Curve().D1(paramf, out _p2d, out Vector2D _d2d);
                    tgline = new Vector3D();
                    if (reversed)
                    {
                        tgline.SetLinearForm(_d2d.X(), d1u1, _d2d.Y(), d1v1);
                    }
                    else
                    {
                        tgline.SetLinearForm(_d2d.X(), d1u2, _d2d.Y(), d1v2);
                    }
                    _u1 = tgline.DotCross(norm2, norm1);
                    TransitionOK = true;
                    if (_u1 > 0.00000001)
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else if (_u1 < -0.00000001)
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }
                    else
                    {
                        TransitionOK = false;
                    }
                }
                if (thesegm.HasLastPoint())
                {
                    double _u1 = 0.0, _v1 = 0.0, _u2 = 0.0, _v2 = 0.0;

                    dolast = true;
                    PStartl = thesegm.LastPoint();
                    paraml = PStartl.Parameter();

                    Point2D _p2d = thesegm.Curve().Value(paraml);
                    HVertex? _vtx = null;
                    if (!PStartl.IsNew())
                    {
                        _vtx = PStartl.Vertex();
                    }
                    Point3D _Pp = PStartl.Value();
                    _thepointAtEnd.SetValue(_Pp, PStartl.Tolerance(), false);
                    if (!reversed)
                    { //-- typeS1 = Pln || Cyl || Sph || Cone
                        Quad.Parameters(_Pp, ref _u1, ref _v1);
                        _u2 = _p2d.X();
                        _v2 = _p2d.Y();
                    }
                    else
                    { //-- typeS1 != Pln && Cyl && Sph && Cone
                        Quad.Parameters(_Pp, ref _u2, ref _v2);
                        _u1 = _p2d.X();
                        _v1 = _p2d.Y();
                    }
                    _thepointAtEnd.SetParameters(_u1, _v1, _u2, _v2);
                    _thepointAtEnd.SetParameter(paraml);
                    if (!PStartl.IsNew())
                    {
                        _thepointAtEnd.SetVertex(reversed, _vtx!);
                    }
                    _thepointAtEnd.SetArc(reversed, thesegm.Curve(), paraml, TLineUnk, TArcUnk);

                    Surf1.D1(_u1, _v1, out _, out Vector3D d1u1, out Vector3D d1v1);
                    norm1 = d1u1.Crossed(d1v1);
                    Surf2.D1(_u2, _v2, out _, out Vector3D d1u2, out Vector3D d1v2);
                    norm2 = d1u2.Crossed(d1v2);

                    thesegm.Curve().D1(paraml, out _p2d, out Vector2D _d2d);
                    tgline = new Vector3D();
                    if (reversed)
                    {
                        tgline.SetLinearForm(_d2d.X(), d1u1, _d2d.Y(), d1v1);
                    }
                    else
                    {
                        tgline.SetLinearForm(_d2d.X(), d1u2, _d2d.Y(), d1v2);
                    }
                    _u1 = tgline.DotCross(norm2, norm1);
                    TransitionOK = true;
                    if (_u1 > 0.00000001)
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else if (_u1 < -0.00000001)
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }
                    else
                    {
                        TransitionOK = false;
                    }
                }
                if (!TransitionOK)
                {
                    //-- rline = new IntPatch_RLine (thesegm.Curve(),reversed,false);
                    rline = new IntPatch_RLine(false);
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
                    rline = new IntPatch_RLine(false, trans1, trans2);
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
                if (thesegm.HasFirstPoint())
                {
                    rline.AddVertex(_thepointAtBeg);
                    rline.SetFirstPoint(rline.NbVertex());
                }

                if (thesegm.HasLastPoint())
                {
                    rline.AddVertex(_thepointAtEnd);
                    rline.SetLastPoint(rline.NbVertex());
                }

                // Polygone sur restriction solution
                if (dofirst && dolast)
                {
                    double prm;
                    Point3D ptpoly;
                    IntSurf_PntOn2S p2s = new();
                    IntSurf_LineOn2S Thelin = new();
                    Curve2DAdaptorBase arcsegm = thesegm.Curve();
                    int nbsample = 100;

                    if (!reversed)
                    {
                        for (int j = 1; j <= nbsample; j++)
                        {
                            prm = paramf + (j - 1) * (paraml - paramf) / (nbsample - 1);
                            arcsegm.D0(prm, out p2d);
                            Surf2.D0(p2d.X(), p2d.Y(), out ptpoly);

                            Quad.Parameters(ptpoly, ref U1, ref V1);
                            p2s.SetValue(ptpoly, U1, V1, p2d.X(), p2d.Y());
                            Thelin.Add(p2s);
                        }
                    }
                    else
                    {
                        for (int j = 1; j <= nbsample; j++)
                        {
                            prm = paramf + (j - 1) * (paraml - paramf) / (nbsample - 1);
                            arcsegm.D0(prm, out p2d);
                            Surf1.D0(p2d.X(), p2d.Y(), out ptpoly);

                            Quad.Parameters(ptpoly, ref U2, ref V2);
                            p2s.SetValue(ptpoly, p2d.X(), p2d.Y(), U2, V2);
                            Thelin.Add(p2s);
                        }
                    }
                    rline.Add(Thelin);
                }

                if (dofirst || dolast)
                {
                    Nblines = slin.Length();
                    for (int j = 1; j <= Nblines; j++)
                    {
                        IntPatch_Line slinj = slin.Value(j);
                        typ = slinj.ArcType();
                        if (typ == IntPatch_Walking)
                        {
                            Nbpts = ((IntPatch_WLine)slinj).NbVertex();
                        }
                        else
                        {
                            Nbpts = ((IntPatch_RLine)slinj).NbVertex();
                        }
                        for (int k = 1; k <= Nbpts; k++)
                        {
                            if (typ == IntPatch_Walking)
                            {
                                ptdeb = ((IntPatch_WLine)slinj).Vertex(k);
                            }
                            else
                            {
                                ptdeb = ((IntPatch_RLine)slinj).Vertex(k);
                            }
                            if (dofirst)
                            {
                                if (ptdeb.Value().Distance(PStartf.Value()) <= TolArc)
                                {
                                    ptdeb.SetMultiple(true);
                                    if (typ == IntPatch_Walking)
                                    {
                                        ((IntPatch_WLine)slinj).Replace(k, ptdeb);
                                    }
                                    else
                                    {
                                        ((IntPatch_RLine)slinj).Replace(k, ptdeb);
                                    }
                                    ptdeb.SetParameter(paramf);
                                    rline.AddVertex(ptdeb);
                                    if (!procf)
                                    {
                                        procf = true;
                                        rline.SetFirstPoint(rline.NbVertex());
                                    }
                                }
                            }
                            if (dolast)
                            {
                                if (dofirst)
                                { //-- on recharge le ptdeb
                                    if (typ == IntPatch_Walking)
                                    {
                                        ptdeb = ((IntPatch_WLine)slinj).Vertex(k);
                                    }
                                    else
                                    {
                                        ptdeb = ((IntPatch_RLine)slinj).Vertex(k);
                                    }
                                }
                                if (ptdeb.Value().Distance(PStartl.Value()) <= TolArc)
                                {
                                    ptdeb.SetMultiple(true);
                                    if (typ == IntPatch_Walking)
                                    {
                                        ((IntPatch_WLine)slinj).Replace(k, ptdeb);
                                    }
                                    else
                                    {
                                        ((IntPatch_RLine)slinj).Replace(k, ptdeb);
                                    }
                                    ptdeb.SetParameter(paraml);
                                    rline.AddVertex(ptdeb);
                                    if (!procl)
                                    {
                                        procl = true;
                                        rline.SetLastPoint(rline.NbVertex());
                                    }
                                }
                            }
                        }
                    }
                }
                slin.Append(rline);
            }
        } // if (NbSegm)
        //
        // on traite les restrictions de la surface implicite

        for (int i = 1, aNbLin = slin.Length(); i <= aNbLin; i++)
        {
            IntPatch_PointLine aL = (IntPatch_PointLine)slin.Value(i);

            if (!reversed)
            {
                IntPatch_RstInt.PutVertexOnLine(aL, Surf1, D1, Surf2, true, TolTang);
            }
            else
            {
                IntPatch_RstInt.PutVertexOnLine(aL, Surf2, D2, Surf1, false, TolTang);
            }

            if (aL.NbPnts() <= 2)
            {
                bool aCond = aL.NbPnts() < 2;
                if (!aCond)
                {
                    aCond = (aL.Point(1).IsSame(aL.Point(2), global::Kerf.Engine.Foundation.Precision.Confusion()));
                }

                if (aCond)
                {
                    slin.Remove(i);
                    i--;
                    aNbLin--;
                    continue;
                }
            }

            if (aL.ArcType() == IntPatch_Walking)
            {
                IntPatch_WLine aWL = (IntPatch_WLine)aL;
                slin.Append(aWL);
                slin.Remove(i);
                i--;
                aNbLin--;
            }
        }

        // Now slin is filled as follows: lower indices correspond to Restriction line,
        // after (higher indices) - only Walking-line.

        double aTol3d = Math.Max(Func.Tolerance(), TolTang);
        SurfaceAdaptorBase aQSurf = (reversed) ? Surf2 : Surf1;
        SurfaceAdaptorBase anOtherSurf = (reversed) ? Surf1 : Surf2;

        for (int i = 1; i <= slin.Length(); i++)
        {
            IntPatch_PointLine aL1 = (IntPatch_PointLine)slin.Value(i);
            IntPatch_RLine? aRL1 = aL1 as IntPatch_RLine;

            if (aRL1 is null)
            {
                // Walking-Walking cases are not supported
                break;
            }

            Curve2DAdaptorBase anArc = (aRL1.IsArcOnS1() ? aRL1.ArcOnS1() : aRL1.ArcOnS2())!;
            if (anArc.GetType() != CurveKind.Line)
            {
                // Restriction line must be isoline.
                // Other cases are not supported by
                // existing algorithms.

                break;
            }

            bool isFirstDeleted = false;

            for (int j = i + 1; j <= slin.Length(); j++)
            {
                IntPatch_PointLine aL2 = (IntPatch_PointLine)slin.Value(j);
                IntPatch_RLine? aRL2 = aL2 as IntPatch_RLine;

                // Here aL1 (i-th line) is Restriction-line and aL2 (j-th line) is
                // Restriction or Walking

                if (aRL2 is not null)
                {
                    Curve2DAdaptorBase anArc2 = (aRL2.IsArcOnS1() ? aRL2.ArcOnS1() : aRL2.ArcOnS2())!;
                    if (anArc2.GetType() != CurveKind.Line)
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
                Direction2D aDir = anArc.Line().Direction();

                double aTol2d = anOtherSurf.UResolution(aTol3d),
                       aPeriod = anOtherSurf.IsVPeriodic ? anOtherSurf.VPeriod() : 0.0;

                if (Math.Abs(aDir.X()) < 0.5)
                { // Restriction directs along V-direction
                    aTol2d = anOtherSurf.VResolution(aTol3d);
                    aPeriod = anOtherSurf.IsUPeriodic ? anOtherSurf.UPeriod() : 0.0;
                }

                bool isCoincide = IsCoincide(Func, aL2, anArc, aRL1.IsArcOnS1(), aTol3d, aTol2d, aPeriod);

                if (isCoincide)
                {
                    if (aRL2 is null)
                    { // Delete Walking-line
                        slin.Remove(j);
                        j--;
                    }
                    else
                    { // Restriction-Restriction
                        Curve2DAdaptorBase anArc2 = (aRL2.IsArcOnS1() ? aRL2.ArcOnS1() : aRL2.ArcOnS2())!;

                        double aRange2 = anArc2.LastParameter() - anArc2.FirstParameter();
                        double aRange1 = anArc.LastParameter() - anArc.FirstParameter();

                        if (aRange2 > aRange1)
                        {
                            isFirstDeleted = true;
                            break;
                        }
                        else
                        { // Delete j-th line
                            slin.Remove(j);
                            j--;
                        }
                    }
                }
            } // for(int j = i + 1; j <= slin.Length(); j++)

            if (isFirstDeleted)
            { // Delete i-th line
                slin.Remove(i);
                i--;
            }
        } // for (int i = 1; i <= slin.Length(); i++)

        empt = (slin.Length() == 0 && spnt.Length() == 0);
        done = true;

        if (slin.Length() == 0)
        {
            return;
        }

        bool isDecomposeRequired = (Quad.TypeQuadric() == SurfaceKind.Cone) || (Quad.TypeQuadric() == SurfaceKind.Sphere)
                                   || (Quad.TypeQuadric() == SurfaceKind.Cylinder)
                                   || (Quad.TypeQuadric() == SurfaceKind.Torus);

        if (!isDecomposeRequired)
        {
            return;
        }

        // post processing for cones and spheres

        TopolTool PDomain = (reversed) ? D1 : D2;

        NCollection_Sequence<IntPatch_Line> dslin = new();
        bool isDecompose = false;
        for (int i = 1; i <= slin.Length(); i++)
        {
            if (DecomposeResult((IntPatch_PointLine)slin.Value(i), reversed, Quad, PDomain, aQSurf, anOtherSurf, TolArc,
                                aTol3d, dslin))
            {
                isDecompose = true;
            }
        }

        if (!isDecompose)
        {
            return;
        }

        slin.Clear();
        for (int i = 1; i <= dslin.Length(); i++)
        {
            slin.Append(dslin.Value(i));
        }
    }
}
