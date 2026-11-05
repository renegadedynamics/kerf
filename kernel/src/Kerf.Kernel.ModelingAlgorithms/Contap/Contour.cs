// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Contour.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Contour.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Contour.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.HiddenLines.Contours.ContourLineType;
using static Kerf.Engine.HiddenLines.Contours.ContourTFunction;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.HiddenLines.Contours;

/// <summary>
/// The contours (silhouettes) of a surface for a view direction, a draft angle or an eye: the lines on the surface
/// where the normal is orthogonal to the view. In closed form on planes, spheres, cylinders and cones
/// (<see cref="ContAna"/>); otherwise by the boundary search, the interior search and the walking.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: <c>Perform</c> computes, the queries read. The file-static functions of
/// Contap_Contour.cxx are private static methods here. OCCT's messages to std::cout when the search of internal points
/// fails are left out.
/// </remarks>
internal sealed class Contour
{
    private const double Tolpetit = 1.0e-10; // for squared distance

    private const double Tole = 5.0e-6;

    private bool _done;
    private readonly NCollection_Sequence<ContourLine> _slin = new();
    private readonly TheSearch _solrst = new();
    private readonly TheSearchInside _solins = new();
    private readonly SurfFunction _sFunc = new();
    private readonly ArcFunction _aFunc = new();
    private bool _modeset;

    public Contour()
    {
        _done = false;
        _modeset = false;
    }

    /// <summary>A contour for the view direction <paramref name="direction"/>, to compute with <see cref="Perform(SurfaceAdaptorBase, TopolTool)"/>.</summary>
    public Contour(in Vector3D direction)
    {
        _done = false;
        _modeset = true;
        _sFunc.Set(new Direction3D(direction));
        _aFunc.Set(new Direction3D(direction));
    }

    /// <summary>A draft contour of angle <paramref name="angle"/> for the direction <paramref name="direction"/>.</summary>
    public Contour(in Vector3D direction, double angle)
    {
        _done = false;
        _modeset = true;
        _sFunc.Set(new Direction3D(direction), angle);
        _aFunc.Set(new Direction3D(direction), angle);
    }

    /// <summary>A contour for the eye <paramref name="eye"/> (a perspective view).</summary>
    public Contour(in Point3D eye)
    {
        _done = false;
        _modeset = true;
        _sFunc.Set(eye);
        _aFunc.Set(eye);
    }

    /// <summary>Computes the contour of the surface in the direction <paramref name="direction"/>.</summary>
    public Contour(SurfaceAdaptorBase surf, TopolTool domain, in Vector3D direction)
    {
        _done = false;
        _modeset = true;
        Perform(surf, domain, direction);
    }

    /// <summary>Computes the draft contour of angle <paramref name="angle"/> in the direction <paramref name="direction"/>.</summary>
    public Contour(SurfaceAdaptorBase surf, TopolTool domain, in Vector3D direction, double angle)
    {
        _done = false;
        _modeset = true;
        Perform(surf, domain, direction, angle);
    }

    /// <summary>Computes the contour for the eye <paramref name="eye"/> (a perspective view).</summary>
    public Contour(SurfaceAdaptorBase surf, TopolTool domain, in Point3D eye)
    {
        _done = false;
        _modeset = true;
        Perform(surf, domain, eye);
    }

    public void Init(in Vector3D direction)
    {
        _done = false;
        _modeset = true;
        _sFunc.Set(new Direction3D(direction));
        _aFunc.Set(new Direction3D(direction));
    }

    public void Init(in Vector3D direction, double angle)
    {
        _done = false;
        _modeset = true;
        _sFunc.Set(new Direction3D(direction), angle);
        _aFunc.Set(new Direction3D(direction), angle);
    }

    public void Init(in Point3D eye)
    {
        _done = false;
        _modeset = true;
        _sFunc.Set(eye);
        _aFunc.Set(eye);
    }

    /// <summary>Computes the contour with the view set at construction or by <c>Init</c>; throws Standard_ConstructionError if none was.</summary>
    public void Perform(SurfaceAdaptorBase surf, TopolTool domain)
    {
        if (!_modeset)
        {
            throw new ConstructionErrorException();
        }
        _sFunc.Set(surf);
        _aFunc.Set(surf);

        SurfaceKind typS = HSurfaceTool.GetType(surf);
        switch (typS)
        {
            case Plane:
            case SurfaceKind.Sphere:
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
                PerformAna(domain); // Surf,Domain,Direction,0.,gp_Pnt(0.,0.,0.),1);
                break;

            default:
                Perform(domain); // Surf,Domain,Direction,0.,gp_Pnt(0.,0.,0.),1);
                break;
        }
    }

    /// <summary>Computes the contour in the direction <paramref name="direction"/>.</summary>
    public void Perform(SurfaceAdaptorBase surf, TopolTool domain, in Vector3D direction)
    {
        Init(direction);
        Perform(surf, domain);
    }

    /// <summary>Computes the draft contour of angle <paramref name="angle"/> in the direction <paramref name="direction"/>.</summary>
    public void Perform(SurfaceAdaptorBase surf, TopolTool domain, in Vector3D direction, double angle)
    {
        Init(direction, angle);
        Perform(surf, domain);
    }

    /// <summary>Computes the contour for the eye <paramref name="eye"/>.</summary>
    public void Perform(SurfaceAdaptorBase surf, TopolTool domain, in Point3D eye)
    {
        Init(eye);
        Perform(surf, domain);
    }

    public bool IsDone => _done;

    /// <summary>True if there is no line.</summary>
    public bool IsEmpty => LineCount == 0;

    /// <summary>The number of lines; throws StdFail_NotDone before a computation.</summary>
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

    /// <summary>The line <paramref name="index"/>; throws StdFail_NotDone before a computation.</summary>
    public ContourLine Line(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _slin.Value(index);
    }

    /// <summary>The internal surface function, to compute tangents on the lines; throws StdFail_NotDone before a computation.</summary>
    public SurfFunction SurfaceFunction()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _sFunc;
    }

    //-- --------------------------------------------------------------------------------
    //-- Recherche des portions utiles sur les lignes

    private static void Recadre(SurfaceAdaptorBase myHS1, ref double u1, ref double v1)
    {
        double f, l, lmf;
        SurfaceKind typs1 = myHS1.GetType();

        bool myHS1IsUPeriodic, myHS1IsVPeriodic;
        switch (typs1)
        {
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
                myHS1IsUPeriodic = true;
                myHS1IsVPeriodic = false;
                break;
            case SurfaceKind.Torus:
                myHS1IsUPeriodic = myHS1IsVPeriodic = true;
                break;
            default:
                myHS1IsUPeriodic = myHS1IsVPeriodic = false;
                break;
        }
        if (myHS1IsUPeriodic)
        {
            lmf = M_PI + M_PI; //-- myHS1->UPeriod();
            f = myHS1.FirstUParameter();
            l = myHS1.LastUParameter();
            while (u1 < f)
            {
                u1 += lmf;
            }
            while (u1 > l)
            {
                u1 -= lmf;
            }
        }
        if (myHS1IsVPeriodic)
        {
            lmf = M_PI + M_PI; //-- myHS1->VPeriod();
            f = myHS1.FirstVParameter();
            l = myHS1.LastVParameter();
            while (v1 < f)
            {
                v1 += lmf;
            }
            while (v1 > l)
            {
                v1 -= lmf;
            }
        }
    }

    private static void LineConstructor(NCollection_Sequence<ContourLine> slin, TopolTool domain,
                                        ContourLine l, SurfaceAdaptorBase surf)
    {
        //-- ------------------------------------------------------------
        //-- on decoupe la ligne en portions  entre 2 vertex
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        ContourLineType typl = l.TypeContour();
        if (typl == ContourLineType.Walking)
        {
            int nbvtx = l.VertexCount;
            for (int i = 1; i < nbvtx; i++)
            {
                int firstp = (int)l.Vertex(i).ParameterOnLine();
                int lastp = (int)l.Vertex(i + 1).ParameterOnLine();
                if (firstp != lastp)
                {
                    int pmid = (firstp + lastp) / 2; //-- entiers
                    PntOn2S Pmid = l.Point(pmid);
                    Pmid.Parameters(out _, out _, out double u2, out double v2);
                    Recadre(surf, ref u2, ref v2);
                    Containment in2 = domain.Classify(new Point2D(u2, v2), tol);
                    if (in2 == global::Kerf.Topology.Containment.Out)
                    {
                    }
                    else
                    {
                        LineOn2S lineOn2S = new();
                        lineOn2S.Add(l.Point(firstp));
                        for (int j = firstp + 1; j <= lastp; j++)
                        {
                            double sqDist = l.Point(j).Value().SquareDistanceTo(l.Point(j - 1).Value());
                            if (sqDist > Constants.Resolution())
                            {
                                lineOn2S.Add(l.Point(j));
                            }
                        }
                        if (lineOn2S.PointCount < 2)
                        {
                            continue;
                        }
                        ContourLine line = new();
                        line.SetLineOn2S(lineOn2S);
                        ContourPoint pvtx = l.Vertex(i);
                        pvtx.SetParameter(1);
                        line.Add(pvtx);

                        pvtx = l.Vertex(i + 1);
                        pvtx.SetParameter(lineOn2S.PointCount);
                        line.Add(pvtx);
                        line.SetTransitionOnS(l.TransitionOnS());
                        slin.Append(line);
                    }
                }
            }
        }
        else if (typl == ContourLineType.Lin)
        {
            int nbvtx = l.VertexCount;
            for (int i = 1; i < nbvtx; i++)
            {
                double firstp = l.Vertex(i).ParameterOnLine();
                double lastp = l.Vertex(i + 1).ParameterOnLine();
                if (firstp != lastp)
                {
                    double u2 = 0.0, v2 = 0.0; // OCCT leaves them unset on other surfaces
                    double pmid = (firstp + lastp) * 0.5;
                    Point3D Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, l.Line());
                    if (HSurfaceTool.GetType(surf) == SurfaceKind.Cylinder)
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Cylinder(surf), Pmid, out u2, out v2);
                    }
                    else if (HSurfaceTool.GetType(surf) == SurfaceKind.Cone)
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Cone(surf), Pmid, out u2, out v2);
                    }
                    else
                    {
                        //-- std::cout<<" Pb ds Contap_ContourGen_2.gxx (type)"<<std::endl;
                    }

                    Recadre(surf, ref u2, ref v2);
                    Containment in2 = domain.Classify(new Point2D(u2, v2), tol);
                    if (in2 == global::Kerf.Topology.Containment.Out)
                    {
                    }
                    else
                    {
                        ContourLine line = new();
                        line.SetValue(l.Line());
                        ContourPoint pvtx = l.Vertex(i);
                        line.Add(pvtx);

                        pvtx = l.Vertex(i + 1);
                        line.Add(pvtx);
                        line.SetTransitionOnS(l.TransitionOnS());
                        slin.Append(line);
                    }
                }
            }
        }
        else if (typl == ContourLineType.Circle)
        {
            int nbvtx = l.VertexCount;
            bool novtx = true;
            if (nbvtx != 0)
            {
                novtx = false;
            }
            for (int i = 1; i < nbvtx || novtx; i++)
            {
                double firstp = 0, lastp = M_PI + M_PI;
                if (!novtx)
                {
                    firstp = l.Vertex(i).ParameterOnLine();
                    lastp = l.Vertex(i + 1).ParameterOnLine();
                }
                if (Math.Abs(firstp - lastp) > 0.000000001)
                {
                    double u2 = 0.0, v2 = 0.0; // OCCT leaves them unset on other surfaces
                    double pmid = (firstp + lastp) * 0.5;
                    Point3D Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, l.Circle());
                    if (HSurfaceTool.GetType(surf) == SurfaceKind.Cylinder)
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Cylinder(surf), Pmid, out u2, out v2);
                    }
                    else if (HSurfaceTool.GetType(surf) == SurfaceKind.Cone)
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Cone(surf), Pmid, out u2, out v2);
                    }
                    else if (HSurfaceTool.GetType(surf) == SurfaceKind.Sphere)
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Sphere(surf), Pmid, out u2, out v2);
                    }
                    else
                    {
                        //-- std::cout<<" Pb ds Contap_ContourGen_2.gxx (typep)"<<std::endl;
                    }

                    Recadre(surf, ref u2, ref v2);
                    Containment in2 = domain.Classify(new Point2D(u2, v2), tol);
                    if (in2 == global::Kerf.Topology.Containment.Out)
                    {
                    }
                    else
                    {
                        ContourLine line = new();
                        line.SetValue(l.Circle());
                        if (!novtx)
                        {
                            ContourPoint pvtx = l.Vertex(i);
                            line.Add(pvtx);
                            pvtx = l.Vertex(i + 1);
                            line.Add(pvtx);
                        }
                        line.SetTransitionOnS(l.TransitionOnS());
                        slin.Append(line);
                    }
                }
                novtx = false;
            }
            if (nbvtx != 0)
            {
                double firstp = l.Vertex(nbvtx).ParameterOnLine();
                double lastp = l.Vertex(1).ParameterOnLine() + M_PI + M_PI;
                if (Math.Abs(firstp - lastp) > 0.0000000001)
                {
                    double u2 = 0.0, v2 = 0.0; // OCCT leaves them unset on other surfaces
                    double pmid = (firstp + lastp) * 0.5;
                    Point3D Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, l.Circle());
                    if (HSurfaceTool.GetType(surf) == SurfaceKind.Cylinder)
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Cylinder(surf), Pmid, out u2, out v2);
                    }
                    else if (HSurfaceTool.GetType(surf) == SurfaceKind.Cone)
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Cone(surf), Pmid, out u2, out v2);
                    }
                    else if (HSurfaceTool.GetType(surf) == SurfaceKind.Sphere)
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Sphere(surf), Pmid, out u2, out v2);
                    }
                    else
                    {
                        //-- std::cout<<" Pb ds Contap_ContourGen_2.gxx (typep)"<<std::endl;
                    }

                    Recadre(surf, ref u2, ref v2);
                    Containment in2 = domain.Classify(new Point2D(u2, v2), tol);
                    if (in2 == global::Kerf.Topology.Containment.Out)
                    {
                    }
                    else
                    {
                        ContourLine line = new();
                        line.SetValue(l.Circle());
                        ContourPoint pvtx = l.Vertex(nbvtx);
                        line.Add(pvtx);

                        pvtx = l.Vertex(1);
                        pvtx.SetParameter(pvtx.ParameterOnLine() + M_PI + M_PI);
                        line.Add(pvtx);
                        line.SetTransitionOnS(l.TransitionOnS());
                        slin.Append(line);
                    }
                }
            }
        }
        else
        {
            //-- std::cout<<" ni WLine ni Lin ni Circ "<<std::endl;
            slin.Append(new ContourLine(l));
        }
        //--
    }

    //-- --------------------------------------------------------------------------------

    private static void KeepInsidePoints(TheSearchInside solins, TheSearch solrst,
                                         SurfFunction func,
                                         NCollection_Sequence<InteriorPoint> seqpins)
    {
        int nba = solrst.SegmentCount;
        int nbp, indp, inda;
        Point2D toproj;
        bool projok, tokeep;
        SurfaceAdaptorBase surf = func.Surface();

        nbp = solins.PointCount;
        for (indp = 1; indp <= nbp; indp++)
        {
            tokeep = true;
            InteriorPoint pti = solins.Value(indp);
            pti.Parameters(out double u, out double v);
            toproj = new Point2D(u, v);
            for (inda = 1; inda <= nba; inda++)
            {
                Curve2DAdaptorBase thearc = solrst.Segment(inda).Curve();
                projok = HContTool.Project(thearc, toproj, out _, out Point2D ptproj);
                if (projok)
                {
                    Point3D pprojete = HSurfaceTool.Value(surf, ptproj.X, ptproj.Y);
                    if (pti.Value().DistanceTo(pprojete) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        tokeep = false;
                        break;
                    }
                }
            }
            if (tokeep)
            {
                seqpins.Append(pti);
            }
        }
    }

    private static void ComputeTangency(TheSearch solrst, TopolTool domain,
                                        SurfFunction func, NCollection_Sequence<PathPoint> seqpdep,
                                        NCollection_Array1<int> destination)
    {
        int i, k;
        int nbPoints = solrst.PointCount;
        int seqlength = 0;

        double theparam, test;
        bool fairpt;
        Orientation arcorien, vtxorien;
        bool ispassing;

        math_Vector x = new(1, 2);
        math_Vector f = new(1, 1);
        Matrix d = new(1, 1, 1, 2);

        Vector3D normale, vectg, tg3drst, v1, v2;
        Direction2D dirtg;
        Vector2D tg2drst;
        Point2D pt2d;

        PathPoint pPoint = new();
        SurfaceAdaptorBase surf = func.Surface();

        for (i = 1; i <= nbPoints; i++)
        {
            if (destination[i] == 0)
            {
                ThePathPointOfTheSearch pStart = solrst.Point(i);
                Curve2DAdaptorBase thearc = pStart.Arc();
                theparam = pStart.Parameter();
                Point2D ptoproj = ContourHCurve2dTool.Value(thearc, theparam);
                //-- lbr le 15 mai 97
                //-- On elimine les points qui sont egalement present sur une restriction solution
                bool surUneRestrictionSolution = false;
                for (int restriction = 1; !surUneRestrictionSolution && restriction <= solrst.SegmentCount;
                     restriction++)
                {
                    Curve2DAdaptorBase thearcsol = solrst.Segment(restriction).Curve();
                    bool projok = HContTool.Project(thearcsol, ptoproj, out _, out Point2D pproj);
                    if (projok)
                    {
                        // gp_Pnt pprojete = Adaptor3d_HSurfaceTool::Value(Surf,Ptoproj.X(),Ptoproj.Y());
                        // IFV - begin
                        Point3D pprojete = HSurfaceTool.Value(surf, pproj.X, pproj.Y);
                        // IFV - end
                        if ((pStart.Value()).DistanceTo(pprojete) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            surUneRestrictionSolution = true;
                        }
                    }
                }
                if (!surUneRestrictionSolution)
                {
                    arcorien = domain.Orientation(thearc);
                    ispassing = (arcorien == Internal || arcorien == External);

                    ContourHCurve2dTool.D1(thearc, theparam, out pt2d, out tg2drst);
                    x[1] = pt2d.X;
                    x[2] = pt2d.Y;
                    pPoint.SetValue(pStart.Value(), x[1], x[2]);

                    func.Values(x, f, d);
                    if (func.IsTangent())
                    {
                        pPoint.SetTangency(true);
                        destination[i] = seqlength + 1;
                        if (!pStart.IsNew())
                        {
                            HVertex vtx = pStart.Vertex();
                            for (k = i + 1; k <= nbPoints; k++)
                            {
                                if (destination[k] == 0)
                                {
                                    ThePathPointOfTheSearch pStart2 = solrst.Point(k);
                                    if (!pStart2.IsNew())
                                    {
                                        HVertex vtx2 = pStart2.Vertex();
                                        if (domain.Identical(vtx, vtx2))
                                        {
                                            Curve2DAdaptorBase thearc2 = pStart2.Arc();
                                            theparam = pStart2.Parameter();
                                            arcorien = domain.Orientation(thearc2);
                                            ispassing = ispassing
                                                        && (arcorien == Internal || arcorien == External);

                                            pt2d = ContourHCurve2dTool.Value(thearc2, theparam);
                                            x[1] = pt2d.X;
                                            x[2] = pt2d.Y;
                                            pPoint.AddUV(x[1], x[2]);
                                            destination[k] = seqlength + 1;
                                        }
                                    }
                                }
                            }
                        }
                        pPoint.SetPassing(ispassing);
                        seqpdep.Append(pPoint);
                        seqlength++;
                    }
                    else
                    { // on a un point de depart potentiel

                        vectg = func.Direction3d();
                        dirtg = func.Direction2d();

                        //	Adaptor3d_HSurfaceTool::D1(Surf,X(1),X(2),ptbid,v1,v2);
                        SurfProps.DerivAndNorm(surf, x[1], x[2], out _, out v1, out v2, out normale);
                        tg3drst = tg2drst.X * v1 + tg2drst.Y * v2;
                        //	normale = v1.Crossed(v2);
                        if (normale.SquareMagnitude < RealEpsilon())
                        {
                            //-- std::cout<<"\n*** Contap_ContourGen_2.gxx  Normale Nulle en U:"<<X(1)<<"
                            // V:"<<X(2)<<std::endl;
                        }
                        else
                        {
                            test = vectg.Dot(normale.Crossed(tg3drst));

                            if (pStart.IsNew())
                            {
                                double tbis = vectg.Normalized().Dot(tg3drst.Normalized());
                                if (Math.Abs(tbis) < 1.0 - Tole)
                                {
                                    if ((test < 0.0 && arcorien == Forward)
                                        || (test > 0.0 && arcorien == Reversed))
                                    {
                                        vectg = vectg.Reversed();
                                        dirtg = dirtg.Reversed();
                                    }
                                    pPoint.SetDirections(vectg, dirtg);
                                }
                                else
                                { // on garde le point comme point d'arret (tangent)
                                    pPoint.SetTangency(true);
                                }
                                pPoint.SetPassing(ispassing);
                                destination[i] = seqlength + 1;
                                seqpdep.Append(pPoint);
                                seqlength++;
                            }
                            else
                            { // traiter la transition complexe
                                Direction3D bidnorm = new(1.0, 1.0, 1.0);

                                bool tobeverified = false;
                                Orientation locTrans;
                                CurveTransition comptrans = new();
                                comptrans.Reset(new Direction3D(vectg), bidnorm, 0.0);
                                if (arcorien != Internal && arcorien != External)
                                {
                                    // pour essai
                                    HVertex vtx = pStart.Vertex();
                                    vtxorien = domain.Orientation(vtx);
                                    test = test / (vectg.Magnitude);
                                    test = test / ((normale.Crossed(tg3drst)).Magnitude);

                                    if (Math.Abs(test) <= Tole)
                                    {
                                        tobeverified = true;
                                        locTrans = External; // et pourquoi pas INTERNAL
                                    }
                                    else
                                    {
                                        if ((test > 0.0 && arcorien == Forward)
                                            || (test < 0.0 && arcorien == Reversed))
                                        {
                                            locTrans = Forward;
                                        }
                                        else
                                        {
                                            locTrans = Reversed;
                                        }
                                        if (arcorien == Reversed)
                                        {
                                            tg3drst = tg3drst.Reversed();
                                        } // pas deja fait ???
                                    }

                                    comptrans.Compare(Tole, new Direction3D(tg3drst), bidnorm, 0.0, locTrans, vtxorien);
                                }
                                destination[i] = seqlength + 1;
                                for (k = i + 1; k <= nbPoints; k++)
                                {
                                    if (destination[k] == 0)
                                    {
                                        ThePathPointOfTheSearch pStart2 = solrst.Point(k);
                                        if (!pStart2.IsNew())
                                        {
                                            HVertex vtx2 = pStart2.Vertex();
                                            if (domain.Identical(pStart.Vertex(), vtx2))
                                            {
                                                Curve2DAdaptorBase thearc2 = pStart2.Arc();
                                                theparam = pStart2.Parameter();
                                                arcorien = domain.Orientation(thearc2);

                                                ContourHCurve2dTool.D1(thearc2, theparam, out pt2d, out tg2drst);
                                                x[1] = pt2d.X;
                                                x[2] = pt2d.Y;
                                                pPoint.AddUV(x[1], x[2]);

                                                if (arcorien != Internal && arcorien != External)
                                                {
                                                    ispassing = false;
                                                    tg3drst = tg2drst.X * v1 + tg2drst.Y * v2;
                                                    test = vectg.Dot(normale.Crossed(tg3drst));
                                                    test = test / (vectg.Magnitude);
                                                    test = test / ((normale.Crossed(tg3drst)).Magnitude);

                                                    vtxorien = domain.Orientation(vtx2);
                                                    if (Math.Abs(test) <= Tole)
                                                    {
                                                        tobeverified = true;
                                                        locTrans = External; // et pourquoi pas INTERNAL
                                                    }
                                                    else
                                                    {
                                                        if ((test > 0.0 && arcorien == Forward)
                                                            || (test < 0.0 && arcorien == Reversed))
                                                        {
                                                            locTrans = Forward;
                                                        }
                                                        else
                                                        {
                                                            locTrans = Reversed;
                                                        }
                                                        if (arcorien == Reversed)
                                                        {
                                                            tg3drst = tg3drst.Reversed();
                                                        } // deja fait????
                                                    }

                                                    comptrans.Compare(Tole, new Direction3D(tg3drst), bidnorm, 0.0, locTrans,
                                                                      vtxorien);
                                                }
                                                destination[k] = seqlength + 1;
                                            }
                                        }
                                    }
                                }
                                fairpt = true;
                                if (!ispassing)
                                {
                                    Containment before = comptrans.StateBefore();
                                    Containment after = comptrans.StateAfter();
                                    if ((before == Unknown) || (after == Unknown))
                                    {
                                        fairpt = false;
                                    }
                                    else if (before == global::Kerf.Topology.Containment.In)
                                    {
                                        if (after == global::Kerf.Topology.Containment.In)
                                        {
                                            ispassing = true;
                                        }
                                        else
                                        {
                                            vectg = vectg.Reversed();
                                            dirtg = dirtg.Reversed();
                                        }
                                    }
                                    else
                                    {
                                        if (after != global::Kerf.Topology.Containment.In)
                                        {
                                            fairpt = false;
                                        }
                                    }
                                }

                                // evite de partir le long d une restriction solution

                                if (fairpt && tobeverified)
                                {
                                    for (k = i; k <= nbPoints; k++)
                                    {
                                        if (destination[k] == seqlength + 1)
                                        {
                                            theparam = solrst.Point(k).Parameter();
                                            Curve2DAdaptorBase thearc2 = solrst.Point(k).Arc();
                                            arcorien = domain.Orientation(thearc2);

                                            if (arcorien == Forward || arcorien == Reversed)
                                            {
                                                ContourHCurve2dTool.D1(thearc2, theparam, out pt2d, out tg2drst);
                                                tg3drst = tg2drst.X * v1 + tg2drst.Y * v2;
                                                vtxorien = domain.Orientation(solrst.Point(k).Vertex());
                                                if ((arcorien == Forward && vtxorien == Reversed)
                                                    || (arcorien == Reversed && vtxorien == Forward))
                                                {
                                                    tg3drst = tg3drst.Reversed();
                                                }
                                                test = vectg.Normalized().Dot(tg3drst.Normalized());
                                                if (test >= 1.0 - Tole)
                                                {
                                                    fairpt = false;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }

                                if (fairpt)
                                {
                                    pPoint.SetDirections(vectg, dirtg);
                                    pPoint.SetPassing(ispassing);
                                    seqpdep.Append(pPoint);
                                    seqlength++;
                                }
                                else
                                { // il faut remettre en "ordre" si on ne garde pas le point.
                                    for (k = i; k <= nbPoints; k++)
                                    {
                                        if (destination[k] == seqlength + 1)
                                        {
                                            destination[k] = -destination[k];
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    private static WalkingTypeTrans ComputeTransitionOnLine(SurfFunction sFunc, double u, double v,
                                                             in Vector3D tgline)
    {
        HSurfaceTool.D1(sFunc.Surface(), u, v, out _, out Vector3D d1u, out Vector3D d1v);

        //------------------------------------------------------
        //--   Calcul de la tangente dans l espace uv        ---
        //------------------------------------------------------

        double det, d1uT, d1vT, normu2, normv2, d1ud1v, alpha, beta;
        d1uT = d1u.Dot(tgline);
        d1vT = d1v.Dot(tgline);
        normu2 = d1u.Dot(d1u);
        normv2 = d1v.Dot(d1v);
        d1ud1v = d1u.Dot(d1v);
        det = normu2 * normv2 - d1ud1v * d1ud1v;
        if (det < RealEpsilon())
        {
            //-- On ne doit pas passer ici !!
            return WalkingTypeTrans.Undecided;
        }

        alpha = (d1uT * normv2 - d1vT * d1ud1v) / det;
        beta = (normu2 * d1vT - d1ud1v * d1uT) / det;
        //-----------------------------------------------------
        //--  Calcul du Gradient de la fonction Utilisee     --
        //--  pour le contour apparent                       --
        //-----------------------------------------------------

        double v1, v2;
        math_Vector x = new(1, 2);
        Matrix df = new(1, 1, 1, 2);
        x[1] = u;
        x[2] = v;
        sFunc.Derivatives(x, df);
        v1 = df[1, 1];
        v2 = df[1, 2];

        //-----------------------------------------------------
        //-- On calcule si la fonction                       --
        //--        F(.) = Normale . Dir_Regard              --
        //-- Croit Losrque l on se deplace sur la Gauche     --
        //--  de la direction de deplacement sur la ligne.   --
        //-----------------------------------------------------

        det = -v1 * beta + v2 * alpha;

        if (det < RealEpsilon())
        { // revoir le test jag 940620
            return WalkingTypeTrans.Undecided;
        }
        if (det > 0.0)
        {
            return (WalkingTypeTrans.Out);
        }
        return (WalkingTypeTrans.In);
    }

    private static void ProcessSegments(TheSearch solrst, NCollection_Sequence<ContourLine> slin,
                                        double tolArc, SurfFunction sFunc, TopolTool domain)
    {
        int i, j, k;
        int nbedg = solrst.SegmentCount;
        int nblines, nbpts;

        ContourPoint ptvtx = new();

        ThePathPointOfTheSearch pStartf = new(), pStartl = new();

        bool dofirst, dolast, procf, procl;
        double paramf = 0.0, paraml = 0.0, u;
        ContourLine theline = new();

        Vector3D tgline = new(); //,norm1,norm2;

        Point2D p2d;

        for (i = 1; i <= nbedg; i++)
        {
            TheSegmentOfTheSearch thesegsol = solrst.Segment(i);
            theline.SetValue(thesegsol.Curve());

            // Traitement des points debut/fin du segment solution.

            dofirst = false;
            dolast = false;
            procf = false;
            procl = false;

            if (thesegsol.HasFirstPoint)
            {
                dofirst = true;
                pStartf = thesegsol.FirstPoint();
                paramf = pStartf.Parameter();
            }
            if (thesegsol.HasLastPoint)
            {
                dolast = true;
                pStartl = thesegsol.LastPoint();
                paraml = pStartl.Parameter();
            }

            // determination de la transition
            if (dofirst && dolast)
            {
                u = (paramf + paraml) / 2.0;
            }
            else if (dofirst)
            {
                u = paramf + 1.0;
            }
            else if (dolast)
            {
                u = paraml - 1.0;
            }
            else
            {
                u = 0.0;
            }

            ContourHCurve2dTool.D1(thesegsol.Curve(), u, out p2d, out Vector2D d2d);
            HSurfaceTool.D1(sFunc.Surface(), p2d.X, p2d.Y, out _, out Vector3D d1u, out Vector3D d1v);
            tgline = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
            WalkingTypeTrans tral = ComputeTransitionOnLine(sFunc, p2d.X, p2d.Y, tgline);

            theline.SetTransitionOnS(tral);

            if (dofirst || dolast)
            {
                nblines = slin.Length();
                for (j = 1; j <= nblines; j++)
                {
                    nbpts = slin[j].VertexCount;
                    for (k = 1; k <= nbpts; k++)
                    {
                        ptvtx = slin[j].Vertex(k);
                        if (dofirst)
                        {
                            if (ptvtx.Value().DistanceTo(pStartf.Value()) <= tolArc)
                            {
                                slin[j].Vertex(k).SetMultiple();
                                ptvtx.SetMultiple();
                                ptvtx.SetParameter(paramf);
                                theline.Add(ptvtx);
                                procf = true;
                            }
                        }
                        if (dolast)
                        {
                            if (ptvtx.Value().DistanceTo(pStartl.Value()) <= tolArc)
                            {
                                slin[j].Vertex(k).SetMultiple();
                                ptvtx.SetMultiple();
                                ptvtx.SetParameter(paraml);
                                theline.Add(ptvtx);
                                procl = true;
                            }
                        }
                    }
                    // Si on a traite le pt debut et/ou fin, on ne doit pas recommencer si
                    // il (ils) correspond(ent) a un point multiple.

                    if (procf)
                    {
                        dofirst = false;
                    }
                    if (procl)
                    {
                        dolast = false;
                    }
                }
            }

            // Si on n a pas trouve le point debut et./ou fin sur une des lignes
            // d intersection, il faut quand-meme le placer sur la restriction solution

            if (dofirst)
            {
                p2d = ContourHCurve2dTool.Value(thesegsol.Curve(), paramf);
                ptvtx.SetValue(pStartf.Value(), p2d.X, p2d.Y);
                ptvtx.SetParameter(paramf);
                if (!pStartf.IsNew())
                {
                    ptvtx.SetVertex(pStartf.Vertex());
                }
                theline.Add(ptvtx);
            }
            if (dolast)
            {
                p2d = ContourHCurve2dTool.Value(thesegsol.Curve(), paraml);
                ptvtx.SetValue(pStartl.Value(), p2d.X, p2d.Y);
                ptvtx.SetParameter(paraml);
                if (!pStartl.IsNew())
                {
                    ptvtx.SetVertex(pStartl.Vertex());
                }
                theline.Add(ptvtx);
            }

            // il faut chercher le points internal sur les restrictions solutions.
            if (thesegsol.HasFirstPoint && thesegsol.HasLastPoint)
            {
                ComputeInternalPointsOnRstr(theline, paramf, paraml, sFunc);
            }
            LineConstructor(slin, domain, theline, sFunc.Surface()); //-- lbr
            //-- slin.Append(theline);
            theline.Clear();
        }
    }

    private static void ComputeInternalPointsOnRstr(ContourLine line, double paramf, double paraml,
                                                    SurfFunction sFunc)
    {
        // On recherche les points ou la tangente a la ligne de contour et
        // la direction sont alignees.
        // 1ere etape : recherche de changement de signe.
        // 2eme etape : localisation de la solution par dichotomie

        int indexinf, indexsup, i;
        Vector3D tgt = new(), vecref = new(), vectest, vtestb, vecregard;
        Point3D pcour;
        Point2D p2d;
        Vector2D d2d;
        bool found, ok = false, toutvu, solution;
        double paramp = 0.0, paraminf, paramsup, toler;

        if (line.TypeContour() != ContourLineType.Restriction)
        {
            return;
        }

        Curve2DAdaptorBase thearc = line.Arc();

        SurfaceAdaptorBase surf = sFunc.Surface();
        ContourTFunction typeFunc = sFunc.FunctionType();

        int nbpnts = HContTool.NbSamplesOnArc(thearc);
        indexinf = 1;
        vecregard = new Vector3D(sFunc.Direction());
        toler = ContourHCurve2dTool.Resolution(thearc, global::Kerf.Engine.Foundation.Precision.Confusion());
        found = false;

        do
        {
            paraminf = ((nbpnts - indexinf) * paramf + (indexinf - 1) * paraml) / (nbpnts - 1);
            ContourHCurve2dTool.D1(thearc, paraminf, out p2d, out d2d);
            HSurfaceTool.D1(surf, p2d.X, p2d.Y, out pcour, out Vector3D d1u, out Vector3D d1v);
            tgt = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

            if (tgt.Magnitude > Constants.Resolution())
            {
                if (typeFunc == ContourTFunction.ContourPrs || typeFunc == ContourTFunction.DraftPrs)
                {
                    vecregard = vecregard.WithXYZ(pcour.XYZ() - sFunc.Eye().XYZ());
                }
                vecref = vecregard.Crossed(tgt);

                if (vecref.Magnitude <= Constants.Resolution())
                {
                    indexinf++;
                }
                else
                {
                    found = true;
                }
            }
            else
            {
                indexinf++;
            }
        } while ((indexinf <= nbpnts) && (!found));

        indexsup = indexinf + 1;
        toutvu = (indexsup > nbpnts);
        while (!toutvu)
        {
            paramsup = ((nbpnts - indexsup) * paramf + (indexsup - 1) * paraml) / (nbpnts - 1);
            ContourHCurve2dTool.D1(thearc, paramsup, out p2d, out d2d);
            HSurfaceTool.D1(surf, p2d.X, p2d.Y, out pcour, out Vector3D d1u, out Vector3D d1v);
            tgt = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

            if (tgt.Magnitude > Constants.Resolution())
            {
                if (typeFunc == ContourTFunction.ContourPrs || typeFunc == ContourTFunction.DraftPrs)
                {
                    vecregard = vecregard.WithXYZ(pcour.XYZ() - sFunc.Eye().XYZ());
                }
                vectest = vecregard.Crossed(tgt);
            }
            else
            {
                vectest = new Vector3D(0.0, 0.0, 0.0);
            }
            if (vectest.Magnitude <= Constants.Resolution())
            {
                // On cherche un vrai changement de signe
                indexsup++;
            }
            else
            {
                if (vectest.Dot(vecref) < 0.0)
                {
                    // Essayer de converger
                    solution = false;
                    while (!solution)
                    {
                        paramp = (paraminf + paramsup) / 2.0;
                        ContourHCurve2dTool.D1(thearc, paramp, out p2d, out d2d);
                        HSurfaceTool.D1(surf, p2d.X, p2d.Y, out pcour, out d1u, out d1v);
                        tgt = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

                        if (tgt.Magnitude > Constants.Resolution())
                        {
                            if (typeFunc == ContourTFunction.ContourPrs || typeFunc == ContourTFunction.DraftPrs)
                            {
                                vecregard = vecregard.WithXYZ(pcour.XYZ() - sFunc.Eye().XYZ());
                            }
                            vtestb = vecregard.Crossed(tgt);
                        }
                        else
                        {
                            vtestb = new Vector3D(0.0, 0.0, 0.0);
                        }

                        if ((vtestb.Magnitude <= Constants.Resolution()) || (Math.Abs(paramp - paraminf) <= toler)
                            || (Math.Abs(paramp - paramsup) <= toler))
                        {
                            // on est a la solution
                            solution = true;
                            ok = true;
                        }
                        else if (vtestb.Dot(vecref) < 0.0)
                        {
                            paramsup = paramp;
                        }
                        else
                        {
                            paraminf = paramp;
                        }
                    }

                    if (ok)
                    {
                        // On verifie que le point trouve ne correspond pas a un ou des
                        // vertex deja existant(s). On teste sur le parametre paramp.
                        for (i = 1; i <= line.VertexCount; i++)
                        {
                            ref ContourPoint thevtx = ref line.Vertex(i);
                            if (Math.Abs(thevtx.ParameterOnLine() - paramp) <= toler)
                            {
                                thevtx.SetInternal();
                                ok = false; // on a correspondence
                            }
                        }
                        if (ok)
                        { // il faut alors rajouter le point
                            ContourPoint internalp = new(pcour, p2d.X, p2d.Y);
                            internalp.SetParameter(paramp);
                            internalp.SetInternal();
                            line.Add(internalp);
                        }
                    }
                    paramsup = ((nbpnts - indexsup) * paramf + (indexsup - 1) * paraml) / (nbpnts - 1);
                }
                vecref = vectest;
                indexinf = indexsup;
                indexsup++;
                paraminf = paramsup;
            }
            toutvu = (indexsup > nbpnts);
        }
    }

    private static void ComputeInternalPoints(ContourLine line, SurfFunction sFunc, double ureso, double vreso)
    {
        // On recherche les points ou la tangente a la ligne de contour et
        // la direction sont alignees.
        // 1ere etape : recheche de changement de signe.
        // 2eme etape : localisation de la solution par simili dichotomie

        int indexinf, indexsup, index;
        Vector3D tgt, vecref = new(), vectest, vtestb, vecregard;
        bool found, ok = false, toutvu, solution;
        double paramp = 0.0, u, V;

        math_Vector xInf = new(1, 2), xSup = new(1, 2), x = new(1, 2), f = new(1, 1);
        Matrix df = new(1, 1, 1, 2);
        math_Vector toler = new(1, 2), infb = new(1, 2), supb = new(1, 2);

        if (line.TypeContour() != ContourLineType.Walking)
        {
            return;
        }

        int nbpnts = line.PntCount;
        SurfaceAdaptorBase surf = sFunc.Surface();
        ContourTFunction typeFunc = sFunc.FunctionType();

        toler[1] = ureso; //-- Trop long !!! Adaptor3d_HSurfaceTool::UResolution(Surf,SFunc.Tolerance());
        toler[2] = vreso; //---Beaucoup trop long !!! Adaptor3d_HSurfaceTool::VResolution(Surf,SFunc.Tolerance());
        infb[1] = HSurfaceTool.FirstUParameter(surf);
        infb[2] = HSurfaceTool.FirstVParameter(surf);
        supb[1] = HSurfaceTool.LastUParameter(surf);
        supb[2] = HSurfaceTool.LastVParameter(surf);

        FunctionSetRoot rsnld = new(sFunc, toler, 30);

        indexinf = 1;
        vecregard = new Vector3D(sFunc.Direction());

        found = false;
        do
        {
            line.Point(indexinf).ParametersOnS2(out xInf[1], out xInf[2]);
            sFunc.Values(xInf, f, df);
            if (!sFunc.IsTangent())
            {
                tgt = sFunc.Direction3d();
                if (typeFunc == ContourTFunction.ContourPrs || typeFunc == ContourTFunction.DraftPrs)
                {
                    vecregard = vecregard.WithXYZ(line.Point(indexinf).Value().XYZ() - sFunc.Eye().XYZ());
                }
                vecref = vecregard.Crossed(tgt);

                if (vecref.Magnitude <= Constants.Resolution())
                {
                    indexinf++;
                }
                else
                {
                    found = true;
                }
            }
            else
            {
                indexinf++;
            }
        } while ((indexinf <= nbpnts) && (!found));

        indexsup = indexinf + 1;
        toutvu = (indexsup > nbpnts);
        while (!toutvu)
        {
            line.Point(indexsup).ParametersOnS2(out xSup[1], out xSup[2]);
            sFunc.Values(xSup, f, df);
            if (!sFunc.IsTangent())
            {
                tgt = sFunc.Direction3d();

                if (typeFunc == ContourTFunction.ContourPrs || typeFunc == ContourTFunction.DraftPrs)
                {
                    vecregard = vecregard.WithXYZ(line.Point(indexsup).Value().XYZ() - sFunc.Eye().XYZ());
                }
                vectest = vecregard.Crossed(tgt);
            }
            else
            {
                vectest = new Vector3D(0.0, 0.0, 0.0);
            }
            if (vectest.Magnitude <= Constants.Resolution())
            {
                // On cherche un vrai changement de signe
                indexsup++;
            }
            else
            {
                if (vectest.Dot(vecref) < 0.0)
                {
                    // Essayer de converger
                    solution = false;
                    while (!solution)
                    {
                        // Selecting the middle point between XInf and XSup leads situation, where X values almost
                        // do not change. To prevent this situation, select shifted point instead of middle.
                        const double coef = 2.0 / 3.0;
                        x[1] = xInf[1] + coef * (xSup[1] - xInf[1]);
                        x[2] = xInf[2] + coef * (xSup[2] - xInf[2]);
                        rsnld.Perform(sFunc, x, infb, supb);

                        if (!rsnld.IsDone)
                        {
                            // OCCT: std::cout << "Echec recherche internal points" (left out)
                            solution = true;
                            ok = false;
                        }
                        else
                        {
                            rsnld.Root(x);
                            sFunc.Values(x, f, df);
                            if (Math.Abs(f[1]) <= sFunc.Tolerance())
                            {
                                if (!sFunc.IsTangent())
                                {
                                    tgt = sFunc.Direction3d();
                                    if (typeFunc == ContourTFunction.ContourPrs || typeFunc == ContourTFunction.DraftPrs)
                                    {
                                        vecregard = vecregard.WithXYZ(sFunc.Point().XYZ() - sFunc.Eye().XYZ());
                                    }
                                    vtestb = vecregard.Crossed(tgt);
                                }
                                else
                                {
                                    vtestb = new Vector3D(0.0, 0.0, 0.0);
                                }
                                if ((vtestb.Magnitude <= Constants.Resolution())
                                    || (Math.Abs(x[1] - xInf[1]) <= toler[1] && Math.Abs(x[2] - xInf[2]) <= toler[2])
                                    || (Math.Abs(x[1] - xSup[1]) <= toler[1] && Math.Abs(x[2] - xSup[2]) <= toler[2]))
                                {
                                    // on est a la solution
                                    solution = true;
                                    ok = true;
                                }
                                else if (vtestb.Dot(vecref) < 0.0)
                                {
                                    xSup.Initialized(x);
                                }
                                else
                                {
                                    xInf.Initialized(x);
                                }
                            }
                            else
                            { // on n est pas sur une solution
                                // OCCT: std::cout << "Echec recherche internal points" (left out)
                                solution = true;
                                ok = false;
                            }
                        }
                    }

                    if (ok)
                    {
                        bool newpoint = false;
                        line.Point(indexinf).ParametersOnS2(out u, out V);
                        Vector2D vinf = new(x[1] - u, x[2] - V);
                        if (Math.Abs(vinf.X) <= toler[1] && Math.Abs(vinf.Y) <= toler[2])
                        {
                            paramp = indexinf;
                        }
                        else
                        {
                            for (index = indexinf + 1; index <= indexsup; index++)
                            {
                                line.Point(index).ParametersOnS2(out u, out V);
                                Vector2D vsup = new(x[1] - u, x[2] - V);
                                if (Math.Abs(vsup.X) <= toler[1] && Math.Abs(vsup.Y) <= toler[2])
                                {
                                    paramp = index;
                                    break;
                                }
                                else if (vinf.Dot(vsup) < 0.0)
                                {
                                    // on est entre les 2 points
                                    paramp = index;
                                    PntOn2S pt2s = new();
                                    pt2s.SetValue(sFunc.Point(), false, x[1], x[2]);
                                    line.LineOn2S().InsertBefore(index, pt2s);

                                    //-- Il faut decaler les parametres des vertex situes entre
                                    //-- index et NbPnts ###################################
                                    for (int vv = 1; vv <= line.VertexCount; vv++)
                                    {
                                        ref ContourPoint vertex = ref line.Vertex(vv);
                                        if (vertex.ParameterOnLine() >= index)
                                        {
                                            vertex.SetParameter(vertex.ParameterOnLine() + 1);
                                        }
                                    }

                                    nbpnts = nbpnts + 1;
                                    indexsup = indexsup + 1;
                                    newpoint = true;
                                    break;
                                }
                                else
                                {
                                    vinf = vsup;
                                }
                            }
                        }

                        int v;
                        if (!newpoint)
                        {
                            // on est sur un point de cheminement. On regarde alors
                            // la correspondance avec un vertex existant.
                            newpoint = true;
                            for (v = 1; v <= line.VertexCount; v++)
                            {
                                ref ContourPoint vertex = ref line.Vertex(v);
                                if (vertex.ParameterOnLine() == paramp)
                                {
                                    vertex.SetInternal();
                                    newpoint = false;
                                }
                            }
                        }

                        if (newpoint && paramp > 1.0 && paramp < nbpnts)
                        {
                            // on doit creer un nouveau vertex.
                            ContourPoint internalp = new(sFunc.Point(), x[1], x[2]);
                            internalp.SetParameter(paramp);
                            internalp.SetInternal();
                            line.Add(internalp);
                        }
                    }
                    line.Point(indexsup).ParametersOnS2(out xSup[1], out xSup[2]);
                }
                vecref = vectest;
                indexinf = indexsup;
                indexsup++;
                xInf.Initialized(xSup);
            }
            toutvu = (indexsup > nbpnts);
        }
    }

    private void Perform(TopolTool domain)
    {
        _done = false;
        _slin.Clear();

        int i, j, k, nbvt1, nbvt2, ivt1, ivt2;
        int nbPointRst, nbPointIns;
        int nblines, nbpts, indfirst, indlast;
        double u, v;
        Point2D pt2d;
        Vector2D d2d;
        Point3D ptonsurf;
        Vector3D d1u, d1v, normale, tgtrst, tgline;
        double currentparam;
        WalkingTransition tLine = new(), tArc = new();

        ContourLine theline = new();
        ContourPoint ptdeb = new(), ptfin = new();

        //  double TolArc = 1.e-5;
        double tolArc = global::Kerf.Engine.Foundation.Precision.Confusion();

        SurfaceAdaptorBase surf = _sFunc.Surface();

        double epsU = HSurfaceTool.UResolution(surf, global::Kerf.Engine.Foundation.Precision.Confusion());
        double epsV = HSurfaceTool.VResolution(surf, global::Kerf.Engine.Foundation.Precision.Confusion());
        double preci = Min(epsU, epsV);
        //  double Fleche = 5.e-1;
        //  double Pas    = 5.e-2;
        double fleche = 0.01;
        double pas = 0.005;
        //   lbr: Il y avait Pas 0.2 -> Manque des Inters sur restr ; devrait faire un mini de 5 pts par
        //   lignes
        //-- le 23 janvier 98 0.05 -> 0.01

        //-- ******************************************************************************** Janvier 98
        Box b1 = new();
        bool box1OK = true;

        double uinf = surf.FirstUParameter();
        double vinf = surf.FirstVParameter();
        double usup = surf.LastUParameter();
        double vsup = surf.LastVParameter();

        bool uinfinfinite = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uinf);
        bool usupinfinite = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(usup);
        bool vinfinfinite = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vinf);
        bool vsupinfinite = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vsup);

        if (uinfinfinite || usupinfinite || vinfinfinite || vsupinfinite)
        {
            box1OK = false;
        }
        else
        {
            AddSurface.Add(surf, 1e-8, ref b1);
        }
        double dx, dy, dz;
        if (box1OK)
        {
            b1.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
            dx = x1 - x0;
            dy = y1 - y0;
            dz = z1 - z0;
        }
        else
        {
            dx = dy = dz = 1.0;
        }
        if (dx < dy)
        {
            dx = dy;
        }
        if (dx < dz)
        {
            dx = dz;
        }
        if (dx > 10000.0)
        {
            dx = 10000.0;
        }
        fleche *= dx;
        tolArc *= dx;
        //-- ********************************************************************************

        // jag 940616  SFunc.Set(1.e-8); // tolerance sur la fonction
        _sFunc.Set(global::Kerf.Engine.Foundation.Precision.Confusion()); // tolerance sur la fonction

        bool recheckOnRegularity = true;
        _solrst.Perform(_aFunc, domain, tolArc, tolArc, recheckOnRegularity);

        if (!_solrst.IsDone)
        {
            return;
        }

        nbPointRst = _solrst.PointCount;
        NCollection_Sequence<PathPoint> seqpdep = new();
        NCollection_Array1<int> destination = new(1, nbPointRst + 1);
        destination.Init(0);
        if (nbPointRst != 0)
        {
            ComputeTangency(_solrst, domain, _sFunc, seqpdep, destination);
        }

        // jag 940616  solins.Perform(SFunc,Surf,Domain,1.e-6); // 1.e-6 : tolerance dans l espace.
        _solins.Perform(_sFunc, surf, domain, global::Kerf.Engine.Foundation.Precision.Confusion());

        nbPointIns = _solins.PointCount;
        NCollection_Sequence<InteriorPoint> seqpins = new();

        if (nbPointIns != 0)
        {
            bool bKeepAllPoints = false;
            // IFV begin
            if (_solrst.SegmentCount <= 0)
            {
                if (_sFunc.FunctionType() == ContourTFunction.ContourStd)
                {
                    SurfaceAdaptorBase surfToCheck = _sFunc.Surface();
                    if (HSurfaceTool.GetType(surfToCheck) == SurfaceKind.Torus)
                    {
                        Torus3D tor = HSurfaceTool.Torus(surfToCheck);
                        Direction3D torDir = tor.Axis.Direction;
                        Direction3D projDir = _sFunc.Direction();

                        if (torDir.Dot(projDir) < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            bKeepAllPoints = true;
                        }
                    }
                }
            }

            if (bKeepAllPoints)
            {
                int nbp = _solins.PointCount, indp;
                for (indp = 1; indp <= nbp; indp++)
                {
                    InteriorPoint pti = _solins.Value(indp);
                    seqpins.Append(pti);
                }
            }
            // IFV - end
            else
            {
                KeepInsidePoints(_solins, _solrst, _sFunc, seqpins);
            }
        }

        if (seqpdep.Length() != 0 || seqpins.Length() != 0)
        {
            bool theToFillHoles = true;
            TheIWalking iwalk = new(preci, fleche, pas, theToFillHoles);
            iwalk.Perform(seqpdep, seqpins, _sFunc, surf);
            if (!iwalk.IsDone)
            {
                return;
            }

            nblines = iwalk.LineCount;
            for (j = 1; j <= nblines; j++)
            {
                WalkingTypeTrans typeTransOnS = WalkingTypeTrans.Undecided;
                TheIWLineOfTheIWalking iwline = iwalk.Value(j);
                nbpts = iwline.PointCount;
                theline.SetLineOn2S(iwline.Line());

                // jag 941018 On calcule une seule fois la transition

                tgline = iwline.TangentVector(out k);
                iwline.Line().Value(k).ParametersOnS2(out u, out v);
                typeTransOnS = ComputeTransitionOnLine(_sFunc, u, v, tgline);
                theline.SetTransitionOnS(typeTransOnS);

                //---------------------------------------------------------------------
                //-- On ajoute a la liste des vertex les 1er et dernier points de la  -
                //-- ligne de cheminement si ceux-ci ne sont pas presents             -
                //---------------------------------------------------------------------

                if (iwline.HasFirstPoint)
                {
                    indfirst = iwline.FirstPointIndex();
                    PathPoint pPoint = seqpdep.Value(indfirst);
                    int themult = pPoint.Multiplicity();
                    for (i = nbPointRst; i >= 1; i--)
                    {
                        if (destination[i] == indfirst)
                        {
                            pPoint.Parameters(themult, out u, out v);
                            ptdeb.SetValue(pPoint.Value(), u, v);
                            ptdeb.SetParameter(1.0);

                            ThePathPointOfTheSearch pStart = _solrst.Point(i);
                            Curve2DAdaptorBase currentarc = pStart.Arc();
                            currentparam = pStart.Parameter();
                            if (!iwline.IsTangentAtBegining)
                            {
                                ContourHCurve2dTool.D1(currentarc, currentparam, out pt2d, out d2d);
                                SurfProps.DerivAndNorm(surf, pt2d.X, pt2d.Y, out ptonsurf, out d1u, out d1v,
                                                              out normale);
                                tgtrst = d2d.X * d1u;
                                tgtrst = tgtrst.Added(d2d.Y * d1v);

                                global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(pPoint.Direction3d(), tgtrst, new Direction3D(normale), ref tLine,
                                                       ref tArc);
                            }
                            else
                            { // a voir. En effet, on a cheminer. Si on est sur un point
                              // debut, on sait qu'on rentre dans la matiere
                                tLine.SetValue();
                                tArc.SetValue();
                            }

                            ptdeb.SetArc(currentarc, currentparam, tLine, tArc);

                            if (!_solrst.Point(i).IsNew())
                            {
                                ptdeb.SetVertex(pStart.Vertex());
                            }
                            theline.Add(ptdeb);
                            themult--;
                        }
                    }
                }
                else
                {
                    iwline.Value(1).ParametersOnS2(out u, out v);
                    ptdeb.SetValue(theline.Point(1).Value(), u, v);
                    ptdeb.SetParameter(1.0);
                    theline.Add(ptdeb);
                }

                if (iwline.HasLastPoint)
                {
                    indlast = iwline.LastPointIndex();
                    PathPoint pPoint = seqpdep.Value(indlast);
                    int themult = pPoint.Multiplicity();
                    for (i = nbPointRst; i >= 1; i--)
                    {
                        if (destination[i] == indlast)
                        {
                            pPoint.Parameters(themult, out u, out v);
                            ptfin.SetValue(pPoint.Value(), u, v);
                            ptfin.SetParameter((double)(nbpts));
                            ThePathPointOfTheSearch pStart = _solrst.Point(i);
                            Curve2DAdaptorBase currentarc = pStart.Arc();
                            currentparam = pStart.Parameter();

                            if (!iwline.IsTangentAtEnd)
                            {
                                ContourHCurve2dTool.D1(currentarc, currentparam, out pt2d, out d2d);

                                SurfProps.DerivAndNorm(surf, pt2d.X, pt2d.Y, out ptonsurf, out d1u, out d1v,
                                                              out normale);
                                tgtrst = d2d.X * d1u;
                                tgtrst = tgtrst.Added(d2d.Y * d1v);
                                global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(pPoint.Direction3d().Reversed(), tgtrst, new Direction3D(normale),
                                                       ref tLine, ref tArc);
                            }
                            else
                            {
                                tLine.SetValue();
                                tArc.SetValue();
                            }

                            ptfin.SetArc(currentarc, currentparam, tLine, tArc);

                            if (!_solrst.Point(i).IsNew())
                            {
                                ptfin.SetVertex(pStart.Vertex());
                            }
                            theline.Add(ptfin);
                            themult--;
                        }
                    }
                }
                else
                {
                    iwline.Value(nbpts).ParametersOnS2(out u, out v);
                    ptfin.SetValue(theline.Point(nbpts).Value(), u, v);
                    ptfin.SetParameter((double)(nbpts));
                    theline.Add(ptfin);
                }

                ComputeInternalPoints(theline, _sFunc, epsU, epsV);
                LineConstructor(_slin, domain, theline, surf); //-- lbr
                //-- slin.Append(theline);
                theline.ResetSeqOfVertex();
            }

            nblines = _slin.Length();
            for (j = 1; j <= nblines - 1; j++)
            {
                ContourLine theli = _slin.Value(j);
                nbvt1 = theli.VertexCount;
                for (ivt1 = 1; ivt1 <= nbvt1; ivt1++)
                {
                    if (!theli.Vertex(ivt1).IsOnArc)
                    {
                        Point3D pttg1 = theli.Vertex(ivt1).Value();

                        for (k = j + 1; k <= nblines; k++)
                        {
                            ContourLine theli2 = _slin.Value(k);
                            nbvt2 = theli2.VertexCount;
                            for (ivt2 = 1; ivt2 <= nbvt2; ivt2++)
                            {
                                if (!theli2.Vertex(ivt2).IsOnArc)
                                {
                                    Point3D pttg2 = theli2.Vertex(ivt2).Value();

                                    if (pttg1.DistanceTo(pttg2) <= tolArc)
                                    {
                                        theli.Vertex(ivt1).SetMultiple();
                                        theli2.Vertex(ivt2).SetMultiple();
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // jag 940620 On ajoute le traitement des restrictions solutions.

        if (_solrst.SegmentCount != 0)
        {
            ProcessSegments(_solrst, _slin, tolArc, _sFunc, domain);
        }

        // Ajout crad pour depanner CMA en attendant mieux
        if (_solrst.SegmentCount != 0)
        {
            nblines = _slin.Length();
            for (j = 1; j <= nblines; j++)
            {
                ContourLine theli = _slin.Value(j);
                if (theli.TypeContour() == ContourLineType.Walking)
                {
                    nbvt1 = theli.VertexCount;
                    for (ivt1 = 1; ivt1 <= nbvt1; ivt1++)
                    {
                        ref ContourPoint ptvt = ref theli.Vertex(ivt1);
                        if (!ptvt.IsOnArc && !ptvt.IsMultiple)
                        {
                            ptvt.Parameters(out double up, out double vp);
                            Point2D toproj = new(up, vp);
                            bool projok;
                            for (k = 1; k <= nblines; k++)
                            {
                                if (_slin.Value(k).TypeContour() == ContourLineType.Restriction)
                                {
                                    Curve2DAdaptorBase thearc = _slin.Value(k).Arc();
                                    projok = HContTool.Project(thearc, toproj, out double paramproj,
                                                                      out Point2D ptproj);

                                    if (projok)
                                    {
                                        double dist = ptproj.DistanceTo(new Point2D(up, vp));
                                        if (dist <= preci)
                                        {
                                            // Calcul de la transition

                                            ContourHCurve2dTool.D1(thearc, paramproj, out ptproj, out d2d);
                                            //		    Adaptor3d_HSurfaceTool::D1(Surf,Ptproj.X(),Ptproj.Y(),
                                            //				       ptonsurf,d1u,d1v);
                                            //		    normale = d1u.Crossed(d1v);

                                            SurfProps.DerivAndNorm(surf, ptproj.X, ptproj.Y, out ptonsurf,
                                                                          out d1u, out d1v, out normale);

                                            tgtrst = d2d.X * d1u;
                                            tgtrst = tgtrst.Added(d2d.Y * d1v);
                                            int paraml = (int)ptvt.ParameterOnLine();

                                            if (paraml == theli.PntCount)
                                            {
                                                tgline = new Vector3D(theli.Point(paraml - 1).Value(), ptvt.Value());
                                            }
                                            else
                                            {
                                                tgline = new Vector3D(ptvt.Value(), theli.Point(paraml + 1).Value());
                                            }
                                            global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(tgline, tgtrst, new Direction3D(normale), ref tLine,
                                                                   ref tArc);
                                            ptvt.SetArc(thearc, paramproj, tLine, tArc);
                                            ptvt.SetMultiple();
                                            ptdeb.SetValue(ptonsurf, ptproj.X, ptproj.Y);
                                            ptdeb.SetParameter(paramproj);
                                            ptdeb.SetMultiple();
                                            _slin.Value(k).Add(ptdeb);
                                            break;
                                        }
                                        else
                                        {
                                            projok = false;
                                        }
                                    }
                                }
                                else
                                {
                                    projok = false;
                                }
                                if (projok)
                                {
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
        _done = true;
    }

    private static bool FindLine(ContourLine line, SurfaceAdaptorBase surf, in Point2D pt2d, out Point3D ptref,
                                 ref double paramin, ref Vector3D tgmin, out Vector3D norm)
    {
        Point3D pt, ptmin = new();
        Vector3D tg;
        double para, dist;
        double dismin = RealLast();

        SurfProps.Normale(surf, pt2d.X, pt2d.Y, out ptref, out norm);

        if (line.TypeContour() == ContourLineType.Lin)
        {
            Line3D lin = line.Line();
            para = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, ptref);
            global::Kerf.Engine.Geometry.ElCLib.D1(para, lin, out pt, out tg);
            dist = pt.DistanceTo(ptref) + Math.Abs(norm.Dot(new Vector3D(lin.Direction)));
        }
        else
        { // Contap__Circle
            Circle3D cir = line.Circle();
            para = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, ptref);
            global::Kerf.Engine.Geometry.ElCLib.D1(para, cir, out pt, out tg);
            dist = pt.DistanceTo(ptref) + Math.Abs(norm.Dot(tg / cir.Radius));
        }
        if (dist < dismin)
        {
            dismin = dist;
            paramin = para;
            ptmin = pt;
            tgmin = tg;
        }
        return ptmin.SquareDistanceTo(ptref) <= Tolpetit;
    }

    private static void PutPointsOnLine(TheSearch solrst, SurfaceAdaptorBase surf,
                                        NCollection_Sequence<ContourLine> slin)
    {
        int i, l; //,index;
        int nbPoints = solrst.PointCount;

        double theparam;

        WalkingTransition tLine = new(), tArc = new();
        bool goon;

        Point2D pt2d;
        Vector2D d2d;

        Vector3D vectg = new(), tgtrst = new();
        double paramlin = 0.0;

        int nbLin = slin.Length();
        for (l = 1; l <= nbLin; l++)
        {
            ContourLine line = slin.Value(l);
            for (i = 1; i <= nbPoints; i++)
            {
                ThePathPointOfTheSearch pStart = solrst.Point(i);
                Curve2DAdaptorBase thearc = pStart.Arc();
                theparam = pStart.Parameter();

                ContourHCurve2dTool.D1(thearc, theparam, out pt2d, out d2d);
                goon = FindLine(line, surf, pt2d, out Point3D ptonsurf, ref paramlin, ref vectg, out Vector3D normale);

                ContourPoint pPoint = new();

                if (goon)
                {
                    HSurfaceTool.D1(surf, pt2d.X, pt2d.Y, out _, out Vector3D d1u, out Vector3D d1v);
                    pPoint.SetValue(ptonsurf, pt2d.X, pt2d.Y);
                    if (normale.Magnitude < RealEpsilon())
                    {
                        tLine.SetValue();
                        tArc.SetValue();
                    }
                    else
                    {
                        // Petit test qui devrait permettre de bien traiter les pointes
                        // des cones, et les sommets d'une sphere. Il faudrait peut-etre
                        // rajouter une methode dans SurfProps

                        if (Math.Abs(d2d.Y) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            tgtrst = d1v.Crossed(normale);
                            if (d2d.X < 0.0)
                            {
                                tgtrst = tgtrst.Reversed();
                            }
                        }
                        else
                        {
                            tgtrst = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
                        }
                        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(vectg, tgtrst, new Direction3D(normale), ref tLine, ref tArc);
                    }

                    pPoint.SetArc(thearc, theparam, tLine, tArc);
                    pPoint.SetParameter(paramlin);
                    if (!pStart.IsNew())
                    {
                        pPoint.SetVertex(pStart.Vertex());
                    }
                    line.Add(pPoint);
                }
            }
        }
    }

    //----------------------------------------------------------------------------------
    //-- Orientation des contours Apparents quand ceux-ci sont des lignes ou des cercles
    //-- On prend un point de la ligne ou du cercle ---> P
    //-- On projete ce point sur la surface P ---> u,v
    //-- et on evalue la transition au point u,v
    //----------------------------------------------------------------------------------

    private static WalkingTypeTrans ComputeTransitionOngpLine(SurfFunction sFunc, in Line3D l)
    {
        SurfaceAdaptorBase surf = sFunc.Surface();
        SurfaceKind typS = HSurfaceTool.GetType(surf);
        global::Kerf.Engine.Geometry.ElCLib.D1(0.0, l, out Point3D p, out Vector3D t);
        double u = 0.0, v = 0.0;
        switch (typS)
        {
            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Cylinder(surf), p, out u, out v);
                break;
            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Cone(surf), p, out u, out v);
                break;
            case SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Sphere(surf), p, out u, out v);
                break;
            default:
                break;
        }
        return (ComputeTransitionOnLine(sFunc, u, v, t));
    }

    private static WalkingTypeTrans ComputeTransitionOngpCircle(SurfFunction sFunc, in Circle3D c)
    {
        SurfaceAdaptorBase surf = sFunc.Surface();
        SurfaceKind typS = HSurfaceTool.GetType(surf);
        global::Kerf.Engine.Geometry.ElCLib.D1(0.0, c, out Point3D p, out Vector3D t);
        double u = 0.0, v = 0.0;
        switch (typS)
        {
            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Cylinder(surf), p, out u, out v);
                break;
            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Cone(surf), p, out u, out v);
                break;
            case SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(HSurfaceTool.Sphere(surf), p, out u, out v);
                break;
            default:
                break;
        }
        return (ComputeTransitionOnLine(sFunc, u, v, t));
    }

    private void PerformAna(TopolTool domain)
    {
        _done = false;
        _slin.Clear();

        double tolArc = 1.0e-5;

        int nbCont, nbPointRst, i;
        ContAna contana = new();
        ContourLine theline = new();
        SurfaceAdaptorBase surf = _sFunc.Surface();
        ContourTFunction typeFunc = _sFunc.FunctionType();
        bool performSolRst = true;

        SurfaceKind typS = HSurfaceTool.GetType(surf);

        switch (typS)
        {
            case Plane:
            {
                Plane3D pl = HSurfaceTool.Plane(surf);
                switch (typeFunc)
                {
                    case ContourTFunction.ContourStd:
                    {
                        Direction3D dirpln = pl.Axis.Direction;
                        if (Math.Abs(_sFunc.Direction().Dot(dirpln)) > global::Kerf.Engine.Foundation.Precision.Angular())
                        {
                            // Aucun point du plan n'est solution, en particulier aucun point
                            // sur restriction.
                            performSolRst = false;
                        }
                    }
                    break;
                    case ContourTFunction.ContourPrs:
                    {
                        Point3D eye = _sFunc.Eye();
                        if (pl.DistanceTo(eye) > global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            // Aucun point du plan n'est solution, en particulier aucun point
                            // sur restriction.
                            performSolRst = false;
                        }
                    }
                    break;
                    case ContourTFunction.DraftStd:
                    {
                        Direction3D dirpln = pl.Axis.Direction;
                        double sina = Math.Sin(_sFunc.Angle());
                        if (Math.Abs(_sFunc.Direction().Dot(dirpln) + sina) > // voir SurfFunction
                            global::Kerf.Engine.Foundation.Precision.Angular())
                        {
                            performSolRst = false;
                        }
                    }
                    break;
                    case ContourTFunction.DraftPrs:
                    default:
                        break;
                }
            }
            break;

            case SurfaceKind.Sphere:
            {
                switch (typeFunc)
                {
                    case ContourTFunction.ContourStd:
                        contana.Perform(HSurfaceTool.Sphere(surf), _sFunc.Direction());
                        break;
                    case ContourTFunction.ContourPrs:
                        contana.Perform(HSurfaceTool.Sphere(surf), _sFunc.Eye());
                        break;
                    case ContourTFunction.DraftStd:
                        contana.Perform(HSurfaceTool.Sphere(surf), _sFunc.Direction(), _sFunc.Angle());
                        break;
                    case ContourTFunction.DraftPrs:
                    default:
                        break;
                }
            }
            break;

            case SurfaceKind.Cylinder:
            {
                switch (typeFunc)
                {
                    case ContourTFunction.ContourStd:
                        contana.Perform(HSurfaceTool.Cylinder(surf), _sFunc.Direction());
                        break;
                    case ContourTFunction.ContourPrs:
                        contana.Perform(HSurfaceTool.Cylinder(surf), _sFunc.Eye());
                        break;
                    case ContourTFunction.DraftStd:
                        contana.Perform(HSurfaceTool.Cylinder(surf), _sFunc.Direction(), _sFunc.Angle());
                        break;
                    case ContourTFunction.DraftPrs:
                    default:
                        break;
                }
            }
            break;

            case SurfaceKind.Cone:
            {
                switch (typeFunc)
                {
                    case ContourTFunction.ContourStd:
                        contana.Perform(HSurfaceTool.Cone(surf), _sFunc.Direction());
                        break;
                    case ContourTFunction.ContourPrs:
                        contana.Perform(HSurfaceTool.Cone(surf), _sFunc.Eye());
                        break;
                    case ContourTFunction.DraftStd:
                        contana.Perform(HSurfaceTool.Cone(surf), _sFunc.Direction(), _sFunc.Angle());
                        break;
                    case ContourTFunction.DraftPrs:
                    default:
                        break;
                }
            }
            break;
            default:
                break;
        }

        if (typS != Plane)
        {
            if (!contana.IsDone)
            {
                return;
            }

            nbCont = contana.ContourCount;

            if (contana.ContourCount == 0)
            {
                _done = true;
                return;
            }

            CurveKind typL = contana.TypeContour();
            if (typL == global::Kerf.Geometry.CurveKind.Circle)
            {
                theline.SetValue(contana.Circle());
                WalkingTypeTrans transCircle;
                transCircle = ComputeTransitionOngpCircle(_sFunc, contana.Circle());
                theline.SetTransitionOnS(transCircle);
                _slin.Append(new ContourLine(theline));
            }
            else if (typL == global::Kerf.Geometry.CurveKind.Line)
            {
                for (i = 1; i <= nbCont; i++)
                {
                    theline.SetValue(contana.Line(i));
                    WalkingTypeTrans transLine;
                    transLine = ComputeTransitionOngpLine(_sFunc, contana.Line(i));
                    theline.SetTransitionOnS(transLine);
                    _slin.Append(new ContourLine(theline));
                    theline.Clear();
                }
            }
        }

        if (performSolRst)
        {
            _solrst.Perform(_aFunc, domain, tolArc, tolArc);
            if (!_solrst.IsDone)
            {
                return;
            }
            nbPointRst = _solrst.PointCount;

            if (nbPointRst != 0)
            {
                PutPointsOnLine(_solrst, surf, _slin);
            }

            if (_solrst.SegmentCount != 0)
            {
                ProcessSegments(_solrst, _slin, tolArc, _sFunc, domain);
            }

            //-- lbr
            int nblinto = _slin.Length();
            NCollection_Sequence<int> seqToDestroy = new();

            for (i = 1; i <= nblinto; i++)
            {
                if (_slin.Value(i).TypeContour() != ContourLineType.Restriction)
                {
                    LineConstructor(_slin, domain, _slin.Value(i), surf);
                    seqToDestroy.Append(i);
                }
            }
            for (i = seqToDestroy.Length(); i >= 1; i--)
            {
                _slin.Remove(seqToDestroy.Value(i));
            }
        }

        _done = true;
    }
}
