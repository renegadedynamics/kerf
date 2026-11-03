// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ALineToWLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ALineToWLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Patches.SpecPntType;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// Converts an analytic intersection line (IntPatch_ALine) of two quadrics into walking lines: points sampled along
/// the line with a step adapted to its curvature, the line's vertices inserted, split at poles and apexes.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. Its quadrics are copies (structs) built in the constructor from the surfaces.
/// </remarks>
public class ALineToWLine
{
    private readonly SurfaceAdaptorBase _s1;
    private readonly SurfaceAdaptorBase _s2;
    private SurfaceQuadric _quad1 = new();
    private SurfaceQuadric _quad2 = new();

    private readonly int _nbPointsInWline;
    private double _tolOpenDomain;
    private double _tolTransition;
    private double _tol3D;

    //=================================================================================================

    private static void AddPointIntoLine(LineOn2S line, ReadOnlySpan<double> arrPeriods,
                                         ref PntOn2S point)
    {
        if (line.PointCount > 0)
        {
            if (point.IsSame(line.Value(line.PointCount), global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                return;
            }

            SpecialPoints.AdjustPointAndVertex(line.Value(line.PointCount), arrPeriods,
                                                        ref point);
        }

        line.Add(point);
    }

    // OCCT's AddPointIntoLine with a non-null vertex pointer.
    private static void AddPointIntoLine(LineOn2S line, ReadOnlySpan<double> arrPeriods,
                                         ref PntOn2S point, ref PatchPoint vertex)
    {
        if (line.PointCount > 0)
        {
            if (point.IsSame(line.Value(line.PointCount), global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                return;
            }

            SpecialPoints.AdjustPointAndVertex(line.Value(line.PointCount), arrPeriods,
                                                        ref point, ref vertex);
        }

        line.Add(point);
    }

    //=======================================================================
    // function : AddVertexPoint
    // purpose  : Extracts IntSurf_PntOn2S from theVertex and adds result in theLine.
    //=======================================================================
    private static void AddVertexPoint(LineOn2S line, ref PatchPoint vertex,
                                       ReadOnlySpan<double> arrPeriods)
    {
        PntOn2S apexPoint = vertex.PntOn2S();
        AddPointIntoLine(line, arrPeriods, ref apexPoint, ref vertex);
    }

    //=======================================================================
    // function : IsPoleOrSeam
    // purpose  : Processes theVertex depending on its type
    //            (pole/apex/point on boundary etc.) and adds it in theLine.
    //           thePIsoRef is the reference point using in case when the
    //            value of correspond parameter cannot be precise.
    //           theSingularSurfaceID contains the ID of surface with
    //            special point (0 - none, 1 - theS1, 2 - theS2)
    //=======================================================================
    private static SpecPntType IsPoleOrSeam(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2,
                                                     in PntOn2S pIsoRef, LineOn2S line,
                                                     ref PatchPoint vertex, ReadOnlySpan<double> arrPeriods,
                                                     double tol3d, out int singularSurfaceID)
    {
        singularSurfaceID = 0;

        for (int i = 0; i < 2; i++)
        {
            bool isReversed = (i > 0);
            SurfaceKind type = isReversed ? s2.GetType() : s1.GetType();

            SpecPntType addedPType = SpecPntType.SPntNone;
            PntOn2S apexPoint = new();

            switch (type)
            {
                case SurfaceKind.Sphere:
                case SurfaceKind.Cone:
                {
                    if (SpecialPoints.AddSingularPole((isReversed ? s2 : s1),
                                                               (isReversed ? s1 : s2), pIsoRef,
                                                               ref vertex, ref apexPoint, isReversed, true))
                    {
                        addedPType = SpecPntType.SPntPole;
                        break;
                    }
                }
                    goto case SurfaceKind.Torus; // [[fallthrough]]
                case SurfaceKind.Torus:
                    if (type == SurfaceKind.Torus)
                    {
                        if (SpecialPoints.AddCrossUVIsoPoint((isReversed ? s2 : s1),
                                                                      (isReversed ? s1 : s2), pIsoRef,
                                                                      tol3d, ref apexPoint, isReversed))
                        {
                            addedPType = SpecPntType.SPntSeamUV;
                            break;
                        }
                    }
                    goto case SurfaceKind.Cylinder; // [[fallthrough]]
                case SurfaceKind.Cylinder:
                    singularSurfaceID = i + 1;
                    AddVertexPoint(line, ref vertex, arrPeriods);
                    return SpecPntType.SPntSeamU;

                default:
                    break;
            }

            if (addedPType != SpecPntType.SPntNone)
            {
                singularSurfaceID = i + 1;
                AddPointIntoLine(line, arrPeriods, ref apexPoint, ref vertex);
                return addedPType;
            }
        }

        return SpecPntType.SPntNone;
    }

    //=================================================================================================

    public ALineToWLine(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, int nbPoints = 200)
    {
        _s1 = s1;
        _s2 = s2;
        _nbPointsInWline = nbPoints;
        _tolOpenDomain = 1.0e-9;
        _tolTransition = 1.0e-8;
        _tol3D = global::Kerf.Engine.Foundation.Precision.Confusion();

        SurfaceKind typs1 = s1.GetType();
        SurfaceKind typs2 = s2.GetType();

        switch (typs1)
        {
            case Plane:
                _quad1.SetValue(s1.Plane());
                break;

            case SurfaceKind.Cylinder:
                _quad1.SetValue(s1.Cylinder());
                break;

            case SurfaceKind.Sphere:
                _quad1.SetValue(s1.Sphere());
                break;

            case SurfaceKind.Cone:
                _quad1.SetValue(s1.Cone());
                break;

            case SurfaceKind.Torus:
                _quad1.SetValue(s1.Torus());
                break;

            default:
                break;
        }

        switch (typs2)
        {
            case Plane:
                _quad2.SetValue(s2.Plane());
                break;
            case SurfaceKind.Cylinder:
                _quad2.SetValue(s2.Cylinder());
                break;

            case SurfaceKind.Sphere:
                _quad2.SetValue(s2.Sphere());
                break;

            case SurfaceKind.Cone:
                _quad2.SetValue(s2.Cone());
                break;

            case SurfaceKind.Torus:
                _quad2.SetValue(s2.Torus());
                break;

            default:
                break;
        }
    }

    /// <summary>Sets the 3D tolerance (the deflection of the walking line from the analytic one).</summary>
    public void SetTol3D(double aT) => _tol3D = aT;

    /// <summary>The 3D tolerance.</summary>
    public double Tol3D() => _tol3D;

    /// <summary>Sets the tolerance of the transitions' test.</summary>
    public void SetTolTransition(double aT) => _tolTransition = aT;

    /// <summary>The tolerance of the transitions' test.</summary>
    public double TolTransition() => _tolTransition;

    /// <summary>Sets the shift of the ends of an open domain.</summary>
    public void SetTolOpenDomain(double aT) => _tolOpenDomain = aT;

    /// <summary>The shift of the ends of an open domain.</summary>
    public double TolOpenDomain() => _tolOpenDomain;

    //=================================================================================================

    /// <summary>Corrects the parameters of a point at a pole or an apex by linear extrapolation from the two
    /// previous points.</summary>
    protected internal void CorrectEndPoint(LineOn2S line, int index)
    {
        const double tol = 1.0e-5;
        const double sqTol = 1.0e-10;

        // Perform linear extrapolation from two previous points
        int indFirst, indSecond;
        if (index == 1)
        {
            indFirst = 3;
            indSecond = 2;
        }
        else
        {
            indFirst = index - 2;
            indSecond = index - 1;
        }
        PntOn2S pntOn2S = line.Value(index);

        for (int ii = 1; ii <= 2; ii++)
        {
            bool isOnFirst = (ii == 1);

            SurfaceQuadric quad = (ii == 1) ? _quad1 : _quad2;
            if (quad.TypeQuadric() == SurfaceKind.Cone)
            {
                Cone cone = quad.Cone();
                Point3D apex = cone.Apex;
                if (apex.SquareDistanceTo(pntOn2S.Value()) > sqTol)
                {
                    continue;
                }
            }
            else if (quad.TypeQuadric() == SurfaceKind.Sphere)
            {
                pntOn2S.ParametersOnSurface(isOnFirst, out double u, out double v);
                if (Math.Abs(v - M_PI / 2) > tol && Math.Abs(v + M_PI / 2) > tol)
                {
                    continue;
                }
            }
            else
            {
                continue;
            }

            Point2D prevPrevP2d = line.Value(indFirst).ValueOnSurface(isOnFirst);
            Point2D prevP2d = line.Value(indSecond).ValueOnSurface(isOnFirst);
            Direction2D dir = new(new Vector2D(prevPrevP2d, prevP2d));
            double x0 = prevPrevP2d.X, y0 = prevPrevP2d.Y;
            pntOn2S.ParametersOnSurface(isOnFirst, out double xend, out double yend);

            if (Math.Abs(dir.Y) < Constants.Resolution())
            {
                continue;
            }

            double newXend = dir.X / dir.Y * (yend - y0) + x0;

            line.SetUV(index, isOnFirst, newXend, yend);
        }
    }

    //=================================================================================================

    /// <summary>The radius of the smallest section of a cone or a sphere through the point (RealLast if neither
    /// surface is one).</summary>
    protected internal double GetSectionRadius(in Point3D pnt3d)
    {
        double retVal = RealLast();
        for (int i = 0; i < 2; i++)
        {
            SurfaceQuadric quad = (i != 0) ? _quad2 : _quad1;
            if (quad.TypeQuadric() == SurfaceKind.Cone)
            {
                Cone cone = quad.Cone();
                Xyz rVec = pnt3d.XYZ() - cone.Apex.XYZ();
                Xyz dir = cone.Axis.Direction.XYZ();

                retVal = Min(retVal, Math.Abs(rVec.Dot(dir) * Math.Tan(cone.SemiAngle)));
            }
            else if (quad.TypeQuadric() == SurfaceKind.Sphere)
            {
                Sphere sphere = quad.Sphere();
                Xyz rVec = pnt3d.XYZ() - sphere.Location.XYZ();
                Xyz dir = sphere.Position.Direction.XYZ();
                double r = sphere.Radius;
                double d = rVec.Dot(dir);
                double delta = r * r - d * d;
                if (delta <= 0.0)
                {
                    retVal = 0.0;
                    break;
                }
                else
                {
                    retVal = Min(retVal, Math.Sqrt(delta));
                }
            }
        }

        return retVal;
    }

    //=================================================================================================

    /// <summary>Converts the analytic line on its whole domain (an open end shifted by TolOpenDomain) into walking
    /// lines appended to theLines.</summary>
    public void MakeWLine(ALine aline, NCollection_Sequence<PatchLine> lines)
    {
        double f = aline.FirstParameter(out bool included);
        if (!included)
        {
            f += _tolOpenDomain;
        }
        double l = aline.LastParameter(out included);
        if (!included)
        {
            l -= _tolOpenDomain;
        }

        MakeWLine(aline, f, l, lines);
    }

    //=================================================================================================

    /// <summary>Converts the analytic line between theFPar and theLPar into walking lines appended to
    /// theLines. The line's vertices' tolerances are unified where they mark the same point.</summary>
    public void MakeWLine(ALine aLine, double fPar, double lPar,
                          NCollection_Sequence<PatchLine> lines)
    {
        int nbVert = aLine.VertexCount;
        if (nbVert == 0)
        {
            return;
        }

        // The same points can be marked by different vertices.
        // The code below unifies tolerances of all vertices
        // marking the same points.
        for (int i = 1; i < nbVert; i++)
        {
            ref PatchPoint curVert = ref aLine.ChangeVertex(i);
            PntOn2S currPt = curVert.PntOn2S();
            double curToler = curVert.Tolerance();
            for (int j = i + 1; j <= nbVert; j++)
            {
                ref PatchPoint vert = ref aLine.ChangeVertex(j);
                PntOn2S newPt = vert.PntOn2S();
                double toler = vert.Tolerance();

                double sumTol = curToler + toler;
                if (currPt.IsSame(newPt, sumTol))
                {
                    curVert.SetTolerance(sumTol);
                    vert.SetTolerance(sumTol);
                }
            }
        }

        double tol = 2.0 * _tol3D + global::Kerf.Engine.Foundation.Precision.Confusion();
        double prmTol = Max(1.0e-4 * (lPar - fPar), global::Kerf.Engine.Foundation.Precision.PConfusion());

        SpecPntType prePointExist = SpecPntType.SPntNone;

        NCollection_Array1<double> vertexParams = new(1, nbVert);
        NCollection_Array1<PatchPoint> seqVertex = new(1, nbVert);

        // It is possible to have several vertices with equal parameters.
        NCollection_Array1<bool> hasVertexBeenChecked = new(1, nbVert);

        LineOn2S linOn2S;
        double parameter = fPar;

        for (int i = vertexParams.Lower(); i <= vertexParams.Upper(); i++)
        {
            PatchPoint vert = aLine.Vertex(i);
            double par = vert.ParameterOnLine();
            vertexParams[i] = par;
            hasVertexBeenChecked[i] = false;
        }

        int singularSurfaceID = 0;
        Span<double> arrPeriods = [0.0,  // U1
                                     0.0,  // V1
                                     0.0,  // U2
                                     0.0]; // V2

        global::Kerf.Engine.Intersection.Walking.IntSurf.SetPeriod(_s1, _s2, arrPeriods);

        PntOn2S prevLPoint = new();

        while (parameter < lPar)
        {
            double step = (lPar - parameter) / (double)(_nbPointsInWline - 1);
            if (step < Epsilon(lPar))
            {
                break;
            }

            bool isStepReduced = false;
            double aLPar = lPar;

            for (int i = vertexParams.Lower(); i <= vertexParams.Upper(); i++)
            {
                if (hasVertexBeenChecked[i])
                {
                    continue;
                }

                aLPar = vertexParams[i];
                if (Math.Abs(aLPar - parameter) < prmTol)
                {
                    continue;
                }

                break;
            }

            if ((step - (aLPar - parameter) > prmTol) && (Math.Abs(aLPar - parameter) > prmTol))
            {
                step = Max((aLPar - parameter) / 5, 1.0e-5);
                isStepReduced = true;
            }

            int newVertID = 0;
            linOn2S = new LineOn2S();
            bool isFirstDegenerated = false, isLastDegenerated = false;

            double stepMin = 0.1 * step, stepMax = 10.0 * step;

            bool isLast = false;
            double prevParam = parameter;
            for (; !isLast; parameter += step)
            {
                PntOn2S pOn2S = new();

                if (lPar <= parameter)
                {
                    isLast = true;
                    if (prePointExist != SpecPntType.SPntNone)
                    {
                        break;
                    }
                    else
                    {
                        parameter = lPar;
                    }
                }

                bool isPointValid = false;
                double tgMagn = 0.0;
                {
                    aLine.D1(parameter, out Point3D pnt3d, out Vector3D tg);
                    if (GetSectionRadius(pnt3d) < 5.0e-6)
                    {
                        // We cannot compute 2D-parameters of
                        // aPOn2S correctly.

                        if (isLastDegenerated)
                        { // the current last point is wrong
                            linOn2S.RemovePoint(linOn2S.PointCount);
                        }

                        isPointValid = false;
                    }
                    else
                    {
                        isPointValid = true;
                    }

                    tgMagn = tg.Magnitude;
                    double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
                    _quad1.Parameters(pnt3d, ref u1, ref v1);
                    _quad2.Parameters(pnt3d, ref u2, ref v2);
                    pOn2S.SetValue(pnt3d, u1, v1, u2, v2);
                }

                if (prePointExist != SpecPntType.SPntNone)
                {
                    double uRes = Max(_s1.UResolution(_tol3D), _s2.UResolution(_tol3D)),
                           vRes = Max(_s1.VResolution(_tol3D), _s2.VResolution(_tol3D));

                    double tol2d = (prePointExist == SpecPntType.SPntPole) ? -1.0
                                    : (prePointExist == SpecPntType.SPntSeamV) ? vRes
                                    : (prePointExist == SpecPntType.SPntSeamUV) ? Max(uRes, vRes)
                                                                              : uRes;

                    PntOn2S rpt = pOn2S;

                    if (prePointExist == SpecPntType.SPntPole)
                    {
                        double prt = 0.5 * (prevParam + lPar);
                        for (int i = vertexParams.Lower(); i <= vertexParams.Upper(); i++)
                        {
                            double param = vertexParams[i];

                            if (param <= prevParam)
                            {
                                continue;
                            }

                            if ((param - prevParam) < prmTol)
                            {
                                Point3D pnt3dV = aLine.Value(param);
                                if (pOn2S.Value().SquareDistanceTo(pnt3dV) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                                {
                                    // i-th vertex is the same as a Pole/Apex.
                                    // So, it should be ignored.
                                    continue;
                                }
                            }

                            prt = 0.5 * (param + prevParam);
                            break;
                        }

                        Point3D pnt3d = aLine.Value(prt);
                        double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
                        _quad1.Parameters(pnt3d, ref u1, ref v1);
                        _quad2.Parameters(pnt3d, ref u2, ref v2);
                        rpt.SetValue(pnt3d, u1, v1, u2, v2);

                        if (pOn2S.IsSame(prevLPoint, Max(global::Kerf.Engine.Foundation.Precision.Approximation(), tol)))
                        {
                            // Set V-parameter as precise value found on the previous step.
                            if (singularSurfaceID == 1)
                            {
                                pOn2S.ParametersOnS1(out u2, out v2);
                                pOn2S.SetValue(true, u1, v2);
                            }
                            else // if (aSingularSurfaceID == 2)
                            {
                                pOn2S.ParametersOnS2(out u1, out v1);
                                pOn2S.SetValue(false, u2, v1);
                            }
                        }
                    }

                    if (SpecialPoints.ContinueAfterSpecialPoint(_s1, _s2, rpt, prePointExist, tol2d,
                                                                         ref prevLPoint, false))
                    {
                        AddPointIntoLine(linOn2S, arrPeriods, ref prevLPoint);
                    }
                    else if (parameter == lPar)
                    { // Strictly equal!!!
                        break;
                    }
                    else if (parameter + step < lPar)
                    {
                        // Prediction of the next point
                        aLine.D1(parameter + step, out Point3D pnt3dNext, out _);
                        double anU1 = 0.0, aV1 = 0.0, anU2 = 0.0, aV2 = 0.0;
                        _quad1.Parameters(pnt3dNext, ref anU1, ref aV1);
                        _quad2.Parameters(pnt3dNext, ref anU2, ref aV2);
                        PntOn2S pOn2SNext = new();
                        pOn2SNext.SetValue(pnt3dNext, anU1, aV1, anU2, aV2);

                        if (pOn2SNext.ValueOnSurface(false).SquareDistanceTo(rpt.ValueOnSurface(false)) > M_PI * M_PI
                            || pOn2SNext.ValueOnSurface(true).SquareDistanceTo(rpt.ValueOnSurface(true)) > M_PI * M_PI)
                        {
                            prevLPoint = rpt;
                            prevParam = parameter;
                            continue;
                        }
                    }
                }

                prePointExist = SpecPntType.SPntNone;

                int vertexNumber = -1;
                for (int i = vertexParams.Lower(); i <= vertexParams.Upper(); i++)
                {
                    if (hasVertexBeenChecked[i])
                    {
                        continue;
                    }

                    PatchPoint vp = aLine.Vertex(i);
                    double param = vertexParams[i];
                    if (((prevParam < param) && (param <= parameter))
                        || ((prevParam == parameter) && (param == parameter))
                        || (pOn2S.IsSame(vp.PntOn2S(), vp.Tolerance())
                            && (Math.Abs(vp.ParameterOnLine() - parameter) < prmTol)))
                    {
                        // We have either jumped over the vertex or "fell" on the vertex.
                        // However, ALine can be self-interfered. Therefore, we need to check
                        // vertex parameter and 3D-distance together.

                        vertexNumber = i;
                        break;
                    }
                }

                prevParam = parameter;

                if (vertexNumber < 0)
                {
                    if (isPointValid)
                    {
                        if (!isStepReduced)
                        {
                            StepComputing(aLine, pOn2S, lPar, parameter, tgMagn, stepMin, stepMax, _tol3D,
                                          ref step);
                        }

                        AddPointIntoLine(linOn2S, arrPeriods, ref pOn2S);
                        prevLPoint = pOn2S;
                    }
                    else
                    {
                        // add point, set corresponding status: to be corrected later
                        bool toAdd = false;
                        if (linOn2S.PointCount == 0)
                        {
                            isFirstDegenerated = true;
                            toAdd = true;
                        }
                        else if (linOn2S.PointCount > 1)
                        {
                            isLastDegenerated = true;
                            toAdd = true;
                        }

                        if (toAdd)
                        {
                            AddPointIntoLine(linOn2S, arrPeriods, ref pOn2S);
                            prevLPoint = pOn2S;
                        }
                    }

                    continue;
                }

                PatchPoint vtx = aLine.Vertex(vertexNumber);
                double newVertexParam = linOn2S.PointCount + 1;
                int nbPointsPrev = linOn2S.PointCount;

                // ATTENTION!!!
                //  IsPoleOrSeam inserts new point in aLinOn2S if aVtx respects
                // to some special point. Otherwise, aLinOn2S is not changed.

                // Find a point for reference parameter. It will be used
                // if real parameter value cannot be precise (see comment to
                // IsPoleOrSeam(...) function).
                PntOn2S prefIso = vtx.PntOn2S();
                if (linOn2S.PointCount < 1)
                {
                    for (int i = vertexNumber + 1; i <= vertexParams.Upper(); i++)
                    {
                        double param = vertexParams[i];
                        if ((param - vertexParams[vertexNumber]) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            double prm = 0.5 * (param + vertexParams[vertexNumber]);
                            Point3D pnt3d = aLine.Value(prm);
                            double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
                            _quad1.Parameters(pnt3d, ref u1, ref v1);
                            _quad2.Parameters(pnt3d, ref u2, ref v2);
                            prefIso.SetValue(pnt3d, u1, v1, u2, v2);
                            break;
                        }
                    }
                }
                else
                {
                    prefIso = linOn2S.Value(linOn2S.PointCount);
                }

                prePointExist = IsPoleOrSeam(_s1, _s2, prefIso, linOn2S, ref vtx, arrPeriods, tol,
                                              out singularSurfaceID);

                if (prePointExist == SpecPntType.SPntPole || prePointExist == SpecPntType.SPntPoleSeamU)
                {
                    // set corresponding status: to be corrected later
                    if (linOn2S.PointCount == 1)
                    {
                        isFirstDegenerated = true;
                    }
                    else
                    {
                        isLastDegenerated = true;
                    }
                }

                double curVertParam = vtx.ParameterOnLine();
                if (prePointExist != SpecPntType.SPntNone)
                {
                    if (nbPointsPrev == linOn2S.PointCount)
                    {
                        // Vertex coincides any point of line and was not added into line
                        newVertexParam = nbPointsPrev;
                    }
                    prevParam = parameter = curVertParam;
                }
                else
                {
                    if (!isPointValid)
                    {
                        // Take a farther point of ALine (with greater parameter)
                        continue;
                    }

                    if (vtx.Tolerance() > tol)
                    {
                        vtx.SetValue(pOn2S);
                        AddPointIntoLine(linOn2S, arrPeriods, ref pOn2S);
                    }
                    else
                    {
                        AddVertexPoint(linOn2S, ref vtx, arrPeriods);
                    }
                }

                prevLPoint = pOn2S = linOn2S.Value(linOn2S.PointCount);

                {
                    bool isFound = false;
                    double sqTol = tol * tol;
                    Point3D p1 = aLine.Value(curVertParam);
                    PntOn2S vertP2S = vtx.PntOn2S();
                    double vertToler = vtx.Tolerance();

                    for (int i = vertexParams.Lower(); i <= vertexParams.Upper(); i++)
                    {
                        if (hasVertexBeenChecked[i])
                        {
                            continue;
                        }

                        Point3D p2 = aLine.Value(vertexParams[i]);

                        if (p1.SquareDistanceTo(p2) < sqTol)
                        {
                            PatchPoint lVtx = aLine.Vertex(i);
                            lVtx.SetValue(vertP2S);
                            lVtx.SetTolerance(vertToler);
                            double param = lVtx.ParameterOnLine();
                            if (Math.Abs(param - lPar) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {                           // in the case of closed curve,
                                lVtx.SetParameter(-1); // we don't know yet the number of points in the curve
                            }
                            else
                            {
                                lVtx.SetParameter(newVertexParam);
                            }
                            seqVertex[++newVertID] = lVtx;
                            hasVertexBeenChecked[i] = true;
                            isFound = true;
                        }
                        else if (isFound)
                        {
                            break;
                        }
                    }
                }

                if ((prePointExist != SpecPntType.SPntNone) && (linOn2S.PointCount > 1))
                {
                    break;
                }

                if (isStepReduced)
                {
                    isStepReduced = false;

                    step = (lPar - parameter) / (double)(_nbPointsInWline - 1);
                    if (step < Epsilon(lPar))
                    {
                        break;
                    }

                    aLPar = vertexParams[nbVert];
                    for (int i = vertexParams.Lower(); i <= vertexParams.Upper(); i++)
                    {
                        if (hasVertexBeenChecked[i])
                        {
                            continue;
                        }

                        aLPar = vertexParams[i];
                        if (Math.Abs(aLPar - parameter) < prmTol)
                        {
                            continue;
                        }

                        break;
                    }

                    if ((step - (aLPar - parameter) > prmTol) && (Math.Abs(aLPar - parameter) > prmTol))
                    {
                        step = Max((aLPar - parameter) / 5, 1.0e-5);
                        isStepReduced = true;
                    }

                    stepMin = 0.1 * step;
                    stepMax = 10.0 * step;
                }
            } // for(; !isLast; aParameter += aStep)

            if (linOn2S.PointCount < 2)
            {
                parameter += step;
                continue;
            }

            // Correct first and last points if needed
            if (linOn2S.PointCount >= 3)
            {
                if (isFirstDegenerated)
                {
                    CorrectEndPoint(linOn2S, 1);
                }
                if (isLastDegenerated)
                {
                    CorrectEndPoint(linOn2S, linOn2S.PointCount);
                }
            }

            //-----------------------------------------------------------------
            //--              W  L  i  n  e     c  r  e  a  t  i  o  n      ---
            //-----------------------------------------------------------------
            WLine wLine;
            //
            if (aLine.TransitionOnS1() == WalkingTypeTrans.Touch)
            {
                wLine = new WLine(linOn2S, aLine.IsTangent, aLine.SituationS1(),
                                            aLine.SituationS2());
                wLine.SetCreatingWayInfo(WLine.IntPatch_WLType.WLImpImp);
            }
            else if (aLine.TransitionOnS1() == WalkingTypeTrans.Undecided)
            {
                wLine = new WLine(linOn2S, aLine.IsTangent);
                wLine.SetCreatingWayInfo(WLine.IntPatch_WLType.WLImpImp);
            }
            else
            {
                // Computation of transitions of the line on two surfaces    ---
                int indice1 = Max(linOn2S.PointCount / 3, 2);
                Point3D pp0 = linOn2S.Value(indice1 - 1).Value(),
                       pp1 = linOn2S.Value(indice1).Value();
                Vector3D tgvalid = new(pp0, pp1);
                Vector3D nq1 = _quad1.Normale(pp0), nq2 = _quad2.Normale(pp0);

                double dotcross = tgvalid.DotCross(nq2, nq1);

                WalkingTypeTrans trans1 = WalkingTypeTrans.Undecided, trans2 = WalkingTypeTrans.Undecided;

                if (dotcross > _tolTransition)
                {
                    trans1 = WalkingTypeTrans.Out;
                    trans2 = WalkingTypeTrans.In;
                }
                else if (dotcross < -_tolTransition)
                {
                    trans1 = WalkingTypeTrans.In;
                    trans2 = WalkingTypeTrans.Out;
                }

                wLine = new WLine(linOn2S, aLine.IsTangent, trans1, trans2);
                wLine.SetCreatingWayInfo(WLine.IntPatch_WLType.WLImpImp);
            }

            for (int i = seqVertex.Lower(); i <= newVertID; i++)
            {
                PatchPoint vtx = seqVertex[i];
                if (vtx.ParameterOnLine() == -1)
                {                                       // in the case of closed curve,
                    vtx.SetParameter(wLine.NbPnts()); // we set the last parameter
                }
                wLine.AddVertex(vtx);
            }

            wLine.SetPeriod(arrPeriods[0], arrPeriods[1], arrPeriods[2], arrPeriods[3]);

            // the method ComputeVertexParameters can reduce the number of points in <aWLine>
            wLine.ComputeVertexParameters(_tol3D);

            if (wLine.NbPnts() > 1)
            {
                wLine.EnablePurging(false);
                lines.Append(wLine);
            }
        } // while(aParameter < theLPar)
    }

    //=======================================================================
    // function : CheckDeflection
    // purpose  : Returns:
    //            -1 - step is too small
    //            0  - step is normal
    //            +1 - step is too big
    //=======================================================================
    protected internal int CheckDeflection(in Xyz midPt, double maxDeflection)
    {
        double dist = Math.Abs(_quad1.Distance(new Point3D(midPt)));
        if (dist > maxDeflection)
        {
            return 1;
        }

        dist = Max(Math.Abs(_quad2.Distance(new Point3D(midPt))), dist);

        if (dist > maxDeflection)
        {
            return 1;
        }

        if ((dist + dist) < maxDeflection)
        {
            return -1;
        }

        return 0;
    }

    //=================================================================================================

    /// <summary>Computes the step along the analytic line from the osculating circle at the point, refined by
    /// bisection against the deflection; false if the step couldn't be computed (theStep unchanged then).</summary>
    protected internal bool StepComputing(ALine aLine, in PntOn2S pOn2S, double lastParOfAline,
                                          double curParam, double tgMagnitude, double stepMin,
                                          double stepMax, double maxDeflection, ref double step)
    {
        if (tgMagnitude < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false;
        }

        double eps = _tol3D;

        // Indeed, 1.0e+15 < 2^50 < 1.0e+16. Therefore,
        // if we apply bisection method to the range with length
        // 1.0e+6 then we will be able to find solution with max error ~1.0e-9.
        const int nbIterMax = 50;

        double notFilledRange = lastParOfAline - curParam;
        double minStep = stepMin, maxStep = Min(stepMax, notFilledRange);

        if (minStep > maxStep)
        {
            step = maxStep;
            return true;
        }

        double r = PointLine.CurvatureRadiusOfIntersLine(_s1, _s2, pOn2S);

        if (r < 0.0)
        {
            return false;
        }
        else
        {
            // The 3D-step is defined as length of the tangent to the osculating circle
            // by the condition that the distance from end point of the tangent to the
            // circle is no greater than anEps. theStep is the step in
            // parameter space of intersection curve (must be converted from 3D-step).

            step = Min(Math.Sqrt(eps * (2.0 * r + eps)) / tgMagnitude, maxStep);
            step = Max(step, minStep);
        }

        // The step value has been computed for osculating circle.
        // Now it should be checked for real intersection curve
        // and is made more precise in case of necessity.

        int nbIter = 0;
        do
        {
            nbIter++;

            Xyz p1 = pOn2S.Value().XYZ();
            Xyz p2 = aLine.Value(curParam + step).XYZ();
            int status = CheckDeflection(0.5 * (p1 + p2), maxDeflection);

            if (status == 0)
            {
                break;
            }

            if (status < 0)
            {
                minStep = step;
            }
            else // if(aStatus > 0)
            {
                maxStep = step;
            }

            step = 0.5 * (minStep + maxStep);
        } while (((maxStep - minStep) > global::Kerf.Engine.Foundation.Precision.PConfusion()) && (nbIter <= nbIterMax));

        return nbIter <= nbIterMax;
    }
}
