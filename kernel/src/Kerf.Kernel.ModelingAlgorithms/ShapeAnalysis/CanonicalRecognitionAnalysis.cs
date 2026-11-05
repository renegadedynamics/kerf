// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_CanonicalRecognition.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_CanonicalRecognition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Measurement;
using Kerf.Engine.Numerics;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Geometry.Conversion.ConvType;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// Finds whether the geometry of a shape (a face, a shell, an edge or a wire) can be represented within a tolerance
/// by a simple analytical entity (plane, cylinder, cone, sphere, line, circle, ellipse), which could replace complex
/// (for example B-spline) geometry. Only reads the shape.
/// </summary>
/// <remarks>
/// A working object (decision 21). The <c>Is*</c> methods take the sample entity by <c>ref</c>: they read its
/// position and parameters as a first guess (for edges and wires) and write the result only on success.
/// </remarks>
internal sealed class CanonicalRecognitionAnalysis
{
    private Shape _shape;
    private ShapeType _sType;
    private double _gap;
    private int _status;

    /// <summary>Empty constructor (status -1: not initialised).</summary>
    public CanonicalRecognitionAnalysis()
    {
        _shape = new Shape();
        _sType = ShapeType.Shape;
        _gap = -1.0;
        _status = -1;
    }

    /// <summary>Constructor with the shape.</summary>
    public CanonicalRecognitionAnalysis(in Shape shape)
    {
        _shape = new Shape();
        _sType = ShapeType.Shape;
        _gap = -1.0;
        _status = -1;
        Init(shape);
    }

    /// <summary>Sets the shape: a shell, a face, a wire or an edge (otherwise the status is 1).</summary>
    public void SetShape(in Shape shape)
    {
        Init(shape);
    }

    /// <summary>The input shape.</summary>
    public Shape GetShape() => _shape;

    /// <summary>The deviation between the input geometry and the analytical entity.</summary>
    public double GetGap() => _gap;

    /// <summary>
    /// The status: -1 not initialised by a shape, 0 no errors, 1 an error during an operation (usually wrong input).
    /// Operations work when it is 0; after one fails, <see cref="ClearStatus"/> resets it.
    /// </summary>
    public int GetStatus() => _status;

    /// <summary>Resets the status to 0.</summary>
    public void ClearStatus()
    {
        _status = 0;
    }

    private void Init(in Shape shape)
    {
        ShapeType t = shape.Type;
        switch (t)
        {
            case ShapeType.Shell:
            case ShapeType.Face:
            case ShapeType.Wire:
            case ShapeType.Edge:
                _shape = shape;
                _sType = t;
                _status = 0;
                break;
            default:
                _status = 1;
                break;
        }
    }

    private bool IsElementarySurf(SurfaceKind target, double tol, ref CoordinateSystem pos,
                                  NCollection_Array1<double> theParams)
    {
        if (_status != 0)
        {
            return false;
        }
        //
        if (_sType == ShapeType.Face)
        {
            Surface? elemSurf = GetSurface(ShapeCasts.Face(_shape), tol, Target, target, ref _gap, ref _status);
            if (elemSurf is null)
            {
                return false;
            }
            //
            bool isOK = SetSurfParams(target, elemSurf, ref pos, theParams);
            if (!isOK)
            {
                _status = 1;
                return false;
            }
            return true;
        }
        else if (_sType == ShapeType.Shell)
        {
            Surface? elemSurf = GetSurface(ShapeCasts.Shell(_shape), tol, Target, target, ref _gap, ref _status);
            if (elemSurf is null)
            {
                return false;
            }
            bool isOK = SetSurfParams(target, elemSurf, ref pos, theParams);
            if (!isOK)
            {
                _status = 1;
                return false;
            }
            return true;
        }
        else if (_sType == ShapeType.Edge)
        {
            Surface? elemSurf = GetSurface(ShapeCasts.Edge(_shape), tol, Target, target, ref pos,
                                                  theParams, ref _gap, ref _status);

            bool isOK = SetSurfParams(target, elemSurf, ref pos, theParams);
            if (!isOK)
            {
                _status = 1;
                return false;
            }
            return true;
        }
        else if (_sType == ShapeType.Wire)
        {
            Surface? elemSurf = GetSurface(ShapeCasts.Wire(_shape), tol, Target, target, ref pos,
                                                  theParams, ref _gap, ref _status);

            bool isOK = SetSurfParams(target, elemSurf, ref pos, theParams);
            if (!isOK)
            {
                _status = 1;
                return false;
            }
            return true;
        }
        return false;
    }

    /// <summary>
    /// True if the underlying surface can be represented by a plane within <paramref name="tol"/>; the plane is
    /// then set in <paramref name="pln"/>. An edge or a wire is also tried with BRepLib_FindSurface.
    /// </summary>
    public bool IsPlane(double tol, ref Plane3D pln)
    {
        CoordinateSystem pos = pln.Position;
        NCollection_Array1<double> aParams = new(1, 1);
        //
        SurfaceKind target = Plane;
        if (IsElementarySurf(target, tol, ref pos, aParams))
        {
            pln = pln.WithPosition(pos);
            return true;
        }
        // Try to build plane for wire or edge
        if (_sType == ShapeType.Edge || _sType == ShapeType.Wire)
        {
            FindSurface fndSurf = new(_shape, tol, true, false);
            if (fndSurf.Found())
            {
                var plane = fndSurf.Surface() as PlaneSurface;
                pln = plane!.Plane;
                _gap = fndSurf.ToleranceReached();
                _status = 0;
                return true;
            }
            else
            {
                _status = 1;
            }
        }
        return false;
    }

    /// <summary>
    /// True if the underlying surface can be represented by a cylinder within <paramref name="tol"/>; then set in
    /// <paramref name="cyl"/>. For an edge or a wire, the sample cylinder (if its radius is set) starts a
    /// least-squares fit.
    /// </summary>
    public bool IsCylinder(double tol, ref Cylinder3D cyl)
    {
        CoordinateSystem pos = cyl.Position;
        NCollection_Array1<double> aParams = new(1, 1);
        aParams[1] = cyl.Radius;
        //
        SurfaceKind target = SurfaceKind.Cylinder;
        if (IsElementarySurf(target, tol, ref pos, aParams))
        {
            cyl = cyl.WithPosition(pos);
            cyl = cyl.WithRadius(aParams[1]);
            return true;
        }

        if (aParams[1] > Kerf.Engine.Foundation.Precision.Infinite())
        {
            // Sample cylinder does not seem to be set, least square method is not applicable.
            return false;
        }
        if (_shape.Type == ShapeType.Edge || _shape.Type == ShapeType.Wire)
        {
            // Try to build surface by least square method;
            Wire wire = MakeWireOf(_shape);
            bool isDone = GetSurfaceByLS(wire, tol, target, ref pos, aParams, ref _gap, ref _status);
            if (isDone)
            {
                cyl = cyl.WithPosition(pos);
                cyl = cyl.WithRadius(aParams[1]);
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// True if the underlying surface can be represented by a cone within <paramref name="tol"/>; then set in
    /// <paramref name="cone"/>. For an edge or a wire, the sample cone (if its radius is set) starts a least-squares
    /// fit.
    /// </summary>
    public bool IsCone(double tol, ref Cone3D cone)
    {
        CoordinateSystem pos = cone.Position;
        NCollection_Array1<double> aParams = new(1, 2);
        aParams[1] = cone.SemiAngle;
        aParams[2] = cone.RefRadius;
        //
        SurfaceKind target = SurfaceKind.Cone;
        if (IsElementarySurf(target, tol, ref pos, aParams))
        {
            cone = cone.WithPosition(pos);
            cone = cone.WithSemiAngle(aParams[1]);
            cone = cone.WithRadius(aParams[2]);
            return true;
        }

        if (aParams[2] > Kerf.Engine.Foundation.Precision.Infinite())
        {
            // Sample cone does not seem to be set, least square method is not applicable.
            return false;
        }
        if (_shape.Type == ShapeType.Edge || _shape.Type == ShapeType.Wire)
        {
            // Try to build surface by least square method;
            Wire wire = MakeWireOf(_shape);
            bool isDone = GetSurfaceByLS(wire, tol, target, ref pos, aParams, ref _gap, ref _status);
            if (isDone)
            {
                cone = cone.WithPosition(pos);
                cone = cone.WithSemiAngle(aParams[1]);
                cone = cone.WithRadius(aParams[2]);
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// True if the underlying surface can be represented by a sphere within <paramref name="tol"/>; then set in
    /// <paramref name="sphere"/>. For an edge or a wire, the sample sphere (if its radius is set) starts a
    /// least-squares fit.
    /// </summary>
    public bool IsSphere(double tol, ref Sphere3D sphere)
    {
        CoordinateSystem pos = sphere.Position;
        NCollection_Array1<double> aParams = new(1, 1);
        aParams[1] = sphere.Radius;
        //
        SurfaceKind target = SurfaceKind.Sphere;
        if (IsElementarySurf(target, tol, ref pos, aParams))
        {
            sphere = sphere.WithPosition(pos);
            sphere = sphere.WithRadius(aParams[1]);
            return true;
        }
        //
        if (aParams[1] > Kerf.Engine.Foundation.Precision.Infinite())
        {
            // Sample sphere does not seem to be set, least square method is not applicable.
            return false;
        }
        if (_shape.Type == ShapeType.Edge || _shape.Type == ShapeType.Wire)
        {
            // Try to build surface by least square method;
            Wire wire = MakeWireOf(_shape);
            bool isDone = GetSurfaceByLS(wire, tol, target, ref pos, aParams, ref _gap, ref _status);
            if (isDone)
            {
                sphere = sphere.WithPosition(pos);
                sphere = sphere.WithRadius(aParams[1]);
                return true;
            }
        }
        return false;
    }

    // Kerf: the wire OCCT builds inline in IsCylinder, IsCone and IsSphere (a new wire holding the edge, or the wire
    // itself). Only that new wire is written.
    private static Wire MakeWireOf(in Shape shape)
    {
        Wire wire = new();
        if (shape.Type == ShapeType.Edge)
        {
            BrepBuilder bb = new();
            bb.MakeWire(ref wire);
            bb.Add(wire, shape);
        }
        else
        {
            wire = ShapeCasts.Wire(shape);
        }
        return wire;
    }

    private bool IsConic(CurveKind target, double tol, ref Frame3D pos, NCollection_Array1<double> theParams)
    {
        if (_status != 0)
        {
            return false;
        }

        if (_sType == ShapeType.Edge)
        {
            Curve3D? conic = GetCurve(ShapeCasts.Edge(_shape), tol, Target, target, ref _gap, ref _status);

            if (conic is null)
            {
                return false;
            }

            bool isOK = SetConicParameters(target, conic, ref pos, theParams);

            if (!isOK)
            {
                _status = 1;
                return false;
            }
            return true;
        }
        else if (_sType == ShapeType.Wire)
        {
            Iterator iter = new(_shape);
            if (!iter.More())
            {
                _status = 1;
                return false;
            }
            Frame3D aPos = new();
            NCollection_Array1<double> aParams = new(1, theParams.Length());
            Shape edge = iter.Value();

            Curve3D? conic = GetCurve(ShapeCasts.Edge(edge), tol, Target, target, ref _gap, ref _status);
            if (conic is null)
            {
                return false;
            }
            bool isOK = SetConicParameters(target, conic, ref pos, theParams);
            if (!isOK)
            {
                _status = 1;
                return false;
            }
            if (!iter.More())
            {
                return true;
            }
            else
            {
                iter.Next();
            }
            for (; iter.More(); iter.Next())
            {
                conic = GetCurve(ShapeCasts.Edge(iter.Value()), tol, Target, target, ref _gap, ref _status);
                if (conic is null)
                {
                    return false;
                }
                isOK = SetConicParameters(target, conic, ref aPos, aParams);
                isOK = CompareConicParams(target, tol, pos, theParams, aPos, aParams);

                if (!isOK)
                {
                    return false;
                }
            }
            return true;
        }
        _status = 1;
        return false;
    }

    /// <summary>
    /// True if the underlying curve (of an edge, or of every edge of a wire) can be represented by a line within
    /// <paramref name="tol"/>; then set in <paramref name="lin"/>.
    /// </summary>
    public bool IsLine(double tol, ref Line3D lin)
    {
        Frame3D pos = new();
        NCollection_Array1<double> aParams = new(1, 1);

        CurveKind target = Line;
        bool isOK = IsConic(target, tol, ref pos, aParams);
        if (isOK)
        {
            lin = lin.WithPosition(pos.Axis);
        }
        return isOK;
    }

    /// <summary>
    /// True if the underlying curve can be represented by a circle within <paramref name="tol"/>; then set in
    /// <paramref name="circ"/>.
    /// </summary>
    public bool IsCircle(double tol, ref Circle3D circ)
    {
        Frame3D pos = new();
        NCollection_Array1<double> aParams = new(1, 1);

        CurveKind target = Circle;
        bool isOK = IsConic(target, tol, ref pos, aParams);
        if (isOK)
        {
            circ = circ.WithPosition(pos);
            circ = circ.WithRadius(aParams[1]);
        }
        return isOK;
    }

    /// <summary>
    /// True if the underlying curve can be represented by an ellipse within <paramref name="tol"/>; then set in
    /// <paramref name="elips"/>.
    /// </summary>
    public bool IsEllipse(double tol, ref Ellipse3D elips)
    {
        Frame3D pos = new();
        NCollection_Array1<double> aParams = new(1, 2);

        CurveKind target = Ellipse;
        bool isOK = IsConic(target, tol, ref pos, aParams);
        if (isOK)
        {
            elips = elips.WithPosition(pos);
            elips = elips.WithMajorRadius(aParams[1]);
            elips = elips.WithMinorRadius(aParams[2]);
        }
        return isOK;
    }

    private static Surface? GetSurface(in Face face, double tol, ConvType type,
                                            SurfaceKind target, ref double gap, ref int status)
    {
        status = 0;
        Surface? surf = Tool.Surface(face, out Placement loc);
        if (surf is null)
        {
            status = 1;
            return surf;
        }
        SurfToAnaSurf conv = new(surf);
        conv.SetConvType(type);
        conv.SetTarget(target);
        Surface? anaSurf = conv.ConvertToAnalytical(tol);
        if (anaSurf is null)
        {
            return anaSurf;
        }
        //
        if (!loc.IsIdentity)
        {
            anaSurf.Transform(loc.Transformation); // a new surface of the converter
        }
        //
        gap = conv.Gap();
        return anaSurf;
    }

    private static Surface? GetSurface(in Shell shell, double tol, ConvType type,
                                            SurfaceKind target, ref double gap, ref int status)
    {
        Surface? elemSurf = null;
        Iterator iter = new(shell);
        if (!iter.More())
        {
            status = 1;
            return elemSurf;
        }
        CoordinateSystem pos1 = new();
        NCollection_Array1<double> params1 = new(1, Max(1, GetNbPars(target)));
        Shape face = iter.Value();

        elemSurf = GetSurface(ShapeCasts.Face(face), tol, type, target, ref gap, ref status);
        if (elemSurf is null)
        {
            return elemSurf;
        }
        SetSurfParams(target, elemSurf, ref pos1, params1);
        if (!iter.More())
        {
            return elemSurf;
        }
        else
        {
            iter.Next();
        }
        CoordinateSystem pos = new();
        NCollection_Array1<double> aParams = new(1, Max(1, GetNbPars(target)));
        bool isOK;
        for (; iter.More(); iter.Next())
        {
            Surface? elemSurf1 = GetSurface(ShapeCasts.Face(iter.Value()), tol, type, target, ref gap, ref status);
            if (elemSurf1 is null)
            {
                return elemSurf1;
            }
            // OCCT: the parameters are taken from the first face's surface again, not from anElemSurf1, so the faces
            // after the first always compare equal (occt-issues.md)
            SetSurfParams(target, elemSurf, ref pos, aParams);
            isOK = CompareSurfParams(target, tol, pos1, params1, pos, aParams);

            if (!isOK)
            {
                elemSurf = null;
                return elemSurf;
            }
        }
        return elemSurf;
    }

    private static Surface? GetSurface(in Edge edge, double tol, ConvType type,
                                            SurfaceKind target, ref CoordinateSystem pos,
                                            NCollection_Array1<double> theParams, ref double gap, ref int status)
    {
        // Get surface list
        List<Surface> surfs = new();
        List<double> gaps = new();
        int j = 0;
        for (;;)
        {
            j++;
            Surface? surf = null;
            Placement loc = new();
            Curve2D? pCurve = null;
            double ff = 0.0, ll = 0.0;
            Tool.CurveOnSurface(edge, ref pCurve, ref surf, ref loc, ref ff, ref ll, j);
            if (surf is null)
            {
                break;
            }
            SurfToAnaSurf conv = new(surf);
            conv.SetConvType(type);
            conv.SetTarget(target);
            Surface? anaSurf = conv.ConvertToAnalytical(tol);
            if (anaSurf is null)
            {
                continue;
            }
            //
            if (!loc.IsIdentity)
            {
                anaSurf.Transform(loc.Transformation); // a new surface of the converter
            }
            //
            gaps.Add(conv.Gap());
            surfs.Add(anaSurf);
        }

        if (surfs.Count == 0)
        {
            status = 1;
            return null;
        }

        CoordinateSystem aPos = new();
        int nbPars = Max(1, GetNbPars(target));
        NCollection_Array1<double> aParams = new(1, nbPars);

        int ifit = -1, i;
        double minDev = RealLast();
        if (surfs.Count == 1)
        {
            ifit = 0;
        }
        else
        {
            for (i = 0; i < surfs.Count; ++i)
            {
                SetSurfParams(target, surfs[i], ref aPos, aParams);
                double dev = DeviationSurfParams(target, pos, theParams, aPos, aParams);
                if (dev < minDev)
                {
                    minDev = dev;
                    ifit = i;
                }
            }
        }
        if (ifit >= 0)
        {
            SetSurfParams(target, surfs[ifit], ref pos, theParams);
            gap = gaps[ifit];
            return surfs[ifit];
        }
        else
        {
            status = 1;
            return null;
        }
    }

    private static Surface? GetSurface(in Wire wire, double tol, ConvType type,
                                            SurfaceKind target, ref CoordinateSystem pos,
                                            NCollection_Array1<double> theParams, ref double gap, ref int status)
    {
        Iterator iter = new(wire);
        if (!iter.More())
        {
            // Empty wire
            status = 1;
            return null;
        }
        // First edge
        Edge edge1 = ShapeCasts.Edge(iter.Value());
        CoordinateSystem pos1 = pos;
        int nbPars = GetNbPars(target);
        NCollection_Array1<double> params1 = new(1, Max(1, nbPars));
        double gap1 = 0.0; // Kerf: uninitialised in OCCT; written whenever the result is used
        int i;
        for (i = 1; i <= nbPars; ++i)
        {
            params1[i] = theParams[i];
        }
        Surface? elemSurf1 = GetSurface(edge1, tol, type, target, ref pos1, params1, ref gap1, ref status);
        if (status != 0 || elemSurf1 is null)
        {
            return null;
        }
        iter.Next();
        for (; iter.More(); iter.Next())
        {
            CoordinateSystem aPos = pos1;
            nbPars = GetNbPars(target);
            NCollection_Array1<double> aParams = new(1, Max(1, nbPars));
            for (i = 1; i <= nbPars; ++i)
            {
                aParams[i] = params1[i];
            }
            double aGap = 0.0;
            Edge edge = ShapeCasts.Edge(iter.Value());
            Surface? elemSurf = GetSurface(edge, tol, type, target, ref aPos, aParams, ref aGap, ref status);
            if (status != 0 || elemSurf is null)
            {
                return null;
            }
            bool isOK = CompareSurfParams(target, tol, pos1, params1, aPos, aParams);
            if (!isOK)
            {
                return null;
            }
        }

        SetSurfParams(target, elemSurf1, ref pos, theParams);
        gap = gap1;
        return elemSurf1;
    }

    private static bool GetSurfaceByLS(in Wire wire, double tol, SurfaceKind target, ref CoordinateSystem pos,
                                       NCollection_Array1<double> theParams, ref double gap, ref int status)
    {
        int nbMaxInt = 100;
        if (!GetSamplePoints(wire, tol, nbMaxInt, out NCollection_HArray1<Xyz>? points))
        {
            return false;
        }

        gap = GetLSGap(points!, target, pos, theParams);
        if (gap <= tol)
        {
            status = 0;
            return true;
        }

        int nbVar = 0;
        if (target == SurfaceKind.Sphere)
        {
            nbVar = 4;
        }
        else if (target == SurfaceKind.Cylinder)
        {
            nbVar = 4;
        }
        else if (target == SurfaceKind.Cone)
        {
            nbVar = 5;
        }
        else
        {
            return false;
        }

        math_Vector fBnd = new(1, nbVar), lBnd = new(1, nbVar), startPoint = new(1, nbVar);

        double relDev = 0.2; // Customer can set parameters of sample surface
                              //  with relative precision about aRelDev.
                              //  For example, if radius of sample surface is R,
                              //  it means, that "exact" value is in interav
                              //[R - aRelDev*R, R + aRelDev*R]. This interval is set
                              //  for R as boundary values for optimization algo.
        FillSolverData(target, pos, theParams, startPoint, fBnd, lBnd, relDev);

        //
        double aTol = Kerf.Engine.Foundation.Precision.Confusion();
        MultipleVarFunction pFunc;
        FuncSphereLSDist funcSph = new(points!);
        FuncCylinderLSDist funcCyl = new(points!, pos.Direction);
        FuncConeLSDist funcCon = new(points!, pos.Direction);
        if (target == SurfaceKind.Sphere)
        {
            pFunc = funcSph;
        }
        else if (target == SurfaceKind.Cylinder)
        {
            pFunc = funcCyl;
        }
        else
        {
            pFunc = funcCon;
        }
        //
        math_Vector steps = new(1, nbVar);
        int nbInt = 10;
        int i;
        for (i = 1; i <= nbVar; ++i)
        {
            steps[i] = (lBnd[i] - fBnd[i]) / nbInt;
        }
        Pso globSolver = new(pFunc, fBnd, lBnd, steps);
        globSolver.Perform(steps, out double _, startPoint);
        SetCanonicParameters(target, startPoint, ref pos, theParams);

        gap = GetLSGap(points!, target, pos, theParams);
        if (gap <= tol)
        {
            status = 0;
            return true;
        }
        //
        Matrix dirMatrix = new(1, nbVar, 1, nbVar, 0.0);
        for (i = 1; i <= nbVar; i++)
        {
            dirMatrix[i, i] = 1.0;
        }

        if (target == SurfaceKind.Cylinder || target == SurfaceKind.Cone)
        {
            // Set search direction for location to be perpendicular to axis to avoid
            // searching along axis
            Direction3D dir = pos.Direction;
            Plane3D pln = new(pos.Location, dir);
            Direction3D uDir = pln.Position.XDirection;
            Direction3D vDir = pln.Position.YDirection;
            for (i = 1; i <= 3; ++i)
            {
                dirMatrix[i, 1] = uDir.Coord(i);
                dirMatrix[i, 2] = vDir.Coord(i);
                Direction3D uvDir = new(uDir.XYZ() + vDir.XYZ());
                dirMatrix[i, 3] = uvDir.Coord(i);
            }
        }

        Powell solver = new(pFunc, aTol);
        solver.Perform(pFunc, startPoint, dirMatrix);

        if (solver.IsDone)
        {
            solver.Location(startPoint);
            status = 0;
            SetCanonicParameters(target, startPoint, ref pos, theParams);
            gap = GetLSGap(points!, target, pos, theParams);
            status = 0;
            if (gap <= tol)
            {
                return true;
            }
        }
        else
        {
            status = 1;
        }

        return false;
    }

    private static Curve3D? GetCurve(in Edge edge, double tol, ConvType type,
                                        CurveKind target, ref double gap, ref int status)
    {
        status = 0;
        Curve3D? curv = Tool.Curve(edge, out Placement loc, out double f, out double l);
        if (curv is null)
        {
            status = 1;
            return curv;
        }
        CurveToAnaCurve conv = new(curv);
        conv.SetConvType(type);
        conv.SetTarget(target);
        Curve3D? anaCurv = null;
        conv.ConvertToAnalytical(tol, ref anaCurv, f, l, out double _, out double _);
        if (anaCurv is null)
        {
            return anaCurv;
        }
        //
        if (!loc.IsIdentity)
        {
            // OCCT: when the edge's curve is already the target conic, the converter returns the edge's own curve,
            // and OCCT transforms that shared curve in place (occt-issues.md). Kerf: frozen: it is frozen here, so
            // the port transforms a copy; the result is the same, and the edge is unchanged.
            if (anaCurv.IsFrozen)
            {
                anaCurv = (Curve3D)anaCurv.Copy();
            }
            anaCurv.Transform(loc.Transformation);
        }
        //
        gap = conv.Gap();
        return anaCurv;
    }

    // Static methods

    private static int GetNbPars(CurveKind target)
    {
        int nbPars = 0;
        switch (target)
        {
            case Line:
                nbPars = 0;
                break;
            case Circle:
                nbPars = 1;
                break;
            case Ellipse:
                nbPars = 2;
                break;
            default:
                nbPars = 0;
                break;
        }

        return nbPars;
    }

    private static int GetNbPars(SurfaceKind target)
    {
        int nbPars = 0;
        switch (target)
        {
            case Plane:
                nbPars = 0;
                break;
            case SurfaceKind.Cylinder:
            case SurfaceKind.Sphere:
                nbPars = 1;
                break;
            case SurfaceKind.Cone:
                nbPars = 2;
                break;
            default:
                nbPars = 0;
                break;
        }

        return nbPars;
    }

    private static bool SetConicParameters(CurveKind target, Curve3D? conic, ref Frame3D pos,
                                           NCollection_Array1<double> theParams)
    {
        if (conic is null)
        {
            return false;
        }
        CurveAdaptor gac = new(conic);
        if (gac.GetType() != target)
        {
            return false;
        }

        if (target == Line)
        {
            Line3D lin = gac.Line();
            pos = pos.WithAxis(lin.Position);
        }
        else if (target == Circle)
        {
            Circle3D circ = gac.Circle();
            pos = circ.Position;
            theParams[1] = circ.Radius;
        }
        else if (target == Ellipse)
        {
            Ellipse3D elips = gac.Ellipse();
            pos = elips.Position;
            theParams[1] = elips.MajorRadius;
            theParams[2] = elips.MinorRadius;
        }
        else
        {
            return false;
        }
        return true;
    }

    private static bool CompareConicParams(CurveKind target, double tol, in Frame3D refPos,
                                           NCollection_Array1<double> refParams, in Frame3D pos,
                                           NCollection_Array1<double> theParams)
    {
        int i, nbPars = GetNbPars(target);

        for (i = 1; i <= nbPars; ++i)
        {
            if (Math.Abs(refParams[i] - theParams[i]) > tol)
            {
                return false;
            }
        }

        double angTol = tol / (2.0 * M_PI);
        double aTol = tol;
        if (target == Line)
        {
            aTol = Kerf.Engine.Foundation.Precision.Infinite();
        }

        Axis3D aRef = refPos.Axis;
        Axis3D ax1 = pos.Axis;
        Axis3D ax1Rev = ax1.Reversed();

        return aRef.IsCoaxial(ax1, angTol, aTol) || aRef.IsCoaxial(ax1Rev, angTol, aTol);
    }

    private static bool SetSurfParams(SurfaceKind target, Surface? elemSurf, ref CoordinateSystem pos,
                                      NCollection_Array1<double> theParams)
    {
        //
        if (elemSurf is null)
        {
            return false;
        }
        SurfaceAdaptor gas = new(elemSurf);
        if (gas.GetType() != target)
        {
            return false;
        }

        double nbPars = GetNbPars(target);
        if (theParams.Length() < nbPars)
        {
            return false;
        }

        if (target == Plane)
        {
            Plane3D pln = gas.Plane();
            pos = pln.Position;
        }
        else if (target == SurfaceKind.Cylinder)
        {
            Cylinder3D cyl = gas.Cylinder();
            pos = cyl.Position;
            theParams[1] = cyl.Radius;
        }
        else if (target == SurfaceKind.Cone)
        {
            Cone3D con = gas.Cone();
            pos = con.Position;
            theParams[1] = con.SemiAngle;
            theParams[2] = con.RefRadius;
        }
        else if (target == SurfaceKind.Sphere)
        {
            Sphere3D sph = gas.Sphere();
            pos = sph.Position;
            theParams[1] = sph.Radius;
        }
        else
        {
            return false;
        }
        return true;
    }

    private static bool CompareSurfParams(SurfaceKind target, double tol, in CoordinateSystem refPos,
                                          NCollection_Array1<double> refParams, in CoordinateSystem pos,
                                          NCollection_Array1<double> theParams)
    {
        if (target != Plane)
        {
            if (Math.Abs(refParams[1] - theParams[1]) > tol)
            {
                return false;
            }
        }
        //
        if (target == SurfaceKind.Sphere)
        {
            Point3D refLoc = refPos.Location, loc = pos.Location;
            return refLoc.SquareDistanceTo(loc) <= tol * tol;
        }
        //
        double angTol = tol / (2.0 * M_PI);
        double aTol = tol;
        if (target == SurfaceKind.Cylinder || target == SurfaceKind.Cone)
        {
            aTol = Kerf.Engine.Foundation.Precision.Infinite();
        }

        Axis3D aRef = refPos.Axis;
        Axis3D ax1 = pos.Axis;
        Axis3D ax1Rev = ax1.Reversed();
        if (!(aRef.IsCoaxial(ax1, angTol, aTol) || aRef.IsCoaxial(ax1Rev, angTol, aTol)))
        {
            return false;
        }

        if (target == SurfaceKind.Cone)
        {
            Cone3D refCone = new(refPos, refParams[1], refParams[2]);
            Cone3D cone = new(pos, theParams[1], theParams[2]);
            Point3D refApex = refCone.Apex;
            Point3D apex = cone.Apex;
            return refApex.SquareDistanceTo(apex) <= tol * tol;
        }

        return true;
    }

    private static double DeviationSurfParams(SurfaceKind target, in CoordinateSystem refPos,
                                              NCollection_Array1<double> refParams, in CoordinateSystem pos,
                                              NCollection_Array1<double> theParams)
    {
        double devPars = 0.0;
        if (target != Plane)
        {
            devPars = Math.Abs(refParams[1] - theParams[1]);
        }
        //
        if (target == SurfaceKind.Sphere)
        {
            Point3D refLoc = refPos.Location, loc = pos.Location;
            devPars += refLoc.DistanceTo(loc);
        }
        else
        {
            Direction3D refDir = refPos.Direction;
            Direction3D dir = pos.Direction;
            double angDev = (1.0 - Math.Abs(refDir * dir));
            devPars += angDev;
        }

        return devPars;
    }

    private static bool GetSamplePoints(in Wire wire, double tol, int maxNbInt,
                                        out NCollection_HArray1<Xyz>? points)
    {
        points = null; // Kerf: OCCT leaves the caller's handle unchanged (null) when it returns false
        List<double> lengths = new();
        List<EdgeCurveAdaptor> curves = new();
        List<Xyz> aPoints = new();
        double aTol = Max(1.0e-3, tol / 10.0);
        double totalLength = 0.0;
        Iterator eIter = new(wire);
        for (; eIter.More(); eIter.Next())
        {
            Edge e = ShapeCasts.Edge(eIter.Value());
            if (Tool.Degenerated(e))
            {
                continue;
            }
            EdgeCurveAdaptor bac = new(e);
            double clength = AbscissaPoint.Length(bac, aTol);
            totalLength += clength;
            curves.Add(bac);
            lengths.Add(clength);
        }

        if (totalLength < tol)
        {
            return false;
        }

        int i, nb = lengths.Count;
        for (i = 0; i < nb; ++i)
        {
            EdgeCurveAdaptor c = curves[i];
            double clength = AbscissaPoint.Length(c, aTol);
            int nbPoints = RealToInt(clength / totalLength * maxNbInt + 1);
            nbPoints = Max(2, nbPoints);
            QuasiUniformAbscissa pointGen = new(c, nbPoints);
            if (!pointGen.IsDone)
            {
                continue;
            }
            nbPoints = pointGen.PointCount;
            int j;
            for (j = 1; j <= nbPoints; ++j)
            {
                double t = pointGen.Parameter(j);
                Point3D p = c.Value(t);
                aPoints.Add(p.XYZ());
            }
        }

        if (aPoints.Count < 1)
        {
            return false;
        }

        points = new NCollection_HArray1<Xyz>(1, aPoints.Count);
        for (i = 0; i < aPoints.Count; ++i)
        {
            points.SetValue(i + 1, aPoints[i]);
        }

        return true;
    }

    private static double GetLSGap(NCollection_HArray1<Xyz> points, SurfaceKind target, in CoordinateSystem pos,
                                   NCollection_Array1<double> theParams)
    {
        double gap = 0.0;
        Xyz loc = pos.Location.XYZ();
        Vector3D dir = new(pos.Direction);

        int i;
        if (target == SurfaceKind.Sphere)
        {
            double r = theParams[1];
            for (i = points.Lower(); i <= points.Upper(); ++i)
            {
                Xyz d = points.Value(i) - loc;
                gap = Max(gap, Math.Abs((d.Modulus() - r)));
            }
        }
        else if (target == SurfaceKind.Cylinder)
        {
            double r = theParams[1];
            for (i = points.Lower(); i <= points.Upper(); ++i)
            {
                Vector3D d = new(points.Value(i) - loc);
                d = d.Crossed(dir);
                gap = Max(gap, Math.Abs((d.Magnitude - r)));
            }
        }
        else if (target == SurfaceKind.Cone)
        {
            double ang = theParams[1];
            double r = theParams[2];
            for (i = points.Lower(); i <= points.Upper(); ++i)
            {
                Point3D pi = new(points.Value(i));
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(pos, r, ang, pi, out double u, out double v);
                global::Kerf.Engine.Geometry.ElSLib.ConeD0(u, v, pos, r, ang, out Point3D pp);
                gap = Max(gap, pi.SquareDistanceTo(pp));
            }
            gap = Math.Sqrt(gap);
        }

        return gap;
    }

    private static void FillSolverData(SurfaceKind target, in CoordinateSystem pos, NCollection_Array1<double> theParams,
                                       math_Vector startPoint, math_Vector fBnd, math_Vector lBnd,
                                       double relDev = 0.2)
    {
        if (target == SurfaceKind.Sphere || target == SurfaceKind.Cylinder)
        {
            startPoint[1] = pos.Location.X;
            startPoint[2] = pos.Location.Y;
            startPoint[3] = pos.Location.Z;
            startPoint[4] = theParams[1];
            double dr = relDev * theParams[1];
            double dxyz = dr;
            int i;
            for (i = 1; i <= 3; ++i)
            {
                fBnd[i] = startPoint[i] - dxyz;
                lBnd[i] = startPoint[i] + dxyz;
            }
            fBnd[4] = startPoint[4] - dr;
            lBnd[4] = startPoint[4] + dr;
        }
        if (target == SurfaceKind.Cone)
        {
            startPoint[1] = pos.Location.X;
            startPoint[2] = pos.Location.Y;
            startPoint[3] = pos.Location.Z;
            startPoint[4] = theParams[1]; // SemiAngle
            startPoint[5] = theParams[2]; // Radius
            double dr = relDev * theParams[2];
            if (dr < Kerf.Engine.Foundation.Precision.Confusion())
            {
                dr = 0.1;
            }
            double dxyz = dr;
            double dAng = relDev * Math.Abs(theParams[1]);
            int i;
            for (i = 1; i <= 3; ++i)
            {
                fBnd[i] = startPoint[i] - dxyz;
                lBnd[i] = startPoint[i] + dxyz;
            }
            if (theParams[1] >= 0.0)
            {
                fBnd[4] = startPoint[4] - dAng;
                lBnd[4] = Min(M_PI_2, startPoint[4] + dr); // OCCT: the angle's bound uses the radius step aDR (occt-issues.md)
            }
            else
            {
                fBnd[4] = Max(-M_PI_2, startPoint[4] - dAng);
                lBnd[4] = startPoint[4] + dAng;
            }
            fBnd[5] = startPoint[5] - dr;
            lBnd[5] = startPoint[5] + dr;
        }
    }

    private static void SetCanonicParameters(SurfaceKind target, math_Vector sol, ref CoordinateSystem pos,
                                             NCollection_Array1<double> theParams)
    {
        Point3D loc = new(sol[1], sol[2], sol[3]);
        pos = pos.WithLocation(loc);
        if (target == SurfaceKind.Sphere || target == SurfaceKind.Cylinder)
        {
            theParams[1] = sol[4]; // radius
        }
        else if (target == SurfaceKind.Cone)
        {
            theParams[1] = sol[4]; // semiangle
            theParams[2] = sol[5]; // radius
        }
    }
}
