// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_GeomTool.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_GeomTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Geometry.DerivativeStatus;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The mesher's geometric tools: the discretization points of a curve or of an iso curve of a surface for given
/// deflections (through <see cref="TangentialDeflection"/>), and the statics the mesher shares (normals,
/// segment intersections, the deflection of a segment, the cell counts of the circles' grid).
/// </summary>
/// <remarks>A working object, as OCCT's: <see cref="AddPoint"/> changes the discretization in place.</remarks>
internal sealed class GeomTool
{
    /// <summary>The states of the intersection check of two segments.</summary>
    public enum IntFlag
    {
        NoIntersection,
        Cross,
        EndPointTouch,
        PointOnSegment,
        Glued,
        Same,
    }

    private readonly Edge? _edge;
    private readonly TangentialDeflection _discretTool = new();
    private readonly IsoType _isoType;

    /// <summary>
    /// Discretizes the curve between the parameters with the linear and angular deflections, in at least
    /// <paramref name="minPointsNb"/> points, segments not shorter than <paramref name="minSize"/>.
    /// </summary>
    public GeomTool(EdgeCurveAdaptor curve,
                             double firstParam,
                             double lastParam,
                             double linDeflection,
                             double angDeflection,
                             int minPointsNb = 2,
                             double minSize = 1.0e-7 /* Precision::Confusion() */)
    {
        _edge = curve.Edge();
        _isoType = NoneIso;
        _discretTool.Initialize(curve,
                                 firstParam,
                                 lastParam,
                                 angDeflection,
                                 linDeflection,
                                 minPointsNb,
                                 global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                 minSize);
    }

    /// <summary>
    /// Discretizes the iso curve of the surface (U or V iso at <paramref name="paramIso"/>) between the parameters
    /// with the linear and angular deflections.
    /// </summary>
    public GeomTool(FaceSurfaceAdaptor surface,
                             IsoType isoType,
                             double paramIso,
                             double firstParam,
                             double lastParam,
                             double linDeflection,
                             double angDeflection,
                             int minPointsNb = 2,
                             double minSize = 1.0e-7 /* Precision::Confusion() */)
    {
        _edge = null;
        _isoType = isoType;
        IsoCurve iso = new(surface, isoType, paramIso, firstParam, lastParam);

        _discretTool.Initialize(iso,
                                 firstParam,
                                 lastParam,
                                 angDeflection,
                                 linDeflection,
                                 minPointsNb,
                                 global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                 minSize);
    }

    /// <summary>
    /// Adds a point to the computed points, or replaces one within the parametric tolerance if
    /// <paramref name="isReplace"/>; returns its index.
    /// </summary>
    public int AddPoint(in Point3D point, double param, bool isReplace = true)
    {
        return _discretTool.AddPoint(point, param, isReplace);
    }

    /// <summary>The number of discretization points.</summary>
    public int PointCount => _discretTool.PointCount;

    /// <summary>
    /// The discretization point with the given index (from 1) on the iso curve: its parameter, the 3D point and the
    /// point in the surface's parametric space, with <paramref name="isoParam"/> as the other coordinate.
    /// </summary>
    public bool Value(int index, double isoParam, out double param, out Point3D point, out Point2D uv)
    {
        param = 0.0;
        point = new Point3D();
        uv = new Point2D();
        if (index < 1 || index > PointCount)
        {
            return false;
        }

        point = _discretTool.Value(index);
        param = _discretTool.Parameter(index);

        if (_isoType == IsoU)
        {
            uv = uv.WithCoord(isoParam, param);
        }
        else
        {
            uv = uv.WithCoord(param, isoParam);
        }

        return true;
    }

    /// <summary>
    /// The discretization point with the given index (from 1) on the edge: its parameter, the 3D point and the point
    /// of the edge's pcurve on the surface's face. False for an iso curve.
    /// </summary>
    public bool Value(int index, FaceSurfaceAdaptor surface, out double param, out Point3D point,
                      out Point2D uv)
    {
        param = 0.0;
        point = new Point3D();
        uv = new Point2D();
        if (index < 1 || index > PointCount)
        {
            return false;
        }

        if (_edge is null)
        {
            return false;
        }

        point = _discretTool.Value(index);
        param = _discretTool.Parameter(index);

        Face face = surface.Face();

        Curve2D? curve = Tool.CurveOnSurface(_edge.Value, face, out _, out _);

        curve!.D0(param, out uv);
        return true;
    }

    // static API

    /// <summary>
    /// The point and the normal of the surface at (U, V), reversed on a reversed face; false if the normal can't be
    /// computed.
    /// </summary>
    public static bool Normal(FaceSurfaceAdaptor surface,
                              double paramU,
                              double paramV,
                              out Point3D point,
                              out Direction3D normal)
    {
        bool isOK = true;
        normal = new Direction3D();
        surface.D1(paramU, paramV, out point, out Vector3D d1U, out Vector3D d1V);

        global::Kerf.Engine.Geometry.CSLib.Normal(d1U, d1V, global::Kerf.Engine.Foundation.Precision.Angular(), out DerivativeStatus status, ref normal);
        if (status != Done)
        {
            surface.D2(paramU, paramV, out point, out d1U, out d1V, out Vector3D d2U, out Vector3D d2V,
                          out Vector3D d2UV);
            global::Kerf.Engine.Geometry.CSLib.Normal(d1U,
                         d1V,
                         d2U,
                         d2V,
                         d2UV,
                         global::Kerf.Engine.Foundation.Precision.Angular(),
                         out isOK,
                         out NormalStatus _,
                         ref normal);
        }

        if (!isOK)
        {
            return false;
        }

        Face face = surface.Face();
        Orientation ori = face.Orientation;
        if (ori == Reversed)
        {
            normal = normal.Reversed();
        }

        return true;
    }

    /// <summary>
    /// The intersection of the two lines through the segments' points: <see cref="IntFlag.Cross"/> with the point and
    /// its parameters on both segments (0 at the start point, 1 at the end point), <see cref="IntFlag.Same"/> for one
    /// line, or <see cref="IntFlag.NoIntersection"/> for parallel lines.
    /// </summary>
    /// <param name="startPnt1">The start point of the first segment.</param>
    /// <param name="endPnt1">The end point of the first segment.</param>
    /// <param name="startPnt2">The start point of the second segment.</param>
    /// <param name="endPnt2">The end point of the second segment.</param>
    /// <param name="intPnt">The intersection point, written on a cross.</param>
    /// <param name="paramOnSegment">Two values (OCCT's <c>double (&amp;)[2]</c>), written on a cross.</param>
    public static IntFlag IntLinLin(in Xy startPnt1,
                                    in Xy endPnt1,
                                    in Xy startPnt2,
                                    in Xy endPnt2,
                                    ref Xy intPnt,
                                    Span<double> paramOnSegment)
    {
        Xy vec1 = endPnt1 - startPnt1;
        Xy vec2 = endPnt2 - startPnt2;
        Xy vecO1O2 = startPnt2 - startPnt1;

        double crossD1D2 = vec1 ^ vec2;
        double crossD1D3 = vecO1O2 ^ vec2;

        double prec = Constants.Resolution();
        // Are edgegs codirectional
        if (Math.Abs(crossD1D2) < prec)
        {
            // Just a parallel case?
            if (Math.Abs(crossD1D3) < prec)
            {
                return IntFlag.Same;
            }
            else
            {
                return IntFlag.NoIntersection;
            }
        }

        paramOnSegment[0] = crossD1D3 / crossD1D2;
        intPnt = startPnt1 + paramOnSegment[0] * vec1;

        double crossD2D3 = vecO1O2 ^ vec1;
        paramOnSegment[1] = crossD2D3 / crossD1D2;
        return IntFlag.Cross;
    }

    /// <summary>
    /// The intersection of the two segments, within both. A touch at end points is
    /// <see cref="IntFlag.EndPointTouch"/> if <paramref name="isConsiderEndPointTouch"/> (else no intersection); an end
    /// point on the other segment is <see cref="IntFlag.PointOnSegment"/> (with the point) if
    /// <paramref name="isConsiderPointOnSegment"/>; <see cref="IntFlag.Cross"/> writes the intersection point.
    /// </summary>
    public static IntFlag IntSegSeg(in Xy startPnt1,
                                    in Xy endPnt1,
                                    in Xy startPnt2,
                                    in Xy endPnt2,
                                    bool isConsiderEndPointTouch,
                                    bool isConsiderPointOnSegment,
                                    ref Point2D intPnt)
    {
        Span<int> pointHash = [ClassifyPoint(startPnt1, endPnt1, startPnt2),
                                ClassifyPoint(startPnt1, endPnt1, endPnt2),
                                ClassifyPoint(startPnt2, endPnt2, startPnt1),
                                ClassifyPoint(startPnt2, endPnt2, endPnt1)];

        int posHash = pointHash[0] + pointHash[1] + pointHash[2] + pointHash[3];

        // Consider case when edges have shared vertex
        if (pointHash[0] < 0 || pointHash[1] < 0)
        {
            if (posHash == -1)
            {
                // -1 means, that 2 points are equal, and 1 point is on another curve
                return IntFlag.Glued;
            }
            else
            {
                if (isConsiderEndPointTouch)
                {
                    return IntFlag.EndPointTouch;
                }

                return IntFlag.NoIntersection;
            }
        }

        // 1) hash code == 1: an end point of one segment on the other.
        // 2) hash code == 2: overlapping segments.
        if (posHash == 1)
        {
            if (isConsiderPointOnSegment)
            {
                // OCCT: aPointHash[0] and [1] classify the second segment's end points on the first segment, [2] and
                // [3] the first's on the second, but the point given back is the first segment's start for [0], its
                // end for [1], ... (occt-issues.md, unit 4A). BRepMesh_Delaun doesn't use the point in this case.
                if (pointHash[0] == 1)
                {
                    intPnt = new Point2D(startPnt1);
                }
                else if (pointHash[1] == 1)
                {
                    intPnt = new Point2D(endPnt1);
                }
                else if (pointHash[2] == 1)
                {
                    intPnt = new Point2D(startPnt2);
                }
                else
                {
                    intPnt = new Point2D(endPnt2);
                }

                return IntFlag.PointOnSegment;
            }

            return IntFlag.NoIntersection;
        }
        else if (posHash == 2)
        {
            return IntFlag.Glued;
        }

        Span<double> param = stackalloc double[2];
        Xy intCoord = intPnt.XY();
        IntFlag intFlag =
            IntLinLin(startPnt1, endPnt1, startPnt2, endPnt2, ref intCoord, param);
        intPnt = new Point2D(intCoord);
        if (intFlag == IntFlag.NoIntersection)
        {
            return IntFlag.NoIntersection;
        }

        if (intFlag == IntFlag.Same)
        {
            if (posHash < -2)
            {
                return IntFlag.Same;
            }
            else if (posHash == -1)
            {
                return IntFlag.Glued;
            }

            return IntFlag.NoIntersection;
        }

        // Cross
        // Intersection is out of segments ranges
        double prec = global::Kerf.Engine.Foundation.Precision.PConfusion();
        double endPrec = 1 - prec;
        for (int i = 0; i < 2; ++i)
        {
            if (param[i] < prec || param[i] > endPrec)
            {
                return IntFlag.NoIntersection;
            }
        }

        return IntFlag.Cross;
    }

    /// <summary>
    /// The square of the deflection of the segment from its middle point: the distance from the line through the
    /// end points, or from the first point if they coincide.
    /// </summary>
    public static double SquareDeflectionOfSegment(in Point3D firstPoint, in Point3D lastPoint, in Point3D midPoint)
    {
        // 23.03.2010 skl for OCC21645 - change precision for comparison
        if (firstPoint.SquareDistanceTo(lastPoint) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            Line3D lin = new(firstPoint, new Direction3D(new Vector3D(firstPoint, lastPoint)));
            return lin.SquareDistanceTo(midPoint);
        }

        return firstPoint.SquareDistanceTo(midPoint);
    }

    /// <summary>
    /// The cell counts along U and V of the circles' grid, estimated from the face's deflection and complexity:
    /// <c>2 ^ log10(estimated points count)</c> along each parametric direction, and
    /// <c>2 ^ log10(initial vertex count)</c> along a linear one. (-1, -1) without a range splitter.
    /// </summary>
    public static (int, int) CellsCount(SurfaceAdaptorBase surface,
                                        int verticesNb,
                                        double deflection,
                                        DefaultRangeSplitter? rangeSplitter)
    {
        if (rangeSplitter is null)
        {
            return (-1, -1);
        }

        SurfaceKind type = surface.GetType();

        ComputeErrFactors(deflection, surface, out double errFactorU, out double errFactorV);

        (double first, double second) rangeU = rangeSplitter.GetRangeU();
        (double first, double second) rangeV = rangeSplitter.GetRangeV();
        (double first, double second) delta = rangeSplitter.GetDelta();

        int cellsCountU, cellsCountV;
        if (type == SurfaceKind.Torus)
        {
            cellsCountU =
                (int)Math.Ceiling(Math.Pow(2, Math.Log10((rangeU.second - rangeU.first) / delta.first)));
            cellsCountV =
                (int)Math.Ceiling(Math.Pow(2, Math.Log10((rangeV.second - rangeV.first) / delta.second)));
        }
        else if (type == SurfaceKind.Cylinder)
        {
            cellsCountU =
                (int)Math.Ceiling(Math.Pow(2,
                                           Math.Log10((rangeU.second - rangeU.first) / delta.first
                                                      / (rangeV.second - rangeV.first))));
            cellsCountV =
                (int)Math.Ceiling(Math.Pow(2, Math.Log10((rangeV.second - rangeV.first) / errFactorV)));
        }
        else
        {
            cellsCountU = (int)Math.Ceiling(
                Math.Pow(2, Math.Log10((rangeU.second - rangeU.first) / delta.first / errFactorU)));
            cellsCountV = (int)Math.Ceiling(
                Math.Pow(2, Math.Log10((rangeV.second - rangeV.first) / delta.second / errFactorV)));
        }

        AdjustCellsCounts(surface, verticesNb, ref cellsCountU, ref cellsCountV);
        return (cellsCountU, cellsCountV);
    }

    // Classifies the point in case of coincidence of two vectors: zero if the point is out of the segment, -1 if it
    // coincides with an end point, 1 if it is between them.
    private static int ClassifyPoint(in Xy point1, in Xy point2, in Xy pointToCheck)
    {
        Xy p1 = point2 - point1;
        Xy p2 = pointToCheck - point1;

        double prec = global::Kerf.Engine.Foundation.Precision.PConfusion();
        double sqPrec = prec * prec;
        double dist = Math.Abs(p1 ^ p2);
        if (dist > prec)
        {
            dist = (dist * dist) / p1.SquareModulus();
            if (dist > sqPrec)
            {
                return 0; // out
            }
        }

        Xy mult = p1.Multiplied(p2);
        if (mult.X() < 0.0 || mult.Y() < 0.0)
        {
            return 0; // out
        }

        if (p1.SquareModulus() < p2.SquareModulus())
        {
            return 0; // out
        }

        if (pointToCheck.IsEqual(point1, prec) || pointToCheck.IsEqual(point2, prec))
        {
            return -1; // coincides with an end point
        }

        return 1;
    }

    private static void ComputeErrFactors(double deflection,
                                          SurfaceAdaptorBase face,
                                          out double errFactorU,
                                          out double errFactorV)
    {
        errFactorU = deflection * 10.0;
        errFactorV = deflection * 10.0;

        switch (face.GetType())
        {
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
                break;

            case SurfaceOfExtrusion:
            case SurfaceOfRevolution:
            {
                CurveAdaptorBase curve = face.BasisCurve();
                if (curve.GetType() == BSplineCurve && curve.Degree() > 2)
                {
                    errFactorV /= (curve.Degree() * curve.KnotCount);
                }
                break;
            }
            case SurfaceKind.BezierSurface:
            {
                if (face.UDegree() > 2)
                {
                    errFactorU /= (face.UDegree());
                }
                if (face.VDegree() > 2)
                {
                    errFactorV /= (face.VDegree());
                }
                break;
            }
            case SurfaceKind.BSplineSurface:
            {
                if (face.UDegree() > 2)
                {
                    errFactorU /= (face.UDegree() * face.UKnotCount);
                }
                if (face.VDegree() > 2)
                {
                    errFactorV /= (face.VDegree() * face.VKnotCount);
                }
                break;
            }

            case Plane:
            default:
                errFactorU = errFactorV = 1.0;
                break;
        }
    }

    private static void AdjustCellsCounts(SurfaceAdaptorBase face,
                                          int nbVertices,
                                          ref int cellsCountU,
                                          ref int cellsCountV)
    {
        SurfaceKind type = face.GetType();
        if (type == OtherSurface)
        {
            // fallback to the default behavior
            cellsCountU = cellsCountV = -1;
            return;
        }

        double sqNbVert = nbVertices;
        if (type == Plane)
        {
            cellsCountU = cellsCountV = (int)Math.Ceiling(Math.Pow(2, Math.Log10(sqNbVert)));
        }
        else if (type == SurfaceKind.Cylinder || type == SurfaceKind.Cone)
        {
            cellsCountV = (int)Math.Ceiling(Math.Pow(2, Math.Log10(sqNbVert)));
        }
        else if (type == SurfaceOfExtrusion || type == SurfaceOfRevolution)
        {
            CurveAdaptorBase curve = face.BasisCurve();
            if (curve.GetType() == Line
                || (curve.GetType() == BSplineCurve && curve.Degree() < 2))
            {
                // planar, cylindrical, conical cases
                if (type == SurfaceOfExtrusion)
                {
                    cellsCountU = (int)Math.Ceiling(Math.Pow(2, Math.Log10(sqNbVert)));
                }
                else
                {
                    cellsCountV = (int)Math.Ceiling(Math.Pow(2, Math.Log10(sqNbVert)));
                }
            }
            if (type == SurfaceOfExtrusion)
            {
                // V is always a line
                cellsCountV = (int)Math.Ceiling(Math.Pow(2, Math.Log10(sqNbVert)));
            }
        }
        else if (type == SurfaceKind.BezierSurface || type == SurfaceKind.BSplineSurface)
        {
            if (face.UDegree() < 2)
            {
                cellsCountU = (int)Math.Ceiling(Math.Pow(2, Math.Log10(sqNbVert)));
            }
            if (face.VDegree() < 2)
            {
                cellsCountV = (int)Math.Ceiling(Math.Pow(2, Math.Log10(sqNbVert)));
            }
        }

        cellsCountU = Max(cellsCountU, 2);
        cellsCountV = Max(cellsCountV, 2);
    }
}
