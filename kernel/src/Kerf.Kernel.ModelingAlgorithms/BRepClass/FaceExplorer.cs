// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FaceExplorer.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FaceExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification.Topological;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Classification;

/// <summary>
/// The exploration of a face for its classification: the half-lines from the point to classify, the wires and their
/// edges (with their pcurves).
/// </summary>
/// <remarks>A working object (decision 21). It only reads the face.</remarks>
public class FaceExplorer : IFaceExplorer<ClassifierEdge>
{
    private const double Probing_Start = 0.123;
    private const double Probing_End = 0.7;
    private const double Probing_Step = 0.2111;

    private Face _face;
    private readonly Explorer _wExplorer = new();
    private readonly Explorer _eExplorer = new();
    private int _curEdgeInd;
    private double _curEdgePar;
    private double _maxTolerance;
    private bool _useBndBox;

    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> _mapVE =
        new(ShapeHasher.Instance);

    private double _uMin;
    private double _uMax;
    private double _vMin;
    private double _vMax;

    /// <summary>An explorer of the face <paramref name="f"/> (taken FORWARD).</summary>
    public FaceExplorer(in Face f)
    {
        _face = f;
        _curEdgeInd = 1;
        _curEdgePar = Probing_Start;
        _maxTolerance = 0.1;
        _useBndBox = false;
        _uMin = global::Kerf.Engine.Foundation.Precision.Infinite();
        _uMax = -global::Kerf.Engine.Foundation.Precision.Infinite();
        _vMin = global::Kerf.Engine.Foundation.Precision.Infinite();
        _vMax = -global::Kerf.Engine.Foundation.Precision.Infinite();
        _face.Orientation(Forward);
    }

    /// <summary>Computes the UV bounds of the face.</summary>
    protected void ComputeFaceBounds()
    {
        Surface surface = Tool.Surface(_face, out Placement _)!;
        surface.Bounds(out _uMin, out _uMax, out _vMin, out _vMax);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_uMin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(_uMax)
            || global::Kerf.Engine.Foundation.Precision.IsInfinite(_vMin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(_vMax))
        {
            BrepTools.UVBounds(_face, out _uMin, out _uMax, out _vMin, out _vMax);
        }
    }

    /// <summary>
    /// Checks the point, and moves it if it is too far from the face's UV box: to the line from the box's centre to
    /// the point. Returns true if the point was not changed.
    /// </summary>
    public bool CheckPoint(ref Point2D point)
    {
        if (_uMin > _uMax)
        {
            ComputeFaceBounds();
        }

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_uMin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(_uMax)
            || global::Kerf.Engine.Foundation.Precision.IsInfinite(_vMin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(_vMax))
        {
            return true;
        }

        Point2D centerPnt = new((_uMin + _uMax) / 2, (_vMin + _vMax) / 2);
        double distance = centerPnt.Distance(point);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(distance))
        {
            point.SetCoord(_uMin - (_uMax - _uMin), _vMin - (_vMax - _vMin));
            return false;
        }
        else
        {
            double epsilon = Epsilon(distance);
            if (epsilon > Max(_uMax - _uMin, _vMax - _vMin))
            {
                Vector2D linVec = new(centerPnt, point);
                Direction2D linDir = new(linVec);
                point = new Point2D(centerPnt.XY() + linDir.XY() * (2.0 * epsilon));
                return false;
            }
        }

        return true;
    }

    /// <summary>True if the point is outside a bounding volume of the face: always false.</summary>
    public virtual bool Reject(in Point2D p) => false;

    /// <summary>
    /// A half-line from <paramref name="p"/> in <paramref name="l"/>, with in <paramref name="par"/> the parameter up
    /// to which it meets the face's boundary at least once. Starts again from the first edge.
    /// </summary>
    public bool Segment(in Point2D p, out Line2D l, out double par)
    {
        _curEdgeInd = 1;
        _curEdgePar = Probing_Start;

        return OtherSegment(p, out l, out par);
    }

    /// <summary>Another half-line from <paramref name="p"/> (see <see cref="Segment"/>): each call gives another one.</summary>
    public bool OtherSegment(in Point2D p, out Line2D l, out double par)
    {
        var expF = new Explorer(_face, ShapeType.Edge);
        int i;
        double fPar;
        double lPar;
        Curve2D? c2d;
        double tolParConf2 = global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion();
        Point2D pOnC;
        double paramIn;
        // C++ leaves L and Par unwritten for the edges it skips; they are written before every return
        l = new Line2D();
        par = 0.0;

        for (i = 1; expF.More(); expF.Next(), i++)
        {
            if (i != _curEdgeInd)
            {
                continue;
            }

            Shape localShape = expF.Current();
            Orientation orientation = localShape.Orientation();

            if (orientation == Forward || orientation == Reversed)
            {
                Edge edge = ShapeCasts.Edge(localShape);

                c2d = Tool.CurveOnSurface(edge, _face, out fPar, out lPar);

                if (c2d is not null)
                {
                    // Treatment of infinite cases.
                    if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(fPar))
                    {
                        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(lPar))
                        {
                            fPar = -1.0;
                            lPar = 1.0;
                        }
                        else
                        {
                            fPar = lPar - 1.0;
                        }
                    }
                    else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(lPar))
                    {
                        lPar = fPar + 1.0;
                    }

                    for (; _curEdgePar < Probing_End; _curEdgePar += Probing_Step)
                    {
                        paramIn = _curEdgePar * fPar + (1.0 - _curEdgePar) * lPar;

                        c2d.D1(paramIn, out pOnC, out Vector2D tanVec);
                        par = pOnC.SquareDistance(p);

                        if (par > tolParConf2)
                        {
                            Vector2D linVec = new(p, pOnC);
                            Direction2D linDir = new(linVec);

                            double tanMod = tanVec.SquareMagnitude();
                            if (tanMod < tolParConf2)
                            {
                                continue;
                            }

                            tanVec /= Math.Sqrt(tanMod);
                            double sinA = tanVec.Crossed(new Vector2D(linDir.XY()));
                            const double smallAngle = 0.001;
                            bool isSmallAngle = false;
                            if (Math.Abs(sinA) < smallAngle)
                            {
                                isSmallAngle = true;
                                // The line from the input point P to the current point on edge
                                // is tangent to the edge curve. This condition is bad for classification.
                                // Therefore try to go to another point in the hope that there will be
                                // no tangent. If there tangent is preserved then leave the last point in
                                // order to get this edge chance to participate in classification.
                                if (_curEdgePar + Probing_Step < Probing_End)
                                {
                                    continue;
                                }
                            }

                            l = new Line2D(p, linDir);

                            // Check if ends of a curve lie on a line.
                            c2d.D0(fPar, out pOnC);
                            Point2D fpOnC = pOnC;
                            if (l.SquareDistance(pOnC) > tolParConf2)
                            {
                                c2d.D0(lPar, out pOnC);
                                if (l.SquareDistance(pOnC) > tolParConf2)
                                {
                                    if (isSmallAngle)
                                    {
                                        // Try to find minimal distance between curve and line

                                        var proj = new ProjectPointOnCurve2D();
                                        proj.Init(p, c2d, fPar, lPar);
                                        if (proj.NbPoints() > 0)
                                        {
                                            Point2D lpOnC = pOnC;
                                            double fDist = p.SquareDistance(fpOnC);
                                            double lDist = p.SquareDistance(lpOnC);
                                            double minDist = proj.LowerDistance();
                                            minDist *= minDist;
                                            pOnC = proj.NearestPoint();
                                            if (minDist > fDist)
                                            {
                                                minDist = fDist;
                                                pOnC = fpOnC;
                                            }

                                            if (minDist > lDist)
                                            {
                                                minDist = lDist;
                                                pOnC = lpOnC;
                                            }

                                            if (minDist < par)
                                            {
                                                par = minDist;
                                                if (par < tolParConf2)
                                                {
                                                    continue;
                                                }

                                                linVec.SetXY((pOnC.XY() - p.XY()));
                                                linDir.SetXY(linVec.XY());
                                                l = new Line2D(p, linDir);
                                            }
                                        }
                                    }

                                    _curEdgePar += Probing_Step;
                                    if (_curEdgePar >= Probing_End)
                                    {
                                        _curEdgeInd++;
                                        _curEdgePar = Probing_Start;
                                    }

                                    par = Math.Sqrt(par);
                                    return true;
                                }
                            }
                        }
                    }
                } // if (!aC2d.IsNull()) {
            } // if (anOrientation == TopAbs_FORWARD ...

            // This curve is not valid for line construction. Go to another edge.
            _curEdgeInd++;
            _curEdgePar = Probing_Start;
        }

        // nothing found, return an horizontal line
        par = RealLast();
        l = new Line2D(p, new Direction2D(Direction2D.D.X));

        return false;
    }

    /// <summary>Starts an exploration of the wires.</summary>
    public void InitWires() => _wExplorer.Init(_face, ShapeType.Wire);

    /// <summary>True if there is a current wire.</summary>
    public bool MoreWires() => _wExplorer.More();

    /// <summary>Moves to the next wire.</summary>
    public void NextWire() => _wExplorer.Next();

    /// <summary>True if the wire's bounding volume doesn't meet the segment: always false.</summary>
    public virtual bool RejectWire(in Line2D l, double par) => false;

    /// <summary>Starts an exploration of the edges of the current wire.</summary>
    public void InitEdges()
    {
        _eExplorer.Init(_wExplorer.Current(), ShapeType.Edge);
        _mapVE.Clear();
        Exploration.MapShapesAndAncestors(_wExplorer.Current(), ShapeType.Vertex, ShapeType.Edge, _mapVE);
    }

    /// <summary>True if there is a current edge.</summary>
    public bool MoreEdges() => _eExplorer.More();

    /// <summary>Moves to the next edge.</summary>
    public void NextEdge() => _eExplorer.Next();

    /// <summary>True if the edge's bounding volume doesn't meet the segment: always false.</summary>
    public virtual bool RejectEdge(in Line2D l, double par) => false;

    /// <summary>
    /// Writes the current edge of the current wire into <paramref name="e"/>, with its orientation. As in OCCT, the
    /// next edge of <paramref name="e"/> is left as it was when the current edge has none (see
    /// <see cref="ClassifierEdge.SetNextEdge"/>).
    /// </summary>
    public void CurrentEdge(ref ClassifierEdge e, out Orientation or)
    {
        // OCCT: the next edge left from the previous edge reaches the intersector's CheckSkip for an edge without one
        // (a closed edge, or a vertex with more than two edges) (occt-issues.md)
        e.ChangeEdge() = ShapeCasts.Edge(_eExplorer.Current());
        e.ChangeFace() = _face;
        or = e.Edge().Orientation();
        e.SetNextEdge(_mapVE);
        e.SetMaxTolerance(_maxTolerance);
        e.SetUseBndBox(_useBndBox);
    }

    /// <summary>The maximum tolerance given to the edges.</summary>
    public double MaxTolerance() => _maxTolerance;

    /// <summary>Sets the maximum tolerance from which the intersector checks for a vertex with a high tolerance.</summary>
    public void SetMaxTolerance(double value) => _maxTolerance = value;

    /// <summary>True if the intersector uses bounding boxes.</summary>
    public bool UseBndBox() => _useBndBox;

    /// <summary>Sets whether the intersector uses bounding boxes.</summary>
    public void SetUseBndBox(bool value) => _useBndBox = value;
}
