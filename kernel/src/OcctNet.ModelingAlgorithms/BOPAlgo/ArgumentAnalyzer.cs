// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ArgumentAnalyzer.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ArgumentAnalyzer.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ArgumentAnalyzer.cxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PArgumentAnalyzer.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Booleans.CheckStatus;
using static Kerf.Engine.Booleans.Operation;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>
/// Checks one or two shapes for a Boolean operation: their types for the operation, self-interference, small edges,
/// faces that can't be rebuilt, vertices and edges of the two shapes too close to merge, C0 geometry and curves off
/// their surfaces. Each test is switched on by its mode (all off by default); the faults found are
/// <see cref="GetCheckResult"/>.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: the modes and the operation are OCCT's modifiable references (<c>ref</c> returns).
/// The shapes are only read, so frozen shapes are checked as they are (the self-interference test runs a
/// non-destructive <see cref="CheckerSI"/>). <see cref="GetCheckResult"/> publishes the results, whose shapes
/// are frozen as they are read (<see cref="CheckResult"/>); ported code reads
/// <see cref="InternalGetCheckResult"/>. The tangency test is not implemented in OCCT. OCCT's pointer typedef
/// <c>BOPAlgo_PArgumentAnalyzer</c> is the class reference.
/// </remarks>
public class ArgumentAnalyzer : BooleanAlgorithm
{
    private Shape _shape1 = new();
    private Shape _shape2 = new();
    private bool _stopOnFirst;
    private Operation _operation;
    private bool _argumentTypeMode;
    private bool _selfInterMode;
    private bool _smallEdgeMode;
    private bool _rebuildFaceMode;
    private bool _tangentMode;
    private bool _mergeVertexMode;
    private bool _mergeEdgeMode;
    private bool _continuityMode;
    private bool _curveOnSurfaceMode;
    private bool _empty1;
    private bool _empty2;
    private readonly NCollection_List<CheckResult> _result = new();

    /// <summary>An analyzer with every test off and the operation unknown.</summary>
    public ArgumentAnalyzer()
    {
        _stopOnFirst = false;
        _operation = Operation.Unknown;
        _argumentTypeMode = false;
        _selfInterMode = false;
        _smallEdgeMode = false;
        _rebuildFaceMode = false;
        _tangentMode = false;
        _mergeVertexMode = false;
        _mergeEdgeMode = false;
        _continuityMode = false;
        _curveOnSurfaceMode = false;
        _empty1 = false;
        _empty2 = false;
    }

    /// <summary>Sets the object shape.</summary>
    public void SetShape1(in Shape theShape) => _shape1 = theShape;

    /// <summary>Sets the tool shape.</summary>
    public void SetShape2(in Shape theShape) => _shape2 = theShape;

    /// <summary>The object shape.</summary>
    public Shape GetShape1() => _shape1;

    /// <summary>The tool shape.</summary>
    public Shape GetShape2() => _shape2;

    /// <summary>The operation the shapes are checked for (modifiable).</summary>
    public ref Operation OperationType() => ref _operation;

    /// <summary>Whether the analysis stops at the first fault (modifiable).</summary>
    public ref bool StopOnFirstFaulty() => ref _stopOnFirst;

    /// <summary>The mode checking the types of the shapes (modifiable).</summary>
    public ref bool ArgumentTypeMode() => ref _argumentTypeMode;

    /// <summary>The mode checking self-interference of the shapes (modifiable).</summary>
    public ref bool SelfInterMode() => ref _selfInterMode;

    /// <summary>The mode checking small edges (modifiable).</summary>
    public ref bool SmallEdgeMode() => ref _smallEdgeMode;

    /// <summary>The mode checking whether faces can be split or rebuilt (modifiable).</summary>
    public ref bool RebuildFaceMode() => ref _rebuildFaceMode;

    /// <summary>The mode checking tangency between sub-shapes (modifiable; not implemented in OCCT).</summary>
    public ref bool TangentMode() => ref _tangentMode;

    /// <summary>The mode checking problems of merging vertices (modifiable).</summary>
    public ref bool MergeVertexMode() => ref _mergeVertexMode;

    /// <summary>The mode checking problems of merging edges (modifiable).</summary>
    public ref bool MergeEdgeMode() => ref _mergeEdgeMode;

    /// <summary>The mode checking the continuity of the shapes (modifiable).</summary>
    public ref bool ContinuityMode() => ref _continuityMode;

    /// <summary>The mode checking invalid curves on surfaces (modifiable).</summary>
    public ref bool CurveOnSurfaceMode() => ref _curveOnSurfaceMode;

    /// <summary>Prepares the data.</summary>
    protected void Prepare()
    {
        bool isS1 = _shape1.IsNull, isS2 = _shape2.IsNull;
        if (!isS1)
        {
            _empty1 = AlgoTools3D.IsEmptyShape(_shape1);
        }
        if (!isS2)
        {
            _empty2 = AlgoTools3D.IsEmptyShape(_shape2);
        }
    }

    /// <summary>Performs the analysis; an exception ends it with a result of unknown status.</summary>
    public override void Perform(ProgressRange range = default)
    {
        using var ps = new ProgressScope(range, "Analyze shapes", 10);
        try
        {
            _result.Clear();
            if (UserBreak(ps))
            {
                return;
            }
            // 1. Prepare
            Prepare();
            //
            // 2. Test types
            if (_argumentTypeMode)
            {
                TestTypes();
                if (UserBreak(ps))
                {
                    return;
                }
            }
            //
            // 3. Test self-interference
            if (_selfInterMode)
            {
                TestSelfInterferences(ps.Next(8));
                if (UserBreak(ps))
                {
                    return;
                }
            }
            //
            // 4. Test small edges
            if (_smallEdgeMode)
            {
                if (_result.IsEmpty() || !_stopOnFirst)
                {
                    TestSmallEdge();
                }
                if (UserBreak(ps))
                {
                    return;
                }
            }
            //
            // 5. Test possibility to rebuild faces
            if (_rebuildFaceMode)
            {
                if (_result.IsEmpty() || !_stopOnFirst)
                {
                    TestRebuildFace();
                }
                if (UserBreak(ps))
                {
                    return;
                }
            }
            //
            // 6. Test tangent
            if (_tangentMode)
            {
                if (_result.IsEmpty() || !_stopOnFirst)
                {
                    TestTangent();
                    if (UserBreak(ps))
                    {
                        return;
                    }
                }
            }
            //
            // 7. Test merge vertices
            if (_mergeVertexMode)
            {
                if (_result.IsEmpty() || !_stopOnFirst)
                {
                    TestMergeVertex();
                }
                if (UserBreak(ps))
                {
                    return;
                }
            }
            //
            // 8. Test merge edges
            if (_mergeEdgeMode)
            {
                if (_result.IsEmpty() || !_stopOnFirst)
                {
                    TestMergeEdge();
                }
                if (UserBreak(ps))
                {
                    return;
                }
            }
            //
            // 9. Test shapes continuity
            if (_continuityMode)
            {
                if (_result.IsEmpty() || !_stopOnFirst)
                {
                    TestContinuity();
                }
                if (UserBreak(ps))
                {
                    return;
                }
            }
            //
            // 10. Test validity of the curves on the surfaces
            if (_curveOnSurfaceMode)
            {
                if (_result.IsEmpty() || !_stopOnFirst)
                {
                    TestCurveOnSurface();
                }
            }
        }
        catch (FailureException)
        {
            CheckResult result = new();
            result.SetCheckStatus(CheckStatus.CheckUnknown);
            _result.Append(result);
        }
    }

    /// <summary>True if a fault was found.</summary>
    public bool HasFaulty => !_result.IsEmpty();

    /// <summary>The faults found (their shapes frozen as they are read).</summary>
    public ISchematiqList<CheckResult> GetCheckResult() => PackedList.Own([.. _result]);

    /// <summary>OCCT's <c>GetCheckResult</c>: the analyzer's own list, for ported code.</summary>
    internal NCollection_List<CheckResult> InternalGetCheckResult() => _result;

    /// <summary>Checks the types of the shapes, and for two shapes their dimensions for the operation.</summary>
    protected void TestTypes()
    {
        bool isS1 = _shape1.IsNull, isS2 = _shape2.IsNull;

        if (isS1 && isS2)
        {
            CheckResult result = new();
            result.SetCheckStatus(CheckStatus.BadType);
            _result.Append(result);
            return;
        }

        // single shape check
        if ((isS1 && !isS2) || (!isS1 && isS2))
        {
            bool bIsEmpty = isS1 ? _empty2 : _empty1;

            if (bIsEmpty || _operation != Operation.Unknown)
            {
                Shape s = isS1 ? _shape2 : _shape1;
                CheckResult result = new();
                result.SetShape1(s);
                result.SetCheckStatus(CheckStatus.BadType);
                _result.Append(result);
                return;
            }
        }
        // two shapes check (begin)
        else
        {
            if (_empty1 || _empty2)
            {
                CheckResult result = new();
                if (_empty1 && _empty2)
                {
                    result.SetShape1(_shape1);
                    result.SetShape2(_shape2);
                }
                else
                {
                    Shape s = _empty1 ? _shape1 : _shape2;
                    if (_empty1)
                    {
                        result.SetShape1(s);
                    }
                    else
                    {
                        result.SetShape2(s);
                    }
                }
                result.SetCheckStatus(CheckStatus.BadType);
                _result.Append(result);
                return;
            }
            //
            if (_operation != Operation.Unknown && _operation != Operation.Common)
            {
                Span<int> iDimMin = stackalloc int[2], iDimMax = stackalloc int[2];
                AlgoTools.Dimensions(_shape1, out iDimMin[0], out iDimMax[0]);
                AlgoTools.Dimensions(_shape2, out iDimMin[1], out iDimMax[1]);

                bool bBadTypes =
                    ((_operation == Operation.Fuse)
                     && (iDimMin[0] != iDimMax[0] || iDimMin[1] != iDimMax[1] || iDimMin[0] != iDimMin[1]))
                    || ((_operation == Operation.Cut) && (iDimMax[0] > iDimMin[1]))
                    || ((_operation == Operation.CUT21) && (iDimMin[0] < iDimMax[1]));

                if (bBadTypes)
                {
                    CheckResult result = new();
                    result.SetShape1(_shape1);
                    result.SetShape2(_shape2);
                    result.SetCheckStatus(CheckStatus.BadType);
                    _result.Append(result);
                }
            }
        }
    }

    /// <summary>Checks each shape on self-interference with <see cref="CheckerSI"/>.</summary>
    protected void TestSelfInterferences(ProgressRange range)
    {
        using var ps = new ProgressScope(range, null, (!_shape1.IsNull && !_shape2.IsNull ? 2 : 1));
        int ii;
        //
        for (ii = 0; ii < 2; ii++)
        {
            Shape s = (ii == 0) ? _shape1 : _shape2;
            if (s.IsNull)
            {
                continue;
            }
            //
            bool bIsEmpty = (ii == 0) ? _empty1 : _empty2;
            if (bIsEmpty)
            {
                continue;
            }
            //
            int n1, n2;
            MapOfPair.Iterator itMPK = new();
            NCollection_List<Shape> anArgs = new();
            CheckerSI checker = new();
            //
            anArgs.Append(s);
            checker.SetArguments(anArgs);
            checker.SetNonDestructive(true);
            checker.SetRunParallel(_runParallel);
            checker.SetFuzzyValue(_fuzzyValue);
            //
            checker.Perform(ps.Next());
            bool hasError = checker.HasErrors;
            //
            DS ds = checker.PDS()!;
            MapOfPair mpk = ds.Interferences();
            //
            itMPK.Initialize(mpk);
            for (; itMPK.More(); itMPK.Next())
            {
                Pair pk = itMPK.Value();
                pk.Indices(out n1, out n2);
                if (ds.IsNewShape(n1) || ds.IsNewShape(n2))
                {
                    continue;
                }
                //
                // OCCT.NET: the argument's own sub-shapes (the checker may have worked on a copy, decision 882)
                Shape s1 = checker.Original(ds.Shape(n1));
                Shape s2 = checker.Original(ds.Shape(n2));
                //
                CheckResult result = new();
                if (ii == 0)
                {
                    result.SetShape1(_shape1);
                    result.AddFaultyShape1(s1);
                    if (!s1.IsSame(s2))
                    {
                        result.AddFaultyShape1(s2);
                    }
                }
                else
                {
                    result.SetShape2(_shape2);
                    result.AddFaultyShape2(s1);
                    if (!s1.IsSame(s2))
                    {
                        result.AddFaultyShape2(s2);
                    }
                }
                result.SetCheckStatus(CheckStatus.SelfIntersect);
                _result.Append(result);
            }
            //
            if (hasError)
            {
                CheckResult result = new();
                if (ii == 0)
                {
                    result.SetShape1(_shape1);
                    result.AddFaultyShape1(_shape1);
                }
                else
                {
                    result.SetShape2(_shape2);
                    result.AddFaultyShape2(_shape2);
                }
                result.SetCheckStatus(CheckStatus.OperationAborted);
                _result.Append(result);
            }
        } // for(ii = 0; ii < 2; ii++) {
    }

    /// <summary>Checks the shapes for micro edges (for a section, only those touching the other shape).</summary>
    protected void TestSmallEdge()
    {
        int i = 0;
        DistanceShapeShape dist = new();
        IntersectionContext ctx;
        //
        ctx = new IntersectionContext();

        for (i = 0; i < 2; i++)
        {
            Shape s = (i == 0) ? _shape1 : _shape2;

            if (s.IsNull)
            {
                continue;
            }

            Explorer exp = new(s, ShapeType.Edge);

            for (; exp.More(); exp.Next())
            {
                Edge edge = ShapeCasts.Edge(exp.Current());
                if (Tool.Degenerated(edge))
                {
                    continue;
                }

                if (AlgoTools.IsMicroEdge(edge, ctx))
                {
                    bool bKeepResult = true;

                    if (_operation == Operation.Section)
                    {
                        Shape otherS = (i == 0) ? _shape2 : _shape1;

                        if (!otherS.IsNull)
                        {
                            dist.LoadS2(otherS);

                            bool bVertexIsOnShape = false;
                            int ii = 0;
                            Explorer expV = new(edge, ShapeType.Vertex);

                            for (; expV.More(); expV.Next())
                            {
                                Shape v = expV.Current();

                                dist.LoadS1(v);
                                dist.Perform();

                                if (dist.IsDone)
                                {
                                    for (ii = 1; ii <= dist.SolutionCount; ii++)
                                    {
                                        double tolerance = Tool.Tolerance(ShapeCasts.Vertex(v));
                                        Shape supportShape = dist.SupportOnShape2(ii);

                                        switch (supportShape.ShapeType())
                                        {
                                            case ShapeType.Vertex:
                                            {
                                                tolerance += Tool.Tolerance(ShapeCasts.Vertex(supportShape));
                                                break;
                                            }
                                            case ShapeType.Edge:
                                            {
                                                tolerance += Tool.Tolerance(ShapeCasts.Edge(supportShape));
                                                break;
                                            }
                                            case ShapeType.Face:
                                            {
                                                tolerance += Tool.Tolerance(ShapeCasts.Face(supportShape));
                                                break;
                                            }
                                            default:
                                                break;
                                        }

                                        if (dist.Value() < tolerance)
                                        {
                                            bVertexIsOnShape = true;
                                            break;
                                        }
                                    }
                                }
                            }

                            if (!bVertexIsOnShape)
                            {
                                bKeepResult = false;
                            }
                        }
                    }

                    if (bKeepResult)
                    {
                        CheckResult result = new();

                        if (i == 0)
                        {
                            result.SetShape1(_shape1);
                            result.AddFaultyShape1(edge);
                        }
                        else
                        {
                            result.SetShape2(_shape2);
                            result.AddFaultyShape2(edge);
                        }

                        result.SetCheckStatus(CheckStatus.TooSmallEdge);
                        _result.Append(result);

                        if (_stopOnFirst)
                        {
                            return;
                        }
                    }
                }
            }
        }
    }

    /// <summary>Checks that each face can be rebuilt from its own edges by <see cref="BuilderFace"/>.</summary>
    protected void TestRebuildFace()
    {
        if ((_operation == Operation.Section) || (_operation == Operation.Unknown))
        {
            return;
        }
        int i = 0;

        for (i = 0; i < 2; i++)
        {
            Shape s = (i == 0) ? _shape1 : _shape2;

            if (s.IsNull)
            {
                continue;
            }

            Explorer exp = new(s, ShapeType.Face);
            NCollection_List<Shape> ls = new();

            for (; exp.More(); exp.Next())
            {
                Face face = ShapeCasts.Face(exp.Current());

                Face ff = face;
                ff.Orientation(Forward);
                Explorer expE = new(ff, ShapeType.Edge);
                int nbstartedges = 0;
                ls.Clear();
                //
                for (; expE.More(); expE.Next())
                {
                    Edge e = ShapeCasts.Edge(expE.Current());
                    Orientation oriE = e.Orientation();
                    //
                    if (oriE == Internal)
                    {
                        Edge ee = e;
                        ee.Orientation(Forward);
                        ls.Append(ee);
                        ee.Orientation(Reversed);
                        ls.Append(ee);
                    }
                    else
                    {
                        ls.Append(e);
                    }
                    nbstartedges++;
                }
                BuilderFace bf = new();
                bf.SetFace(face);
                bf.SetShapes(ls);
                bf.Perform();
                NCollection_List<Shape> lf = bf.Areas();
                bool bBadFace = false;

                if (lf.Extent() != 1)
                {
                    bBadFace = true;
                }
                else
                {
                    int nbedgeused = 0;
                    expE.Init(lf.First(), ShapeType.Edge);

                    for (; expE.More(); expE.Next(), nbedgeused++)
                    {
                    }

                    if (nbstartedges != nbedgeused)
                    {
                        bBadFace = true;
                    }
                }

                if (bBadFace)
                {
                    CheckResult result = new();

                    if (i == 0)
                    {
                        result.SetShape1(_shape1);
                        result.AddFaultyShape1(face);
                    }
                    else
                    {
                        result.SetShape2(_shape2);
                        result.AddFaultyShape2(face);
                    }

                    result.SetCheckStatus(CheckStatus.NonRecoverableFace);
                    _result.Append(result);

                    if (_stopOnFirst)
                    {
                        return;
                    }
                }
            }
        }
    }

    /// <summary>Not implemented (as in OCCT).</summary>
    protected void TestTangent()
    {
        // not implemented
    }

    /// <summary>Finds sub-shapes of one shape that would merge with several of the other's.</summary>
    protected void TestMergeSubShapes(ShapeType type)
    {
        if (_shape1.IsNull || _shape2.IsNull)
        {
            return;
        }

        if (_empty1 || _empty2)
        {
            return;
        }

        CheckStatus status = CheckStatus.CheckUnknown;

        switch (type)
        {
            case ShapeType.Vertex:
            {
                status = CheckStatus.IncompatibilityOfVertex;
                break;
            }
            case ShapeType.Edge:
            {
                status = CheckStatus.IncompatibilityOfEdge;
                break;
            }
            case ShapeType.Face:
            {
                status = CheckStatus.IncompatibilityOfFace;
                break;
            }
            default:
                return;
        }
        Explorer exp1 = new(_shape1, type);
        Explorer exp2 = new(_shape2, type);
        NCollection_Sequence<Shape> seq1 = new(), seq2 = new();
        NCollection_Map<Shape> map1 = new(ShapeHasher.Instance), map2 = new(ShapeHasher.Instance);

        for (; exp1.More(); exp1.Next())
        {
            Shape s1 = exp1.Current();

            if (map1.Contains(s1))
            {
                continue;
            }
            seq1.Append(s1);
            map1.Add(s1);
        }

        for (; exp2.More(); exp2.Next())
        {
            Shape s2 = exp2.Current();
            if (map2.Contains(s2))
            {
                continue;
            }
            seq2.Append(s2);
            map2.Add(s2);
        }

        // OCCT.NET: an empty side makes OCCT's Array2(1, 0, ...) an empty array; NCollection_Array2 needs a size
        bool[,] arrayOfFlag = new bool[seq1.Length() + 1, seq2.Length() + 1];
        int i = 0, j = 0;
        for (i = 1; i <= seq1.Length(); i++)
        {
            for (j = 1; j <= seq2.Length(); j++)
            {
                arrayOfFlag[i, j] = false;
            }
        }

        for (i = 1; i <= seq1.Length(); i++)
        {
            Shape s1 = seq1.Value(i);
            NCollection_List<Shape> listOfS2 = new();
            int nbs = 0;

            for (j = 1; j <= seq2.Length(); j++)
            {
                Shape s2 = seq2.Value(j);
                bool bIsEqual = false;

                if (type == ShapeType.Vertex)
                {
                    Vertex v1 = ShapeCasts.Vertex(s1);
                    Vertex v2 = ShapeCasts.Vertex(s2);
                    Point3D p1 = Tool.Pnt(v1);
                    Point3D p2 = Tool.Pnt(v2);
                    double dist = p1.Distance(p2);

                    if (dist <= (Tool.Tolerance(v1) + Tool.Tolerance(v2)))
                    {
                        bIsEqual = true;
                    }
                }
                else if (type == ShapeType.Edge)
                {
                    Edge e1 = ShapeCasts.Edge(s1);
                    Edge e2 = ShapeCasts.Edge(s2);
                    //
                    EdgeEdge ee = new(e1, e2);
                    //
                    ee.Perform();
                    if (ee.IsDone)
                    {
                        NCollection_Sequence<CommonPrt> cPrts = ee.CommonParts();
                        int ii = 0;

                        for (ii = 1; ii <= cPrts.Length(); ii++)
                        {
                            CommonPrt cPart = cPrts.Value(ii);

                            if (cPart.Type() == ShapeType.Edge)
                            {
                                bIsEqual = true;
                            }
                        }
                    }
                }
                else if (type == ShapeType.Face)
                {
                    // not yet implemented!
                }

                if (bIsEqual)
                {
                    arrayOfFlag[i, j] = true;
                    listOfS2.Append(s2);
                    nbs++;
                }
            }

            if (nbs > 1)
            {
                CheckResult result = new();

                result.SetShape1(_shape1);
                result.SetShape2(_shape2);
                result.AddFaultyShape1(s1);
                NCollection_List<Shape>.Iterator it = new(listOfS2);

                for (; it.More(); it.Next())
                {
                    result.AddFaultyShape2(it.Value());
                }

                result.SetCheckStatus(status);
                _result.Append(result);

                if (_stopOnFirst)
                {
                    return;
                }
            }
        }

        for (i = 1; i <= seq2.Length(); i++)
        {
            Shape s2 = seq2.Value(i);
            NCollection_List<Shape> listOfS1 = new();
            int nbs = 0;

            for (j = 1; j <= seq1.Length(); j++)
            {
                Shape s1 = seq1.Value(j);

                if (arrayOfFlag[j, i])
                {
                    listOfS1.Append(s1);
                    nbs++;
                }
            }

            if (nbs > 1)
            {
                CheckResult result = new();

                result.SetShape1(_shape1);
                result.SetShape2(_shape2);
                NCollection_List<Shape>.Iterator it = new(listOfS1);

                for (; it.More(); it.Next())
                {
                    result.AddFaultyShape1(it.Value());
                }
                result.AddFaultyShape2(s2);

                result.SetCheckStatus(status);
                _result.Append(result);

                if (_stopOnFirst)
                {
                    return;
                }
            }
        }
    }

    /// <summary>Checks the vertices of the two shapes for merging problems.</summary>
    protected void TestMergeVertex()
    {
        TestMergeSubShapes(ShapeType.Vertex);
    }

    /// <summary>Checks the edges of the two shapes for merging problems.</summary>
    protected void TestMergeEdge()
    {
        TestMergeSubShapes(ShapeType.Edge);
    }

    /// <summary>Lists the edges and faces whose geometry is only C0.</summary>
    protected void TestContinuity()
    {
        int i, j, nbS;
        Explorer exp = new();
        //
        for (i = 0; i < 2; ++i)
        {
            Shape s = i == 0 ? _shape1 : _shape2;
            if (s.IsNull)
            {
                continue;
            }
            //
            NCollection_IndexedMap<Shape> ms = new(ShapeHasher.Instance);
            // Edges
            exp.Init(s, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                if (Tool.Degenerated(e))
                {
                    continue;
                }
                Curve3D? c = Tool.Curve(e, out _, out _);
                if (c!.Continuity() == Continuity.C0)
                {
                    ms.Add(e);
                }
            }
            // Faces
            exp.Init(s, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                Face f = ShapeCasts.Face(exp.Current());
                Surface surf = Tool.Surface(f)!;
                if (surf.Continuity() == Continuity.C0)
                {
                    ms.Add(f);
                }
            }
            //
            // add shapes with continuity C0 to result
            nbS = ms.Extent();
            for (j = 1; j <= nbS; ++j)
            {
                Shape fs = ms.FindKey(j);
                CheckResult result = new();
                if (i == 0)
                {
                    result.SetShape1(_shape1);
                    result.AddFaultyShape1(fs);
                }
                else
                {
                    result.SetShape2(_shape2);
                    result.AddFaultyShape2(fs);
                }
                result.SetCheckStatus(CheckStatus.GeomAbs_C0);
                _result.Append(result);
            }
        }
    }

    /// <summary>Lists the edges whose curves are farther from their faces' surfaces than their tolerance.</summary>
    protected void TestCurveOnSurface()
    {
        int i;
        double t, d, tolE;
        Explorer expF = new(), expE = new();
        //
        for (i = 0; i < 2; i++)
        {
            Shape s = (i == 0) ? _shape1 : _shape2;
            if (s.IsNull)
            {
                continue;
            }
            //
            expF.Init(s, ShapeType.Face);
            for (; expF.More(); expF.Next())
            {
                Face f = ShapeCasts.Face(expF.Current());
                //
                expE.Init(f, ShapeType.Edge);
                for (; expE.More(); expE.Next())
                {
                    Edge e = ShapeCasts.Edge(expE.Current());
                    //
                    if (AlgoTools.ComputeTolerance(f, e, out d, out t))
                    {
                        tolE = Tool.Tolerance(e);
                        if (d > tolE)
                        {
                            CheckResult result = new();
                            result.SetCheckStatus(CheckStatus.InvalidCurveOnSurface);
                            if (i == 0)
                            {
                                result.SetShape1(_shape1);
                                result.AddFaultyShape1(e);
                                result.AddFaultyShape1(f);
                                result.SetMaxDistance1(d);
                                result.SetMaxParameter1(t);
                            }
                            else
                            {
                                result.SetShape2(_shape2);
                                result.AddFaultyShape2(e);
                                result.AddFaultyShape2(f);
                                result.SetMaxDistance2(d);
                                result.SetMaxParameter2(t);
                            }
                            _result.Append(result);
                        }
                    }
                }
            }
        }
    }
}
