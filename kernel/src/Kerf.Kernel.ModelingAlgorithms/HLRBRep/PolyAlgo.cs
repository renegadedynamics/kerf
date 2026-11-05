// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_PolyAlgo.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_PolyAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using System.Runtime.CompilerServices;
using static Kerf.Engine.Geometry.DerivativeStatus;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.HiddenLines.Algorithm.PolyMask;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The polygonal hidden-line removal: hides the edges of meshed shapes (their faces' triangulations, their edges'
/// polygons on them, the polygons of free edges) for a projector, with the outlines found on the meshes.
/// </summary>
/// <remarks>
/// <para>
/// A working object (a transient), as OCCT's. Load shapes, set the projector, <see cref="Update"/>, then iterate on
/// the segments with <see cref="InitHide"/>/<see cref="MoreHide"/>/<see cref="NextHide"/> and <see cref="Hide"/>
/// (or read HLRBRep_PolyHLRToShape's compounds). Faces without a triangulation are skipped: the shapes must be meshed
/// first.
/// </para>
/// <para>
/// The algorithm only reads its shapes (their triangulations, polygons and geometry); it builds its own internal
/// triangulation, so nothing is copied at the entry (unit 7Q2). The shapes the segments come from
/// (<see cref="Hide"/>, <see cref="Show"/>) are the caller's own edges and faces.
/// </para>
/// </remarks>
internal class PolyAlgo : Transient
{
    // Node flags of the internal triangulation (OCCT's file-scope enum)
    private const int NMsk_Vert = 1;
    private const int NMsk_OutL = 2;
    private const int NMsk_Norm = 4;
    private const int NMsk_Fuck = 8;
    private const int NMsk_Edge = 16;
    private const int NMsk_Move = 32;

    private AlgorithmProjector _proj = new();
    private readonly double[,] _tMat = new double[3, 3];
    private readonly double[] _tLoc = new double[3];
    private readonly double[,] _ttMa = new double[3, 3];
    private readonly double[] _ttLo = new double[3];
    private readonly double[,] _tiMa = new double[3, 3];
    private readonly double[] _tiLo = new double[3];
    private readonly NCollection_Sequence<Shape> _shapes = new();
    private readonly IndexedSet<Shape> _eMap = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _fMap = new(ShapeHasher.Instance);
    private readonly AlgoPolyAlgo _algo;
    private bool _debug;
    private double _tolSta;
    private double _tolEnd;
    private double _tolAngular;
    private Surface? _gSurf;
    private readonly FaceSurfaceAdaptor _bSurf = new();
    private readonly EdgeCurveAdaptor _bCurv = new();
    private readonly EdgeCurve2DAdaptor _pc = new();

    public PolyAlgo()
    {
        _debug = false;
        _tolSta = 0.1;
        _tolEnd = 0.9;
        _tolAngular = 0.001;
        _algo = new AlgoPolyAlgo();
    }

    /// <summary>OCCT's copy: shares the other's HLRAlgo_PolyAlgo, copies its settings, projector and shapes.</summary>
    public PolyAlgo(PolyAlgo other)
    {
        _debug = other.Debug();
        _tolAngular = other.TolAngular();
        _tolSta = other.TolCoef();
        _tolEnd = 1.0 - _tolSta;
        _algo = other.Algo();
        _proj = other.Projector();

        int nbShapes = other.ShapeCount;
        for (int i = 1; i <= nbShapes; ++i)
        {
            Load(other.Shape(i));
        }
    }

    public PolyAlgo(in Shape shape)
    {
        _debug = false;
        _tolSta = 0.1;
        _tolEnd = 0.9;
        _tolAngular = 0.001;
        _shapes.Append(shape);
        _algo = new AlgoPolyAlgo();
    }

    public int ShapeCount => _shapes.Length();

    /// <summary>The shape of index theIndex.</summary>
    public Shape Shape(int index) => _shapes.Value(index);

    /// <summary>Removes the shape of index theIndex.</summary>
    public void Remove(int index)
    {
        Standard_OutOfRange_Raise_if(index == 0 || index > _shapes.Length(), "HLRBRep_PolyAlgo::Remove : unknown Shape");
        _shapes.Remove(index);
        _algo.Clear();
        _eMap.Clear();
        _fMap.Clear();
    }

    /// <summary>The index of the shape theShape, 0 if it isn't loaded.</summary>
    public int Index(in Shape shape)
    {
        int i = 1;
        for (int k = 1; k <= _shapes.Length(); ++k, ++i)
        {
            if (_shapes.Value(k).IsEqual(shape))
            {
                return i;
            }
        }
        return 0;
    }

    /// <summary>Adds the shape theShape (meshed: its faces' triangulations are used).</summary>
    public void Load(in Shape shape) => _shapes.Append(shape);

    public AlgoPolyAlgo Algo() => _algo;

    public AlgorithmProjector Projector() => _proj;

    public void Projector(in AlgorithmProjector proj) => _proj = proj;

    public double TolAngular() => _tolAngular;

    public void TolAngular(double tol) => _tolAngular = tol;

    public double TolCoef() => _tolSta;

    public void TolCoef(double tol)
    {
        _tolSta = tol;
        _tolEnd = 1.0 - tol;
    }

    /// <summary>Builds the polygonal data of the shapes and prepares the hiding.</summary>
    public void Update()
    {
        _algo.Clear();
        _eMap.Clear();
        _fMap.Clear();

        Shape shape = MakeShape();
        if (shape.IsNull)
        {
            return;
        }

        bool isIsoledF = false, isIsoledE = false;
        int nbShell = InitShape(shape, ref isIsoledF, ref isIsoledE);
        if (nbShell <= 0)
        {
            return;
        }

        Exploration.MapShapes(shape, ShapeType.Edge, _eMap);
        Exploration.MapShapes(shape, ShapeType.Face, _fMap);
        int nbEdge = _eMap.Count;
        int nbFace = _fMap.Count;
        NCollection_Array1<int> es = new(0, nbEdge); // index of the Shell
        NCollection_Array1<PolyData?> pd = new(0, nbFace);
        NCollection_Array1<PolyInternalData?> pid = new(0, nbFace);
        HashSet<Shape> shapeMap1 = new(ShapeHasher.Instance),
                                      shapeMap2 = new(ShapeHasher.Instance);
        NCollection_Array1<PolyShellData?> shell = _algo.ChangePolyShell();
        int iShell = 0;
        for (Explorer shellIter = new(shape, ShapeType.Shell); shellIter.More(); shellIter.Next())
        {
            StoreShell(shellIter.Current(), ref iShell, shell, false, false, es, pd, pid, shapeMap1, shapeMap2);
        }
        if (isIsoledF)
        {
            StoreShell(shape, ref iShell, shell, isIsoledF, false, es, pd, pid, shapeMap1, shapeMap2);
        }
        if (isIsoledE)
        {
            StoreShell(shape, ref iShell, shell, false, isIsoledE, es, pd, pid, shapeMap1, shapeMap2);
        }
        _algo.Update();
    }

    public void InitHide() => _algo.InitHide();

    public bool MoreHide() => _algo.MoreHide();

    public void NextHide() => _algo.NextHide();

    /// <summary>
    /// Hides the current segment: sets theStatus to its visible and hidden parts and returns its points; theShape is
    /// the edge it comes from (the face, for an internal outline).
    /// </summary>
    public ref AlgoBiPoint.PointsT Hide(EdgeStatus status, out Shape shape, out bool reg1,
                                           out bool regn, out bool outl, out bool intl)
    {
        int index = 0;
        ref AlgoBiPoint.PointsT points =
            ref _algo.Hide(status, out index, out reg1, out regn, out outl, out intl);
        shape = intl ? _fMap[index - 1] : _eMap[index - 1];
        return ref points;
    }

    public void InitShow() => _algo.InitShow();

    public bool MoreShow() => _algo.MoreShow();

    public void NextShow() => _algo.NextShow();

    /// <summary>The current segment without hiding: its points, and the edge (or face) it comes from.</summary>
    public ref AlgoBiPoint.PointsT Show(out Shape shape, out bool reg1, out bool regn,
                                           out bool outl, out bool intl)
    {
        int index = 0;
        ref AlgoBiPoint.PointsT points = ref _algo.Show(out index, out reg1, out regn, out outl, out intl);
        shape = intl ? _fMap[index - 1] : _eMap[index - 1];
        return ref points;
    }

    public bool Debug() => _debug;

    public void Debug(bool debug) => _debug = debug;

    private Shape MakeShape()
    {
        if (_shapes.IsEmpty())
        {
            return new Shape();
        }

        BrepBuilder builder = new();
        Compound comp = new();
        builder.MakeCompound(ref comp);
        for (int i = 1; i <= _shapes.Length(); i++)
        {
            builder.Add(comp, _shapes.Value(i));
        }
        return comp;
    }

    private int InitShape(in Shape shape, ref bool isoledF, ref bool isoledE)
    {
        HashSet<Shape> shapeMap0 = new(ShapeHasher.Instance);
        int nbShell = 0;
        isoledF = false;
        isoledE = false;
        Explorer faceIter = new();
        Placement loc;

        for (Explorer shellIter = new(shape, ShapeType.Shell); shellIter.More(); shellIter.Next())
        {
            bool hasTrian = false;
            for (faceIter.Init(shellIter.Current(), ShapeType.Face); faceIter.More(); faceIter.Next())
            {
                Face face = ShapeCasts.Face(faceIter.Current());
                if (Tool.Triangulation(face, out loc) is not null)
                {
                    if (shapeMap0.Add(face))
                    {
                        hasTrian = true;
                    }
                }
            }
            if (hasTrian)
            {
                ++nbShell;
            }
        }

        for (faceIter.Init(shape, ShapeType.Face, ShapeType.Shell); faceIter.More() && !isoledF; faceIter.Next())
        {
            Face face = ShapeCasts.Face(faceIter.Current());
            if (Tool.Triangulation(face, out loc) is not null)
            {
                if (shapeMap0.Add(face))
                {
                    isoledF = true;
                }
            }
        }
        if (isoledF)
        {
            ++nbShell;
        }

        for (Explorer edgeIter = new(shape, ShapeType.Edge,
            ShapeType.Face); edgeIter.More() && !isoledE; edgeIter.Next())
        {
            isoledE = true;
        }
        if (isoledE)
        {
            ++nbShell;
        }
        if (nbShell > 0)
        {
            _algo.Init(nbShell);
        }
        return nbShell;
    }


    private void StoreShell(in Shape shape, ref int iShell,
        NCollection_Array1<PolyShellData?> shell, bool isoledF, bool isoledE,
        NCollection_Array1<int> es, NCollection_Array1<PolyData?> pd,
        NCollection_Array1<PolyInternalData?> pid, HashSet<Shape> shapeMap1,
        HashSet<Shape> shapeMap2)
    {
        Placement loc;
        Explorer faceExp = new(), edgeExp = new();
        int nbFaceShell = 0;
        bool isClosed = false;
        Transform3D projTrsf = _proj.Transformation();
        {
            Xyz trsfVec = projTrsf.TranslationXyz();
            _tLoc[0] = trsfVec.X();
            _tLoc[1] = trsfVec.Y();
            _tLoc[2] = trsfVec.Z();

            Matrix3x3 trsfMat = projTrsf.VectorialPart;
            _tMat[0, 0] = trsfMat.Value(1, 1);
            _tMat[0, 1] = trsfMat.Value(1, 2);
            _tMat[0, 2] = trsfMat.Value(1, 3);
            _tMat[1, 0] = trsfMat.Value(2, 1);
            _tMat[1, 1] = trsfMat.Value(2, 2);
            _tMat[1, 2] = trsfMat.Value(2, 3);
            _tMat[2, 0] = trsfMat.Value(3, 1);
            _tMat[2, 1] = trsfMat.Value(3, 2);
            _tMat[2, 2] = trsfMat.Value(3, 3);
        }
        {
            Transform3D trsfInv = _proj.InvertedTransformation();
            Xyz trsfInvVec = trsfInv.TranslationXyz();
            _tiLo[0] = trsfInvVec.X();
            _tiLo[1] = trsfInvVec.Y();
            _tiLo[2] = trsfInvVec.Z();

            Matrix3x3 trsfInvMat = trsfInv.VectorialPart;
            _tiMa[0, 0] = trsfInvMat.Value(1, 1);
            _tiMa[0, 1] = trsfInvMat.Value(1, 2);
            _tiMa[0, 2] = trsfInvMat.Value(1, 3);
            _tiMa[1, 0] = trsfInvMat.Value(2, 1);
            _tiMa[1, 1] = trsfInvMat.Value(2, 2);
            _tiMa[1, 2] = trsfInvMat.Value(2, 3);
            _tiMa[2, 0] = trsfInvMat.Value(3, 1);
            _tiMa[2, 1] = trsfInvMat.Value(3, 2);
            _tiMa[2, 2] = trsfInvMat.Value(3, 3);
        }
        if (!isoledE)
        {
            if (!isoledF)
            {
                isClosed = shape.ClosedFlag;
                if (!isClosed)
                {
                    IndexedSet<Shape> em = new(ShapeHasher.Instance);
                    Exploration.MapShapes(shape, ShapeType.Edge, em);
                    int nbEdge = em.Count;
                    NCollection_Array1<int> flagArray = new(1, Max(nbEdge, 1));
                    flagArray.Init(0);
                    for (edgeExp.Init(shape, ShapeType.Edge); edgeExp.More(); edgeExp.Next())
                    {
                        Edge edge = ShapeCasts.Edge(edgeExp.Current());
                        int edgeIndex = em.IndexOf(edge) + 1;
                        Orientation edgeOrient = edge.Orientation;
                        if (!Tool.Degenerated(edge))
                        {
                            if (edgeOrient == Forward)
                            {
                                flagArray[edgeIndex] += 1;
                            }
                            else if (edgeOrient == Reversed)
                            {
                                flagArray[edgeIndex] -= 1;
                            }
                        }
                    }
                    isClosed = true;

                    for (int edgeIter = 1; edgeIter <= nbEdge && isClosed; ++edgeIter)
                    {
                        isClosed = (flagArray[edgeIter] == 0);
                    }
                }

                faceExp.Init(shape, ShapeType.Face);
            }
            else
            {
                faceExp.Init(shape, ShapeType.Face, ShapeType.Shell);
            }

            for (; faceExp.More(); faceExp.Next())
            {
                Face face = ShapeCasts.Face(faceExp.Current());
                if (Tool.Triangulation(face, out loc) is not null)
                {
                    if (shapeMap1.Add(face))
                    {
                        ++nbFaceShell;
                    }
                }
            }
        }
        if (nbFaceShell > 0 || isoledE)
        {
            ++iShell;
            shell.SetValue(iShell, new PolyShellData(nbFaceShell));
        }

        if (nbFaceShell > 0)
        {
            PolyShellData psd = shell[iShell]!;
            int iFace = 0;
            if (!isoledF)
            {
                faceExp.Init(shape, ShapeType.Face);
            }
            else
            {
                faceExp.Init(shape, ShapeType.Face, ShapeType.Shell);
            }

            for (; faceExp.More(); faceExp.Next())
            {
                Face face = ShapeCasts.Face(faceExp.Current());
                Triangulation? tr = Tool.Triangulation(face, out loc);
                if (tr is not null)
                {
                    if (shapeMap2.Add(face))
                    {
                        iFace++;
                        int faceIndex = _fMap.IndexOf(face) + 1;
                        bool isReversed = face.Orientation == Reversed;
                        Transform3D tt = loc.Transformation;
                        tt = tt.PreMultiplied(projTrsf);
                        {
                            Xyz tTrsfVec = tt.TranslationXyz();
                            _ttLo[0] = tTrsfVec.X();
                            _ttLo[1] = tTrsfVec.Y();
                            _ttLo[2] = tTrsfVec.Z();

                            Matrix3x3 tTrsfMat = tt.VectorialPart;
                            _ttMa[0, 0] = tTrsfMat.Value(1, 1);
                            _ttMa[0, 1] = tTrsfMat.Value(1, 2);
                            _ttMa[0, 2] = tTrsfMat.Value(1, 3);
                            _ttMa[1, 0] = tTrsfMat.Value(2, 1);
                            _ttMa[1, 1] = tTrsfMat.Value(2, 2);
                            _ttMa[1, 2] = tTrsfMat.Value(2, 3);
                            _ttMa[2, 0] = tTrsfMat.Value(3, 1);
                            _ttMa[2, 1] = tTrsfMat.Value(3, 2);
                            _ttMa[2, 2] = tTrsfMat.Value(3, 3);
                        }
                        int nbNodes = tr.NodeCount;
                        int nbTris = tr.TriangleCount;
                        pd.SetValue(faceIndex, new PolyData());
                        psd.PolyData().SetValue(iFace, pd.Value(faceIndex));
                        pid.SetValue(faceIndex, new PolyInternalData(nbNodes, nbTris));
                        PolyInternalData aPid = pid[faceIndex]!;
                        Surface? surf = Tool.Surface(face);
                        if (surf is not null)
                        {
                            if (surf is TrimmedSurface rectTrimSurf)
                            {
                                surf = rectTrimSurf.BasisSurface;
                            }
                            SurfaceAdaptor surfAdapt = new(surf);
                            aPid.Planar(surfAdapt.GetType() == Plane);
                        }
                        else
                        {
                            aPid.Planar(false);
                        }

                        NCollection_Array1<TriangleData> tData = aPid.TData();
                        NCollection_Array1<PolyInternalSegment> piSeg = aPid.PISeg();
                        NCollection_Array1<PolyInternalNode?> piNod = aPid.PINod();
                        for (int triIter = 1; triIter <= nbTris; ++triIter)
                        {
                            Triangle1 polyTri = tr.Triangle(triIter);
                            ref TriangleData triData = ref tData[triIter];
                            triData.Flags = 0;
                            if (isReversed)
                            {
                                polyTri.Get(out triData.Node3, out triData.Node2, out triData.Node1);
                            }
                            else
                            {
                                polyTri.Get(out triData.Node1, out triData.Node2, out triData.Node3);
                            }
                        }

                        for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                        {
                            Point3D pnt = tr.Node(nodeIter);
                            PolyInternalNode polyINode = piNod[nodeIter]!;
                            ref PolyInternalNode.NodeData nod1RValues = ref polyINode.Data();
                            ref PolyInternalNode.NodeIndices nodIndices = ref polyINode.Indices();
                            nodIndices.NdSg = 0;
                            nodIndices.Flag = 0;
                            nod1RValues.Point = pnt.XYZ();
                            TTMultiply(ref nod1RValues.Point);
                        }
                        aPid.UpdateLinks(tData, piSeg, piNod);
                        if (tr.HasUVNodes)
                        {
                            bool hasSurf = Tool.IsGeometric(face);
                            _bSurf.Initialize(face, false);
                            for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                            {
                                PolyInternalNode polyINode =
                                    piNod[nodeIter]!;
                                ref PolyInternalNode.NodeIndices nodIndices = ref polyINode.Indices();
                                ref PolyInternalNode.NodeData nod1RValues = ref polyINode.Data();
                                if (tr.HasUVNodes)
                                {
                                    nod1RValues.UV = tr.UVNode(nodeIter).XY();
                                }
                                if (tr.HasNormals)
                                {
                                    nod1RValues.Normal = tr.Normal(nodeIter).XYZ();
                                }

                                if ((tr.HasNormals || (hasSurf && tr.HasUVNodes)) && Normal(nodeIter,
                                    ref nodIndices, ref nod1RValues, tData, piSeg, piNod, false))
                                {
                                    nodIndices.Flag |= NMsk_Norm;
                                }
                                else
                                {
                                    nodIndices.Flag &= ~NMsk_Norm;
                                    nod1RValues.Scal = 0;
                                }
                            }
                        }

                        for (int triIter = 1; triIter <= nbTris; ++triIter)
                        {
                            ref TriangleData triData = ref tData[triIter];
                            PolyInternalNode pn1 = piNod[triData.Node1]!;
                            PolyInternalNode pn2 = piNod[triData.Node2]!;
                            PolyInternalNode pn3 = piNod[triData.Node3]!;
                            OrientTriangle(triIter, ref triData, ref pn1.Indices(), ref pn1.Data(),
                                ref pn2.Indices(), ref pn2.Data(), ref pn3.Indices(), ref pn3.Data());
                        }
                    }
                }
            }

            int nbFaces = _fMap.Count;
            NCollection_List<AlgoBiPoint> list = psd.Edges();
            IndexedMap<Shape, NCollection_List<Shape>> ef = new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(shape, ShapeType.Edge, ShapeType.Face, ef);
            for (int faceIter = 1; faceIter <= nbFaces; ++faceIter)
            {
                if (pid.Value(faceIter) is not null)
                {
                    for (edgeExp.Init(_fMap[faceIter - 1], ShapeType.Edge); edgeExp.More(); edgeExp.Next())
                    {
                        Edge edge = ShapeCasts.Edge(edgeExp.Current());
                        if (shapeMap1.Add(edge))
                        {
                            int edgeIndex = _eMap.IndexOf(edge) + 1;
                            es.SetValue(edgeIndex, iShell);
                            int indexE = ef.IndexOf(edge) + 1;
                            if (indexE > 0)
                            {
                                NCollection_List<Shape> LS = ef.ValueAt(indexE - 1);
                                InitBiPointsWithConnexity(edgeIndex, ref edge, list, pid, LS, true);
                            }
                            else
                            {
                                NCollection_List<Shape> LS = new();
                                InitBiPointsWithConnexity(edgeIndex, ref edge, list, pid, LS, false);
                            }
                        }
                    }
                }
            }
            InsertOnOutLine(pid);
            CheckFrBackTriangles(list, pid);
            UpdateOutLines(list, pid);
            UpdateEdgesBiPoints(list, pid, isClosed);
            UpdatePolyData(pd, pid, isClosed);

            for (int faceIter = 1; faceIter <= nbFaces; ++faceIter)
            {
                pid[faceIter] = null;
            }
        }
        else if (isoledE)
        {
            PolyShellData psd = shell[iShell]!;
            NCollection_List<AlgoBiPoint> list = psd.Edges();
            for (edgeExp.Init(shape, ShapeType.Edge, ShapeType.Face); edgeExp.More(); edgeExp.Next())
            {
                Edge edge = ShapeCasts.Edge(edgeExp.Current());
                if (shapeMap1.Add(edge))
                {
                    int edgeIndex = _eMap.IndexOf(edge) + 1;
                    es.SetValue(edgeIndex, iShell);
                    NCollection_List<Shape> ls = new();
                    InitBiPointsWithConnexity(edgeIndex, ref edge, list, pid, ls, false);
                }
            }
        }
    }


    private bool Normal(int nodeIndex, ref PolyInternalNode.NodeIndices nodIndices,
        ref PolyInternalNode.NodeData nod1RValues, NCollection_Array1<TriangleData> triData,
        NCollection_Array1<PolyInternalSegment> piSeg, NCollection_Array1<PolyInternalNode?> piNod,
        bool toOrient)
    {
        if (nod1RValues.Normal.SquareModulus() < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Vector3D d1U, d1V;
            Point3D pnt;
            DerivativeStatus status = D1IsNull;
            _bSurf.D1(nod1RValues.UV.X(), nod1RValues.UV.Y(), out pnt, out d1U, out d1V);
            Direction3D norm = new();
            global::Kerf.Engine.Geometry.CSLib.Normal(d1U, d1V, global::Kerf.Engine.Foundation.Precision.Angular(), out status, ref norm);
            if (status != Done)
            {
                Vector3D d2U, d2V, d2UV;
                bool isOK = false;
                NormalStatus nromStatus;
                _bSurf.D2(nod1RValues.UV.X(), nod1RValues.UV.Y(), out pnt, out d1U, out d1V, out d2U, out d2V, out d2UV);
                global::Kerf.Engine.Geometry.CSLib.Normal(d1U, d1V, d2U, d2V, d2UV, global::Kerf.Engine.Foundation.Precision.Angular(), out isOK, out nromStatus, ref norm);
                if (!isOK)
                {
                    return false;
                }
            }
            nod1RValues.Normal = norm.XYZ();
        }

        TMultiply(ref nod1RValues.Normal, _proj.Perspective());

        Xyz averNorm = new();
        if (AverageNormal(nodeIndex, ref nodIndices, triData, piSeg, piNod, ref averNorm))
        {
            if (nod1RValues.Normal * averNorm < 0)
            {
                nod1RValues.Normal.Reverse();
            }

            Xyz eyeDir = new(0.0, 0.0, -1.0);
            if (_proj.Perspective())
            {
                eyeDir.SetCoord(nod1RValues.Point.X(), nod1RValues.Point.Y(),
                    nod1RValues.Point.Z() - _proj.Focus());
                double eyeMod = eyeDir.Modulus();
                if (eyeMod > 0.0)
                {
                    eyeDir /= eyeMod;
                }
            }
            nod1RValues.Scal = (nod1RValues.Normal * eyeDir);
        }
        else
        {
            nod1RValues.Scal = 0;
            nod1RValues.Normal = new Xyz(1.0, 0.0, 0.0);
        }
        if (nod1RValues.Scal > 0)
        {
            if (nod1RValues.Scal < _tolAngular)
            {
                nod1RValues.Scal = 0;
                nodIndices.Flag |= NMsk_OutL;
            }
        }
        else
        {
            if (-nod1RValues.Scal < _tolAngular)
            {
                nod1RValues.Scal = 0;
                nodIndices.Flag |= NMsk_OutL;
            }
        }

        if (toOrient)
        {
            UpdateAroundNode(nodeIndex, ref nodIndices, triData, piSeg, piNod);
        }
        return true;
    }


    private bool AverageNormal(int iNode, ref PolyInternalNode.NodeIndices nodeIndices,
        NCollection_Array1<TriangleData> tData, NCollection_Array1<PolyInternalSegment> piSeg,
        NCollection_Array1<PolyInternalNode?> piNod, ref double x, ref double y, ref double z)
    {
        bool isOK = false;
        int jNode = 0, kNode = 0, iiii;
        x = 0;
        y = 0;
        z = 0;
        iiii = nodeIndices.NdSg;
        while (iiii != 0 && !isOK)
        {
            ref PolyInternalSegment segIndices = ref piSeg[iiii];
            int iTri1 = segIndices.Conex1;
            int iTri2 = segIndices.Conex2;
            if (iTri1 != 0)
            {
                AddNormalOnTriangle(iTri1, iNode, ref jNode, tData, piNod, ref x, ref y, ref z, ref isOK);
            }
            if (iTri2 != 0)
            {
                AddNormalOnTriangle(iTri2, iNode, ref jNode, tData, piNod, ref x, ref y, ref z, ref isOK);
            }
            if (segIndices.LstSg1 == iNode)
            {
                iiii = segIndices.NxtSg1;
            }
            else
            {
                iiii = segIndices.NxtSg2;
            }
        }

        if (jNode != 0)
        {
            iiii = nodeIndices.NdSg;

            while (iiii != 0 && !isOK)
            {
                ref PolyInternalSegment segIndices = ref piSeg[iiii];
                int iTri1 = segIndices.Conex1;
                int iTri2 = segIndices.Conex2;
                if (iTri1 != 0)
                {
                    AddNormalOnTriangle(iTri1, jNode, ref kNode, tData, piNod, ref x, ref y, ref z, ref isOK);
                }
                if (iTri2 != 0)
                {
                    AddNormalOnTriangle(iTri2, jNode, ref kNode, tData, piNod, ref x, ref y, ref z, ref isOK);
                }
                if (segIndices.LstSg1 == jNode)
                {
                    iiii = segIndices.NxtSg1;
                }
                else
                {
                    iiii = segIndices.NxtSg2;
                }
            }
        }
        double d = Math.Sqrt(x * x + y * y + z * z);
        if (isOK && d < 1.0e-10)
        {
            isOK = false;
        }
        return isOK;
    }


    private void AddNormalOnTriangle(int iTri, int iNode, ref int jNode,
        NCollection_Array1<TriangleData> tData, NCollection_Array1<PolyInternalNode?> piNod,
        ref double x, ref double y, ref double z, ref bool isOK)
    {
        ref TriangleData triangle = ref tData[iTri];
        ref PolyInternalNode.NodeData nod1RValues = ref piNod.Value(triangle.Node1)!.Data();
        ref PolyInternalNode.NodeData nod2RValues = ref piNod.Value(triangle.Node2)!.Data();
        ref PolyInternalNode.NodeData nod3RValues = ref piNod.Value(triangle.Node3)!.Data();
        Xyz d1 = nod2RValues.Point - nod1RValues.Point;
        double d1Norm = d1.Modulus();
        if (d1Norm < 1.0e-10)
        {
            if (triangle.Node1 == iNode)
            {
                jNode = triangle.Node2;
            }
            else if (triangle.Node2 == iNode)
            {
                jNode = triangle.Node1;
            }
        }
        else
        {
            Xyz d2 = nod3RValues.Point - nod2RValues.Point;
            double d2Norm = d2.Modulus();
            if (d2Norm < 1.0e-10)
            {
                if (triangle.Node2 == iNode)
                {
                    jNode = triangle.Node3;
                }
                else if (triangle.Node3 == iNode)
                {
                    jNode = triangle.Node2;
                }
            }
            else
            {
                Xyz d3 = nod1RValues.Point - nod3RValues.Point;
                double d3Norm = d3.Modulus();
                if (d3Norm < 1.0e-10)
                {
                    if (triangle.Node3 == iNode)
                    {
                        jNode = triangle.Node1;
                    }
                    else if (triangle.Node1 == iNode)
                    {
                        jNode = triangle.Node3;
                    }
                }
                else
                {
                    Xyz dn = (1 / (d1Norm * d2Norm)) * (d1 ^ d2);
                    double dnNorm = dn.Modulus();
                    if (dnNorm > 1.0e-10)
                    {
                        isOK = true;
                        x += dn.X();
                        y += dn.Y();
                        z += dn.Z();
                    }
                }
            }
        }
    }


    private void InitBiPointsWithConnexity(int iEdge, ref Edge edge,
        NCollection_List<AlgoBiPoint> list, NCollection_Array1<PolyInternalData?> pid,
        NCollection_List<Shape> ls, bool isConnex)
    {
        double x1, y1, z1, x2, y2, z2;
        double xti1, yti1, zti1, xti2, yti2, zti2;
        double u1 = 0.0, u2 = 0.0;
        PolygonOnTriangulation?[] hPol = new PolygonOnTriangulation?[2];
        Placement loc;
        _bCurv.Initialize(edge);
        if (isConnex)
        {
            int nbConnex = ls.Extent();
            if (nbConnex == 1)
            {
                NCollection_List<Shape>.Iterator itn = new(ls);
                Face f1 = ShapeCasts.Face(itn.Value());
                int i1 = _fMap.IndexOf(f1) + 1;
                Triangulation? tr1 = Tool.Triangulation(f1, out loc);
                hPol[0] = Tool.PolygonOnTriangulation(edge, tr1, loc);
                PolyInternalData pid1 = pid.Value(i1)!;
                if (hPol[0] is not null)
                {
                    _pc.Initialize(edge, f1);
                    NCollection_HArray1<double> par = hPol[0]!.Parameters()!;
                    NCollection_Array1<int> pol1 = hPol[0]!.Nodes();
                    int nbPol = pol1.Upper();
                    NCollection_Array1<TriangleData> tData1 = pid1.TData();
                    NCollection_Array1<PolyInternalSegment> piSeg1 = pid1.PISeg();
                    NCollection_Array1<PolyInternalNode?> piNod1 = pid1.PINod();
                    PolyInternalNode pi1p1 = piNod1[pol1[1]]!;
                    ref PolyInternalNode.NodeIndices node11Indices = ref pi1p1.Indices();
                    ref PolyInternalNode.NodeData nod11RValues = ref pi1p1.Data();
                    PolyInternalNode pi1p2 = piNod1[pol1[nbPol]]!;
                    ref PolyInternalNode.NodeIndices node12Indices = ref pi1p2.Indices();
                    ref PolyInternalNode.NodeData nod12RValues = ref pi1p2.Data();
                    node11Indices.Flag |= NMsk_Vert;
                    node12Indices.Flag |= NMsk_Vert;

                    for (int iPol = 1; iPol <= nbPol; iPol++)
                    {
                        PolyInternalNode pi1pA = piNod1[pol1[iPol]]!;
                        ref PolyInternalNode.NodeIndices nodeIndices1A = ref pi1pA.Indices();
                        ref PolyInternalNode.NodeData Nod1ARValues = ref pi1pA.Data();
                        if (nodeIndices1A.Edg1 == 0 || nodeIndices1A.Edg1 == iEdge)
                        {
                            nodeIndices1A.Edg1 = iEdge;
                            Nod1ARValues.PCu1 = par.Value(iPol);
                        }
                        else
                        {
                            nodeIndices1A.Edg2 = iEdge;
                            Nod1ARValues.PCu2 = par.Value(iPol);
                        }
                    }

                    int i1p2 = pol1[1];
                    node12Indices = ref node11Indices;
                    nod12RValues = ref nod11RValues;
                    xti2 = x2 = nod12RValues.Point.X();
                    yti2 = y2 = nod12RValues.Point.Y();
                    zti2 = z2 = nod12RValues.Point.Z();
                    if (node12Indices.Edg1 == iEdge)
                    {
                        u2 = nod12RValues.PCu1;
                    }
                    else if (node12Indices.Edg2 == iEdge)
                    {
                        u2 = nod12RValues.PCu2;
                    }
                    node12Indices.Flag |= NMsk_Edge;
                    TIMultiply(ref xti2, ref yti2, ref zti2);
                    if (pol1[1] == pol1[nbPol] && _pc.IsPeriodic)
                    {
                        u2 = u2 - _pc.Period();
                    }

                    if (nbPol == 2 && Tool.Degenerated(edge))
                    {
                        CheckDegeneratedSegment(ref node11Indices, ref nod11RValues, ref node12Indices, ref nod12RValues);
                        UpdateAroundNode(pol1[1], ref node11Indices, tData1, piSeg1, piNod1);
                        UpdateAroundNode(pol1[nbPol], ref node12Indices, tData1, piSeg1, piNod1);
                    }
                    else
                    {
                        for (int iPol = 2; iPol <= nbPol; iPol++)
                        {
                            int i1p1 = i1p2;
                            node11Indices = ref node12Indices;
                            nod11RValues = ref nod12RValues;
                            i1p2 = pol1[iPol];
                            PolyInternalNode pi1p2iPol =
                                piNod1[pol1[iPol]]!;
                            node12Indices = ref pi1p2iPol.Indices();
                            nod12RValues = ref pi1p2iPol.Data();
                            x1 = x2;
                            y1 = y2;
                            z1 = z2;
                            xti1 = xti2;
                            yti1 = yti2;
                            zti1 = zti2;
                            u1 = u2;
                            xti2 = x2 = nod12RValues.Point.X();
                            yti2 = y2 = nod12RValues.Point.Y();
                            zti2 = z2 = nod12RValues.Point.Z();
                            if (node12Indices.Edg1 == iEdge)
                            {
                                u2 = nod12RValues.PCu1;
                            }
                            else if (node12Indices.Edg2 == iEdge)
                            {
                                u2 = nod12RValues.PCu2;
                            }
                            node12Indices.Flag |= NMsk_Edge;
                            TIMultiply(ref xti2, ref yti2, ref zti2);
                            Interpolation(list, ref x1, ref y1, ref z1, ref x2, ref y2, ref z2, ref xti1, ref yti1,
                                ref zti1, ref xti2, ref yti2, ref zti2, iEdge, ref u1, ref u2, ref node11Indices,
                                ref nod11RValues, ref node12Indices, ref nod12RValues, i1p1, i1p2, i1, pid1,
                                ref tData1, ref piSeg1, ref piNod1);
                        }
                    }
                }
            }
            else if (nbConnex == 2)
            {
                NCollection_List<Shape>.Iterator itn = new(ls);
                Face f1 = ShapeCasts.Face(itn.Value());
                int i1 = _fMap.IndexOf(f1) + 1;
                Triangulation? tr1 = Tool.Triangulation(f1, out loc);
                hPol[0] = Tool.PolygonOnTriangulation(edge, tr1, loc);
                itn.Next();
                Face f2 = ShapeCasts.Face(itn.Value());
                int i2 = _fMap.IndexOf(f2) + 1;
                if (i1 == i2)
                {
                    edge.Reverse();
                }
                Triangulation? tr2 = Tool.Triangulation(f2, out loc);
                hPol[1] = Tool.PolygonOnTriangulation(edge, tr2, loc);
                Continuity rg = Tool.Continuity(edge, f1, f2);
                PolyInternalData pid1 = pid.Value(i1)!;
                PolyInternalData pid2 = pid.Value(i2)!;
                if (hPol[0] is not null && hPol[1] is not null)
                {
                    _pc.Initialize(edge, f1);
                    NCollection_Array1<int> pol1 = hPol[0]!.Nodes();
                    NCollection_Array1<int> pol2 = hPol[1]!.Nodes();
                    NCollection_HArray1<double> par = hPol[0]!.Parameters()!;
                    int nbPol1 = pol1.Upper();
                    NCollection_Array1<TriangleData> tData1 = pid1.TData();
                    NCollection_Array1<PolyInternalSegment> piSeg1 = pid1.PISeg();
                    NCollection_Array1<PolyInternalNode?> piNod1 = pid1.PINod();
                    NCollection_Array1<TriangleData> tData2 = pid2.TData();
                    NCollection_Array1<PolyInternalSegment> piSeg2 = pid2.PISeg();
                    NCollection_Array1<PolyInternalNode?> piNod2 = pid2.PINod();
                    PolyInternalNode pi1p1 = piNod1[pol1[1]]!;
                    ref PolyInternalNode.NodeIndices node11Indices = ref pi1p1.Indices();
                    ref PolyInternalNode.NodeData nod11RValues = ref pi1p1.Data();
                    PolyInternalNode pi1p2nbPol1 = piNod1[pol1[nbPol1]]!;
                    ref PolyInternalNode.NodeIndices node12Indices = ref pi1p2nbPol1.Indices();
                    ref PolyInternalNode.NodeData nod12RValues = ref pi1p2nbPol1.Data();
                    PolyInternalNode pi2p1 = piNod2[pol2[1]]!;
                    ref PolyInternalNode.NodeIndices nod21Indices = ref pi2p1.Indices();
                    ref PolyInternalNode.NodeData nod21RValues = ref pi2p1.Data();
                    PolyInternalNode pi2p2 = piNod2[pol2[nbPol1]]!;
                    ref PolyInternalNode.NodeIndices nod22Indices = ref pi2p2.Indices();
                    ref PolyInternalNode.NodeData nod22RValues = ref pi2p2.Data();
                    node11Indices.Flag |= NMsk_Vert;
                    node12Indices.Flag |= NMsk_Vert;
                    nod21Indices.Flag |= NMsk_Vert;
                    nod22Indices.Flag |= NMsk_Vert;

                    for (int iPol = 1; iPol <= nbPol1; iPol++)
                    {
                        PolyInternalNode pi1pA = piNod1[pol1[iPol]]!;
                        ref PolyInternalNode.NodeIndices nod1AIndices = ref pi1pA.Indices();
                        ref PolyInternalNode.NodeData nod1ARValues = ref pi1pA.Data();
                        PolyInternalNode pi2pA = piNod2[pol2[iPol]]!;
                        ref PolyInternalNode.NodeIndices nod2AIndices = ref pi2pA.Indices();
                        ref PolyInternalNode.NodeData nod2ARValues = ref pi2pA.Data();
                        double pCu = par.Value(iPol);
                        if (nod1AIndices.Edg1 == 0 || nod1AIndices.Edg1 == iEdge)
                        {
                            nod1AIndices.Edg1 = iEdge;
                            nod1ARValues.PCu1 = pCu;
                        }
                        else
                        {
                            nod1AIndices.Edg2 = iEdge;
                            nod1ARValues.PCu2 = pCu;
                        }
                        if (nod2AIndices.Edg1 == 0 || nod2AIndices.Edg1 == iEdge)
                        {
                            nod2AIndices.Edg1 = iEdge;
                            nod2ARValues.PCu1 = pCu;
                        }
                        else
                        {
                            nod2AIndices.Edg2 = iEdge;
                            nod2ARValues.PCu2 = pCu;
                        }
                    }

                    int i1p2 = pol1[1];
                    node12Indices = ref node11Indices;
                    nod12RValues = ref nod11RValues;
                    int i2p2 = pol2[1];
                    nod22Indices = ref nod21Indices;
                    nod22RValues = ref nod21RValues;
                    xti2 = x2 = nod12RValues.Point.X();
                    yti2 = y2 = nod12RValues.Point.Y();
                    zti2 = z2 = nod12RValues.Point.Z();
                    if (node12Indices.Edg1 == iEdge)
                    {
                        u2 = nod12RValues.PCu1;
                    }
                    else if (node12Indices.Edg2 == iEdge)
                    {
                        u2 = nod12RValues.PCu2;
                    }
                    node12Indices.Flag |= NMsk_Edge;
                    nod22Indices.Flag |= NMsk_Edge;
                    TIMultiply(ref xti2, ref yti2, ref zti2);
                    if (pol1[1] == pol1[nbPol1] && _pc.IsPeriodic)
                    {
                        u2 = u2 - _pc.Period();
                    }

                    if (nbPol1 == 2 && Tool.Degenerated(edge))
                    {
                        CheckDegeneratedSegment(ref node11Indices, ref nod11RValues, ref node12Indices, ref nod12RValues);
                        CheckDegeneratedSegment(ref nod21Indices, ref nod21RValues, ref nod22Indices, ref nod22RValues);
                        UpdateAroundNode(pol1[1], ref node11Indices, tData1, piSeg1, piNod1);
                        UpdateAroundNode(pol1[nbPol1], ref node12Indices, tData1, piSeg1, piNod1);
                        UpdateAroundNode(pol2[1], ref nod21Indices, tData2, piSeg2, piNod2);
                        UpdateAroundNode(pol2[nbPol1], ref nod22Indices, tData2, piSeg2, piNod2);
                    }
                    else
                    {
                        for (int iPol = 2; iPol <= nbPol1; iPol++)
                        {
                            int i1p1 = i1p2;
                            node11Indices = ref node12Indices;
                            nod11RValues = ref nod12RValues;
                            int i2p1 = i2p2;
                            nod21Indices = ref nod22Indices;
                            nod21RValues = ref nod22RValues;
                            i1p2 = pol1[iPol];
                            PolyInternalNode pi1p2iPol = piNod1[pol1[iPol]]!;
                            node12Indices = ref pi1p2iPol.Indices();
                            nod12RValues = ref pi1p2iPol.Data();
                            i2p2 = pol2[iPol];
                            PolyInternalNode pi2p2iPol = piNod2[pol2[iPol]]!;
                            nod22Indices = ref pi2p2iPol.Indices();
                            nod22RValues = ref pi2p2iPol.Data();
                            x1 = x2;
                            y1 = y2;
                            z1 = z2;
                            xti1 = xti2;
                            yti1 = yti2;
                            zti1 = zti2;
                            u1 = u2;
                            xti2 = x2 = nod12RValues.Point.X();
                            yti2 = y2 = nod12RValues.Point.Y();
                            zti2 = z2 = nod12RValues.Point.Z();
                            if (node12Indices.Edg1 == iEdge)
                            {
                                u2 = nod12RValues.PCu1;
                            }
                            else if (node12Indices.Edg2 == iEdge)
                            {
                                u2 = nod12RValues.PCu2;
                            }
                            node12Indices.Flag |= NMsk_Edge;
                            node12Indices.Flag |= NMsk_Edge;
                            TIMultiply(ref xti2, ref yti2, ref zti2);
                            Interpolation(list, ref x1, ref y1, ref z1, ref x2, ref y2, ref z2, ref xti1, ref yti1,
                                ref zti1, ref xti2, ref yti2, ref zti2, iEdge, ref u1, ref u2, rg, ref node11Indices,
                                ref nod11RValues, ref node12Indices, ref nod12RValues, i1p1, i1p2, i1, pid1,
                                ref tData1, ref piSeg1, ref piNod1, ref nod21Indices, ref nod21RValues,
                                ref nod22Indices, ref nod22RValues, i2p1, i2p2, i2, pid2, ref tData2, ref piSeg2,
                                ref piNod2);
                        }
                    }
                }
            }
        }
        else
        { // no connexity
            Polygon3D? polyg = Tool.Polygon3D(edge, out loc);
            if (polyg is not null)
            {
                NCollection_Array1<Point3D> pol = polyg.Nodes();
                Transform3D tt = loc.Transformation;
                Transform3D projTrsf = _proj.Transformation();
                tt = tt.PreMultiplied(projTrsf);
                {
                    Xyz tTrsfVec = tt.TranslationXyz();
                    _ttLo[0] = tTrsfVec.X();
                    _ttLo[1] = tTrsfVec.Y();
                    _ttLo[2] = tTrsfVec.Z();
                    Matrix3x3 tTrsfMat = tt.VectorialPart;
                    _ttMa[0, 0] = tTrsfMat.Value(1, 1);
                    _ttMa[0, 1] = tTrsfMat.Value(1, 2);
                    _ttMa[0, 2] = tTrsfMat.Value(1, 3);
                    _ttMa[1, 0] = tTrsfMat.Value(2, 1);
                    _ttMa[1, 1] = tTrsfMat.Value(2, 2);
                    _ttMa[1, 2] = tTrsfMat.Value(2, 3);
                    _ttMa[2, 0] = tTrsfMat.Value(3, 1);
                    _ttMa[2, 1] = tTrsfMat.Value(3, 2);
                    _ttMa[2, 2] = tTrsfMat.Value(3, 3);
                }
                int nbPol1 = pol.Upper();
                Xyz p1 = pol[1].XYZ();
                x2 = p1.X();
                y2 = p1.Y();
                z2 = p1.Z();
                TTMultiply(ref x2, ref y2, ref z2);
                xti2 = x2;
                yti2 = y2;
                zti2 = z2;
                TIMultiply(ref xti2, ref yti2, ref zti2);

                for (int jPol = 2; jPol <= nbPol1; jPol++)
                {
                    x1 = x2;
                    y1 = y2;
                    z1 = z2;
                    xti1 = xti2;
                    yti1 = yti2;
                    zti1 = zti2;
                    Xyz p2 = pol[jPol].XYZ();
                    x2 = p2.X();
                    y2 = p2.Y();
                    z2 = p2.Z();
                    TTMultiply(ref x2, ref y2, ref z2);
                    xti2 = x2;
                    yti2 = y2;
                    zti2 = z2;
                    TIMultiply(ref xti2, ref yti2, ref zti2);
                    list.Prepend(new AlgoBiPoint(xti1, yti1, zti1, xti2, yti2, zti2, x1, y1, z1, x2, y2, z2,
                        iEdge, 0));
                }
            }
        }
    }


    private void Interpolation(NCollection_List<AlgoBiPoint> list, ref double x1, ref double y1,
        ref double z1, ref double x2, ref double y2, ref double z2, ref double xti1, ref double yti1,
        ref double zti1, ref double xti2, ref double yti2, ref double zti2, int iEdge, ref double u1,
        ref double u2, ref PolyInternalNode.NodeIndices nod11Indices,
        ref PolyInternalNode.NodeData nod11RValues, ref PolyInternalNode.NodeIndices nod12Indices,
        ref PolyInternalNode.NodeData nod12RValues, int i1p1, int i1p2, int i1,
        PolyInternalData pid1, ref NCollection_Array1<TriangleData> tData1,
        ref NCollection_Array1<PolyInternalSegment> piSeg1,
        ref NCollection_Array1<PolyInternalNode?> piNod1)
    {
        bool mP3P1 = false;
        double x3 = 0.0, y3 = 0.0, z3 = 0.0, xti3 = 0.0, yti3 = 0.0, zti3 = 0.0, coef3 = 0.0, u3 = 0.0;
        //  gp_Pnt P3, PT3;
        bool insP3 = Interpolation(u1, u2, ref nod11RValues, ref nod12RValues, ref x3, ref y3, ref z3,
            ref xti3, ref yti3, ref zti3, ref coef3, ref u3, ref mP3P1);
        MoveOrInsertPoint(list, ref x1, ref y1, ref z1, ref x2, ref y2, ref z2, ref xti1,
            ref yti1, ref zti1, ref xti2, ref yti2, ref zti2, iEdge, ref u1, ref u2,
            ref nod11Indices, ref nod11RValues, ref nod12Indices, ref nod12RValues, i1p1, i1p2, i1,
            pid1, ref tData1, ref piSeg1, ref piNod1, x3, y3, z3, xti3, yti3, zti3, coef3, u3, insP3, mP3P1,
            0);
    }


    private void Interpolation(NCollection_List<AlgoBiPoint> list, ref double x1, ref double y1,
        ref double z1, ref double x2, ref double y2, ref double z2, ref double xti1, ref double yti1,
        ref double zti1, ref double xti2, ref double yti2, ref double zti2, int iEdge, ref double u1,
        ref double u2, Continuity rg, ref PolyInternalNode.NodeIndices nod11Indices,
        ref PolyInternalNode.NodeData nod11RValues, ref PolyInternalNode.NodeIndices nod12Indices,
        ref PolyInternalNode.NodeData nod12RValues, int i1p1, int i1p2, int i1,
        PolyInternalData pid1, ref NCollection_Array1<TriangleData> tData1,
        ref NCollection_Array1<PolyInternalSegment> piSeg1,
        ref NCollection_Array1<PolyInternalNode?> piNod1,
        ref PolyInternalNode.NodeIndices nod21Indices, ref PolyInternalNode.NodeData nod21RValues,
        ref PolyInternalNode.NodeIndices nod22Indices, ref PolyInternalNode.NodeData nod22RValues,
        int i2p1, int i2p2, int i2, PolyInternalData pid2,
        ref NCollection_Array1<TriangleData> tData2,
        ref NCollection_Array1<PolyInternalSegment> piSeg2,
        ref NCollection_Array1<PolyInternalNode?> piNod2)
    {
        bool mP3P1 = false, mP4P1 = false;
        double x3 = 0.0, y3 = 0.0, z3 = 0.0, xti3 = 0.0, yti3 = 0.0, zti3 = 0.0, coef3 = 0.0, u3 = 0.0;
        double x4 = 0.0, y4 = 0.0, z4 = 0.0, xti4 = 0.0, yti4 = 0.0, zti4 = 0.0, coef4 = 0.0, u4 = 0.0;
        //  gp_Pnt P3, PT3, P4, PT4;
        int flag = 0;
        if (rg >= G1)
        {
            flag += 1;
        }
        if (rg >= G2)
        {
            flag += 2;
        }
        bool insP3 = Interpolation(u1, u2, ref nod11RValues, ref nod12RValues, ref x3, ref y3, ref z3,
            ref xti3, ref yti3, ref zti3, ref coef3, ref u3, ref mP3P1);
        bool insP4 = Interpolation(u1, u2, ref nod21RValues, ref nod22RValues, ref x4, ref y4, ref z4,
            ref xti4, ref yti4, ref zti4, ref coef4, ref u4, ref mP4P1);
        bool isOK = insP3 || insP4;
        if (isOK)
        {
            if (!insP4) // p1 i1p3 p2
            {
                MoveOrInsertPoint(list, ref x1, ref y1, ref z1, ref x2, ref y2, ref z2, ref xti1,
                    ref yti1, ref zti1, ref xti2, ref yti2, ref zti2, iEdge, ref u1, ref u2,
                    ref nod11Indices, ref nod11RValues, ref nod12Indices, ref nod12RValues, i1p1, i1p2,
                    i1, pid1, ref tData1, ref piSeg1, ref piNod1, ref nod21Indices,
                    ref nod21RValues, ref nod22Indices, ref nod22RValues, i2p1, i2p2, i2, pid2,
                    ref tData2, ref piSeg2, ref piNod2, x3, y3, z3, xti3, yti3, zti3, coef3, u3, insP3, mP3P1,
                    flag);
            }
            else if (!insP3) // p1 i2p4 p2
            {
                MoveOrInsertPoint(list, ref x1, ref y1, ref z1, ref x2, ref y2, ref z2, ref xti1,
                    ref yti1, ref zti1, ref xti2, ref yti2, ref zti2, iEdge, ref u1, ref u2,
                    ref nod21Indices, ref nod21RValues, ref nod22Indices, ref nod22RValues, i2p1, i2p2,
                    i2, pid2, ref tData2, ref piSeg2, ref piNod2, ref nod11Indices,
                    ref nod11RValues, ref nod12Indices, ref nod12RValues, i1p1, i1p2, i1, pid1,
                    ref tData1, ref piSeg1, ref piNod1, x4, y4, z4, xti4, yti4, zti4, coef4, u4, insP4, mP4P1,
                    flag);
            }
            else if (Math.Abs(coef4 - coef3) < _tolSta) // p1 i1p3-i2p4 p2
            {
                MoveOrInsertPoint(list, ref x1, ref y1, ref z1, ref x2, ref y2, ref z2, ref xti1,
                    ref yti1, ref zti1, ref xti2, ref yti2, ref zti2, iEdge, ref u1, ref u2,
                    ref nod21Indices, ref nod21RValues, ref nod22Indices, ref nod22RValues, i2p1, i2p2,
                    i2, pid2, ref tData2, ref piSeg2, ref piNod2, ref nod11Indices,
                    ref nod11RValues, ref nod12Indices, ref nod12RValues, i1p1, i1p2, i1, pid1,
                    ref tData1, ref piSeg1, ref piNod1, x4, y4, z4, xti4, yti4, zti4, coef4, u4, insP4, mP4P1,
                    flag);
            }
            else if (coef4 < coef3) // p1 i2p4 i1p3 p2
            {
                MoveOrInsertPoint(list, ref x1, ref y1, ref z1, ref x2, ref y2, ref z2, ref xti1,
                    ref yti1, ref zti1, ref xti2, ref yti2, ref zti2, iEdge, ref u1, ref u2,
                    ref nod21Indices, ref nod21RValues, ref nod22Indices, ref nod22RValues, i2p1, i2p2,
                    i2, pid2, ref tData2, ref piSeg2, ref piNod2, ref nod11Indices,
                    ref nod11RValues, ref nod12Indices, ref nod12RValues, i1p1, i1p2, i1, pid1,
                    ref tData1, ref piSeg1, ref piNod1, x4, y4, z4, xti4, yti4, zti4, coef4, u4, insP4, mP4P1,
                    x3, y3, z3, xti3, yti3, zti3, coef3, u3, insP3, mP3P1, flag);
            }
            else // p1 i1p3 i2p4 p2
            {
                MoveOrInsertPoint(list, ref x1, ref y1, ref z1, ref x2, ref y2, ref z2, ref xti1,
                    ref yti1, ref zti1, ref xti2, ref yti2, ref zti2, iEdge, ref u1, ref u2,
                    ref nod11Indices, ref nod11RValues, ref nod12Indices, ref nod12RValues, i1p1, i1p2,
                    i1, pid1, ref tData1, ref piSeg1, ref piNod1, ref nod21Indices,
                    ref nod21RValues, ref nod22Indices, ref nod22RValues, i2p1, i2p2, i2, pid2,
                    ref tData2, ref piSeg2, ref piNod2, x3, y3, z3, xti3, yti3, zti3, coef3, u3, insP3, mP3P1,
                    x4, y4, z4, xti4, yti4, zti4, coef4, u4, insP4, mP4P1, flag);
            }
        }
        else // p1 p2
        {
            list.Prepend(new AlgoBiPoint(xti1, yti1, zti1, xti2, yti2, zti2, x1, y1,
                z1, x2, y2, z2, iEdge, i1, i1p1, i1p2, i2, i2p1, i2p2, flag));
        }
    }


    private bool Interpolation(double u1, double u2, ref PolyInternalNode.NodeData nod1RValues,
        ref PolyInternalNode.NodeData nod2RValues, ref double x3, ref double y3, ref double z3,
        ref double xti3, ref double yti3, ref double zti3, ref double coef3, ref double u3,
        ref bool themP3P1)
    {
        if (NewNode(ref nod1RValues, ref nod2RValues, ref coef3, ref themP3P1))
        {
            u3 = u1 + (u2 - u1) * coef3;
            Point3D p3 = _bCurv.Value(u3);
            xti3 = x3 = p3.X;
            yti3 = y3 = p3.Y;
            zti3 = z3 = p3.Z;
            TMultiply(ref x3, ref y3, ref z3);
            return true;
        }

        x3 = y3 = z3 = xti3 = yti3 = zti3 = coef3 = u3 = 0.0;
        return false;
    }


    private void MoveOrInsertPoint(NCollection_List<AlgoBiPoint> list, ref double x1, ref double y1,
        ref double z1, ref double x2, ref double y2, ref double z2, ref double xti1, ref double yti1,
        ref double zti1, ref double xti2, ref double yti2, ref double zti2, int iEdge, ref double u1,
        ref double u2, ref PolyInternalNode.NodeIndices nod11Indices,
        ref PolyInternalNode.NodeData nod11RValues, ref PolyInternalNode.NodeIndices nod12Indices,
        ref PolyInternalNode.NodeData nod12RValues, int i1p1, int i1p2, int i1,
        PolyInternalData pid1, ref NCollection_Array1<TriangleData> tData1,
        ref NCollection_Array1<PolyInternalSegment> piSeg1,
        ref NCollection_Array1<PolyInternalNode?> piNod1, double x3, double y3, double z3,
        double xti3, double yti3, double zti3, double coef3, double u3, bool insP3, bool themP3P1,
        int flag)
    {
        NCollection_Array1<TriangleData> tData2 = null!;
        NCollection_Array1<PolyInternalSegment> piSeg2 = null!;
        NCollection_Array1<PolyInternalNode?> piNod2 = null!;
        bool ins3 = insP3;
        if (ins3 && themP3P1) // P1 ---> P3
        {
            if (((nod11Indices.Flag & NMsk_Vert) == 0) && coef3 < _tolSta)
            {
                ins3 = false;
                ChangeNode(i1p1, i1p2, ref nod11Indices, ref nod11RValues, ref nod12Indices,
                    ref nod12RValues, coef3, x3, y3, z3, true, tData1, piSeg1, piNod1);
                x1 = x3;
                y1 = y3;
                z1 = z3;
                xti1 = xti3;
                yti1 = yti3;
                zti1 = zti3;
                u1 = u3;
                nod11RValues.Point = new Xyz(x3, y3, z3);
                if (nod11Indices.Edg1 == iEdge)
                {
                    nod11RValues.PCu1 = u3;
                }
                else if (nod11Indices.Edg2 == iEdge)
                {
                    nod11RValues.PCu2 = u3;
                }
                nod11RValues.Scal = 0;
                nod11Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(i1p1, ref nod11Indices, tData1, piSeg1, piNod1);
                ref AlgoBiPoint.PointsT points = ref list.ChangeFirst().Points();
                points.PntP2 = new Xyz(x3, y3, z3);
                points.Pnt2 = new Xyz(xti3, yti3, zti3);
            }
        }
        if (ins3 && !themP3P1) // P2 ---> P3
        {
            if (((nod12Indices.Flag & NMsk_Vert) == 0) && coef3 > _tolEnd)
            {
                ins3 = false;
                ChangeNode(i1p1, i1p2, ref nod11Indices, ref nod11RValues, ref nod12Indices,
                    ref nod12RValues, coef3, x3, y3, z3, false, tData1, piSeg1, piNod1);
                x2 = x3;
                y2 = y3;
                z2 = z3;
                xti2 = xti3;
                yti2 = yti3;
                zti2 = zti3;
                u2 = u3;
                nod12RValues.Point = new Xyz(x3, y3, z3);
                if (nod12Indices.Edg1 == iEdge)
                {
                    nod12RValues.PCu1 = u3;
                }
                else if (nod12Indices.Edg2 == iEdge)
                {
                    nod12RValues.PCu2 = u3;
                }
                nod12RValues.Scal = 0;
                nod12Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(i1p2, ref nod12Indices, tData1, piSeg1, piNod1);
            }
        }
        if (ins3) // p1 i1p3 p2
        {
            int i1p3 = pid1.AddNode(ref nod11RValues, ref nod12RValues, ref piNod1, ref piNod2, coef3,
                x3, y3, z3);
            PolyInternalNode pi1p3 = piNod1[i1p3]!;
            ref PolyInternalNode.NodeIndices nod13Indices = ref pi1p3.Indices();
            ref PolyInternalNode.NodeData nod13RValues = ref pi1p3.Data();
            nod13Indices.Edg1 = iEdge;
            nod13RValues.PCu1 = u3;
            nod13RValues.Scal = 0;
            nod13Indices.Flag |= NMsk_OutL;
            nod13Indices.Flag |= NMsk_Edge;
            pid1.UpdateLinks(i1p1, i1p2, i1p3, ref tData1, ref tData2, ref piSeg1, ref piSeg2,
                ref piNod1, ref piNod2);
            UpdateAroundNode(i1p3, ref nod13Indices, tData1, piSeg1, piNod1);
            list.Prepend(new AlgoBiPoint(xti1, yti1, zti1, xti3, yti3, zti3, x1, y1,
                z1, x3, y3, z3, iEdge, i1, i1p1, i1p3, flag));
            list.Prepend(new AlgoBiPoint(xti3, yti3, zti3, xti2, yti2, zti2, x3, y3,
                z3, x2, y2, z2, iEdge, i1, i1p3, i1p2, flag));
        }
        else // p1 p2
        {
            list.Prepend(new AlgoBiPoint(xti1, yti1, zti1, xti2, yti2, zti2, x1, y1,
                z1, x2, y2, z2, iEdge, i1, i1p1, i1p2, flag));
        }
    }


    private void MoveOrInsertPoint(NCollection_List<AlgoBiPoint> list, ref double x1, ref double y1,
        ref double z1, ref double x2, ref double y2, ref double z2, ref double xti1, ref double yti1,
        ref double zti1, ref double xti2, ref double yti2, ref double zti2, int iEdge, ref double u1,
        ref double u2, ref PolyInternalNode.NodeIndices nod11Indices,
        ref PolyInternalNode.NodeData nod11RValues, ref PolyInternalNode.NodeIndices nod12Indices,
        ref PolyInternalNode.NodeData nod12RValues, int i1p1, int i1p2, int i1,
        PolyInternalData pid1, ref NCollection_Array1<TriangleData> tData1,
        ref NCollection_Array1<PolyInternalSegment> piSeg1,
        ref NCollection_Array1<PolyInternalNode?> piNod1,
        ref PolyInternalNode.NodeIndices nod21Indices, ref PolyInternalNode.NodeData nod21RValues,
        ref PolyInternalNode.NodeIndices nod22Indices, ref PolyInternalNode.NodeData nod22RValues,
        int i2p1, int i2p2, int i2, PolyInternalData pid2,
        ref NCollection_Array1<TriangleData> tData2,
        ref NCollection_Array1<PolyInternalSegment> piSeg2,
        ref NCollection_Array1<PolyInternalNode?> piNod2, double x3, double y3, double z3,
        double xti3, double yti3, double zti3, double coef3, double u3, bool insP3, bool themP3P1,
        int flag)
    {
        bool ins3 = insP3;
        if (ins3 && themP3P1) // P1 ---> P3
        {
            if (((nod11Indices.Flag & NMsk_Vert) == 0) && coef3 < _tolSta)
            {
                ins3 = false;
                ChangeNode(i1p1, i1p2, ref nod11Indices, ref nod11RValues, ref nod12Indices,
                    ref nod12RValues, coef3, x3, y3, z3, true, tData1, piSeg1, piNod1);
                ChangeNode(i2p1, i2p2, ref nod21Indices, ref nod21RValues, ref nod22Indices,
                    ref nod22RValues, coef3, x3, y3, z3, true, tData2, piSeg2, piNod2);
                x1 = x3;
                y1 = y3;
                z1 = z3;
                xti1 = xti3;
                yti1 = yti3;
                zti1 = zti3;
                u1 = u3;
                nod11RValues.Point = new Xyz(x3, y3, z3);
                if (nod11Indices.Edg1 == iEdge)
                {
                    nod11RValues.PCu1 = u3;
                }
                else if (nod11Indices.Edg2 == iEdge)
                {
                    nod11RValues.PCu2 = u3;
                }
                nod11RValues.Scal = 0;
                nod11Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(i1p1, ref nod11Indices, tData1, piSeg1, piNod1);
                nod21RValues.Point = new Xyz(x3, y3, z3);
                if (nod21Indices.Edg1 == iEdge)
                {
                    nod21RValues.PCu1 = u3;
                }
                else if (nod21Indices.Edg2 == iEdge)
                {
                    nod21RValues.PCu2 = u3;
                }
                nod21RValues.Scal = 0;
                nod21Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(i2p1, ref nod21Indices, tData2, piSeg2, piNod2);
                ref AlgoBiPoint.PointsT points = ref list.ChangeFirst().Points();
                points.PntP2 = new Xyz(x3, y3, z3);
                points.Pnt2 = new Xyz(xti3, yti3, zti3);
            }
        }
        if (ins3 && !themP3P1) // P2 ---> P3
        {
            if (((nod12Indices.Flag & NMsk_Vert) == 0) && coef3 > _tolEnd)
            {
                ins3 = false;
                ChangeNode(i1p1, i1p2, ref nod11Indices, ref nod11RValues, ref nod12Indices,
                    ref nod12RValues, coef3, x3, y3, z3, false, tData1, piSeg1, piNod1);
                ChangeNode(i2p1, i2p2, ref nod21Indices, ref nod21RValues, ref nod22Indices,
                    ref nod22RValues, coef3, x3, y3, z3, false, tData2, piSeg2, piNod2);
                x2 = x3;
                y2 = y3;
                z2 = z3;
                xti2 = xti3;
                yti2 = yti3;
                zti2 = zti3;
                u2 = u3;
                nod12RValues.Point = new Xyz(x3, y3, z3);
                if (nod12Indices.Edg1 == iEdge)
                {
                    nod12RValues.PCu1 = u3;
                }
                else if (nod12Indices.Edg2 == iEdge)
                {
                    nod12RValues.PCu2 = u3;
                }
                nod12RValues.Scal = 0;
                nod12Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(i1p2, ref nod12Indices, tData1, piSeg1, piNod1);
                nod22RValues.Point = new Xyz(x3, y3, z3);
                if (nod22Indices.Edg1 == iEdge)
                {
                    nod22RValues.PCu1 = u3;
                }
                else if (nod22Indices.Edg2 == iEdge)
                {
                    nod22RValues.PCu2 = u3;
                }
                nod22RValues.Scal = 0;
                nod22Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(i2p2, ref nod22Indices, tData2, piSeg2, piNod2);
            }
        }
        if (ins3) // p1 i1p3 p2
        {
            int i1p3 = pid1.AddNode(ref nod11RValues, ref nod12RValues, ref piNod1, ref piNod2,
                coef3, x3, y3, z3);
            int i2p3 = pid2.AddNode(ref nod21RValues, ref nod22RValues, ref piNod2, ref piNod1,
                coef3, x3, y3, z3);
            PolyInternalNode pi1p3 = piNod1[i1p3]!;
            ref PolyInternalNode.NodeIndices nod13Indices = ref pi1p3.Indices();
            ref PolyInternalNode.NodeData nod13RValues = ref pi1p3.Data();
            PolyInternalNode pi2p3 = piNod2[i2p3]!;
            ref PolyInternalNode.NodeIndices nod23Indices = ref pi2p3.Indices();
            ref PolyInternalNode.NodeData nod23RValues = ref pi2p3.Data();
            nod13Indices.Edg1 = iEdge;
            nod13RValues.PCu1 = u3;
            nod13RValues.Scal = 0;
            nod13Indices.Flag |= NMsk_OutL;
            nod13Indices.Flag |= NMsk_Edge;
            nod23Indices.Edg1 = iEdge;
            nod23RValues.PCu1 = u3;
            nod23RValues.Scal = 0;
            nod23Indices.Flag |= NMsk_OutL;
            nod23Indices.Flag |= NMsk_Edge;
            pid1.UpdateLinks(i1p1, i1p2, i1p3, ref tData1, ref tData2, ref piSeg1, ref piSeg2,
                ref piNod1, ref piNod2);
            pid2.UpdateLinks(i2p1, i2p2, i2p3, ref tData2, ref tData1, ref piSeg2, ref piSeg1,
                ref piNod2, ref piNod1);
            UpdateAroundNode(i1p3, ref nod13Indices, tData1, piSeg1, piNod1);
            UpdateAroundNode(i2p3, ref nod23Indices, tData2, piSeg2, piNod2);
            list.Prepend(new AlgoBiPoint(xti1, yti1, zti1, xti3, yti3, zti3, x1, y1,
                z1, x3, y3, z3, iEdge, i1, i1p1, i1p3, i2, i2p1, i2p3, flag));
            list.Prepend(new AlgoBiPoint(xti3, yti3, zti3, xti2, yti2, zti2, x3, y3,
                z3, x2, y2, z2, iEdge, i1, i1p3, i1p2, i2, i2p3, i2p2, flag));
        }
        else // p1 p2
        {
            list.Prepend(new AlgoBiPoint(xti1, yti1, zti1, xti2, yti2, zti2, x1, y1,
                z1, x2, y2, z2, iEdge, i1, i1p1, i1p2, i2, i2p1, i2p2, flag));
        }
    }


    private void MoveOrInsertPoint(NCollection_List<AlgoBiPoint> list, ref double x1, ref double y1,
        ref double z1, ref double x2, ref double y2, ref double z2, ref double xti1, ref double yti1,
        ref double zti1, ref double xti2, ref double yti2, ref double zti2, int iEdge, ref double u1,
        ref double u2, ref PolyInternalNode.NodeIndices nod11Indices,
        ref PolyInternalNode.NodeData nod11RValues, ref PolyInternalNode.NodeIndices nod12Indices,
        ref PolyInternalNode.NodeData nod12RValues, int i1p1, int i1p2, int i1,
        PolyInternalData pid1, ref NCollection_Array1<TriangleData> tData1,
        ref NCollection_Array1<PolyInternalSegment> piSeg1,
        ref NCollection_Array1<PolyInternalNode?> piNod1,
        ref PolyInternalNode.NodeIndices nod21Indices, ref PolyInternalNode.NodeData nod21RValues,
        ref PolyInternalNode.NodeIndices nod22Indices, ref PolyInternalNode.NodeData nod22RValues,
        int i2p1, int i2p2, int i2, PolyInternalData pid2,
        ref NCollection_Array1<TriangleData> tData2,
        ref NCollection_Array1<PolyInternalSegment> piSeg2,
        ref NCollection_Array1<PolyInternalNode?> piNod2, double x3, double y3, double z3,
        double xti3, double yti3, double zti3, double coef3, double u3, bool insP3, bool themP3P1,
        double x4, double y4, double z4, double xti4, double yti4, double zti4, double coef4,
        double u4, bool insP4, bool themP4P1, int flag)
    {
        bool ins3 = insP3;
        bool ins4 = insP4;
        if (ins3 && themP3P1) // P1 ---> P3
        {
            if (((nod11Indices.Flag & NMsk_Vert) == 0) && coef3 < _tolSta)
            {
                ins3 = false;
                ChangeNode(i1p1, i1p2, ref nod11Indices, ref nod11RValues, ref nod12Indices,
                    ref nod12RValues, coef3, x3, y3, z3, true, tData1, piSeg1, piNod1);
                ChangeNode(i2p1, i2p2, ref nod21Indices, ref nod21RValues, ref nod22Indices,
                    ref nod22RValues, coef3, x3, y3, z3, true, tData2, piSeg2, piNod2);
                x1 = x3;
                y1 = y3;
                z1 = z3;
                xti1 = xti3;
                yti1 = yti3;
                zti1 = zti3;
                u1 = u3;
                nod11RValues.Point = new Xyz(x3, y3, z3);
                if (nod11Indices.Edg1 == iEdge)
                {
                    nod11RValues.PCu1 = u3;
                }
                else if (nod11Indices.Edg2 == iEdge)
                {
                    nod11RValues.PCu2 = u3;
                }
                nod11RValues.Scal = 0;
                nod11Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(i1p1, ref nod11Indices, tData1, piSeg1, piNod1);
                nod21RValues.Point = new Xyz(x3, y3, z3);
                if (nod21Indices.Edg1 == iEdge)
                {
                    nod21RValues.PCu1 = u3;
                }
                else if (nod21Indices.Edg2 == iEdge)
                {
                    nod21RValues.PCu2 = u3;
                }
                nod21RValues.Scal = 0;
                nod21Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(i2p1, ref nod21Indices, tData2, piSeg2, piNod2);
                ref AlgoBiPoint.PointsT points = ref list.ChangeFirst().Points();
                points.PntP2 = new Xyz(x3, y3, z3);
                points.Pnt2 = new Xyz(xti3, yti3, zti3);
            }
        }
        if (ins4 && !themP4P1) // P2 ---> P4
        {
            if (((nod12Indices.Flag & NMsk_Vert) == 0) && coef4 > _tolEnd)
            {
                ins4 = false;
                ChangeNode(i2p1, i2p2, ref nod21Indices, ref nod21RValues, ref nod22Indices,
                    ref nod22RValues, coef4, x4, y4, z4, false, tData2, piSeg2, piNod2);
                ChangeNode(i1p1, i1p2, ref nod11Indices, ref nod11RValues, ref nod12Indices,
                    ref nod12RValues, coef4, x4, y4, z4, false, tData1, piSeg1, piNod1);
                x2 = x4;
                y2 = y4;
                z2 = z4;
                xti2 = xti4;
                yti2 = yti4;
                zti2 = zti4;
                u2 = u4;
                nod12RValues.Point = new Xyz(x4, y4, z4);
                if (nod12Indices.Edg1 == iEdge)
                {
                    nod12RValues.PCu1 = u4;
                }
                else if (nod12Indices.Edg2 == iEdge)
                {
                    nod12RValues.PCu2 = u4;
                }
                nod12RValues.Scal = 0;
                nod12Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(i1p2, ref nod12Indices, tData1, piSeg1, piNod1);
                nod22RValues.Point = new Xyz(x4, y4, z4);
                if (nod22Indices.Edg1 == iEdge)
                {
                    nod22RValues.PCu1 = u4;
                }
                else if (nod22Indices.Edg2 == iEdge)
                {
                    nod22RValues.PCu2 = u4;
                }
                nod22RValues.Scal = 0;
                nod22Indices.Flag |= NMsk_OutL;
                UpdateAroundNode(i2p2, ref nod22Indices, tData2, piSeg2, piNod2);
            }
        }
        if (ins3 || ins4)
        {
            if (!ins4) // p1 i1p3 p2
            {
                MoveOrInsertPoint(list, ref x1, ref y1, ref z1, ref x2, ref y2, ref z2, ref xti1,
                    ref yti1, ref zti1, ref xti2, ref yti2, ref zti2, iEdge, ref u1, ref u2,
                    ref nod11Indices, ref nod11RValues, ref nod12Indices, ref nod12RValues, i1p1, i1p2,
                    i1, pid1, ref tData1, ref piSeg1, ref piNod1, ref nod21Indices,
                    ref nod21RValues, ref nod22Indices, ref nod22RValues, i2p1, i2p2, i2, pid2,
                    ref tData2, ref piSeg2, ref piNod2, x3, y3, z3, xti3, yti3, zti3,
                    coef3, u3, insP3, themP3P1, flag);
            }
            else if (!ins3) // p1 i2p4 p2
            {
                MoveOrInsertPoint(list, ref x1, ref y1, ref z1, ref x2, ref y2, ref z2, ref xti1,
                    ref yti1, ref zti1, ref xti2, ref yti2, ref zti2, iEdge, ref u1, ref u2,
                    ref nod21Indices, ref nod21RValues, ref nod22Indices, ref nod22RValues, i2p1, i2p2,
                    i2, pid2, ref tData2, ref piSeg2, ref piNod2, ref nod11Indices,
                    ref nod11RValues, ref nod12Indices, ref nod12RValues, i1p1, i1p2, i1, pid1,
                    ref tData1, ref piSeg1, ref piNod1, x4, y4, z4, xti4, yti4, zti4,
                    coef4, u4, insP4, themP4P1, flag);
            }
            else // p1 i1p3 i2p4 p2
            {
                int i1p3 = pid1.AddNode(ref nod11RValues, ref nod12RValues, ref piNod1, ref piNod2,
                    coef3, x3, y3, z3);
                int i2p3 = pid2.AddNode(ref nod21RValues, ref nod22RValues, ref piNod2, ref piNod1,
                    coef3, x3, y3, z3);
                int i1p4 = pid1.AddNode(ref nod11RValues, ref nod12RValues, ref piNod1, ref piNod2,
                    coef4, x4, y4, z4);
                int i2p4 = pid2.AddNode(ref nod21RValues, ref nod22RValues, ref piNod2, ref piNod1,
                    coef4, x4, y4, z4);
                PolyInternalNode pi1p3 = piNod1[i1p3]!;
                ref PolyInternalNode.NodeIndices nod13Indices = ref pi1p3.Indices();
                ref PolyInternalNode.NodeData nod13RValues = ref pi1p3.Data();
                PolyInternalNode pi1p4 = piNod1[i1p4]!;
                ref PolyInternalNode.NodeIndices nod14Indices = ref pi1p4.Indices();
                ref PolyInternalNode.NodeData nod14RValues = ref pi1p4.Data();
                PolyInternalNode pi2p3 = piNod2[i2p3]!;
                ref PolyInternalNode.NodeIndices nod23Indices = ref pi2p3.Indices();
                ref PolyInternalNode.NodeData nod23RValues = ref pi2p3.Data();
                PolyInternalNode pi2p4 = piNod2[i2p4]!;
                ref PolyInternalNode.NodeIndices nod24Indices = ref pi2p4.Indices();
                ref PolyInternalNode.NodeData nod24RValues = ref pi2p4.Data();
                nod13Indices.Edg1 = iEdge;
                nod13RValues.PCu1 = u3;
                nod13RValues.Scal = 0;
                nod13Indices.Flag |= NMsk_OutL;
                nod13Indices.Flag |= NMsk_Edge;
                nod23Indices.Edg1 = iEdge;
                nod23RValues.PCu1 = u3;
                nod23RValues.Scal = 0;
                nod23Indices.Flag |= NMsk_OutL;
                nod23Indices.Flag |= NMsk_Edge;
                nod14Indices.Edg1 = iEdge;
                nod14RValues.PCu1 = u4;
                nod14RValues.Scal = 0;
                nod14Indices.Flag |= NMsk_OutL;
                nod14Indices.Flag |= NMsk_Edge;
                nod24Indices.Edg1 = iEdge;
                nod24RValues.PCu1 = u4;
                nod24RValues.Scal = 0;
                nod24Indices.Flag |= NMsk_OutL;
                nod24Indices.Flag |= NMsk_Edge;
                pid1.UpdateLinks(i1p1, i1p2, i1p3, ref tData1, ref tData2, ref piSeg1, ref piSeg2,
                    ref piNod1, ref piNod2);
                pid2.UpdateLinks(i2p1, i2p2, i2p3, ref tData2, ref tData1, ref piSeg2, ref piSeg1,
                    ref piNod2, ref piNod1);
                pid2.UpdateLinks(i2p3, i2p2, i2p4, ref tData2, ref tData1, ref piSeg2, ref piSeg1,
                    ref piNod2, ref piNod1);
                pid1.UpdateLinks(i1p3, i1p2, i1p4, ref tData1, ref tData2, ref piSeg1, ref piSeg2,
                    ref piNod1, ref piNod2);
                UpdateAroundNode(i1p3, ref nod13Indices, tData1, piSeg1, piNod1);
                UpdateAroundNode(i2p3, ref nod23Indices, tData2, piSeg2, piNod2);
                UpdateAroundNode(i1p4, ref nod14Indices, tData1, piSeg1, piNod1);
                UpdateAroundNode(i2p4, ref nod24Indices, tData2, piSeg2, piNod2);
                list.Prepend(new AlgoBiPoint(xti1, yti1, zti1, xti3, yti3, zti3, x1, y1,
                    z1, x3, y3, z3, iEdge, i1, i1p1, i1p3, i2, i2p1, i2p3, flag));
                list.Prepend(new AlgoBiPoint(xti3, yti3, zti3, xti4, yti4, zti4, x3, y3,
                    z3, x4, y4, z4, iEdge, i1, i1p3, i1p4, i2, i2p3, i2p4, flag));
                list.Prepend(new AlgoBiPoint(xti4, yti4, zti4, xti2, yti2, zti2, x4, y4,
                    z4, x2, y2, z2, iEdge, i1, i1p4, i1p2, i2, i2p4, i2p2, flag));
            }
        }
        else // p1 p2
        {
            list.Prepend(new AlgoBiPoint(xti1, yti1, zti1, xti2, yti2, zti2, x1, y1,
                z1, x2, y2, z2, iEdge, i1, i1p1, i1p2, i2, i2p1, i2p2, flag));
        }
    }


    private void InsertOnOutLine(NCollection_Array1<PolyInternalData?> pid)
    {
        NCollection_Array1<TriangleData> tData2 = null!;
        NCollection_Array1<PolyInternalSegment> piSeg2 = null!;
        NCollection_Array1<PolyInternalNode?> piNod2 = null!;

        Placement loc;
        bool mP3P1 = false;
        double u3 = 0.0, v3 = 0.0, coef3 = 0.0, x3 = 0.0, y3 = 0.0, z3 = 0.0;

        Transform3D projTrsf = _proj.Transformation();

        int nbFaces = _fMap.Count;
        for (int faceIter = 1; faceIter <= nbFaces; ++faceIter)
        {
            PolyInternalData aPid = pid[faceIter]!;
            if (aPid is null)
            {
                continue;
            }

            bool isIntOutL = false;
            NCollection_Array1<TriangleData> tData1 = aPid.TData();
            NCollection_Array1<PolyInternalSegment> piSeg1 = aPid.PISeg();
            NCollection_Array1<PolyInternalNode?> piNod1 = aPid.PINod();
            Shape localShape = _fMap[faceIter - 1];
            Face face = ShapeCasts.Face(localShape);
            _bSurf.Initialize(face, false);
            _gSurf = Tool.Surface(face, out loc);
            {
                Transform3D tt = loc.Transformation;
                tt = tt.PreMultiplied(projTrsf);
                Xyz tTrsfVec = tt.TranslationXyz();
                _ttLo[0] = tTrsfVec.X();
                _ttLo[1] = tTrsfVec.Y();
                _ttLo[2] = tTrsfVec.Z();
                Matrix3x3 tTrsfMat = tt.VectorialPart;
                _ttMa[0, 0] = tTrsfMat.Value(1, 1);
                _ttMa[0, 1] = tTrsfMat.Value(1, 2);
                _ttMa[0, 2] = tTrsfMat.Value(1, 3);
                _ttMa[1, 0] = tTrsfMat.Value(2, 1);
                _ttMa[1, 1] = tTrsfMat.Value(2, 2);
                _ttMa[1, 2] = tTrsfMat.Value(2, 3);
                _ttMa[2, 0] = tTrsfMat.Value(3, 1);
                _ttMa[2, 1] = tTrsfMat.Value(3, 2);
                _ttMa[2, 2] = tTrsfMat.Value(3, 3);
            }


            int nbSegs = aPid.PISegCount;
            for (int segIter = 1; segIter <= nbSegs; ++segIter)
            {
                ref PolyInternalSegment segIndices = ref piSeg1[segIter];
                //	bool Cutted = false;
                if (segIndices.Conex1 != 0 && segIndices.Conex2 != 0)
                {
                    int ip1 = segIndices.LstSg1;
                    int ip2 = segIndices.LstSg2;
                    PolyInternalNode pip1 = piNod1[ip1]!;
                    ref PolyInternalNode.NodeIndices nod1Indices = ref pip1.Indices();
                    ref PolyInternalNode.NodeData nod1RValues = ref pip1.Data();
                    PolyInternalNode pip2 = piNod1[ip2]!;
                    ref PolyInternalNode.NodeIndices nod2Indices = ref pip2.Indices();
                    ref PolyInternalNode.NodeData nod2RValues = ref pip2.Data();
                    if ((nod1Indices.Flag & NMsk_OutL) != 0 && (nod2Indices.Flag & NMsk_OutL) != 0)
                    {
                        isIntOutL = true;
                    }
                    else if ((nod1RValues.Scal >= _tolAngular && nod2RValues.Scal <= -_tolAngular) || (nod2RValues.Scal >= _tolAngular && nod1RValues.Scal <= -_tolAngular))
                    {
                        isIntOutL = true;
                        bool isInsP3 = NewNode(ref nod1RValues, ref nod2RValues, ref coef3, ref mP3P1);
                        if (isInsP3)
                        {
                            UVNode(ref nod1RValues, ref nod2RValues, coef3, ref u3, ref v3);
                            if (_gSurf is not null)
                            {
                                Point3D pt3 = _gSurf.Value(u3, v3);
                                x3 = pt3.X;
                                y3 = pt3.Y;
                                z3 = pt3.Z;
                            }
                            else
                            {
                                // simple averaging - this could be improved
                                double coef2 = 1.0 - coef3;
                                Point3D pt3 = new Point3D(coef2 * nod1RValues.Point + coef3 * nod2RValues.Point);
                                x3 = pt3.X;
                                y3 = pt3.Y;
                                z3 = pt3.Z;
                            }
                            TTMultiply(ref x3, ref y3, ref z3);
                        }

                        if (isInsP3 && mP3P1) // P1 ---> P3
                        {
                            if ((nod1Indices.Flag & NMsk_Edge) == 0 && coef3 < _tolSta)
                            {
                                isInsP3 = false;
                                ChangeNode(ip1, ip2, ref nod1Indices, ref nod1RValues, ref nod2Indices,
                                    ref nod2RValues, coef3, x3, y3, z3, true, tData1, piSeg1, piNod1);
                                nod1RValues.Scal = 0;
                                nod1Indices.Flag |= NMsk_OutL;
                            }
                        }
                        if (isInsP3 && !mP3P1) // P2 ---> P3
                        {
                            if ((nod2Indices.Flag & NMsk_Edge) == 0 && coef3 > _tolEnd)
                            {
                                isInsP3 = false;
                                ChangeNode(ip1, ip2, ref nod1Indices, ref nod1RValues, ref nod2Indices,
                                    ref nod2RValues, coef3, x3, y3, z3, false, tData1, piSeg1, piNod1);
                                nod2RValues.Scal = 0;
                                nod2Indices.Flag |= NMsk_OutL;
                            }
                        }
                        if (isInsP3) // p1 ip3 p2
                        {
                            int ip3 =
                                aPid.AddNode(ref nod1RValues, ref nod2RValues, ref piNod1, ref piNod2, coef3, x3, y3, z3);
                            PolyInternalNode pip3 = piNod1[ip3]!;
                            ref PolyInternalNode.NodeIndices nod3Indices = ref pip3.Indices();
                            ref PolyInternalNode.NodeData nod3RValues = ref pip3.Data();
                            aPid.UpdateLinks(ip1, ip2, ip3, ref tData1, ref tData2, ref piSeg1, ref piSeg2, ref piNod1, ref piNod2);
                            UpdateAroundNode(ip3, ref nod3Indices, tData1, piSeg1, piNod1);
                            nod3RValues.Scal = 0;
                            nod3Indices.Flag |= NMsk_OutL;
                        }
                    }
                }
            }
            if (isIntOutL)
            {
                aPid.IntOutL(true);
            }

        }
    }


    private void CheckFrBackTriangles(NCollection_List<AlgoBiPoint> list,
        NCollection_Array1<PolyInternalData?> thePID)
    {
        double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0, x3 = 0.0, y3 = 0.0;
        double d1, d2, d3;
        double dd, dX, dY, nX, nY;

        NCollection_Array1<TriangleData> tData1 = null!;
        NCollection_Array1<PolyInternalSegment> piSeg1 = null!;
        NCollection_Array1<PolyInternalNode?> piNod1 = null!;
        NCollection_Array1<TriangleData> tData2 = null!;
        NCollection_Array1<PolyInternalSegment> piSeg2 = null!;
        NCollection_Array1<PolyInternalNode?> piNod2 = null!;

        int nbFaces = _fMap.Count;
        bool isModif = true;
        int iLoop = 0;

        while (isModif && iLoop < 4)
        {
            iLoop++;
            isModif = false;
            bool isFrBackInList = false;
            for (int faceIter = 1; faceIter <= nbFaces; ++faceIter)
            {
                PolyInternalData pid = thePID[faceIter]!;
                if (pid is null)
                {
                    continue;
                }

                int nbTris = pid.TDataCount;
                NCollection_Array1<TriangleData> tData = pid.TData();
                NCollection_Array1<PolyInternalSegment> piSeg = pid.PISeg();
                NCollection_Array1<PolyInternalNode?> piNod = pid.PINod();
                for (int triIter = 1; triIter <= nbTris; ++triIter)
                {
                    ref TriangleData tdata = ref tData[triIter];
                    if ((tdata.Flags & HLRAlgo_PolyMask_FMskSide) == 0 && ((tdata.Flags & HLRAlgo_PolyMask_FMskFrBack) != 0))
                    {
                        isModif = true;
                        PolyInternalNode pi1p1 = piNod[tdata.Node1]!;
                        ref PolyInternalNode.NodeIndices nod11Indices = ref pi1p1.Indices();
                        ref PolyInternalNode.NodeData nod11RValues = ref pi1p1.Data();
                        PolyInternalNode pi1p2 = piNod[tdata.Node2]!;
                        ref PolyInternalNode.NodeIndices nod12Indices = ref pi1p2.Indices();
                        ref PolyInternalNode.NodeData nod12RValues = ref pi1p2.Data();
                        PolyInternalNode pi1p3 = piNod[tdata.Node3]!;
                        ref PolyInternalNode.NodeIndices nod13Indices = ref pi1p3.Indices();
                        ref PolyInternalNode.NodeData nod13RValues = ref pi1p3.Data();
                        d1 = 0.0;
                        d2 = 0.0;
                        d3 = 0.0;
                        if (((nod11Indices.Flag & NMsk_Edge) == 0 || iLoop > 1) && ((nod11Indices.Flag & NMsk_OutL) == 0 || iLoop > 1) && ((nod11Indices.Flag & NMsk_Vert) == 0))
                        {
                            dX = nod13RValues.Point.X() - nod12RValues.Point.X();
                            dY = nod13RValues.Point.Y() - nod12RValues.Point.Y();
                            d1 = dX * dX + dY * dY;
                            d1 = Math.Sqrt(d1);
                            nX = -dY / d1;
                            nY = dX / d1;
                            dX = nod11RValues.Point.X() - nod12RValues.Point.X();
                            dY = nod11RValues.Point.Y() - nod12RValues.Point.Y();
                            dd = -(dX * nX + dY * nY);
                            if (dd < 0)
                            {
                                dd -= d1 * 0.01;
                            }
                            else
                            {
                                dd += d1 * 0.01;
                            }
                            x1 = nX * dd;
                            y1 = nY * dd;
                        }
                        if (((nod12Indices.Flag & NMsk_Edge) == 0 || iLoop > 1) && ((nod12Indices.Flag & NMsk_OutL) == 0 || iLoop > 1) && ((nod12Indices.Flag & NMsk_Vert) == 0))
                        {
                            dX = nod11RValues.Point.X() - nod13RValues.Point.X();
                            dY = nod11RValues.Point.Y() - nod13RValues.Point.Y();
                            d2 = dX * dX + dY * dY;
                            d2 = Math.Sqrt(d2);
                            nX = -dY / d2;
                            nY = dX / d2;
                            dX = nod12RValues.Point.X() - nod13RValues.Point.X();
                            dY = nod12RValues.Point.Y() - nod13RValues.Point.Y();
                            dd = -(dX * nX + dY * nY);
                            if (dd < 0)
                            {
                                dd -= d2 * 0.01;
                            }
                            else
                            {
                                dd += d2 * 0.01;
                            }
                            x2 = nX * dd;
                            y2 = nY * dd;
                        }
                        if (((nod13Indices.Flag & NMsk_Edge) == 0 || iLoop > 1) && ((nod13Indices.Flag & NMsk_OutL) == 0 || iLoop > 1) && ((nod13Indices.Flag & NMsk_Vert) == 0))
                        {
                            dX = nod12RValues.Point.X() - nod11RValues.Point.X();
                            dY = nod12RValues.Point.Y() - nod11RValues.Point.Y();
                            d3 = dX * dX + dY * dY;
                            d3 = Math.Sqrt(d3);
                            nX = -dY / d3;
                            nY = dX / d3;
                            dX = nod13RValues.Point.X() - nod11RValues.Point.X();
                            dY = nod13RValues.Point.Y() - nod11RValues.Point.Y();
                            dd = -(dX * nX + dY * nY);
                            if (dd < 0)
                            {
                                dd -= d3 * 0.01;
                            }
                            else
                            {
                                dd += d3 * 0.01;
                            }
                            x3 = nX * dd;
                            y3 = nY * dd;
                        }
                        if (d1 > d2 && d1 > d3)
                        {
                            nod11RValues.Point.ChangeCoord(1) += x1;
                            nod11RValues.Point.ChangeCoord(2) += y1;
                            nod11Indices.Flag |= NMsk_Move;
                            UpdateAroundNode(tdata.Node1, ref nod11Indices, tData, piSeg, piNod);
                            isFrBackInList = true;
                        }
                        else if (d2 > d3 && d2 > d1)
                        {
                            nod12RValues.Point.ChangeCoord(1) += x2;
                            nod12RValues.Point.ChangeCoord(2) += y2;
                            nod12Indices.Flag |= NMsk_Move;
                            UpdateAroundNode(tdata.Node2, ref nod12Indices, tData, piSeg, piNod);
                            isFrBackInList = true;
                        }
                        else if (d3 > d1 && d3 > d2)
                        {
                            nod13RValues.Point.ChangeCoord(1) += x3;
                            nod13RValues.Point.ChangeCoord(2) += y3;
                            nod13Indices.Flag |= NMsk_Move;
                            UpdateAroundNode(tdata.Node3, ref nod13Indices, tData, piSeg, piNod);
                            isFrBackInList = true;
                        }
                    }
                }
            }
            if (isFrBackInList)
            {
                for (NCollection_List<AlgoBiPoint>.Iterator bPointIter = new(list); bPointIter.More(); bPointIter.Next())
                {
                    ref AlgoBiPoint bp = ref bPointIter.ChangeValue();
                    ref AlgoBiPoint.IndicesT theIndices = ref bp.Indices();
                    if (theIndices.FaceConex1 != 0)
                    {
                        PolyInternalData pid1 = thePID.Value(theIndices.FaceConex1)!;
                        tData1 = pid1.TData();
                        piSeg1 = pid1.PISeg();
                        piNod1 = pid1.PINod();
                    }
                    if (theIndices.FaceConex2 != 0)
                    {
                        if (theIndices.FaceConex1 == theIndices.FaceConex2)
                        {
                            tData2 = tData1;
                            piSeg2 = piSeg1;
                            piNod2 = piNod1;
                        }
                        else
                        {
                            PolyInternalData pid2 = thePID.Value(theIndices.FaceConex2)!;
                            tData2 = pid2.TData();
                            piSeg2 = pid2.PISeg();
                            piNod2 = pid2.PINod();
                        }
                    }
                    if (theIndices.FaceConex1 != 0)
                    {
                        ref PolyInternalNode.NodeIndices nod11Indices = ref piNod1[theIndices.Face1Pt1]!.Indices();
                        if ((nod11Indices.Flag & NMsk_Move) != 0)
                        {
                            ref PolyInternalNode.NodeData nod11RValues = ref
                                piNod1.Value(theIndices.Face1Pt1)!.Data();
                            ref AlgoBiPoint.PointsT points = ref bp.Points();
                            points.Pnt1 = points.PntP1 = nod11RValues.Point;
                            TIMultiply(ref points.Pnt1);
                            if (theIndices.FaceConex2 != 0)
                            {
                                ref PolyInternalNode.NodeIndices nod12Indices = ref
                                    piNod2[theIndices.Face2Pt1]!.Indices();
                                ref PolyInternalNode.NodeData nod12RValues = ref
                                    piNod2[theIndices.Face2Pt1]!.Data();
                                nod12RValues.Point.ChangeCoord(1) = nod11RValues.Point.X();
                                nod12RValues.Point.ChangeCoord(2) = nod11RValues.Point.Y();
                                UpdateAroundNode(theIndices.Face2Pt1, ref nod12Indices, tData2, piSeg2, piNod2);
                            }
                        }
                        nod11Indices = ref piNod1[theIndices.Face1Pt2]!.Indices();
                        if ((nod11Indices.Flag & NMsk_Move) != 0)
                        {
                            ref PolyInternalNode.NodeData nod11RValues = ref
                                piNod1.Value(theIndices.Face1Pt2)!.Data();
                            ref AlgoBiPoint.PointsT points = ref bp.Points();
                            points.Pnt2 = points.PntP2 = nod11RValues.Point;
                            TIMultiply(ref points.Pnt2);
                            if (theIndices.FaceConex2 != 0)
                            {
                                ref PolyInternalNode.NodeIndices nod12Indices = ref
                                    piNod2[theIndices.Face2Pt2]!.Indices();
                                ref PolyInternalNode.NodeData nod12RValues = ref
                                    piNod2[theIndices.Face2Pt2]!.Data();
                                nod12RValues.Point.ChangeCoord(1) = nod11RValues.Point.X();
                                nod12RValues.Point.ChangeCoord(2) = nod11RValues.Point.Y();
                                UpdateAroundNode(theIndices.Face2Pt2, ref nod12Indices, tData2, piSeg2, piNod2);
                            }
                        }
                    }
                    if (theIndices.FaceConex2 != 0)
                    {
                        PolyInternalData pid2 = thePID.Value(theIndices.FaceConex2)!;
                        piNod2 = pid2.PINod();
                        ref PolyInternalNode.NodeIndices nod11Indices = ref piNod2[theIndices.Face2Pt1]!.Indices();
                        if ((nod11Indices.Flag & NMsk_Move) != 0)
                        {
                            ref PolyInternalNode.NodeData nod11RValues = ref
                                piNod2.Value(theIndices.Face2Pt1)!.Data();
                            ref AlgoBiPoint.PointsT points = ref bp.Points();
                            points.Pnt1 = points.PntP1 = nod11RValues.Point;
                            TIMultiply(ref points.Pnt1);
                            if (theIndices.FaceConex1 != 0)
                            {
                                ref PolyInternalNode.NodeIndices nod12Indices = ref
                                    piNod1[theIndices.Face1Pt1]!.Indices();
                                ref PolyInternalNode.NodeData nod12RValues = ref
                                    piNod1[theIndices.Face1Pt1]!.Data();
                                nod12RValues.Point.ChangeCoord(1) = nod11RValues.Point.X();
                                nod12RValues.Point.ChangeCoord(2) = nod11RValues.Point.Y();
                                UpdateAroundNode(theIndices.Face1Pt1, ref nod12Indices, tData1, piSeg1, piNod1);
                            }
                        }
                        nod11Indices = ref piNod2[theIndices.Face2Pt2]!.Indices();
                        if ((nod11Indices.Flag & NMsk_Move) != 0)
                        {
                            ref PolyInternalNode.NodeData nod11RValues = ref piNod2.Value(theIndices.Face2Pt2)!.Data();
                            ref AlgoBiPoint.PointsT points = ref bp.Points();
                            points.Pnt2 = points.PntP2 = nod11RValues.Point;
                            TIMultiply(ref points.Pnt2);
                            if (theIndices.FaceConex1 != 0)
                            {
                                ref PolyInternalNode.NodeIndices nod12Indices = ref
                                    piNod1[theIndices.Face1Pt2]!.Indices();
                                ref PolyInternalNode.NodeData nod12RValues = ref
                                    piNod1[theIndices.Face1Pt2]!.Data();
                                nod12RValues.Point.ChangeCoord(1) = nod11RValues.Point.X();
                                nod12RValues.Point.ChangeCoord(2) = nod11RValues.Point.Y();
                                UpdateAroundNode(theIndices.Face1Pt2, ref nod12Indices, tData1, piSeg1, piNod1);
                            }
                        }
                    }
                }

                for (int faceIter = 1; faceIter <= nbFaces; ++faceIter)
                {
                    PolyInternalData aPid = thePID[faceIter]!;
                    if (aPid is not null)
                    {
                        int nbNodes = aPid.PINodCount;
                        NCollection_Array1<PolyInternalNode?> piNod = aPid.PINod();
                        for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
                        {
                            ref PolyInternalNode.NodeIndices nod11Indices = ref
                                piNod[nodeIter]!.Indices();
                            nod11Indices.Flag &= ~NMsk_Move;
                        }
                    }
                }
            }
        }
    }


    private void FindEdgeOnTriangle(in TriangleData triangle, int ip1, int ip2, ref int jtrouv,
        ref bool isDirect)
    {
        int n1 = triangle.Node1;
        int n2 = triangle.Node2;
        int n3 = triangle.Node3;
        if (ip1 == n1 && ip2 == n2)
        {
            jtrouv = 0;
            isDirect = true;
            return;
        }
        else if (ip2 == n1 && ip1 == n2)
        {
            jtrouv = 0;
            isDirect = false;
            return;
        }
        else if (ip1 == n2 && ip2 == n3)
        {
            jtrouv = 1;
            isDirect = true;
            return;
        }
        else if (ip2 == n2 && ip1 == n3)
        {
            jtrouv = 1;
            isDirect = false;
            return;
        }
        else if (ip1 == n3 && ip2 == n1)
        {
            jtrouv = 2;
            isDirect = true;
            return;
        }
        else if (ip2 == n3 && ip1 == n1)
        {
            jtrouv = 2;
            isDirect = false;
            return;
        }
    }


    private void ChangeNode(int ip1, int ip2, ref PolyInternalNode.NodeIndices nod1Indices,
        ref PolyInternalNode.NodeData nod1RValues, ref PolyInternalNode.NodeIndices nod2Indices,
        ref PolyInternalNode.NodeData nod2RValues, double coef1, double x3, double y3, double z3,
        bool isFirst, NCollection_Array1<TriangleData> tData,
        NCollection_Array1<PolyInternalSegment> piSeg,
        NCollection_Array1<PolyInternalNode?> piNod)
    {
        double coef2 = 1.0 - coef1;
        if (isFirst)
        {
            nod1RValues.Point = new Xyz(x3, y3, z3);
            nod1RValues.UV = coef2 * nod1RValues.UV + coef1 * nod2RValues.UV;
            nod1RValues.Scal = nod1RValues.Scal * coef2 + nod2RValues.Scal * coef1;
            Xyz xyz = coef2 * nod1RValues.Normal + coef1 * nod2RValues.Normal;
            double norm = xyz.Modulus();
            if (norm > 0)
            {
                nod1RValues.Normal = (1 / norm) * xyz;
            }
            else
            {
                nod1RValues.Normal = new Xyz(1.0, 0.0, 0.0);
            }
            UpdateAroundNode(ip1, ref nod1Indices, tData, piSeg, piNod);
        }
        else
        {
            nod2RValues.Point = new Xyz(x3, y3, z3);
            nod2RValues.UV = coef2 * nod1RValues.UV + coef1 * nod2RValues.UV;
            nod2RValues.Scal = nod1RValues.Scal * coef2 + nod2RValues.Scal * coef1;
            Xyz xyz = coef2 * nod1RValues.Normal + coef1 * nod2RValues.Normal;
            double norm = xyz.Modulus();
            if (norm > 0)
            {
                nod2RValues.Normal = (1 / norm) * xyz;
            }
            else
            {
                nod2RValues.Normal = new Xyz(1.0, 0.0, 0.0);
            }
            UpdateAroundNode(ip2, ref nod2Indices, tData, piSeg, piNod);
        }
    }


    private void UpdateAroundNode(int iNode, ref PolyInternalNode.NodeIndices nod1Indices,
        NCollection_Array1<TriangleData> tData, NCollection_Array1<PolyInternalSegment> piSeg,
        NCollection_Array1<PolyInternalNode?> piNod)
    {
        int iiii = nod1Indices.NdSg;
        while (iiii != 0)
        {
            ref PolyInternalSegment segIndices = ref piSeg[iiii];
            int iTri1 = segIndices.Conex1;
            int iTri2 = segIndices.Conex2;
            if (iTri1 != 0)
            {
                ref TriangleData triangle = ref tData[iTri1];
                PolyInternalNode pn1 = piNod[triangle.Node1]!;
                PolyInternalNode pn2 = piNod[triangle.Node2]!;
                PolyInternalNode pn3 = piNod[triangle.Node3]!;
                ref PolyInternalNode.NodeIndices aNod1Indices = ref pn1.Indices();
                ref PolyInternalNode.NodeIndices nod2Indices = ref pn2.Indices();
                ref PolyInternalNode.NodeIndices nod3Indices = ref pn3.Indices();
                ref PolyInternalNode.NodeData nod1RValues = ref pn1.Data();
                ref PolyInternalNode.NodeData nod2RValues = ref pn2.Data();
                ref PolyInternalNode.NodeData nod3RValues = ref pn3.Data();
                OrientTriangle(iTri1, ref triangle, ref aNod1Indices, ref nod1RValues, ref nod2Indices,
                    ref nod2RValues, ref nod3Indices, ref nod3RValues);
            }
            if (iTri2 != 0)
            {
                ref TriangleData triangle2 = ref tData[iTri2];
                PolyInternalNode pn1 = piNod[triangle2.Node1]!;
                PolyInternalNode pn2 = piNod[triangle2.Node2]!;
                PolyInternalNode pn3 = piNod[triangle2.Node3]!;
                ref PolyInternalNode.NodeIndices aNod1Indices = ref pn1.Indices();
                ref PolyInternalNode.NodeIndices nod2Indices = ref pn2.Indices();
                ref PolyInternalNode.NodeIndices nod3Indices = ref pn3.Indices();
                ref PolyInternalNode.NodeData nod1RValues = ref pn1.Data();
                ref PolyInternalNode.NodeData nod2RValues = ref pn2.Data();
                ref PolyInternalNode.NodeData nod3RValues = ref pn3.Data();
                OrientTriangle(iTri2, ref triangle2, ref aNod1Indices, ref nod1RValues, ref nod2Indices,
                    ref nod2RValues, ref nod3Indices, ref nod3RValues);
            }
            if (segIndices.LstSg1 == iNode)
            {
                iiii = segIndices.NxtSg1;
            }
            else
            {
                iiii = segIndices.NxtSg2;
            }
        }
    }


    private void OrientTriangle(int iTri, ref TriangleData triangle,
        ref PolyInternalNode.NodeIndices nod1Indices, ref PolyInternalNode.NodeData nod1RValues,
        ref PolyInternalNode.NodeIndices nod2Indices, ref PolyInternalNode.NodeData nod2RValues,
        ref PolyInternalNode.NodeIndices nod3Indices, ref PolyInternalNode.NodeData nod3RValues)
    {
        bool o1 = (nod1Indices.Flag & NMsk_OutL) != 0;
        bool o2 = (nod2Indices.Flag & NMsk_OutL) != 0;
        bool o3 = (nod3Indices.Flag & NMsk_OutL) != 0;
        triangle.Flags &= ~HLRAlgo_PolyMask_FMskFlat;
        triangle.Flags &= ~HLRAlgo_PolyMask_FMskOnOutL;
        if (o1 && o2 && o3)
        {
            triangle.Flags |= HLRAlgo_PolyMask_FMskSide;
            triangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
            triangle.Flags |= HLRAlgo_PolyMask_FMskOnOutL;
            _ = iTri;
        }
        else
        {
            double s1 = nod1RValues.Scal;
            double s2 = nod2RValues.Scal;
            double s3 = nod3RValues.Scal;
            double as1 = s1;
            double as2 = s2;
            double as3 = s3;
            if (s1 < 0)
            {
                as1 = -s1;
            }
            if (s2 < 0)
            {
                as2 = -s2;
            }
            if (s3 < 0)
            {
                as3 = -s3;
            }
            double s = 0;
            double as_ = 0;
            if (!o1)
            {
                s = s1;
                as_ = as1;
            }
            if (!o2 && as_ < as2)
            {
                s = s2;
                as_ = as2;
            }
            if (!o3 && as_ < as3)
            {
                s = s3;
                as_ = as3;
            }
            if (s > 0)
            {
                triangle.Flags &= ~HLRAlgo_PolyMask_FMskSide;
                triangle.Flags |= HLRAlgo_PolyMask_FMskBack;
            }
            else
            {
                triangle.Flags &= ~HLRAlgo_PolyMask_FMskSide;
                triangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
            }
            Xyz d12 = nod2RValues.Point - nod1RValues.Point;
            double d12Norm = d12.Modulus();
            if (d12Norm <= 1.0e-10)
            {
                triangle.Flags |= HLRAlgo_PolyMask_FMskFlat;
                triangle.Flags |= HLRAlgo_PolyMask_FMskSide;
                triangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
            }
            else
            {
                Xyz d23 = nod3RValues.Point - nod2RValues.Point;
                double d23Norm = d23.Modulus();
                if (d23Norm < 1.0e-10)
                {
                    triangle.Flags |= HLRAlgo_PolyMask_FMskFlat;
                    triangle.Flags |= HLRAlgo_PolyMask_FMskSide;
                    triangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
                }
                else
                {
                    Xyz d31 = nod1RValues.Point - nod3RValues.Point;
                    double d31Norm = d31.Modulus();
                    if (d31Norm < 1.0e-10)
                    {
                        triangle.Flags |= HLRAlgo_PolyMask_FMskFlat;
                        triangle.Flags |= HLRAlgo_PolyMask_FMskSide;
                        triangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
                    }
                    else
                    {
                        d12 *= 1 / d12Norm;
                        d23 *= 1 / d23Norm;
                        Xyz d = d12 ^ d23;
                        double dNorm = d.Modulus();
                        if (dNorm < 1.0e-5)
                        {
                            triangle.Flags |= HLRAlgo_PolyMask_FMskFlat;
                            triangle.Flags |= HLRAlgo_PolyMask_FMskSide;
                            triangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
                        }
                        else
                        {
                            double o;
                            if (_proj.Perspective())
                            {
                                d *= 1 / dNorm;
                                o = d.Z() * _proj.Focus() - d * nod1RValues.Point;
                            }
                            else
                            {
                                o = d.Z() / dNorm;
                            }
                            if (o < 0)
                            {
                                triangle.Flags |= HLRAlgo_PolyMask_FMskOrBack;
                                o = -o;
                            }
                            else
                            {
                                triangle.Flags &= ~HLRAlgo_PolyMask_FMskOrBack;
                            }
                            if (o < 1.0e-10)
                            {
                                triangle.Flags |= HLRAlgo_PolyMask_FMskSide;
                                triangle.Flags &= ~HLRAlgo_PolyMask_FMskBack;
                            }
                        }
                    }
                }
            }
        }

        if ((((triangle.Flags & HLRAlgo_PolyMask_FMskBack) == 0) && ((triangle.Flags & HLRAlgo_PolyMask_FMskOrBack) != 0)) || (((triangle.Flags & HLRAlgo_PolyMask_FMskBack) != 0) && ((triangle.Flags & HLRAlgo_PolyMask_FMskOrBack) == 0)))
        {
            triangle.Flags |= HLRAlgo_PolyMask_FMskFrBack;
        }
        else
        {
            triangle.Flags &= ~HLRAlgo_PolyMask_FMskFrBack;
        }
    }


    private bool Triangles(int ip1, int ip2, ref PolyInternalNode.NodeIndices nod1Indices,
        ref NCollection_Array1<PolyInternalSegment> piSeg, ref int iTri1, ref int iTri2)
    {
        int iiii = nod1Indices.NdSg;
        while (iiii != 0)
        {
            ref PolyInternalSegment segIndices = ref piSeg[iiii];
            if (segIndices.LstSg1 == ip1)
            {
                if (segIndices.LstSg2 == ip2)
                {
                    iTri1 = segIndices.Conex1;
                    iTri2 = segIndices.Conex2;
                    return true;
                }
                else
                {
                    iiii = segIndices.NxtSg1;
                }
            }
            else
            {
                if (segIndices.LstSg1 == ip2)
                {
                    iTri1 = segIndices.Conex1;
                    iTri2 = segIndices.Conex2;
                    return true;
                }
                else
                {
                    iiii = segIndices.NxtSg2;
                }
            }
        }
        iTri1 = 0;
        iTri2 = 0;
        return false;
    }


    private bool NewNode(ref PolyInternalNode.NodeData nod1RValues,
        ref PolyInternalNode.NodeData nod2RValues, ref double coef1, ref bool toMoveP1)
    {
        double tolAng = _tolAngular * 0.5;
        if ((nod1RValues.Scal >= tolAng && nod2RValues.Scal <= -tolAng) || (nod2RValues.Scal >= tolAng && nod1RValues.Scal <= -tolAng))
        {
            coef1 = nod1RValues.Scal / (nod2RValues.Scal - nod1RValues.Scal);
            if (coef1 < 0)
            {
                coef1 = -coef1;
            }
            toMoveP1 = coef1 < 0.5;
            return true;
        }
        return false;
    }


    private void UVNode(ref PolyInternalNode.NodeData nod1RValues,
        ref PolyInternalNode.NodeData nod2RValues, double coef1, ref double u3, ref double v3)
    {
        double coef2 = 1.0 - coef1;
        Xy uv3 = coef2 * nod1RValues.UV + coef1 * nod2RValues.UV;
        u3 = uv3.X();
        v3 = uv3.Y();
    }


    private void CheckDegeneratedSegment(ref PolyInternalNode.NodeIndices nod1Indices,
        ref PolyInternalNode.NodeData nod1RValues, ref PolyInternalNode.NodeIndices nod2Indices,
        ref PolyInternalNode.NodeData nod2RValues)
    {
        nod1Indices.Flag |= NMsk_Fuck;
        nod2Indices.Flag |= NMsk_Fuck;
        if ((nod1RValues.Scal >= _tolAngular && nod2RValues.Scal <= -_tolAngular) || (nod2RValues.Scal >= _tolAngular && nod1RValues.Scal <= -_tolAngular))
        {
            nod1RValues.Scal = 0.0;
            nod1Indices.Flag |= NMsk_OutL;
            nod2RValues.Scal = 0.0;
            nod2Indices.Flag |= NMsk_OutL;
        }
    }


    private void UpdateOutLines(NCollection_List<AlgoBiPoint> list,
        NCollection_Array1<PolyInternalData?> pid)
    {
        double x1, y1, z1, x2, y2, z2;
        double xti1, yti1, zti1, xti2, yti2, zti2;
        int nbFaces = _fMap.Count;
        for (int faceIter = 1; faceIter <= nbFaces; ++faceIter)
        {
            PolyInternalData aPid = pid[faceIter]!;
            if (aPid is null)
            {
                continue;
            }

            if (!aPid.IntOutL())
            {
                continue;
            }

            NCollection_Array1<TriangleData> tData = aPid.TData();
            NCollection_Array1<PolyInternalSegment> piSeg = aPid.PISeg();
            NCollection_Array1<PolyInternalNode?> piNod = aPid.PINod();
            int j, it1, it2, tn1, tn2, tn3, pd, pf;
            bool isOutl = false;
            int nbSegs = aPid.PISegCount;
            for (int segIter = 1; segIter <= nbSegs; ++segIter)
            {
                ref PolyInternalSegment psg = ref piSeg[segIter];
                it1 = psg.Conex1;
                it2 = psg.Conex2;
                if (it1 != 0 && it2 != 0 && it1 != it2) // debile but sure !
                {
                    ref TriangleData triangle = ref tData[it1];
                    ref TriangleData triangle2 = ref tData[it2];
                    if (((triangle.Flags & HLRAlgo_PolyMask_FMskSide) == 0) && ((triangle2.Flags & HLRAlgo_PolyMask_FMskSide) == 0))
                    {
                        isOutl = ((triangle.Flags & HLRAlgo_PolyMask_FMskBack) != 0)
                                          != ((triangle2.Flags & HLRAlgo_PolyMask_FMskBack) != 0);
                    }
                    else if (((triangle.Flags & HLRAlgo_PolyMask_FMskSide) != 0) && ((triangle2.Flags & HLRAlgo_PolyMask_FMskSide) != 0))
                    {
                        isOutl = false;
                    }
                    else if ((triangle.Flags & HLRAlgo_PolyMask_FMskSide) != 0)
                    {
                        isOutl = ((triangle.Flags & HLRAlgo_PolyMask_FMskFlat) == 0)
                                          && ((triangle2.Flags & HLRAlgo_PolyMask_FMskBack) == 0);
                    }
                    else
                    {
                        isOutl = ((triangle2.Flags & HLRAlgo_PolyMask_FMskFlat) == 0)
                                          && ((triangle.Flags & HLRAlgo_PolyMask_FMskBack) == 0);
                    }

                    if (isOutl)
                    {
                        pd = psg.LstSg1;
                        pf = psg.LstSg2;
                        tn1 = triangle.Node1;
                        tn2 = triangle.Node2;
                        tn3 = triangle.Node3;
                        if (((triangle.Flags & HLRAlgo_PolyMask_FMskSide) == 0) && ((triangle.Flags & HLRAlgo_PolyMask_FMskOrBack) != 0))
                        {
                            j = tn1;
                            tn1 = tn3;
                            tn3 = j;
                        }
                        if ((tn1 == pd && tn2 == pf) || (tn1 == pf && tn2 == pd))
                        {
                            triangle.Flags |= HLRAlgo_PolyMask_EMskOutLin1;
                        }
                        else if ((tn2 == pd && tn3 == pf) || (tn2 == pf && tn3 == pd))
                        {
                            triangle.Flags |= HLRAlgo_PolyMask_EMskOutLin2;
                        }
                        else if ((tn3 == pd && tn1 == pf) || (tn3 == pf && tn1 == pd))
                        {
                            triangle.Flags |= HLRAlgo_PolyMask_EMskOutLin3;
                        }
                        tn1 = triangle2.Node1;
                        tn2 = triangle2.Node2;
                        tn3 = triangle2.Node3;
                        if (((triangle2.Flags & HLRAlgo_PolyMask_FMskSide) == 0) && ((triangle2.Flags & HLRAlgo_PolyMask_FMskOrBack) != 0))
                        {
                            j = tn1;
                            tn1 = tn3;
                            tn3 = j;
                        }
                        if ((tn1 == pd && tn2 == pf) || (tn1 == pf && tn2 == pd))
                        {
                            triangle2.Flags |= HLRAlgo_PolyMask_EMskOutLin1;
                        }
                        else if ((tn2 == pd && tn3 == pf) || (tn2 == pf && tn3 == pd))
                        {
                            triangle2.Flags |= HLRAlgo_PolyMask_EMskOutLin2;
                        }
                        else if ((tn3 == pd && tn1 == pf) || (tn3 == pf && tn1 == pd))
                        {
                            triangle2.Flags |= HLRAlgo_PolyMask_EMskOutLin3;
                        }
                        ref PolyInternalNode.NodeData nod1RValues = ref piNod.Value(pd)!.Data();
                        ref PolyInternalNode.NodeData nod2RValues = ref piNod.Value(pf)!.Data();
                        xti1 = x1 = nod1RValues.Point.X();
                        yti1 = y1 = nod1RValues.Point.Y();
                        zti1 = z1 = nod1RValues.Point.Z();
                        xti2 = x2 = nod2RValues.Point.X();
                        yti2 = y2 = nod2RValues.Point.Y();
                        zti2 = z2 = nod2RValues.Point.Z();
                        TIMultiply(ref xti1, ref yti1, ref zti1);
                        TIMultiply(ref xti2, ref yti2, ref zti2);
                        list.Append(new AlgoBiPoint(xti1, yti1, zti1, xti2, yti2, zti2, x1, y1, z1, x2, y2, z2,
                            faceIter, faceIter, pd, pf, faceIter, pd, pf, 12));
                    }
                }
            }
        }
    }


    private void UpdateEdgesBiPoints(NCollection_List<AlgoBiPoint> list,
        NCollection_Array1<PolyInternalData?> pid, bool isClosed)
    {
        int itri1 = 0, itri2 = 0, tbid = 0;
        for (NCollection_List<AlgoBiPoint>.Iterator bPntIter = new(list); bPntIter.More(); bPntIter.Next())
        {
            ref AlgoBiPoint bp = ref bPntIter.ChangeValue();
            ref AlgoBiPoint.IndicesT indices = ref bp.Indices();
            if (indices.FaceConex1 != 0 && indices.FaceConex2 != 0)
            {
                PolyInternalData pid1 = pid.Value(indices.FaceConex1)!;
                PolyInternalData pid2 = pid.Value(indices.FaceConex2)!;
                NCollection_Array1<PolyInternalSegment> piSeg1 = pid1.PISeg();
                NCollection_Array1<PolyInternalSegment> piSeg2 = pid2.PISeg();
                ref PolyInternalNode.NodeIndices nod11Indices = ref
                    pid1.PINod()[indices.Face1Pt1]!.Indices();
                ref PolyInternalNode.NodeIndices nod21Indices = ref
                    pid2.PINod()[indices.Face2Pt1]!.Indices();
                Triangles(indices.Face1Pt1, indices.Face1Pt2, ref nod11Indices, ref piSeg1, ref itri1, ref tbid);
                Triangles(indices.Face2Pt1, indices.Face2Pt2, ref nod21Indices, ref piSeg2, ref itri2, ref tbid);

                if (itri1 != 0 && itri2 != 0)
                {
                    if (indices.FaceConex1 != indices.FaceConex2 || itri1 != itri2)
                    {
                        NCollection_Array1<TriangleData> tData1 = pid1.TData();
                        NCollection_Array1<TriangleData> tData2 = pid2.TData();
                        ref TriangleData triangle = ref tData1[itri1];
                        ref TriangleData triangle2 = ref tData2[itri2];
                        if (isClosed)
                        {
                            if ((((triangle.Flags & HLRAlgo_PolyMask_FMskBack) != 0) && ((triangle2.Flags & HLRAlgo_PolyMask_FMskBack) != 0)) || (((triangle.Flags & HLRAlgo_PolyMask_FMskSide) != 0) && ((triangle2.Flags & HLRAlgo_PolyMask_FMskSide) != 0)) || (((triangle.Flags & HLRAlgo_PolyMask_FMskBack) != 0) && ((triangle2.Flags & HLRAlgo_PolyMask_FMskSide) != 0)) || (((triangle.Flags & HLRAlgo_PolyMask_FMskSide) != 0) && ((triangle2.Flags & HLRAlgo_PolyMask_FMskBack) != 0)))
                            {
                                bp.Hidden(true);
                            }
                        }

                        bool isOutl;
                        if (((triangle.Flags & HLRAlgo_PolyMask_FMskSide) == 0) && ((triangle2.Flags & HLRAlgo_PolyMask_FMskSide) == 0))
                        {
                            isOutl = ((triangle.Flags & HLRAlgo_PolyMask_FMskBack) != 0)
                                              != ((triangle2.Flags & HLRAlgo_PolyMask_FMskBack) != 0);
                        }
                        else if (((triangle.Flags & HLRAlgo_PolyMask_FMskSide) != 0) && ((triangle2.Flags & HLRAlgo_PolyMask_FMskSide) != 0))
                        {
                            isOutl = false;
                        }
                        else if (((triangle.Flags & HLRAlgo_PolyMask_FMskSide) != 0))
                        {
                            isOutl = ((triangle.Flags & HLRAlgo_PolyMask_FMskFlat) == 0)
                                              && ((triangle2.Flags & HLRAlgo_PolyMask_FMskBack) == 0);
                        }
                        else
                        {
                            isOutl = ((triangle2.Flags & HLRAlgo_PolyMask_FMskFlat) == 0)
                                              && ((triangle.Flags & HLRAlgo_PolyMask_FMskBack) == 0);
                        }
                        bp.OutLine(isOutl);
                    }
                }
            }
        }
    }


    private void UpdatePolyData(NCollection_Array1<PolyData?> pd,
        NCollection_Array1<PolyInternalData?> pid, bool isClosed)
    {
        int nbFaces = _fMap.Count;
        for (int faceIter = 1; faceIter <= nbFaces; ++faceIter)
        {
            PolyInternalData aPid = pid[faceIter]!;
            if (aPid is null)
            {
                continue;
            }

            int nbNodes = aPid.PINodCount;
            int nbTris = aPid.TDataCount;
            NCollection_HArray1<Xyz> hNodes = new NCollection_HArray1<Xyz>(1, nbNodes);
            NCollection_HArray1<TriangleData> htData =
                new NCollection_HArray1<TriangleData>(1, nbTris);
            NCollection_Array1<Xyz> nodes = hNodes.ChangeArray1();
            NCollection_Array1<TriangleData> trian = htData.ChangeArray1();
            NCollection_Array1<TriangleData> tData = aPid.TData();
            NCollection_Array1<PolyInternalNode?> piNod = aPid.PINod();
            int nbHidden = 0;

            for (int nodeIter = 1; nodeIter <= nbNodes; ++nodeIter)
            {
                ref PolyInternalNode.NodeData nod1RValues = ref piNod.Value(nodeIter)!.Data();
                nodes.SetValue(nodeIter, nod1RValues.Point);
            }

            for (int triIter = 1; triIter <= nbTris; ++triIter)
            {
                ref TriangleData ot = ref tData[triIter];
                ref TriangleData nt = ref trian[triIter];
                if (((ot.Flags & HLRAlgo_PolyMask_FMskSide) == 0))
                {
                    if ((ot.Flags & HLRAlgo_PolyMask_FMskOrBack) != 0)
                    {
                        int j = ot.Node1;
                        ot.Node1 = ot.Node3;
                        ot.Node3 = j;
                        ot.Flags |= HLRAlgo_PolyMask_FMskBack;
                    }
                    else
                    {
                        ot.Flags &= ~HLRAlgo_PolyMask_FMskBack;
                        // Tri1Flags |= HLRAlgo_PolyMask_FMskBack;//OCC349
                    }
                }
                nt.Node1 = ot.Node1;
                nt.Node2 = ot.Node2;
                nt.Node3 = ot.Node3;
                nt.Flags = ot.Flags;
                if (((nt.Flags & HLRAlgo_PolyMask_FMskSide) == 0) && (((nt.Flags & HLRAlgo_PolyMask_FMskBack) == 0) || !isClosed))
                {
                    nt.Flags |= HLRAlgo_PolyMask_FMskHiding;
                    ++nbHidden;
                }
                else
                {
                    nt.Flags &= ~HLRAlgo_PolyMask_FMskHiding;
                }
            }

            NCollection_HArray1<PolyHidingData>? hphDat = null;
            if (nbHidden > 0)
            {
                hphDat = new NCollection_HArray1<PolyHidingData>(1, nbHidden);
            }

            PolyData aPd = pd[faceIter]!;
            aPd.HNodes(hNodes);
            aPd.HTData(htData);
            aPd.HPHDat(hphDat);
            aPd.FaceIndex(faceIter);
        }
    }


    private void TMultiply(ref double x, ref double y, ref double z, bool vpo = false)
    {
        double xt = _tMat[0, 0] * x + _tMat[0, 1] * y + _tMat[0, 2] * z + (vpo ? 0 : _tLoc[0]);//OCC349
        double yt = _tMat[1, 0] * x + _tMat[1, 1] * y + _tMat[1, 2] * z + (vpo ? 0 : _tLoc[1]);//OCC349
        z = _tMat[2, 0] * x + _tMat[2, 1] * y + _tMat[2, 2] * z + (vpo ? 0 : _tLoc[2]);//OCC349
        x = xt;
        y = yt;
    }


    private void TTMultiply(ref double x, ref double y, ref double z, bool vpo = false)
    {
        double xt = _ttMa[0, 0] * x + _ttMa[0, 1] * y + _ttMa[0, 2] * z + (vpo ? 0 : _ttLo[0]);//OCC349
        double yt = _ttMa[1, 0] * x + _ttMa[1, 1] * y + _ttMa[1, 2] * z + (vpo ? 0 : _ttLo[1]);//OCC349
        z = _ttMa[2, 0] * x + _ttMa[2, 1] * y + _ttMa[2, 2] * z + (vpo ? 0 : _ttLo[2]);//OCC349
        x = xt;
        y = yt;
    }


    private void TIMultiply(ref double x, ref double y, ref double z, bool vpo = false)
    {
        double xt = _tiMa[0, 0] * x + _tiMa[0, 1] * y + _tiMa[0, 2] * z + (vpo ? 0 : _tiLo[0]);//OCC349
        double yt = _tiMa[1, 0] * x + _tiMa[1, 1] * y + _tiMa[1, 2] * z + (vpo ? 0 : _tiLo[1]);//OCC349
        z = _tiMa[2, 0] * x + _tiMa[2, 1] * y + _tiMa[2, 2] * z + (vpo ? 0 : _tiLo[2]);//OCC349
        x = xt;
        y = yt;
    }


    /// <summary>
    /// The shape theShape in a compound with the internal outlines found on its faces' meshes (3D segments), or
    /// theShape alone before <see cref="Update"/>.
    /// </summary>
    public Shape OutLinedShape(in Shape shape)
    {
        if (shape.IsNull)
        {
            return new Shape();
        }

        Compound result = new();
        BrepBuilder builder = new();
        builder.MakeCompound(ref result);
        builder.Add(result, shape);
        if (_fMap.Count == 0)
        {
            result.TShape()?.FreezeGraph();
            return result;
        }

        HashSet<Shape> map = new(ShapeHasher.Instance);
        {
            Explorer shapeExp = new();
            for (shapeExp.Init(shape, ShapeType.Edge); shapeExp.More(); shapeExp.Next())
            {
                map.Add(shapeExp.Current());
            }
            for (shapeExp.Init(shape, ShapeType.Face); shapeExp.More(); shapeExp.Next())
            {
                map.Add(shapeExp.Current());
            }
        }

        NCollection_Array1<PolyShellData?> shell = _algo.PolyShell();
        int nbShells = shell.Upper();
        NCollection_List<AlgoBiPoint>.Iterator bPntIter = new();
        for (int shellIter = 1; shellIter <= nbShells; ++shellIter)
        {
            NCollection_List<AlgoBiPoint> list = shell.Value(shellIter)!.Edges();
            for (bPntIter.Initialize(list); bPntIter.More(); bPntIter.Next())
            {
                ref AlgoBiPoint bp = ref bPntIter.ChangeValue();
                if (bp.IntLine())
                {
                    ref AlgoBiPoint.IndicesT indices = ref bp.Indices();
                    if (map.Contains(_fMap[indices.ShapeIndex - 1]))
                    {
                        ref AlgoBiPoint.PointsT points = ref bp.Points();
                        builder.Add(result, new LibraryMakeEdge(new Point3D(points.Pnt1), new Point3D(points.Pnt2)).InternalEdge());
                    }
                }
            }
        }

        result.TShape()?.FreezeGraph();
        return result;
    }

    private void TMultiply(ref Xyz point, bool vecPartOnly = false)
    {
        TMultiply(ref point.ChangeCoord(1), ref point.ChangeCoord(2), ref point.ChangeCoord(3), vecPartOnly);
    }

    private void TTMultiply(ref Xyz point, bool vecPartOnly = false)
    {
        TTMultiply(ref point.ChangeCoord(1), ref point.ChangeCoord(2), ref point.ChangeCoord(3), vecPartOnly);
    }

    private void TIMultiply(ref Xyz point, bool vecPartOnly = false)
    {
        TIMultiply(ref point.ChangeCoord(1), ref point.ChangeCoord(2), ref point.ChangeCoord(3), vecPartOnly);
    }

    private bool AverageNormal(int iNode, ref PolyInternalNode.NodeIndices nodeIndices,
                               NCollection_Array1<TriangleData> tData,
                               NCollection_Array1<PolyInternalSegment> piSeg,
                               NCollection_Array1<PolyInternalNode?> piNod, ref Xyz normal)
    {
        return AverageNormal(iNode, ref nodeIndices, tData, piSeg, piNod, ref normal.ChangeCoord(1),
                             ref normal.ChangeCoord(2), ref normal.ChangeCoord(3));
    }
}
