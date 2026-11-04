// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_InternalAlgo.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_InternalAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.HiddenLines.Topology;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The exact hidden-line removal: loads outliners (shapes), builds their data structure for a projector, and hides
/// their edges by their own faces and by each other's.
/// </summary>
/// <remarks>
/// A working object (a transient), as OCCT's. The outliners write in working overlays (unit 10H6), so the
/// algorithm never writes into the caller's shapes. <see cref="Debug(bool)"/> turns on OCCT's traces, written to
/// standard output.
/// </remarks>
internal class InternalAlgo : Transient
{
    private HiddenLineData? _ds;
    private AlgorithmProjector _proj = new();
    private readonly NCollection_Sequence<ShapeBounds> _shapes = new();
    private readonly Dictionary<Shape, AdaptorTool> _mapOfShapeTool =
        new(ShapeHasher.Instance);
    private bool _debug;

    // HLRBRep_InternalAlgo.cxx's file statics, constants in OCCT's code (never changed)
    private const bool HLRBRep_InternalAlgo_TRACE = true;
    private const bool HLRBRep_InternalAlgo_TRACE10 = true;

    // The function-scope static of HideSelected's trace (only with myDebug, which OCCT's trace reaches only when
    // HLRBRep_InternalAlgo_TRACE10 is false: never)
    [ThreadStatic]
    private static int s_rty;

    public InternalAlgo()
    {
        _debug = false;
    }

    /// <summary>
    /// OCCT's copy: shares A's data structure, copies its projector and its shapes' bounds, but not its map of face
    /// adaptors.
    /// </summary>
    public InternalAlgo(InternalAlgo a)
    {
        _ds = a.DataStructure();
        _proj = a.Projector();
        NCollection_Sequence<ShapeBounds> shapes = a.SeqOfShapeBounds();
        for (int i = 1; i <= shapes.Length(); i++)
        {
            _shapes.Append(new ShapeBounds(shapes[i]));
        }
        _debug = a.Debug();
    }

    /// <summary>Set the projector.</summary>
    public void Projector(in AlgorithmProjector p)
    {
        _proj = p;
    }

    /// <summary>The projector (a reference to the stored one, as OCCT's).</summary>
    public ref AlgorithmProjector Projector() => ref _proj;

    /// <summary>Update the DataStructure.</summary>
    public void Update()
    {
        if (!_shapes.IsEmpty())
        {
            int n = _shapes.Length();
            HiddenLineData[] ds = new HiddenLineData[n];

            int i, dv, de, df, nv = 0, ne = 0, nf = 0;

            for (i = 1; i <= n; i++)
            {
                ShapeBounds sb = _shapes[i];
                try
                {
                    ds[i - 1] = ShapeToHlr.Load(sb.Shape()!, _proj, _mapOfShapeTool, sb.OfIsoCount);
                    dv = ds[i - 1].VertexCount;
                    de = ds[i - 1].EdgeCount;
                    df = ds[i - 1].FaceCount;
                }
                catch (Exception exception) when (IsCaught(exception))
                {
                    if (_debug)
                    {
                        Console.Write("An exception was caught when preparing the Shape " + i);
                        Console.Write(" and computing its OutLines " + '\n');
                        Console.Write(exception.Message + '\n');
                    }
                    ds[i - 1] = new HiddenLineData(0, 0, 0);
                    dv = 0;
                    de = 0;
                    df = 0;
                }

                _shapes[i] = new ShapeBounds(sb.Shape()!, sb.ShapeData(), sb.OfIsoCount, 1, dv, 1, de, 1, df);
                nv += dv;
                ne += de;
                nf += df;
            }

            if (n == 1)
            {
                _ds = ds[0];
            }
            else
            {
                _ds = new HiddenLineData(nv, ne, nf);
                nv = 0;
                ne = 0;
                nf = 0;

                for (i = 1; i <= n; i++)
                {
                    ShapeBounds sb = _shapes[i];
                    sb.Sizes(out dv, out de, out df);
                    sb.Translate(nv, ne, nf);
                    _ds.Write(ds[i - 1], nv, ne, nf);
                    nv += dv;
                    ne += de;
                    nf += df;
                }
            }

            _ds.Update(_proj);

            // OCCT leaves these unset (a shape without edges then adds its faces' boxes to garbage); zeros here
            EdgesBlock.MinMaxIndices shapMin = new(), shapMax = new(), minMaxShap = new();
            EdgesBlock.MinMaxIndices theMin = new(), theMax = new();
            NCollection_Array1<HiddenLineEdgeData> eDataArray = _ds.EDataArray();
            NCollection_Array1<HiddenLineFaceData> fDataArray = _ds.FDataArray();

            for (i = 1; i <= n; i++)
            {
                bool firstTime = true;
                ShapeBounds sb = _shapes[i];
                int v1, v2, e1, e2, f1, f2;
                sb.Bounds(out v1, out v2, out e1, out e2, out f1, out f2);

                for (int e = e1; e <= e2; e++)
                {
                    HiddenLineEdgeData ed = eDataArray[e];
                    HlrAlgo.DecodeMinMax(ed.MinMax(), ref theMin, ref theMax);
                    if (firstTime)
                    {
                        firstTime = false;
                        HlrAlgo.CopyMinMax(ref theMin, ref theMax, ref shapMin, ref shapMax);
                    }
                    else
                    {
                        HlrAlgo.AddMinMax(ref theMin, ref theMax, ref shapMin, ref shapMax);
                    }
                }

                for (int f = f1; f <= f2; f++)
                {
                    HiddenLineFaceData fd = fDataArray[f];
                    HlrAlgo.DecodeMinMax(fd.Wires()!.MinMax(), ref theMin, ref theMax);
                    HlrAlgo.AddMinMax(ref theMin, ref theMax, ref shapMin, ref shapMax);
                }
                HlrAlgo.EncodeMinMax(ref shapMin, ref shapMax, ref minMaxShap);
                sb.UpdateMinMax(minMaxShap);
            }
        }
    }

    // The exceptions OCCT's catch (Standard_Failure) handles under OCC_CATCH_SIGNALS
    private static bool IsCaught(Exception exception)
        => exception is FailureException
               or NullReferenceException
               or InvalidCastException
               or IndexOutOfRangeException
               or ArgumentOutOfRangeException
               or DivideByZeroException;

    /// <summary>Add the shape S, with the shape data SData and nbIso isolines.</summary>
    public void Load(OutLiner s, Transient? sData, int nbIso = 0)
    {
        _shapes.Append(new ShapeBounds(s, sData, nbIso, 0, 0, 0, 0, 0, 0));
        _ds = null;
    }

    /// <summary>Add the shape S, with nbIso isolines.</summary>
    public void Load(OutLiner s, int nbIso = 0)
    {
        _shapes.Append(new ShapeBounds(s, nbIso, 0, 0, 0, 0, 0, 0));
        _ds = null;
    }

    /// <summary>Return the index of the Shape S and return 0 if the Shape S is not found.</summary>
    public int Index(OutLiner s)
    {
        int n = _shapes.Length();

        for (int i = 1; i <= n; i++)
        {
            if (_shapes[i].Shape() == s)
            {
                return i;
            }
        }

        return 0;
    }

    /// <summary>Remove the Shape of Index I.</summary>
    public void Remove(int i)
    {
        Standard_OutOfRange_Raise_if(i == 0 || i > _shapes.Length(), "HLRBRep_InternalAlgo::Remove : unknown Shape");
        _shapes.Remove(i);

        _mapOfShapeTool.Clear();
        _ds = null;
    }

    /// <summary>Change the Shape Data of the Shape of index I.</summary>
    public void ShapeData(int i, Transient? sData)
    {
        Standard_OutOfRange_Raise_if(i == 0 || i > _shapes.Length(), "HLRBRep_InternalAlgo::ShapeData : unknown Shape");

        _shapes[i].ShapeData(sData);
    }

    /// <summary>The sequence of the shapes' bounds (the stored one, as OCCT's reference).</summary>
    public NCollection_Sequence<ShapeBounds> SeqOfShapeBounds() => _shapes;

    public int ShapeCount => _shapes.Length();

    public ShapeBounds ShapeBounds(int i)
    {
        Standard_OutOfRange_Raise_if(i == 0 || i > _shapes.Length(), "HLRBRep_InternalAlgo::ShapeBounds : unknown Shape");

        return _shapes[i];
    }

    /// <summary>Init the status of the selected edges depending of the back faces of a closed shell.</summary>
    public void InitEdgeStatus()
    {
        bool visible;
        FaceIterator faceIt = new();

        NCollection_Array1<HiddenLineEdgeData> eDataArray = _ds!.EDataArray();
        NCollection_Array1<HiddenLineFaceData> fDataArray = _ds.FDataArray();
        int ne = _ds.EdgeCount;
        int nf = _ds.FaceCount;

        for (int e = 1; e <= ne; e++)
        {
            HiddenLineEdgeData ed = eDataArray[e];
            if (ed.Selected())
            {
                ed.Status().ShowAll();
            }
        }
        //  for (int f = 1; f <= nf; f++) {
        int f;
        for (f = 1; f <= nf; f++)
        {
            HiddenLineFaceData fd = fDataArray[f];
            if (fd.Selected())
            {
                for (faceIt.InitEdge(fd); faceIt.MoreEdge(); faceIt.NextEdge())
                {
                    HiddenLineEdgeData edf = _ds.EDataArray()[faceIt.Edge()];
                    if (edf.Selected())
                    {
                        edf.Status().HideAll();
                    }
                }
            }
        }

        for (f = 1; f <= nf; f++)
        {
            HiddenLineFaceData fd = fDataArray[f];
            visible = true;
            if (fd.Selected() && fd.Closed())
            {
                if (fd.Side())
                {
                    visible = false;
                }
                else if (!fd.WithOutL())
                {
                    switch (fd.Orientation())
                    {
                        case Reversed:
                            visible = fd.Back();
                            break;
                        case Forward:
                            visible = !fd.Back();
                            break;
                        case External:
                        case Internal:
                            visible = true;
                            break;
                    }
                }
            }
            if (visible)
            {
                for (faceIt.InitEdge(fd); faceIt.MoreEdge(); faceIt.NextEdge())
                {
                    int E = faceIt.Edge();
                    HiddenLineEdgeData edf = _ds.EDataArray()[E];
                    if (edf.Selected() && !edf.Vertical())
                    {
                        edf.Status().ShowAll();
                    }
                }
            }
        }
    }

    /// <summary>Select all the DataStructure.</summary>
    public void Select()
    {
        if (_ds is not null)
        {
            NCollection_Array1<HiddenLineEdgeData> eDataArray = _ds.EDataArray();
            NCollection_Array1<HiddenLineFaceData> fDataArray = _ds.FDataArray();
            int ne = _ds.EdgeCount;
            int nf = _ds.FaceCount;

            for (int e = 1; e <= ne; e++)
            {
                HiddenLineEdgeData ed = eDataArray[e];
                ed.Selected(true);
            }

            for (int f = 1; f <= nf; f++)
            {
                HiddenLineFaceData fd = fDataArray[f];
                fd.Selected(true);
            }
        }
    }

    /// <summary>Select only the Shape of index I.</summary>
    public void Select(int i)
    {
        if (_ds is not null)
        {
            Standard_OutOfRange_Raise_if(i == 0 || i > _shapes.Length(), "HLRBRep_InternalAlgo::Select : unknown Shape");

            int v1, v2, e1, e2, f1, f2;
            _shapes[i].Bounds(out v1, out v2, out e1, out e2, out f1, out f2);

            NCollection_Array1<HiddenLineEdgeData> eDataArray = _ds.EDataArray();
            NCollection_Array1<HiddenLineFaceData> fDataArray = _ds.FDataArray();
            int ne = _ds.EdgeCount;
            int nf = _ds.FaceCount;

            for (int e = 1; e <= ne; e++)
            {
                HiddenLineEdgeData ed = eDataArray[e];
                ed.Selected(e >= e1 && e <= e2);
            }

            for (int f = 1; f <= nf; f++)
            {
                HiddenLineFaceData fd = fDataArray[f];
                fd.Selected(f >= f1 && f <= f2);
            }
        }
    }

    /// <summary>Select only the edges of the Shape I.</summary>
    public void SelectEdge(int i)
    {
        if (_ds is not null)
        {
            Standard_OutOfRange_Raise_if(i == 0 || i > _shapes.Length(), "HLRBRep_InternalAlgo::SelectEdge : unknown Shape");

            int v1, v2, e1, e2, f1, f2;
            _shapes[i].Bounds(out v1, out v2, out e1, out e2, out f1, out f2);

            NCollection_Array1<HiddenLineEdgeData> eDataArray = _ds.EDataArray();
            int ne = _ds.EdgeCount;

            for (int e = 1; e <= ne; e++)
            {
                HiddenLineEdgeData ed = eDataArray[e];
                ed.Selected(e >= e1 && e <= e2);
            }
        }
    }

    /// <summary>Select only the faces of the Shape I.</summary>
    public void SelectFace(int i)
    {
        if (_ds is not null)
        {
            Standard_OutOfRange_Raise_if(i == 0 || i > _shapes.Length(), "HLRBRep_InternalAlgo::SelectFace : unknown Shape");

            int v1, v2, e1, e2, f1, f2;
            _shapes[i].Bounds(out v1, out v2, out e1, out e2, out f1, out f2);

            NCollection_Array1<HiddenLineFaceData> fDataArray = _ds.FDataArray();
            int nf = _ds.FaceCount;

            for (int f = 1; f <= nf; f++)
            {
                HiddenLineFaceData fd = fDataArray[f];
                fd.Selected(f >= f1 && f <= f2);
            }
        }
    }

    /// <summary>Set to visible all the edges.</summary>
    public void ShowAll()
    {
        if (_ds is not null)
        {
            NCollection_Array1<HiddenLineEdgeData> eDataArray = _ds.EDataArray();
            int ne = _ds.EdgeCount;

            for (int ie = 1; ie <= ne; ie++)
            {
                HiddenLineEdgeData ed = eDataArray[ie];
                ed.Status().ShowAll();
            }
        }
    }

    /// <summary>Set to visible all the edges of the Shape I.</summary>
    public void ShowAll(int i)
    {
        if (_ds is not null)
        {
            Standard_OutOfRange_Raise_if(i == 0 || i > _shapes.Length(), "HLRBRep_InternalAlgo::ShowAll : unknown Shape");

            Select(i);

            NCollection_Array1<HiddenLineEdgeData> eDataArray = _ds.EDataArray();
            int ne = _ds.EdgeCount;

            for (int e = 1; e <= ne; e++)
            {
                HiddenLineEdgeData ed = eDataArray[e];
                if (ed.Selected())
                {
                    ed.Status().ShowAll();
                }
            }
        }
    }

    /// <summary>Set to hide all the edges.</summary>
    public void HideAll()
    {
        if (_ds is not null)
        {
            NCollection_Array1<HiddenLineEdgeData> eDataArray = _ds.EDataArray();
            int ne = _ds.EdgeCount;

            for (int ie = 1; ie <= ne; ie++)
            {
                HiddenLineEdgeData ed = eDataArray[ie];
                ed.Status().HideAll();
            }
        }
    }

    /// <summary>Set to hide all the edges of the Shape I.</summary>
    public void HideAll(int i)
    {
        if (_ds is not null)
        {
            Standard_OutOfRange_Raise_if(i == 0 || i > _shapes.Length(), "HLRBRep_InternalAlgo::HideAll : unknown Shape");

            Select(i);

            NCollection_Array1<HiddenLineEdgeData> eDataArray = _ds.EDataArray();
            int ne = _ds.EdgeCount;

            for (int e = 1; e <= ne; e++)
            {
                HiddenLineEdgeData ed = eDataArray[e];
                if (ed.Selected())
                {
                    ed.Status().HideAll();
                }
            }
        }
    }

    /// <summary>Own hiding of all the shapes of the DataStructure without hiding by each other.</summary>
    public void PartialHide()
    {
        if (_ds is not null)
        {
            int i, n = _shapes.Length();

            if (_debug)
            {
                Console.Write(" Partial hiding" + '\n' + '\n');
            }

            for (i = 1; i <= n; i++)
            {
                Hide(i);
            }

            Select();
        }
    }

    /// <summary>Hide all the DataStructure.</summary>
    public void Hide()
    {
        if (_ds is not null)
        {
            int i, j, n = _shapes.Length();

            if (_debug)
            {
                Console.Write(" Total hiding" + '\n');
            }

            for (i = 1; i <= n; i++)
            {
                Hide(i);
            }

            for (i = 1; i <= n; i++)
            {
                for (j = 1; j <= n; j++)
                {
                    if (i != j)
                    {
                        Hide(i, j);
                    }
                }
            }

            Select();
        }
    }

    /// <summary>Hide the Shape I by itself.</summary>
    public void Hide(int i)
    {
        if (_ds is not null)
        {
            Standard_OutOfRange_Raise_if(i == 0 || i > _shapes.Length(), "HLRBRep_InternalAlgo::Hide : unknown Shape");

            if (_debug)
            {
                Console.Write(" hiding the shape " + i + " by itself" + '\n');
            }

            Select(i);
            InitEdgeStatus();
            HideSelected(i, true);
        }
    }

    /// <summary>Hide the Shape I by the shape J.</summary>
    public void Hide(int i, int j)
    {
        if (_ds is not null)
        {
            Standard_OutOfRange_Raise_if(i == 0 || i > _shapes.Length() || j == 0 || j > _shapes.Length(),
                                         "HLRBRep_InternalAlgo::Hide : unknown Shapes");

            if (i == j)
            {
                Hide(i);
            }
            else
            {
                ref EdgesBlock.MinMaxIndices minMaxShBI = ref _shapes[i].MinMax();
                ref EdgesBlock.MinMaxIndices minMaxShBJ = ref _shapes[j].MinMax();
                if (((minMaxShBJ.Max[0] - minMaxShBI.Min[0]) & 0x80008000) == 0
                    && ((minMaxShBI.Max[0] - minMaxShBJ.Min[0]) & 0x80008000) == 0
                    && ((minMaxShBJ.Max[1] - minMaxShBI.Min[1]) & 0x80008000) == 0
                    && ((minMaxShBI.Max[1] - minMaxShBJ.Min[1]) & 0x80008000) == 0
                    && ((minMaxShBJ.Max[2] - minMaxShBI.Min[2]) & 0x80008000) == 0
                    && ((minMaxShBI.Max[2] - minMaxShBJ.Min[2]) & 0x80008000) == 0
                    && ((minMaxShBJ.Max[3] - minMaxShBI.Min[3]) & 0x80008000) == 0
                    && ((minMaxShBI.Max[3] - minMaxShBJ.Min[3]) & 0x80008000) == 0
                    && ((minMaxShBJ.Max[4] - minMaxShBI.Min[4]) & 0x80008000) == 0
                    && ((minMaxShBI.Max[4] - minMaxShBJ.Min[4]) & 0x80008000) == 0
                    && ((minMaxShBJ.Max[5] - minMaxShBI.Min[5]) & 0x80008000) == 0
                    && ((minMaxShBI.Max[5] - minMaxShBJ.Min[5]) & 0x80008000) == 0
                    && ((minMaxShBJ.Max[6] - minMaxShBI.Min[6]) & 0x80008000) == 0
                    && ((minMaxShBJ.Max[7] - minMaxShBI.Min[7]) & 0x80008000) == 0)
                {
                    if (_debug)
                    {
                        Console.Write(" hiding the shape " + i);
                        Console.Write(" by the shape : " + j + '\n');
                    }
                    SelectEdge(i);
                    SelectFace(j);
                    HideSelected(i, false);
                }
            }
        }
    }

    /// <summary>
    /// First, if SideFace, own hiding of the side faces; then hiding of the selected parts of the DataStructure.
    /// </summary>
    private void HideSelected(int I, bool sideFace)
    {
        int e, f, j, nbVisEdges, nbSelEdges, nbSelFaces, nbCache;
        int nbFSide, nbFSimp;

        // OCCT_DEBUG: resetting the global counters left out

        ShapeBounds sb = _shapes[I];
        int v1, v2, e1, e2, f1, f2;
        sb.Bounds(out v1, out v2, out e1, out e2, out f1, out f2);

        if (e2 >= e1)
        {
            _ds!.InitBoundSort(sb.MinMax(), e1, e2);
            Hider cache = new(_ds);
            NCollection_Array1<HiddenLineEdgeData> eDataArray = _ds.EDataArray();
            NCollection_Array1<HiddenLineFaceData> fDataArray = _ds.FDataArray();
            int ne = _ds.EdgeCount;
            int nf = _ds.FaceCount;

            if (_debug)
            {
                nbVisEdges = 0;
                nbSelEdges = 0;
                nbSelFaces = 0;
                nbCache = 0;
                nbFSide = 0;
                nbFSimp = 0;

                for (e = 1; e <= ne; e++)
                {
                    HiddenLineEdgeData ed = eDataArray[e];
                    if (ed.Selected())
                    {
                        nbSelEdges++;
                        if (!ed.Status().AllHidden())
                        {
                            nbVisEdges++;
                        }
                    }
                }

                for (f = 1; f <= nf; f++)
                {
                    HiddenLineFaceData fd = fDataArray[f];
                    if (fd.Selected())
                    {
                        nbSelFaces++;
                        if (fd.Hiding())
                        {
                            nbCache++;
                        }
                        if (fd.Side())
                        {
                            nbFSide++;
                        }
                        if (fd.Simple())
                        {
                            nbFSimp++;
                        }
                    }
                }

                if (_debug)
                {
                    Console.Write('\n');
                    Console.Write("Vertices  : " + $"{_ds.VertexCount,5}" + '\n');
                    Console.Write("Edges     : " + $"{_ds.EdgeCount,5}" + " , ");
                    Console.Write("Selected  : " + $"{nbSelEdges,5}" + " , ");
                    Console.Write("Visible   : " + $"{nbVisEdges,5}" + '\n');
                    Console.Write("Faces     : " + $"{_ds.FaceCount,5}" + " , ");
                    Console.Write("Selected  : " + $"{nbSelFaces,5}" + " , ");
                    Console.Write("Simple    : " + $"{nbFSimp,5}" + '\n');
                    if (sideFace)
                    {
                        Console.Write("Side      : " + $"{nbFSide,5}" + " , ");
                    }
                    Console.Write("Cachantes : " + $"{nbCache,5}" + '\n' + '\n');
                }
            }

            if (nf == 0)
            {
                return;
            }

            int qwe = 0, qweqwe;
            qweqwe = nf / 10;

            if (sideFace)
            {
                j = 0;

                for (f = 1; f <= nf; f++)
                {
                    HiddenLineFaceData fd = fDataArray[f];
                    if (fd.Selected())
                    {
                        if (fd.Side())
                        {
                            if (HLRBRep_InternalAlgo_TRACE10)
                            {
                                if (++qwe > qweqwe)
                                {
                                    qwe = 0;
                                    if (_debug)
                                    {
                                        Console.Write("*");
                                    }
                                }
                            }
                            else
                            {
#pragma warning disable CS0162 // OCCT's trace switches are constants
                                if (_debug && HLRBRep_InternalAlgo_TRACE)
                                {
                                    j++;
                                    Console.Write(" OwnHiding " + j + " of face : " + f + '\n');
                                }
#pragma warning restore CS0162
                            }
                            cache.OwnHiding(f);
                        }
                    }
                }
            }

            //--
            NCollection_Array1<int> val = new(1, nf);
            NCollection_Array1<double> size = new(1, nf);
            NCollection_Array1<int> index = new(1, nf);

            for (f = 1; f <= nf; f++)
            {
                HiddenLineFaceData fd = fDataArray[f];
                if (fd.Plane())
                {
                    val[f] = 10;
                }
                else if (fd.Cylinder())
                {
                    val[f] = 9;
                }
                else if (fd.Cone())
                {
                    val[f] = 8;
                }
                else if (fd.Sphere())
                {
                    val[f] = 7;
                }
                else if (fd.Torus())
                {
                    val[f] = 6;
                }
                else
                {
                    val[f] = 0;
                }
                if (fd.Cut())
                {
                    val[f] -= 10;
                }
                if (fd.Side())
                {
                    val[f] -= 100;
                }
                if (fd.WithOutL())
                {
                    val[f] -= 20;
                }

                size[f] = fd.Size();
            }

            for (int tt = 1; tt <= nf; tt++)
            {
                index[tt] = tt;
            }

            //-- ======================================================================
            // OCCT's commented-out bubble sort left out
            //-- ======================================================================
            if (nf > 2)
            {
                int i, ir, k, l;
                int rra;
                l = (nf >> 1) + 1;
                ir = nf;
                for (;;)
                {
                    if (l > 1)
                    {
                        rra = index[--l];
                    }
                    else
                    {
                        rra = index[ir];
                        index[ir] = index[1];
                        if (--ir == 1)
                        {
                            index[1] = rra;
                            break;
                        }
                    }
                    i = l;
                    k = l + l;
                    while (k <= ir)
                    {
                        if (k < ir)
                        {
                            if (val[index[k]] > val[index[k + 1]])
                            {
                                k++;
                            }
                            else if (val[index[k]] == val[index[k + 1]])
                            {
                                if (size[index[k]] > size[index[k + 1]])
                                {
                                    k++;
                                }
                            }
                        }
                        if (val[rra] > val[index[k]])
                        {
                            index[i] = index[k];
                            i = k;
                            k <<= 1;
                        }
                        else if ((val[rra] == val[index[k]]) && (size[rra] > size[index[k]]))
                        {
                            index[i] = index[k];
                            i = k;
                            k <<= 1;
                        }
                        else
                        {
                            k = ir + 1;
                        }
                    }
                    index[i] = rra;
                }
            }

            j = 0;

            qwe = 0;
            for (f = 1; f <= nf; f++)
            {
                int fi = index[f];
                HiddenLineFaceData fd = fDataArray[fi];
                if (fd.Selected())
                {
                    if (fd.Hiding())
                    {
#pragma warning disable CS0162 // OCCT's trace switches are constants
                        if (HLRBRep_InternalAlgo_TRACE10 && !HLRBRep_InternalAlgo_TRACE)
                        {
                            if (++qwe > qweqwe)
                            {
                                if (_debug)
                                {
                                    Console.Write(".");
                                }
                                qwe = 0;
                            }
                        }
                        else if (_debug && HLRBRep_InternalAlgo_TRACE)
                        {
                            j++;
                            Console.Write($"{fi,6}");
                            if (++s_rty > 25)
                            {
                                s_rty = 0;
                                Console.Write("\n");
                            }
                        }
#pragma warning restore CS0162
                        cache.Hide(fi, _mapOfShapeTool);
                    }
                }
            }

            // OCCT_DEBUG: the counters' trace left out
        }
    }

    public void Debug(bool deb)
    {
        _debug = deb;
    }

    public bool Debug() => _debug;

    /// <summary>The data structure (null before <see cref="Update"/>).</summary>
    public HiddenLineData? DataStructure() => _ds;
}
