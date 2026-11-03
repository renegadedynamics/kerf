// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_InternalAlgo.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_InternalAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The exact hidden-line removal: loads outliners (shapes), builds their data structure for a projector, and hides
/// their edges by their own faces and by each other's.
/// </summary>
/// <remarks>
/// A working object (a transient), as OCCT's. The outliners copy frozen shapes at the entry (decision 1095), so the
/// algorithm never writes into the caller's shapes. <see cref="Debug(bool)"/> turns on OCCT's traces, written to
/// standard output.
/// </remarks>
public class HLRBRep_InternalAlgo : Transient
{
    private HLRBRep_Data? myDS;
    private HLRAlgo_Projector myProj = new();
    private readonly NCollection_Sequence<HLRBRep_ShapeBounds> myShapes = new();
    private readonly NCollection_DataMap<TopoDS_Shape, BRepTopAdaptor_Tool> myMapOfShapeTool =
        new(TopTools_ShapeMapHasher.Instance);
    private bool myDebug;

    // HLRBRep_InternalAlgo.cxx's file statics, constants in OCCT's code (never changed)
    private const bool HLRBRep_InternalAlgo_TRACE = true;
    private const bool HLRBRep_InternalAlgo_TRACE10 = true;

    // The function-scope static of HideSelected's trace (only with myDebug, which OCCT's trace reaches only when
    // HLRBRep_InternalAlgo_TRACE10 is false: never)
    [ThreadStatic]
    private static int rty;

    public HLRBRep_InternalAlgo()
    {
        myDebug = false;
    }

    /// <summary>
    /// OCCT's copy: shares A's data structure, copies its projector and its shapes' bounds, but not its map of face
    /// adaptors.
    /// </summary>
    public HLRBRep_InternalAlgo(HLRBRep_InternalAlgo A)
    {
        myDS = A.DataStructure();
        myProj = A.Projector();
        NCollection_Sequence<HLRBRep_ShapeBounds> aShapes = A.SeqOfShapeBounds();
        for (int i = 1; i <= aShapes.Length(); i++)
        {
            myShapes.Append(new HLRBRep_ShapeBounds(aShapes[i]));
        }
        myDebug = A.Debug();
    }

    /// <summary>Set the projector.</summary>
    public void Projector(in HLRAlgo_Projector P)
    {
        myProj = P;
    }

    /// <summary>The projector (a reference to the stored one, as OCCT's).</summary>
    public ref HLRAlgo_Projector Projector() => ref myProj;

    /// <summary>Update the DataStructure.</summary>
    public void Update()
    {
        if (!myShapes.IsEmpty())
        {
            int n = myShapes.Length();
            HLRBRep_Data[] DS = new HLRBRep_Data[n];

            int i, dv, de, df, nv = 0, ne = 0, nf = 0;

            for (i = 1; i <= n; i++)
            {
                HLRBRep_ShapeBounds SB = myShapes[i];
                try
                {
                    DS[i - 1] = HLRBRep_ShapeToHLR.Load(SB.Shape()!, myProj, myMapOfShapeTool, SB.NbOfIso());
                    dv = DS[i - 1].NbVertices();
                    de = DS[i - 1].NbEdges();
                    df = DS[i - 1].NbFaces();
                }
                catch (Exception anException) when (IsCaught(anException))
                {
                    if (myDebug)
                    {
                        Console.Write("An exception was caught when preparing the Shape " + i);
                        Console.Write(" and computing its OutLines " + '\n');
                        Console.Write(anException.Message + '\n');
                    }
                    DS[i - 1] = new HLRBRep_Data(0, 0, 0);
                    dv = 0;
                    de = 0;
                    df = 0;
                }

                myShapes[i] = new HLRBRep_ShapeBounds(SB.Shape()!, SB.ShapeData(), SB.NbOfIso(), 1, dv, 1, de, 1, df);
                nv += dv;
                ne += de;
                nf += df;
            }

            if (n == 1)
            {
                myDS = DS[0];
            }
            else
            {
                myDS = new HLRBRep_Data(nv, ne, nf);
                nv = 0;
                ne = 0;
                nf = 0;

                for (i = 1; i <= n; i++)
                {
                    HLRBRep_ShapeBounds SB = myShapes[i];
                    SB.Sizes(out dv, out de, out df);
                    SB.Translate(nv, ne, nf);
                    myDS.Write(DS[i - 1], nv, ne, nf);
                    nv += dv;
                    ne += de;
                    nf += df;
                }
            }

            myDS.Update(myProj);

            // OCCT leaves these unset (a shape without edges then adds its faces' boxes to garbage); zeros here
            HLRAlgo_EdgesBlock.MinMaxIndices ShapMin = new(), ShapMax = new(), MinMaxShap = new();
            HLRAlgo_EdgesBlock.MinMaxIndices TheMin = new(), TheMax = new();
            NCollection_Array1<HLRBRep_EdgeData> aEDataArray = myDS.EDataArray();
            NCollection_Array1<HLRBRep_FaceData> aFDataArray = myDS.FDataArray();

            for (i = 1; i <= n; i++)
            {
                bool FirstTime = true;
                HLRBRep_ShapeBounds SB = myShapes[i];
                int v1, v2, e1, e2, f1, f2;
                SB.Bounds(out v1, out v2, out e1, out e2, out f1, out f2);

                for (int e = e1; e <= e2; e++)
                {
                    HLRBRep_EdgeData ed = aEDataArray[e];
                    HLRAlgo.DecodeMinMax(ed.MinMax(), ref TheMin, ref TheMax);
                    if (FirstTime)
                    {
                        FirstTime = false;
                        HLRAlgo.CopyMinMax(ref TheMin, ref TheMax, ref ShapMin, ref ShapMax);
                    }
                    else
                    {
                        HLRAlgo.AddMinMax(ref TheMin, ref TheMax, ref ShapMin, ref ShapMax);
                    }
                }

                for (int f = f1; f <= f2; f++)
                {
                    HLRBRep_FaceData fd = aFDataArray[f];
                    HLRAlgo.DecodeMinMax(fd.Wires()!.MinMax(), ref TheMin, ref TheMax);
                    HLRAlgo.AddMinMax(ref TheMin, ref TheMax, ref ShapMin, ref ShapMax);
                }
                HLRAlgo.EncodeMinMax(ref ShapMin, ref ShapMax, ref MinMaxShap);
                SB.UpdateMinMax(MinMaxShap);
            }
        }
    }

    // The exceptions OCCT's catch (Standard_Failure) handles under OCC_CATCH_SIGNALS
    private static bool IsCaught(Exception theException)
        => theException is FailureException
               or NullReferenceException
               or InvalidCastException
               or IndexOutOfRangeException
               or ArgumentOutOfRangeException
               or DivideByZeroException;

    /// <summary>Add the shape S, with the shape data SData and nbIso isolines.</summary>
    public void Load(HLRTopoBRep_OutLiner S, Transient? SData, int nbIso = 0)
    {
        myShapes.Append(new HLRBRep_ShapeBounds(S, SData, nbIso, 0, 0, 0, 0, 0, 0));
        myDS = null;
    }

    /// <summary>Add the shape S, with nbIso isolines.</summary>
    public void Load(HLRTopoBRep_OutLiner S, int nbIso = 0)
    {
        myShapes.Append(new HLRBRep_ShapeBounds(S, nbIso, 0, 0, 0, 0, 0, 0));
        myDS = null;
    }

    /// <summary>Return the index of the Shape S and return 0 if the Shape S is not found.</summary>
    public int Index(HLRTopoBRep_OutLiner S)
    {
        int n = myShapes.Length();

        for (int i = 1; i <= n; i++)
        {
            if (myShapes[i].Shape() == S)
            {
                return i;
            }
        }

        return 0;
    }

    /// <summary>Remove the Shape of Index I.</summary>
    public void Remove(int I)
    {
        Standard_OutOfRange_Raise_if(I == 0 || I > myShapes.Length(), "HLRBRep_InternalAlgo::Remove : unknown Shape");
        myShapes.Remove(I);

        myMapOfShapeTool.Clear();
        myDS = null;
    }

    /// <summary>Change the Shape Data of the Shape of index I.</summary>
    public void ShapeData(int I, Transient? SData)
    {
        Standard_OutOfRange_Raise_if(I == 0 || I > myShapes.Length(), "HLRBRep_InternalAlgo::ShapeData : unknown Shape");

        myShapes[I].ShapeData(SData);
    }

    /// <summary>The sequence of the shapes' bounds (the stored one, as OCCT's reference).</summary>
    public NCollection_Sequence<HLRBRep_ShapeBounds> SeqOfShapeBounds() => myShapes;

    public int NbShapes() => myShapes.Length();

    public HLRBRep_ShapeBounds ShapeBounds(int I)
    {
        Standard_OutOfRange_Raise_if(I == 0 || I > myShapes.Length(), "HLRBRep_InternalAlgo::ShapeBounds : unknown Shape");

        return myShapes[I];
    }

    /// <summary>Init the status of the selected edges depending of the back faces of a closed shell.</summary>
    public void InitEdgeStatus()
    {
        bool visible;
        HLRBRep_FaceIterator faceIt = new();

        NCollection_Array1<HLRBRep_EdgeData> aEDataArray = myDS!.EDataArray();
        NCollection_Array1<HLRBRep_FaceData> aFDataArray = myDS.FDataArray();
        int ne = myDS.NbEdges();
        int nf = myDS.NbFaces();

        for (int e = 1; e <= ne; e++)
        {
            HLRBRep_EdgeData ed = aEDataArray[e];
            if (ed.Selected())
            {
                ed.Status().ShowAll();
            }
        }
        //  for (int f = 1; f <= nf; f++) {
        int f;
        for (f = 1; f <= nf; f++)
        {
            HLRBRep_FaceData fd = aFDataArray[f];
            if (fd.Selected())
            {
                for (faceIt.InitEdge(fd); faceIt.MoreEdge(); faceIt.NextEdge())
                {
                    HLRBRep_EdgeData edf = myDS.EDataArray()[faceIt.Edge()];
                    if (edf.Selected())
                    {
                        edf.Status().HideAll();
                    }
                }
            }
        }

        for (f = 1; f <= nf; f++)
        {
            HLRBRep_FaceData fd = aFDataArray[f];
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
                        case TopAbs_REVERSED:
                            visible = fd.Back();
                            break;
                        case TopAbs_FORWARD:
                            visible = !fd.Back();
                            break;
                        case TopAbs_EXTERNAL:
                        case TopAbs_INTERNAL:
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
                    HLRBRep_EdgeData edf = myDS.EDataArray()[E];
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
        if (myDS is not null)
        {
            NCollection_Array1<HLRBRep_EdgeData> aEDataArray = myDS.EDataArray();
            NCollection_Array1<HLRBRep_FaceData> aFDataArray = myDS.FDataArray();
            int ne = myDS.NbEdges();
            int nf = myDS.NbFaces();

            for (int e = 1; e <= ne; e++)
            {
                HLRBRep_EdgeData ed = aEDataArray[e];
                ed.Selected(true);
            }

            for (int f = 1; f <= nf; f++)
            {
                HLRBRep_FaceData fd = aFDataArray[f];
                fd.Selected(true);
            }
        }
    }

    /// <summary>Select only the Shape of index I.</summary>
    public void Select(int I)
    {
        if (myDS is not null)
        {
            Standard_OutOfRange_Raise_if(I == 0 || I > myShapes.Length(), "HLRBRep_InternalAlgo::Select : unknown Shape");

            int v1, v2, e1, e2, f1, f2;
            myShapes[I].Bounds(out v1, out v2, out e1, out e2, out f1, out f2);

            NCollection_Array1<HLRBRep_EdgeData> aEDataArray = myDS.EDataArray();
            NCollection_Array1<HLRBRep_FaceData> aFDataArray = myDS.FDataArray();
            int ne = myDS.NbEdges();
            int nf = myDS.NbFaces();

            for (int e = 1; e <= ne; e++)
            {
                HLRBRep_EdgeData ed = aEDataArray[e];
                ed.Selected(e >= e1 && e <= e2);
            }

            for (int f = 1; f <= nf; f++)
            {
                HLRBRep_FaceData fd = aFDataArray[f];
                fd.Selected(f >= f1 && f <= f2);
            }
        }
    }

    /// <summary>Select only the edges of the Shape I.</summary>
    public void SelectEdge(int I)
    {
        if (myDS is not null)
        {
            Standard_OutOfRange_Raise_if(I == 0 || I > myShapes.Length(), "HLRBRep_InternalAlgo::SelectEdge : unknown Shape");

            int v1, v2, e1, e2, f1, f2;
            myShapes[I].Bounds(out v1, out v2, out e1, out e2, out f1, out f2);

            NCollection_Array1<HLRBRep_EdgeData> aEDataArray = myDS.EDataArray();
            int ne = myDS.NbEdges();

            for (int e = 1; e <= ne; e++)
            {
                HLRBRep_EdgeData ed = aEDataArray[e];
                ed.Selected(e >= e1 && e <= e2);
            }
        }
    }

    /// <summary>Select only the faces of the Shape I.</summary>
    public void SelectFace(int I)
    {
        if (myDS is not null)
        {
            Standard_OutOfRange_Raise_if(I == 0 || I > myShapes.Length(), "HLRBRep_InternalAlgo::SelectFace : unknown Shape");

            int v1, v2, e1, e2, f1, f2;
            myShapes[I].Bounds(out v1, out v2, out e1, out e2, out f1, out f2);

            NCollection_Array1<HLRBRep_FaceData> aFDataArray = myDS.FDataArray();
            int nf = myDS.NbFaces();

            for (int f = 1; f <= nf; f++)
            {
                HLRBRep_FaceData fd = aFDataArray[f];
                fd.Selected(f >= f1 && f <= f2);
            }
        }
    }

    /// <summary>Set to visible all the edges.</summary>
    public void ShowAll()
    {
        if (myDS is not null)
        {
            NCollection_Array1<HLRBRep_EdgeData> aEDataArray = myDS.EDataArray();
            int ne = myDS.NbEdges();

            for (int ie = 1; ie <= ne; ie++)
            {
                HLRBRep_EdgeData ed = aEDataArray[ie];
                ed.Status().ShowAll();
            }
        }
    }

    /// <summary>Set to visible all the edges of the Shape I.</summary>
    public void ShowAll(int I)
    {
        if (myDS is not null)
        {
            Standard_OutOfRange_Raise_if(I == 0 || I > myShapes.Length(), "HLRBRep_InternalAlgo::ShowAll : unknown Shape");

            Select(I);

            NCollection_Array1<HLRBRep_EdgeData> aEDataArray = myDS.EDataArray();
            int ne = myDS.NbEdges();

            for (int e = 1; e <= ne; e++)
            {
                HLRBRep_EdgeData ed = aEDataArray[e];
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
        if (myDS is not null)
        {
            NCollection_Array1<HLRBRep_EdgeData> aEDataArray = myDS.EDataArray();
            int ne = myDS.NbEdges();

            for (int ie = 1; ie <= ne; ie++)
            {
                HLRBRep_EdgeData ed = aEDataArray[ie];
                ed.Status().HideAll();
            }
        }
    }

    /// <summary>Set to hide all the edges of the Shape I.</summary>
    public void HideAll(int I)
    {
        if (myDS is not null)
        {
            Standard_OutOfRange_Raise_if(I == 0 || I > myShapes.Length(), "HLRBRep_InternalAlgo::HideAll : unknown Shape");

            Select(I);

            NCollection_Array1<HLRBRep_EdgeData> aEDataArray = myDS.EDataArray();
            int ne = myDS.NbEdges();

            for (int e = 1; e <= ne; e++)
            {
                HLRBRep_EdgeData ed = aEDataArray[e];
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
        if (myDS is not null)
        {
            int i, n = myShapes.Length();

            if (myDebug)
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
        if (myDS is not null)
        {
            int i, j, n = myShapes.Length();

            if (myDebug)
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
    public void Hide(int I)
    {
        if (myDS is not null)
        {
            Standard_OutOfRange_Raise_if(I == 0 || I > myShapes.Length(), "HLRBRep_InternalAlgo::Hide : unknown Shape");

            if (myDebug)
            {
                Console.Write(" hiding the shape " + I + " by itself" + '\n');
            }

            Select(I);
            InitEdgeStatus();
            HideSelected(I, true);
        }
    }

    /// <summary>Hide the Shape I by the shape J.</summary>
    public void Hide(int I, int J)
    {
        if (myDS is not null)
        {
            Standard_OutOfRange_Raise_if(I == 0 || I > myShapes.Length() || J == 0 || J > myShapes.Length(),
                                         "HLRBRep_InternalAlgo::Hide : unknown Shapes");

            if (I == J)
            {
                Hide(I);
            }
            else
            {
                ref HLRAlgo_EdgesBlock.MinMaxIndices MinMaxShBI = ref myShapes[I].MinMax();
                ref HLRAlgo_EdgesBlock.MinMaxIndices MinMaxShBJ = ref myShapes[J].MinMax();
                if (((MinMaxShBJ.Max[0] - MinMaxShBI.Min[0]) & 0x80008000) == 0
                    && ((MinMaxShBI.Max[0] - MinMaxShBJ.Min[0]) & 0x80008000) == 0
                    && ((MinMaxShBJ.Max[1] - MinMaxShBI.Min[1]) & 0x80008000) == 0
                    && ((MinMaxShBI.Max[1] - MinMaxShBJ.Min[1]) & 0x80008000) == 0
                    && ((MinMaxShBJ.Max[2] - MinMaxShBI.Min[2]) & 0x80008000) == 0
                    && ((MinMaxShBI.Max[2] - MinMaxShBJ.Min[2]) & 0x80008000) == 0
                    && ((MinMaxShBJ.Max[3] - MinMaxShBI.Min[3]) & 0x80008000) == 0
                    && ((MinMaxShBI.Max[3] - MinMaxShBJ.Min[3]) & 0x80008000) == 0
                    && ((MinMaxShBJ.Max[4] - MinMaxShBI.Min[4]) & 0x80008000) == 0
                    && ((MinMaxShBI.Max[4] - MinMaxShBJ.Min[4]) & 0x80008000) == 0
                    && ((MinMaxShBJ.Max[5] - MinMaxShBI.Min[5]) & 0x80008000) == 0
                    && ((MinMaxShBI.Max[5] - MinMaxShBJ.Min[5]) & 0x80008000) == 0
                    && ((MinMaxShBJ.Max[6] - MinMaxShBI.Min[6]) & 0x80008000) == 0
                    && ((MinMaxShBJ.Max[7] - MinMaxShBI.Min[7]) & 0x80008000) == 0)
                {
                    if (myDebug)
                    {
                        Console.Write(" hiding the shape " + I);
                        Console.Write(" by the shape : " + J + '\n');
                    }
                    SelectEdge(I);
                    SelectFace(J);
                    HideSelected(I, false);
                }
            }
        }
    }

    /// <summary>
    /// First, if SideFace, own hiding of the side faces; then hiding of the selected parts of the DataStructure.
    /// </summary>
    private void HideSelected(int I, bool SideFace)
    {
        int e, f, j, nbVisEdges, nbSelEdges, nbSelFaces, nbCache;
        int nbFSide, nbFSimp;

        // OCCT_DEBUG: resetting the global counters left out

        HLRBRep_ShapeBounds SB = myShapes[I];
        int v1, v2, e1, e2, f1, f2;
        SB.Bounds(out v1, out v2, out e1, out e2, out f1, out f2);

        if (e2 >= e1)
        {
            myDS!.InitBoundSort(SB.MinMax(), e1, e2);
            HLRBRep_Hider Cache = new(myDS);
            NCollection_Array1<HLRBRep_EdgeData> aEDataArray = myDS.EDataArray();
            NCollection_Array1<HLRBRep_FaceData> aFDataArray = myDS.FDataArray();
            int ne = myDS.NbEdges();
            int nf = myDS.NbFaces();

            if (myDebug)
            {
                nbVisEdges = 0;
                nbSelEdges = 0;
                nbSelFaces = 0;
                nbCache = 0;
                nbFSide = 0;
                nbFSimp = 0;

                for (e = 1; e <= ne; e++)
                {
                    HLRBRep_EdgeData ed = aEDataArray[e];
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
                    HLRBRep_FaceData fd = aFDataArray[f];
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

                if (myDebug)
                {
                    Console.Write('\n');
                    Console.Write("Vertices  : " + $"{myDS.NbVertices(),5}" + '\n');
                    Console.Write("Edges     : " + $"{myDS.NbEdges(),5}" + " , ");
                    Console.Write("Selected  : " + $"{nbSelEdges,5}" + " , ");
                    Console.Write("Visible   : " + $"{nbVisEdges,5}" + '\n');
                    Console.Write("Faces     : " + $"{myDS.NbFaces(),5}" + " , ");
                    Console.Write("Selected  : " + $"{nbSelFaces,5}" + " , ");
                    Console.Write("Simple    : " + $"{nbFSimp,5}" + '\n');
                    if (SideFace)
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

            int QWE = 0, QWEQWE;
            QWEQWE = nf / 10;

            if (SideFace)
            {
                j = 0;

                for (f = 1; f <= nf; f++)
                {
                    HLRBRep_FaceData fd = aFDataArray[f];
                    if (fd.Selected())
                    {
                        if (fd.Side())
                        {
                            if (HLRBRep_InternalAlgo_TRACE10)
                            {
                                if (++QWE > QWEQWE)
                                {
                                    QWE = 0;
                                    if (myDebug)
                                    {
                                        Console.Write("*");
                                    }
                                }
                            }
                            else
                            {
#pragma warning disable CS0162 // OCCT's trace switches are constants
                                if (myDebug && HLRBRep_InternalAlgo_TRACE)
                                {
                                    j++;
                                    Console.Write(" OwnHiding " + j + " of face : " + f + '\n');
                                }
#pragma warning restore CS0162
                            }
                            Cache.OwnHiding(f);
                        }
                    }
                }
            }

            //--
            NCollection_Array1<int> Val = new(1, nf);
            NCollection_Array1<double> Size = new(1, nf);
            NCollection_Array1<int> Index = new(1, nf);

            for (f = 1; f <= nf; f++)
            {
                HLRBRep_FaceData fd = aFDataArray[f];
                if (fd.Plane())
                {
                    Val[f] = 10;
                }
                else if (fd.Cylinder())
                {
                    Val[f] = 9;
                }
                else if (fd.Cone())
                {
                    Val[f] = 8;
                }
                else if (fd.Sphere())
                {
                    Val[f] = 7;
                }
                else if (fd.Torus())
                {
                    Val[f] = 6;
                }
                else
                {
                    Val[f] = 0;
                }
                if (fd.Cut())
                {
                    Val[f] -= 10;
                }
                if (fd.Side())
                {
                    Val[f] -= 100;
                }
                if (fd.WithOutL())
                {
                    Val[f] -= 20;
                }

                Size[f] = fd.Size();
            }

            for (int tt = 1; tt <= nf; tt++)
            {
                Index[tt] = tt;
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
                        rra = Index[--l];
                    }
                    else
                    {
                        rra = Index[ir];
                        Index[ir] = Index[1];
                        if (--ir == 1)
                        {
                            Index[1] = rra;
                            break;
                        }
                    }
                    i = l;
                    k = l + l;
                    while (k <= ir)
                    {
                        if (k < ir)
                        {
                            if (Val[Index[k]] > Val[Index[k + 1]])
                            {
                                k++;
                            }
                            else if (Val[Index[k]] == Val[Index[k + 1]])
                            {
                                if (Size[Index[k]] > Size[Index[k + 1]])
                                {
                                    k++;
                                }
                            }
                        }
                        if (Val[rra] > Val[Index[k]])
                        {
                            Index[i] = Index[k];
                            i = k;
                            k <<= 1;
                        }
                        else if ((Val[rra] == Val[Index[k]]) && (Size[rra] > Size[Index[k]]))
                        {
                            Index[i] = Index[k];
                            i = k;
                            k <<= 1;
                        }
                        else
                        {
                            k = ir + 1;
                        }
                    }
                    Index[i] = rra;
                }
            }

            j = 0;

            QWE = 0;
            for (f = 1; f <= nf; f++)
            {
                int fi = Index[f];
                HLRBRep_FaceData fd = aFDataArray[fi];
                if (fd.Selected())
                {
                    if (fd.Hiding())
                    {
#pragma warning disable CS0162 // OCCT's trace switches are constants
                        if (HLRBRep_InternalAlgo_TRACE10 && !HLRBRep_InternalAlgo_TRACE)
                        {
                            if (++QWE > QWEQWE)
                            {
                                if (myDebug)
                                {
                                    Console.Write(".");
                                }
                                QWE = 0;
                            }
                        }
                        else if (myDebug && HLRBRep_InternalAlgo_TRACE)
                        {
                            j++;
                            Console.Write($"{fi,6}");
                            if (++rty > 25)
                            {
                                rty = 0;
                                Console.Write("\n");
                            }
                        }
#pragma warning restore CS0162
                        Cache.Hide(fi, myMapOfShapeTool);
                    }
                }
            }

            // OCCT_DEBUG: the counters' trace left out
        }
    }

    public void Debug(bool deb)
    {
        myDebug = deb;
    }

    public bool Debug() => myDebug;

    /// <summary>The data structure (null before <see cref="Update"/>).</summary>
    public HLRBRep_Data? DataStructure() => myDS;
}
