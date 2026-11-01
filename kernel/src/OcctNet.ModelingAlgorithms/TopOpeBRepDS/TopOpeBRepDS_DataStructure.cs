// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_DataStructure.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_DataStructure.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopOpeBRepDS_Config;

namespace Occt;

/// <summary>
/// The legacy topological data structure: the new geometries (surfaces, curves, points) a topological operation
/// creates, the shapes they touch, and the interferences that attach each to the others.
/// </summary>
/// <remarks>
/// <para>
/// Surfaces, curves and points are numbered from 1 in the order added; a removed curve or surface keeps its number
/// (unbound, or not kept). Shapes are numbered from 1 in the order added (an indexed map keyed by
/// <see cref="TopTools_ShapeMapHasher"/>: same TShape and location, any orientation).
/// </para>
/// <para>
/// A working object, internal to the fillet builders and to unit 7R's reconstruction (plan 2.1): the mutators are
/// internal; the queries are public, so a caller can read a builder's data structure. The <c>Change*</c> accessors
/// give the stored value for writing (by <c>ref</c> for the value types <see cref="TopOpeBRepDS_Curve"/>,
/// <see cref="TopOpeBRepDS_Point"/> and <see cref="TopOpeBRepDS_Surface"/>; the lists themselves for interferences).
/// </para>
/// <para>
/// As in OCCT, an accessor asked for a geometry or shape that isn't there returns the data structure's own empty
/// object, and the <c>Change*</c> accessors return it for writing: what is written there is seen by later calls that
/// miss (decision 1066). OCCT's two function-scope statics (the empty shape-with-state map and value) are members here.
/// </para>
/// </remarks>
public sealed class TopOpeBRepDS_DataStructure
{
    private int myNbSurfaces;
    internal readonly NCollection_DataMap<int, TopOpeBRepDS_SurfaceData> mySurfaces = new();
    private int myNbCurves;
    internal readonly NCollection_DataMap<int, TopOpeBRepDS_CurveData> myCurves = new();
    private int myNbPoints;
    internal readonly NCollection_DataMap<int, TopOpeBRepDS_PointData> myPoints = new();
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, TopOpeBRepDS_ShapeData> myShapes = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> mySectionEdges = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_List<TopOpeBRepDS_Interference> myEmptyListOfInterference = new();
    private readonly NCollection_List<TopoDS_Shape> myEmptyListOfShape = new();
    private TopoDS_Shape myEmptyShape = new();
    private TopOpeBRepDS_Point myEmptyPoint = new();
    private TopOpeBRepDS_Surface myEmptySurface = new();
    private TopOpeBRepDS_Curve myEmptyCurve = new();
    private readonly NCollection_DataMap<TopoDS_Shape, Geom_Surface> myNewSurface = new(TopTools_ShapeMapHasher.Instance);
    private bool myIsfafa;
    private int myI;
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, TopOpeBRepDS_ShapeWithState> myMapOfShapeWithStateObj = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, TopOpeBRepDS_ShapeWithState> myMapOfShapeWithStateTool = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapOfRejectedShapesObj = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapOfRejectedShapesTool = new(TopTools_ShapeMapHasher.Instance);

    // OCCT.NET: OCCT's function-scope statics of ChangeMapOfShapeWithState and GetShapeWithState (decision 1063)
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, TopOpeBRepDS_ShapeWithState> myDummyMapOfShapeWithState = new(TopTools_ShapeMapHasher.Instance);
    private readonly TopOpeBRepDS_ShapeWithState myDummyShapeWithState = new();

    /// <summary>An empty data structure.</summary>
    public TopOpeBRepDS_DataStructure()
    {
        myNbSurfaces = 0;
        myNbCurves = 0;
        myNbPoints = 0;
        myIsfafa = false;
        myI = 0;
        _ = myI;
    }

    /// <summary>Empties the data structure (the new surfaces of faces and the fafa flag are kept, as in OCCT).</summary>
    internal void Init()
    {
        myNbSurfaces = 0;
        myNbCurves = 0;
        myNbPoints = 0;
        mySurfaces.Clear();
        myCurves.Clear();
        myPoints.Clear();
        myShapes.Clear();
        // Begin modified by NIZHNY-MZV  Tue Apr 18 16:33:26 2000
        myMapOfShapeWithStateObj.Clear();
        myMapOfShapeWithStateTool.Clear();
        myMapOfRejectedShapesObj.Clear();
        myMapOfRejectedShapesTool.Clear();
        // End modified by NIZHNY-MZV  Tue Apr 18 16:33:32 2000
        InitSectionEdges();
    }

    /// <summary>Adds a copy of the surface <paramref name="S"/>; returns its index.</summary>
    internal int AddSurface(in TopOpeBRepDS_Surface S)
    {
        myNbSurfaces++;
        // modified by NIZNHY-PKV Tue Oct 30 09:22:04 2001 f
        TopOpeBRepDS_SurfaceData aSD = new(S);
        mySurfaces.Bind(myNbSurfaces, aSD);
        // mySurfaces.Bind(myNbSurfaces,S);
        // modified by NIZNHY-PKV Tue Oct 30 09:22:20 2001 t
        return myNbSurfaces;
    }

    /// <summary>Removes the surface <paramref name="I"/> (its index is not reused).</summary>
    internal void RemoveSurface(int I)
    {
        mySurfaces.UnBind(I);
    }

    /// <summary>Whether the surface <paramref name="I"/> is kept.</summary>
    public bool KeepSurface(int I)
    {
        TopOpeBRepDS_Surface S = Surface(I);
        bool b = S.Keep();
        return b;
    }

    /// <summary>Whether the surface <paramref name="S"/> is kept.</summary>
    public bool KeepSurface(in TopOpeBRepDS_Surface S)
    {
        bool b = S.Keep();
        return b;
    }

    internal void ChangeKeepSurface(int I, bool FindKeep)
    {
        ref TopOpeBRepDS_Surface S = ref ChangeSurface(I);
        S.ChangeKeep(FindKeep);
    }

    internal static void ChangeKeepSurface(ref TopOpeBRepDS_Surface S, bool FindKeep)
    {
        S.ChangeKeep(FindKeep);
    }

    /// <summary>
    /// Adds a copy of the curve <paramref name="C"/>; returns its index. As in OCCT, <paramref name="C"/>'s own
    /// <see cref="TopOpeBRepDS_Curve.DSIndex"/> is set to it first.
    /// </summary>
    internal int AddCurve(ref TopOpeBRepDS_Curve C)
    {
        myNbCurves++;

        // NYI : modifier const & sur Curve dans le CDL NYI
        C.ChangeDSIndex(myNbCurves);

        TopOpeBRepDS_CurveData CD = new(C);
        myCurves.Bind(myNbCurves, CD);
        return myNbCurves;
    }

    /// <summary>
    /// Adds a copy of the curve <paramref name="C"/> (OCCT's calls with a temporary); returns its index. Use the
    /// <c>ref</c> overload where the caller reads the curve's <see cref="TopOpeBRepDS_Curve.DSIndex"/> afterwards.
    /// </summary>
    internal int AddCurve(TopOpeBRepDS_Curve C) => AddCurve(ref C);

    /// <summary>Removes the curve <paramref name="I"/>: its two surface/curve interferences leave their shapes' lists, and the curve is no longer kept.</summary>
    internal void RemoveCurve(int I)
    {
        ref TopOpeBRepDS_Curve C = ref ChangeCurve(I);
        C.GetShapes(out TopoDS_Shape S1, out TopoDS_Shape S2);
        TopOpeBRepDS_Interference? I1 = C.GetSCI1();
        TopOpeBRepDS_Interference? I2 = C.GetSCI2();
        if (I1 is not null)
        {
            RemoveShapeInterference(S1, I1);
        }
        if (I2 is not null)
        {
            RemoveShapeInterference(S2, I2);
        }
        C.ChangeKeep(false);

        // NYI : nullify interferences I1,I2
    }

    /// <summary>Whether the curve <paramref name="I"/> is kept.</summary>
    public bool KeepCurve(int I)
    {
        TopOpeBRepDS_Curve C = Curve(I);
        bool b = C.Keep();
        return b;
    }

    /// <summary>Whether the curve <paramref name="C"/> is kept.</summary>
    public bool KeepCurve(in TopOpeBRepDS_Curve C)
    {
        bool b = C.Keep();
        return b;
    }

    internal void ChangeKeepCurve(int I, bool FindKeep)
    {
        ref TopOpeBRepDS_Curve C = ref ChangeCurve(I);
        C.ChangeKeep(FindKeep);
    }

    internal static void ChangeKeepCurve(ref TopOpeBRepDS_Curve C, bool FindKeep)
    {
        C.ChangeKeep(FindKeep);
    }

    /// <summary>Adds a copy of the point <paramref name="PDS"/>; returns its index.</summary>
    internal int AddPoint(in TopOpeBRepDS_Point PDS)
    {
        myNbPoints++;
        myPoints.Bind(myNbPoints, new TopOpeBRepDS_PointData(PDS));
        return myNbPoints;
    }

    /// <summary>Adds a copy of the point <paramref name="PDS"/> (the shapes are ignored, as in OCCT); returns its index.</summary>
    internal int AddPointSS(in TopOpeBRepDS_Point PDS, in TopoDS_Shape S1, in TopoDS_Shape S2)
    {
        _ = S1;
        _ = S2;
        int i = AddPoint(PDS);
        return i;
    }

    /// <summary>Marks the point <paramref name="I"/> as not kept.</summary>
    internal void RemovePoint(int I)
    {
        ref TopOpeBRepDS_Point P = ref ChangePoint(I);
        P.ChangeKeep(false);
    }

    /// <summary>Whether the point <paramref name="I"/> is kept.</summary>
    public bool KeepPoint(int I)
    {
        TopOpeBRepDS_Point P = Point(I);
        bool b = P.Keep();
        return b;
    }

    /// <summary>Whether the point <paramref name="P"/> is kept.</summary>
    public bool KeepPoint(in TopOpeBRepDS_Point P)
    {
        bool b = P.Keep();
        return b;
    }

    internal void ChangeKeepPoint(int I, bool FindKeep)
    {
        ref TopOpeBRepDS_Point P = ref ChangePoint(I);
        P.ChangeKeep(FindKeep);
    }

    internal static void ChangeKeepPoint(ref TopOpeBRepDS_Point P, bool FindKeep)
    {
        P.ChangeKeep(FindKeep);
    }

    /// <summary>Adds the shape <paramref name="S"/> if it is new (its own same-domain reference); returns its index.</summary>
    internal int AddShape(in TopoDS_Shape S)
    {
        int iS = myShapes.FindIndex(S);
        if (iS == 0)
        {
            TopOpeBRepDS_ShapeData SD = new();
            iS = myShapes.Add(S, SD);
            // a shape is its own reference, oriented as itself
            SameDomainRef(iS, iS);
            SameDomainOri(iS, TopOpeBRepDS_SAMEORIENTED);
        }
        return iS;
    }

    /// <summary>
    /// Adds the shape <paramref name="S"/> if it is new, with the ancestor rank <paramref name="Ianc"/>; returns its
    /// index. The rank of a shape already there is not changed.
    /// </summary>
    internal int AddShape(in TopoDS_Shape S, int Ianc)
    {
        int iS = myShapes.FindIndex(S);
        if (iS == 0)
        {
            TopOpeBRepDS_ShapeData SD = new();
            iS = myShapes.Add(S, SD);
            // a shape is its own reference, oriented as itself
            SameDomainRef(iS, iS);
            SameDomainOri(iS, TopOpeBRepDS_SAMEORIENTED);
            AncestorRank(iS, Ianc);
        }
        return iS;
    }

    /// <summary>Whether the shape <paramref name="I"/> exists and (if <paramref name="FindKeep"/>) is kept.</summary>
    public bool KeepShape(int I, bool FindKeep = true)
    {
        bool b = false;
        if (I >= 1 && I <= myShapes.Extent())
        {
            TopoDS_Shape S = myShapes.FindKey(I);
            if (FindKeep)
            {
                b = KeepShape(S);
            }
            else
            {
                b = true;
            }
        }
        return b;
    }

    /// <summary>Whether the shape <paramref name="S"/> is in the data structure and (if <paramref name="FindKeep"/>) is kept.</summary>
    public bool KeepShape(in TopoDS_Shape S, bool FindKeep = true)
    {
        bool b = false;
        if ((!myShapes.IsEmpty()) && (!S.IsNull()) && myShapes.Contains(S))
        {
            TopOpeBRepDS_ShapeData SD = myShapes.FindFromKey(S);
            if (FindKeep)
            {
                b = SD.Keep();
            }
            else
            {
                b = true;
            }
        }
        return b;
    }

    internal void ChangeKeepShape(int I, bool FindKeep)
    {
        if (I >= 1 && I <= myShapes.Extent())
        {
            TopoDS_Shape S = myShapes.FindKey(I);
            ChangeKeepShape(S, FindKeep);
        }
    }

    internal void ChangeKeepShape(in TopoDS_Shape S, bool FindKeep)
    {
        if (!S.IsNull() && myShapes.Contains(S))
        {
            TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromKey(S);
            SD.ChangeKeep(FindKeep);
        }
    }

    internal void InitSectionEdges()
    {
        mySectionEdges.Clear();
    }

    /// <summary>Adds the section edge <paramref name="E"/> if it is new; returns its index among the section edges.</summary>
    internal int AddSectionEdge(in TopoDS_Edge E)
    {
        int iE = mySectionEdges.FindIndex(E);
        if (iE == 0)
        {
            iE = mySectionEdges.Add(E);
        }
        return iE;
    }

    /// <summary>The map of the shapes and their data.</summary>
    internal NCollection_IndexedDataMap<TopoDS_Shape, TopOpeBRepDS_ShapeData> ChangeShapes() => myShapes;

    /// <summary>The interferences of the surface <paramref name="I"/> (the shared empty list if there is none).</summary>
    public NCollection_List<TopOpeBRepDS_Interference> SurfaceInterferences(int I)
    {
        if (!mySurfaces.IsBound(I))
        {
            return myEmptyListOfInterference;
        }
        TopOpeBRepDS_SurfaceData SD = mySurfaces.Find(I);
        NCollection_List<TopOpeBRepDS_Interference> LI = SD.Interferences();
        return LI;
    }

    internal NCollection_List<TopOpeBRepDS_Interference> ChangeSurfaceInterferences(int I)
    {
        if (!mySurfaces.IsBound(I))
        {
            return myEmptyListOfInterference;
        }
        TopOpeBRepDS_SurfaceData SD = mySurfaces.ChangeFind(I);
        NCollection_List<TopOpeBRepDS_Interference> LI = SD.ChangeInterferences();
        return LI;
    }

    /// <summary>The interferences of the curve <paramref name="I"/> (the shared empty list if there is none).</summary>
    public NCollection_List<TopOpeBRepDS_Interference> CurveInterferences(int I)
    {
        if (!myCurves.IsBound(I))
        {
            return myEmptyListOfInterference;
        }
        TopOpeBRepDS_CurveData CD = myCurves.Find(I);
        NCollection_List<TopOpeBRepDS_Interference> LI = CD.Interferences();
        return LI;
    }

    internal NCollection_List<TopOpeBRepDS_Interference> ChangeCurveInterferences(int I)
    {
        if (!myCurves.IsBound(I))
        {
            return myEmptyListOfInterference;
        }
        TopOpeBRepDS_CurveData CD = myCurves.ChangeFind(I);
        NCollection_List<TopOpeBRepDS_Interference> LI = CD.ChangeInterferences();
        return LI;
    }

    /// <summary>The interferences of the point <paramref name="I"/> (the shared empty list if there is none).</summary>
    public NCollection_List<TopOpeBRepDS_Interference> PointInterferences(int I)
    {
        if (!myPoints.IsBound(I))
        {
            return myEmptyListOfInterference;
        }
        TopOpeBRepDS_PointData PD = myPoints.Find(I);
        NCollection_List<TopOpeBRepDS_Interference> LI = PD.Interferences();
        return LI;
    }

    internal NCollection_List<TopOpeBRepDS_Interference> ChangePointInterferences(int I)
    {
        if (!myPoints.IsBound(I))
        {
            return myEmptyListOfInterference;
        }
        TopOpeBRepDS_PointData PD = myPoints.ChangeFind(I);
        NCollection_List<TopOpeBRepDS_Interference> LI = PD.ChangeInterferences();
        return LI;
    }

    /// <summary>The interferences of the shape <paramref name="S"/> (the shared empty list if it isn't there or kept).</summary>
    public NCollection_List<TopOpeBRepDS_Interference> ShapeInterferences(in TopoDS_Shape S, bool FindKeep = true)
    {
        if (HasShape(S, FindKeep))
        {
            return myShapes.FindFromKey(S).myInterferences;
        }
        return myEmptyListOfInterference;
    }

    internal NCollection_List<TopOpeBRepDS_Interference> ChangeShapeInterferences(in TopoDS_Shape S)
    {
        if (!HasShape(S))
        {
            // OCCT: the shared empty list, for writing (decision 1066)
            return myEmptyListOfInterference;
        }
        TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromKey(S);
        return SD.myInterferences;
    }

    /// <summary>The interferences of the shape <paramref name="I"/> (the shared empty list if it isn't kept).</summary>
    public NCollection_List<TopOpeBRepDS_Interference> ShapeInterferences(int I, bool FindKeep = true)
    {
        if (FindKeep && !KeepShape(I))
        {
            return myEmptyListOfInterference;
        }
        return myShapes.FindFromIndex(I).myInterferences;
    }

    internal NCollection_List<TopOpeBRepDS_Interference> ChangeShapeInterferences(int I)
    {
        TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromIndex(I);
        return SD.myInterferences;
    }

    /// <summary>The shapes sharing the geometrical domain of <paramref name="S"/>.</summary>
    public NCollection_List<TopoDS_Shape> ShapeSameDomain(in TopoDS_Shape S)
    {
        if (!S.IsNull())
        {
            if (myShapes.Contains(S))
            {
                TopOpeBRepDS_ShapeData SD = myShapes.FindFromKey(S);
                NCollection_List<TopoDS_Shape> l = SD.mySameDomain;
                return l;
            }
        }
        return myEmptyListOfShape;
    }

    internal NCollection_List<TopoDS_Shape> ChangeShapeSameDomain(in TopoDS_Shape S)
    {
        TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromKey(S);
        return SD.mySameDomain;
    }

    /// <summary>The shapes sharing the geometrical domain of the shape <paramref name="I"/>.</summary>
    public NCollection_List<TopoDS_Shape> ShapeSameDomain(int I)
    {
        if (I >= 1 && I <= myShapes.Extent())
        {
            TopOpeBRepDS_ShapeData SD = myShapes.FindFromIndex(I);
            NCollection_List<TopoDS_Shape> l = SD.mySameDomain;
            return l;
        }
        else
        {
            return myEmptyListOfShape;
        }
    }

    internal NCollection_List<TopoDS_Shape> ChangeShapeSameDomain(int I)
    {
        TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromIndex(I);
        return SD.mySameDomain;
    }

    internal void AddShapeSameDomain(in TopoDS_Shape S, in TopoDS_Shape SSD)
    {
        bool append = true;
        {
            NCollection_List<TopoDS_Shape>.Iterator it = new(ShapeSameDomain(S));
            for (; it.More(); it.Next())
            {
                TopoDS_Shape itS = it.Value();
                if (itS.IsSame(SSD))
                {
                    append = false;
                    break;
                }
            }
        }
        if (append)
        {
            ChangeShapeSameDomain(S).Append(SSD);
        }
    }

    internal void RemoveShapeSameDomain(in TopoDS_Shape S, in TopoDS_Shape SSD)
    {
        NCollection_List<TopoDS_Shape> L = ChangeShapeSameDomain(S);
        NCollection_List<TopoDS_Shape>.Iterator it = new(L);
        while (it.More())
        {
            TopoDS_Shape itS = it.Value();
            bool remove = itS.IsSame(SSD);
            if (remove)
            {
                L.Remove(it);
            }
            else
            {
                it.Next();
            }
        }
    }

    /// <summary>The index of the same-domain reference of the shape <paramref name="I"/>, or 0.</summary>
    public int SameDomainRef(int I)
    {
        if (I >= 1 && I <= myShapes.Extent())
        {
            return myShapes.FindFromIndex(I).mySameDomainRef;
        }
        return 0;
    }

    /// <summary>The index of the same-domain reference of the shape <paramref name="S"/>, or 0.</summary>
    public int SameDomainRef(in TopoDS_Shape S)
    {
        if (S.IsNull())
        {
            return 0;
        }
        if (myShapes.Contains(S))
        {
            return myShapes.FindFromKey(S).mySameDomainRef;
        }
        return 0;
    }

    internal void SameDomainRef(int I, int Ref)
    {
        if (I >= 1 && I <= myShapes.Extent())
        {
            TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromIndex(I);
            SD.mySameDomainRef = Ref;
        }
    }

    internal void SameDomainRef(in TopoDS_Shape S, int Ref)
    {
        if (S.IsNull())
        {
            return;
        }
        if (myShapes.Contains(S))
        {
            TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromKey(S);
            SD.mySameDomainRef = Ref;
        }
    }

    /// <summary>How the shape <paramref name="I"/> lies with respect to its same-domain reference.</summary>
    public TopOpeBRepDS_Config SameDomainOri(int I)
    {
        if (I >= 1 && I <= myShapes.Extent())
        {
            return myShapes.FindFromIndex(I).mySameDomainOri;
        }
        return TopOpeBRepDS_UNSHGEOMETRY;
    }

    /// <summary>How the shape <paramref name="S"/> lies with respect to its same-domain reference.</summary>
    public TopOpeBRepDS_Config SameDomainOri(in TopoDS_Shape S)
    {
        if (!S.IsNull())
        {
            if (myShapes.Contains(S))
            {
                return myShapes.FindFromKey(S).mySameDomainOri;
            }
        }
        return TopOpeBRepDS_UNSHGEOMETRY;
    }

    internal void SameDomainOri(int I, TopOpeBRepDS_Config Ori)
    {
        if (I >= 1 && I <= myShapes.Extent())
        {
            TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromIndex(I);
            SD.mySameDomainOri = Ori;
        }
    }

    internal void SameDomainOri(in TopoDS_Shape S, TopOpeBRepDS_Config Ori)
    {
        if (!S.IsNull())
        {
            if (myShapes.Contains(S))
            {
                TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromKey(S);
                SD.mySameDomainOri = Ori;
            }
        }
    }

    /// <summary>The same-domain index (1 or 2: which argument) of the shape <paramref name="I"/>, or 0.</summary>
    public int SameDomainInd(int I)
    {
        if (I >= 1 && I <= myShapes.Extent())
        {
            return myShapes.FindFromIndex(I).mySameDomainInd;
        }
        return 0;
    }

    /// <summary>The same-domain index of the shape <paramref name="S"/>, or 0.</summary>
    public int SameDomainInd(in TopoDS_Shape S)
    {
        if (!S.IsNull())
        {
            if (myShapes.Contains(S))
            {
                return myShapes.FindFromKey(S).mySameDomainInd;
            }
        }
        return 0;
    }

    internal void SameDomainInd(int I, int Ind)
    {
        if (I >= 1 && I <= myShapes.Extent())
        {
            TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromIndex(I);
            SD.mySameDomainInd = Ind;
        }
    }

    internal void SameDomainInd(in TopoDS_Shape S, int Ind)
    {
        if (S.IsNull())
        {
            return;
        }
        if (myShapes.Contains(S))
        {
            TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromKey(S);
            SD.mySameDomainInd = Ind;
        }
    }

    /// <summary>The ancestor rank (the argument it comes from) of the shape <paramref name="I"/>, or 0.</summary>
    public int AncestorRank(int I)
    {
        if (I >= 1 && I <= myShapes.Extent())
        {
            return myShapes.FindFromIndex(I).myAncestorRank;
        }
        return 0;
    }

    /// <summary>The ancestor rank of the shape <paramref name="S"/>, or 0.</summary>
    public int AncestorRank(in TopoDS_Shape S)
    {
        if (S.IsNull())
        {
            return 0;
        }
        if (myShapes.Contains(S))
        {
            return myShapes.FindFromKey(S).myAncestorRank;
        }
        return 0;
    }

    internal void AncestorRank(int I, int Ianc)
    {
        if (I >= 1 && I <= myShapes.Extent())
        {
            TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromIndex(I);
            SD.myAncestorRank = Ianc;
        }
    }

    internal void AncestorRank(in TopoDS_Shape S, int Ianc)
    {
        if (S.IsNull())
        {
            return;
        }
        if (myShapes.Contains(S))
        {
            TopOpeBRepDS_ShapeData SD = myShapes.ChangeFromKey(S);
            SD.myAncestorRank = Ianc;
        }
    }

    /// <summary>Appends the interference <paramref name="I"/> to the shape's list (to the shared empty list if the shape isn't there).</summary>
    internal void AddShapeInterference(in TopoDS_Shape S, TopOpeBRepDS_Interference I)
    {
        ChangeShapeInterferences(S).Append(I);
    }

    internal void RemoveShapeInterference(in TopoDS_Shape S, TopOpeBRepDS_Interference I)
    {
        NCollection_List<TopOpeBRepDS_Interference> L = ChangeShapeInterferences(S);
        NCollection_List<TopOpeBRepDS_Interference>.Iterator it = new(L);
        bool b = FindInterference(it, I);
        if (b)
        {
            L.Remove(it);
        }
    }

    /// <summary>
    /// Records that <paramref name="S1"/> (of the first argument) and <paramref name="S2"/> (of the second) share
    /// their geometrical domain, and gives both the same reference.
    /// </summary>
    internal void FillShapesSameDomain(in TopoDS_Shape S1, in TopoDS_Shape S2, bool refFirst = true)
    {
        TopAbs_Orientation o1 = S1.Orientation();
        TopAbs_Orientation o2 = S2.Orientation();

        int iS1 = AddShape(S1, 1);
        TopOpeBRepDS_ShapeData SD1 = myShapes.ChangeFromIndex(iS1);
        bool isdef1 = SD1.myOrientationDef;
        bool todef1 = (!isdef1);
        if (isdef1 && SD1.mySameDomainOri == TopOpeBRepDS_UNSHGEOMETRY)
        {
            todef1 = true;
        }
        if (todef1)
        {
            SD1.myOrientation = o1;
            SD1.myOrientationDef = true;
        }

        int iS2 = AddShape(S2, 2);
        TopOpeBRepDS_ShapeData SD2 = myShapes.ChangeFromIndex(iS2);
        bool isdef2 = SD2.myOrientationDef;
        bool todef2 = (!isdef2);
        if (isdef2 && SD2.mySameDomainOri == TopOpeBRepDS_UNSHGEOMETRY)
        {
            todef2 = true;
        }
        if (todef2)
        {
            SD2.myOrientation = o2;
            SD2.myOrientationDef = true;
        }

        int n1 = ShapeSameDomain(S1).Extent();
        int n2 = ShapeSameDomain(S2).Extent();

        AddShapeSameDomain(S1, S2);
        AddShapeSameDomain(S2, S1);

        int n11 = ShapeSameDomain(S1).Extent();
        int n22 = ShapeSameDomain(S2).Extent();

        bool cond = (n11 == n1) && (n22 == n2);
        cond = cond && (!todef1) && (!todef2);
        if (cond)
        {
            // nothing changed in SameDomain data of S1 and S2 : return
            return;
        }

        int r1 = SameDomainRef(S1);
        int r2 = SameDomainRef(S2);
        int r = 0;
        // r1 == i1  r2 == i2 : shapes have no SameDomain ref : take S1 as reference
        // r1 == i1  r2 != i2 : S2 has a SameDomain reference : give it to S1
        // r1 != i1  r2 == i2 : S1 has a SameDomain reference : give it to S2
        // r1 != i1  r2 != i2 : S1,S2 have SameDomain reference : check equality
        if (r1 == iS1 && r2 == iS2)
        {
            r = (refFirst ? iS1 : iS2);
        }
        else if (r1 == iS1 && r2 != iS2)
        {
            r = r2;
        }
        else if (r1 != iS1 && r2 == iS2)
        {
            r = r1;
        }
        else if (r1 != iS1 && r2 != iS2)
        {
            if (r1 != r2)
            {
                //      throw Standard_ProgramError("FacesFiller::Insert SD 1");
            }
            r = (refFirst ? r1 : r2);
        }

        if (r == 0)
        {
            throw new Standard_ProgramError("FacesFiller::Insert SD 2");
        }

        TopoDS_Shape Sr = Shape(r);
        TopOpeBRepDS_ShapeData SD = myShapes.FindFromKey(Sr);
        TopAbs_Orientation oSr = SD.myOrientation;
        Sr.Orientation(oSr);

        if (r != r1 || todef1)
        { // S1 gets a new reference r
            TopOpeBRepDS_Config o = TopOpeBRepDS_SAMEORIENTED;
            if (r != iS1 || todef1)
            {
                bool sso = TopOpeBRepTool_ShapeTool.ShapesSameOriented(S1, Sr);
                if (!sso)
                {
                    o = TopOpeBRepDS_DIFFORIENTED;
                }
            }
            SameDomainRef(iS1, r);
            SameDomainOri(iS1, o);
        }

        if (r != r2 || todef2)
        { // S2 gets a new reference r
            TopOpeBRepDS_Config o = TopOpeBRepDS_SAMEORIENTED;
            if (r != iS2 || todef2)
            {
                bool sso = TopOpeBRepTool_ShapeTool.ShapesSameOriented(S2, Sr);
                if (!sso)
                {
                    o = TopOpeBRepDS_DIFFORIENTED;
                }
            }
            SameDomainRef(iS2, r);
            SameDomainOri(iS2, o);
        }

        // index
        SameDomainInd(S1, 1);
        SameDomainInd(S2, 2);
    }

    /// <summary>As the other overload, with the configurations <paramref name="c1"/> and <paramref name="c2"/> given.</summary>
    internal void FillShapesSameDomain(in TopoDS_Shape S1,
                                       in TopoDS_Shape S2,
                                       TopOpeBRepDS_Config c1,
                                       TopOpeBRepDS_Config c2,
                                       bool refFirst = true)
    {
        int iS1 = AddShape(S1, 1);
        TopOpeBRepDS_ShapeData SD1 = myShapes.ChangeFromIndex(iS1);
        bool isdef1 = SD1.myOrientationDef;
        bool todef1 = true;
        if (c1 == TopOpeBRepDS_UNSHGEOMETRY && isdef1)
        {
            todef1 = false;
        }

        int iS2 = AddShape(S2, 2);
        TopOpeBRepDS_ShapeData SD2 = myShapes.ChangeFromIndex(iS2);
        bool isdef2 = SD2.myOrientationDef;
        bool todef2 = true;
        if (c2 == TopOpeBRepDS_UNSHGEOMETRY && isdef2)
        {
            todef2 = false;
        }

        if (todef1 || todef2)
        {
            FillShapesSameDomain(S1, S2, refFirst);
        }

        if (todef1 && c1 == TopOpeBRepDS_UNSHGEOMETRY)
        {
            SameDomainOri(S1, TopOpeBRepDS_UNSHGEOMETRY);
        }

        if (todef2 && c2 == TopOpeBRepDS_UNSHGEOMETRY)
        {
            SameDomainOri(S2, TopOpeBRepDS_UNSHGEOMETRY);
        }
    }

    internal void UnfillShapesSameDomain(in TopoDS_Shape S1, in TopoDS_Shape S2)
    {
        RemoveShapeSameDomain(S1, S2);
        RemoveShapeSameDomain(S2, S1);
    }

    /// <summary>The number of surfaces added (removed ones included).</summary>
    public int NbSurfaces()
    {
        int n = myNbSurfaces;
        return n;
    }

    /// <summary>The number of curves added (removed ones included).</summary>
    public int NbCurves()
    {
        int n = myNbCurves;
        return n;
    }

    internal void ChangeNbCurves(int n)
    {
        myNbCurves = n;
    }

    /// <summary>The number of points added (removed ones included).</summary>
    public int NbPoints()
    {
        int n = myNbPoints;
        return n;
    }

    /// <summary>The number of shapes.</summary>
    public int NbShapes()
    {
        int n = myShapes.Extent();
        return n;
    }

    /// <summary>The number of section edges.</summary>
    public int NbSectionEdges()
    {
        int n = mySectionEdges.Extent();
        return n;
    }

    /// <summary>The surface <paramref name="I"/> (the data structure's empty surface if there is none).</summary>
    public TopOpeBRepDS_Surface Surface(int I)
    {
        if (mySurfaces.IsBound(I))
        {
            return mySurfaces[I].mySurface;
        }
        else
        {
            return myEmptySurface;
        }
    }

    /// <summary>The surface <paramref name="I"/> for writing (the data structure's empty surface if there is none).</summary>
    internal ref TopOpeBRepDS_Surface ChangeSurface(int I)
    {
        if (mySurfaces.IsBound(I))
        {
            return ref mySurfaces[I].mySurface;
        }
        else
        {
            return ref myEmptySurface;
        }
    }

    /// <summary>The curve <paramref name="I"/> (the data structure's empty curve if there is none).</summary>
    public TopOpeBRepDS_Curve Curve(int I)
    {
        if (myCurves.IsBound(I))
        {
            TopOpeBRepDS_CurveData CD = myCurves[I];
            TopOpeBRepDS_Curve C = CD.myCurve;
            return C;
        }
        else
        {
            return myEmptyCurve;
        }
    }

    /// <summary>The curve <paramref name="I"/> for writing (the data structure's empty curve if there is none).</summary>
    internal ref TopOpeBRepDS_Curve ChangeCurve(int I)
    {
        if (myCurves.IsBound(I))
        {
            TopOpeBRepDS_CurveData CD = myCurves.ChangeFind(I);
            return ref CD.myCurve;
        }
        return ref myEmptyCurve;
    }

    /// <summary>The point <paramref name="I"/>.</summary>
    /// <exception cref="Standard_ProgramError"><paramref name="I"/> is not between 1 and <see cref="NbPoints"/>.</exception>
    public TopOpeBRepDS_Point Point(int I)
    {
        if (I < 1 || I > myNbPoints)
        {
            throw new Standard_ProgramError("TopOpeBRepDS_DataStructure::Point");
        }

        if (myPoints.IsBound(I))
        {
            return myPoints[I].myPoint;
        }
        else
        {
            return myEmptyPoint;
        }
    }

    /// <summary>The point <paramref name="I"/> for writing.</summary>
    /// <exception cref="Standard_ProgramError"><paramref name="I"/> is not between 1 and <see cref="NbPoints"/>.</exception>
    internal ref TopOpeBRepDS_Point ChangePoint(int I)
    {
        if (I < 1 || I > myNbPoints)
        {
            throw new Standard_ProgramError("TopOpeBRepDS_DataStructure::Point");
        }

        if (myPoints.IsBound(I))
        {
            return ref myPoints[I].myPoint;
        }
        else
        {
            return ref myEmptyPoint;
        }
    }

    /// <summary>The shape <paramref name="I"/>, or a null shape if it isn't there or (if <paramref name="FindKeep"/>) kept.</summary>
    public TopoDS_Shape Shape(int I, bool FindKeep = true)
    {
        if (KeepShape(I, FindKeep))
        {
            TopoDS_Shape S = myShapes.FindKey(I);
            return S;
        }
        return myEmptyShape;
    }

    /// <summary>The index of the shape <paramref name="S"/>, or 0 if it isn't there or (if <paramref name="FindKeep"/>) kept.</summary>
    public int Shape(in TopoDS_Shape S, bool FindKeep = true)
    {
        int i = 0;
        bool hs = HasShape(S, FindKeep);
        if (hs)
        {
            i = myShapes.FindIndex(S);
        }
        return i;
    }

    /// <summary>The section edge <paramref name="I"/>, or a null edge if it isn't a kept shape of the data structure.</summary>
    public TopoDS_Edge SectionEdge(int I, bool FindKeep = true)
    {
        TopoDS_Shape S = mySectionEdges.FindKey(I);
        if (HasShape(S, FindKeep))
        {
            return TopoDS.Edge(S);
        }
        return TopoDS.Edge(myEmptyShape);
    }

    /// <summary>The index of the section edge <paramref name="S"/>, or 0.</summary>
    public int SectionEdge(in TopoDS_Edge S, bool FindKeep = true)
    {
        int i = 0;
        if (KeepShape(S, FindKeep))
        {
            i = mySectionEdges.FindIndex(S);
        }
        return i;
    }

    /// <summary>Whether <paramref name="S"/> is a section edge.</summary>
    public bool IsSectionEdge(in TopoDS_Edge S, bool FindKeep = true)
    {
        bool b = KeepShape(S, FindKeep);
        if (b)
        {
            b = mySectionEdges.Contains(S);
        }
        return b;
    }

    /// <summary>Whether the shape <paramref name="S"/> is in the data structure with at least one interference.</summary>
    public bool HasGeometry(in TopoDS_Shape S)
    {
        bool has = HasShape(S);
        if (has)
        {
            has = !ShapeInterferences(S).IsEmpty();
        }
        return has;
    }

    /// <summary>Whether the shape <paramref name="S"/> is in the data structure and (if <paramref name="FindKeep"/>) kept.</summary>
    public bool HasShape(in TopoDS_Shape S, bool FindKeep = true)
    {
        bool res = KeepShape(S, FindKeep);
        return res;
    }

    /// <summary>Sets the new surface of the face <paramref name="F"/>.</summary>
    internal void SetNewSurface(in TopoDS_Shape F, Geom_Surface S)
    {
        myNewSurface.Bind(F, S);
    }

    /// <summary>Whether the face <paramref name="F"/> has a new surface.</summary>
    public bool HasNewSurface(in TopoDS_Shape F)
    {
        bool b = myNewSurface.IsBound(F);
        return b;
    }

    /// <summary>The new surface of the face <paramref name="F"/>, or null.</summary>
    public Geom_Surface? NewSurface(in TopoDS_Shape F)
    {
        if (HasNewSurface(F))
        {
            return myNewSurface.Find(F);
        }
        return null;
    }

    private static bool FindInterference(NCollection_List<TopOpeBRepDS_Interference>.Iterator IT,
                                         TopOpeBRepDS_Interference I)
    {
        for (; IT.More(); IT.Next())
        {
            if (ReferenceEquals(IT.Value(), I))
            {
                return true;
            }
        }
        return false;
    }

    internal void Isfafa(bool isfafa)
    {
        myIsfafa = isfafa;
    }

    /// <summary>Whether the data structure comes from a face/face operation.</summary>
    public bool Isfafa() => myIsfafa;

    internal NCollection_IndexedDataMap<TopoDS_Shape, TopOpeBRepDS_ShapeWithState> ChangeMapOfShapeWithStateObj()
        => myMapOfShapeWithStateObj;

    internal NCollection_IndexedDataMap<TopoDS_Shape, TopOpeBRepDS_ShapeWithState> ChangeMapOfShapeWithStateTool()
        => myMapOfShapeWithStateTool;

    /// <summary>
    /// The map of shapes with state (the object's or the tool's) that holds <paramref name="aShape"/>; if neither does,
    /// <paramref name="aFlag"/> is false and the result is an empty map.
    /// </summary>
    /// <remarks>OCCT returns a function-scope static map, shared by every data structure; the port's is this one's (decision 1063).</remarks>
    internal NCollection_IndexedDataMap<TopoDS_Shape, TopOpeBRepDS_ShapeWithState> ChangeMapOfShapeWithState(in TopoDS_Shape aShape,
                                                                                                             out bool aFlag)
    {
        aFlag = true;

        if (myMapOfShapeWithStateObj.Contains(aShape))
        {
            return myMapOfShapeWithStateObj;
        }

        if (myMapOfShapeWithStateTool.Contains(aShape))
        {
            return myMapOfShapeWithStateTool;
        }

        aFlag = false;
        return myDummyMapOfShapeWithState;
    }

    /// <summary>The shape with state of <paramref name="aShape"/>, or an empty one.</summary>
    public TopOpeBRepDS_ShapeWithState GetShapeWithState(in TopoDS_Shape aShape)
    {
        if (myMapOfShapeWithStateObj.Contains(aShape))
        {
            return myMapOfShapeWithStateObj.FindFromKey(aShape);
        }
        else if (myMapOfShapeWithStateTool.Contains(aShape))
        {
            return myMapOfShapeWithStateTool.FindFromKey(aShape);
        }
        return myDummyShapeWithState;
    }

    internal NCollection_IndexedMap<TopoDS_Shape> ChangeMapOfRejectedShapesObj() => myMapOfRejectedShapesObj;

    internal NCollection_IndexedMap<TopoDS_Shape> ChangeMapOfRejectedShapesTool() => myMapOfRejectedShapesTool;
}
