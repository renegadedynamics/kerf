// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_ShapeSet.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_ShapeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The shapes to rebuild from: a list of shapes (wires or shells, kept as they are), a list of start elements (edges
/// or faces) and, for each sub-shape (vertex or edge) of the elements, the elements incident to it.
/// </summary>
/// <remarks>
/// <para>
/// A working object. The maps of shapes already added compare with IsEqual (OCCT's default hasher), the map of
/// sub-shapes with IsSame (TopTools_ShapeMapHasher). The checking of added shapes (<see cref="CheckShape(in TopoDS_Shape, bool)"/>)
/// is always off, as in OCCT (whose constructor turns it off, "temporary NYI").
/// </para>
/// <para>
/// The debugging members (<c>DumpName</c>, <c>DumpCheck</c>, <c>DumpSS</c>, <c>DumpBB</c>, <c>DEBName</c>,
/// <c>DEBNumber</c>, <c>SName</c>, <c>SNameori</c>) are left out (decision 493).
/// </para>
/// </remarks>
public class TopOpeBRepBuild_ShapeSet
{
    protected TopAbs_ShapeEnum myShapeType;
    protected TopAbs_ShapeEnum mySubShapeType;
    protected readonly TopOpeBRepTool_ShapeExplorer mySubShapeExplorer = new();
    protected readonly NCollection_List<TopoDS_Shape> myStartShapes = new();
    protected readonly NCollection_List<TopoDS_Shape>.Iterator myStartShapesIter = new();
    protected readonly NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mySubShapeMap =
        new(TopTools_ShapeMapHasher.Instance);
    protected readonly NCollection_List<TopoDS_Shape>.Iterator myIncidentShapesIter = new();
    protected readonly NCollection_List<TopoDS_Shape> myShapes = new();
    protected readonly NCollection_List<TopoDS_Shape>.Iterator myShapesIter = new();
    protected TopoDS_Shape myCurrentShape = new();
    protected readonly NCollection_List<TopoDS_Shape> myCurrentShapeNeighbours = new();
    protected readonly NCollection_IndexedMap<TopoDS_Shape> myOMSS = new(EqualityComparer<TopoDS_Shape>.Default);
    protected readonly NCollection_IndexedMap<TopoDS_Shape> myOMES = new(EqualityComparer<TopoDS_Shape>.Default);
    protected readonly NCollection_IndexedMap<TopoDS_Shape> myOMSH = new(EqualityComparer<TopoDS_Shape>.Default);
    protected bool myCheckShape;

    /// <summary>
    /// A shape set to build shapes connected by sub-shapes of type <paramref name="SubShapeType"/> (EDGE: faces into
    /// shells; VERTEX: edges into wires).
    /// </summary>
    /// <exception cref="ProgramErrorException">Another type.</exception>
    public TopOpeBRepBuild_ShapeSet(TopAbs_ShapeEnum SubShapeType, bool checkshape = true)
    {
        mySubShapeType = SubShapeType;
        myCheckShape = checkshape;
        if (SubShapeType == TopAbs_EDGE)
        {
            myShapeType = TopAbs_FACE;
        }
        else if (SubShapeType == TopAbs_VERTEX)
        {
            myShapeType = TopAbs_EDGE;
        }
        else
        {
            throw new ProgramErrorException("ShapeSet : bad ShapeType");
        }

        myCheckShape = false; // temporary NYI
    }

    /// <summary>Adds <paramref name="S"/> (a wire or a shell) to the shapes.</summary>
    public virtual void AddShape(in TopoDS_Shape S)
    {
        bool chk = CheckShape(S);
        if (!chk)
        {
            return;
        }
        ProcessAddShape(S);
    }

    /// <summary>Adds <paramref name="S"/> (an edge or a face) to the start elements, and to the elements.</summary>
    public virtual void AddStartElement(in TopoDS_Shape S)
    {
        bool chk = CheckShape(S);
        if (!chk)
        {
            return;
        }
        ProcessAddStartElement(S);
    }

    /// <summary>Adds <paramref name="S"/> to the elements incident to each of its sub-shapes.</summary>
    public virtual void AddElement(in TopoDS_Shape S)
    {
        bool chk = CheckShape(S);
        if (!chk)
        {
            return;
        }
        ProcessAddElement(S);
    }

    protected void ProcessAddShape(in TopoDS_Shape S)
    {
        if (!myOMSH.Contains(S))
        {
            myOMSH.Add(S);
            myShapes.Append(S);
        }
    }

    protected void ProcessAddStartElement(in TopoDS_Shape S)
    {
        if (!myOMSS.Contains(S))
        {
            myOMSS.Add(S);
            myStartShapes.Append(S);
            ProcessAddElement(S);
        }
    }

    protected void ProcessAddElement(in TopoDS_Shape S)
    {
        if (!myOMES.Contains(S))
        {
            myOMES.Add(S);
            TopOpeBRepTool_ShapeExplorer Ex = new(S, mySubShapeType);
            for (; Ex.More(); Ex.Next())
            {
                TopoDS_Shape subshape = Ex.Current();
                bool b = !mySubShapeMap.Contains(subshape);
                if (b)
                {
                    // OCCT copies an empty list into the map
                    mySubShapeMap.Add(subshape, new NCollection_List<TopoDS_Shape>());
                }
                mySubShapeMap.ChangeFromKey(subshape).Append(S);
            }
        }
    }

    /// <summary>The start elements.</summary>
    public NCollection_List<TopoDS_Shape> StartElements() => myStartShapes;

    /// <summary>Starts the iteration on the shapes.</summary>
    public void InitShapes()
    {
        myShapesIter.Initialize(myShapes);
    }

    /// <summary>Whether there is a current shape.</summary>
    public bool MoreShapes()
    {
        bool b = myShapesIter.More();
        return b;
    }

    /// <summary>Moves to the next shape.</summary>
    public void NextShape()
    {
        myShapesIter.Next();
    }

    /// <summary>The current shape.</summary>
    public TopoDS_Shape Shape()
    {
        TopoDS_Shape S = myShapesIter.Value();
        return S;
    }

    /// <summary>Starts the iteration on the start elements.</summary>
    public void InitStartElements()
    {
        myStartShapesIter.Initialize(myStartShapes);
    }

    /// <summary>Whether there is a current start element.</summary>
    public bool MoreStartElements()
    {
        bool b = myStartShapesIter.More();
        return b;
    }

    /// <summary>Moves to the next start element.</summary>
    public void NextStartElement()
    {
        myStartShapesIter.Next();
    }

    /// <summary>The current start element.</summary>
    public TopoDS_Shape StartElement()
    {
        TopoDS_Shape S = myStartShapesIter.Value();
        return S;
    }

    /// <summary>Starts the iteration on the neighbours of the element <paramref name="S"/>.</summary>
    public virtual void InitNeighbours(in TopoDS_Shape S)
    {
        mySubShapeExplorer.Init(S, mySubShapeType);
        myCurrentShape = S;
        FindNeighbours();
    }

    /// <summary>Whether there is a current neighbour.</summary>
    public bool MoreNeighbours()
    {
        bool b = myIncidentShapesIter.More();
        return b;
    }

    /// <summary>Moves to the next neighbour.</summary>
    public void NextNeighbour()
    {
        myIncidentShapesIter.Next();
        bool noisimore = !myIncidentShapesIter.More();
        if (noisimore)
        {
            bool ssemore = mySubShapeExplorer.More();
            if (ssemore)
            {
                mySubShapeExplorer.Next();
                FindNeighbours();
            }
        }
    }

    /// <summary>The current neighbour.</summary>
    public TopoDS_Shape Neighbour()
    {
        TopoDS_Shape S = myIncidentShapesIter.Value();
        return S;
    }

    /// <summary>The start elements, to change.</summary>
    public NCollection_List<TopoDS_Shape> ChangeStartShapes() => myStartShapes;

    /// <summary>
    /// Finds the next sub-shape of the current element that has neighbours, and starts the iteration on them.
    /// </summary>
    public virtual void FindNeighbours()
    {
        while (mySubShapeExplorer.More())
        {
            // l = list of edges neighbour of edge myCurrentShape through
            // the vertex mySubShapeExplorer.Current(), which is a vertex of the
            // edge myCurrentShape.
            TopoDS_Shape V = mySubShapeExplorer.Current();
            NCollection_List<TopoDS_Shape> l = MakeNeighboursList(myCurrentShape, V);

            // myIncidentShapesIter iterates on the neighbour edges of the edge
            // given as InitNeighbours() argument (this edge has been stored
            // in the field myCurrentShape).

            myIncidentShapesIter.Initialize(l);
            if (myIncidentShapesIter.More())
            {
                break;
            }
            else
            {
                mySubShapeExplorer.Next();
            }
        }
    }

    /// <summary>The elements incident to the sub-shape <paramref name="V"/> of <paramref name="E"/>.</summary>
    public virtual NCollection_List<TopoDS_Shape> MakeNeighboursList(in TopoDS_Shape E, in TopoDS_Shape V)
    {
        NCollection_List<TopoDS_Shape> l = mySubShapeMap.FindFromKey(V);
        return l;
    }

    /// <summary>The largest number of elements incident to one sub-shape of <paramref name="Shape"/>.</summary>
    public int MaxNumberSubShape(in TopoDS_Shape Shape)
    {
        int i, m = 0;
        TopOpeBRepTool_ShapeExplorer SE = new(Shape, mySubShapeType);
        NCollection_List<TopoDS_Shape>.Iterator LI = new();
        while (SE.More())
        {
            TopoDS_Shape SubShape = SE.Current();
            if (!mySubShapeMap.Contains(SubShape))
            {
                SE.Next();
                continue;
            }
            NCollection_List<TopoDS_Shape> l = mySubShapeMap.FindFromKey(SubShape);
            LI.Initialize(l);
            for (i = 0; LI.More(); LI.Next(), i++)
            {
            }
            m = Max(m, i);
            SE.Next();
        }
        return m;
    }

    /// <summary>Turns the checking of added shapes on or off.</summary>
    public void CheckShape(bool checkshape)
    {
        myCheckShape = checkshape;
    }

    /// <summary>Whether added shapes are checked.</summary>
    public bool CheckShape() => myCheckShape;

    /// <summary>Whether <paramref name="S"/> is valid (BRepCheck), or true when checking is off.</summary>
    public bool CheckShape(in TopoDS_Shape S, bool checkgeom = false)
    {
        if (!myCheckShape)
        {
            return true;
        }

        BRepCheck_Analyzer ana = new(S, checkgeom);
        bool val = ana.IsValid();
        return val;
    }
}
