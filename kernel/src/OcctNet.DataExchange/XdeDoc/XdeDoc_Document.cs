// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port; its methods
// follow XCAF's tools (src/DataExchange/TKXCAF/XCAFDoc/XCAFDoc_ShapeTool, _ColorTool, _LayerTool, _DocumentTool)
// so that code ported from STEPCAFControl and the XDE Draw commands can call them in place of the tools'.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using System.Globalization;
using System.Text;

namespace Occt;

/// <summary>
/// A mutable assembly document with XCAF's label interface: what STEPCAFControl's reader fills and its writer reads,
/// in place of a <c>TDocStd_Document</c> with XCAF's tools (decision 1231). <see cref="ToTree"/> freezes it into an
/// <see cref="XdeDoc_Tree"/>; <see cref="FromTree"/> opens a tree.
/// </summary>
/// <remarks>
/// <para>
/// The labels are laid out as XCAF's: the shapes under 0:1:1 (top-level shapes, with an assembly's components and a
/// part's sub-shapes as their children), the layers under 0:1:3. Each method has the name and the behaviour of the
/// XCAF tool method it replaces (<c>XCAFDoc_ShapeTool</c>: <see cref="AddShape"/>, <see cref="NewShape"/>,
/// <see cref="AddComponent(in XdeDoc_Label, in XdeDoc_Label, in TopLoc_Location)"/>, <see cref="Search"/>,
/// <see cref="SearchUsingMap"/>, <see cref="FindShape(in TopoDS_Shape, out XdeDoc_Label, bool)"/>,
/// <see cref="AddSubShape(in XdeDoc_Label, in TopoDS_Shape)"/>, <see cref="GetFreeShapes"/>, <see cref="GetComponents"/>,
/// <see cref="GetReferredShape"/>, <see cref="GetLocation"/>, the <c>Is...</c> tests...; <c>XCAFDoc_ColorTool</c>:
/// <see cref="SetColor(in XdeDoc_Label, in Quantity_ColorRGBA, XdeDoc_ColorType)"/>, <see cref="GetColor(in XdeDoc_Label, XdeDoc_ColorType, out Quantity_ColorRGBA)"/>,
/// <see cref="SetVisibility"/>, <see cref="IsVisible"/>; <c>XCAFDoc_LayerTool</c>: <see cref="AddLayer(string)"/>,
/// <see cref="SetLayer(in XdeDoc_Label, in XdeDoc_Label, bool)"/>, <see cref="GetLayers(in XdeDoc_Label, NCollection_Sequence{string})"/>...;
/// <c>TDataStd_Name</c>: <see cref="SetName"/>, <see cref="GetName"/>; the validation properties
/// (<c>XCAFDoc_Volume</c>, <c>_Area</c>, <c>_Centroid</c>) and the length unit (<c>XCAFDoc_DocumentTool</c>).
/// </para>
/// <para>
/// Left out: SHUOs (styles of an instance of a sub-assembly's component), colour labels (colours are values here),
/// materials, GD&amp;T, views, clipping planes, notes and named data, OCAF's undo and storage. <c>AddShape</c>'s
/// <c>makePrepare</c> (which gives components with an identity location a unit scaling, modifying the compound) is
/// accepted and ignored. Not thread-safe; a tree made from it is.
/// </para>
/// </remarks>
public sealed class XdeDoc_Document
{
    private const int MainIndex = 0;
    private const int ShapesIndex = 1;
    private const int ColorsIndex = 2;
    private const int LayersIndex = 3;

    private sealed class Node
    {
        public int Father = -1;
        public int Tag;
        public readonly List<int> Children = [];
        public int NextTag = 1;

        public TopoDS_Shape Shape;           // the shape set on the label (none for a reference)
        public bool IsAssembly;
        public bool IsDirty;                 // an assembly whose compound must be remade from its components
        public int Referred = -1;            // a reference's referred label
        public TopLoc_Location RefLocation;  // and its location
        public readonly List<int> Users = []; // the references to this label, in order

        public XdeDoc_Attributes Attributes = XdeDoc_Attributes.Empty; // all but the layers
        public bool HasName;
        public List<int>? Layers;            // layer labels (a shape's), in order
        public List<int>? LayerShapes;       // shape labels (a layer's), in order

        public NCollection_IndexedMap<TopoDS_Shape>? SubShapeMap; // XCAFDoc_ShapeMapTool's map
        public Dictionary<TopoDS_Shape, int>? SubShapeLabels;      // sub-shape children by shape
    }

    private readonly List<Node> myNodes = [];
    private readonly Dictionary<TopoDS_Shape, int> myTopLevelShapes = new(TopTools_ShapeMapHasher.Instance);
    private readonly Dictionary<TopoDS_Shape, int> myShapeLabels = new(TopTools_ShapeMapHasher.Instance);
    private readonly Dictionary<TopoDS_Shape, int> mySubShapes = new(TopTools_ShapeMapHasher.Instance);

    /// <summary>An empty document.</summary>
    public XdeDoc_Document()
    {
        myNodes.Add(new Node { Tag = 1 });   // 0:1, the main label
        newChild(MainIndex);                  // 0:1:1, the shapes
        newChild(MainIndex);                  // 0:1:2, the colours (colours are values here: no child)
        newChild(MainIndex);                  // 0:1:3, the layers
    }

    /// <summary>The length unit, or null if not set (<c>XCAFDoc_DocumentTool::SetLengthUnit</c>).</summary>
    public XdeDoc_LengthUnit? LengthUnit { get; private set; }

    /// <summary>Sets the length unit.</summary>
    public void SetLengthUnit(XdeDoc_LengthUnit? theUnit) => LengthUnit = theUnit;

    /// <summary>Sets the length unit by its size in metres (<c>XCAFDoc_DocumentTool::SetLengthUnit</c>).</summary>
    public void SetLengthUnit(double theUnitValue, string theName = "")
        => LengthUnit = new XdeDoc_LengthUnit(theName, theUnitValue);

    /// <summary>The length unit's size in metres, false if not set (<c>XCAFDoc_DocumentTool::GetLengthUnit</c>).</summary>
    public bool GetLengthUnit(out double theResult)
    {
        theResult = LengthUnit?.UnitValue ?? 0.0;
        return LengthUnit is not null;
    }

    // ---------------------------------------------------------------------------------------------- labels

    /// <summary>The document's main label (0:1).</summary>
    public XdeDoc_Label Main() => new(this, MainIndex);

    /// <summary>The label of the shapes (0:1:1), XCAF's <c>XCAFDoc_ShapeTool::Label()</c>.</summary>
    public XdeDoc_Label ShapesLabel() => new(this, ShapesIndex);

    /// <summary>The label of the layers (0:1:3), XCAF's <c>XCAFDoc_LayerTool::Label()</c>.</summary>
    public XdeDoc_Label LayersLabel() => new(this, LayersIndex);

    /// <summary>The label of the entry ("0:1:1:2"), or the null label if there is none.</summary>
    public XdeDoc_Label FindLabel(string theEntry)
    {
        string[] aParts = theEntry.Split(':');
        if (aParts.Length < 2 || aParts[0] != "0" || aParts[1] != "1")
        {
            return default;
        }
        int aCurrent = MainIndex;
        for (int i = 2; i < aParts.Length; i++)
        {
            if (!int.TryParse(aParts[i], NumberStyles.None, CultureInfo.InvariantCulture, out int aTag))
            {
                return default;
            }
            int aFound = -1;
            foreach (int aChild in myNodes[aCurrent].Children)
            {
                if (myNodes[aChild].Tag == aTag)
                {
                    aFound = aChild;
                    break;
                }
            }
            if (aFound < 0)
            {
                return default;
            }
            aCurrent = aFound;
        }
        return new XdeDoc_Label(this, aCurrent);
    }

    /// <summary>The label's children, in order.</summary>
    public IReadOnlyList<XdeDoc_Label> Children(in XdeDoc_Label theLabel)
        => node(theLabel).Children.Select(i => new XdeDoc_Label(this, i)).ToList();

    internal XdeDoc_Label FatherOf(in XdeDoc_Label theLabel)
    {
        int aFather = node(theLabel).Father;
        return aFather < 0 ? default : new XdeDoc_Label(this, aFather);
    }

    internal int TagOf(in XdeDoc_Label theLabel) => node(theLabel).Tag;

    internal string EntryOf(in XdeDoc_Label theLabel)
    {
        var aTags = new List<int>();
        for (int i = index(theLabel); i >= 0; i = myNodes[i].Father)
        {
            aTags.Add(myNodes[i].Tag);
        }
        var aBuilder = new StringBuilder("0");
        for (int i = aTags.Count - 1; i >= 0; i--)
        {
            aBuilder.Append(':').Append(aTags[i].ToString(CultureInfo.InvariantCulture));
        }
        return aBuilder.ToString();
    }

    private int index(in XdeDoc_Label theLabel)
    {
        if (!ReferenceEquals(theLabel.Document, this))
        {
            throw new ArgumentException(theLabel.IsNull() ? "Null label" : "Label of another document", nameof(theLabel));
        }
        return theLabel.Index;
    }

    private Node node(in XdeDoc_Label theLabel) => myNodes[index(theLabel)];

    private bool isMine(in XdeDoc_Label theLabel) => ReferenceEquals(theLabel.Document, this);

    private XdeDoc_Label label(int theIndex) => theIndex < 0 ? default : new XdeDoc_Label(this, theIndex);

    private int newChild(int theFather)
    {
        Node aFather = myNodes[theFather];
        var aNode = new Node { Father = theFather, Tag = aFather.NextTag++ };
        myNodes.Add(aNode);
        int anIndex = myNodes.Count - 1;
        aFather.Children.Add(anIndex);
        return anIndex;
    }

    // ---------------------------------------------------------------------------------------------- shape tool: tests

    /// <summary>True if the label is a top-level shape label (a child of 0:1:1).</summary>
    public bool IsTopLevel(in XdeDoc_Label theLabel) => isMine(theLabel) && node(theLabel).Father == ShapesIndex;

    /// <summary>True if no reference refers to the label.</summary>
    public bool IsFree(in XdeDoc_Label theLabel) => !isMine(theLabel) || node(theLabel).Users.Count == 0;

    /// <summary>True for a simple shape, an assembly or a reference.</summary>
    public bool IsShape(in XdeDoc_Label theLabel) => IsSimpleShape(theLabel) || IsAssembly(theLabel) || IsReference(theLabel);

    /// <summary>True for a label holding a shape that is neither an assembly nor a reference.</summary>
    public bool IsSimpleShape(in XdeDoc_Label theLabel)
    {
        if (!isMine(theLabel))
        {
            return false;
        }
        Node aNode = node(theLabel);
        return !aNode.Shape.IsNull() && !aNode.IsAssembly && aNode.Referred < 0;
    }

    /// <summary>True for a reference (a component, or a located top-level shape).</summary>
    public bool IsReference(in XdeDoc_Label theLabel) => isMine(theLabel) && node(theLabel).Referred >= 0;

    /// <summary>True for an assembly.</summary>
    public bool IsAssembly(in XdeDoc_Label theLabel) => isMine(theLabel) && node(theLabel).IsAssembly;

    /// <summary>True for a component: a reference whose father is an assembly.</summary>
    public bool IsComponent(in XdeDoc_Label theLabel) => IsReference(theLabel) && IsAssembly(theLabel.Father());

    /// <summary>True for a compound: XCAF tests the name "COMPOUND" (<c>XCAFDoc_ShapeTool::IsCompound</c>).</summary>
    public bool IsCompound(in XdeDoc_Label theLabel) => GetName(theLabel, out string? aName) && aName == "COMPOUND";

    /// <summary>True for a sub-shape label: a simple shape whose father is a shape.</summary>
    public bool IsSubShape(in XdeDoc_Label theLabel) => IsSimpleShape(theLabel) && IsShape(theLabel.Father());

    /// <summary>True if the shape is a sub-shape of the label's shape (with its location there), or the shape itself.</summary>
    public bool IsSubShape(in XdeDoc_Label theShapeLabel, in TopoDS_Shape theSub)
    {
        NCollection_IndexedMap<TopoDS_Shape>? aMap = subShapeMap(index(theShapeLabel));
        return aMap is not null && aMap.Contains(theSub);
    }

    /// <summary>True if the label refers to external files (<c>XCAFDoc_ShapeTool::IsExternRef</c>).</summary>
    public bool IsExternRef(in XdeDoc_Label theLabel) => isMine(theLabel) && node(theLabel).Attributes.ExternRefs.Length > 0;

    private NCollection_IndexedMap<TopoDS_Shape>? subShapeMap(int theIndex)
    {
        Node aNode = myNodes[theIndex];
        if (aNode.SubShapeMap is null)
        {
            TopoDS_Shape aShape = getShape(theIndex);
            if (aShape.IsNull())
            {
                return null;
            }
            aNode.SubShapeMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            addSubShapes(aShape, aNode.SubShapeMap);
        }
        return aNode.SubShapeMap;
    }

    // XCAFDoc_ShapeMapTool's UpdateMap: the shape and its sub-shapes, with their locations in it
    private static void addSubShapes(in TopoDS_Shape theShape, NCollection_IndexedMap<TopoDS_Shape> theMap)
    {
        var aStack = new Stack<TopoDS_Shape>();
        aStack.Push(theShape);
        while (aStack.Count > 0)
        {
            TopoDS_Shape aShape = aStack.Pop();
            theMap.Add(aShape);
            var aChildren = new List<TopoDS_Shape>();
            for (var anIt = new TopoDS_Iterator(aShape); anIt.More(); anIt.Next())
            {
                aChildren.Add(anIt.Value());
            }
            for (int i = aChildren.Count - 1; i >= 0; i--)
            {
                aStack.Push(aChildren[i]);
            }
        }
    }

    // ---------------------------------------------------------------------------------------------- shape tool: shapes

    /// <summary>
    /// The label's shape: its own, or for a reference the referred shape moved to the reference's location, or for an
    /// assembly the compound of its components (remade when components were added). False if it has none.
    /// </summary>
    public bool GetShape(in XdeDoc_Label theLabel, out TopoDS_Shape theShape)
    {
        theShape = isMine(theLabel) ? getShape(theLabel.Index) : new TopoDS_Shape();
        return !theShape.IsNull();
    }

    /// <summary>The label's shape, or a null shape.</summary>
    public TopoDS_Shape GetShape(in XdeDoc_Label theLabel)
    {
        GetShape(theLabel, out TopoDS_Shape aShape);
        return aShape;
    }

    private TopoDS_Shape getShape(int theIndex) => getShape(theIndex, 0);

    private TopoDS_Shape getShape(int theIndex, int theDepth)
    {
        if (theDepth > myNodes.Count)
        {
            throw new InvalidOperationException("XdeDoc_Document: a cycle of references");
        }
        Node aNode = myNodes[theIndex];
        if (aNode.Referred >= 0)
        {
            TopoDS_Shape aReferred = getShape(aNode.Referred, theDepth + 1);
            return aReferred.IsNull() || aNode.RefLocation.IsIdentity() ? aReferred : aReferred.Moved(aNode.RefLocation);
        }
        if (aNode.IsAssembly && aNode.IsDirty)
        {
            var aBuilder = new BRep_Builder();
            TopoDS_Compound aCompound = new();
            aBuilder.MakeCompound(ref aCompound);
            foreach (int aChild in aNode.Children)
            {
                if (myNodes[aChild].Referred >= 0 && aNode.IsAssembly)
                {
                    TopoDS_Shape aSub = getShape(aChild, theDepth + 1);
                    if (!aSub.IsNull())
                    {
                        aBuilder.Add(aCompound, aSub);
                    }
                }
            }
            aCompound.TShape()!.FreezeGraph();
            aNode.IsDirty = false;
            setOwnShape(theIndex, aCompound);
        }
        return aNode.Shape;
    }

    // Sets a label's own shape, indexing it; assemblies using the label must remake their compounds
    private void setOwnShape(int theIndex, in TopoDS_Shape theShape)
    {
        Node aNode = myNodes[theIndex];
        aNode.Shape = theShape;
        aNode.SubShapeMap = null;
        if (aNode.Father == ShapesIndex && !theShape.IsNull())
        {
            myTopLevelShapes.TryAdd(theShape, theIndex);
        }
        markUsersDirty(theIndex);
    }

    private void markUsersDirty(int theIndex)
    {
        var aStack = new Stack<int>();
        aStack.Push(theIndex);
        var aSeen = new HashSet<int>();
        while (aStack.Count > 0)
        {
            int anIndex = aStack.Pop();
            foreach (int aUser in myNodes[anIndex].Users)
            {
                int anAssembly = myNodes[aUser].Father;
                if (anAssembly >= 0 && myNodes[anAssembly].IsAssembly && aSeen.Add(anAssembly))
                {
                    myNodes[anAssembly].IsDirty = true;
                    aStack.Push(anAssembly);
                }
                if (aSeen.Add(-1 - aUser))
                {
                    aStack.Push(aUser);
                }
            }
        }
    }

    /// <summary>
    /// The top-level label of the shape (with its location if <paramref name="findInstance"/>, else without), false if
    /// there is none (<c>XCAFDoc_ShapeTool::FindShape</c>).
    /// </summary>
    public bool FindShape(in TopoDS_Shape theShape, out XdeDoc_Label theLabel, bool findInstance = false)
    {
        theLabel = default;
        if (theShape.IsNull())
        {
            return false;
        }
        TopoDS_Shape aS0 = findInstance ? theShape : theShape.Located(new TopLoc_Location());
        if (myTopLevelShapes.TryGetValue(aS0, out int anIndex) && getShape(anIndex).IsSame(aS0))
        {
            theLabel = label(anIndex);
            return true;
        }
        // the shapes held may have changed: XCAF's search label by label
        foreach (int aChild in myNodes[ShapesIndex].Children)
        {
            if (getShape(aChild).IsSame(aS0))
            {
                theLabel = label(aChild);
                return true;
            }
        }
        return false;
    }

    /// <summary>The top-level label of the shape, or the null label.</summary>
    public XdeDoc_Label FindShape(in TopoDS_Shape theShape, bool findInstance = false)
        => FindShape(theShape, out XdeDoc_Label aLabel, findInstance) ? aLabel : default;

    /// <summary>
    /// Searches the shape (<c>XCAFDoc_ShapeTool::Search</c>): if located, among the top-level instances and the
    /// assemblies' components; then among the top-level shapes; then as a sub-shape of a top-level simple shape, adding
    /// a sub-shape label for it.
    /// </summary>
    public bool Search(in TopoDS_Shape theShape,
                       out XdeDoc_Label theLabel,
                       bool findInstance = true,
                       bool findComponent = true,
                       bool findSubShape = true)
    {
        theLabel = default;
        if (theShape.IsNull())
        {
            return false;
        }
        if (!theShape.Location().IsIdentity())
        {
            if (findInstance && FindShape(theShape, out theLabel, true))
            {
                return true;
            }
            if (findComponent)
            {
                foreach (int aTop in myNodes[ShapesIndex].Children)
                {
                    if (!myNodes[aTop].IsAssembly)
                    {
                        continue;
                    }
                    foreach (int aComp in myNodes[aTop].Children)
                    {
                        if (myNodes[aComp].Referred >= 0 && getShape(aComp).IsSame(theShape))
                        {
                            theLabel = label(aComp);
                            return true;
                        }
                    }
                }
            }
        }
        if (FindShape(theShape, out theLabel, false))
        {
            return true;
        }
        if (!findSubShape)
        {
            return false;
        }
        XdeDoc_Label aMain = FindMainShape(theShape);
        if (aMain.IsNull())
        {
            return false;
        }
        theLabel = AddSubShape(aMain, theShape);
        return !theLabel.IsNull();
    }

    /// <summary>
    /// Searches the shape through the map of shapes added (<c>XCAFDoc_ShapeTool::SearchUsingMap</c>): the shape's own
    /// label, or a user of its unlocated shape's label holding it, or (if <paramref name="findWithoutLoc"/>) the
    /// unlocated shape's label; then as a sub-shape, through the map of sub-shapes, adding a sub-shape label.
    /// </summary>
    public bool SearchUsingMap(in TopoDS_Shape theShape, out XdeDoc_Label theLabel, bool findWithoutLoc, bool findSubShape)
    {
        theLabel = default;
        if (theShape.IsNull())
        {
            return false;
        }
        if (myShapeLabels.TryGetValue(theShape, out int anIndex))
        {
            theLabel = label(anIndex);
            return true;
        }
        TopoDS_Shape aS0 = theShape.Located(new TopLoc_Location());
        if (myShapeLabels.TryGetValue(aS0, out int anIndex1))
        {
            var aUsers = new NCollection_Sequence<XdeDoc_Label>();
            if (GetUsers(label(anIndex1), aUsers, true) > 0)
            {
                foreach (XdeDoc_Label aUser in aUsers)
                {
                    if (GetShape(aUser).IsSame(theShape))
                    {
                        theLabel = aUser;
                        return true;
                    }
                }
            }
            if (findWithoutLoc)
            {
                theLabel = label(anIndex1);
                return true;
            }
        }
        if (!findSubShape)
        {
            return false;
        }
        XdeDoc_Label aMain = FindMainShapeUsingMap(theShape);
        if (aMain.IsNull())
        {
            return false;
        }
        theLabel = AddSubShape(aMain, theShape);
        return !theLabel.IsNull();
    }

    /// <summary>The top-level simple shape having the shape as a sub-shape, through the map of sub-shapes added.</summary>
    public XdeDoc_Label FindMainShapeUsingMap(in TopoDS_Shape theSub)
        => mySubShapes.TryGetValue(theSub, out int anIndex) ? label(anIndex) : default;

    /// <summary>The first top-level simple shape having the shape as a sub-shape, or the null label.</summary>
    public XdeDoc_Label FindMainShape(in TopoDS_Shape theSub)
    {
        foreach (int aTop in myNodes[ShapesIndex].Children)
        {
            if (IsSimpleShape(label(aTop)) && IsSubShape(label(aTop), theSub))
            {
                return label(aTop);
            }
        }
        return default;
    }

    /// <summary>A new top-level label holding an empty compound (<c>XCAFDoc_ShapeTool::NewShape</c>).</summary>
    public XdeDoc_Label NewShape()
    {
        int anIndex = newChild(ShapesIndex);
        var aBuilder = new BRep_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aCompound.TShape()!.FreezeGraph();
        setOwnShape(anIndex, aCompound);
        return label(anIndex);
    }

    /// <summary>Sets the label's shape (frozen here, if it isn't yet), as <c>XCAFDoc_ShapeTool::SetShape</c>.</summary>
    public void SetShape(in XdeDoc_Label theLabel, in TopoDS_Shape theShape)
    {
        int anIndex = index(theLabel);
        theShape.TShape()?.FreezeGraph();
        myNodes[anIndex].IsDirty = false;
        setOwnShape(anIndex, theShape);
        myShapeLabels.TryAdd(theShape, anIndex);
    }

    /// <summary>
    /// Adds the shape (frozen here, if it isn't yet) at the top level, or finds its label (<c>XCAFDoc_ShapeTool::AddShape</c>):
    /// a located shape becomes a reference to its unlocated shape's label; a compound becomes an assembly of its
    /// sub-shapes if <paramref name="makeAssembly"/>.
    /// </summary>
    public XdeDoc_Label AddShape(in TopoDS_Shape theShape, bool makeAssembly = true, bool makePrepare = true)
    {
        _ = makePrepare; // decision 1231: the compound isn't modified
        if (!theShape.IsNull())
        {
            theShape.TShape()?.FreezeGraph();
        }
        int anIndex = addShape(theShape, makeAssembly);
        if (anIndex >= 0)
        {
            myShapeLabels.TryAdd(theShape, anIndex);
        }
        return label(anIndex);
    }

    private int addShape(in TopoDS_Shape theShape, bool makeAssembly)
    {
        if (theShape.IsNull())
        {
            return -1;
        }
        if (FindShape(theShape, out XdeDoc_Label aFound, true))
        {
            return aFound.Index;
        }
        int aShapeIndex = newChild(ShapesIndex);
        if (!theShape.Location().IsIdentity())
        {
            int aReferred = addShape(theShape.Located(new TopLoc_Location()), makeAssembly);
            makeReference(aShapeIndex, aReferred, theShape.Location());
            return aShapeIndex;
        }

        setOwnShape(aShapeIndex, theShape);
        if (makeAssembly && theShape.ShapeType() == TopAbs_ShapeEnum.TopAbs_COMPOUND)
        {
            myNodes[aShapeIndex].IsAssembly = true;
            for (var anIt = new TopoDS_Iterator(theShape); anIt.More(); anIt.Next())
            {
                TopoDS_Shape aComp = anIt.Value();
                int aCompIndex = addShape(aComp.Located(new TopLoc_Location()), makeAssembly);
                int aRef = newChild(aShapeIndex);
                makeReference(aRef, aCompIndex, aComp.Location());
            }
            myNodes[aShapeIndex].IsDirty = false; // the compound given is its components' compound already
        }
        if (!myNodes[aShapeIndex].IsAssembly)
        {
            NCollection_IndexedMap<TopoDS_Shape> aMap = subShapeMap(aShapeIndex)!;
            for (int i = 1; i <= aMap.Extent(); i++)
            {
                TopoDS_Shape aSub = aMap.FindKey(i);
                mySubShapes[aSub] = aShapeIndex;
                if (!aSub.Location().IsIdentity())
                {
                    mySubShapes[aSub.Located(new TopLoc_Location())] = aShapeIndex;
                }
            }
        }
        return aShapeIndex;
    }

    private void makeReference(int theIndex, int theReferred, in TopLoc_Location theLocation)
    {
        Node aNode = myNodes[theIndex];
        aNode.Referred = theReferred;
        aNode.RefLocation = theLocation;
        myNodes[theReferred].Users.Add(theIndex);
        if (aNode.Father == ShapesIndex)
        {
            TopoDS_Shape aShape = getShape(theIndex);
            if (!aShape.IsNull())
            {
                myTopLevelShapes.TryAdd(aShape, theIndex);
            }
        }
        int aFather = aNode.Father;
        if (aFather >= 0 && myNodes[aFather].IsAssembly)
        {
            myNodes[aFather].IsDirty = true;
            markUsersDirty(aFather);
        }
    }

    /// <summary>
    /// Adds a component to the assembly (a simple shape becomes an assembly): a reference to the label at the location
    /// (<c>XCAFDoc_ShapeTool::AddComponent</c>). The null label if the father is neither, or if the component contains
    /// the assembly (a cycle, which XCAF doesn't check).
    /// </summary>
    public XdeDoc_Label AddComponent(in XdeDoc_Label theAssembly, in XdeDoc_Label theComponent, in TopLoc_Location theLocation)
    {
        int anAssembly = index(theAssembly);
        int aComponent = index(theComponent);
        if (reaches(aComponent, anAssembly))
        {
            return default;
        }
        if (!myNodes[anAssembly].IsAssembly)
        {
            if (!IsSimpleShape(theAssembly))
            {
                return default;
            }
            myNodes[anAssembly].IsAssembly = true;
        }
        int aRef = newChild(anAssembly);
        makeReference(aRef, aComponent, theLocation);
        TopoDS_Shape aShape = getShape(aRef);
        if (!aShape.IsNull())
        {
            myShapeLabels.TryAdd(aShape, aRef);
        }
        return label(aRef);
    }

    // True if the label is the target, or refers to it, or has a component reaching it
    private bool reaches(int theFrom, int theTarget)
    {
        var aStack = new Stack<int>();
        var aSeen = new HashSet<int>();
        aStack.Push(theFrom);
        while (aStack.Count > 0)
        {
            int anIndex = aStack.Pop();
            if (anIndex == theTarget)
            {
                return true;
            }
            if (!aSeen.Add(anIndex))
            {
                continue;
            }
            Node aNode = myNodes[anIndex];
            if (aNode.Referred >= 0)
            {
                aStack.Push(aNode.Referred);
            }
            if (aNode.IsAssembly)
            {
                foreach (int aChild in aNode.Children)
                {
                    if (myNodes[aChild].Referred >= 0)
                    {
                        aStack.Push(aChild);
                    }
                }
            }
        }
        return false;
    }

    /// <summary>Adds the shape at the top level (an assembly if <paramref name="expand"/>) and as a component.</summary>
    public XdeDoc_Label AddComponent(in XdeDoc_Label theAssembly, in TopoDS_Shape theComponent, bool expand = false)
    {
        XdeDoc_Label aCompLabel = AddShape(theComponent.Located(new TopLoc_Location()), expand);
        return AddComponent(theAssembly, aCompLabel, theComponent.Location());
    }

    /// <summary>Remakes the assemblies' compounds from their components (<c>XCAFDoc_ShapeTool::UpdateAssemblies</c>).</summary>
    public void UpdateAssemblies()
    {
        foreach (int aTop in myNodes[ShapesIndex].Children.ToList())
        {
            getShape(aTop);
        }
    }

    /// <summary>Sets the top-level shape labels (those holding a shape) in <paramref name="theLabels"/>.</summary>
    public void GetShapes(NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        theLabels.Clear();
        foreach (int aTop in myNodes[ShapesIndex].Children)
        {
            if (!getShape(aTop).IsNull())
            {
                theLabels.Append(label(aTop));
            }
        }
    }

    /// <summary>Sets the free top-level shape labels (the roots) in <paramref name="theLabels"/>.</summary>
    public void GetFreeShapes(NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        theLabels.Clear();
        foreach (int aTop in myNodes[ShapesIndex].Children)
        {
            if (myNodes[aTop].Users.Count == 0 && !getShape(aTop).IsNull())
            {
                theLabels.Append(label(aTop));
            }
        }
    }

    /// <summary>The free shapes: the only one, or a compound of them (<c>XCAFDoc_ShapeTool::GetOneShape</c>).</summary>
    public TopoDS_Shape GetOneShape()
    {
        var aFree = new NCollection_Sequence<XdeDoc_Label>();
        GetFreeShapes(aFree);
        if (aFree.Length() == 1)
        {
            return GetShape(aFree.Value(1));
        }
        var aBuilder = new BRep_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        foreach (XdeDoc_Label aLabel in aFree)
        {
            TopoDS_Shape aShape = GetShape(aLabel);
            if (!aShape.IsNull())
            {
                aBuilder.Add(aCompound, aShape);
            }
        }
        aCompound.TShape()!.FreezeGraph();
        return aCompound;
    }

    /// <summary>
    /// Appends the references to the label (and, if <paramref name="getsubchilds"/>, first those to their fathers, up
    /// the tree) to <paramref name="theLabels"/>; returns their number (<c>XCAFDoc_ShapeTool::GetUsers</c>).
    /// </summary>
    public int GetUsers(in XdeDoc_Label theLabel, NCollection_Sequence<XdeDoc_Label> theLabels, bool getsubchilds = false)
    {
        int aCount = 0;
        foreach (int aUser in node(theLabel).Users)
        {
            if (getsubchilds)
            {
                int aFather = myNodes[aUser].Father;
                if (aFather >= 0)
                {
                    aCount += GetUsers(label(aFather), theLabels, getsubchilds);
                }
            }
            theLabels.Append(label(aUser));
            aCount++;
        }
        return aCount;
    }

    /// <summary>The label's location: a reference's, or its shape's.</summary>
    public TopLoc_Location GetLocation(in XdeDoc_Label theLabel)
    {
        Node aNode = node(theLabel);
        return aNode.Referred >= 0 ? aNode.RefLocation : aNode.Shape.IsNull() ? new TopLoc_Location() : aNode.Shape.Location();
    }

    /// <summary>The label a reference refers to; false if the label isn't a reference.</summary>
    public bool GetReferredShape(in XdeDoc_Label theLabel, out XdeDoc_Label theReferred)
    {
        theReferred = default;
        if (!IsReference(theLabel))
        {
            return false;
        }
        theReferred = label(node(theLabel).Referred);
        return true;
    }

    /// <summary>The number of an assembly's components (with those of its sub-assemblies if <paramref name="getsubchilds"/>).</summary>
    public int NbComponents(in XdeDoc_Label theLabel, bool getsubchilds = false)
    {
        var aLabels = new NCollection_Sequence<XdeDoc_Label>();
        GetComponents(theLabel, aLabels, getsubchilds);
        return aLabels.Length();
    }

    /// <summary>
    /// Appends an assembly's components to <paramref name="theLabels"/> (each after those of its referred assembly if
    /// <paramref name="getsubchilds"/>); false if the label isn't an assembly.
    /// </summary>
    public bool GetComponents(in XdeDoc_Label theLabel, NCollection_Sequence<XdeDoc_Label> theLabels, bool getsubchilds = false)
    {
        if (!IsAssembly(theLabel))
        {
            return false;
        }
        foreach (int aChild in node(theLabel).Children)
        {
            XdeDoc_Label aComp = label(aChild);
            if (IsComponent(aComp))
            {
                if (getsubchilds && GetReferredShape(aComp, out XdeDoc_Label anUnder))
                {
                    GetComponents(anUnder, theLabels, getsubchilds);
                }
                theLabels.Append(aComp);
            }
        }
        return true;
    }

    // ---------------------------------------------------------------------------------------------- shape tool: sub-shapes

    /// <summary>The sub-shape label of the shape under the shape label; false if there is none.</summary>
    public bool FindSubShape(in XdeDoc_Label theShapeLabel, in TopoDS_Shape theSub, out XdeDoc_Label theLabel)
    {
        theLabel = default;
        if (theSub.IsNull())
        {
            return false;
        }
        Node aNode = node(theShapeLabel);
        if (aNode.SubShapeLabels is not null && aNode.SubShapeLabels.TryGetValue(theSub, out int anIndex))
        {
            theLabel = label(anIndex);
            return true;
        }
        return false;
    }

    /// <summary>
    /// Adds a sub-shape label for the shape under a top-level simple shape (<c>XCAFDoc_ShapeTool::AddSubShape</c>);
    /// the existing label if there is one; the null label if the shape isn't a sub-shape.
    /// </summary>
    public XdeDoc_Label AddSubShape(in XdeDoc_Label theShapeLabel, in TopoDS_Shape theSub)
    {
        AddSubShape(theShapeLabel, theSub, out XdeDoc_Label aLabel);
        return aLabel;
    }

    /// <summary>
    /// As <see cref="AddSubShape(in XdeDoc_Label, in TopoDS_Shape)"/>, true only if a label was added (false if it
    /// existed, with the label set, or if none can be).
    /// </summary>
    public bool AddSubShape(in XdeDoc_Label theShapeLabel, in TopoDS_Shape theSub, out XdeDoc_Label theAdded)
    {
        theAdded = default;
        if (!IsSimpleShape(theShapeLabel) || !IsTopLevel(theShapeLabel))
        {
            return false;
        }
        TopoDS_Shape aSubShape = theSub;
        if (!IsSubShape(theShapeLabel, theSub))
        {
            bool isDefined = false;
            // the sub-shape without its location: take it with its location in the main shape
            if (theSub.Location().IsIdentity())
            {
                for (var anIt = new TopoDS_Iterator(GetShape(theShapeLabel)); anIt.More() && !isDefined; anIt.Next())
                {
                    TopoDS_Shape aShape = anIt.Value();
                    if (theSub.IsSame(aShape.Located(new TopLoc_Location())))
                    {
                        isDefined = true;
                        aSubShape = aShape;
                    }
                }
            }
            if (!isDefined)
            {
                return false;
            }
        }
        if (FindSubShape(theShapeLabel, aSubShape, out theAdded))
        {
            return false;
        }
        int aFather = index(theShapeLabel);
        int anIndex = newChild(aFather);
        aSubShape.TShape()?.FreezeGraph();
        myNodes[anIndex].Shape = aSubShape;
        (myNodes[aFather].SubShapeLabels ??= new(TopTools_ShapeMapHasher.Instance)).TryAdd(aSubShape, anIndex);
        theAdded = label(anIndex);
        return true;
    }

    /// <summary>Appends the label's sub-shape labels to <paramref name="theLabels"/>; false if it has none.</summary>
    public bool GetSubShapes(in XdeDoc_Label theLabel, NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        foreach (int aChild in node(theLabel).Children)
        {
            if (IsSubShape(label(aChild)))
            {
                theLabels.Append(label(aChild));
            }
        }
        return theLabels.Length() > 0;
    }

    /// <summary>Records the external files the label's shape was read from (<c>XCAFDoc_ShapeTool::SetExternRefs</c>).</summary>
    public void SetExternRefs(in XdeDoc_Label theLabel, IEnumerable<string> theRefs)
    {
        Node aNode = node(theLabel);
        aNode.Attributes = aNode.Attributes with { ExternRefs = [.. theRefs] };
    }

    /// <summary>Appends the external files of the label to <paramref name="theRefs"/>.</summary>
    public void GetExternRefs(in XdeDoc_Label theLabel, NCollection_Sequence<string> theRefs)
    {
        foreach (string aRef in node(theLabel).Attributes.ExternRefs)
        {
            theRefs.Append(aRef);
        }
    }

    // ---------------------------------------------------------------------------------------------- names

    /// <summary>Sets the label's name (<c>TDataStd_Name::Set</c>).</summary>
    public void SetName(in XdeDoc_Label theLabel, string theName)
    {
        Node aNode = node(theLabel);
        aNode.Attributes = aNode.Attributes.WithName(theName);
        aNode.HasName = true;
    }

    /// <summary>The label's name; false if it has none.</summary>
    public bool GetName(in XdeDoc_Label theLabel, out string? theName)
    {
        theName = null;
        if (!isMine(theLabel))
        {
            return false;
        }
        Node aNode = node(theLabel);
        theName = aNode.Attributes.Name;
        return aNode.HasName;
    }

    // ---------------------------------------------------------------------------------------------- colour tool

    /// <summary>Sets the label's colour of the type (<c>XCAFDoc_ColorTool::SetColor</c>).</summary>
    public void SetColor(in XdeDoc_Label theLabel, in Quantity_ColorRGBA theColor, XdeDoc_ColorType theType)
    {
        Node aNode = node(theLabel);
        aNode.Attributes = aNode.Attributes.WithColor(theType, theColor);
    }

    /// <summary>Sets the label's colour of the type, opaque.</summary>
    public void SetColor(in XdeDoc_Label theLabel, in Quantity_Color theColor, XdeDoc_ColorType theType)
        => SetColor(theLabel, new Quantity_ColorRGBA(theColor), theType);

    /// <summary>Sets the colour of the shape's label, found by <see cref="Search"/>; false if not found.</summary>
    public bool SetColor(in TopoDS_Shape theShape, in Quantity_ColorRGBA theColor, XdeDoc_ColorType theType)
    {
        if (!Search(theShape, out XdeDoc_Label aLabel))
        {
            return false;
        }
        SetColor(aLabel, theColor, theType);
        return true;
    }

    /// <summary>The label's colour of the type; false if not set.</summary>
    public bool GetColor(in XdeDoc_Label theLabel, XdeDoc_ColorType theType, out Quantity_ColorRGBA theColor)
    {
        theColor = default;
        if (!isMine(theLabel) || node(theLabel).Attributes.Color(theType) is not Quantity_ColorRGBA aColor)
        {
            return false;
        }
        theColor = aColor;
        return true;
    }

    /// <summary>The label's colour of the type, without its alpha; false if not set.</summary>
    public bool GetColor(in XdeDoc_Label theLabel, XdeDoc_ColorType theType, out Quantity_Color theColor)
    {
        bool isSet = GetColor(theLabel, theType, out Quantity_ColorRGBA aColor);
        theColor = aColor.GetRGB();
        return isSet;
    }

    /// <summary>The colour of the shape's label (found by <see cref="Search"/>); false if not found or not set.</summary>
    public bool GetColor(in TopoDS_Shape theShape, XdeDoc_ColorType theType, out Quantity_ColorRGBA theColor)
    {
        theColor = default;
        return Search(theShape, out XdeDoc_Label aLabel) && GetColor(aLabel, theType, out theColor);
    }

    /// <summary>True if the label has a colour of the type.</summary>
    public bool IsSet(in XdeDoc_Label theLabel, XdeDoc_ColorType theType)
        => isMine(theLabel) && node(theLabel).Attributes.Color(theType) is not null;

    /// <summary>Removes the label's colour of the type.</summary>
    public void UnSetColor(in XdeDoc_Label theLabel, XdeDoc_ColorType theType)
    {
        Node aNode = node(theLabel);
        aNode.Attributes = aNode.Attributes.WithColor(theType, null);
    }

    /// <summary>Shows or hides the shape or layer of the label (<c>XCAFDoc_ColorTool</c>'s and <c>XCAFDoc_LayerTool</c>'s <c>SetVisibility</c>).</summary>
    public void SetVisibility(in XdeDoc_Label theLabel, bool isVisible = true)
    {
        Node aNode = node(theLabel);
        aNode.Attributes = aNode.Attributes with { IsVisible = isVisible };
    }

    /// <summary>False if the shape or layer of the label is hidden.</summary>
    public bool IsVisible(in XdeDoc_Label theLabel) => !isMine(theLabel) || node(theLabel).Attributes.IsVisible;

    // ---------------------------------------------------------------------------------------------- layer tool

    /// <summary>The first layer of the name (<c>XCAFDoc_LayerTool::FindLayer</c>), or the null label.</summary>
    public XdeDoc_Label FindLayer(string theLayer, bool toFindWithProperty = false, bool toFindVisible = true)
    {
        foreach (int aChild in myNodes[LayersIndex].Children)
        {
            Node aNode = myNodes[aChild];
            if (aNode.Attributes.Name == theLayer && (!toFindWithProperty || aNode.Attributes.IsVisible == toFindVisible))
            {
                return label(aChild);
            }
        }
        return default;
    }

    /// <summary>The first layer of the name; false if there is none.</summary>
    public bool FindLayer(string theLayer, out XdeDoc_Label theLabel)
    {
        theLabel = FindLayer(theLayer);
        return !theLabel.IsNull();
    }

    /// <summary>The layer of the name, added if there is none (<c>XCAFDoc_LayerTool::AddLayer</c>).</summary>
    public XdeDoc_Label AddLayer(string theLayer)
    {
        XdeDoc_Label aFound = FindLayer(theLayer);
        return aFound.IsNull() ? newLayer(theLayer) : aFound;
    }

    /// <summary>
    /// The layer of the name and visibility, added if there is none (<c>XCAFDoc_LayerTool::AddLayer(name, visible)</c>:
    /// the visibility is only searched; the caller sets it).
    /// </summary>
    public XdeDoc_Label AddLayer(string theLayer, bool toFindVisible)
    {
        XdeDoc_Label aFound = FindLayer(theLayer, true, toFindVisible);
        return aFound.IsNull() ? newLayer(theLayer) : aFound;
    }

    private XdeDoc_Label newLayer(string theLayer)
    {
        int anIndex = newChild(LayersIndex);
        myNodes[anIndex].Attributes = XdeDoc_Attributes.Empty.WithName(theLayer);
        myNodes[anIndex].HasName = true;
        return label(anIndex);
    }

    /// <summary>The layers' labels, in order.</summary>
    public void GetLayerLabels(NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        theLabels.Clear();
        foreach (int aChild in myNodes[LayersIndex].Children)
        {
            theLabels.Append(label(aChild));
        }
    }

    /// <summary>The layer's name; false if the label isn't a named layer.</summary>
    public bool GetLayer(in XdeDoc_Label theLayer, out string? theName)
    {
        theName = null;
        if (!isMine(theLayer) || node(theLayer).Father != LayersIndex)
        {
            return false;
        }
        return GetName(theLayer, out theName);
    }

    /// <summary>Puts the shape label in the layer (alone in it if <paramref name="shapeInOneLayer"/>).</summary>
    public void SetLayer(in XdeDoc_Label theLabel, in XdeDoc_Label theLayer, bool shapeInOneLayer = false)
    {
        int aShape = index(theLabel);
        int aLayer = index(theLayer);
        if (shapeInOneLayer)
        {
            UnSetLayers(theLabel);
        }
        List<int> aLayers = myNodes[aShape].Layers ??= [];
        if (!aLayers.Contains(aLayer))
        {
            aLayers.Add(aLayer);
            (myNodes[aLayer].LayerShapes ??= []).Add(aShape);
        }
    }

    /// <summary>Puts the shape label in the layer of the name (added if there is none).</summary>
    public void SetLayer(in XdeDoc_Label theLabel, string theLayer, bool shapeInOneLayer = false)
        => SetLayer(theLabel, AddLayer(theLayer), shapeInOneLayer);

    /// <summary>Takes the shape label out of all its layers.</summary>
    public void UnSetLayers(in XdeDoc_Label theLabel)
    {
        int aShape = index(theLabel);
        if (myNodes[aShape].Layers is List<int> aLayers)
        {
            foreach (int aLayer in aLayers)
            {
                myNodes[aLayer].LayerShapes?.Remove(aShape);
            }
            myNodes[aShape].Layers = null;
        }
    }

    /// <summary>True if the shape label is in the layer.</summary>
    public bool IsSet(in XdeDoc_Label theLabel, in XdeDoc_Label theLayer)
        => node(theLabel).Layers?.Contains(index(theLayer)) == true;

    /// <summary>Appends the shape label's layers to <paramref name="theLayers"/>; false if it has none.</summary>
    public bool GetLayers(in XdeDoc_Label theLabel, NCollection_Sequence<XdeDoc_Label> theLayers)
    {
        if (node(theLabel).Layers is not List<int> aLayers || aLayers.Count == 0)
        {
            return false;
        }
        foreach (int aLayer in aLayers)
        {
            theLayers.Append(label(aLayer));
        }
        return true;
    }

    /// <summary>Appends the names of the shape label's layers to <paramref name="theLayers"/>; false if it has none.</summary>
    public bool GetLayers(in XdeDoc_Label theLabel, NCollection_Sequence<string> theLayers)
    {
        if (node(theLabel).Layers is not List<int> aLayers || aLayers.Count == 0)
        {
            return false;
        }
        foreach (int aLayer in aLayers)
        {
            theLayers.Append(myNodes[aLayer].Attributes.Name ?? "");
        }
        return true;
    }

    /// <summary>Appends the shape labels in the layer to <paramref name="theLabels"/>.</summary>
    public void GetShapesOfLayer(in XdeDoc_Label theLayer, NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        theLabels.Clear();
        foreach (int aShape in node(theLayer).LayerShapes ?? [])
        {
            theLabels.Append(label(aShape));
        }
    }

    // ---------------------------------------------------------------------------------------------- validation properties

    /// <summary>Sets the label's volume (<c>XCAFDoc_Volume::Set</c>).</summary>
    public void SetVolume(in XdeDoc_Label theLabel, double theVolume)
        => setProps(theLabel, p => p with { Volume = theVolume });

    /// <summary>Sets the label's area (<c>XCAFDoc_Area::Set</c>).</summary>
    public void SetArea(in XdeDoc_Label theLabel, double theArea) => setProps(theLabel, p => p with { Area = theArea });

    /// <summary>Sets the label's centroid (<c>XCAFDoc_Centroid::Set</c>).</summary>
    public void SetCentroid(in XdeDoc_Label theLabel, in gp_Pnt theCentroid)
    {
        gp_Pnt aCentroid = theCentroid;
        setProps(theLabel, p => p with { Centroid = aCentroid });
    }

    /// <summary>The label's validation properties, or null.</summary>
    public XdeDoc_ValidationProps? GetValidationProps(in XdeDoc_Label theLabel)
        => isMine(theLabel) ? node(theLabel).Attributes.ValidationProps : null;

    private void setProps(in XdeDoc_Label theLabel, Func<XdeDoc_ValidationProps, XdeDoc_ValidationProps> theChange)
    {
        Node aNode = node(theLabel);
        aNode.Attributes = aNode.Attributes with
        {
            ValidationProps = theChange(aNode.Attributes.ValidationProps ?? XdeDoc_ValidationProps.None),
        };
    }

    // ---------------------------------------------------------------------------------------------- attributes

    /// <summary>The label's attributes (name, colours, visibility, layer names, properties) as a tree carries them.</summary>
    public XdeDoc_Attributes GetAttributes(in XdeDoc_Label theLabel)
    {
        Node aNode = node(theLabel);
        XdeDoc_Attributes anAttributes = aNode.Attributes;
        if (aNode.Layers is { Count: > 0 } aLayers)
        {
            anAttributes = anAttributes with
            {
                Layers = [.. aLayers.Select(l => myNodes[l].Attributes.Name ?? "").Distinct(StringComparer.Ordinal)],
            };
        }
        return anAttributes;
    }

    /// <summary>Sets the label's attributes (the layers by name, added if missing).</summary>
    public void SetAttributes(in XdeDoc_Label theLabel, XdeDoc_Attributes theAttributes)
    {
        Node aNode = node(theLabel);
        aNode.Attributes = theAttributes with { Layers = [] };
        aNode.HasName = theAttributes.Name is not null;
        UnSetLayers(theLabel);
        foreach (string aLayer in theAttributes.Layers)
        {
            SetLayer(theLabel, aLayer);
        }
    }

    // ---------------------------------------------------------------------------------------------- the tree

    /// <summary>
    /// The document frozen into a tree: the free shapes become the roots (a free reference an instance of its referred
    /// shape at its location), each top-level shape a product (shared by its references), each component an instance,
    /// each sub-shape label a sub-shape entry. Labels that no free shape reaches are left out. The layers keep their
    /// order; a layer whose name an earlier layer has is merged into it (the tree's layers are named uniquely).
    /// </summary>
    /// <exception cref="InvalidOperationException">The references make a cycle.</exception>
    public XdeDoc_Tree ToTree()
    {
        var aProducts = new Dictionary<int, XdeDoc_Product>();
        var aInProgress = new HashSet<int>();

        XdeDoc_Product productOf(int theIndex)
        {
            if (aProducts.TryGetValue(theIndex, out XdeDoc_Product? aKnown))
            {
                return aKnown;
            }
            if (!aInProgress.Add(theIndex))
            {
                throw new InvalidOperationException("XdeDoc_Document: a cycle of references");
            }
            Node aNode = myNodes[theIndex];
            XdeDoc_Attributes anAttributes = GetAttributes(label(theIndex));
            XdeDoc_Product aProduct;
            if (aNode.IsAssembly)
            {
                var aComponents = new List<XdeDoc_Instance>();
                foreach (int aChild in aNode.Children)
                {
                    if (myNodes[aChild].Referred >= 0)
                    {
                        aComponents.Add(instanceOf(aChild));
                    }
                }
                aProduct = XdeDoc_Product.Assembly(aComponents, anAttributes);
            }
            else
            {
                var aSubs = new List<XdeDoc_SubShape>();
                foreach (int aChild in aNode.Children)
                {
                    Node aSub = myNodes[aChild];
                    if (aSub.Referred < 0 && !aSub.Shape.IsNull())
                    {
                        aSubs.Add(new XdeDoc_SubShape(aSub.Shape, GetAttributes(label(aChild))));
                    }
                }
                TopoDS_Shape aShape = getShape(theIndex);
                aProduct = aShape.IsNull()
                             ? XdeDoc_Product.Assembly([], anAttributes)
                             : XdeDoc_Product.Part(aShape, anAttributes, aSubs);
            }
            aInProgress.Remove(theIndex);
            aProducts[theIndex] = aProduct;
            return aProduct;
        }

        XdeDoc_Instance instanceOf(int theRef)
        {
            // a reference to a reference: the locations composed, the last product
            TopLoc_Location aLocation = myNodes[theRef].RefLocation;
            int aReferred = myNodes[theRef].Referred;
            int aGuard = 0;
            while (myNodes[aReferred].Referred >= 0)
            {
                if (++aGuard > myNodes.Count)
                {
                    throw new InvalidOperationException("XdeDoc_Document: a cycle of references");
                }
                aLocation = aLocation.Multiplied(myNodes[aReferred].RefLocation);
                aReferred = myNodes[aReferred].Referred;
            }
            return new XdeDoc_Instance(productOf(aReferred), aLocation, GetAttributes(label(theRef)));
        }

        var aRoots = new List<XdeDoc_Instance>();
        var aFree = new NCollection_Sequence<XdeDoc_Label>();
        GetFreeShapes(aFree);
        foreach (XdeDoc_Label aRoot in aFree)
        {
            aRoots.Add(myNodes[aRoot.Index].Referred >= 0
                         ? instanceOf(aRoot.Index)
                         : new XdeDoc_Instance(productOf(aRoot.Index)));
        }

        var aLayers = new List<XdeDoc_Layer>();
        var aNames = new HashSet<string>(StringComparer.Ordinal);
        foreach (int aChild in myNodes[LayersIndex].Children)
        {
            string aName = myNodes[aChild].Attributes.Name ?? "";
            if (aNames.Add(aName))
            {
                aLayers.Add(new XdeDoc_Layer(aName, myNodes[aChild].Attributes.IsVisible));
            }
        }
        return new XdeDoc_Tree(aRoots, aLayers, LengthUnit);
    }

    /// <summary>
    /// A document holding the tree, as XCAF would after reading it: each product a top-level label (in depth-first
    /// order, a product before its components' products), each component a reference under its assembly, each
    /// sub-shape entry a sub-shape label; a root placed or carrying attributes of its own becomes a top-level
    /// reference (made before its product's label, as <c>XCAFDoc_ShapeTool::AddShape</c> does), another root is its
    /// product's label. The labels hold the tree's shapes, so the document's shapes are the tree's.
    /// </summary>
    public static XdeDoc_Document FromTree(XdeDoc_Tree theTree)
    {
        var aDoc = new XdeDoc_Document();
        aDoc.LengthUnit = theTree.LengthUnit;
        foreach (XdeDoc_Layer aLayer in theTree.Layers)
        {
            XdeDoc_Label aLabel = aDoc.newLayer(aLayer.Name);
            aDoc.SetVisibility(aLabel, aLayer.IsVisible);
        }

        var aLabels = new Dictionary<XdeDoc_Product, int>(ReferenceEqualityComparer.Instance);
        int labelOf(XdeDoc_Product theProduct)
        {
            if (aLabels.TryGetValue(theProduct, out int aKnown))
            {
                return aKnown;
            }
            int anIndex = aDoc.newChild(ShapesIndex);
            aLabels[theProduct] = anIndex;
            Node aNode = aDoc.myNodes[anIndex];
            aDoc.SetAttributes(aDoc.label(anIndex), theProduct.Attributes);
            if (theProduct.IsAssembly)
            {
                aNode.IsAssembly = true;
                foreach (XdeDoc_Instance aComponent in theProduct.Components)
                {
                    int aReferred = labelOf(aComponent.Product);
                    int aRef = aDoc.newChild(anIndex);
                    aDoc.makeReference(aRef, aReferred, aComponent.Location);
                    aDoc.SetAttributes(aDoc.label(aRef), aComponent.Attributes);
                    TopoDS_Shape aCompShape = aDoc.getShape(aRef);
                    if (!aCompShape.IsNull())
                    {
                        aDoc.myShapeLabels.TryAdd(aCompShape, aRef);
                    }
                }
                aDoc.setOwnShape(anIndex, theProduct.Shape);
                aNode.IsDirty = false;
            }
            else
            {
                aDoc.setOwnShape(anIndex, theProduct.Shape);
                foreach (XdeDoc_SubShape aSub in theProduct.SubShapes)
                {
                    int aSubIndex = aDoc.newChild(anIndex);
                    aDoc.myNodes[aSubIndex].Shape = aSub.Shape;
                    (aNode.SubShapeLabels ??= new(TopTools_ShapeMapHasher.Instance)).TryAdd(aSub.Shape, aSubIndex);
                    aDoc.SetAttributes(aDoc.label(aSubIndex), aSub.Attributes);
                }
                NCollection_IndexedMap<TopoDS_Shape> aMap = aDoc.subShapeMap(anIndex)!;
                for (int i = 1; i <= aMap.Extent(); i++)
                {
                    aDoc.mySubShapes.TryAdd(aMap.FindKey(i), anIndex);
                }
            }
            aDoc.myShapeLabels.TryAdd(theProduct.Shape, anIndex);
            return anIndex;
        }

        foreach (XdeDoc_Instance aRoot in theTree.Roots)
        {
            if (aRoot.Location.IsIdentity() && aRoot.Attributes.IsEmpty)
            {
                labelOf(aRoot.Product);
                continue;
            }
            int aRef = aDoc.newChild(ShapesIndex);
            int aReferred = labelOf(aRoot.Product);
            aDoc.makeReference(aRef, aReferred, aRoot.Location);
            aDoc.SetAttributes(aDoc.label(aRef), aRoot.Attributes);
        }
        return aDoc;
    }
}
