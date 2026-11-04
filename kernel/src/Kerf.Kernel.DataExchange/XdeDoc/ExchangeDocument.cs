// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port; its methods
// follow XCAF's tools (src/DataExchange/TKXCAF/XCAFDoc/XCAFDoc_ShapeTool, _ColorTool, _LayerTool, _DocumentTool)
// so that code ported from STEPCAFControl and the XDE Draw commands can call them in place of the tools'.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Collections.Immutable;
using System.Globalization;
using System.Text;

namespace Kerf.Engine.Exchange.Documents;

/// <summary>
/// A mutable assembly document with XCAF's label interface: what STEPCAFControl's reader fills and its writer reads,
/// in place of a <c>TDocStd_Document</c> with XCAF's tools (decision 1231). <see cref="ToTree"/> freezes it into an
/// <see cref="Tree"/>; <see cref="FromTree"/> opens a tree.
/// </summary>
/// <remarks>
/// <para>
/// The labels are laid out as XCAF's: the shapes under 0:1:1 (top-level shapes, with an assembly's components and a
/// part's sub-shapes as their children), the layers under 0:1:3. Each method has the name and the behaviour of the
/// XCAF tool method it replaces (<c>XCAFDoc_ShapeTool</c>: <see cref="AddShape"/>, <see cref="NewShape"/>,
/// <see cref="AddComponent(in Label, in Label, in Placement)"/>, <see cref="Search"/>,
/// <see cref="SearchUsingMap"/>, <see cref="FindShape(in Shape, out Label, bool)"/>,
/// <see cref="AddSubShape(in Label, in Shape)"/>, <see cref="GetFreeShapes"/>, <see cref="GetComponents"/>,
/// <see cref="GetReferredShape"/>, <see cref="GetLocation"/>, the <c>Is...</c> tests...; <c>XCAFDoc_ColorTool</c>:
/// <see cref="SetColor(in Label, in ColorWithAlpha, ColorType)"/>, <see cref="GetColor(in Label, ColorType, out ColorWithAlpha)"/>,
/// <see cref="SetVisibility"/>, <see cref="IsVisible"/>; <c>XCAFDoc_LayerTool</c>: <see cref="AddLayer(string)"/>,
/// <see cref="SetLayer(in Label, in Label, bool)"/>, <see cref="GetLayers(in Label, NCollection_Sequence{string})"/>...;
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
internal sealed class ExchangeDocument
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

        public Shape Shape;           // the shape set on the label (none for a reference)
        public bool IsAssembly;
        public bool IsDirty;                 // an assembly whose compound must be remade from its components
        public int Referred = -1;            // a reference's referred label
        public Placement RefLocation;  // and its location
        public readonly List<int> Users = []; // the references to this label, in order

        public Attributes Attributes = global::Kerf.Engine.Exchange.Documents.Attributes.Empty; // all but the layers
        public bool HasName;
        public List<int>? Layers;            // layer labels (a shape's), in order
        public List<int>? LayerShapes;       // shape labels (a layer's), in order

        public IndexedSet<Shape>? SubShapeMap; // XCAFDoc_ShapeMapTool's map
        public Dictionary<Shape, int>? SubShapeLabels;      // sub-shape children by shape

        public ColorWithAlpha? Color;    // a colour label's colour (XCAFDoc_Color)
        public int[]? ColorRefs;             // a shape label's colour labels, by XdeDoc_ColorType (-1: none)
    }

    private readonly List<Node> _nodes = [];
    private readonly Dictionary<Shape, int> _topLevelShapes = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, int> _shapeLabels = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, int> _subShapes = new(ShapeHasher.Instance);

    /// <summary>An empty document.</summary>
    public ExchangeDocument()
    {
        _nodes.Add(new Node { Tag = 1 });   // 0:1, the main label
        NewChild(MainIndex);                  // 0:1:1, the shapes
        NewChild(MainIndex);                  // 0:1:2, the colours (colours are values here: no child)
        NewChild(MainIndex);                  // 0:1:3, the layers
    }

    /// <summary>
    /// Whether adding shapes names their labels as XCAF's auto-naming does (unit 8G1): a shape label by its shape's
    /// type ("SOLID"), an assembly "ASSEMBLY", a reference "=&gt;[entry of the referred label]". XCAF's switch
    /// (<c>XCAFDoc_ShapeTool::SetAutoNaming</c>) is process-wide and on by default; here it is the document's, off by
    /// default so that trees made from documents carry only the names given. Draw-lite's documents turn it on.
    /// </summary>
    public bool AutoNaming { get; set; }

    /// <summary>The length unit, or null if not set (<c>XCAFDoc_DocumentTool::SetLengthUnit</c>).</summary>
    public DocumentLengthUnit? LengthUnit { get; private set; }

    /// <summary>Sets the length unit.</summary>
    public void SetLengthUnit(DocumentLengthUnit? unit) => LengthUnit = unit;

    /// <summary>Sets the length unit by its size in metres (<c>XCAFDoc_DocumentTool::SetLengthUnit</c>).</summary>
    public void SetLengthUnit(double unitValue, string name = "")
        => LengthUnit = new DocumentLengthUnit(name, unitValue);

    /// <summary>The length unit's size in metres, false if not set (<c>XCAFDoc_DocumentTool::GetLengthUnit</c>).</summary>
    public bool GetLengthUnit(out double result)
    {
        result = LengthUnit?.UnitValue ?? 0.0;
        return LengthUnit is not null;
    }

    // ---------------------------------------------------------------------------------------------- labels

    /// <summary>The document's main label (0:1).</summary>
    public Label Main() => new(this, MainIndex);

    /// <summary>The label of the shapes (0:1:1), XCAF's <c>XCAFDoc_ShapeTool::Label()</c>.</summary>
    public Label ShapesLabel() => new(this, ShapesIndex);

    /// <summary>The label of the layers (0:1:3), XCAF's <c>XCAFDoc_LayerTool::Label()</c>.</summary>
    public Label LayersLabel() => new(this, LayersIndex);

    /// <summary>The label of the entry ("0:1:1:2"), or the null label if there is none.</summary>
    public Label FindLabel(string entry)
    {
        string[] parts = entry.Split(':');
        if (parts.Length < 2 || parts[0] != "0" || parts[1] != "1")
        {
            return default;
        }
        int current = MainIndex;
        for (int i = 2; i < parts.Length; i++)
        {
            if (!int.TryParse(parts[i], NumberStyles.None, CultureInfo.InvariantCulture, out int tag))
            {
                return default;
            }
            int found = -1;
            foreach (int child in _nodes[current].Children)
            {
                if (_nodes[child].Tag == tag)
                {
                    found = child;
                    break;
                }
            }
            if (found < 0)
            {
                return default;
            }
            current = found;
        }
        return new Label(this, current);
    }

    /// <summary>The label's children, in order.</summary>
    public IReadOnlyList<Label> Children(in Label label)
        => NodeOf(label).Children.Select(i => new Label(this, i)).ToList();

    internal Label FatherOf(in Label label)
    {
        int father = NodeOf(label).Father;
        return father < 0 ? default : new Label(this, father);
    }

    internal int TagOf(in Label label) => NodeOf(label).Tag;

    internal string EntryOf(in Label label)
    {
        var tags = new List<int>();
        for (int i = this.Index(label); i >= 0; i = _nodes[i].Father)
        {
            tags.Add(_nodes[i].Tag);
        }
        var builder = new StringBuilder("0");
        for (int i = tags.Count - 1; i >= 0; i--)
        {
            builder.Append(':').Append(tags[i].ToString(CultureInfo.InvariantCulture));
        }
        return builder.ToString();
    }

    private int Index(in Label label)
    {
        if (!ReferenceEquals(label.Document, this))
        {
            throw new ArgumentException(label.IsNull ? "Null label" : "Label of another document", nameof(label));
        }
        return label.Index;
    }

    private Node NodeOf(in Label label) => _nodes[this.Index(label)];

    private bool IsMine(in Label label) => ReferenceEquals(label.Document, this);

    private Label Label(int index) => index < 0 ? default : new Label(this, index);

    private int NewChild(int father)
    {
        Node aFather = _nodes[father];
        var node = new Node { Father = father, Tag = aFather.NextTag++ };
        _nodes.Add(node);
        int index = _nodes.Count - 1;
        aFather.Children.Add(index);
        return index;
    }

    // ---------------------------------------------------------------------------------------------- shape tool: tests

    /// <summary>True if the label is a top-level shape label (a child of 0:1:1).</summary>
    public bool IsTopLevel(in Label label) => IsMine(label) && NodeOf(label).Father == ShapesIndex;

    /// <summary>True if no reference refers to the label.</summary>
    public bool IsFree(in Label label) => !IsMine(label) || NodeOf(label).Users.Count == 0;

    /// <summary>True for a simple shape, an assembly or a reference.</summary>
    public bool IsShape(in Label label) => IsSimpleShape(label) || IsAssembly(label) || IsReference(label);

    /// <summary>True for a label holding a shape that is neither an assembly nor a reference.</summary>
    public bool IsSimpleShape(in Label label)
    {
        if (!IsMine(label))
        {
            return false;
        }
        Node node = NodeOf(label);
        return !node.Shape.IsNull && !node.IsAssembly && node.Referred < 0;
    }

    /// <summary>True for a reference (a component, or a located top-level shape).</summary>
    public bool IsReference(in Label label) => IsMine(label) && NodeOf(label).Referred >= 0;

    /// <summary>True for an assembly.</summary>
    public bool IsAssembly(in Label label) => IsMine(label) && NodeOf(label).IsAssembly;

    /// <summary>True for a component: a reference whose father is an assembly.</summary>
    public bool IsComponent(in Label label) => IsReference(label) && IsAssembly(label.Father());

    /// <summary>True for a compound: XCAF tests the name "COMPOUND" (<c>XCAFDoc_ShapeTool::IsCompound</c>).</summary>
    public bool IsCompound(in Label label) => GetName(label, out string? name) && name == "COMPOUND";

    /// <summary>True for a sub-shape label: a simple shape whose father is a shape.</summary>
    public bool IsSubShape(in Label label) => IsSimpleShape(label) && IsShape(label.Father());

    /// <summary>
    /// True if the shape is a sub-shape of the label's shape, with its location there (XCAF's map of sub-shapes, which
    /// leaves the shape itself out).
    /// </summary>
    public bool IsSubShape(in Label shapeLabel, in Shape sub)
    {
        IndexedSet<Shape>? map = SubShapeMap(this.Index(shapeLabel));
        return map is not null && map.Contains(sub);
    }

    /// <summary>True if the label refers to external files (<c>XCAFDoc_ShapeTool::IsExternRef</c>).</summary>
    public bool IsExternRef(in Label label) => IsMine(label) && NodeOf(label).Attributes.ExternRefs.Length > 0;

    private IndexedSet<Shape>? SubShapeMap(int index)
    {
        Node node = _nodes[index];
        if (node.SubShapeMap is null)
        {
            Shape shape = GetShapeCore(index);
            if (shape.IsNull)
            {
                return null;
            }
            node.SubShapeMap = new IndexedSet<Shape>(ShapeHasher.Instance);
            // XCAFDoc_ShapeMapTool::SetShape: the shape's sub-shapes, not the shape (unit 8G2's fix)
            for (var it = new Iterator(shape); it.More(); it.Next())
            {
                AddSubShapes(it.Value(), node.SubShapeMap);
            }
        }
        return node.SubShapeMap;
    }

    // XCAFDoc_ShapeMapTool's AddSubShape: the shape and its sub-shapes, with their locations in it
    private static void AddSubShapes(in Shape shape, IndexedSet<Shape> map)
    {
        var stack = new Stack<Shape>();
        stack.Push(shape);
        while (stack.Count > 0)
        {
            Shape aShape = stack.Pop();
            map.Add(aShape);
            var children = new List<Shape>();
            for (var it = new Iterator(aShape); it.More(); it.Next())
            {
                children.Add(it.Value());
            }
            for (int i = children.Count - 1; i >= 0; i--)
            {
                stack.Push(children[i]);
            }
        }
    }

    // ---------------------------------------------------------------------------------------------- shape tool: shapes

    /// <summary>
    /// The label's shape: its own, or for a reference the referred shape moved to the reference's location, or for an
    /// assembly the compound of its components (remade when components were added). False if it has none.
    /// </summary>
    public bool GetShape(in Label label, out Shape shape)
    {
        shape = IsMine(label) ? GetShapeCore(label.Index) : new Shape();
        return !shape.IsNull;
    }

    /// <summary>The label's shape, or a null shape.</summary>
    public Shape GetShape(in Label label)
    {
        GetShape(label, out Shape shape);
        return shape;
    }

    private Shape GetShapeCore(int index) => GetShapeCore(index, 0);

    private Shape GetShapeCore(int index, int depth)
    {
        if (depth > _nodes.Count)
        {
            throw new InvalidOperationException("XdeDoc_Document: a cycle of references");
        }
        Node node = _nodes[index];
        if (node.Referred >= 0)
        {
            Shape referred = GetShapeCore(node.Referred, depth + 1);
            return referred.IsNull || node.RefLocation.IsIdentity ? referred : referred.Moved(node.RefLocation);
        }
        if (node.IsAssembly && node.IsDirty)
        {
            var builder = new BrepBuilder();
            Compound compound = new();
            builder.MakeCompound(ref compound);
            foreach (int child in node.Children)
            {
                if (_nodes[child].Referred >= 0 && node.IsAssembly)
                {
                    Shape sub = GetShapeCore(child, depth + 1);
                    if (!sub.IsNull)
                    {
                        builder.Add(compound, sub);
                    }
                }
            }
            compound.TShape()!.FreezeGraph();
            node.IsDirty = false;
            SetOwnShape(index, compound);
        }
        return node.Shape;
    }

    // Sets a label's own shape, indexing it; assemblies using the label must remake their compounds
    private void SetOwnShape(int index, in Shape shape)
    {
        Node node = _nodes[index];
        node.Shape = shape;
        node.SubShapeMap = null;
        if (node.Father == ShapesIndex && !shape.IsNull)
        {
            _topLevelShapes.TryAdd(shape, index);
        }
        MarkUsersDirty(index);
    }

    private void MarkUsersDirty(int index)
    {
        var stack = new Stack<int>();
        stack.Push(index);
        var seen = new HashSet<int>();
        while (stack.Count > 0)
        {
            int anIndex = stack.Pop();
            foreach (int user in _nodes[anIndex].Users)
            {
                int assembly = _nodes[user].Father;
                if (assembly >= 0 && _nodes[assembly].IsAssembly && seen.Add(assembly))
                {
                    _nodes[assembly].IsDirty = true;
                    stack.Push(assembly);
                }
                if (seen.Add(-1 - user))
                {
                    stack.Push(user);
                }
            }
        }
    }

    /// <summary>
    /// The top-level label of the shape (with its location if <paramref name="findInstance"/>, else without), false if
    /// there is none (<c>XCAFDoc_ShapeTool::FindShape</c>).
    /// </summary>
    public bool FindShape(in Shape shape, out Label label, bool findInstance = false)
    {
        label = default;
        if (shape.IsNull)
        {
            return false;
        }
        Shape s0 = findInstance ? shape : shape.Located(new Placement());
        if (_topLevelShapes.TryGetValue(s0, out int index) && GetShapeCore(index).IsSame(s0))
        {
            label = Label(index);
            return true;
        }
        // the shapes held may have changed: XCAF's search label by label
        foreach (int child in _nodes[ShapesIndex].Children)
        {
            if (GetShapeCore(child).IsSame(s0))
            {
                label = Label(child);
                return true;
            }
        }
        return false;
    }

    /// <summary>The top-level label of the shape, or the null label.</summary>
    public Label FindShape(in Shape shape, bool findInstance = false)
        => FindShape(shape, out Label label, findInstance) ? label : default;

    /// <summary>
    /// Searches the shape (<c>XCAFDoc_ShapeTool::Search</c>): if located, among the top-level instances and the
    /// assemblies' components; then among the top-level shapes; then as a sub-shape of a top-level simple shape, adding
    /// a sub-shape label for it.
    /// </summary>
    public bool Search(in Shape shape,
                       out Label label,
                       bool findInstance = true,
                       bool findComponent = true,
                       bool findSubShape = true)
    {
        label = default;
        if (shape.IsNull)
        {
            return false;
        }
        if (!shape.Placement.IsIdentity)
        {
            if (findInstance && FindShape(shape, out label, true))
            {
                return true;
            }
            if (findComponent)
            {
                foreach (int top in _nodes[ShapesIndex].Children)
                {
                    if (!_nodes[top].IsAssembly)
                    {
                        continue;
                    }
                    foreach (int comp in _nodes[top].Children)
                    {
                        if (_nodes[comp].Referred >= 0 && GetShapeCore(comp).IsSame(shape))
                        {
                            label = Label(comp);
                            return true;
                        }
                    }
                }
            }
        }
        if (FindShape(shape, out label, false))
        {
            return true;
        }
        if (!findSubShape)
        {
            return false;
        }
        Label main = FindMainShape(shape);
        if (main.IsNull)
        {
            return false;
        }
        label = AddSubShape(main, shape);
        return !label.IsNull;
    }

    /// <summary>
    /// Searches the shape through the map of shapes added (<c>XCAFDoc_ShapeTool::SearchUsingMap</c>): the shape's own
    /// label, or a user of its unlocated shape's label holding it, or (if <paramref name="findWithoutLoc"/>) the
    /// unlocated shape's label; then as a sub-shape, through the map of sub-shapes, adding a sub-shape label.
    /// </summary>
    public bool SearchUsingMap(in Shape shape, out Label label, bool findWithoutLoc, bool findSubShape)
    {
        label = default;
        if (shape.IsNull)
        {
            return false;
        }
        if (_shapeLabels.TryGetValue(shape, out int index))
        {
            label = Label(index);
            return true;
        }
        Shape s0 = shape.Located(new Placement());
        if (_shapeLabels.TryGetValue(s0, out int index1))
        {
            var users = new NCollection_Sequence<Label>();
            if (GetUsers(Label(index1), users, true) > 0)
            {
                foreach (Label user in users)
                {
                    if (GetShape(user).IsSame(shape))
                    {
                        label = user;
                        return true;
                    }
                }
            }
            if (findWithoutLoc)
            {
                label = Label(index1);
                return true;
            }
        }
        if (!findSubShape)
        {
            return false;
        }
        Label main = FindMainShapeUsingMap(shape);
        if (main.IsNull)
        {
            return false;
        }
        label = AddSubShape(main, shape);
        return !label.IsNull;
    }

    /// <summary>The top-level simple shape having the shape as a sub-shape, through the map of sub-shapes added.</summary>
    public Label FindMainShapeUsingMap(in Shape sub)
        => _subShapes.TryGetValue(sub, out int index) ? Label(index) : default;

    /// <summary>The first top-level simple shape having the shape as a sub-shape, or the null label.</summary>
    public Label FindMainShape(in Shape sub)
    {
        foreach (int top in _nodes[ShapesIndex].Children)
        {
            if (IsSimpleShape(Label(top)) && IsSubShape(Label(top), sub))
            {
                return Label(top);
            }
        }
        return default;
    }

    /// <summary>A new top-level label holding an empty compound (<c>XCAFDoc_ShapeTool::NewShape</c>).</summary>
    public Label NewShape()
    {
        int index = NewChild(ShapesIndex);
        var builder = new BrepBuilder();
        Compound compound = new();
        builder.MakeCompound(ref compound);
        compound.TShape()!.FreezeGraph();
        SetOwnShape(index, compound);
        return Label(index);
    }

    /// <summary>Sets the label's shape (frozen here, if it isn't yet), as <c>XCAFDoc_ShapeTool::SetShape</c>.</summary>
    public void SetShape(in Label label, in Shape shape)
    {
        int index = this.Index(label);
        shape.TShape()?.FreezeGraph();
        _nodes[index].IsDirty = false;
        SetOwnShape(index, shape);
        _shapeLabels.TryAdd(shape, index);
    }

    /// <summary>
    /// Adds the shape (frozen here, if it isn't yet) at the top level, or finds its label (<c>XCAFDoc_ShapeTool::AddShape</c>):
    /// a located shape becomes a reference to its unlocated shape's label; a compound becomes an assembly of its
    /// sub-shapes if <paramref name="makeAssembly"/>.
    /// </summary>
    public Label AddShape(in Shape shape, bool makeAssembly = true, bool makePrepare = true)
    {
        _ = makePrepare; // decision 1231: the compound isn't modified
        if (!shape.IsNull)
        {
            shape.TShape()?.FreezeGraph();
        }
        int index = AddShapeCore(shape, makeAssembly);
        if (index >= 0)
        {
            _shapeLabels.TryAdd(shape, index);
        }
        return Label(index);
    }

    private int AddShapeCore(in Shape shape, bool makeAssembly)
    {
        if (shape.IsNull)
        {
            return -1;
        }
        if (FindShape(shape, out Label found, true))
        {
            return found.Index;
        }
        int shapeIndex = NewChild(ShapesIndex);
        if (!shape.Placement.IsIdentity)
        {
            int referred = AddShapeCore(shape.Located(new Placement()), makeAssembly);
            MakeReference(shapeIndex, referred, shape.Placement);
            return shapeIndex;
        }

        SetOwnShape(shapeIndex, shape);
        if (AutoNaming)
        {
            // SetLabelNameByShape: the shape type, as TopAbs::Print writes it
            SetAutoName(shapeIndex, global::Kerf.Engine.Topology.TopAbs.ShapeTypeToString(shape.Type));
        }
        if (makeAssembly && shape.Type == ShapeType.Compound)
        {
            _nodes[shapeIndex].IsAssembly = true;
            if (AutoNaming)
            {
                SetNameCore(shapeIndex, "ASSEMBLY");
            }
            for (var it = new Iterator(shape); it.More(); it.Next())
            {
                Shape comp = it.Value();
                int compIndex = AddShapeCore(comp.Located(new Placement()), makeAssembly);
                int aRef = NewChild(shapeIndex);
                MakeReference(aRef, compIndex, comp.Placement);
            }
            // the compound given is its components' compound already; searching the components' labels above may
            // have remade it from the components added so far (unit 8G1): it is set back
            _nodes[shapeIndex].IsDirty = false;
            if (!ReferenceEquals(_nodes[shapeIndex].Shape.TShape(), shape.TShape()))
            {
                SetOwnShape(shapeIndex, shape);
            }
        }
        if (!_nodes[shapeIndex].IsAssembly)
        {
            IndexedSet<Shape> map = SubShapeMap(shapeIndex)!;
            for (int i = 0; i < map.Count; i++)
            {
                Shape sub = map[i];
                _subShapes[sub] = shapeIndex;
                if (!sub.Placement.IsIdentity)
                {
                    _subShapes[sub.Located(new Placement())] = shapeIndex;
                }
            }
        }
        return shapeIndex;
    }

    private void MakeReference(int index, int referred, in Placement location, bool toName = true)
    {
        Node node = _nodes[index];
        node.Referred = referred;
        node.RefLocation = location;
        _nodes[referred].Users.Add(index);
        if (toName && AutoNaming)
        {
            // SetLabelNameByLink
            SetNameCore(index, "=>[" + EntryOf(Label(referred)) + "]");
        }
        if (node.Father == ShapesIndex)
        {
            Shape shape = GetShapeCore(index);
            if (!shape.IsNull)
            {
                _topLevelShapes.TryAdd(shape, index);
            }
        }
        int father = node.Father;
        if (father >= 0 && _nodes[father].IsAssembly)
        {
            _nodes[father].IsDirty = true;
            MarkUsersDirty(father);
        }
    }

    /// <summary>
    /// Adds a component to the assembly (a simple shape becomes an assembly): a reference to the label at the location
    /// (<c>XCAFDoc_ShapeTool::AddComponent</c>). The null label if the father is neither, or if the component contains
    /// the assembly (a cycle, which XCAF doesn't check).
    /// </summary>
    public Label AddComponent(in Label assembly, in Label component, in Placement location)
    {
        int anAssembly = this.Index(assembly);
        int aComponent = this.Index(component);
        if (Reaches(aComponent, anAssembly))
        {
            return default;
        }
        if (!_nodes[anAssembly].IsAssembly)
        {
            if (!IsSimpleShape(assembly))
            {
                return default;
            }
            _nodes[anAssembly].IsAssembly = true;
        }
        int aRef = NewChild(anAssembly);
        MakeReference(aRef, aComponent, location);
        Shape shape = GetShapeCore(aRef);
        if (!shape.IsNull)
        {
            _shapeLabels.TryAdd(shape, aRef);
        }
        return Label(aRef);
    }

    // True if the label is the target, or refers to it, or has a component reaching it
    private bool Reaches(int theFrom, int target)
    {
        var stack = new Stack<int>();
        var seen = new HashSet<int>();
        stack.Push(theFrom);
        while (stack.Count > 0)
        {
            int index = stack.Pop();
            if (index == target)
            {
                return true;
            }
            if (!seen.Add(index))
            {
                continue;
            }
            Node node = _nodes[index];
            if (node.Referred >= 0)
            {
                stack.Push(node.Referred);
            }
            if (node.IsAssembly)
            {
                foreach (int child in node.Children)
                {
                    if (_nodes[child].Referred >= 0)
                    {
                        stack.Push(child);
                    }
                }
            }
        }
        return false;
    }

    /// <summary>Adds the shape at the top level (an assembly if <paramref name="expand"/>) and as a component.</summary>
    public Label AddComponent(in Label assembly, in Shape component, bool expand = false)
    {
        Label compLabel = AddShape(component.Located(new Placement()), expand);
        return AddComponent(assembly, compLabel, component.Placement);
    }

    /// <summary>Remakes the assemblies' compounds from their components (<c>XCAFDoc_ShapeTool::UpdateAssemblies</c>).</summary>
    public void UpdateAssemblies()
    {
        foreach (int top in _nodes[ShapesIndex].Children.ToList())
        {
            GetShapeCore(top);
        }
    }

    /// <summary>Sets the top-level shape labels (those holding a shape) in <paramref name="labels"/>.</summary>
    public void GetShapes(NCollection_Sequence<Label> labels)
    {
        labels.Clear();
        foreach (int top in _nodes[ShapesIndex].Children)
        {
            if (!GetShapeCore(top).IsNull)
            {
                labels.Append(Label(top));
            }
        }
    }

    /// <summary>Sets the free top-level shape labels (the roots) in <paramref name="labels"/>.</summary>
    public void GetFreeShapes(NCollection_Sequence<Label> labels)
    {
        labels.Clear();
        foreach (int top in _nodes[ShapesIndex].Children)
        {
            if (_nodes[top].Users.Count == 0 && !GetShapeCore(top).IsNull)
            {
                labels.Append(Label(top));
            }
        }
    }

    /// <summary>The free shapes: the only one, or a compound of them (<c>XCAFDoc_ShapeTool::GetOneShape</c>).</summary>
    public Shape GetOneShape()
    {
        var free = new NCollection_Sequence<Label>();
        GetFreeShapes(free);
        if (free.Length() == 1)
        {
            return GetShape(free.Value(1));
        }
        var builder = new BrepBuilder();
        Compound compound = new();
        builder.MakeCompound(ref compound);
        foreach (Label label in free)
        {
            Shape shape = GetShape(label);
            if (!shape.IsNull)
            {
                builder.Add(compound, shape);
            }
        }
        compound.TShape()!.FreezeGraph();
        return compound;
    }

    /// <summary>
    /// Appends the references to the label (and, if <paramref name="getsubchilds"/>, first those to their fathers, up
    /// the tree) to <paramref name="labels"/>; returns their number (<c>XCAFDoc_ShapeTool::GetUsers</c>).
    /// </summary>
    public int GetUsers(in Label label, NCollection_Sequence<Label> labels, bool getsubchilds = false)
    {
        int count = 0;
        foreach (int user in NodeOf(label).Users)
        {
            if (getsubchilds)
            {
                int father = _nodes[user].Father;
                if (father >= 0)
                {
                    count += GetUsers(Label(father), labels, getsubchilds);
                }
            }
            labels.Append(Label(user));
            count++;
        }
        return count;
    }

    /// <summary>The label's location: a reference's, or its shape's.</summary>
    public Placement GetLocation(in Label label)
    {
        Node node = NodeOf(label);
        return node.Referred >= 0 ? node.RefLocation : node.Shape.IsNull ? new Placement() : node.Shape.Placement;
    }

    /// <summary>The label a reference refers to; false if the label isn't a reference.</summary>
    public bool GetReferredShape(in Label label, out Label referred)
    {
        referred = default;
        if (!IsReference(label))
        {
            return false;
        }
        referred = Label(NodeOf(label).Referred);
        return true;
    }

    /// <summary>The number of an assembly's components (with those of its sub-assemblies if <paramref name="getsubchilds"/>).</summary>
    public int NbComponents(in Label label, bool getsubchilds = false)
    {
        var labels = new NCollection_Sequence<Label>();
        GetComponents(label, labels, getsubchilds);
        return labels.Length();
    }

    /// <summary>
    /// Appends an assembly's components to <paramref name="labels"/> (each after those of its referred assembly if
    /// <paramref name="getsubchilds"/>); false if the label isn't an assembly.
    /// </summary>
    public bool GetComponents(in Label label, NCollection_Sequence<Label> labels, bool getsubchilds = false)
    {
        if (!IsAssembly(label))
        {
            return false;
        }
        foreach (int child in NodeOf(label).Children)
        {
            Label comp = Label(child);
            if (IsComponent(comp))
            {
                if (getsubchilds && GetReferredShape(comp, out Label under))
                {
                    GetComponents(under, labels, getsubchilds);
                }
                labels.Append(comp);
            }
        }
        return true;
    }

    // ---------------------------------------------------------------------------------------------- shape tool: components

    // XCAFDoc_ShapeTool.cxx's IsSame: the same TShape, orientation and location
    private static bool IsSameInstance(in Shape shape1, in Shape shape2)
        => ReferenceEquals(shape1.TShape(), shape2.TShape()) && shape1.Orientation == shape2.Orientation
           && shape1.Placement.IsEqual(shape2.Placement);

    // XCAFDoc_ShapeTool.cxx's checkForShape: the component, or a chain of components up the assemblies, placing the
    // shape where the given shape is
    private bool CheckForShape(in Shape shape, in Shape curSh, int userL,
                               NCollection_Sequence<Label> labels, int depth)
    {
        if (depth > _nodes.Count)
        {
            return false;
        }
        // the label of an assembly which contains this component
        int superUserL = _nodes[userL].Father;
        if (superUserL < 0)
        {
            return false;
        }
        Placement supLoc = GetLocation(Label(superUserL));
        Placement compLoc = GetLocation(Label(userL));
        compLoc = compLoc.Multiplied(curSh.Placement);
        supLoc = supLoc.Multiplied(compLoc);
        Shape copySh = curSh.Located(supLoc);
        if (IsSameInstance(shape, copySh))
        {
            labels.Prepend(Label(userL));
            return true;
        }
        // try to search deeply (upper by assembly structure)
        var newLabels = new NCollection_Sequence<Label>();
        for (int j = 1; j <= labels.Length(); j++)
        {
            newLabels.Append(labels.Value(j));
        }
        newLabels.Prepend(Label(userL));
        var users = new NCollection_Sequence<Label>();
        GetUsers(Label(superUserL), users);
        for (int i = 1; i <= users.Length(); i++)
        {
            if (CheckForShape(shape, copySh, users.Value(i).Index, newLabels, depth + 1))
            {
                // get solution
                labels.Clear();
                foreach (Label label in newLabels)
                {
                    labels.Append(label);
                }
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// The chain of component labels placing a located shape (<c>XCAFDoc_ShapeTool::FindComponent</c>): from the
    /// top-level assembly's component down to the one referring to the shape's own label; false if there is none
    /// (unit 8G2).
    /// </summary>
    public bool FindComponent(in Shape shape, NCollection_Sequence<Label> labels)
    {
        labels.Clear();
        // search for a top-level shape that corresponds to this component
        Shape s0 = shape.Located(new Placement());
        Label refL = FindShape(s0);
        if (refL.IsNull)
        {
            return false; // cannot find top-level shape.
        }

        var users = new NCollection_Sequence<Label>();
        GetUsers(refL, users);
        for (int i = 1; i <= users.Length(); i++)
        {
            if (CheckForShape(shape, s0, users.Value(i).Index, labels, 0))
            {
                break;
            }
        }

        return labels.Length() > 0;
    }

    /// <summary>
    /// Sets the colour of an instance of a component (<c>XCAFDoc_ColorTool::SetInstanceColor</c>): the colour of the
    /// component found by <see cref="FindComponent"/>. False if there is none, or if the instance is a component of a
    /// sub-assembly placed in an assembly (XCAF's SHUO, which the document hasn't: decision 1278).
    /// </summary>
    public bool SetInstanceColor(in Shape shape, ColorType type, in ColorWithAlpha color)
    {
        // find shuo label structure
        var labels = new NCollection_Sequence<Label>();
        if (!FindComponent(shape, labels))
        {
            return false;
        }
        if (labels.Length() == 1)
        {
            // set color directly for component as NAUO
            SetColor(labels.Value(1), color, type);
            return true;
        }
        // OCCT: a SHUO (SetSHUO) carries the colour; not there
        return false;
    }

    /// <summary>As <see cref="SetInstanceColor(in Shape, ColorType, in ColorWithAlpha)"/>, opaque.</summary>
    public bool SetInstanceColor(in Shape shape, ColorType type, in LinearColor color)
        => SetInstanceColor(shape, type, new ColorWithAlpha(color));

    // ---------------------------------------------------------------------------------------------- labels: OCAF's tree

    /// <summary>True if the label has children (<c>TDF_Label::HasChild</c>).</summary>
    public bool HasChild(in Label label) => NodeOf(label).Children.Count > 0;

    /// <summary>The number of the label's children (<c>TDF_Label::NbChildren</c>).</summary>
    public int NbChildren(in Label label) => NodeOf(label).Children.Count;

    /// <summary>The label's child of the tag, or the null label (<c>TDF_Label::FindChild(tag, false)</c>).</summary>
    public Label FindChild(in Label label, int tag)
    {
        foreach (int child in NodeOf(label).Children)
        {
            if (_nodes[child].Tag == tag)
            {
                return Label(child);
            }
        }
        return default;
    }

    /// <summary>True if the label is the other one or below it (<c>TDF_Label::IsDescendant</c>).</summary>
    public bool IsDescendant(in Label label, in Label ancestor)
    {
        if (!IsMine(label) || !IsMine(ancestor))
        {
            return false;
        }
        for (int i = label.Index; i >= 0; i = _nodes[i].Father)
        {
            if (i == ancestor.Index)
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>The label's descendants, depth first, in order (<c>TDF_ChildIterator(label, true)</c>).</summary>
    public IReadOnlyList<Label> Descendants(in Label label)
    {
        var result = new List<Label>();
        var stack = new Stack<int>();
        List<int> root = NodeOf(label).Children;
        for (int i = root.Count - 1; i >= 0; i--)
        {
            stack.Push(root[i]);
        }
        while (stack.Count > 0)
        {
            int index = stack.Pop();
            result.Add(Label(index));
            List<int> children = _nodes[index].Children;
            for (int i = children.Count - 1; i >= 0; i--)
            {
                stack.Push(children[i]);
            }
        }
        return result;
    }

    // ---------------------------------------------------------------------------------------------- shape tool: sub-shapes

    /// <summary>The sub-shape label of the shape under the shape label; false if there is none.</summary>
    public bool FindSubShape(in Label shapeLabel, in Shape sub, out Label label)
    {
        label = default;
        if (sub.IsNull)
        {
            return false;
        }
        Node node = NodeOf(shapeLabel);
        if (node.SubShapeLabels is not null && node.SubShapeLabels.TryGetValue(sub, out int index))
        {
            label = Label(index);
            return true;
        }
        return false;
    }

    /// <summary>
    /// Adds a sub-shape label for the shape under a top-level simple shape (<c>XCAFDoc_ShapeTool::AddSubShape</c>);
    /// the existing label if there is one; the null label if the shape isn't a sub-shape.
    /// </summary>
    public Label AddSubShape(in Label shapeLabel, in Shape sub)
    {
        AddSubShape(shapeLabel, sub, out Label label);
        return label;
    }

    /// <summary>
    /// As <see cref="AddSubShape(in Label, in Shape)"/>, true only if a label was added (false if it
    /// existed, with the label set, or if none can be).
    /// </summary>
    public bool AddSubShape(in Label shapeLabel, in Shape sub, out Label added)
    {
        added = default;
        if (!IsSimpleShape(shapeLabel) || !IsTopLevel(shapeLabel))
        {
            return false;
        }
        Shape subShape = sub;
        if (!IsSubShape(shapeLabel, sub))
        {
            bool isDefined = false;
            // the sub-shape without its location: take it with its location in the main shape
            if (sub.Placement.IsIdentity)
            {
                for (var it = new Iterator(GetShape(shapeLabel)); it.More() && !isDefined; it.Next())
                {
                    Shape shape = it.Value();
                    if (sub.IsSame(shape.Located(new Placement())))
                    {
                        isDefined = true;
                        subShape = shape;
                    }
                }
            }
            if (!isDefined)
            {
                return false;
            }
        }
        if (FindSubShape(shapeLabel, subShape, out added))
        {
            return false;
        }
        int father = this.Index(shapeLabel);
        int index = NewChild(father);
        subShape.TShape()?.FreezeGraph();
        _nodes[index].Shape = subShape;
        (_nodes[father].SubShapeLabels ??= new(ShapeHasher.Instance)).TryAdd(subShape, index);
        added = Label(index);
        return true;
    }

    /// <summary>Appends the label's sub-shape labels to <paramref name="labels"/>; false if it has none.</summary>
    public bool GetSubShapes(in Label label, NCollection_Sequence<Label> labels)
    {
        foreach (int child in NodeOf(label).Children)
        {
            if (IsSubShape(Label(child)))
            {
                labels.Append(Label(child));
            }
        }
        return labels.Length() > 0;
    }

    /// <summary>Records the external files the label's shape was read from (<c>XCAFDoc_ShapeTool::SetExternRefs</c>).</summary>
    public void SetExternRefs(in Label label, IEnumerable<string> refs)
    {
        Node node = NodeOf(label);
        node.Attributes = node.Attributes with { ExternRefs = [.. refs] };
    }

    /// <summary>Appends the external files of the label to <paramref name="refs"/>.</summary>
    public void GetExternRefs(in Label label, NCollection_Sequence<string> refs)
    {
        foreach (string aRef in NodeOf(label).Attributes.ExternRefs)
        {
            refs.Append(aRef);
        }
    }

    // ---------------------------------------------------------------------------------------------- removal

    private static readonly ColorType[] ColorTypes =
        [ColorType.ColorGen, ColorType.ColorSurf, ColorType.ColorCurv];

    /// <summary>
    /// Removes a free top-level label's attributes (<c>XCAFDoc_ShapeTool::RemoveShape</c>): the label and its children
    /// stay, empty. With <paramref name="removeCompletely"/>, a reference's referred label is removed too, if it is
    /// free then. False if the label isn't a free top-level label.
    /// </summary>
    public bool RemoveShape(in Label label, bool removeCompletely = true)
    {
        if (!IsTopLevel(label) || !IsFree(label))
        {
            return false;
        }
        int index = this.Index(label);
        int referred = removeCompletely ? _nodes[index].Referred : -1;
        Forget(index, true);
        return referred < 0 || RemoveShape(Label(referred));
    }

    /// <summary>Removes a component's attributes (<c>XCAFDoc_ShapeTool::RemoveComponent</c>); nothing for another label.</summary>
    public void RemoveComponent(in Label label)
    {
        if (IsComponent(label))
        {
            Forget(this.Index(label), true);
        }
    }

    /// <summary>
    /// Removes the label's attributes, and its children's (<c>TDF_Label::ForgetAllAttributes</c>): its shape, its
    /// reference (the referred label loses a user, the assemblies using it are remade), its name, colours, layers,
    /// properties; a colour label's colour (the shapes of that colour lose it); a layer's shapes leave it.
    /// </summary>
    public void ForgetAllAttributes(in Label label, bool clearChildren = true) => Forget(this.Index(label), clearChildren);

    private void Forget(int index, bool clearChildren)
    {
        Node node = _nodes[index];
        if (clearChildren)
        {
            foreach (int child in node.Children)
            {
                Forget(child, true);
            }
        }
        // a reference: the referred label loses a user; the father assembly is remade
        if (node.Referred >= 0)
        {
            _nodes[node.Referred].Users.Remove(index);
            node.Referred = -1;
            node.RefLocation = new Placement();
            if (node.Father >= 0 && _nodes[node.Father].IsAssembly)
            {
                _nodes[node.Father].IsDirty = true;
                MarkUsersDirty(node.Father);
            }
        }
        // the references to this label lose their father (TDataStd_TreeNode::BeforeForget)
        foreach (int user in node.Users)
        {
            _nodes[user].Referred = -1;
        }
        node.Users.Clear();
        // a colour label: the shapes linked to it lose the colour
        if (node.Color is not null)
        {
            for (int i = 0; i < _nodes.Count; i++)
            {
                if (_nodes[i].ColorRefs is int[] refs)
                {
                    for (int type = 0; type < 3; type++)
                    {
                        if (refs[type] == index)
                        {
                            UnSetColorCore(i, (ColorType)type);
                        }
                    }
                }
            }
            node.Color = null;
        }
        UnSetLayers(Label(index));
        if (node.LayerShapes is List<int> shapes)
        {
            foreach (int shape in shapes)
            {
                _nodes[shape].Layers?.Remove(index);
            }
            node.LayerShapes = null;
        }
        if (!node.Shape.IsNull)
        {
            RemoveFromMaps(index);
        }
        node.Shape = new Shape();
        node.IsAssembly = false;
        node.IsDirty = false;
        node.Attributes = Attributes.Empty;
        node.HasName = false;
        node.ColorRefs = null;
        node.SubShapeMap = null;
        node.SubShapeLabels = null;
        if (node.Father >= 0 && _nodes[node.Father].SubShapeLabels is { } subLabels)
        {
            foreach (KeyValuePair<Shape, int> entry in subLabels.Where(e => e.Value == index).ToList())
            {
                subLabels.Remove(entry.Key);
            }
        }
    }

    // The maps of shapes added no longer give the label (XCAF's keep stale entries; nothing here relies on them)
    private void RemoveFromMaps(int index)
    {
        foreach (Dictionary<Shape, int> map in new[] { _topLevelShapes, _shapeLabels, _subShapes })
        {
            foreach (KeyValuePair<Shape, int> entry in map.Where(e => e.Value == index).ToList())
            {
                map.Remove(entry.Key);
            }
        }
    }

    // ---------------------------------------------------------------------------------------------- names

    /// <summary>Sets the label's name (<c>TDataStd_Name::Set</c>).</summary>
    public void SetName(in Label label, string name) => SetNameCore(this.Index(label), name);

    private void SetNameCore(int index, string name)
    {
        Node node = _nodes[index];
        node.Attributes = node.Attributes.WithName(name);
        node.HasName = true;
    }

    // XCAF's SetLabelNameByShape names only a label without a name
    private void SetAutoName(int index, string name)
    {
        if (!_nodes[index].HasName)
        {
            SetNameCore(index, name);
        }
    }

    /// <summary>Removes the label's name (<c>TDF_Label::ForgetAttribute</c> of <c>TDataStd_Name</c>).</summary>
    public void UnSetName(in Label label)
    {
        Node node = NodeOf(label);
        node.Attributes = node.Attributes.WithName(null);
        node.HasName = false;
    }

    /// <summary>The label's name; false if it has none.</summary>
    public bool GetName(in Label label, out string? name)
    {
        name = null;
        if (!IsMine(label))
        {
            return false;
        }
        Node node = NodeOf(label);
        name = node.Attributes.Name;
        return node.HasName;
    }

    // ---------------------------------------------------------------------------------------------- colour tool

    /// <summary>The label of the colours (0:1:2), XCAF's <c>XCAFDoc_ColorTool::Label()</c>.</summary>
    public Label ColorsLabel() => new(this, ColorsIndex);

    /// <summary>True if the label is a colour label (<c>XCAFDoc_ColorTool::IsColor</c>).</summary>
    public bool IsColor(in Label label) => IsMine(label) && NodeOf(label).Color is not null;

    /// <summary>The colour of a colour label; false if the label isn't one (<c>XCAFDoc_ColorTool::GetColor(lab, col)</c>).</summary>
    public bool GetColor(in Label colorLabel, out ColorWithAlpha color)
    {
        color = default;
        if (!IsMine(colorLabel) || NodeOf(colorLabel).Color is not ColorWithAlpha aColor)
        {
            return false;
        }
        color = aColor;
        return true;
    }

    /// <summary>
    /// The first colour label of a colour equal to the colour (<c>Quantity_ColorRGBA::IsEqual</c>), or the null label
    /// (<c>XCAFDoc_ColorTool::FindColor</c>).
    /// </summary>
    public Label FindColor(in ColorWithAlpha color)
    {
        foreach (int child in _nodes[ColorsIndex].Children)
        {
            if (_nodes[child].Color is ColorWithAlpha aColor && aColor.IsEqual(color))
            {
                return Label(child);
            }
        }
        return default;
    }

    /// <summary>
    /// The colour label of the colour, added under 0:1:2 if there is none (<c>XCAFDoc_ColorTool::AddColor</c>), named
    /// as XCAF's colour auto-naming does: "RED (#FF0000)".
    /// </summary>
    public Label AddColor(in ColorWithAlpha color)
    {
        Label found = FindColor(color);
        if (!found.IsNull)
        {
            return found;
        }
        int index = NewChild(ColorsIndex);
        _nodes[index].Color = color;
        SetNameCore(index, LinearColor.StringName(color.GetRGB().Name()) + " (" + ColorWithAlpha.ColorToHex(color) + ")");
        return Label(index);
    }

    /// <summary>
    /// Removes a colour label's attributes (<c>XCAFDoc_ColorTool::RemoveColor</c>): the label stays, empty; the shapes
    /// of that colour lose it.
    /// </summary>
    public void RemoveColor(in Label colorLabel) => Forget(this.Index(colorLabel), true);

    /// <summary>Sets the colour labels in <paramref name="labels"/> (<c>XCAFDoc_ColorTool::GetColors</c>).</summary>
    public void GetColors(NCollection_Sequence<Label> labels)
    {
        labels.Clear();
        foreach (int child in _nodes[ColorsIndex].Children)
        {
            if (_nodes[child].Color is not null)
            {
                labels.Append(Label(child));
            }
        }
    }

    /// <summary>
    /// Sets the label's colour of the type (<c>XCAFDoc_ColorTool::SetColor</c>): a link to the colour's label, added if
    /// there is none; the label's colour is then the colour label's.
    /// </summary>
    public void SetColor(in Label label, in ColorWithAlpha color, ColorType type)
        => SetColorRef(this.Index(label), AddColor(color).Index, type);

    /// <summary>Sets the label's colour of the type to the colour label's (<c>XCAFDoc_ColorTool::SetColor(L, colorL, type)</c>).</summary>
    public void SetColor(in Label label, in Label colorLabel, ColorType type)
    {
        int color = this.Index(colorLabel);
        if (_nodes[color].Color is null)
        {
            throw new ArgumentException("Not a colour label", nameof(colorLabel));
        }
        SetColorRef(this.Index(label), color, type);
    }

    private void SetColorRef(int index, int colorIndex, ColorType type)
    {
        Node node = _nodes[index];
        (node.ColorRefs ??= [-1, -1, -1])[(int)type] = colorIndex;
        node.Attributes = node.Attributes.WithColor(type, _nodes[colorIndex].Color);
    }

    /// <summary>The colour label the label's colour of the type links to; false if none.</summary>
    public bool GetColor(in Label label, ColorType type, out Label colorLabel)
    {
        colorLabel = default;
        if (!IsMine(label) || NodeOf(label).ColorRefs is not int[] refs || refs[(int)type] < 0)
        {
            return false;
        }
        colorLabel = Label(refs[(int)type]);
        return true;
    }

    /// <summary>Sets the label's colour of the type, opaque.</summary>
    public void SetColor(in Label label, in LinearColor color, ColorType type)
        => SetColor(label, new ColorWithAlpha(color), type);

    /// <summary>Sets the colour of the shape's label, found by <see cref="Search"/>; false if not found.</summary>
    public bool SetColor(in Shape shape, in ColorWithAlpha color, ColorType type)
    {
        if (!Search(shape, out Label label))
        {
            return false;
        }
        SetColor(label, color, type);
        return true;
    }

    /// <summary>The label's colour of the type; false if not set.</summary>
    public bool GetColor(in Label label, ColorType type, out ColorWithAlpha color)
    {
        color = default;
        if (!IsMine(label) || NodeOf(label).Attributes.Color(type) is not ColorWithAlpha aColor)
        {
            return false;
        }
        color = aColor;
        return true;
    }

    /// <summary>The label's colour of the type, without its alpha; false if not set.</summary>
    public bool GetColor(in Label label, ColorType type, out LinearColor color)
    {
        bool isSet = GetColor(label, type, out ColorWithAlpha aColor);
        color = aColor.GetRGB();
        return isSet;
    }

    /// <summary>The colour of the shape's label (found by <see cref="Search"/>); false if not found or not set.</summary>
    public bool GetColor(in Shape shape, ColorType type, out ColorWithAlpha color)
    {
        color = default;
        return Search(shape, out Label label) && GetColor(label, type, out color);
    }

    /// <summary>True if the label has a colour of the type.</summary>
    public bool IsSet(in Label label, ColorType type)
        => IsMine(label) && NodeOf(label).Attributes.Color(type) is not null;

    /// <summary>Removes the label's colour of the type.</summary>
    public void UnSetColor(in Label label, ColorType type) => UnSetColorCore(this.Index(label), type);

    private void UnSetColorCore(int index, ColorType type)
    {
        Node node = _nodes[index];
        if (node.ColorRefs is int[] refs)
        {
            refs[(int)type] = -1;
        }
        node.Attributes = node.Attributes.WithColor(type, null);
    }

    /// <summary>Shows or hides the shape or layer of the label (<c>XCAFDoc_ColorTool</c>'s and <c>XCAFDoc_LayerTool</c>'s <c>SetVisibility</c>).</summary>
    public void SetVisibility(in Label label, bool isVisible = true)
    {
        Node node = NodeOf(label);
        node.Attributes = node.Attributes with { IsVisible = isVisible };
    }

    /// <summary>False if the shape or layer of the label is hidden.</summary>
    public bool IsVisible(in Label label) => !IsMine(label) || NodeOf(label).Attributes.IsVisible;

    // ---------------------------------------------------------------------------------------------- layer tool

    /// <summary>The first layer of the name (<c>XCAFDoc_LayerTool::FindLayer</c>), or the null label.</summary>
    public Label FindLayer(string layer, bool toFindWithProperty = false, bool toFindVisible = true)
    {
        foreach (int child in _nodes[LayersIndex].Children)
        {
            Node node = _nodes[child];
            if (node.Attributes.Name == layer && (!toFindWithProperty || node.Attributes.IsVisible == toFindVisible))
            {
                return Label(child);
            }
        }
        return default;
    }

    /// <summary>The first layer of the name; false if there is none.</summary>
    public bool FindLayer(string layer, out Label label)
    {
        label = FindLayer(layer);
        return !label.IsNull;
    }

    /// <summary>The layer of the name, added if there is none (<c>XCAFDoc_LayerTool::AddLayer</c>).</summary>
    public Label AddLayer(string layer)
    {
        Label found = FindLayer(layer);
        return found.IsNull ? NewLayer(layer) : found;
    }

    /// <summary>
    /// The layer of the name and visibility, added if there is none (<c>XCAFDoc_LayerTool::AddLayer(name, visible)</c>:
    /// the visibility is only searched; the caller sets it).
    /// </summary>
    public Label AddLayer(string layer, bool toFindVisible)
    {
        Label found = FindLayer(layer, true, toFindVisible);
        return found.IsNull ? NewLayer(layer) : found;
    }

    private Label NewLayer(string layer)
    {
        int index = NewChild(LayersIndex);
        _nodes[index].Attributes = Attributes.Empty.WithName(layer);
        _nodes[index].HasName = true;
        return Label(index);
    }

    /// <summary>The layers' labels, in order (removed layers left out).</summary>
    public void GetLayerLabels(NCollection_Sequence<Label> labels)
    {
        labels.Clear();
        foreach (int child in _nodes[LayersIndex].Children)
        {
            if (_nodes[child].HasName)
            {
                labels.Append(Label(child));
            }
        }
    }

    /// <summary>True if the label is a named layer (<c>XCAFDoc_LayerTool::IsLayer</c>).</summary>
    public bool IsLayer(in Label label) => GetLayer(label, out _);

    /// <summary>
    /// Removes a layer's attributes (<c>XCAFDoc_LayerTool::RemoveLayer</c>): its shapes leave it; the label stays,
    /// empty, and is no longer a layer.
    /// </summary>
    public void RemoveLayer(in Label layer) => Forget(this.Index(layer), true);

    /// <summary>Takes the shape label out of the first layer of the name; false if there is none or it isn't in it.</summary>
    public bool UnSetOneLayer(in Label label, string layer)
    {
        Label aLayer = FindLayer(layer);
        return !aLayer.IsNull && UnSetOneLayer(label, aLayer);
    }

    /// <summary>Takes the shape label out of the layer (<c>XCAFDoc_LayerTool::UnSetOneLayer</c>).</summary>
    public bool UnSetOneLayer(in Label label, in Label layer)
    {
        int shape = this.Index(label);
        int aLayer = this.Index(layer);
        if (_nodes[shape].Layers is not List<int> layers || !layers.Remove(aLayer))
        {
            return _nodes[shape].Layers is not null && _nodes[aLayer].LayerShapes is not null;
        }
        _nodes[aLayer].LayerShapes?.Remove(shape);
        return true;
    }

    /// <summary>The layer's name; false if the label isn't a named layer.</summary>
    public bool GetLayer(in Label layer, out string? name)
    {
        name = null;
        if (!IsMine(layer) || NodeOf(layer).Father != LayersIndex)
        {
            return false;
        }
        return GetName(layer, out name);
    }

    /// <summary>Puts the shape label in the layer (alone in it if <paramref name="shapeInOneLayer"/>).</summary>
    public void SetLayer(in Label label, in Label layer, bool shapeInOneLayer = false)
    {
        int shape = this.Index(label);
        int aLayer = this.Index(layer);
        if (shapeInOneLayer)
        {
            UnSetLayers(label);
        }
        List<int> layers = _nodes[shape].Layers ??= [];
        if (!layers.Contains(aLayer))
        {
            layers.Add(aLayer);
            (_nodes[aLayer].LayerShapes ??= []).Add(shape);
        }
    }

    /// <summary>Puts the shape label in the layer of the name (added if there is none).</summary>
    public void SetLayer(in Label label, string layer, bool shapeInOneLayer = false)
        => SetLayer(label, AddLayer(layer), shapeInOneLayer);

    /// <summary>Takes the shape label out of all its layers.</summary>
    public void UnSetLayers(in Label label)
    {
        int shape = this.Index(label);
        if (_nodes[shape].Layers is List<int> layers)
        {
            foreach (int layer in layers)
            {
                _nodes[layer].LayerShapes?.Remove(shape);
            }
            _nodes[shape].Layers = null;
        }
    }

    /// <summary>True if the shape label is in the layer.</summary>
    public bool IsSet(in Label label, in Label layer)
        => NodeOf(label).Layers?.Contains(this.Index(layer)) == true;

    /// <summary>Appends the shape label's layers to <paramref name="layers"/>; false if it has none.</summary>
    public bool GetLayers(in Label label, NCollection_Sequence<Label> layers)
    {
        if (NodeOf(label).Layers is not List<int> aLayers || aLayers.Count == 0)
        {
            return false;
        }
        foreach (int layer in aLayers)
        {
            layers.Append(Label(layer));
        }
        return true;
    }

    /// <summary>Appends the names of the shape label's layers to <paramref name="layers"/>; false if it has none.</summary>
    public bool GetLayers(in Label label, NCollection_Sequence<string> layers)
    {
        if (NodeOf(label).Layers is not List<int> aLayers || aLayers.Count == 0)
        {
            return false;
        }
        foreach (int layer in aLayers)
        {
            layers.Append(_nodes[layer].Attributes.Name ?? "");
        }
        return true;
    }

    /// <summary>Appends the shape labels in the layer to <paramref name="labels"/>.</summary>
    public void GetShapesOfLayer(in Label layer, NCollection_Sequence<Label> labels)
    {
        labels.Clear();
        foreach (int shape in NodeOf(layer).LayerShapes ?? [])
        {
            labels.Append(Label(shape));
        }
    }

    // ---------------------------------------------------------------------------------------------- validation properties

    /// <summary>Sets the label's volume (<c>XCAFDoc_Volume::Set</c>).</summary>
    public void SetVolume(in Label label, double volume)
        => SetProps(label, p => p with { Volume = volume });

    /// <summary>Sets the label's area (<c>XCAFDoc_Area::Set</c>).</summary>
    public void SetArea(in Label label, double area) => SetProps(label, p => p with { Area = area });

    /// <summary>Sets the label's centroid (<c>XCAFDoc_Centroid::Set</c>).</summary>
    public void SetCentroid(in Label label, in Point3D centroid)
    {
        Point3D aCentroid = centroid;
        SetProps(label, p => p with { Centroid = aCentroid });
    }

    /// <summary>The label's validation properties, or null.</summary>
    public DocumentValidationProps? GetValidationProps(in Label label)
        => IsMine(label) ? NodeOf(label).Attributes.ValidationProps : null;

    private void SetProps(in Label label, Func<DocumentValidationProps, DocumentValidationProps> change)
    {
        Node node = NodeOf(label);
        node.Attributes = node.Attributes with
        {
            ValidationProps = change(node.Attributes.ValidationProps ?? DocumentValidationProps.None),
        };
    }

    // ---------------------------------------------------------------------------------------------- attributes

    /// <summary>The label's attributes (name, colours, visibility, layer names, properties) as a tree carries them.</summary>
    public Attributes GetAttributes(in Label label)
    {
        Node node = NodeOf(label);
        Attributes attributes = node.Attributes;
        if (node.Layers is { Count: > 0 } layers)
        {
            attributes = attributes with
            {
                Layers = [.. layers.Select(l => _nodes[l].Attributes.Name ?? "").Distinct(StringComparer.Ordinal)],
            };
        }
        return attributes;
    }

    /// <summary>Sets the label's attributes (the colours and the layers through their labels, added if missing).</summary>
    public void SetAttributes(in Label label, Attributes attributes)
    {
        Node node = NodeOf(label);
        node.Attributes = attributes with { Layers = [], GenericColor = null, SurfaceColor = null, CurveColor = null };
        node.HasName = attributes.Name is not null;
        node.ColorRefs = null;
        foreach (ColorType type in ColorTypes)
        {
            if (attributes.Color(type) is ColorWithAlpha color)
            {
                SetColor(label, color, type);
            }
        }
        UnSetLayers(label);
        foreach (string layer in attributes.Layers)
        {
            SetLayer(label, layer);
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
    public Tree ToTree()
    {
        var products = new Dictionary<int, DocumentProduct>();
        var inProgress = new HashSet<int>();

        DocumentProduct ProductOf(int index)
        {
            if (products.TryGetValue(index, out DocumentProduct? known))
            {
                return known;
            }
            if (!inProgress.Add(index))
            {
                throw new InvalidOperationException("XdeDoc_Document: a cycle of references");
            }
            Node node = _nodes[index];
            Attributes attributes = GetAttributes(Label(index));
            DocumentProduct product;
            if (node.IsAssembly)
            {
                var components = new List<Instance>();
                foreach (int child in node.Children)
                {
                    if (_nodes[child].Referred >= 0)
                    {
                        components.Add(InstanceOf(child));
                    }
                }
                product = DocumentProduct.Assembly(components, attributes);
            }
            else
            {
                var subs = new List<SubShape>();
                foreach (int child in node.Children)
                {
                    Node sub = _nodes[child];
                    if (sub.Referred < 0 && !sub.Shape.IsNull)
                    {
                        subs.Add(new SubShape(sub.Shape, GetAttributes(Label(child))));
                    }
                }
                Shape shape = GetShapeCore(index);
                product = shape.IsNull
                             ? DocumentProduct.Assembly([], attributes)
                             : DocumentProduct.Part(shape, attributes, subs);
            }
            inProgress.Remove(index);
            products[index] = product;
            return product;
        }

        Instance InstanceOf(int theRef)
        {
            // a reference to a reference: the locations composed, the last product
            Placement location = _nodes[theRef].RefLocation;
            int referred = _nodes[theRef].Referred;
            int guard = 0;
            while (_nodes[referred].Referred >= 0)
            {
                if (++guard > _nodes.Count)
                {
                    throw new InvalidOperationException("XdeDoc_Document: a cycle of references");
                }
                location = location.Multiplied(_nodes[referred].RefLocation);
                referred = _nodes[referred].Referred;
            }
            return new Instance(ProductOf(referred), location, GetAttributes(Label(theRef)));
        }

        var roots = new List<Instance>();
        var free = new NCollection_Sequence<Label>();
        GetFreeShapes(free);
        foreach (Label root in free)
        {
            roots.Add(_nodes[root.Index].Referred >= 0
                         ? InstanceOf(root.Index)
                         : new Instance(ProductOf(root.Index)));
        }

        var layers = new List<Layer>();
        var names = new HashSet<string>(StringComparer.Ordinal);
        foreach (int child in _nodes[LayersIndex].Children)
        {
            if (!_nodes[child].HasName)
            {
                continue;
            }
            string name = _nodes[child].Attributes.Name ?? "";
            if (names.Add(name))
            {
                layers.Add(new Layer(name, _nodes[child].Attributes.IsVisible));
            }
        }
        return new Tree(roots, layers, LengthUnit);
    }

    /// <summary>
    /// A document holding the tree, as XCAF would after reading it: each product a top-level label (in depth-first
    /// order, a product before its components' products), each component a reference under its assembly, each
    /// sub-shape entry a sub-shape label; a root placed or carrying attributes of its own becomes a top-level
    /// reference (made before its product's label, as <c>XCAFDoc_ShapeTool::AddShape</c> does), another root is its
    /// product's label. The labels hold the tree's shapes, so the document's shapes are the tree's.
    /// </summary>
    public static ExchangeDocument FromTree(Tree tree)
    {
        var doc = new ExchangeDocument();
        doc.LengthUnit = tree.LengthUnit;
        foreach (Layer layer in tree.Layers)
        {
            Label label = doc.NewLayer(layer.Name);
            doc.SetVisibility(label, layer.IsVisible);
        }

        var labels = new Dictionary<DocumentProduct, int>(ReferenceEqualityComparer.Instance);
        int LabelOf(DocumentProduct product)
        {
            if (labels.TryGetValue(product, out int known))
            {
                return known;
            }
            int index = doc.NewChild(ShapesIndex);
            labels[product] = index;
            Node node = doc._nodes[index];
            doc.SetAttributes(doc.Label(index), product.Attributes);
            if (product.IsAssembly)
            {
                node.IsAssembly = true;
                foreach (Instance component in product.Components)
                {
                    int referred = LabelOf(component.Product);
                    int aRef = doc.NewChild(index);
                    doc.MakeReference(aRef, referred, component.Location, false);
                    doc.SetAttributes(doc.Label(aRef), component.Attributes);
                    Shape compShape = doc.GetShapeCore(aRef);
                    if (!compShape.IsNull)
                    {
                        doc._shapeLabels.TryAdd(compShape, aRef);
                    }
                }
                doc.SetOwnShape(index, product.Shape);
                node.IsDirty = false;
            }
            else
            {
                doc.SetOwnShape(index, product.Shape);
                foreach (SubShape sub in product.SubShapes)
                {
                    int subIndex = doc.NewChild(index);
                    doc._nodes[subIndex].Shape = sub.Shape;
                    (node.SubShapeLabels ??= new(ShapeHasher.Instance)).TryAdd(sub.Shape, subIndex);
                    doc.SetAttributes(doc.Label(subIndex), sub.Attributes);
                }
                IndexedSet<Shape> map = doc.SubShapeMap(index)!;
                for (int i = 0; i < map.Count; i++)
                {
                    doc._subShapes.TryAdd(map[i], index);
                }
            }
            doc._shapeLabels.TryAdd(product.Shape, index);
            return index;
        }

        foreach (Instance root in tree.Roots)
        {
            if (root.Location.IsIdentity && root.Attributes.IsEmpty)
            {
                LabelOf(root.Product);
                continue;
            }
            int aRef = doc.NewChild(ShapesIndex);
            int referred = LabelOf(root.Product);
            doc.MakeReference(aRef, referred, root.Location, false);
            doc.SetAttributes(doc.Label(aRef), root.Attributes);
        }
        return doc;
    }
}
