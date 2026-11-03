// Port of: src/Draw/TKXDEDRAW/XDEDRAW/XDEDRAW.cxx (XDumpAssemblyTree, XDumpAssemblyGraph, XDumpNomenclature),
//          src/DataExchange/TKXCAF/XCAFDoc/XCAFDoc_AssemblyIterator.cxx, XCAFDoc_AssemblyGraph.cxx, XCAFDoc_AssemblyItemId.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Foundation;
using System.Text;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// XDEDRAW's assembly dumps (unit 8G1), with the parts of XCAFDoc_AssemblyIterator and XCAFDoc_AssemblyGraph they
/// use, over Draw-lite's documents. The graph's adjacency lists (OCCT's TColStd_PackedMapOfInteger) are written in
/// ascending order.
/// </summary>
internal static class XDEDRAWAssemblyCommands
{
    // ---------------------------------------------------------------------------------------------- the iterator

    private readonly record struct AuxItem(List<string> Path, Label Label);

    private static AuxItem createItem(ExchangeDocument theDoc, in Label theLabel, IEnumerable<string> theParentPath)
    {
        var aPath = new List<string>(theParentPath) { theLabel.Entry() };
        return new AuxItem(aPath, theDoc.GetReferredShape(theLabel, out Label anOriginal) ? anOriginal : theLabel);
    }

    // XCAFDoc_AssemblyIterator with XCAFDoc_AssemblyTool::Traverse: the items' paths, depth first
    private static IEnumerable<List<string>> traverse(ExchangeDocument theDoc, List<string>? theRoot, int theMaxLevel)
    {
        var aFringe = new List<AuxItem>();
        int aSeedLevel = 1;
        if (theRoot is null)
        {
            var aRoots = new NCollection_Sequence<Label>();
            theDoc.GetFreeShapes(aRoots);
            foreach (Label aRoot in aRoots)
            {
                aFringe.Add(createItem(theDoc, aRoot, []));
            }
        }
        else
        {
            aSeedLevel = theRoot.Count;
            Label aSeedLabel = DDocStdCommands.Label(theDoc, theRoot[^1]);
            if (aSeedLabel.IsNull)
            {
                yield break;
            }
            List<string> aSeedPath = theRoot;
            if (theDoc.GetReferredShape(aSeedLabel, out Label anOriginal))
            {
                if (!theDoc.IsAssembly(aSeedLabel))
                {
                    aSeedLabel = anOriginal;
                }
                else
                {
                    // the path without its last entry (OCCT's ToString, SearchFromEnd ("/"), Remove)
                    string aPathStr = string.Join('/', theRoot);
                    int anIndex = aPathStr.LastIndexOf('/');
                    if (anIndex >= 0)
                    {
                        aPathStr = aPathStr[..anIndex];
                    }
                    aSeedPath = [.. aPathStr.Split('/', StringSplitOptions.RemoveEmptyEntries)];
                }
            }
            aFringe.Add(new AuxItem(aSeedPath, aSeedLabel));
        }

        while (aFringe.Count > 0)
        {
            AuxItem aCurrent = aFringe[^1];
            yield return aCurrent.Path;
            aFringe.RemoveAt(aFringe.Count - 1);
            int aCurrentDepth = aCurrent.Path.Count - aSeedLevel;
            if (aCurrentDepth < theMaxLevel)
            {
                var aComponents = new NCollection_Sequence<Label>();
                if (theDoc.IsAssembly(aCurrent.Label))
                {
                    theDoc.GetComponents(aCurrent.Label, aComponents);
                }
                else if (theDoc.IsComponent(aCurrent.Label))
                {
                    aComponents.Append(aCurrent.Label);
                }
                for (int l = aComponents.Length(); l >= 1; --l)
                {
                    aFringe.Add(createItem(theDoc, aComponents.Value(l), aCurrent.Path));
                }
            }
        }
    }

    internal static int XDumpAssemblyTree(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 2)
        {
            di.Append("Usage :\n ").Append(argv[0]).Append(" Doc [-root label] [-level l] [-names]\n")
              .Append("   Doc         - document name. \n")
              .Append("   -root label - starting root label. \n")
              .Append("   -level l    - depth level (infinite by default). \n")
              .Append("   -names      - prints names instead of entries. \n");
            return 1;
        }
        ExchangeDocument? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }

        List<string>? aRoot = null;
        int aLevel = int.MaxValue;
        bool aPrintNames = false;
        for (int iarg = 2; iarg < argc; ++iarg)
        {
            if (argv[iarg] == "-root")
            {
                if (iarg + 1 >= argc)
                {
                    throw new ProgramErrorException("Root is expected!");
                }
                // XCAFDoc_AssemblyItemId::Init (string): the entries separated by '/'
                aRoot = [.. argv[++iarg].Split('/', StringSplitOptions.RemoveEmptyEntries)];
                if (aRoot.Count == 0)
                {
                    throw new NullObjectException("Null assembly item!");
                }
            }
            else if (argv[iarg] == "-level")
            {
                if (iarg + 1 >= argc)
                {
                    throw new ProgramErrorException("Level is expected!");
                }
                if (!int.TryParse(argv[++iarg], System.Globalization.NumberStyles.AllowLeadingSign, System.Globalization.CultureInfo.InvariantCulture, out aLevel))
                {
                    throw new ProgramErrorException("Integer value is expected!");
                }
                if (aLevel < 0)
                {
                    throw new RangeErrorException("Null document!");
                }
            }
            else if (argv[iarg] == "-names")
            {
                aPrintNames = true;
            }
        }

        var aSS = new StringBuilder();
        foreach (List<string> aPath in traverse(aDoc, aRoot, aLevel))
        {
            if (aPrintNames)
            {
                bool aFirst = true;
                foreach (string anEntry in aPath)
                {
                    if (!aFirst)
                    {
                        aSS.Append('/');
                    }
                    aFirst = false;
                    Label aL = DDocStdCommands.Label(aDoc, anEntry);
                    if (!aL.IsNull && aDoc.GetName(aL, out string? aName))
                    {
                        aSS.Append(aName);
                        continue;
                    }
                    aSS.Append(anEntry);
                }
                aSS.Append('\n');
            }
            else
            {
                aSS.Append(string.Join('/', aPath)).Append('\n');
            }
        }
        di.Append(aSS.ToString());
        return 0;
    }

    // ---------------------------------------------------------------------------------------------- the graph

    private enum NodeType
    {
        UNDEFINED,
        AssemblyRoot,
        Subassembly,
        Occurrence,
        Part,
        Subshape,
    }

    private sealed class Graph
    {
        private readonly ExchangeDocument myDoc;
        public readonly List<Label> Nodes = [];                  // ids from 1
        private readonly Dictionary<Label, int> myIds = [];
        public readonly Dictionary<int, NodeType> Types = [];
        public readonly Dictionary<int, SortedSet<int>> Links = [];
        public readonly Dictionary<int, int> Usages = [];

        public Graph(ExchangeDocument theDoc, Label theLabel)
        {
            myDoc = theDoc;
            var aRoots = new NCollection_Sequence<Label>();
            if (theLabel.IsNull || theLabel == theDoc.ShapesLabel())
            {
                theDoc.GetFreeShapes(aRoots);
            }
            else
            {
                aRoots.Append(theLabel);
            }
            foreach (Label aLabel in aRoots)
            {
                int aRootId, anIdToProceed;
                if (!theDoc.GetReferredShape(aLabel, out Label anOriginal))
                {
                    anOriginal = aLabel;
                    aRootId = addNode(anOriginal, 0);
                    anIdToProceed = aRootId;
                }
                else
                {
                    aRootId = addNode(aLabel, 0);
                    if (aRootId == 0)
                    {
                        continue;
                    }
                    anIdToProceed = addNode(anOriginal, aRootId);
                }
                if (aRootId == 0 || anIdToProceed == 0)
                {
                    continue;
                }
                if (theDoc.IsAssembly(anOriginal))
                {
                    addComponents(anOriginal, anIdToProceed);
                }
            }
        }

        private void addComponents(in Label theParent, int theParentId)
        {
            if (!myDoc.IsShape(theParent))
            {
                return;
            }
            foreach (Label aComponent in myDoc.Children(theParent))
            {
                int aComponentId = addNode(aComponent, theParentId);
                if (aComponentId == 0)
                {
                    continue;
                }
                if (!myDoc.GetReferredShape(aComponent, out Label aChildOriginal))
                {
                    continue;
                }
                int aChildId = addNode(aChildOriginal, aComponentId);
                if (aChildId == 0)
                {
                    continue;
                }
                addComponents(aChildOriginal, aChildId);
            }
        }

        private int addNode(in Label theLabel, int theParentId)
        {
            NodeType aNodeType = NodeType.UNDEFINED;
            if (myDoc.IsAssembly(theLabel))
            {
                aNodeType = myDoc.IsFree(theLabel) ? NodeType.AssemblyRoot : NodeType.Subassembly;
            }
            else if (myDoc.IsReference(theLabel))
            {
                aNodeType = NodeType.Occurrence;
            }
            else if (myDoc.IsSubShape(theLabel))
            {
                aNodeType = NodeType.Subshape;
            }
            else if (myDoc.IsSimpleShape(theLabel))
            {
                aNodeType = NodeType.Part;
            }
            if (aNodeType == NodeType.UNDEFINED)
            {
                return 0;
            }
            if (!myIds.TryGetValue(theLabel, out int aChildId))
            {
                Nodes.Add(theLabel);
                aChildId = Nodes.Count;
                myIds[theLabel] = aChildId;
            }
            Types[aChildId] = aNodeType;
            if (aNodeType != NodeType.Occurrence)
            {
                Usages[aChildId] = Usages.TryGetValue(aChildId, out int aUsage) ? aUsage + 1 : 1;
            }
            if (theParentId > 0)
            {
                if (!Links.TryGetValue(theParentId, out SortedSet<int>? aSet))
                {
                    Links[theParentId] = aSet = [];
                }
                aSet.Add(aChildId);
            }
            return aChildId;
        }
    }

    private static string typeName(NodeType theType) => theType switch
    {
        NodeType.AssemblyRoot => "R",
        NodeType.Subassembly => "A",
        NodeType.Occurrence => "O",
        NodeType.Part => "P",
        NodeType.Subshape => "S",
        _ => "?",
    };

    private static string nodeEntry(ExchangeDocument theDoc, in Label theLabel, bool thePrintNames)
    {
        if (thePrintNames && theDoc.GetName(theLabel, out string? aName))
        {
            return "'" + aName + "'";
        }
        return theLabel.Entry();
    }

    internal static int XDumpAssemblyGraph(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 2)
        {
            di.Append("Usage :\n ").Append(argv[0]).Append(" Doc [-root label] [-verbose] \n")
              .Append("   Doc         - is the document name. \n")
              .Append("   -root label - is the optional starting label. \n")
              .Append("   -names      - prints names instead of entries. \n");
            return 1;
        }
        ExchangeDocument? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        bool aPrintNames = false;
        Label aLabel = aDoc.ShapesLabel();
        for (int iarg = 2; iarg < argc; ++iarg)
        {
            if (argv[iarg] == "-root")
            {
                if (iarg + 1 >= argc)
                {
                    throw new ProgramErrorException("Root is expected!");
                }
                aLabel = DDocStdCommands.Label(aDoc, argv[++iarg]);
            }
            else if (argv[iarg] == "-names")
            {
                aPrintNames = true;
            }
        }
        if (aLabel.IsNull)
        {
            throw new NullObjectException("Null label!");
        }

        var aG = new Graph(aDoc, aLabel);
        var aSS = new StringBuilder();
        for (int aNode = 1; aNode <= aG.Nodes.Count; aNode++)
        {
            aSS.Append(aNode).Append(' ').Append(typeName(aG.Types[aNode])).Append(' ').Append(nodeEntry(aDoc, aG.Nodes[aNode - 1], aPrintNames));
            if (aG.Links.TryGetValue(aNode, out SortedSet<int>? aLinks))
            {
                foreach (int aLink in aLinks)
                {
                    aSS.Append(' ').Append(aLink);
                }
            }
            aSS.Append('\n');
        }
        di.Append(aSS.ToString());
        return 0;
    }

    internal static int XDumpNomenclature(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 2)
        {
            di.Append("Usage :\n ").Append(argv[0]).Append(" Doc [-names] \n")
              .Append("   Doc    - is the document name. \n")
              .Append("   -names - prints names instead of entries. \n");
            return 1;
        }
        ExchangeDocument? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        bool aPrintNames = false;
        for (int iarg = 2; iarg < argc; ++iarg)
        {
            if (argv[iarg] == "-names")
            {
                aPrintNames = true;
            }
        }

        var aG = new Graph(aDoc, default);
        var aSS = new StringBuilder();
        for (int aNode = 1; aNode <= aG.Nodes.Count; aNode++)
        {
            NodeType aType = aG.Types[aNode];
            if (aType is not (NodeType.AssemblyRoot or NodeType.Subassembly or NodeType.Part))
            {
                continue;
            }
            aSS.Append(aNode).Append(' ').Append(typeName(aType)).Append(' ').Append(nodeEntry(aDoc, aG.Nodes[aNode - 1], aPrintNames))
               .Append(' ').Append(aG.Usages.TryGetValue(aNode, out int aUsage) ? aUsage : 0).Append('\n');
        }
        di.Append(aSS.ToString());
        return 0;
    }
}
