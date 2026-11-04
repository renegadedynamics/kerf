// Port of: src/DataExchange/TKXSBase/LibCtl/LibCtl_Library.gxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_GlobalNodeOfGeneralLib.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_GlobalNodeOfGeneralLib_0.cxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_NodeOfGeneralLib.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_NodeOfGeneralLib_0.cxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_GlobalNodeOfReaderLib.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_GlobalNodeOfReaderLib_0.cxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_NodeOfReaderLib.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_NodeOfReaderLib_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// A library of modules: the modules that serve the objects of a protocol (and of the protocols it uses as
/// resources), selected for an object by the protocol's case number. OCCT's generic <c>LibCtl_Library</c>
/// (a <c>.gxx</c> file instantiated by macros as <c>Interface_GeneralLib</c>, <c>Interface_ReaderLib</c>,
/// <c>StepData_WriterLib</c>...), here a C# generic: each instantiation derives from it.
/// </summary>
/// <remarks>
/// <para>
/// The global list of (module, protocol) couples is per instantiation (a static of the closed generic type), as OCCT's
/// static per <c>_0.cxx</c> file. It is process-wide state (plan 2.7, decision 1182): a protocol registers its modules
/// once, in its one-time initialisation, before any library is built from it; registering again the same protocol
/// object replaces its module, as OCCT's <c>SetGlobal</c> does. Writers take a lock; readers traverse without one,
/// which OCCT's contract (registration before parallel use) makes safe. A library itself is a working object, one per
/// thread.
/// </para>
/// </remarks>
/// <typeparam name="TModule">The module class (OCCT's <c>TheModule</c>).</typeparam>
/// <typeparam name="TProtocol">The protocol class (OCCT's <c>TheProtocol</c>).</typeparam>
public class Library<TModule, TProtocol>
    where TModule : Transient
    where TProtocol : Protocol
{
    //  Global registry of (Module, Protocol) pairs. SetGlobal / AddProtocol are
    //  expected to run only during one-time controller initialization, before any
    //  parallel work. Writers serialize against each other; readers traverse
    //  without locking and rely on this register-before-parallel-use contract.
    private static volatile GlobalNode<TModule, TProtocol>? s_theGlobal;
    private static readonly Lock LibCtl_TheWriterLock = new();

    private Node<TModule, TProtocol>? _thelist;
    private Node<TModule, TProtocol>? _thecurr;

    //  Build/extend `theTargetList` for `theProtocol`, recursing into Resources.
    private static void LibCtl_AddProtocolImpl(ref Node<TModule, TProtocol>? targetList,
                                               Transient? protocol)
    {
        if (protocol is not TProtocol aProtocol)
        {
            return;
        }
        for (GlobalNode<TModule, TProtocol>? current = s_theGlobal; current is not null;)
        {
            TProtocol? nodeProto = current.Protocol();
            if (nodeProto is not null && nodeProto.DynamicType() == aProtocol.DynamicType())
            {
                targetList ??= new Node<TModule, TProtocol>();
                targetList.AddNode(current);
                break;
            }
            current = current.Next();
        }
        int nbResources = aProtocol.ResourceCount;
        for (int ind = 1; ind <= nbResources; ind++)
        {
            LibCtl_AddProtocolImpl(ref targetList, aProtocol.Resource(ind));
        }
    }

    /// <summary>
    /// Adds a couple (module, protocol) into the global definition set for this class of library. Substitutes the
    /// module if the same protocol object is already there.
    /// </summary>
    /// <remarks>Process-wide: call it once per protocol, in the protocol's one-time initialisation.</remarks>
    public static void SetGlobal(TModule module, TProtocol protocol)
    {
        lock (LibCtl_TheWriterLock)
        {
            var aGlobal = s_theGlobal ?? new GlobalNode<TModule, TProtocol>();
            aGlobal.Add(module, protocol);
            s_theGlobal = aGlobal;
        }
    }

    /// <summary>
    /// Creates a library which complies with a protocol, that is, of the same class (criterion IsInstance): it gets
    /// the modules from the global set which are bound to the protocol and its resources. A null protocol gives an
    /// empty library.
    /// </summary>
    public Library(TProtocol? protocol)
    {
        if (protocol is null)
        {
            return; // NO protocol = EMPTY Lib
        }
        LibCtl_AddProtocolImpl(ref _thelist, protocol);
    }

    /// <summary>Creates an empty library: it will later be filled by <see cref="AddProtocol"/>.</summary>
    public Library()
    {
    }

    /// <summary>A copy of another library (C++'s copy: the same list of modules and the same iteration point).</summary>
    public Library(Library<TModule, TProtocol> other)
    {
        _thelist = other._thelist;
        _thecurr = other._thecurr;
    }

    /// <summary>
    /// Adds a couple (module, protocol) to the library, given the class of a protocol. Takes resources into account.
    /// (If <paramref name="protocol"/> is not of type <typeparamref name="TProtocol"/>, it is not added.)
    /// </summary>
    public void AddProtocol(Transient? protocol) => LibCtl_AddProtocolImpl(ref _thelist, protocol);

    /// <summary>Clears the list of modules of the library (it can then be refilled by <see cref="AddProtocol"/>).</summary>
    public void Clear() => _thelist = new Node<TModule, TProtocol>();

    /// <summary>Sets the library to the complete global list (all the couples protocol/module recorded in it).</summary>
    public void SetComplete()
    {
        _thelist = new Node<TModule, TProtocol>();
        for (GlobalNode<TModule, TProtocol>? current = s_theGlobal; current is not null;)
        {
            TProtocol? nodeProto = current.Protocol();
            if (nodeProto is not null)
            {
                _thelist.AddNode(current);
            }
            current = current.Next();
        }
    }

    /// <summary>
    /// Selects a module from the library, given an object. Returns true if select has succeeded, with the selected
    /// module and the case number determined by the associated protocol; else <paramref name="module"/> is null and
    /// <paramref name="cn"/> is zero.
    /// </summary>
    public bool Select(Transient? obj, out TModule? module, out int cn)
    {
        module = null;
        cn = 0; // Response "not found"
        if (_thelist is null)
        {
            return false;
        }
        for (Node<TModule, TProtocol>? current = _thelist; current is not null;)
        {
            TProtocol? protocol = current.Protocol();
            if (protocol is not null)
            {
                cn = protocol.CaseNumber(obj);
                if (cn > 0)
                {
                    module = current.Module();
                    return true;
                }
            }
            current = current.Next(); // this formula is refused in "for"
        }
        return false; // here, not found
    }

    /// <summary>Starts iteration on the modules (sets it on the first one).</summary>
    public void Start() => _thecurr = _thelist;

    /// <summary>True if there are more modules to iterate on.</summary>
    public bool More() => _thecurr is not null;

    /// <summary>Iterates by getting the next module in the list.</summary>
    public void Next()
    {
        if (_thecurr is not null)
        {
            _thecurr = _thecurr.Next();
        }
    }

    /// <summary>The current module in the iteration.</summary>
    /// <exception cref="NoSuchObjectException">The iteration is over.</exception>
    public TModule? Module()
    {
        if (_thecurr is null)
        {
            throw new NoSuchObjectException("Library from LibCtl");
        }
        return _thecurr.Module();
    }

    /// <summary>The current protocol in the iteration.</summary>
    /// <exception cref="NoSuchObjectException">The iteration is over.</exception>
    public TProtocol? Protocol()
    {
        if (_thecurr is null)
        {
            throw new NoSuchObjectException("Library from LibCtl");
        }
        return _thecurr.Protocol();
    }
}

/// <summary>
/// A node of the global list of a <see cref="Library{TModule, TProtocol}"/>: a module bound with a protocol,
/// and the next node (OCCT's <c>Interface_GlobalNodeOfGeneralLib</c> and the other instantiations).
/// </summary>
internal sealed class GlobalNode<TModule, TProtocol> : Transient
    where TModule : Transient
    where TProtocol : Protocol
{
    private TModule? _themod;
    private TProtocol? _theprot;
    private GlobalNode<TModule, TProtocol>? _thenext;

    /// <summary>Creates an empty global node, with no next.</summary>
    public GlobalNode()
    {
    }

    /// <summary>
    /// Adds a module bound with a protocol to the list: does nothing if already in the list, that is, the same object;
    /// replaces the module of the same protocol object.
    /// </summary>
    internal void Add(TModule amodule, TProtocol aprotocol)
    {
        if (_themod == amodule)
        {
            return;
        }
        if (_theprot == aprotocol)
        {
            _themod = amodule;
        }
        else if (_thenext is null)
        {
            if (_themod is null)
            {
                _themod = amodule;
                _theprot = aprotocol;
            }
            else
            {
                _thenext = new GlobalNode<TModule, TProtocol>();
                _thenext.Add(amodule, aprotocol);
            }
        }
        else
        {
            _thenext.Add(amodule, aprotocol);
        }
    }

    /// <summary>The module stored in this node.</summary>
    public TModule? Module() => _themod;

    /// <summary>The protocol attached to the module.</summary>
    public TProtocol? Protocol() => _theprot;

    /// <summary>The next node, null if none.</summary>
    public GlobalNode<TModule, TProtocol>? Next() => _thenext;
}

/// <summary>
/// A node of a library's own list: designates a global node (OCCT's <c>Interface_NodeOfGeneralLib</c> and the other
/// instantiations).
/// </summary>
internal sealed class Node<TModule, TProtocol> : Transient
    where TModule : Transient
    where TProtocol : Protocol
{
    private GlobalNode<TModule, TProtocol>? _thenode;
    private Node<TModule, TProtocol>? _thenext;

    /// <summary>Creates an empty node, with no next.</summary>
    public Node()
    {
    }

    /// <summary>Adds a global node: stores it into itself if not yet done, else creates a next node to do it.</summary>
    internal void AddNode(GlobalNode<TModule, TProtocol> anode)
    {
        if (_thenode == anode)
        {
            return;
        }
        if (_thenext is null)
        {
            if (_thenode is null)
            {
                _thenode = anode;
            }
            else
            {
                _thenext = new Node<TModule, TProtocol>();
                _thenext.AddNode(anode);
            }
        }
        else
        {
            _thenext.AddNode(anode);
        }
    }

    /// <summary>The module designated by this node.</summary>
    public TModule? Module() => _thenode!.Module(); // OCCT: a node left empty by a library's Clear() dereferences null

    /// <summary>The protocol designated by this node.</summary>
    public TProtocol? Protocol() => _thenode!.Protocol();

    /// <summary>The next node, null if none.</summary>
    public Node<TModule, TProtocol>? Next() => _thenext;
}
