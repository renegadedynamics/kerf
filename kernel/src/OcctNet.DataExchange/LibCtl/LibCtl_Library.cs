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

namespace Occt;

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
public class LibCtl_Library<TModule, TProtocol>
    where TModule : Standard_Transient
    where TProtocol : Interface_Protocol
{
    //  Global registry of (Module, Protocol) pairs. SetGlobal / AddProtocol are
    //  expected to run only during one-time controller initialization, before any
    //  parallel work. Writers serialize against each other; readers traverse
    //  without locking and rely on this register-before-parallel-use contract.
    private static volatile LibCtl_GlobalNode<TModule, TProtocol>? THE_GLOBAL;
    private static readonly Lock LibCtl_TheWriterLock = new();

    private LibCtl_Node<TModule, TProtocol>? thelist;
    private LibCtl_Node<TModule, TProtocol>? thecurr;

    //  Build/extend `theTargetList` for `theProtocol`, recursing into Resources.
    private static void LibCtl_AddProtocolImpl(ref LibCtl_Node<TModule, TProtocol>? theTargetList,
                                               Standard_Transient? theProtocol)
    {
        if (theProtocol is not TProtocol aProtocol)
        {
            return;
        }
        for (LibCtl_GlobalNode<TModule, TProtocol>? aCurrent = THE_GLOBAL; aCurrent is not null;)
        {
            TProtocol? aNodeProto = aCurrent.Protocol();
            if (aNodeProto is not null && aNodeProto.DynamicType() == aProtocol.DynamicType())
            {
                theTargetList ??= new LibCtl_Node<TModule, TProtocol>();
                theTargetList.AddNode(aCurrent);
                break;
            }
            aCurrent = aCurrent.Next();
        }
        int aNbResources = aProtocol.NbResources();
        for (int anInd = 1; anInd <= aNbResources; anInd++)
        {
            LibCtl_AddProtocolImpl(ref theTargetList, aProtocol.Resource(anInd));
        }
    }

    /// <summary>
    /// Adds a couple (module, protocol) into the global definition set for this class of library. Substitutes the
    /// module if the same protocol object is already there.
    /// </summary>
    /// <remarks>Process-wide: call it once per protocol, in the protocol's one-time initialisation.</remarks>
    public static void SetGlobal(TModule theModule, TProtocol theProtocol)
    {
        lock (LibCtl_TheWriterLock)
        {
            var aGlobal = THE_GLOBAL ?? new LibCtl_GlobalNode<TModule, TProtocol>();
            aGlobal.Add(theModule, theProtocol);
            THE_GLOBAL = aGlobal;
        }
    }

    /// <summary>
    /// Creates a library which complies with a protocol, that is, of the same class (criterion IsInstance): it gets
    /// the modules from the global set which are bound to the protocol and its resources. A null protocol gives an
    /// empty library.
    /// </summary>
    public LibCtl_Library(TProtocol? theProtocol)
    {
        if (theProtocol is null)
        {
            return; // NO protocol = EMPTY Lib
        }
        LibCtl_AddProtocolImpl(ref thelist, theProtocol);
    }

    /// <summary>Creates an empty library: it will later be filled by <see cref="AddProtocol"/>.</summary>
    public LibCtl_Library()
    {
    }

    /// <summary>A copy of another library (C++'s copy: the same list of modules and the same iteration point).</summary>
    public LibCtl_Library(LibCtl_Library<TModule, TProtocol> theOther)
    {
        thelist = theOther.thelist;
        thecurr = theOther.thecurr;
    }

    /// <summary>
    /// Adds a couple (module, protocol) to the library, given the class of a protocol. Takes resources into account.
    /// (If <paramref name="theProtocol"/> is not of type <typeparamref name="TProtocol"/>, it is not added.)
    /// </summary>
    public void AddProtocol(Standard_Transient? theProtocol) => LibCtl_AddProtocolImpl(ref thelist, theProtocol);

    /// <summary>Clears the list of modules of the library (it can then be refilled by <see cref="AddProtocol"/>).</summary>
    public void Clear() => thelist = new LibCtl_Node<TModule, TProtocol>();

    /// <summary>Sets the library to the complete global list (all the couples protocol/module recorded in it).</summary>
    public void SetComplete()
    {
        thelist = new LibCtl_Node<TModule, TProtocol>();
        for (LibCtl_GlobalNode<TModule, TProtocol>? aCurrent = THE_GLOBAL; aCurrent is not null;)
        {
            TProtocol? aNodeProto = aCurrent.Protocol();
            if (aNodeProto is not null)
            {
                thelist.AddNode(aCurrent);
            }
            aCurrent = aCurrent.Next();
        }
    }

    /// <summary>
    /// Selects a module from the library, given an object. Returns true if select has succeeded, with the selected
    /// module and the case number determined by the associated protocol; else <paramref name="theModule"/> is null and
    /// <paramref name="theCN"/> is zero.
    /// </summary>
    public bool Select(Standard_Transient? theObj, out TModule? theModule, out int theCN)
    {
        theModule = null;
        theCN = 0; // Response "not found"
        if (thelist is null)
        {
            return false;
        }
        for (LibCtl_Node<TModule, TProtocol>? aCurrent = thelist; aCurrent is not null;)
        {
            TProtocol? protocol = aCurrent.Protocol();
            if (protocol is not null)
            {
                theCN = protocol.CaseNumber(theObj);
                if (theCN > 0)
                {
                    theModule = aCurrent.Module();
                    return true;
                }
            }
            aCurrent = aCurrent.Next(); // this formula is refused in "for"
        }
        return false; // here, not found
    }

    /// <summary>Starts iteration on the modules (sets it on the first one).</summary>
    public void Start() => thecurr = thelist;

    /// <summary>True if there are more modules to iterate on.</summary>
    public bool More() => thecurr is not null;

    /// <summary>Iterates by getting the next module in the list.</summary>
    public void Next()
    {
        if (thecurr is not null)
        {
            thecurr = thecurr.Next();
        }
    }

    /// <summary>The current module in the iteration.</summary>
    /// <exception cref="Standard_NoSuchObject">The iteration is over.</exception>
    public TModule? Module()
    {
        if (thecurr is null)
        {
            throw new Standard_NoSuchObject("Library from LibCtl");
        }
        return thecurr.Module();
    }

    /// <summary>The current protocol in the iteration.</summary>
    /// <exception cref="Standard_NoSuchObject">The iteration is over.</exception>
    public TProtocol? Protocol()
    {
        if (thecurr is null)
        {
            throw new Standard_NoSuchObject("Library from LibCtl");
        }
        return thecurr.Protocol();
    }
}

/// <summary>
/// A node of the global list of a <see cref="LibCtl_Library{TModule, TProtocol}"/>: a module bound with a protocol,
/// and the next node (OCCT's <c>Interface_GlobalNodeOfGeneralLib</c> and the other instantiations).
/// </summary>
public sealed class LibCtl_GlobalNode<TModule, TProtocol> : Standard_Transient
    where TModule : Standard_Transient
    where TProtocol : Interface_Protocol
{
    private TModule? themod;
    private TProtocol? theprot;
    private LibCtl_GlobalNode<TModule, TProtocol>? thenext;

    /// <summary>Creates an empty global node, with no next.</summary>
    public LibCtl_GlobalNode()
    {
    }

    /// <summary>
    /// Adds a module bound with a protocol to the list: does nothing if already in the list, that is, the same object;
    /// replaces the module of the same protocol object.
    /// </summary>
    internal void Add(TModule amodule, TProtocol aprotocol)
    {
        if (themod == amodule)
        {
            return;
        }
        if (theprot == aprotocol)
        {
            themod = amodule;
        }
        else if (thenext is null)
        {
            if (themod is null)
            {
                themod = amodule;
                theprot = aprotocol;
            }
            else
            {
                thenext = new LibCtl_GlobalNode<TModule, TProtocol>();
                thenext.Add(amodule, aprotocol);
            }
        }
        else
        {
            thenext.Add(amodule, aprotocol);
        }
    }

    /// <summary>The module stored in this node.</summary>
    public TModule? Module() => themod;

    /// <summary>The protocol attached to the module.</summary>
    public TProtocol? Protocol() => theprot;

    /// <summary>The next node, null if none.</summary>
    public LibCtl_GlobalNode<TModule, TProtocol>? Next() => thenext;
}

/// <summary>
/// A node of a library's own list: designates a global node (OCCT's <c>Interface_NodeOfGeneralLib</c> and the other
/// instantiations).
/// </summary>
public sealed class LibCtl_Node<TModule, TProtocol> : Standard_Transient
    where TModule : Standard_Transient
    where TProtocol : Interface_Protocol
{
    private LibCtl_GlobalNode<TModule, TProtocol>? thenode;
    private LibCtl_Node<TModule, TProtocol>? thenext;

    /// <summary>Creates an empty node, with no next.</summary>
    public LibCtl_Node()
    {
    }

    /// <summary>Adds a global node: stores it into itself if not yet done, else creates a next node to do it.</summary>
    internal void AddNode(LibCtl_GlobalNode<TModule, TProtocol> anode)
    {
        if (thenode == anode)
        {
            return;
        }
        if (thenext is null)
        {
            if (thenode is null)
            {
                thenode = anode;
            }
            else
            {
                thenext = new LibCtl_Node<TModule, TProtocol>();
                thenext.AddNode(anode);
            }
        }
        else
        {
            thenext.AddNode(anode);
        }
    }

    /// <summary>The module designated by this node.</summary>
    public TModule? Module() => thenode!.Module(); // OCCT: a node left empty by a library's Clear() dereferences null

    /// <summary>The protocol designated by this node.</summary>
    public TProtocol? Protocol() => thenode!.Protocol();

    /// <summary>The next node, null if none.</summary>
    public LibCtl_Node<TModule, TProtocol>? Next() => thenext;
}
