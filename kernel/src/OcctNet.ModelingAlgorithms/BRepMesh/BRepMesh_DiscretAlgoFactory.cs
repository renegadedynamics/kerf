// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretAlgoFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretAlgoFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// An abstract factory of meshing algorithms, with a process-wide registry of the factories by name.
/// </summary>
/// <remarks>
/// <para>
/// The registry is process-wide state (decision 492). Its mutators, <see cref="RegisterFactory"/> and
/// <see cref="UnregisterFactory"/>, are internal. The two factories OCCT registers when its libraries load,
/// <see cref="BRepMesh_IncrementalMeshFactory"/> ("FastDiscret", preferred) and <see cref="XBRepMesh_Factory"/>
/// ("XBRepMesh"), register when this type initialises, in that order. There is no plugin loading (OCCT 8 has none
/// either). The registry is locked, so it can be read from several threads.
/// </para>
/// <para>Names are compared ignoring the case of ASCII letters, as OCCT's <c>IsSameString(..., false)</c>.</para>
/// </remarks>
public abstract class BRepMesh_DiscretAlgoFactory : Transient
{
    private static readonly Lock theLock = new();
    private static readonly NCollection_List<BRepMesh_DiscretAlgoFactory> TheFactories = new();

    /// <summary>The factory's name.</summary>
    protected readonly string myName;

    static BRepMesh_DiscretAlgoFactory()
    {
        // OCCT's self-registering instances (FactoryInitializer), in the order TKMesh and TKXMesh load.
        BRepMesh_IncrementalMeshFactory.Register();
        XBRepMesh_Factory.Register();
    }

    /// <summary>A factory with the name that identifies it.</summary>
    protected BRepMesh_DiscretAlgoFactory(string theName)
    {
        myName = theName;
    }

    /// <summary>
    /// Registers a factory: first (the default) if <paramref name="theIsPreferred"/>, replacing any of the same name;
    /// otherwise last, unless one of the same name is already registered. Internal (decision 492).
    /// </summary>
    internal static void RegisterFactory(BRepMesh_DiscretAlgoFactory theFactory, bool theIsPreferred = false)
    {
        lock (theLock)
        {
            string aName = theFactory.Name();
            NCollection_List<BRepMesh_DiscretAlgoFactory> aFactories = TheFactories;
            if (theIsPreferred)
            {
                UnregisterFactory(aName);
                aFactories.Prepend(theFactory);
                return;
            }

            for (NCollection_List<BRepMesh_DiscretAlgoFactory>.Iterator anIter = new(aFactories); anIter.More(); anIter.Next())
            {
                if (isSameString(anIter.Value().Name(), aName))
                {
                    return;
                }
            }

            aFactories.Append(theFactory);
        }
    }

    /// <summary>Unregisters the factories of the name. Internal (decision 492).</summary>
    internal static void UnregisterFactory(string theName)
    {
        lock (theLock)
        {
            NCollection_List<BRepMesh_DiscretAlgoFactory> aFactories = TheFactories;
            for (NCollection_List<BRepMesh_DiscretAlgoFactory>.Iterator anIter = new(aFactories); anIter.More();)
            {
                if (isSameString(anIter.Value().Name(), theName))
                {
                    aFactories.Remove(anIter);
                }
                else
                {
                    anIter.Next();
                }
            }
        }
    }

    /// <summary>The default (first registered) factory, or null if none is registered.</summary>
    public static BRepMesh_DiscretAlgoFactory? DefaultFactory()
    {
        lock (theLock)
        {
            return !TheFactories.IsEmpty() ? TheFactories.First() : null;
        }
    }

    /// <summary>The factory of the name, or null if none is registered.</summary>
    public static BRepMesh_DiscretAlgoFactory? FindFactory(string theName)
    {
        lock (theLock)
        {
            for (NCollection_List<BRepMesh_DiscretAlgoFactory>.Iterator anIter = new(TheFactories); anIter.More(); anIter.Next())
            {
                if (isSameString(anIter.Value().Name(), theName))
                {
                    return anIter.Value();
                }
            }

            return null;
        }
    }

    /// <summary>The registered factories, in order (a copy: OCCT returns the registry itself).</summary>
    public static NCollection_List<BRepMesh_DiscretAlgoFactory> Factories()
    {
        lock (theLock)
        {
            return new NCollection_List<BRepMesh_DiscretAlgoFactory>(TheFactories);
        }
    }

    /// <summary>A new meshing algorithm for the shape with the linear and angular deflections, not yet performed.</summary>
    public abstract BRepMesh_DiscretRoot CreateAlgorithm(in TopoDS_Shape theShape,
                                                         double theLinDeflection,
                                                         double theAngDeflection);

    /// <summary>The factory's name.</summary>
    public string Name() => myName;

    // TCollection_AsciiString::IsSameString(a, b, false): equal ignoring the case of ASCII letters.
    private static bool isSameString(string theString1, string theString2)
    {
        if (theString1.Length != theString2.Length)
        {
            return false;
        }

        for (int i = 0; i < theString1.Length; ++i)
        {
            char a = theString1[i], b = theString2[i];
            if (a != b && !(char.IsAsciiLetter(a) && char.IsAsciiLetter(b) && char.ToLowerInvariant(a) == char.ToLowerInvariant(b)))
            {
                return false;
            }
        }

        return true;
    }
}
