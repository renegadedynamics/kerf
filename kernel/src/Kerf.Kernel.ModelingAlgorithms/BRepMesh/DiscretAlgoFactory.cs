// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretAlgoFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretAlgoFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Meshing;

/// <summary>
/// An abstract factory of meshing algorithms, with a registry of the factories by name.
/// </summary>
/// <remarks>
/// <para>
/// The registry is built once (decision 492; unit 10I): the two factories OCCT registers when its libraries load,
/// <see cref="IncrementalMeshFactory"/> ("FastDiscret", preferred) and <see cref="Factory"/> ("XBRepMesh"), register
/// when this type initialises, in that order, and the registry doesn't change afterwards, so it is read without a lock.
/// OCCT's <c>RegisterFactory</c> and <c>UnregisterFactory</c> work on the registry being built. There is no plugin
/// loading (OCCT 8 has none either). The meshing algorithm a caller gets by default is a setting of the operation
/// context (<see cref="DiscretFactory"/>).
/// </para>
/// <para>Names are compared ignoring the case of ASCII letters, as OCCT's <c>IsSameString(..., false)</c>.</para>
/// </remarks>
internal abstract class DiscretAlgoFactory : Transient
{
    private static readonly DiscretAlgoFactory[] TheFactories = RegisterAll();

    /// <summary>The factory's name.</summary>
    protected readonly string _name;

    // OCCT's self-registering instances (FactoryInitializer), in the order TKMesh and TKXMesh load.
    private static DiscretAlgoFactory[] RegisterAll()
    {
        var factories = new NCollection_List<DiscretAlgoFactory>();
        RegisterFactory(factories, new IncrementalMeshFactory(), true);
        RegisterFactory(factories, new Factory());
        return [.. factories];
    }

    /// <summary>A factory with the name that identifies it.</summary>
    protected DiscretAlgoFactory(string name)
    {
        _name = name;
    }

    /// <summary>
    /// Registers a factory in the registry being built: first (the default) if <paramref name="isPreferred"/>,
    /// replacing any of the same name; otherwise last, unless one of the same name is already registered.
    /// </summary>
    private static void RegisterFactory(NCollection_List<DiscretAlgoFactory> factories, DiscretAlgoFactory factory,
                                        bool isPreferred = false)
    {
        string name = factory.Name();
        if (isPreferred)
        {
            UnregisterFactory(factories, name);
            factories.Prepend(factory);
            return;
        }

        for (NCollection_List<DiscretAlgoFactory>.Iterator iter = new(factories); iter.More(); iter.Next())
        {
            if (IsSameString(iter.Value().Name(), name))
            {
                return;
            }
        }

        factories.Append(factory);
    }

    /// <summary>Unregisters the factories of the name from the registry being built.</summary>
    private static void UnregisterFactory(NCollection_List<DiscretAlgoFactory> factories, string name)
    {
        for (NCollection_List<DiscretAlgoFactory>.Iterator iter = new(factories); iter.More();)
        {
            if (IsSameString(iter.Value().Name(), name))
            {
                factories.Remove(iter);
            }
            else
            {
                iter.Next();
            }
        }
    }

    /// <summary>The default (first registered) factory, or null if none is registered.</summary>
    public static DiscretAlgoFactory? DefaultFactory() => TheFactories.Length > 0 ? TheFactories[0] : null;

    /// <summary>The factory of the name, or null if none is registered.</summary>
    public static DiscretAlgoFactory? FindFactory(string name)
    {
        foreach (DiscretAlgoFactory factory in TheFactories)
        {
            if (IsSameString(factory.Name(), name))
            {
                return factory;
            }
        }
        return null;
    }

    /// <summary>The registered factories, in order (a copy: OCCT returns the registry itself).</summary>
    public static NCollection_List<DiscretAlgoFactory> Factories()
    {
        var factories = new NCollection_List<DiscretAlgoFactory>();
        foreach (DiscretAlgoFactory factory in TheFactories)
        {
            factories.Append(factory);
        }
        return factories;
    }

    /// <summary>A new meshing algorithm for the shape with the linear and angular deflections, not yet performed.</summary>
    public abstract DiscretRoot CreateAlgorithm(in Shape shape,
                                                         double linDeflection,
                                                         double angDeflection);

    /// <summary>The factory's name.</summary>
    public string Name() => _name;

    // TCollection_AsciiString::IsSameString(a, b, false): equal ignoring the case of ASCII letters.
    private static bool IsSameString(string string1, string string2)
    {
        if (string1.Length != string2.Length)
        {
            return false;
        }

        for (int i = 0; i < string1.Length; ++i)
        {
            char a = string1[i], b = string2[i];
            if (a != b && !(char.IsAsciiLetter(a) && char.IsAsciiLetter(b) && char.ToLowerInvariant(a) == char.ToLowerInvariant(b)))
            {
                return false;
            }
        }

        return true;
    }
}
