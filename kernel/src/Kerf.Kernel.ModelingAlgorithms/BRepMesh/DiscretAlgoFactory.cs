// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretAlgoFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretAlgoFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>
/// An abstract factory of meshing algorithms, with a process-wide registry of the factories by name.
/// </summary>
/// <remarks>
/// <para>
/// The registry is process-wide state (decision 492). Its mutators, <see cref="RegisterFactory"/> and
/// <see cref="UnregisterFactory"/>, are internal. The two factories OCCT registers when its libraries load,
/// <see cref="IncrementalMeshFactory"/> ("FastDiscret", preferred) and <see cref="Factory"/>
/// ("XBRepMesh"), register when this type initialises, in that order. There is no plugin loading (OCCT 8 has none
/// either). The registry is locked, so it can be read from several threads.
/// </para>
/// <para>Names are compared ignoring the case of ASCII letters, as OCCT's <c>IsSameString(..., false)</c>.</para>
/// </remarks>
public abstract class DiscretAlgoFactory : Transient
{
    private static readonly Lock Lock = new();
    private static readonly NCollection_List<DiscretAlgoFactory> TheFactories = new();

    /// <summary>The factory's name.</summary>
    protected readonly string _name;

    static DiscretAlgoFactory()
    {
        // OCCT's self-registering instances (FactoryInitializer), in the order TKMesh and TKXMesh load.
        IncrementalMeshFactory.Register();
        Factory.Register();
    }

    /// <summary>A factory with the name that identifies it.</summary>
    protected DiscretAlgoFactory(string name)
    {
        _name = name;
    }

    /// <summary>
    /// Registers a factory: first (the default) if <paramref name="isPreferred"/>, replacing any of the same name;
    /// otherwise last, unless one of the same name is already registered. Internal (decision 492).
    /// </summary>
    internal static void RegisterFactory(DiscretAlgoFactory factory, bool isPreferred = false)
    {
        lock (global::Kerf.Engine.Meshing.DiscretAlgoFactory.Lock)
        {
            string name = factory.Name();
            NCollection_List<DiscretAlgoFactory> factories = TheFactories;
            if (isPreferred)
            {
                UnregisterFactory(name);
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
    }

    /// <summary>Unregisters the factories of the name. Internal (decision 492).</summary>
    internal static void UnregisterFactory(string name)
    {
        lock (global::Kerf.Engine.Meshing.DiscretAlgoFactory.Lock)
        {
            NCollection_List<DiscretAlgoFactory> factories = TheFactories;
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
    }

    /// <summary>The default (first registered) factory, or null if none is registered.</summary>
    public static DiscretAlgoFactory? DefaultFactory()
    {
        lock (global::Kerf.Engine.Meshing.DiscretAlgoFactory.Lock)
        {
            return !TheFactories.IsEmpty() ? TheFactories.First() : null;
        }
    }

    /// <summary>The factory of the name, or null if none is registered.</summary>
    public static DiscretAlgoFactory? FindFactory(string name)
    {
        lock (global::Kerf.Engine.Meshing.DiscretAlgoFactory.Lock)
        {
            for (NCollection_List<DiscretAlgoFactory>.Iterator iter = new(TheFactories); iter.More(); iter.Next())
            {
                if (IsSameString(iter.Value().Name(), name))
                {
                    return iter.Value();
                }
            }

            return null;
        }
    }

    /// <summary>The registered factories, in order (a copy: OCCT returns the registry itself).</summary>
    public static NCollection_List<DiscretAlgoFactory> Factories()
    {
        lock (global::Kerf.Engine.Meshing.DiscretAlgoFactory.Lock)
        {
            return new NCollection_List<DiscretAlgoFactory>(TheFactories);
        }
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
