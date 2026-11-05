// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Reflection;
using Kerf.Engine;

namespace Kerf.Kernel.Benchmarks;

/// <summary>A shape to mesh: its BRep text without meshes (F1) and the viewer's deflection for it.</summary>
public sealed record Case(string Name, string F1, double Deflection, int Faces);

/// <summary>
/// The benchmark's shapes (phase4-plan 2.5): data/occ files of several sizes, and shapes built by native OCCT (a filleted
/// box cut by cylinders, a fused part, a sphere, a torus), all written as BRep text without meshes so both sides read
/// the same input. The deflection is the viewer's: 0.1% of the bounding box's diagonal (build123d.NET's
/// MeshTolerances.For), with an angle of 0.2 rad.
/// </summary>
public static class Cases
{
    public const double Angle = 0.2;

    public static readonly string[] DataOccFiles =
    [
        "Pump_Nut.brep", "bottle.brep", "CrankArm.brep", "hammer.brep", "MODERN_Chair_1.brep", "Motor-c.brep", "Axis_of_bearing.brep",
    ];

    private static string Metadata(string theKey)
        => typeof(Cases).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    public static string DataOcc => Path.Combine(Metadata("OcctSourceDir"), "data", "occ");

    private static readonly Lazy<string> TheDirectory = new(() => Directory.CreateTempSubdirectory("occtnet-bench-").FullName);

    private static int theFileNumber;

    private static string NewFile(string theName)
        => Path.Combine(TheDirectory.Value, $"{Interlocked.Increment(ref theFileNumber)}-{theName.Replace(' ', '_')}.brep");

    public static Shape PortRead(string theFile)
    {
        if (!BrepTools.Read(out Shape aShape, theFile))
        {
            throw new InvalidOperationException("cannot read " + theFile);
        }
        return aShape;
    }

    /// <summary>The viewer's deflection: 0.1% of the bounding box's diagonal.</summary>
    public static double ViewerDeflection(in Shape theShape)
    {
        var aBox = new Box();
        Bounds.Add(theShape, ref aBox, true);
        double aDiagonal = aBox.IsVoid ? 0.0 : Math.Sqrt(aBox.SquareExtent());
        return aDiagonal > 1e-6 ? aDiagonal * 0.1 / 100 : 1e-3;
    }

    public static List<Case> All()
    {
        var aCases = new List<Case>();
        foreach (string aName in DataOccFiles)
        {
            string aPath = Path.Combine(DataOcc, aName);
            if (File.Exists(aPath))
            {
                aCases.Add(Make(aName, F1OfFile(aPath)));
            }
        }
        return aCases;
    }

    private static Case Make(string theName, string theF1)
    {
        Shape aShape = PortRead(theF1);
        return new Case(theName, theF1, ViewerDeflection(aShape), global::Kerf.Engine.Meshing.MeshExtraction.Faces(aShape).Count);
    }

    /// <summary>The file without its meshes, written again (by native OCCT where it is available, as the oracle's F1).</summary>
    private static string F1OfFile(string thePath)
    {
        string aFile = NewFile(Path.GetFileNameWithoutExtension(thePath));
        BrepTools.Write(BrepTools.Cleaned(PortRead(thePath)), aFile, false, false, FormatVersion.FormatVersion_CURRENT);
        return aFile;
    }


    /// <summary>What Bento's viewer does with the port: mesh (a copy of the frozen input) and extract.</summary>
    internal static global::Kerf.Engine.Meshing.MeshBuffers PortMeshAndExtract(in Shape theShape, double theDeflection, bool theParallel)
    {
        Shape aMeshed = new IncrementalMesh(theShape, theDeflection, false, Angle, theParallel).Shape();
        return global::Kerf.Engine.Meshing.MeshExtraction.Extract(aMeshed, theDeflection, Angle);
    }
}
