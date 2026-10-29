// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Context.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The intersection context: caches of the tools the Boolean component builds per shape (point classifiers,
/// projectors, solid classifiers, hatchers, boxes), so that each is built once per algorithm run.
/// </summary>
/// <remarks>
/// <para>
/// A working object, not a value (plan section 6). Each thread of a parallel loop has its own context
/// (<see cref="BOPTools_Parallel"/>, decision 5005), so a context is used by one thread at a time and needs no lock.
/// </para>
/// <para>
/// Unit 6A ports the members BOPDS needs (BndBox, OBB); unit 6B ports the rest of the
/// class (the classifiers, projectors, hatchers and the computations on them) into this file, and audits each cached
/// tool for state carried from one call to the next (plan section 2.4). The maps are keyed by shape with
/// <see cref="TopTools_ShapeMapHasher"/>; the allocator goes (translation rules, section 9).
/// </para>
/// </remarks>
public class IntTools_Context : Standard_Transient
{
    private readonly NCollection_DataMap<TopoDS_Shape, Bnd_Box> myBndBoxDataMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, Bnd_OBB> myOBBMap = new(TopTools_ShapeMapHasher.Instance);
    private int myCreateFlag;
    private double myPOnSTolerance;

    /// <summary>An empty context.</summary>
    public IntTools_Context()
    {
        myCreateFlag = 0;
        myPOnSTolerance = 1.0e-12;
    }

    /// <summary>
    /// The bounding box of the shape (<see cref="BRepBndLib.Add"/>), computed on the first call. OCCT returns a
    /// reference into its cache; the port returns the box by value.
    /// </summary>
    public Bnd_Box BndBox(in TopoDS_Shape aS)
    {
        if (!myBndBoxDataMap.Find(aS, out Bnd_Box aBox))
        {
            aBox = new Bnd_Box();
            BRepBndLib.Add(aS, ref aBox);
            myBndBoxDataMap.Bind(aS, aBox);
        }
        return aBox;
    }

    /// <summary>
    /// The oriented bounding box of the shape (<see cref="BRepBndLib.AddOBB"/>) enlarged by the gap (the fuzzy value,
    /// <see cref="Precision.Confusion"/> by default), computed on the first call. As in OCCT, a box cached with one gap
    /// is returned for any later gap.
    /// </summary>
    public Bnd_OBB OBB(in TopoDS_Shape aS) => OBB(aS, Precision.Confusion());

    /// <inheritdoc cref="OBB(in TopoDS_Shape)"/>
    public Bnd_OBB OBB(in TopoDS_Shape aS, double theGap)
    {
        if (!myOBBMap.Find(aS, out Bnd_OBB aBox))
        {
            aBox = new Bnd_OBB();
            BRepBndLib.AddOBB(aS, ref aBox);
            aBox.Enlarge(theGap);
            myOBBMap.Bind(aS, aBox);
        }
        return aBox;
    }

    /// <summary>The tolerance used to check whether a point lies on a surface (1e-12 by default).</summary>
    public void SetPOnSProjectionTolerance(double theValue) => myPOnSTolerance = theValue;

    // myCreateFlag tells OCCT whether the context made its allocator itself; kept for the members 6B ports.
    internal int CreateFlag() => myCreateFlag;

    internal double POnSProjectionTolerance() => myPOnSTolerance;
}
