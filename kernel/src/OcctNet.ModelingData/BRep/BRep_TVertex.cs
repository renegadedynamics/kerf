// Port of: src/ModelingData/TKBRep/BRep/BRep_TVertex.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_TVertex.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_TVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Schematiq.Collections;

namespace Occt;

/// <summary>The geometry of a vertex: a 3D point, a tolerance and a list of point representations.</summary>
/// <remarks>
/// The setters and the list of representations are internal to OCCT.NET and throw
/// <see cref="ImmutableObjectException"/> once the vertex is frozen. <see cref="PointRepresentations"/> is the public,
/// read-only view of the list.
/// </remarks>
public class BRep_TVertex : TopoDS_TVertex
{
    private Point3D myPnt;
    private double myTolerance;
    private readonly NCollection_List<BRep_PointRepresentation> myPoints = new();

    /// <summary>An empty vertex at the origin, with a tolerance of RealEpsilon().</summary>
    public BRep_TVertex()
    {
        myPnt = new Point3D();
        myTolerance = RealEpsilon();
    }

    /// <summary>The tolerance.</summary>
    public double Tolerance() => myTolerance;

    internal void Tolerance(double T)
    {
        CheckMutable();
        myTolerance = T;
    }

    /// <summary>Sets the tolerance to the max of <paramref name="T"/> and the current tolerance.</summary>
    internal void UpdateTolerance(double T)
    {
        CheckMutable();
        if (T > myTolerance)
        {
            myTolerance = T;
        }
    }

    /// <summary>The point, in the vertex's own coordinate system.</summary>
    public Point3D Pnt() => myPnt;

    internal void Pnt(in Point3D P)
    {
        CheckMutable();
        myPnt = P;
    }

    /// <summary>The point representations (internal: ported code reads the list itself).</summary>
    internal NCollection_List<BRep_PointRepresentation> Points() => myPoints;

    internal NCollection_List<BRep_PointRepresentation> ChangePoints()
    {
        CheckMutable();
        return myPoints;
    }

    /// <summary>The point representations, as a new read-only list (OCCT.NET addition).</summary>
    public ISchematiqList<BRep_PointRepresentation> PointRepresentations() => PackedList.Own([.. myPoints]);

    /// <summary>A new vertex with the same point and tolerance, without representations.</summary>
    public override TopoDS_TShape EmptyCopy()
    {
        BRep_TVertex TV = new BRep_TVertex();
        TV.Pnt(myPnt);
        TV.Tolerance(myTolerance);
        return TV;
    }

    internal override void FreezeRepresentations()
    {
        foreach (BRep_PointRepresentation aPR in myPoints)
        {
            aPR.Freeze();
        }
    }
}
