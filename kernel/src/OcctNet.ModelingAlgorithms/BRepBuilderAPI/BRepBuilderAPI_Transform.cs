// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Transform.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Transform.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;

using Kerf.Topology;

namespace Occt;

/// <summary>
/// Geometric transformation (a <see cref="Transform3D"/>) of a shape: of the curves of its edges and the surfaces of its
/// faces.
/// </summary>
/// <remarks>
/// A direct isometry without <c>theCopyGeom</c> gives the shape at a new location (sharing its TShapes); otherwise the
/// transformation is applied to a duplicate (<see cref="TrsfModification"/>). The framework can be reused on
/// other shapes with <see cref="Perform"/>.
/// </remarks>
public class BRepBuilderAPI_Transform : BRepBuilderAPI_ModifyShape
{
    private readonly Transform3D myTrsf;
    private Placement myLocation = new();
    private bool myUseModif; // OCCT leaves it uninitialised until Perform

    /// <summary>A framework applying <paramref name="T"/>; give the shape with <see cref="Perform"/>.</summary>
    public BRepBuilderAPI_Transform(in Transform3D T)
    {
        myTrsf = T;
        myModification = new TrsfModification(T);
    }

    /// <summary>Applies <paramref name="theTrsf"/> to <paramref name="theShape"/>.</summary>
    /// <param name="theShape">The shape.</param>
    /// <param name="theTrsf">The transformation.</param>
    /// <param name="theCopyGeom">Copy the geometry even for a direct isometry.</param>
    /// <param name="theCopyMesh">Copy the triangulations and assign the copies to the result.</param>
    public BRepBuilderAPI_Transform(in Shape theShape, in Transform3D theTrsf, bool theCopyGeom = false,
                                    bool theCopyMesh = false)
    {
        myTrsf = theTrsf;
        myModification = new TrsfModification(theTrsf);
        Perform(theShape, theCopyGeom, theCopyMesh);
    }

    /// <summary>
    /// Applies the transformation to <paramref name="theShape"/>: a new location for a direct isometry without
    /// <paramref name="theCopyGeom"/>, else the transformation of a duplicate.
    /// </summary>
    public void Perform(in Shape theShape, bool theCopyGeom = false, bool theCopyMesh = false)
    {
        myUseModif = theCopyGeom || myTrsf.IsNegative
                     || (Math.Abs(Math.Abs(myTrsf.ScaleFactor()) - 1.0) > Placement.ScalePrec());
        if (myUseModif)
        {
            var theModif = (TrsfModification)myModification!;
            theModif.Trsf() = myTrsf;
            theModif.IsCopyMesh() = theCopyMesh;
            DoModif(theShape, myModification!);
        }
        else
        {
            myLocation = new Placement(myTrsf);
            myShape = theShape.Moved(myLocation);
            Done();
        }
    }

    internal override Shape InternalModifiedShape(in Shape S)
    {
        if (myUseModif)
        {
            return myModifier.InternalModifiedShape(S);
        }
        return S.Moved(myLocation);
    }

    internal override NCollection_List<Shape> InternalModified(in Shape F)
    {
        if (!myUseModif)
        {
            myGenerated.Clear();
            myGenerated.Append(F.Moved(myLocation));
            return myGenerated;
        }
        return base.InternalModified(F);
    }
}
