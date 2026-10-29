// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Iterator.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Iterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Iterates on the direct sub-shapes (children) of a shape. Each child is returned with its orientation composed with
/// the shape's and its location multiplied by the shape's, unless asked otherwise.
/// </summary>
/// <remarks>A class (a working object), where OCCT's is used by value. OCCT's range-for support is left out.</remarks>
public sealed class TopoDS_Iterator
{
    private static readonly NCollection_List<TopoDS_Shape> theEmptyList = new();

    private TopoDS_Shape myShape;
    private readonly NCollection_List<TopoDS_Shape>.Iterator myIterator = new();
    private TopAbs_Orientation myOrientation;
    private TopLoc_Location myLocation;

    /// <summary>An empty iterator.</summary>
    public TopoDS_Iterator()
    {
        myShape = new TopoDS_Shape();
        myOrientation = TopAbs_Orientation.TopAbs_FORWARD;
        myLocation = new TopLoc_Location();
        myIterator.Initialize(theEmptyList);
    }

    /// <summary>An iterator on the sub-shapes of <paramref name="S"/>.</summary>
    /// <param name="S">The shape whose children to visit.</param>
    /// <param name="cumOri">Compose the children's orientations with the orientation of S.</param>
    /// <param name="cumLoc">Multiply the children's locations by the location of S.</param>
    public TopoDS_Iterator(in TopoDS_Shape S, bool cumOri = true, bool cumLoc = true)
    {
        myShape = new TopoDS_Shape();
        myOrientation = TopAbs_Orientation.TopAbs_FORWARD;
        myLocation = new TopLoc_Location();
        Initialize(S, cumOri, cumLoc);
    }

    /// <summary>Starts iterating on the sub-shapes of <paramref name="S"/>.</summary>
    /// <param name="S">The shape whose children to visit.</param>
    /// <param name="cumOri">Compose the children's orientations with the orientation of S.</param>
    /// <param name="cumLoc">Multiply the children's locations by the location of S.</param>
    public void Initialize(in TopoDS_Shape S, bool cumOri = true, bool cumLoc = true)
    {
        if (cumLoc)
        {
            myLocation = S.Location();
        }
        else
        {
            myLocation.Identity();
        }

        if (cumOri)
        {
            myOrientation = S.Orientation();
        }
        else
        {
            myOrientation = TopAbs_Orientation.TopAbs_FORWARD;
        }

        if (S.IsNull())
        {
            myIterator.Initialize(theEmptyList);
        }
        else
        {
            myIterator.Initialize(S.TShape()!.myShapes);
        }

        if (More())
        {
            updateCurrentShape();
        }
    }

    /// <summary>True if there is a current sub-shape.</summary>
    public bool More() => myIterator.More();

    /// <summary>Moves to the next sub-shape.</summary>
    public void Next()
    {
        myIterator.Next();
        if (More())
        {
            updateCurrentShape();
        }
    }

    /// <summary>The current sub-shape.</summary>
    /// <exception cref="Standard_NoSuchObject">There is no current sub-shape.</exception>
    public TopoDS_Shape Value()
    {
        Standard_NoSuchObject_Raise_if(!More(), "TopoDS_Iterator::Value");
        return myShape;
    }

    // Updates myShape from the current iterator position.
    private void updateCurrentShape()
    {
        myShape = myIterator.Value();
        myShape.Orientation(TopAbs.Compose(myOrientation, myShape.Orientation()));
        if (!myLocation.IsIdentity())
        {
            myShape.Move(myLocation, false);
        }
    }
}
