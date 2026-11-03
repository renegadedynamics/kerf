// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Gluer.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Gluer.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Gluer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Glues a new shape on a basis shape along faces in contact: a local operation, where the faces and edges of the new
/// shape bound to those of the basis shape are rebuilt into it.
/// </summary>
/// <remarks>
/// LocOpe_Gluer copies the frozen shapes it is given and ends its working shapes in <c>Perform</c> (decision 1013):
/// the result shares the caller's unchanged sub-shapes, and the history answers for the caller's faces.
/// </remarks>
public sealed class BRepFeat_Gluer : BRepBuilderAPI_MakeShape
{
    private readonly LocOpe_Gluer myGluer;

    /// <summary>Initializes an empty constructor.</summary>
    public BRepFeat_Gluer()
    {
        myGluer = new LocOpe_Gluer();
    }

    /// <summary>Initializes the shapes to be glued, the new shape Snew and the basis shape Sbase.</summary>
    public BRepFeat_Gluer(in Shape Snew, in Shape Sbase)
    {
        myGluer = new LocOpe_Gluer(Sbase, Snew); // Attention a l'inversion
    }

    /// <summary>Initializes the new shape Snew and the basis shape Sbase for the local glueing operation.</summary>
    public void Init(in Shape Snew, in Shape Sbase)
    {
        myGluer.Init(Sbase, Snew);
    }

    /// <summary>
    /// Defines a contact between Fnew on the new shape and Fbase on the basis shape. The contact faces of the glued
    /// shape must not have parts outside the contact faces of the basis shape.
    /// </summary>
    public void Bind(in Face Fnew, in Face Fbase)
    {
        myGluer.Bind(Fnew, Fbase);
    }

    /// <summary>The edge Enew in the new shape is the same as the edge Ebase in the basis shape.</summary>
    public void Bind(in Edge Enew, in Edge Ebase)
    {
        myGluer.Bind(Enew, Ebase);
    }

    /// <summary>Determine which operation type to use glueing or sliding.</summary>
    public LocOpe_Operation OpeType() => myGluer.OpeType();

    /// <summary>Returns the basis shape (the caller's).</summary>
    public Shape BasisShape() => myGluer.BasisShape();

    /// <summary>Returns the glued shape (the caller's).</summary>
    public Shape GluedShape() => myGluer.GluedShape();

    /// <summary>Glues the shapes.</summary>
    public override void Build(ProgressRange theRange = default)
    {
        myGluer.Perform();
        if (myGluer.IsDone())
        {
            Done();
            myShape = myGluer.InternalResultingShape();
        }
        else
        {
            NotDone();
        }
    }

    /// <summary>Returns the status of the face <paramref name="F"/> after the shape creation.</summary>
    public override bool IsDeleted(in Shape F)
    {
        return myGluer.InternalDescendantFaces(ShapeCasts.Face(F)).IsEmpty();
    }

    /// <summary>The faces descending from the face <paramref name="F"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<Shape> InternalModified(in Shape F)
    {
        if (F.ShapeType() == ShapeType.Face)
        {
            NCollection_List<Shape> LS = myGluer.InternalDescendantFaces(ShapeCasts.Face(F));
            if (!LS.IsEmpty())
            {
                if (!LS.First().IsSame(F))
                {
                    return myGluer.InternalDescendantFaces(ShapeCasts.Face(F));
                }
            }
        }
        // OCCT returns a function-static empty list
        myGenerated.Clear();
        return myGenerated;
    }

    // The history's lists hold sub-shapes of the result: publishing them publishes the result (decision 414).
    private protected override void BeforePublishingLists()
    {
        myShape.TShape()?.FreezeGraph();
    }
}
