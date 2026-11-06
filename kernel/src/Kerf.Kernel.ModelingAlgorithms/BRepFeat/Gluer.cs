// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Gluer.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Gluer.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Gluer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;

namespace Kerf.Engine.Features;

/// <summary>
/// Glues a new shape on a basis shape along faces in contact: a local operation, where the faces and edges of the new
/// shape bound to those of the basis shape are rebuilt into it.
/// </summary>
/// <remarks>
/// LocOpe_Gluer writes into the shapes it is given in its working overlay and ends it in <c>Perform</c> (decision 1013,
/// unit 10H5): the result shares the caller's unchanged sub-shapes, and the history answers for the caller's faces.
/// </remarks>
internal sealed class Gluer : MakeShape
{
    private readonly LocalGluer _gluer;

    /// <summary>Initializes an empty constructor.</summary>
    public Gluer()
    {
        _gluer = new LocalGluer();
    }

    /// <summary>Initializes the shapes to be glued, the new shape Snew and the basis shape Sbase.</summary>
    public Gluer(in Shape snew, in Shape sbase)
    {
        _gluer = new LocalGluer(sbase, snew); // Attention a l'inversion
    }

    /// <summary>Initializes the new shape Snew and the basis shape Sbase for the local glueing operation.</summary>
    public void Init(in Shape snew, in Shape sbase)
    {
        _gluer.Init(sbase, snew);
    }

    /// <summary>
    /// Defines a contact between Fnew on the new shape and Fbase on the basis shape. The contact faces of the glued
    /// shape must not have parts outside the contact faces of the basis shape.
    /// </summary>
    public void Bind(in Face fnew, in Face fbase)
    {
        _gluer.Bind(fnew, fbase);
    }

    /// <summary>The edge Enew in the new shape is the same as the edge Ebase in the basis shape.</summary>
    public void Bind(in Edge enew, in Edge ebase)
    {
        _gluer.Bind(enew, ebase);
    }

    /// <summary>Determine which operation type to use glueing or sliding.</summary>
    public FeatureOperation OpeType() => _gluer.OpeType();

    /// <summary>Returns the basis shape (the caller's).</summary>
    public Shape BasisShape() => _gluer.BasisShape();

    /// <summary>Returns the glued shape (the caller's).</summary>
    public Shape GluedShape() => _gluer.GluedShape();

    /// <summary>Glues the shapes.</summary>
    public override void Build(ProgressRange range = default)
    {
        _gluer.Perform();
        if (_gluer.IsDone)
        {
            Done();
            _shape = _gluer.InternalResultingShape();
        }
        else
        {
            NotDone();
        }
    }

    /// <summary>Returns the status of the face <paramref name="f"/> after the shape creation.</summary>
    public override bool IsDeleted(in Shape f)
    {
        return _gluer.InternalDescendantFaces(ShapeCasts.Face(f)).IsEmpty();
    }

    /// <summary>The faces descending from the face <paramref name="f"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<Shape> InternalModified(in Shape f)
    {
        if (f.Type == ShapeType.Face)
        {
            NCollection_List<Shape> ls = _gluer.InternalDescendantFaces(ShapeCasts.Face(f));
            if (!ls.IsEmpty())
            {
                if (!ls.First().IsSame(f))
                {
                    return _gluer.InternalDescendantFaces(ShapeCasts.Face(f));
                }
            }
        }
        // OCCT returns a function-static empty list
        _generated.Clear();
        return _generated;
    }

    // The history's lists hold sub-shapes of the result: publishing them publishes the result (decision 414).
    private protected override void BeforePublishingLists()
    {
        _shape.TShape()?.FreezeGraph();
    }
}
