// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_GTransform.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_GTransform.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// General geometric transformation (a <see cref="GeneralTransform3D"/>: an affinity, or any transformation defined by a 3 × 4
/// matrix) of a shape. The shape is first converted to NURBS geometry (<see cref="NurbsConvert"/>),
/// then its curves and surfaces are transformed (<see cref="GTrsfModification"/>): a circle may become an
/// ellipse.
/// </summary>
internal class GTransform : ModifyShape
{
    private readonly GeneralTransform3D _gTrsf;
    private readonly Collect _hist = new();

    /// <summary>A framework applying <paramref name="t"/>; give the shape with <see cref="Perform"/>.</summary>
    public GTransform(in GeneralTransform3D t)
    {
        _gTrsf = t;
        _modification = new GTrsfModification(t);
    }

    /// <summary>Applies <paramref name="t"/> to <paramref name="s"/>.</summary>
    /// <param name="s">The shape.</param>
    /// <param name="t">The transformation.</param>
    /// <param name="copy">Passed to the NURBS conversion, which ignores it.</param>
    public GTransform(in Shape s, in GeneralTransform3D t, bool copy = false)
    {
        _gTrsf = t;
        _modification = new GTrsfModification(t);
        Perform(s, copy);
    }

    /// <summary>Applies the transformation to <paramref name="s"/>.</summary>
    public void Perform(in Shape s, bool copy = false)
    {
        var nc = new NurbsConvert();
        nc.Perform(s, copy);
        _hist.Add(s, nc);
        // Kerf: the converted shape is only an intermediate, read without freezing it
        Shape slocal = nc.InternalShape();
        var theModif = (GTrsfModification)_modification!;
        theModif.GTrsf() = _gTrsf;
        DoModif(slocal, _modification!);
        //  myHist.Filter (Shape());
    }

    internal override NCollection_List<Shape> InternalModified(in Shape f)
    {
        _generated.Clear();
        Dictionary<Shape, NCollection_List<Shape>> m = _hist.Modification();
        if (m.ContainsKey(f))
        {
            // OCCT copies each image's list into a local that it never reads; the list returned is the one the last
            // base Modified call filled (the image of the last converted shape).
            var li = new NCollection_List<Shape>();
            var itL = new NCollection_List<Shape>.Iterator(m.GetValue(f));
            for (; itL.More(); itL.Next())
            {
                li.Assign(base.InternalModified(itL.Value()));
            }
        }
        return _generated;
    }

    internal override Shape InternalModifiedShape(in Shape s)
    {
        Dictionary<Shape, NCollection_List<Shape>> mapModif = _hist.Modification();
        Shape shape = s;

        if (mapModif.ContainsKey(s))
        {
            NCollection_List<Shape> listModShape = mapModif.GetValue(s);
            int nbShapes = listModShape.Extent();

            if (nbShapes > 0)
            {
                shape = listModShape.First();
            }
        }

        return base.InternalModifiedShape(shape);
    }
}
