// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Transform.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Transform.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Geometric transformation (a <see cref="Transform3D"/>) of a shape: of the curves of its edges and the surfaces of its
/// faces.
/// </summary>
/// <remarks>
/// A direct isometry without <c>theCopyGeom</c> gives the shape at a new location (sharing its TShapes); otherwise the
/// transformation is applied to a duplicate (<see cref="TrsfModification"/>). The framework can be reused on
/// other shapes with <see cref="Perform"/>.
/// </remarks>
internal class Transform : ModifyShape
{
    private readonly Transform3D _trsf;
    private Placement _location = new();
    private bool _useModif; // OCCT leaves it uninitialised until Perform

    /// <summary>A framework applying <paramref name="t"/>; give the shape with <see cref="Perform"/>.</summary>
    public Transform(in Transform3D t)
    {
        _trsf = t;
        _modification = new TrsfModification(t);
    }

    /// <summary>Applies <paramref name="trsf"/> to <paramref name="shape"/>.</summary>
    /// <param name="shape">The shape.</param>
    /// <param name="trsf">The transformation.</param>
    /// <param name="copyGeom">Copy the geometry even for a direct isometry.</param>
    /// <param name="copyMesh">Copy the triangulations and assign the copies to the result.</param>
    public Transform(in Shape shape, in Transform3D trsf, bool copyGeom = false,
                                    bool copyMesh = false)
    {
        _trsf = trsf;
        _modification = new TrsfModification(trsf);
        Perform(shape, copyGeom, copyMesh);
    }

    /// <summary>
    /// Applies the transformation to <paramref name="shape"/>: a new location for a direct isometry without
    /// <paramref name="copyGeom"/>, else the transformation of a duplicate.
    /// </summary>
    public void Perform(in Shape shape, bool copyGeom = false, bool copyMesh = false)
    {
        _useModif = copyGeom || _trsf.IsNegative
                     || (Math.Abs(Math.Abs(_trsf.ScaleFactor) - 1.0) > Placement.ScalePrec());
        if (_useModif)
        {
            var theModif = (TrsfModification)_modification!;
            theModif.Trsf() = _trsf;
            theModif.IsCopyMesh() = copyMesh;
            DoModif(shape, _modification!);
        }
        else
        {
            _location = new Placement(_trsf);
            _shape = shape.Moved(_location);
            Done();
        }
    }

    internal override Shape InternalModifiedShape(in Shape s)
    {
        if (_useModif)
        {
            return _modifier.InternalModifiedShape(s);
        }
        return s.Moved(_location);
    }

    internal override NCollection_List<Shape> InternalModified(in Shape f)
    {
        if (!_useModif)
        {
            _generated.Clear();
            _generated.Append(f.Moved(_location));
            return _generated;
        }
        return base.InternalModified(f);
    }
}
