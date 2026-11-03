// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_PurgeLocations.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_PurgeLocations.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Topology;

using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// Removes from a shape the locations that aren't rigid motions (mirrors and scalings), by applying them to the
/// geometry with <see cref="TrsfModification"/>, and rebuilding the shape with <see cref="ReShape"/>.
/// </summary>
/// <remarks>
/// A working object. <see cref="GetResult"/> and <see cref="ModifiedShape"/> are publication points: they freeze the
/// shapes they return (docs/phase3-design.md, section 4.2). The input is never written into.
/// </remarks>
public sealed class PurgeLocations
{
    private bool _done;
    private Shape _shape = new();
    private readonly IndexedSet<Shape> _mapShapes = new(ShapeHasher.Instance);
    private readonly LocationSet _locations = new();
    private readonly Dictionary<Shape, Shape> _mapNewShapes = new(ShapeHasher.Instance);
    private ReShape? _reShape;

    /// <summary>A purge, not yet performed.</summary>
    public PurgeLocations()
    {
        _done = false;
    }

    /// <summary>Removes the locations that aren't rigid motions from the shape. True if every one could be applied.</summary>
    public bool Perform(in Shape shape)
    {
        _shape = shape;
        _mapShapes.Clear();
        _locations.Clear();
        _done = true;
        AddShape(_shape);

        // Check locations;
        int ind;
        var badTrsfInds = new List<int>();
        for (ind = 1; ; ++ind)
        {
            Placement loc = _locations.Location(ind);

            if (loc.IsIdentity)
            {
                break;
            }

            Transform3D trsf = loc.Transformation;
            bool isBadTrsf =
                trsf.IsNegative
                || (Math.Abs(Math.Abs(trsf.ScaleFactor) - 1.0) > Placement.ScalePrec());
            if (isBadTrsf)
            {
                badTrsfInds.Add(ind);
            }
        }

        if (badTrsfInds.Count == 0)
        {
            return _done;
        }

        int nbShapes = _mapShapes.Count;
        _mapNewShapes.Clear();
        int inds;
        for (inds = 1; inds <= nbShapes; ++inds)
        {
            Shape s = _mapShapes[inds - 1];
            int locInd = _locations.Index(s.Placement);
            if (locInd == 0)
            {
                continue;
            }
            int il;
            for (il = 0; il < badTrsfInds.Count; ++il)
            {
                if (badTrsfInds[il] == locInd)
                {
                    bool isDone = PurgeLocation(s, out Shape trS);
                    _done = _done && isDone;
                    _mapNewShapes[s] = trS;
                    break;
                }
            }
        }

        if (_reShape is null)
        {
            _reShape = new ReShape();
        }
        else
        {
            _reShape.Clear();
        }
        // OCCT.NET: the map's order isn't OCCT's (decision 18); the replacements are of distinct shapes, so the
        // order doesn't change the result.
        foreach (KeyValuePair<Shape, Shape> iter in _mapNewShapes)
        {
            Shape oldS = iter.Key;
            Shape newS = iter.Value;
            _reShape.Replace(oldS, newS);
        }

        _shape = _reShape.InternalApply(_shape);

        return _done;
    }

    private bool PurgeLocation(in Shape s, out Shape res)
    {
        bool isDone = true;
        Placement refLoc = s.Placement;
        bool isEmpty = refLoc.IsIdentity;
        if (isEmpty)
        {
            res = s;
            return isDone;
        }

        Placement nullLoc = new Placement();
        res = s.Located(nullLoc);

        while (!isEmpty)
        {
            Datum3D fd = refLoc.FirstDatum();
            Transform3D trsf = fd.Trsf();
            int fp = refLoc.FirstPower();
            bool isBad = trsf.IsNegative
                         || (Math.Abs(Math.Abs(trsf.ScaleFactor) - 1.0) > Placement.ScalePrec());
            Placement loc = new Placement(fd);
            loc = loc.Powered(fp);
            trsf = loc.Transformation;
            if (isBad)
            {
                TrsfModification modification = new TrsfModification(trsf);
                Modifier modifier = new Modifier(res, modification);
                if (modifier.IsDone)
                {
                    res = modifier.InternalModifiedShape(res);
                }
                else
                {
                    isDone = false;
                    res = res.Moved(loc);
                }
            }
            else
            {
                res = res.Moved(loc);
            }

            refLoc = refLoc.NextLocation();
            isEmpty = refLoc.IsIdentity;
        }

        return isDone;
    }

    private void AddShape(in Shape s)
    {
        _mapShapes.Add(s);
        _locations.Add(s.Placement);

        var it = new Iterator(s, false, false);
        while (it.More())
        {
            AddShape(it.Value());
            it.Next();
        }
    }

    /// <summary>The shape without the locations that aren't rigid motions, frozen.</summary>
    public Shape GetResult()
    {
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    /// <summary>True if every location could be applied.</summary>
    public bool IsDone => _done;

    /// <summary>The shape that a sub-shape of the input became (the sub-shape itself if unchanged), frozen.</summary>
    public Shape ModifiedShape(in Shape initShape)
    {
        Shape shape = initShape;
        if (_mapNewShapes.ContainsKey(initShape))
        {
            shape = _mapNewShapes.GetValue(initShape);
        }
        shape.TShape()?.FreezeGraph();
        return shape;
    }
}
