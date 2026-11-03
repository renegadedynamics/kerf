// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeSolid.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Construction.Library.LibraryShapeModification;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Api;

/// <summary>Builds a solid from shells (the outer skin and the cavities), from a compsolid, or from a solid and shells to add.</summary>
/// <remarks>
/// A wrapper of <see cref="LibraryMakeSolid"/>. The result is BRepLib's, read without freezing it; <see cref="MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// </remarks>
public class MakeSolid : MakeShape
{
    private readonly LibraryMakeSolid _makeSolid;

    // OCCT's "if (myMakeSolid.IsDone()) { Done(); myShape = myMakeSolid.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (_makeSolid.IsDone)
        {
            Done();
            _shape = _makeSolid.InternalShape();
        }
    }

    public MakeSolid()
    {
        _makeSolid = new LibraryMakeSolid();
    }

    public MakeSolid(in CompSolid s)
    {
        _makeSolid = new LibraryMakeSolid(s);
        TakeResult();
    }

    public MakeSolid(in Shell s)
    {
        _makeSolid = new LibraryMakeSolid(s);
        TakeResult();
    }

    public MakeSolid(in Shell s1, in Shell s2)
    {
        _makeSolid = new LibraryMakeSolid(s1, s2);
        TakeResult();
    }

    public MakeSolid(in Shell s1, in Shell s2, in Shell s3)
    {
        _makeSolid = new LibraryMakeSolid(s1, s2, s3);
        TakeResult();
    }

    public MakeSolid(in Solid so)
    {
        _makeSolid = new LibraryMakeSolid(so);
        TakeResult();
    }

    public MakeSolid(in Solid so, in Shell s)
    {
        _makeSolid = new LibraryMakeSolid(so, s);
        TakeResult();
    }

    public void Add(in Shell s)
    {
        _makeSolid.Add(s);
        TakeResult();
    }


    public override bool IsDone
    {
        get
        {
            return _makeSolid.IsDone;
        }
    }

    /// <summary>The solid (frozen).</summary>
    public Solid Solid()
    {
        return _makeSolid.Solid();
    }

    /// <summary><see cref="Solid"/> without freezing, for ported code.</summary>
    internal Solid InternalSolid()
    {
        return _makeSolid.InternalSolid();
    }

    /// <summary>The solid (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Solid(MakeSolid maker) => maker.Solid();

    /// <summary>True if the face <paramref name="s"/> was deleted (BRepLib's face status).</summary>
    public override bool IsDeleted(in Shape s)
    {
        if (s.Type == ShapeType.Face)
        {
            LibraryShapeModification status = _makeSolid.FaceStatus(ShapeCasts.Face(s));

            if (status == LibraryShapeModification.Deleted)
            {
                return true;
            }
        }

        return false;
    }
}
