// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_SignShape.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_SignShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Signs HShape according to its real content (type of Shape) Context is not used
/// </summary>
public class MoniTool_SignShape : MoniTool_SignText
{
    public MoniTool_SignShape()
    {
    }

    /// <summary>
    /// Returns "SHAPE"
    /// </summary>
    public override string Name()
    {
        return "SHAPE";
    }

    /// <summary>
    /// Returns for a HShape, the string of its ShapeEnum The Model is absolutely useless (may be null)
    /// </summary>
    public override string Text(Standard_Transient? ent, Standard_Transient? context)
    {
        if (ent is null)
        {
            return "";
        }
        TopoDS_HShape HS = (ent as TopoDS_HShape);
        if (HS is null)
        {
            return ent.DynamicType().Name;
        }
        TopoDS_Shape sh = HS.Shape();
        if (sh.IsNull())
        {
            return "SHAPE";
        }
        return TopAbs.ShapeTypeToString(sh.ShapeType());
    }
}
