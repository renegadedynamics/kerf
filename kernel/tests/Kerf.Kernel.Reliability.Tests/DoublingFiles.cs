// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;

namespace Kerf.Kernel.Reliability.Tests;

/// <summary>
/// Small files whose compounds or assemblies each hold the last one twice: 40 levels are about 2^40 shapes, in a few kB
/// (decision 1916). Depth alone doesn't catch them: 40 is well within the limit on nesting.
/// </summary>
internal static class DoublingFiles
{
    public const int Levels = 40;

    /// <summary>A BREP of a vertex, in compounds.</summary>
    public static MemoryStream Brep()
    {
        var text = new StringBuilder();
        text.Append("DBRep_DrawableShape\n\nCASCADE Topology V1, (c) Matra-Datavision\nLocations 0\nCurve2ds 0\nCurves 0\nPolygon3D 0\n");
        text.Append("PolygonOnTriangulations 0\nSurfaces 0\nTriangulations 0\n\n");
        text.Append("TShapes ").Append(Levels + 1).Append('\n');
        text.Append("Ve\n1e-07\n0 0 0\n0 0\n\n0101101\n*\n");
        for (int level = 1; level <= Levels; level++)
        {
            int below = Levels + 2 - level;
            text.Append("Co\n\n1100000\n+").Append(below).Append(" 0 +").Append(below).Append(" 0 *\n");
        }

        text.Append("\n+1 0\n");
        return new MemoryStream(Encoding.ASCII.GetBytes(text.ToString()));
    }

    /// <summary>A STEP assembly, each level using the one below twice, and the last level an empty part.</summary>
    public static MemoryStream Step()
    {
        var text = new StringBuilder();
        text.Append("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('x'),'2;1');\nFILE_NAME('x','2026-11-07T00:00:00',('a'),('k'),'k','k','u');\n");
        text.Append("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n");
        text.Append("""
            #1 = APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#2);
            #2 = APPLICATION_CONTEXT('core data for automotive mechanical design processes');
            #3 = PRODUCT_CONTEXT('',#2,'mechanical');
            #4 = PRODUCT_DEFINITION_CONTEXT('part definition',#2,'design');
            #5 = AXIS2_PLACEMENT_3D('',#6,#7,#8);
            #6 = CARTESIAN_POINT('',(0.,0.,0.));
            #7 = DIRECTION('',(0.,0.,1.));
            #8 = DIRECTION('',(1.,0.,-0.));
            #9 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#13)) GLOBAL_UNIT_ASSIGNED_CONTEXT((#10,#11,#12)) REPRESENTATION_CONTEXT('c','3D') );
            #10 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );
            #11 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );
            #12 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );
            #13 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#10,'distance_accuracy_value','confusion accuracy');
            #14 = ITEM_DEFINED_TRANSFORMATION('','',#5,#5);

            """.Replace("\r", "", StringComparison.Ordinal));
        int next = 100;
        int childDefinition = 0, childRepresentation = 0;
        for (int level = 0; level <= Levels; level++)
        {
            int product = next++, formation = next++, definition = next++, shape = next++, representation = next++, link = next++;
            text.Append(FormattableString.Invariant($"#{product} = PRODUCT('p{level}','p{level}','',(#3));\n"));
            text.Append(FormattableString.Invariant($"#{formation} = PRODUCT_DEFINITION_FORMATION('','',#{product});\n"));
            text.Append(FormattableString.Invariant($"#{definition} = PRODUCT_DEFINITION('design','',#{formation},#4);\n"));
            text.Append(FormattableString.Invariant($"#{shape} = PRODUCT_DEFINITION_SHAPE('','',#{definition});\n"));
            text.Append(FormattableString.Invariant($"#{representation} = SHAPE_REPRESENTATION('',(#5),#9);\n"));
            text.Append(FormattableString.Invariant($"#{link} = SHAPE_DEFINITION_REPRESENTATION(#{shape},#{representation});\n"));
            if (level > 0)
            {
                for (int use = 0; use < 2; use++)
                {
                    int usage = next++, usageShape = next++, relation = next++, dependent = next++;
                    text.Append(FormattableString.Invariant($"#{usage} = NEXT_ASSEMBLY_USAGE_OCCURRENCE('{level}.{use}','','',#{definition},#{childDefinition},$);\n"));
                    text.Append(FormattableString.Invariant($"#{usageShape} = PRODUCT_DEFINITION_SHAPE('','',#{usage});\n"));
                    text.Append(FormattableString.Invariant($"#{relation} = ( REPRESENTATION_RELATIONSHIP('','',#{childRepresentation},#{representation}) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#14) SHAPE_REPRESENTATION_RELATIONSHIP() );\n"));
                    text.Append(FormattableString.Invariant($"#{dependent} = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{relation},#{usageShape});\n"));
                }
            }

            childDefinition = definition;
            childRepresentation = representation;
        }

        text.Append("ENDSEC;\nEND-ISO-10303-21;\n");
        return new MemoryStream(Encoding.ASCII.GetBytes(text.ToString()));
    }

    /// <summary>A glTF of no meshes, each node with the one below as both its children.</summary>
    public static MemoryStream Gltf()
    {
        var text = new StringBuilder("{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],\"nodes\":[");
        for (int node = 0; node < Levels; node++)
        {
            text.Append(FormattableString.Invariant($"{{\"children\":[{node + 1},{node + 1}]}},"));
        }

        text.Append("{}]}");
        return new MemoryStream(Encoding.ASCII.GetBytes(text.ToString()));
    }
}
