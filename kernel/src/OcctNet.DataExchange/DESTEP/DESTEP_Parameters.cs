// Port of: src/DataExchange/TKDESTEP/DESTEP/DESTEP_Parameters.hxx
// Port of: src/DataExchange/TKDESTEP/DESTEP/DESTEP_Parameters.cxx
// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_StepModelType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2023-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Resource_FormatType;
using static Occt.UnitsMethods_LengthUnit;

namespace Occt;

/// <summary>
/// Gives you the choice of translation mode for an Open CASCADE shape that is being translated to STEP: AsIs (its
/// highest possible STEP representation), ManifoldSolidBrep (a manifold_solid_brep or brep_with_voids entity),
/// FacetedBrep, ShellBasedSurfaceModel, GeometricCurveSet...
/// </summary>
public enum STEPControl_StepModelType
{
    STEPControl_AsIs,
    STEPControl_ManifoldSolidBrep,
    STEPControl_BrepWithVoids,
    STEPControl_FacetedBrep,
    STEPControl_FacetedBrepAndBrepWithVoids,
    STEPControl_ShellBasedSurfaceModel,
    STEPControl_GeometricCurveSet,
    STEPControl_Hybrid,
}

/// <summary>The parameters of STEP reading and writing, as a value (a copy of what the statics hold).</summary>
/// <remarks>
/// A C++ value class with public members, a struct here, so that assigning it copies it as C++ does (unit 8B ports it
/// for <see cref="StepData_StepModel.InternalParameters"/>; its use belongs to STEPControl, unit 8F). Its default is
/// OCCT's: write <c>new DESTEP_Parameters()</c>, never <c>default</c>.
/// </remarks>
public struct DESTEP_Parameters
{
    public enum ReadMode_BSplineContinuity
    {
        ReadMode_BSplineContinuity_C0 = 0,
        ReadMode_BSplineContinuity_C1,
        ReadMode_BSplineContinuity_C2,
    }

    public enum ReadMode_Precision
    {
        ReadMode_Precision_File = 0,
        ReadMode_Precision_User,
    }

    public enum ReadMode_MaxPrecision
    {
        ReadMode_MaxPrecision_Preferred = 0,
        ReadMode_MaxPrecision_Forced,
    }

    public enum ReadMode_SurfaceCurve
    {
        ReadMode_SurfaceCurve_Default = 0,
        ReadMode_SurfaceCurve_2DUse_Preferred = 2,
        ReadMode_SurfaceCurve_2DUse_Forced = -2,
        ReadMode_SurfaceCurve_3DUse_Preferred = 3,
        ReadMode_SurfaceCurve_3DUse_Forced = -3,
    }

    public enum AngleUnitMode
    {
        AngleUnitMode_File = 0,
        AngleUnitMode_Rad,
        AngleUnitMode_Deg,
    }

    public enum ReadMode_ProductContext
    {
        ReadMode_ProductContext_All = 1,
        ReadMode_ProductContext_Design,
        ReadMode_ProductContext_Analysis,
    }

    public enum ReadMode_ShapeRepr
    {
        ReadMode_ShapeRepr_All = 1,
        ReadMode_ShapeRepr_ABSR,
        ReadMode_ShapeRepr_MSSR,
        ReadMode_ShapeRepr_GBSSR,
        ReadMode_ShapeRepr_FBSR,
        ReadMode_ShapeRepr_EBWSR,
        ReadMode_ShapeRepr_GBWSR,
    }

    public enum ReadMode_AssemblyLevel
    {
        ReadMode_AssemblyLevel_All = 1,
        ReadMode_AssemblyLevel_Assembly,
        ReadMode_AssemblyLevel_Structure,
        ReadMode_AssemblyLevel_Shape,
    }

    public enum RWMode_Tessellated
    {
        RWMode_Tessellated_Off = 0,
        RWMode_Tessellated_On,
        RWMode_Tessellated_OnNoBRep,
    }

    public enum WriteMode_PrecisionMode
    {
        WriteMode_PrecisionMode_Least = -1,
        WriteMode_PrecisionMode_Average = 0,
        WriteMode_PrecisionMode_Greatest = 1,
        WriteMode_PrecisionMode_Session = 2,
    }

    public enum WriteMode_Assembly
    {
        WriteMode_Assembly_Off = 0,
        WriteMode_Assembly_On,
        WriteMode_Assembly_Auto,
    }

    public enum WriteMode_StepSchema
    {
        WriteMode_StepSchema_AP214CD = 1,
        WriteMode_StepSchema_AP214DIS,
        WriteMode_StepSchema_AP203,
        WriteMode_StepSchema_AP214IS,
        WriteMode_StepSchema_AP242DIS,
    }

    public enum WriteMode_VertexMode
    {
        WriteMode_VertexMode_OneCompound = 0,
        WriteMode_VertexMode_SingleVertex,
    }

    public DESTEP_Parameters()
    {
    }

    /// <summary>Initialize parameters from a parameter set (OCCT: from the process-wide statics, decision 1181).</summary>
    public void InitFromStatic(Interface_StaticSet theStatics)
    {
        ReadBSplineContinuity = (ReadMode_BSplineContinuity)theStatics.IVal("read.iges.bspline.continuity");
        ReadPrecisionMode = (ReadMode_Precision)theStatics.IVal("read.precision.mode");
        ReadPrecisionVal = theStatics.RVal("read.precision.val");
        ReadMaxPrecisionMode = (ReadMode_MaxPrecision)theStatics.IVal("read.maxprecision.mode");
        ReadMaxPrecisionVal = theStatics.RVal("read.maxprecision.val");
        ReadSameParamMode = theStatics.IVal("read.stdsameparameter.mode") == 1;
        ReadSurfaceCurveMode = (ReadMode_SurfaceCurve)theStatics.IVal("read.surfacecurve.mode");
        EncodeRegAngle = theStatics.RVal("read.encoderegularity.angle") * 180.0 / Math.PI;
        AngleUnit = (AngleUnitMode)theStatics.IVal("step.angleunit.mode");

        ReadProductMode = theStatics.IVal("read.step.product.mode") == 1;
        ReadProductContext = (ReadMode_ProductContext)theStatics.IVal("read.step.product.context");
        ReadShapeRepr = (ReadMode_ShapeRepr)theStatics.IVal("read.step.shape.repr");
        ReadTessellated = (RWMode_Tessellated)theStatics.IVal("read.step.tessellated");
        ReadAssemblyLevel = (ReadMode_AssemblyLevel)theStatics.IVal("read.step.assembly.level");
        ReadRelationship = theStatics.IVal("read.step.shape.relationship") == 1;
        ReadShapeAspect = theStatics.IVal("read.step.shape.aspect") == 1;
        ReadConstrRelation = theStatics.IVal("read.step.constructivegeom.relationship") == 1;
        ReadSubshapeNames = theStatics.IVal("read.stepcaf.subshapes.name") == 1;
        ReadCodePage = (Resource_FormatType)theStatics.IVal("read.step.codepage");
        ReadNonmanifold = theStatics.IVal("read.step.nonmanifold") == 1;
        ReadIdeas = theStatics.IVal("read.step.ideas") == 1;
        ReadAllShapes = theStatics.IVal("read.step.all.shapes") == 1;
        ReadRootTransformation = theStatics.IVal("read.step.root.transformation") == 1;

        WritePrecisionMode = (WriteMode_PrecisionMode)theStatics.IVal("write.precision.mode");
        WritePrecisionVal = theStatics.RVal("write.precision.val");
        WriteAssembly = (WriteMode_Assembly)theStatics.IVal("write.step.assembly");
        WriteSchema = (WriteMode_StepSchema)theStatics.IVal("write.step.schema");
        WriteTessellated = (RWMode_Tessellated)theStatics.IVal("write.step.tessellated");
        WriteProductName = theStatics.CVal("write.step.product.name");
        WriteSurfaceCurMode = theStatics.IVal("write.surfacecurve.mode") == 1;
        WriteUnit = (UnitsMethods_LengthUnit)theStatics.IVal("write.step.unit");
        WriteVertexMode = (WriteMode_VertexMode)theStatics.IVal("write.step.vertex.mode");
        WriteSubshapeNames = theStatics.IVal("write.stepcaf.subshapes.name") == 1;
        WriteColor = theStatics.IVal("write.color") == 1;
        WriteNonmanifold = theStatics.IVal("write.step.nonmanifold") == 1;
        WriteName = theStatics.IVal("write.name") == 1;
        WriteLayer = theStatics.IVal("write.layer") == 1;
        WriteProps = theStatics.IVal("write.props") == 1;
        WriteModelType = (STEPControl_StepModelType)theStatics.IVal("write.model.type");
    }

    /// <summary>Reset parameters to default values.</summary>
    public void Reset()
    {
        DESTEP_Parameters aParameters = new DESTEP_Parameters();
        this = aParameters;
    }

    /// <summary>Returns default parameters for shape fixing.</summary>
    public static DE_ShapeFixParameters GetDefaultShapeFixParameters()
    {
        return new DE_ShapeFixParameters();
    }

    public readonly string GetString(ReadMode_ProductContext theMode)
    {
        switch (theMode)
        {
            case ReadMode_ProductContext.ReadMode_ProductContext_All:
                return "all";
            case ReadMode_ProductContext.ReadMode_ProductContext_Design:
                return "design";
            case ReadMode_ProductContext.ReadMode_ProductContext_Analysis:
                return "analysis";
            default:
                return "";
        }
    }

    // Common
    public ReadMode_BSplineContinuity ReadBSplineContinuity = ReadMode_BSplineContinuity.ReadMode_BSplineContinuity_C1; // Manages the continuity of BSpline curves
    public ReadMode_Precision ReadPrecisionMode = ReadMode_Precision.ReadMode_Precision_File; // Reads the precision mode value
    public double ReadPrecisionVal = 0.0001; // ReadMode_Precision for shape construction (if enabled user mode)
    public ReadMode_MaxPrecision ReadMaxPrecisionMode = ReadMode_MaxPrecision.ReadMode_MaxPrecision_Preferred; // Defines the mode of applying the maximum allowed tolerance
    public double ReadMaxPrecisionVal = 1; // Defines the maximum allowable tolerance
    public bool ReadSameParamMode = false; // Defines the using of BRepLib::SameParameter
    public ReadMode_SurfaceCurve ReadSurfaceCurveMode = ReadMode_SurfaceCurve.ReadMode_SurfaceCurve_Default; // Reference for the computation of curves in case of 2D/3D
    public double EncodeRegAngle = 0.57295779513; // Continuity which these two faces are connected with at that edge
    public AngleUnitMode AngleUnit = AngleUnitMode.AngleUnitMode_File; // Indicates what angle units should be used when a STEP file is read
    // Read
    public bool ReadProductMode = true; // Defines the approach used for selection of top-level STEP entities for translation, and for recognition of assembly structures
    public ReadMode_ProductContext ReadProductContext = ReadMode_ProductContext.ReadMode_ProductContext_All; // When reading AP 209 STEP files, allows selecting either only 'design' or 'analysis', or both types of products for translation
    public ReadMode_ShapeRepr ReadShapeRepr = ReadMode_ShapeRepr.ReadMode_ShapeRepr_All; // Specifies preferred type of representation of the shape of the product
    public RWMode_Tessellated ReadTessellated = RWMode_Tessellated.RWMode_Tessellated_On; // Defines whether tessellated shapes should be translated
    public ReadMode_AssemblyLevel ReadAssemblyLevel = ReadMode_AssemblyLevel.ReadMode_AssemblyLevel_All; // Specifies which data should be read for the products found in the STEP file
    public bool ReadRelationship = true; // Defines whether shapes associated with the main SHAPE_DEFINITION_REPRESENTATION entity of the product via SHAPE_REPRESENTATIONSHIP_RELATION should be translated
    public bool ReadShapeAspect = true; // Defines whether shapes associated with the PRODUCT_DEFINITION_SHAPE entity of the product via SHAPE_ASPECT should be translated
    public bool ReadConstrRelation = false; // Flag regulating translation of "CONSTRUCTIVE_GEOMETRY_REPRESENTATION_RELATIONSHIP" entities
    public bool ReadSubshapeNames = false; // Indicates whether to read sub-shape names from 'Name' attributes of STEP Representation Items
    public Resource_FormatType ReadCodePage = Resource_FormatType.Resource_FormatType_UTF8; // STEP file encoding for names translation
    public bool ReadNonmanifold = false; // Defines non-manifold topology reading
    public bool ReadIdeas = false; // Defines !I-Deas-like STEP processing
    public bool ReadAllShapes = false; // Parameter to read all top level solids and shells
    public bool ReadRootTransformation = true; // Mode to variate apply or not transformation placed in the root shape representation
    public bool ReadColor = true; // ColorMode is used to indicate read Colors or not
    public bool ReadName = true; // NameMode is used to indicate read Name or not
    public bool ReadLayer = true; // LayerMode is used to indicate read Layers or not
    public bool ReadProps = true; // PropsMode is used to indicate read Validation properties or not
    public bool ReadMetadata = true; // Parameter for metadata reading
    public bool ReadProductMetadata = false; // Parameter for product metadata reading

    // Write
    public WriteMode_PrecisionMode WritePrecisionMode = WriteMode_PrecisionMode.WriteMode_PrecisionMode_Average; // Specifies the mode of writing the resolution value into the STEP file
    public double WritePrecisionVal = 0.0001; // Resolution value for an STEP file when WriteMode_PrecisionMode is Greatest
    public WriteMode_Assembly WriteAssembly = WriteMode_Assembly.WriteMode_Assembly_Auto; // Writing assembly mode
    public WriteMode_StepSchema WriteSchema = WriteMode_StepSchema.WriteMode_StepSchema_AP214IS; // Defines the version of schema used for the output STEP file
    public RWMode_Tessellated WriteTessellated = RWMode_Tessellated.RWMode_Tessellated_OnNoBRep; // Defines whether tessellated shapes should be translated
    public string WriteProductName = ""; // Defines the text string that will be used for field 'name' of PRODUCT entities written to the STEP file
    public bool WriteSurfaceCurMode = true; // Indicates whether parametric curves (curves in parametric space of surface) should be written into the STEP file
    public UnitsMethods_LengthUnit WriteUnit = UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Millimeter; // Defines a unit in which the STEP file should be written
    public WriteMode_VertexMode WriteVertexMode = WriteMode_VertexMode.WriteMode_VertexMode_OneCompound; // Indicates which of free vertices writing mode is switch on
    public bool WriteSubshapeNames = false; // Indicates whether to write sub-shape names to 'Name' attributes of STEP Representation Items
    public bool WriteColor = true; // ColorMode is used to indicate write Colors or not
    public bool WriteNonmanifold = false; // Defines non-manifold topology writing
    public bool WriteName = true; // NameMode is used to indicate write Name or not
    public bool WriteLayer = true; // LayerMode is used to indicate write Layers or not
    public bool WriteProps = true; // PropsMode is used to indicate write Validation properties or not
    public bool WriteMetadata = true; // MetadataMode is used to indicate write metadata properties or not.
    public bool WriteMaterial = true; // MaterialMode is used to indicate write Material or not
    public bool WriteVisMaterial = false; // VisMaterialMode is used to indicate write Visual Material or not
    public STEPControl_StepModelType WriteModelType = STEPControl_StepModelType.STEPControl_AsIs; // Gives you the choice of translation mode for an Open CASCADE shape that is being translated to STEP
    public bool CleanDuplicates = false; // Indicates whether to remove duplicate entities from the STEP file
    public bool WriteScalingTrsf = true; // Indicates if scaling should be written as Cartesian Operator or skipped
}
